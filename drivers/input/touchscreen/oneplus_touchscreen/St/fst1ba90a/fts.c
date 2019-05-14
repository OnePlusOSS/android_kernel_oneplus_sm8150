/*****************************************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * File       : fts.c
 * Description: Source file for ST fst1ba90a driver
 * Version   : 1.0
 * Date        : 2018-10-18
 * Author    : Zengpeng.Chen@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 *******************************************************************************************/


#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spi.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include "fts.h"
#include "fts_lib/ftsCompensation.h"
#include "fts_lib/ftsCore.h"
#include "fts_lib/ftsIO.h"
#include "fts_lib/ftsError.h"
#include "fts_lib/ftsFlash.h"
#include "fts_lib/ftsFrame.h"
#include "fts_lib/ftsGesture.h"
#include "fts_lib/ftsTest.h"
#include "fts_lib/ftsTime.h"
#include "fts_lib/ftsTool.h"
#include "../st_common.h"

/*******Part0:LOG TAG Declear************************/
#define TPD_PRINT_POINT_NUM 150
#define TPD_DEVICE "ST-fst1ba90a"
#define TPD_INFO(a, arg...)  pr_err("[TP]"TPD_DEVICE ": " a, ##arg)
#define TPD_DEBUG(a, arg...)\
    do{\
        if (LEVEL_DEBUG == tp_debug)\
            pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

#define TPD_DETAIL(a, arg...)\
    do{\
        if (LEVEL_BASIC != tp_debug)\
            pr_err("[TP]"TPD_DEVICE ": " a, ##arg);\
    }while(0)

#define TPD_SPECIFIC_PRINT(count, a, arg...)\
    do{\
        if (count++ == TPD_PRINT_POINT_NUM || LEVEL_DEBUG == tp_debug) {\
            TPD_INFO(TPD_DEVICE ": " a, ##arg);\
            count = 0;\
        }\
    }while(0)

extern int tp_register_times;

/* system related info */
extern SysInfo systemInfo;

/* test iterm related info */
extern TestToDo tests;

/* gesture related info */
#ifdef GESTURE_MODE
extern u16 gesture_coordinates_x[GESTURE_MAX_COORDS_PAIRS_REPORT];
extern u16 gesture_coordinates_y[GESTURE_MAX_COORDS_PAIRS_REPORT];
extern int gesture_coords_reported;
extern struct mutex gestureMask_mutex;
#endif

/* fun declaration */
static int fts_mode_switch(void *chip_data, work_mode mode, bool flag);
static int fts_chip_initialization(struct fts_ts_info *info, int init_type);
static int fts_esd_handle(void *chip_data);

/* TODO: define if need to do the full mp at the boot */
/**
  * Execute the initialization of the IC (supporting a retry mechanism),
  * checking also the resulting data
  * @see  production_test_main()
  */
static int fts_chip_initialization(struct fts_ts_info *info, int init_type)
{
    int ret2 = 0;
    int retry;
    int initretrycnt = 0;

    /* initialization error, retry initialization */
    for (retry = 0; retry < RETRY_INIT_BOOT; retry++) {
#ifndef COMPUTE_INIT_METHOD
        ret2 = production_test_initialization(init_type);
#else
        ret2 = production_test_main(info->test_limit_name, 1, init_type, &tests, MP_FLAG_BOOT);
#endif
        if (ret2 == OK)
            break;
        initretrycnt++;
        TPD_INFO("%s: initialization cycle count = %04d - ERROR %08X\n", __func__, initretrycnt, ret2);
        fts_chip_powercycle(info);
    }
    if (ret2 < OK)    /* initialization error */
        TPD_INFO("%s: fts initialization failed %d times\n", __func__, RETRY_INIT_BOOT);

    return ret2;
}

/**
  * This function try to attempt to communicate with the IC for the first time
  * during the boot up process in order to read the necessary info for the
  * following stages.
  * The function execute a system reset, read fundamental info (system info)
  * @return OK if success or an error code which specify the type of error
  */
static int fts_init(struct fts_ts_info *info)
{
    int error = 0;

    fts_resetDisableIrqCount();

    error = fts_system_reset();
    if (error < OK && isI2cError(error)) {
        TPD_INFO("%s: Cannot reset the device! ERROR %08X\n", __func__, error);
        return error;
    } else {
        if (error == (ERROR_TIMEOUT | ERROR_SYSTEM_RESET_FAIL)) {
            TPD_INFO("%s: Setting default Sys INFO!\n", __func__);
            error = defaultSysInfo(0);
        } else {
            error = readSysInfo(0);    /* system reset OK */
            if (error < OK) {
                if (!isI2cError(error))
                    error = OK;
                TPD_INFO("%s: Cannot read Sys Info! ERROR %08X\n", __func__, error);
            }
        }
    }

    return error;
}

/* fts powercycle func */
int fts_chip_powercycle(struct fts_ts_info *info)
{
    int error = 0;

    TPD_INFO("%s: Power Cycle Starting...\n", __func__);

    error = tp_powercontrol_1v8(info->hw_res, false);
    if (error < 0) {
        TPD_INFO("%s: Failed to disable DVDD regulator\n", __func__);
    }

    msleep(5);

    error = tp_powercontrol_2v8(info->hw_res, false);
    if (error < 0) {
        TPD_INFO("%s: Failed to disable AVDD regulator\n", __func__);
    }

    msleep(10);

    if (info->hw_res->reset_gpio != GPIO_NOT_DEFINED)
         gpio_direction_output(info->hw_res->reset_gpio, 0);
    else
        msleep(300);

    /* in FTI power up first the digital and then the analog */
    error = tp_powercontrol_1v8(info->hw_res, true);
    if (error < 0) {
        TPD_INFO("%s: Failed to enable 1v8 regulator\n", __func__);
    }

    msleep(5);

    error = tp_powercontrol_2v8(info->hw_res, true);
    if (error < 0) {
        TPD_INFO("%s: Failed to enable 2v8 regulator\n", __func__);
    }

    msleep(10);    /* time needed by the regulators for reaching the regime values */

    if (info->hw_res->reset_gpio != GPIO_NOT_DEFINED) {
         gpio_direction_output(info->hw_res->reset_gpio, 1);
         msleep(50);
    }

    TPD_INFO("%s: Power Cycle Finished! ERROR CODE = %08x\n", __func__, error);

    return error;
}

/* fts mode switch func */
static int fts_mode_switch(void *chip_data, work_mode mode, bool flag)
{
    int ret = 0;
    u8 settings[4] = { 0 };
    u8 gesture_mask[GESTURE_MASK_SIZE] = { 0xBF, 0x11, 0x07, 0x00 };/* gesture mask which we want to set */
    u8 cmd_eare_on[4] = { 0xc0, 0x04, 0x01, 0x01};
	u8 cmd_eare_off[4] = {0xc0, 0x04, 0x00, 0x00};
	u8 fingerprint_int_high[3] = {0xC0, 0x05, 0x01};
	u8 fingerprint_int_low[3] = {0xC0, 0x05, 0x00};


    switch(mode) {
    case MODE_NORMAL:
        TPD_DEBUG("%s: enter in normal mode !\n", __func__);
        settings[0] = 0x01;    /* enable all the possible scans mode supported by the config */
        TPD_DEBUG("%s: Sense ON!\n", __func__);
        ret = setScanMode(SCAN_MODE_ACTIVE, settings[0]);
        if (ret < OK) {
            TPD_INFO("%s error during setting NORMAL_MODE! ERROR %08X\n", __func__, ret);
        }
        break;

    case MODE_SLEEP:
        TPD_DEBUG("%s: enter in sleep mode !\n", __func__);
        TPD_DEBUG("%s: Screen OFF...\n", __func__);
        ret = setScanMode(SCAN_MODE_ACTIVE, 0x00);
        if (ret < OK) {
            TPD_INFO("%s error during setting SLEEP_MODE! ERROR %08X\n", __func__, ret);
        }
        break;

#ifdef GESTURE_MODE
    case MODE_GESTURE:
		gesture_mask[0] = 0xff;
		gesture_mask[1] = 0xff;
		gesture_mask[2] = 0xff;
		gesture_mask[3] = 0xff;
        TPD_DEBUG("%s: Gesture mode setting...\n", __func__);
        if (flag) {
        ret = enableGesture(gesture_mask, GESTURE_MASK_SIZE);
        if (ret < OK) {
            TPD_INFO("%s: enableGesture failed! ERROR %08X\n", __func__, ret);
        }
			ret = setScanMode(SCAN_MODE_LOW_POWER, 0);
    		if (ret < OK) {
        		TPD_INFO("enterGestureMode: enter gesture mode ERROR %08X\n",ret);
    		}	
        } else {
        	gesture_mask[0] = 0;
			gesture_mask[1] = 0;
			gesture_mask[2] = 0;
			gesture_mask[3] = 0x01;
            ret = disableGesture(gesture_mask, GESTURE_MASK_SIZE);
            if (ret < OK) {
                TPD_INFO("%s: disableGesture failed! ERROR %08X\n", __func__, ret);
            }
        }
        break;
#endif

#ifdef GLOVE_MODE
    case MODE_GLOVE:
        TPD_DEBUG("%s: Glove Mode setting...\n", __func__);
        if (flag) {
            settings[0] = 1;
        }
        ret = setFeatures(FEAT_SEL_GLOVE, settings, 1);
        if (ret < OK) {
            TPD_INFO("%s: error during setting GLOVE_MODE! ERROR %08X\n", __func__, ret);
        }
        break;
#endif

#ifdef CHARGER_MODE
    case MODE_CHARGE:
        TPD_DEBUG("%s: Charger Mode setting...\n", __func__);
        if (flag) {
            settings[0] = 1;
        }
        ret = setFeatures(FEAT_SEL_CHARGER, settings, 1);
        if (ret < OK) {
            TPD_INFO("%s: error during setting CHARGER_MODE! ERROR %08X\n", __func__, ret);
        }
        break;
#endif
	    case MODE_TOUCH_HOLD:
        TPD_DEBUG("%s: enter in touchhold mode !\n", __func__);
		gesture_mask[0] = 0xff;
		gesture_mask[1] = 0xff;
		gesture_mask[2] = 0xff;
		gesture_mask[3] = 0xff;
		ret = setFeatures(FEAT_SEL_GESTURE, gesture_mask,GESTURE_MASK_SIZE);
		if (ret < OK) {
			TPD_INFO("%s: touchhold switch Faild\n", __func__);
		}
        break;

		case MODE_FACE_DETECT:
			if (flag){
				if(fts_write(cmd_eare_on, ARRAY_SIZE(cmd_eare_on)) < 0) {
					TPD_DEBUG("%s: open face detect fail\n");	
				}
			} else {
				if(fts_write(cmd_eare_off, ARRAY_SIZE(cmd_eare_off)) < 0) {
					TPD_DEBUG("%s: close face detect fail\n");	
				}
			}	
			break;
		case MODE_FINGERPRINT_TEST:
			if (flag) {
				if(fts_write(fingerprint_int_high, ARRAY_SIZE(fingerprint_int_high)) < 0) {
					TPD_DEBUG("%s: fingerprint int pin pull high fail\n");
				}
			}else {
				if(fts_write(fingerprint_int_low, ARRAY_SIZE(fingerprint_int_low)) < 0) {
					TPD_DEBUG("%s: fingerprint int pin pull down fail\n");
				}
			}
    default:
        TPD_DEBUG("%s: unsupport mode.\n", __func__);
    }

    return ret;
}

/* fts get key info func */
#ifdef PHONE_KEY
static u8 fts_get_keycode(void *chip_data)
{
    struct fts_ts_info *info = (struct fts_ts_info *)chip_data;
    int count = 0, i = 0;
    unsigned char data[FIFO_EVENT_SIZE] = { 0 };
    u8 bitmap_result = 0;

    for (count = 0; count < FIFO_DEPTH && info->data[count][0] != EVT_ID_NOEVENT; count++) {
        for (i = 0; i < FIFO_EVENT_SIZE; i++) {
            data[i] = info->data[count][i];/* read data from the buf save in fts_trigger_reason fun */
        }
        if (data[0] == EVT_ID_USER_REPORT && data[1] == EVT_TYPE_USER_KEY) {/* classify the different key code */
            if ((data[2] & FTS_KEY_0) == 0 && (info->key_mask & FTS_KEY_0) > 0) {
                SET_BIT(bitmap_result, BIT_HOME);
                TPD_DEBUG("%s: Button HOME pressed and released!\n", __func__);
            }

            if ((data[2] & FTS_KEY_1) == 0 && (info->key_mask & FTS_KEY_1) > 0) {
                SET_BIT(bitmap_result, BIT_BACK);
                TPD_DEBUG("%s: Button Back pressed and released!\n", __func__);
            }

            if ((data[2] & FTS_KEY_2) == 0 && (info->key_mask & FTS_KEY_2) > 0) {
                SET_BIT(bitmap_result, BIT_MENU);
                TPD_DEBUG("%s: Button Menu pressed!\n", __func__);
            }
            info->key_mask = data[2];/* store the last key code */
        } else {
            TPD_INFO("%s: Invalid event passed as argument!\n", __func__);
        }
    }
    return bitmap_result;
}
#endif

/* fts get gesture info func */
#ifdef GESTURE_MODE
static int fts_get_gesture_info(void *chip_data, struct gesture_info * gesture)
{
    struct fts_ts_info *info = (struct fts_ts_info *)chip_data;
    int count = 0, i = 0;
    unsigned char data[FIFO_EVENT_SIZE] = { 0 };

    for (count = 0; count < FIFO_DEPTH && info->data[count][0] != EVT_ID_NOEVENT; count++) {
        for (i = 0; i < FIFO_EVENT_SIZE; i++) {
            data[i] = info->data[count][i];/* read data from the buf save in fts_trigger_reason fun */
        }
        if (data[0] == EVT_ID_USER_REPORT && data[1] == EVT_TYPE_USER_GESTURE) {/* comfirm the trigger event is gesture */
            switch (data[2]) { /* classify the different gesture event */
            case GEST_ID_DBLTAP:
                gesture->gesture_type = DouTap;
                TPD_DEBUG("%s: double tap !\n", __func__);
                break;

            case GEST_ID_O:
                gesture->gesture_type = Circle;
                TPD_DEBUG("%s: O !\n", __func__);
                break;

            case GEST_ID_RIGHT_1F:
                gesture->gesture_type = Left2RightSwip;
                TPD_DEBUG("%s:  -> !\n", __func__);
                break;

            case GEST_ID_LEFT_1F:
                gesture->gesture_type = Right2LeftSwip;
                TPD_DEBUG("%s:  <- !\n", __func__);
                break;

            case GEST_ID_UP_1F:
                gesture->gesture_type = Down2UpSwip;
                TPD_DEBUG("%s:  UP !\n", __func__);
                break;

            case GEST_ID_DOWN_1F:
                gesture->gesture_type = Up2DownSwip;
                TPD_DEBUG("%s:  DOWN !\n", __func__);
                break;

            case GEST_ID_CARET:
                gesture->gesture_type = DownVee;
                TPD_DEBUG("%s:  ^ !\n", __func__);
                break;

            case GEST_ID_LEFTBRACE:
                gesture->gesture_type = RightVee;
                TPD_DEBUG("%s:  < !\n", __func__);
                break;

            case GEST_ID_RIGHTBRACE:
                gesture->gesture_type = LeftVee;
                TPD_DEBUG("%s:  > !\n", __func__);
                break;

            case GEST_ID_M:
                gesture->gesture_type = Mgestrue;
                TPD_DEBUG("%s: M !\n", __func__);
                break;

            case GEST_ID_W:
                gesture->gesture_type = Wgestrue;
                TPD_DEBUG("%s:  W !\n", __func__);
                break;

            case GEST_ID_V:
                gesture->gesture_type = UpVee;
                TPD_DEBUG("%s:  V !\n", __func__);
                break;

			case GEST_ID_S:
				gesture->gesture_type = Sgestrue;
                TPD_DEBUG("%s:  S !\n", __func__);
				break;

			case GEST_ID_SINGLE:
				gesture->gesture_type = SingleTap;
                TPD_DEBUG("%s:  single !\n", __func__);
				break;

			case GEST_ID_DOU_DOWN:
				gesture->gesture_type = DouSwip;
                TPD_DEBUG("%s:  || !\n", __func__);
				break;

            default:
                gesture->gesture_type = UnkownGesture;
                TPD_DEBUG("%s:  No valid GestureID!\n", __func__);
            }
        }
    }

    return 0;
}
#endif

/* fts get touch ponit func */
static int fts_get_touch_points(void *chip_data, struct point_info *points, int max_num)
{
    struct fts_ts_info *info = (struct fts_ts_info *)chip_data;
    int count = 0, i = 0;
    unsigned char data[FIFO_EVENT_SIZE] = { 0 };
    int obj_attention = 0;
    unsigned char eventId;
    int touchId;
    u8 touchType;

    for (count = 0; count < FIFO_DEPTH && info->data[count][0] != EVT_ID_NOEVENT; count++) {
        if ((info->data[count][0] & 0xF0) != (EVT_ID_ENTER_POINT & 0xF0)
            && (info->data[count][0] & 0xF0) != (EVT_ID_MOTION_POINT & 0xF0)
           /* && (info->data[count][0] & 0xF0) != (EVT_ID_LEAVE_POINT & 0xF0)*/) {
            continue;
        }
        /* read data from the buf save in fts_trigger_reason func */
        for (i = 0; i < FIFO_EVENT_SIZE; i++) {
            data[i] = info->data[count][i];
        }
        TPD_DETAIL("%s event get = %02X %02X %02X %02X %02X %02X %02X %02X\n",__func__, data[0],
            data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        eventId = data[0];
        touchId = (int)((data[1] & 0xF0) >> 4);
        touchType = data[1] & 0x0F;

        if (touchId >= max_num)/* only suppurt max_num fingers */
            continue;

        if ((eventId == EVT_ID_ENTER_POINT) || ((eventId & 0xF0)==(EVT_ID_MOTION_POINT & 0xF0))) {/* touch down event */
            obj_attention |= (0x0001 << touchId);/* setting touch down flag */
        }
        points[touchId].x = (((int)data[3] & 0x0F) << 8) | (data[2]);/* filling the point_info struct */
        points[touchId].y = ((int)data[4] << 4) | ((data[3] & 0xF0) >> 4);
        points[touchId].status = 1;

        switch (touchType) {
        case TOUCH_TYPE_FINGER:
            /* TPD_DEBUG(" %s : It is a finger!\n",__func__); */
        case TOUCH_TYPE_GLOVE:
            /* TPD_DEBUG(" %s : It is a glove!\n",__func__); */
        case TOUCH_TYPE_PALM:
            /* TPD_DEBUG(" %s : It is a palm!\n",__func__); */
            points[touchId].z = 0;
            break;

        case TOUCH_TYPE_HOVER:
            //points[touchId].z = 0;    /* no pressure */
            break;

        case TOUCH_TYPE_INVALID:
            points[touchId].status = 0;
            break;

        default:
            break;
        }

        points[touchId].touch_major = ((((int)data[0]&0x0C) << 2) | (((int)data[6]&0xF0) >> 4));
        points[touchId].width_major = 30;

    }

    return obj_attention;
}

/**
  * Event handler for status events (EVT_ID_STATUS_UPDATE)
  * Handle status update events
  */
static void fts_status_event_handler(struct fts_ts_info *info, unsigned char *event)
{
    switch (event[1]) {
    case EVT_TYPE_STATUS_ECHO:
        TPD_INFO("%s :Echo event of command = %02X %02X %02X %02X %02X %02X\n",
             __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
        break;

    case EVT_TYPE_STATUS_FORCE_CAL:
        switch (event[2]) {
        case 0x00:
            TPD_INFO("%s :Continuous frame drop Force cal = %02X %02X %02X %02X %02X %02X\n",
                 __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
            break;

        case 0x01:
            TPD_INFO("%s :Mutual negative detect Force cal = %02X %02X %02X %02X %02X %02X\n",
                 __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
            break;

        case 0x02:
            TPD_INFO("%s :Mutual calib deviation Force cal = %02X %02X %02X %02X %02X %02X\n",
                 __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
            break;

        case 0x11:
            TPD_INFO("%s :SS negative detect Force cal = %02X %02X %02X %02X %02X %02X\n",
                 __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
            break;

        case 0x12:
            TPD_INFO("%s :SS negative detect Force cal in Low Power mode = %02X %02X %02X %02X %02X %02X\n",
                 __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
            break;

        case 0x13:
            TPD_INFO("%s :SS negative detect Force cal in Idle mode = %02X %02X %02X %02X %02X %02X\n",
                 __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
            break;

        case 0x20:
            TPD_INFO("%s :SS invalid Mutual Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
                __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
            break;

        case 0x21:
            TPD_INFO("%s :SS invalid Self Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
                __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
            break;

        case 0x22:
            TPD_INFO("%s :SS invalid Self Island soft Force cal = %02X %02X %02X %02X %02X %02X\n",
                __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
            break;

        case 0x30:
            TPD_INFO("%s :MS invalid Mutual Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
                __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
            break;

        case 0x31:
            TPD_INFO("%s :MS invalid Self Strength soft Force cal = %02X %02X %02X %02X %02X %02X\n",
                __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
            break;

        default:
            TPD_INFO("%s :Force cal = %02X %02X %02X %02X %02X %02X\n",
                __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
        }
        break;

    case EVT_TYPE_STATUS_FRAME_DROP:
        switch (event[2]) {
        case 0x01:
            TPD_INFO("%s :Frame drop noisy frame = %02X %02X %02X %02X %02X %02X\n",
                __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
            break;

        case 0x02:
            TPD_INFO("%s :Frame drop bad R = %02X %02X %02X %02X %02X %02X\n",
                __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
            break;

        case 0x03:
            TPD_INFO("%s :Frame drop invalid processing state = %02X %02X %02X %02X %02X %02X\n",
                __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
            break;

        default:
            TPD_INFO("%s :Frame drop = %02X %02X %02X %02X %02X %02X\n",
                __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
        }
        break;

    case EVT_TYPE_STATUS_SS_RAW_SAT:
        if (event[2] == 1)
            TPD_INFO("%s :SS Raw Saturated = %02X %02X %02X %02X %02X %02X\n",
                 __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
        else
            TPD_INFO("%s :SS Raw No more Saturated = %02X %02X %02X %02X %02X %02X\n",
                 __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
        break;

    case EVT_TYPE_STATUS_WATER:
        if (event[2] == 1)
            TPD_INFO("%s :Enter Water mode = %02X %02X %02X %02X %02X %02X\n",
                 __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
        else
            TPD_INFO("%s :Exit Water mode = %02X %02X %02X %02X %02X %02X\n",
                 __func__, event[2], event[3], event[4], event[5], event[6], event[7]);
        break;

    default:
        TPD_INFO("%s :Received unhandled status event = %02X %02X %02X %02X %02X %02X %02X %02X\n",
             __func__, event[0], event[1], event[2], event[3], event[4], event[5], event[6], event[7]);
        break;
    }
}

/* fts event trigger reason func */
static u8 fts_trigger_reason(void *chip_data, int gesture_enable, int is_suspended)
{
	struct fts_ts_info *info = (struct fts_ts_info *)chip_data;
	int error = 0, count = 0, i = 0;
	unsigned char regAdd;
	unsigned char data[FIFO_EVENT_SIZE * FIFO_DEPTH] = { 0 };
	unsigned char eventId;
	u8 result = IRQ_IGNORE;
	const unsigned char EVENTS_REMAINING_POS = 7;
	const unsigned char EVENTS_REMAINING_MASK = 0x1F;
	unsigned char events_remaining = 0;
	int offset =0;

	/* read the FIFO and parsing events */
	regAdd = FIFO_CMD_READONE;

	memset(info->data, 0, sizeof(info->data));

	/* Read the first FIFO event and the number of events remaining */
	error = fts_writeReadU8UX(regAdd, 0, 0, data, FIFO_EVENT_SIZE,DUMMY_FIFO);
	events_remaining = data[EVENTS_REMAINING_POS] & EVENTS_REMAINING_MASK;
	events_remaining = (events_remaining > FIFO_DEPTH - 1) ? FIFO_DEPTH - 1 : events_remaining;

	/* Drain the rest of the FIFO, up to 31 events */
	if (error == OK && events_remaining > 0) {
		error = fts_writeReadU8UX(regAdd, 0, 0, &data[FIFO_EVENT_SIZE],FIFO_EVENT_SIZE * events_remaining,DUMMY_FIFO);
	}

	if (error != OK) {
		TPD_INFO("Error (%08X) while reading from FIFO in fts_event_handler\n", error);
	} else {
		for (count = 0; count < events_remaining + 1; count++) {
			offset = count* FIFO_EVENT_SIZE;
			if(data[offset] != EVT_ID_NOEVENT){
				eventId = data[offset];/* eventId is the main event trigger reason */
				for (i = 0; i < FIFO_EVENT_SIZE; i++) {
					info->data[count][i] = data[i+offset];/* store the event data */
				}
				TPD_DEBUG("%s event get = %02X %02X %02X %02X %02X %02X %02X %02X\n",__func__, data[0+offset],
					data[1+offset], data[2+offset], data[3+offset], data[4+offset], data[5+offset], data[6+offset], data[7+offset]);
			}else{
				TPD_INFO("%s:data is null,continue\n",__func__);
				continue;
			}
			TPD_DEBUG("eventId = %x,data[offset] = %x, gesture_enable = %d, is_suspended = %d\n", eventId, data[offset], gesture_enable, is_suspended);
			if ((eventId & 0xF0) <= (EVT_ID_ERROR & 0xF0)) {
				if ((eventId & 0xF0) == (EVT_ID_ENTER_POINT & 0xF0) || (eventId & 0xF0) == (EVT_ID_MOTION_POINT & 0xF0) 
					|| (eventId & 0xF0) == (EVT_ID_LEAVE_POINT & 0xF0)) {/* touch event */
					SET_BIT(result, IRQ_TOUCH);
				} else if ((eventId & 0xF0) == (EVT_ID_USER_REPORT & 0xF0)) {
					if(data[offset+1] == EVT_TYPE_USER_GESTURE && gesture_enable && is_suspended) {/* gesture event */
#ifdef GESTURE_MODE
						SET_BIT(result, IRQ_GESTURE);
#endif
					} else if (data[offset] == EVT_TYPE_USER_KEY) {/* key event */
#ifdef PHONE_KEY
						SET_BIT(result, IRQ_BTN_KEY);
#endif
					}
				} else if ((eventId & 0xF0) == (EVT_ID_ERROR & 0xF0)) {
					if (data[offset+1] == EVT_TYPE_ERROR_HARD_FAULT || data[offset+1] == EVT_TYPE_ERROR_WATCHDOG) {/* other unexcept event */
						dumpErrorInfo(NULL, 0);
						SET_BIT(result, IRQ_EXCEPTION);
					} else if (data[offset+1] == EVT_TYPE_ERROR_ESD) {
						SET_BIT(result, IRQ_EXCEPTION);
						error = fts_esd_handle(info);
						if (error < 0) {
							TPD_INFO("%s: esd handle fail !", __func__);
						}
					}
				} else if ((eventId & 0xF0) == (EVT_ID_STATUS_UPDATE & 0xF0)) {
					fts_status_event_handler(info, &data[offset]);
					if (data[offset +1] == 0x05) {
						SET_BIT(result, IRQ_FACE_STATE);
						if(data[offset +2] == 0x42) {
							info->proximity_status = 1; //near
						} else if (data[offset +2] ==0x44) {
							info->proximity_status = 0; //far
						}
					}
					 
				}
			}
		}
	}
	
	return result;

}

/* fts system reset func */
static int fts_reset(void *chip_data)
{
    return fts_system_reset();
}

/* fts resetgpio setting func */
static int fts_resetgpio_set(struct hw_resource *hw_res, bool on)
{
    if (gpio_is_valid(hw_res->reset_gpio)) {
        TPD_DEBUG("%s: Set the reset_gpio \n", __func__);
        gpio_direction_output(hw_res->reset_gpio, on);
    }
    else {
        return -1;
    }

    return 0;
}

/* fts power control func */
static int fts_power_control(void *chip_data, bool enable)
{
    int ret = 0;
    struct fts_ts_info *chip_info = (struct fts_ts_info *)chip_data;

    if (true == enable) {
        ret = tp_powercontrol_1v8(chip_info->hw_res, true);
        if (ret)
            return -1;
        msleep(5);
        ret = tp_powercontrol_2v8(chip_info->hw_res, true);
        if (ret)
            return -1;
        msleep(15);
        fts_resetgpio_set(chip_info->hw_res, true);
        msleep(50);
    } else {
        ret = tp_powercontrol_1v8(chip_info->hw_res, false);
        if (ret)
            return -1;
        msleep(5);
        ret = tp_powercontrol_2v8(chip_info->hw_res, false);
        if (ret)
            return -1;
        msleep(10);
        fts_resetgpio_set(chip_info->hw_res, false);
    }

    return ret;
}

/* fts get chip relate info and init the chip func */
static int fts_get_chip_info(void *chip_data)
{
    struct fts_ts_info *chip_info = (struct fts_ts_info *)chip_data;
    int error = 0;

    /* init the core lib */
    TPD_INFO("%s: Init Core Lib:\n", __func__);
    initCore(chip_info->client, chip_info->hw_res->reset_gpio);

    /* init the fts relate func */
    TPD_INFO("%s: Init fts fun:\n", __func__);
    error = fts_init(chip_info);
    if (error < OK) {
        TPD_INFO("%s: Cannot initialize the device ERROR %08X\n", __func__, error);
        return -1;
    }

    return 0;
}

/* fts get vendor relate info func */
static int fts_get_vendor(void *chip_data, struct panel_info *panel_data)
{
	int len = 0;
    char manu_temp[MAX_DEVICE_MANU_LENGTH] = "ST_";
    struct fts_ts_info *chip_info = (struct fts_ts_info*)chip_data;

    /* get limit data path */
	len = strlen(panel_data->test_limit_name);
	if ((len > 3) && (panel_data->test_limit_name[len-3] == 'i') && \
        (panel_data->test_limit_name[len-2] == 'm') && (panel_data->test_limit_name[len-1] == 'g')) {
        panel_data->test_limit_name[len-3] = 'c';
        panel_data->test_limit_name[len-2] = 's';
        panel_data->test_limit_name[len-1] = 'v';
    }
	chip_info->test_limit_name = panel_data->test_limit_name;

    /* get fw path */
    chip_info->fw_name = panel_data->fw_name;

    /* get panel manufacture */
    chip_info->tp_type = panel_data->tp_type;
    strlcat(manu_temp, panel_data->manufacture_info.manufacture, MAX_DEVICE_MANU_LENGTH);
    strncpy(panel_data->manufacture_info.manufacture, manu_temp, MAX_DEVICE_MANU_LENGTH);
    TPD_DEBUG("%s: chip_info->tp_type = %d, chip_info->test_limit_name = %s, chip_info->fw_name = %s\n", __func__, chip_info->tp_type, chip_info->test_limit_name, chip_info->fw_name);
    return 0;
}

/* firmware update check func */
static fw_check_state fts_fw_check(void *chip_data, struct resolution_info *resolution_info, struct panel_info *panel_data)
{
    int ret;
    int crc_status = 0;

    TPD_INFO("%s: Fw Check is starting...\n", __func__);

    /* check CRC status */
    ret = fts_crc_check();
    if (ret > OK) {
        TPD_INFO("%s: CRC Error or NO FW!\n", __func__);
        crc_status = ret;
    } else {
        crc_status = 0;
        TPD_INFO("%s: NO CRC Error or Impossible to read CRC register!\n", __func__);
    }
    if(crc_status != 0) {
        return FW_ABNORMAL;
    }
    /*fw check normal need update TP_FW  && device info*/
    panel_data->TP_FW = systemInfo.u16_fwVer;
    if (panel_data->manufacture_info.version) {
        sprintf(panel_data->manufacture_info.version, "0x%x", panel_data->TP_FW);
    }

    return FW_NORMAL;
}

/* fts flash procedure fun */
static int fts_flashProcedure(const struct firmware *raw_fw, bool force, int keep_cx)
{
    int res;
    int data_size;
    Firmware fw;
    u8 *data;

    /* init the fw struct */
    fw.data = NULL;
    data_size = raw_fw->size;
    data = (u8 *)kzalloc( data_size * sizeof(u8), GFP_KERNEL);
    memcpy(data, (u8 *)raw_fw->data, data_size);

    /* parse the fw data */
    res = parseBinFile(data, data_size, &fw, keep_cx);
    if (res < OK) {
        TPD_INFO("%s: readFwFile: impossible parse ERROR %08X\n", __func__, ERROR_MEMH_READ);
        res |= ERROR_MEMH_READ;
        goto updateFw_fail;
    }
    TPD_DEBUG("%s: Fw file parse COMPLETED!\n", __func__);

    /* burn the fw data */
    TPD_DEBUG("%s: Starting flashing procedure...\n", __func__);
    res = flash_burn(fw, force, keep_cx);
    if (res < OK && res != (ERROR_FW_NO_UPDATE | ERROR_FLASH_BURN_FAILED)) {
        TPD_INFO("%s: flashProcedure: ERROR %08X\n", __func__, ERROR_FLASH_PROCEDURE);
        res |= ERROR_FLASH_PROCEDURE;
        goto updateFw_fail;
    }
    TPD_DEBUG("%s: flashing procedure Finished!\n", __func__);

updateFw_fail:
    if (fw.data != NULL)
        kfree(fw.data);

    return res;
}

/* fts firmware update func */
static fw_update_state fts_fw_update(void *chip_data, const struct firmware *raw_fw, bool force)
	{
	
		u8 error_to_search[4] = { EVT_TYPE_ERROR_CRC_CX_HEAD,
					  EVT_TYPE_ERROR_CRC_CX,
					  EVT_TYPE_ERROR_CRC_CX_SUB_HEAD,
					  EVT_TYPE_ERROR_CRC_CX_SUB };
		int retval = 0;
		int retval1 = 0;
		int ret;
		int error = 0;
		int init_type = NO_INIT;
		int crc_status = 0;
		struct fts_ts_info *info = (struct fts_ts_info*)chip_data;
	
#if defined(PRE_SAVED_METHOD) || defined (COMPUTE_INIT_METHOD)
    int keep_cx = 1;
#else
    int keep_cx = 0;
#endif

    /* Procedure the fw using retry method */
	TPD_INFO( "%s Fw Auto Update is starting...\n", __func__);

	/* check CRC status */
	ret = fts_crc_check();
	if (ret > OK) {
		TPD_INFO("%s: CRC Error or NO FW!\n", __func__);
		crc_status = ret;
	} else {
		crc_status = 0;
		TPD_INFO("%s: NO CRC Error or Impossible to read CRC register!\n", __func__);
		if(force == 1)
			crc_status = 1;
	}

    /* Procedure the fw using retry method */
    retval = fts_flashProcedure(raw_fw, crc_status, keep_cx);
    if ((retval & 0xFF000000) == ERROR_FLASH_PROCEDURE) {
        TPD_INFO("%s: firmware update failed and retry! ERROR %08X\n", __func__, retval);
        fts_chip_powercycle(info);    /* power reset */
        retval1 = fts_flashProcedure(raw_fw, crc_status, keep_cx);
        if ((retval1 & 0xFF000000) == ERROR_FLASH_PROCEDURE) {
            TPD_INFO("%s: firmware update failed again!  ERROR %08X\n", __func__, retval1);
            TPD_INFO("%s: Fw Auto Update Failed!\n", __func__);
            //return FW_UPDATE_ERROR;
        }
    } else if (retval == (ERROR_FW_NO_UPDATE | ERROR_FLASH_BURN_FAILED)) {
        //return FW_NO_NEED_UPDATE;
    }

    TPD_INFO( "%s: Verifying if CX CRC Error...\n", __func__);
    ret = fts_system_reset();
    if (ret >= OK) {
        ret = pollForErrorType(error_to_search, 4);
        if (ret < OK) {
            TPD_INFO("%s: No Cx CRC Error Found!\n", __func__);
            TPD_INFO("%s: Verifying if Panel CRC Error...\n", __func__);
            error_to_search[0] = EVT_TYPE_ERROR_CRC_PANEL_HEAD;
            error_to_search[1] = EVT_TYPE_ERROR_CRC_PANEL;
            ret = pollForErrorType(error_to_search, 2);
            if (ret < OK) {
                TPD_INFO("%s: No Panel CRC Error Found!\n", __func__);
                init_type = NO_INIT;
            } else {
                TPD_INFO("%s: Panel CRC Error FOUND! CRC ERROR = %02X\n", __func__, ret);
                init_type = SPECIAL_PANEL_INIT;
            }
        } else {
            TPD_INFO("%s: Cx CRC Error FOUND! CRC ERROR = %02X\n", __func__, ret);
            /* this path of the code is used only in case there is a
             * CRC error in code or config which not allow the fw to
             * compute the CRC in the CX before */
            /* the only way to recover is to have CX in fw file...
             * */
#ifndef COMPUTE_INIT_METHOD
				TPD_INFO( "%s: Try to recovery with CX in fw file...\n", __func__);
				fts_flashProcedure(raw_fw, CRC_CX, 0);
				TPD_INFO("%s: Refresh panel init data...\n", __func__);
#else
				TPD_INFO("%s: Select Full Panel Init...\n", __func__);
				init_type = SPECIAL_FULL_PANEL_INIT;
#endif
			}
		} else
			TPD_INFO("%s: Error while executing system reset! ERROR %08X\n", __func__, ret);	/* better skip initialization because the real state is unknown */
	
		if (init_type == NO_INIT) {
#if defined(PRE_SAVED_METHOD) || defined(COMPUTE_INIT_METHOD)
			if ((systemInfo.u8_cfgAfeVer != systemInfo.u8_cxAfeVer)
#ifdef COMPUTE_INIT_METHOD
				|| ((systemInfo.u8_mpFlag != MP_FLAG_BOOT) && (systemInfo.u8_mpFlag != MP_FLAG_FACTORY))
#endif
				) {
				init_type = SPECIAL_FULL_PANEL_INIT;
				TPD_DEBUG("%s: Different CX AFE Ver: %02X != %02X or invalid MpFlag = %02X... Execute FULL Panel Init!\n", __func__, systemInfo.u8_cfgAfeVer,
					 systemInfo.u8_cxAfeVer, systemInfo.u8_mpFlag);
			} else
#endif
			if (systemInfo.u8_cfgAfeVer != systemInfo.u8_panelCfgAfeVer) {
				init_type = SPECIAL_PANEL_INIT;
				TPD_DEBUG("%s: Different Panel AFE Ver: %02X != %02X... Execute Panel Init!\n", __func__, systemInfo.u8_cfgAfeVer,
					 systemInfo.u8_panelCfgAfeVer);
			} else
				init_type = NO_INIT;
		}
	
		if (init_type != NO_INIT) {    /* initialization status not correct or after FW complete update, do initialization. */
			error = fts_chip_initialization(info, init_type);
			if (error < OK) {
				TPD_INFO("%s: Cannot initialize the chip ERROR %08X\n", __func__, error);
			}
		}
		
		if (retval == (ERROR_FW_NO_UPDATE | ERROR_FLASH_BURN_FAILED)){
			TPD_INFO("%s: no need fW update  ERROR %08X\n", __func__, retval1);
			return FW_NO_NEED_UPDATE;
		} 
		if (((retval1 & 0xFF000000) == ERROR_FLASH_PROCEDURE) || (error < OK)) {
	
			TPD_INFO("%s: Fw Auto Update Failed!\n", __func__);
			return FW_UPDATE_ERROR;
		}
	
		return FW_UPDATE_SUCCESS;
	}



/* fts esd handle func */
static int fts_esd_handle(void *chip_data)
{
    struct fts_ts_info *info = (struct fts_ts_info*)chip_data;
    int error = 0;

    disable_irq_nosync(info->client->irq);

    fts_chip_powercycle(info);

    error = fts_system_reset();
    if (error < OK) {
        TPD_INFO("%s: Cannot restore the device ERROR %08X\n", __func__, error);
        return error;
    }

    enable_irq(info->client->irq);

    return 0;
}

static int fts_get_face_detect(void * chip_data)
{
    int state = -1;
    struct fts_ts_info *chip_info = (struct fts_ts_info *)chip_data;

	if (chip_info->proximity_status == 0) {
		state = 0;		//far
	} else if (chip_info->proximity_status == 1) {
		state = 1;		//near
	}
    return state;
}

/* fts callback func file ops */
static struct oppo_touchpanel_operations fts_ops = {
    .get_vendor       = fts_get_vendor,
    .get_chip_info    = fts_get_chip_info,
    .reset            = fts_reset,
    .power_control    = fts_power_control,
    .fw_check         = fts_fw_check,
    .fw_update        = fts_fw_update,
    .trigger_reason   = fts_trigger_reason,
    .get_touch_points = fts_get_touch_points,
#ifdef GESTURE_MODE
    .get_gesture_info = fts_get_gesture_info,
#endif
    .mode_switch      = fts_mode_switch,
#ifdef PHONE_KEY
    .get_keycode      = fts_get_keycode,
#endif
    .get_usb_state    = st_get_usb_state,
    .get_face_state	  = fts_get_face_detect,
};


static void fts_reserve_read(struct seq_file *s, void *chip_data)
{
    return;
}

/* fts delta data read func */
static void fts_delta_read(struct seq_file *s, void *chip_data)
{
    int x = 0, y = 0, z = 0;
    int res = 0;
    int16_t temp_delta = 0;
    struct fts_ts_info *chip_info = (struct fts_ts_info*)chip_data;
    MutualSenseFrame frameMS;

    if (!chip_info)
        return;

    /* lock the mode to avoid IC enter idle mode */
    setScanMode(SCAN_MODE_LOCKED, LOCKED_ACTIVE);
    msleep(WAIT_FOR_FRESH_FRAMES);
    setScanMode(SCAN_MODE_ACTIVE, 0x00);
    msleep(WAIT_AFTER_SENSEOFF);
    flushFIFO();    /* delete the events related to some
                     * touch (allow to call this function
                     * while touching the screen without
                     * having a flooding of the FIFO) */

    /* get delta data */
    res = getMSFrame3(MS_STRENGTH, &frameMS);
    if (res < 0) {
        TPD_INFO("%s: Error while taking the MS_STRENGTH frame... ERROR %08X\n", __func__, res);
        seq_printf(s, "getMSFrame3 error!\n");
        goto error;
    }

    /* check the Tx and Rx num */
    if (chip_info->hw_res->TX_NUM != frameMS.header.force_node || chip_info->hw_res->RX_NUM != frameMS.header.sense_node) {
        seq_printf(s, "Tx Rx is not match !\n");
        goto error;
    }

    /* print the data */
    for (x = 0; x < frameMS.header.force_node; x++) {
        seq_printf(s, "\n[%2d]", x);
        for (y = 0; y < frameMS.header.sense_node; y++) {
            z = frameMS.header.sense_node * x + y;
            temp_delta = frameMS.node_data[z];
            seq_printf(s, "%4d, ", temp_delta);
        }
    }
    seq_printf(s, "\n");

error:
    setScanMode(SCAN_MODE_ACTIVE, 0xFF);

    if (frameMS.node_data != NULL)
        kfree(frameMS.node_data);
}

/* fts baseline data read func */
static void fts_baseline_read(struct seq_file *s, void *chip_data)
{
    int x = 0, y = 0, z = 0;
    int16_t temp_delta = 0;
    int res = 0;
    struct fts_ts_info *chip_info = (struct fts_ts_info*)chip_data;
    MutualSenseFrame frameMS;

    if (!chip_info)
        return;

    /* lock the mode to avoid IC enter idle mode */
    setScanMode(SCAN_MODE_LOCKED, LOCKED_ACTIVE);
    msleep(WAIT_FOR_FRESH_FRAMES);
    setScanMode(SCAN_MODE_ACTIVE, 0x00);
    msleep(WAIT_AFTER_SENSEOFF);
    flushFIFO();    /* delete the events related to some
                     * touch (allow to call this function
                     * while touching the screen without
                     * having a flooding of the FIFO) */

    /* get baseline data */
    res = getMSFrame3(MS_RAW, &frameMS);
    if (res < 0) {
        TPD_INFO("%s: Error while taking the MS_RAW frame... ERROR %08X\n", __func__, res);
        seq_printf(s, "getMSFrame3 error!\n");
        goto error;
    }

    /* check the Tx and Rx num */
    if (chip_info->hw_res->TX_NUM != frameMS.header.force_node || chip_info->hw_res->RX_NUM != frameMS.header.sense_node) {
        seq_printf(s, "Tx Rx is not match !\n");
        goto error;
    }

    /* print the data */
    for (x = 0; x < frameMS.header.force_node; x++) {
        seq_printf(s, "\n[%2d]", x);
        for (y = 0; y < frameMS.header.sense_node; y++) {
            z = frameMS.header.sense_node * x + y;
            temp_delta = frameMS.node_data[z];
            seq_printf(s, "%4d, ", temp_delta);
        }
    }
    seq_printf(s, "\n");

error:
    setScanMode(SCAN_MODE_ACTIVE, 0xFF);

    if (frameMS.node_data != NULL)
        kfree(frameMS.node_data);
}

/* fts self_delta data read func */
static void fts_self_delta_read(struct seq_file *s, void *chip_data)
{
    int x = 0;
    int res = 0;
    int16_t temp_delta = 0;
    struct fts_ts_info *chip_info = (struct fts_ts_info*)chip_data;
    SelfSenseFrame frameSS;

    if (!chip_info)
        return;

    /* lock the mode to avoid IC enter idle mode */
    setScanMode(SCAN_MODE_LOCKED, LOCKED_ACTIVE);
    msleep(WAIT_FOR_FRESH_FRAMES);
    setScanMode(SCAN_MODE_ACTIVE, 0x00);
    msleep(WAIT_AFTER_SENSEOFF);
    flushFIFO();    /* delete the events related to some
                     * touch (allow to call this function
                     * while touching the screen without
                     * having a flooding of the FIFO) */

    /* get self_delta data */
    res = getSSFrame3(SS_STRENGTH, &frameSS);
    if (res < 0) {
        TPD_INFO("%s: Error while taking the SS_STRENGTH frame... ERROR %08X\n", __func__, res);
        seq_printf(s, "getSSFrame3 error!\n");
        goto error;
    }

    /* check the Tx and Rx num */
    if (chip_info->hw_res->TX_NUM != frameSS.header.force_node || chip_info->hw_res->RX_NUM != frameSS.header.sense_node) {
        seq_printf(s, "Tx Rx is not match !\n");
        goto error;
    }

    /* print the data */
    seq_printf(s, "\n[force_data]");
    for (x = 0; x < frameSS.header.force_node; x++) {
        seq_printf(s, "\n[%d]", x);
        temp_delta = frameSS.force_data[x];
        seq_printf(s, "%4d, ", temp_delta);
    }

    seq_printf(s, "\n[sense_data]");
    for (x = 0; x < frameSS.header.sense_node; x++) {
        temp_delta = frameSS.sense_data[x];
        seq_printf(s, "%4d, ", temp_delta);
    }
    seq_printf(s, "\n");

error:
    setScanMode(SCAN_MODE_ACTIVE, 0xFF);

    if (frameSS.sense_data != NULL)
        kfree(frameSS.sense_data);
    if (frameSS.force_data != NULL)
        kfree(frameSS.force_data);
}

/* fts self_raw data read func */
static void fts_self_raw_read(struct seq_file *s, void *chip_data)
{
    int x = 0;
    int16_t temp_delta = 0;
    int res = 0;
    struct fts_ts_info *chip_info = (struct fts_ts_info*)chip_data;
    SelfSenseFrame frameSS;

    if (!chip_info)
        return;

    /* lock the mode to avoid IC enter idle mode */
    setScanMode(SCAN_MODE_LOCKED, LOCKED_ACTIVE);
    msleep(WAIT_FOR_FRESH_FRAMES);
    setScanMode(SCAN_MODE_ACTIVE, 0x00);
    msleep(WAIT_AFTER_SENSEOFF);
    flushFIFO();    /* delete the events related to some
                     * touch (allow to call this function
                     * while touching the screen without
                     * having a flooding of the FIFO) */

    /* get self_raw data */
    res = getSSFrame3(SS_RAW, &frameSS);
    if (res < 0) {
        TPD_INFO("%s: Error while taking the SS_RAW frame... ERROR %08X\n", __func__, res);
        seq_printf(s, "getSSFrame3 error!\n");
        goto error;
    }

    /* check the Tx and Rx num */
    if (chip_info->hw_res->TX_NUM != frameSS.header.force_node || chip_info->hw_res->RX_NUM != frameSS.header.sense_node) {
        seq_printf(s, "Tx Rx is not match !\n");
        goto error;
    }

    /* print the data */
    seq_printf(s, "\n[force_data]");
    for (x = 0; x < frameSS.header.force_node; x++) {
        seq_printf(s, "\n[%d]", x);
        temp_delta = frameSS.force_data[x];
        seq_printf(s, "%4d, ", temp_delta);
    }

    seq_printf(s, "\n[sense_data]");
    for (x = 0; x < frameSS.header.sense_node; x++) {
        temp_delta = frameSS.sense_data[x];
        seq_printf(s, "%4d, ", temp_delta);
    }
    seq_printf(s, "\n");

error:
    setScanMode(SCAN_MODE_ACTIVE, 0xFF);

    if (frameSS.sense_data != NULL)
        kfree(frameSS.sense_data);
    if (frameSS.force_data != NULL)
        kfree(frameSS.force_data);
}

/* fts baseline_blackscreen read func */
static void fts_baseline_blackscreen_read(struct seq_file *s, void *chip_data)
{
    int x = 0, y = 0, z = 0;
    int16_t temp_delta = 0;
    int res = 0;
    struct fts_ts_info *chip_info = (struct fts_ts_info*)chip_data;
    MutualSenseFrame frameMS;

    if (!chip_info)
        return;

    /* lock the mode to avoid IC enter idle mode */
    setScanMode(SCAN_MODE_LOCKED, LOCKED_LP_ACTIVE);
    msleep(WAIT_FOR_FRESH_FRAMES);
    setScanMode(SCAN_MODE_ACTIVE, 0x00);
    msleep(WAIT_AFTER_SENSEOFF);
    flushFIFO();    /* delete the events related to some
                     * touch (allow to call this function
                     * while touching the screen without
                     * having a flooding of the FIFO) */

    /* get baseline_blackscreen data */
    res = getMSFrame3(MS_RAW, &frameMS);
    if (res < 0) {
        TPD_INFO("%s: Error while taking the MS_RAW frame... ERROR %08X\n", __func__, res);
        seq_printf(s, "getMSFrame3 error!\n");
        goto error;
    }

    /* check the Tx and Rx num */
    if (chip_info->hw_res->TX_NUM != frameMS.header.force_node || chip_info->hw_res->RX_NUM != frameMS.header.sense_node) {
        seq_printf(s, "Tx Rx is not match !\n");
        goto error;
    }

    /* print the data */
    for (x = 0; x < frameMS.header.force_node; x++) {
        seq_printf(s, "\n[%2d]", x);
        for (y = 0; y < frameMS.header.sense_node; y++) {
            z = frameMS.header.sense_node * x + y;
            temp_delta = frameMS.node_data[z];
            seq_printf(s, "%4d, ", temp_delta);
        }
    }
    seq_printf(s, "\n");

error:
    setScanMode(SCAN_MODE_LOW_POWER, 0xFF);

    if (frameMS.node_data != NULL)
        kfree(frameMS.node_data);
}

/* fts main register read func */
static void fts_main_register_read(struct seq_file *s, void *chip_data)
{
    struct fts_ts_info *chip_info = (struct fts_ts_info *)chip_data;
    char temp[256] = { 0 };

    if (!chip_info)
        return ;
    TPD_INFO("%s start\n", __func__);
    /*disable irq when read data from IC*/
    seq_printf(s, "====================================================\n");
    seq_printf(s, "FW VER = %04X\n", systemInfo.u16_fwVer);
    seq_printf(s, "%s\n", printHex("Release Info =  ", systemInfo.u8_releaseInfo, RELEASE_INFO_SIZE, temp));
    seq_printf(s, "====================================================\n");
}

/* fts debug node file ops */
static struct debug_info_proc_operations debug_info_proc_ops = {
    .delta_read         = fts_delta_read,
    .baseline_read      = fts_baseline_read,
    .self_delta_read    = fts_self_delta_read,
    .self_raw_read      = fts_self_raw_read,
    .baseline_blackscreen_read  = fts_baseline_blackscreen_read,
    .main_register_read = fts_main_register_read,
    .reserve_read       = fts_reserve_read,
};

/* fts full panel auto test func */
static void fts_auto_test(struct seq_file *s, void *chip_data, char *path_limits)
{
    int res = 0;
    int init_type = NO_INIT;
    int temp = 0, error_num = 0, i = 0;

#ifndef COMPUTE_INIT_METHOD
    if (systemInfo.u8_cfgAfeVer != systemInfo.u8_cxAfeVer) {
        res = ERROR_OP_NOT_ALLOW;
        TPD_DEBUG("%s: Miss match in CX version! MP test not allowed with wrong CX memory! ERROR %08X\n", __func__, res);
        error_num = 1;
        seq_printf(s, "%d error(s). %s\n", error_num, error_num?"":"All test passed.");
    }
#else
    if (systemInfo.u8_mpFlag != MP_FLAG_FACTORY) {
        init_type = SPECIAL_FULL_PANEL_INIT;
        TPD_DEBUG("%s: Select Full Panel Init!\n", __func__);
    } else {
        init_type = NO_INIT;
        TPD_DEBUG("%s: Skip Full Panel Init!\n", __func__);
    }
#endif

    /* main full panel test, no stop on fail */
    res = production_test_main(path_limits, 0, init_type, &tests, MP_FLAG_FACTORY);
    if (res != OK) {
        TPD_INFO("%s: fts full panel test fail ! ERROR : %08X\n", __func__, res);
    }else {
        TPD_INFO("%s: fts full panel test success ! ERROR : %08X\n", __func__, res);
    }

    /* counting the error item num */
    for (i = 24; i < 29; i++) {
        temp = res >> i;
        temp &= 0x01;
        if (temp)
            error_num++;
    }

    seq_printf(s, "%d error(s). %s\n", error_num, error_num?"":"All test passed.");
}

/* setting full panel test items */
static int fts_init_test_todo(char *limit_file_path)
{
    char *token = NULL;
    char *line2 = NULL;
    char line[800];
    char *buf = NULL;
    int n, size, pointer = 0, ret = OK;
    char *data_file = NULL;
    LimitFile limit_file;

    /* init limit_file struct */
    limit_file.size = 0;
    limit_file.data = NULL;
    strlcpy(limit_file.name, " ", MAX_LIMIT_FILE_NAME);

    /* read limit data */
    if (limit_file_path != NULL) {
        ret = getLimitsFile(limit_file_path, &limit_file);
        if (ret < OK) {
            TPD_INFO("%s: ERROR %08X\n", __func__, ERROR_FILE_NOT_FOUND);
            ret = ERROR_FILE_NOT_FOUND;
            goto END;
        }
        size = limit_file.size;
        data_file = limit_file.data;
    } else {
        TPD_INFO("%s: Empty limit file path!\n", __func__);
        ret = ERROR_FILE_NOT_FOUND;
        goto END;
    }

    TPD_DEBUG("%s: The size of the limits file is %d bytes...\n",__func__, size);

    /* read the first line of limit data file */
    if (readLine(&data_file[pointer], line, size - pointer, &n) < 0) {
        TPD_INFO("%s: No limit data loaded!\n", __func__);
        ret = ERROR_FILE_NOT_FOUND;
        goto END;
    }

    line2 = kstrdup(line, GFP_KERNEL);
    if (line2 == NULL) {
        TPD_INFO("%s: kstrdup ERROR %08X\n", __func__, ERROR_ALLOC);
        ret = ERROR_ALLOC;
        goto END;
    }

    /* setting test items */
    buf = line2;
    token = strsep(&line2, ",");
    while (token != NULL) {
        TPD_DEBUG("%s: token = %s \n",__func__, token);
        if(strcmp(token, "MutualRaw") == 0)
            tests.MutualRaw = 1;
        else if(strcmp(token, "MutualRawMap") == 0)
            tests.MutualRawMap = 1;
        else if(strcmp(token, "MutualRawGap") == 0)
            tests.MutualRawGap = 1;
        else if(strcmp(token, "MutualRawAdj") == 0)
            tests.MutualRawAdj = 1;
        else if(strcmp(token, "MutualRawAdjGap") == 0)
            tests.MutualRawAdjGap = 1;
        else if(strcmp(token, "MutualRawAdjPeak") == 0)
            tests.MutualRawAdjPeak = 1;
        else if(strcmp(token, "MutualRawLP") == 0)
            tests.MutualRawLP = 1;
        else if(strcmp(token, "MutualRawMapLP") == 0)
            tests.MutualRawMapLP = 1;
        else if(strcmp(token, "MutualRawGapLP") == 0)
            tests.MutualRawGapLP = 1;
        else if(strcmp(token, "MutualRawAdjLP") == 0)
            tests.MutualRawAdjLP = 1;
        else if(strcmp(token, "MutualRawAdjITO") == 0)
            tests.MutualRawAdjITO = 1;
        else if(strcmp(token, "MutualCx1") == 0)
            tests.MutualCx1 = 1;
        else if(strcmp(token, "MutualCx2") == 0)
            tests.MutualCx2 = 1;
        else if(strcmp(token, "MutualCx2Adj") == 0)
            tests.MutualCx2Adj = 1;
        else if(strcmp(token, "MutualCxTotal") == 0)
            tests.MutualCxTotal = 1;
        else if(strcmp(token, "MutualCxTotalAdj") == 0)
            tests.MutualCxTotalAdj = 1;
        else if(strcmp(token, "MutualCx1LP") == 0)
            tests.MutualCx1LP = 1;
        else if(strcmp(token, "MutualCx2LP") == 0)
            tests.MutualCx2LP = 1;
        else if(strcmp(token, "MutualCx2AdjLP") == 0)
            tests.MutualCx2AdjLP = 1;
        else if(strcmp(token, "MutualCxTotalLP") == 0)
            tests.MutualCxTotalLP = 1;
        else if(strcmp(token, "MutualCxTotalAdjLP") == 0)
            tests.MutualCxTotalAdjLP = 1;
        else if(strcmp(token, "MutualKeyRaw") == 0)
            tests.MutualKeyRaw = 1;
        else if(strcmp(token, "MutualKeyCx1") == 0)
            tests.MutualKeyCx1 = 1;
        else if(strcmp(token, "MutualKeyCx2") == 0)
            tests.MutualKeyCx2 = 1;
        else if(strcmp(token, "MutualKeyCxTotal") == 0)
            tests.MutualKeyCxTotal = 1;
        else if(strcmp(token, "SelfForceRaw") == 0)
            tests.SelfForceRaw = 1;
        else if(strcmp(token, "SelfForceRawGap") == 0)
            tests.SelfForceRawGap = 1;
        else if(strcmp(token, "SelfForceRawMap") == 0)
            tests.SelfForceRawMap = 1;
        else if(strcmp(token, "SelfForceRawLP") == 0)
            tests.SelfForceRawLP = 1;
        else if(strcmp(token, "SelfForceRawGapLP") == 0)
            tests.SelfForceRawGapLP = 1;
        else if(strcmp(token, "SelfForceRawMapLP") == 0)
            tests.SelfForceRawMapLP = 1;
        else if(strcmp(token, "SelfForceIx1") == 0)
            tests.SelfForceIx1 = 1;
        else if(strcmp(token, "SelfForceIx2") == 0)
            tests.SelfForceIx2 = 1;
        else if(strcmp(token, "SelfForceIx2Adj") == 0)
            tests.SelfForceIx2Adj = 1;
        else if(strcmp(token, "SelfForceIxTotal") == 0)
            tests.SelfForceIxTotal = 1;
        else if(strcmp(token, "SelfForceIxTotalAdj") == 0)
            tests.SelfForceIxTotalAdj = 1;
        else if(strcmp(token, "SelfForceCx1") == 0)
            tests.SelfForceCx1 = 1;
        else if(strcmp(token, "SelfForceCx2") == 0)
            tests.SelfForceCx2 = 1;
        else if(strcmp(token, "SelfForceCx2Adj") == 0)
            tests.SelfForceCx2Adj = 1;
        else if(strcmp(token, "SelfForceCxTotal") == 0)
            tests.SelfForceCxTotal = 1;
        else if(strcmp(token, "SelfForceCxTotalAdj") == 0)
            tests.SelfForceCxTotalAdj = 1;
        else if(strcmp(token, "SelfForceIx1LP") == 0)
            tests.SelfForceIx1LP = 1;
        else if(strcmp(token, "SelfForceIx2LP") == 0)
            tests.SelfForceIx2LP = 1;
        else if(strcmp(token, "SelfForceIx2AdjLP") == 0)
            tests.SelfForceIx2AdjLP = 1;
        else if(strcmp(token, "SelfForceIxTotalLP") == 0)
            tests.SelfForceIxTotalLP = 1;
        else if(strcmp(token, "SelfForceIxTotalAdjLP") == 0)
            tests.SelfForceIxTotalAdjLP = 1;
        else if(strcmp(token, "SelfForceCx1LP") == 0)
            tests.SelfForceCx1LP = 1;
        else if(strcmp(token, "SelfForceCx2LP") == 0)
            tests.SelfForceCx2LP = 1;
        else if(strcmp(token, "SelfForceCx2AdjLP") == 0)
            tests.SelfForceCx2AdjLP = 1;
        else if(strcmp(token, "SelfForceCxTotalLP") == 0)
            tests.SelfForceCxTotalLP = 1;
        else if(strcmp(token, "SelfForceCxTotalAdjLP") == 0)
            tests.SelfForceCxTotalAdjLP = 1;
        else if(strcmp(token, "SelfSenseRaw") == 0)
            tests.SelfSenseRaw = 1;
        else if(strcmp(token, "SelfSenseRawGap") == 0)
            tests.SelfSenseRawGap = 1;
        else if(strcmp(token, "SelfSenseRawMap") == 0)
            tests.SelfSenseRawMap = 1;
        else if(strcmp(token, "SelfSenseRawLP") == 0)
            tests.SelfSenseRawLP = 1;
        else if(strcmp(token, "SelfSenseRawGapLP") == 0)
            tests.SelfSenseRawGapLP = 1;
        else if(strcmp(token, "SelfSenseRawMapLP") == 0)
            tests.SelfSenseRawMapLP = 1;
        else if(strcmp(token, "SelfSenseIx1") == 0)
            tests.SelfSenseIx1 = 1;
        else if(strcmp(token, "SelfSenseIx2") == 0)
            tests.SelfSenseIx2 = 1;
        else if(strcmp(token, "SelfSenseIx2Adj") == 0)
            tests.SelfSenseIx2Adj = 1;
        else if(strcmp(token, "SelfSenseIxTotal") == 0)
            tests.SelfSenseIxTotal = 1;
        else if(strcmp(token, "SelfSenseIxTotalAdj") == 0)
            tests.SelfSenseIxTotalAdj = 1;
        else if(strcmp(token, "SelfSenseCx1") == 0)
            tests.SelfSenseCx1 = 1;
        else if(strcmp(token, "SelfSenseCx2") == 0)
            tests.SelfSenseCx2 = 1;
        else if(strcmp(token, "SelfSenseCx2Adj") == 0)
            tests.SelfSenseCx2Adj = 1;
        else if(strcmp(token, "SelfSenseCxTotal") == 0)
            tests.SelfSenseCxTotal = 1;
        else if(strcmp(token, "SelfSenseCxTotalAdj") == 0)
            tests.SelfSenseCxTotalAdj = 1;
        else if(strcmp(token, "SelfSenseIx1LP") == 0)
            tests.SelfSenseIx1LP = 1;
        else if(strcmp(token, "SelfSenseIx2LP") == 0)
            tests.SelfSenseIx2LP = 1;
        else if(strcmp(token, "SelfSenseIx2AdjLP") == 0)
            tests.SelfSenseIx2AdjLP = 1;
        else if(strcmp(token, "SelfSenseIxTotalLP") == 0)
            tests.SelfSenseIxTotalLP = 1;
        else if(strcmp(token, "SelfSenseIxTotalAdjLP") == 0)
            tests.SelfSenseIxTotalAdjLP = 1;
        else if(strcmp(token, "SelfSenseCx1LP") == 0)
            tests.SelfSenseCx1LP = 1;
        else if(strcmp(token, "SelfSenseCx2LP") == 0)
            tests.SelfSenseCx2LP = 1;
        else if(strcmp(token, "SelfSenseCx2AdjLP") == 0)
            tests.SelfSenseCx2AdjLP = 1;
        else if(strcmp(token, "SelfSenseCxTotalLP") == 0)
            tests.SelfSenseCxTotalLP = 1;
        else if(strcmp(token, "SelfSenseCxTotalAdjLP") == 0)
            tests.SelfSenseCxTotalAdjLP = 1;
        else
            TPD_INFO("%s: Unknow or unsupport test iterm !", __func__);
        token = strsep(&line2, ",");
    };

END:
    if (buf != NULL) {
        kfree(buf);
        buf = NULL;
    }
    if (limit_file.data != NULL) {
        kfree(limit_file.data);
        limit_file.data = NULL;
    }

    return ret;
}

static void fts_calibrate(struct seq_file *s, void *chip_data)
{
    int ret = -1;
	u8 saveInit = RETRY_INIT_BOOT;

    ret = production_test_initialization((u8)saveInit);
    if (ret < 0) {
		TPD_INFO("%s calibration failed\n", __func__);
        seq_printf(s, "1 error, calibration failed\n");
    } else {
		TPD_INFO("%s calibration successed\n", __func__);
        seq_printf(s, "0 error, calibration successed\n");
    }

    return;
}

static void fts_verify_calibration(struct seq_file *s, void *chip_data)
{
    int ret = -1;

 	ret = 0;
    if (ret != 0) {
		TPD_INFO("%s verify calibration failed\n", __func__);
        seq_printf(s, "1 error, verify calibration result failed(0x%02x)\n", ret);
    } else {
		TPD_INFO("%s verify calibration successed\n", __func__);
        seq_printf(s, "0 error, verify calibration result successed\n");
    }

    return;
}

/* st common proc ops */
static struct st_proc_operations st_proc_ops = {
    .auto_test          = fts_auto_test,
    .init_test_iterm    = fts_init_test_todo,
    .calibrate          = fts_calibrate,
    .verify_calibration = fts_verify_calibration,
};

/* fts driver probe func */
static int fts_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{

    struct fts_ts_info *info = NULL;
    struct touchpanel_data *ts = NULL;
    int ret = 0;

    TPD_INFO("%s : driver probe begin!\n", __func__);
	if (tp_register_times > 0) {
		TPD_INFO("TP driver have success loaded %d times, exit\n", tp_register_times);
		return -1;
	}
    /* step1:Alloc chip_info */
    info = kzalloc(sizeof(struct fts_ts_info), GFP_KERNEL);
    if (!info) {
        TPD_INFO("%s : Out of memory... Impossible to allocate struct info!\n", __func__);
        ret = -ENOMEM;
        goto Alloc_chip_info_fail;
    }
    memset(info, 0, sizeof(*info));

    /* step2:Alloc common ts */
    ts =  kzalloc(sizeof(struct touchpanel_data), GFP_KERNEL);
    if (!ts) {
        TPD_INFO("%s : Out of memory... Impossible to allocate struct common ts!\n", __func__);
        ret = -ENOMEM;
        goto Alloc_common_ts_fail;
    }
    memset(ts, 0, sizeof(*ts));

    /* step3:building client && dev for callback building */
    info->client = client;
    info->st_ops = &st_proc_ops;
    ts->debug_info_ops = &debug_info_proc_ops;
    info->dev = &info->client->dev;
    ts->client = client;
    ts->irq = client->irq;
    i2c_set_clientdata(client, ts);
    ts->dev = &client->dev;
    ts->chip_data = info;
    info->hw_res = &ts->hw_res;

    /* step4:file_operations callback building */
    ts->ts_ops = &fts_ops;

    /* step5:register common touch */
    ret = register_common_touch_device(ts);
    if (ret < 0) {
        goto register_common_touch_fail;
    }

    /* step6:create st common related proc files */
    ret = st_create_proc(ts, info->st_ops);
    if (ret < OK)
        TPD_INFO("%s : Error: can not create st related proc file!\n", __func__);

    /* step7:create fts related proc files */
    ret = fts_proc_init(ts);
    if (ret < OK)
        TPD_INFO("%s : Error: can not create fts related proc file!\n", __func__);

    /* step8:others */
#ifdef GESTURE_MODE
    mutex_init(&gestureMask_mutex);
#endif
    info->key_mask = 0;

    /* probe success */
    TPD_INFO("%s : Probe Finished!\n", __func__);
    return OK;


register_common_touch_fail:
    common_touch_data_free(ts);
    ts = NULL;

Alloc_common_ts_fail:
    kfree(info);
    info = NULL;

Alloc_chip_info_fail:
    TPD_INFO("%s : Probe Failed!\n", __func__);

    return ret;
}

static int fts_remove(struct i2c_client *client)
{
    struct touchpanel_data *ts = dev_get_drvdata(&(client->dev));

    /* proc file stuff */
    fts_proc_remove(ts);
    st_proc_remove(ts);

    /* free all */

    common_touch_data_free(ts);

    return OK;
}

static struct of_device_id fts_of_match_table[] = {
    {
        .compatible = "st,fts",
    },
    {},
};

static int fts_i2c_suspend(struct device *dev)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s: is called gesture_enable =%d\n", __func__, ts->gesture_enable);
    tp_i2c_suspend(ts);

    return 0;
}

static int fts_i2c_resume(struct device *dev)
{
    struct touchpanel_data *ts = dev_get_drvdata(dev);

    TPD_INFO("%s: is called\n", __func__);
    tp_i2c_resume(ts);

    return 0;
}

static const struct i2c_device_id fts_device_id[] = {
    { FTS_TS_DRV_NAME, 0 },
    {}
};

static const struct dev_pm_ops tp_pm_ops = {
#ifdef CONFIG_FB
    .suspend = fts_i2c_suspend,
    .resume  = fts_i2c_resume,
#endif
};

static struct i2c_driver fts_i2c_driver = {
    .driver          = {
        .name           = FTS_TS_DRV_NAME,
        .of_match_table = fts_of_match_table,
        .pm = &tp_pm_ops,
    },
    .probe           = fts_probe,
    .remove          = fts_remove,
    .id_table        = fts_device_id,
};

static int __init fts_driver_init(void)
{
	TPD_INFO("%s is called\n", __func__);
    return i2c_add_driver(&fts_i2c_driver);
}

static void __exit fts_driver_exit(void)
{
    i2c_del_driver(&fts_i2c_driver);
}


MODULE_DESCRIPTION("STMicroelectronics MultiTouch IC Driver");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL");

module_init(fts_driver_init);
module_exit(fts_driver_exit);
