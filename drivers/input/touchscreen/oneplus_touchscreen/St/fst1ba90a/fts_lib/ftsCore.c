/*****************************************************************************************
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * File       : ftsCore.c
 * Description: Source file for ST fst1ba90a driver lib
 * Version   : 1.0
 * Date        : 2018-10-18
 * Author    : Zengpeng.Chen@Bsp.Group.Tp
 * TAG         : BSP.TP.Init
 * ---------------- Revision History: --------------------------
 *   <version>    <date>          < author >                            <desc>
 *******************************************************************************************/


/*!
  * \file ftsCore.c
  * \brief Contains the implementation of the Core functions
  */

#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include "ftsCompensation.h"
#include "ftsCore.h"
#include "ftsError.h"
#include "ftsIO.h"
#include "ftsTest.h"
#include "ftsTime.h"
#include "ftsTool.h"
#include "../../../touchpanel_common.h"

/*******Part0:LOG TAG Declear************************/
#define TPD_PRINT_POINT_NUM 150
#define TPD_DEVICE "fts-lib"
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



SysInfo     systemInfo;    /* /< Global System Info variable, accessible in all the driver */

static int  reset_gpio = GPIO_NOT_DEFINED;    /* /< gpio number of the rest
                                             * pin, the value is
                                             *  GPIO_NOT_DEFINED if the
                                             * reset pin is not connected */
static int  disable_irq_count = 1;    /* /< count the number of call to
                                     * disable_irq, start with 1 because at
                                     * the boot IRQ are already disabled */

/**
  * Initialize core variables of the library.
  * Must be called during the probe before any other lib function
  * @param info pointer to fts_ts_info which contains info about the device and
  * its hw setup
  * @return OK if success or an error code which specify the type of error
  */
int initCore(struct i2c_client *client, int reset_gpio)
{
    int ret = OK;

    TPD_DEBUG("%s: Initialization of the Core...\n", __func__);
    ret |= openChannel(client);
    ret |= resetErrorList();
    ret |= initTestToDo();
    if (gpio_is_valid(reset_gpio)) {
        setResetGpio(reset_gpio);
    }
    if (ret < OK)
        TPD_DEBUG("%s: Initialization Core ERROR %08X!\n", __func__, ret);
    else
        TPD_DEBUG("%s: Initialization Finished!\n", __func__);
    return ret;
}

/**
  * Set the reset_gpio variable with the actual gpio number of the board link to
  * the reset pin
  * @param gpio gpio number link to the reset pin of the IC
  */
void setResetGpio(int gpio)
{
    reset_gpio = gpio;
    TPD_INFO("setResetGpio: reset_gpio = %d\n", reset_gpio);
}

/**
  * Perform a system reset of the IC.
  * If the reset pin is associated to a gpio, the function execute an hw reset
  * (toggling of reset pin) otherwise send an hw command to the IC
  * @return OK if success or an error code which specify the type of error
  */
int fts_system_reset(void)
{
    u8 readData[FIFO_EVENT_SIZE];
    int event_to_search;
    int res = -1;
    int i = 0;
    u8 data[1] = { SYSTEM_RESET_VALUE };

    event_to_search = (int)EVT_ID_CONTROLLER_READY;

    fts_disableInterruptNoSync();

    TPD_DEBUG("System resetting...\n");
    for (i = 0; i < RETRY_SYSTEM_RESET && res < 0; i++) {

        resetErrorList();

        if (reset_gpio == GPIO_NOT_DEFINED)
            res = fts_writeU8UX(FTS_CMD_HW_REG_W, ADDR_SIZE_HW_REG, ADDR_SYSTEM_RESET, data, ARRAY_SIZE(data));
        else {
            gpio_direction_output(reset_gpio, 0);
            msleep(10);
            gpio_direction_output(reset_gpio, 1);
            res = OK;
        }
        if (res < OK)
            TPD_INFO("fts_system_reset: ERROR %08X\n", ERROR_BUS_W);
        else {
            res = pollForEvent(&event_to_search, 1, readData, GENERAL_TIMEOUT);
            if (res < OK)
                TPD_INFO("fts_system_reset: ERROR %08X\n", res);
        }
    }

    fts_enableInterrupt();

    if (res < OK) {
        TPD_INFO("fts_system_reset...failed after 3 attempts: ERROR %08X\n", (res | ERROR_SYSTEM_RESET_FAIL));
        return res | ERROR_SYSTEM_RESET_FAIL;
    } else {
        TPD_DEBUG("System reset DONE!\n");
        return OK;
    }
}

/**
  * Poll the FIFO looking for a specified event within a timeout. Support a
  * retry mechanism.
  * @param event_to_search pointer to an array of int where each element
  * correspond to a byte of the event to find.
  * If the element of the array has value -1, the byte of the event,
  * in the same position of the element is ignored.
  * @param event_bytes size of event_to_search
  * @param readData pointer to an array of byte which will contain the event
  * found
  * @param time_to_wait time to wait before going in timeout
  * @return OK if success or an error code which specify the type of error
  */
int pollForEvent(int *event_to_search, int event_bytes, u8 *readData, int time_to_wait)
{
    int i, find, retry, count_err;
    int time_to_count;
    int err_handling = OK;
    StopWatch clock;

    u8 cmd[1] = { FIFO_CMD_READONE };

    find = 0;
    retry = 0;
    count_err = 0;
    time_to_count = time_to_wait / TIMEOUT_RESOLUTION;

    startStopWatch(&clock);
    while (find != 1 && retry < time_to_count &&
        fts_writeReadU8UX(cmd[0], 0, 0, readData, FIFO_EVENT_SIZE, DUMMY_FIFO) >= OK) {
        /* Log of errors */
        if (readData[0] == EVT_ID_ERROR) {
			TPD_INFO("ERROR EVENT = %02X %02X %02X %02X %02X %02X %02X %02X\n",
			readData[0],readData[1],readData[2],readData[3],readData[4],readData[5],readData[6],readData[7]);
            count_err++;
            err_handling = errorHandler(readData, FIFO_EVENT_SIZE);
            if ((err_handling & 0xF0FF0000) == ERROR_HANDLER_STOP_PROC) {
                TPD_INFO("pollForEvent: forced to be stopped! ERROR %08X\n", err_handling);
                return err_handling;
            }
        } else {
            if (readData[0] != EVT_ID_NOEVENT) {
				TPD_INFO("READ EVENT = %02X %02X %02X %02X %02X %02X %02X %02X\n",
				readData[0],readData[1],readData[2],readData[3],readData[4],readData[5],readData[6],readData[7]);
            }
            if (readData[0] == EVT_ID_CONTROLLER_READY && event_to_search[0] != EVT_ID_CONTROLLER_READY) {
                TPD_INFO("pollForEvent: Unmanned Controller Ready Event! Setting reset flags...\n");
            }
        }

        find = 1;

        for (i = 0; i < event_bytes; i++) {
            if (event_to_search[i] != -1 && (int)readData[i] != event_to_search[i]) {
                find = 0;
                break;
            }
        }

        retry++;
        msleep(TIMEOUT_RESOLUTION);
    }
    stopStopWatch(&clock);
    if ((retry >= time_to_count) && find != 1) {
        TPD_INFO("pollForEvent: ERROR %08X\n", ERROR_TIMEOUT);
        return ERROR_TIMEOUT;
    } else if (find == 1) {
		TPD_INFO("FOUND EVENT = %02X %02X %02X %02X %02X %02X %02X %02X\n",
		readData[0],readData[1],readData[2],readData[3],readData[4],readData[5],readData[6],readData[7]);
        TPD_DEBUG("Event found in %d ms (%d iterations)! Number of errors found = %d\n", elapsedMillisecond(&clock), retry, count_err);
        return count_err;
    } else {
        TPD_INFO("pollForEvent: ERROR %08X\n", ERROR_BUS_R);
        return ERROR_BUS_R;
    }
}

/**
  * Check that the FW sent the echo even after a command was sent
  * @param cmd pointer to an array of byte which contain the command previously
  * sent
  * @param size size of cmd
  * @return OK if success or an error code which specify the type of error
  */
int checkEcho(u8 *cmd, int size)
{
    int ret, i;
    int event_to_search[FIFO_EVENT_SIZE];
    u8 readData[FIFO_EVENT_SIZE];


    if (size < 1) {
        TPD_INFO("checkEcho: Invalid Size = %d not valid! ERROR %08X\n", size, ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    } else {
        if ((size + 3) > FIFO_EVENT_SIZE)
            size = FIFO_EVENT_SIZE - 3;
        /* Echo event 0x43 0x01 xx xx xx xx xx fifo_status
         * therefore command with more than 5 bytes will be trunked */

        event_to_search[0] = EVT_ID_STATUS_UPDATE;
        event_to_search[1] = EVT_TYPE_STATUS_ECHO;
        for (i = 2; i < size + 2; i++)
            event_to_search[i] = cmd[i - 2];
        ret = pollForEvent(event_to_search, size + 2, readData, TIEMOUT_ECHO);
        if (ret < OK) {
            TPD_INFO("checkEcho: Echo Event not found! ERROR %08X\n", ret);
            return ret | ERROR_CHECK_ECHO_FAIL;
        } else if (ret > OK) {
            TPD_INFO("checkEcho: Echo Event found but with some error events before! num_error = %d\n", ret);
            return ERROR_CHECK_ECHO_FAIL;
        }

        TPD_DEBUG("ECHO OK!\n");
        return ret;
    }
}

/**
  * Set a scan mode in the IC
  * @param mode scan mode to set; possible values @link scan_opt Scan Mode
  * Option @endlink
  * @param settings option for the selected scan mode
  * (for example @link active_bitmask Active Mode Bitmask @endlink)
  * @return OK if success or an error code which specify the type of error
  */
int setScanMode(u8 mode, u8 settings)
{
    u8 cmd[3] = { FTS_CMD_SCAN_MODE, mode, settings };
    int ret, size = 3;

    TPD_DEBUG("%s: Setting scan mode: mode = %02X settings = %02X !\n", __func__, mode, settings);
    if (mode == SCAN_MODE_LOW_POWER)
        size = 2;
    ret = fts_write(cmd, size);    /* use write instead of writeFw because
                                 * can be called while the interrupt are
                                 * enabled */
    if (ret < OK) {
        TPD_INFO("%s: write failed...ERROR %08X !\n", __func__, ret);
        return ret | ERROR_SET_SCAN_MODE_FAIL;
    }
    TPD_DEBUG("%s: Setting scan mode OK!\n", __func__);
    return OK;
}

/**
  * Set a feature and its option in the IC
  * @param feat feature to set; possible values @link feat_opt Feature Selection
  * Option @endlink
  * @param settings pointer to an array of byte which store the options for
  * the selected feature (for example the gesture mask to activate
  * @link gesture_opt Gesture IDs @endlink)
  * @param size in bytes of settings
  * @return OK if success or an error code which specify the type of error
  */
int setFeatures(u8 feat, u8 *settings, int size)
{
    u8 cmd[2 + size];
    int i = 0;
    int ret;

    TPD_DEBUG("%s: Setting feature: feat = %02X !\n", __func__,
         feat);
    cmd[0] = FTS_CMD_FEATURE;
    cmd[1] = feat;
    TPD_DEBUG("%s: Settings = ", __func__);
    for (i = 0; i < size; i++) {
        cmd[2 + i] = settings[i];
        TPD_DEBUG( "%02X ", settings[i]);
    }
    TPD_DEBUG( "\n");
    ret = fts_write(cmd, 2 + size);    /* use write instead of writeFw because
                     * can be called while the interrupts
                     * are enabled */
    if (ret < OK) {
        TPD_INFO("%s: write failed...ERROR %08X !\n", __func__, ret);
        return ret | ERROR_SET_FEATURE_FAIL;
    }
    TPD_DEBUG("%s: Setting feature OK!\n", __func__);
    return OK;
}

/**
  * Write a system command to the IC
  * @param sys_cmd System Command to execute; possible values
  * @link sys_opt System Command Option @endlink
  * @param sett settings option for the selected system command
  * (@link sys_special_opt Special Command Option @endlink, @link ito_opt
  * ITO Test Option @endlink, @link load_opt Load Host Data Option @endlink)
  * @param size in bytes of settings
  * @return OK if success or an error code which specify the type of error
  */
int writeSysCmd(u8 sys_cmd, u8 *sett, int size)
{
    u8 cmd[2 + size];
    int ret;

    cmd[0] = FTS_CMD_SYSTEM;
    cmd[1] = sys_cmd;

    TPD_DEBUG("%s: Command = %02X %02X ", __func__, cmd[0], cmd[1]);
    for (ret = 0; ret < size; ret++) {
        cmd[2 + ret] = sett[ret];
        TPD_DEBUG( "%02X ", cmd[2 + ret]);
    }
    TPD_DEBUG( "\n");
    TPD_DEBUG("%s: Writing Sys command...\n", __func__);
    if (sys_cmd != SYS_CMD_LOAD_DATA)
        ret = fts_writeFwCmd(cmd, 2 + size);
    else {
        if (size >= 1)
            ret = requestSyncFrame(sett[0]);
        else {
            TPD_INFO("%s: No setting argument! ERROR %08X\n", __func__, ERROR_OP_NOT_ALLOW);
            return ERROR_OP_NOT_ALLOW;
        }
    }
    if (ret < OK)
        TPD_INFO("%s: ERROR %08X\n", __func__, ret);

    else
        TPD_DEBUG("%s: FINISHED!\n", __func__);

    return ret;
}

/**
  * Initialize the System Info Struct with default values according to the error
  * found during the reading
  * @param i2cError 1 if there was an I2C error while reading the System Info
  * data from memory, other value if another error occurred
  * @return OK if success or an error code which specify the type of error
  */
int defaultSysInfo(int i2cError)
{
    int i;

    TPD_DEBUG("Setting default System Info...\n");

    if (i2cError == 1) {
        systemInfo.u16_fwVer = 0xFFFF;
        systemInfo.u16_cfgProjectId = 0xFFFF;
        for (i = 0; i < RELEASE_INFO_SIZE; i++)
            systemInfo.u8_releaseInfo[i] = 0xFF;
        systemInfo.u16_cxVer = 0xFFFF;
    } else {
        systemInfo.u16_fwVer = 0x0000;
        systemInfo.u16_cfgProjectId = 0x0000;
        for (i = 0; i < RELEASE_INFO_SIZE; i++)
            systemInfo.u8_releaseInfo[i] = 0x00;
        systemInfo.u16_cxVer = 0x0000;
    }

    systemInfo.u8_scrRxLen = 0;
    systemInfo.u8_scrTxLen = 0;

    TPD_DEBUG("default System Info DONE!\n");
    return OK;
}

/**
  * Read the System Info data from memory. System Info is loaded automatically
  * after every system reset.
  * @param request if 1, will be asked to the FW to reload the data, otherwise
  * attempt to read it directly from memory
  * @return OK if success or an error code which specify the type of error
  */
int readSysInfo(int request)
{
    int ret, i, index = 0;
    u8 sett = LOAD_SYS_INFO;
    u8 data[SYS_INFO_SIZE] = { 0 };
    char temp[256] = { 0 };

    if (request == 1) {
        TPD_DEBUG("%s: Requesting System Info...\n", __func__);

        ret = writeSysCmd(SYS_CMD_LOAD_DATA, &sett, 1);
        if (ret < OK) {
            TPD_INFO("%s: error while writing the sys cmd ERROR %08X\n",__func__, ret);
            goto FAIL;
        }
    }

    TPD_DEBUG("%s: Reading System Info...\n", __func__);
    ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, ADDR_FRAMEBUFFER, data, SYS_INFO_SIZE, DUMMY_FRAMEBUFFER);
    if (ret < OK) {
        TPD_INFO("%s: error while reading the system data ERROR %08X\n",__func__, ret);
        goto FAIL;
    }

    TPD_DEBUG("%s: Parsing System Info...\n", __func__);

    if (data[0] != HEADER_SIGNATURE) {
        TPD_INFO("%s: The Header Signature is wrong!  sign: %02X != %02X ERROR %08X\n", __func__, data[0], HEADER_SIGNATURE, ERROR_WRONG_DATA_SIGN);
        ret = ERROR_WRONG_DATA_SIGN;
        goto FAIL;
    }

    if (data[1] != LOAD_SYS_INFO) {
        TPD_INFO("%s: The Data ID is wrong!  ids: %02X != %02X ERROR %08X\n", __func__, data[3], LOAD_SYS_INFO, ERROR_DIFF_DATA_TYPE);
        ret = ERROR_DIFF_DATA_TYPE;
        goto FAIL;
    }

    index += 4;
    u8ToU16(&data[index], &systemInfo.u16_apiVer_rev);
    index += 2;
    systemInfo.u8_apiVer_minor = data[index++];
    systemInfo.u8_apiVer_major = data[index++];
    u8ToU16(&data[index], &systemInfo.u16_chip0Ver);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_chip0Id);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_chip1Ver);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_chip1Id);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_fwVer);
    index += 2;
    TPD_INFO("FW VER = %04X\n", systemInfo.u16_fwVer);
    u8ToU16(&data[index], &systemInfo.u16_svnRev);
    index += 2;
    TPD_INFO("SVN REV = %04X\n", systemInfo.u16_svnRev);
    u8ToU16(&data[index], &systemInfo.u16_cfgVer);
    index += 2;
    TPD_INFO("CONFIG VER = %04X\n", systemInfo.u16_cfgVer);
    u8ToU16(&data[index], &systemInfo.u16_cfgProjectId);
    index += 2;
    TPD_INFO("CONFIG PROJECT ID = %04X\n", systemInfo.u16_cfgProjectId);
    u8ToU16(&data[index], &systemInfo.u16_cxVer);
    index += 2;
    TPD_INFO("CX VER = %04X\n", systemInfo.u16_cxVer);
    u8ToU16(&data[index], &systemInfo.u16_cxProjectId);
    index += 2;
    TPD_INFO("CX PROJECT ID = %04X\n", systemInfo.u16_cxProjectId);
    systemInfo.u8_cfgAfeVer = data[index++];
    systemInfo.u8_cxAfeVer =  data[index++];
    systemInfo.u8_panelCfgAfeVer = data[index++];
    TPD_INFO("AFE VER: CFG = %02X - CX = %02X - PANEL = %02X\n",
         systemInfo.u8_cfgAfeVer, systemInfo.u8_cxAfeVer, systemInfo.u8_panelCfgAfeVer);
    systemInfo.u8_protocol = data[index++];
    TPD_INFO("Protocol = %02X\n", systemInfo.u8_protocol);
    /* index+= 1;
     * reserved area */
    for (i = 0; i < DIE_INFO_SIZE; i++)
        systemInfo.u8_dieInfo[i] = data[index++];
    /* TPD_INFO( "%02X ", systemInfo.u8_dieInfo[i]); */
    /* TPD_INFO( "\n"); */
    TPD_INFO("%s\n", printHex("Die Info =  ", systemInfo.u8_dieInfo, DIE_INFO_SIZE, temp));
    memset(temp, 0, 256);

    /* TPD_INFO("Release Info =  "); */
    for (i = 0; i < RELEASE_INFO_SIZE; i++)
        systemInfo.u8_releaseInfo[i] = data[index++];
    /* TPD_INFO( "%02X ", systemInfo.u8_releaseInfo[i]); */
    /* TPD_INFO( "\n"); */

    TPD_INFO("%s\n", printHex("Release Info =  ", systemInfo.u8_releaseInfo, RELEASE_INFO_SIZE, temp));
    memset(temp, 0, 256);

    u8ToU32(&data[index], &systemInfo.u32_fwCrc);
    index += 4;
    u8ToU32(&data[index], &systemInfo.u32_cfgCrc);
    index += 4;

    index += 4;    /* skip reserved area */

    systemInfo.u8_mpFlag = data[index];
    TPD_INFO("MP FLAG = %02X\n", systemInfo.u8_mpFlag);
	index +=4; 
	u8ToU32(&data[index], &systemInfo.u32_flash_org_info);
	TPD_INFO("Flash Organization Information= 0X%08X \n", systemInfo.u32_flash_org_info);

    index += 4; /* +3 remaining from mp flag address */

    systemInfo.u8_ssDetScanSet = data[index];
    TPD_INFO("SS Detect Scan Select = %d \n", systemInfo.u8_ssDetScanSet);
    index += 4;

    u8ToU16(&data[index], &systemInfo.u16_scrResX);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_scrResY);
    index += 2;
    TPD_INFO("Screen Resolution = %d x %d\n", systemInfo.u16_scrResX, systemInfo.u16_scrResY);
    systemInfo.u8_scrTxLen = data[index++];
    TPD_INFO("TX Len = %d\n", systemInfo.u8_scrTxLen);
    systemInfo.u8_scrRxLen = data[index++];
    TPD_INFO("RX Len = %d\n", systemInfo.u8_scrRxLen);
    systemInfo.u8_keyLen = data[index++];
    TPD_INFO("Key Len = %d\n", systemInfo.u8_keyLen);
    systemInfo.u8_forceLen = data[index++];
    TPD_INFO("Force Len = %d\n", systemInfo.u8_forceLen);

    index += 40;    /* skip reserved area */

    u8ToU16(&data[index], &systemInfo.u16_dbgInfoAddr);
    index += 2;

    index += 6;    /* skip reserved area */

    u8ToU16(&data[index], &systemInfo.u16_msTchRawAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_msTchFilterAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_msTchStrenAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_msTchBaselineAddr);
    index += 2;

    u8ToU16(&data[index], &systemInfo.u16_ssTchTxRawAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssTchTxFilterAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssTchTxStrenAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssTchTxBaselineAddr);
    index += 2;

    u8ToU16(&data[index], &systemInfo.u16_ssTchRxRawAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssTchRxFilterAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssTchRxStrenAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssTchRxBaselineAddr);
    index += 2;

    u8ToU16(&data[index], &systemInfo.u16_keyRawAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_keyFilterAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_keyStrenAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_keyBaselineAddr);
    index += 2;

    u8ToU16(&data[index], &systemInfo.u16_frcRawAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_frcFilterAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_frcStrenAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_frcBaselineAddr);
    index += 2;

    u8ToU16(&data[index], &systemInfo.u16_ssHvrTxRawAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssHvrTxFilterAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssHvrTxStrenAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssHvrTxBaselineAddr);
    index += 2;

    u8ToU16(&data[index], &systemInfo.u16_ssHvrRxRawAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssHvrRxFilterAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssHvrRxStrenAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssHvrRxBaselineAddr);
    index += 2;

    u8ToU16(&data[index], &systemInfo.u16_ssPrxTxRawAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssPrxTxFilterAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssPrxTxStrenAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssPrxTxBaselineAddr);
    index += 2;

    u8ToU16(&data[index], &systemInfo.u16_ssPrxRxRawAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssPrxRxFilterAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssPrxRxStrenAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssPrxRxBaselineAddr);
    index += 2;

    u8ToU16(&data[index], &systemInfo.u16_ssDetRawAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssDetFilterAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssDetStrenAddr);
    index += 2;
    u8ToU16(&data[index], &systemInfo.u16_ssDetBaselineAddr);
    index += 2;

    TPD_INFO("Parsed %d bytes!\n", index);

    if (index != SYS_INFO_SIZE) {
        TPD_INFO("%s: index = %d different from %d ERROR %08X\n",__func__, index, SYS_INFO_SIZE, ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }

    TPD_INFO("System Info Read DONE!\n");
    return OK;

FAIL:
    defaultSysInfo(isI2cError(ret));
    return ret;
}

/**
  * Read data from the Config Memory
  * @param offset Starting address in the Config Memory of data to read
  * @param outBuf pointer of a byte array which contain the bytes to read
  * @param len number of bytes to read
  * @return OK if success or an error code which specify the type of error
  */
int readConfig(u16 offset, u8 *outBuf, int len)
{
    int ret;
    u64 final_address = offset + ADDR_CONFIG_OFFSET;

    TPD_DEBUG("%s: Starting to read config memory at %08llX ...\n", __func__, final_address);
    ret = fts_writeReadU8UX(FTS_CMD_CONFIG_R, BITS_16, final_address,
                outBuf, len, DUMMY_CONFIG);
    if (ret < OK) {
        TPD_INFO("%s: Impossible to read Config Memory... ERROR %08X!\n",__func__, ret);
        return ret;
    }

    TPD_DEBUG("%s: Read config memory FINISHED!\n", __func__);
    return OK;
}

/**
  * Write data into the Config Memory
  * @param offset Starting address in the Config Memory where write the data
  * @param data pointer of a byte array which contain the data to write
  * @param len number of bytes to write
  * @return OK if success or an error code which specify the type of error
  */
int writeConfig(u16 offset, u8 *data, int len)
{
    int ret;
    u64 final_address = offset + ADDR_CONFIG_OFFSET;

    TPD_DEBUG("%s: Starting to write config memory at %08llX ...\n",__func__, final_address);
    ret = fts_writeU8UX(FTS_CMD_CONFIG_W, BITS_16, final_address, data, len);
    if (ret < OK) {
        TPD_INFO("%s: Impossible to write Config Memory... ERROR %08X!\n",__func__, ret);
        return ret;
    }

    TPD_DEBUG("%s: Write config memory FINISHED!\n", __func__);
    return OK;
}

/**
  * Disable the interrupt so the ISR of the driver can not be called
  * @return OK if success or an error code which specify the type of error
  */
int fts_disableInterrupt(void)
{
    if (getClient() != NULL) {
        TPD_DEBUG("Number of disable = %d\n", disable_irq_count);
        if (disable_irq_count == 0) {
            TPD_DEBUG("Executing Disable...\n");
            disable_irq(getClient()->irq);
            disable_irq_count++;
        }
        /* disable_irq is re-entrant so it is required to keep track
         * of the number of calls of this when reenabling */
        TPD_DEBUG("Interrupt Disabled!\n");
        return OK;
    } else {
        TPD_INFO("%s: Impossible get client irq... ERROR %08X\n",__func__, ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }
}


/**
  * Disable the interrupt async so the ISR of the driver can not be called
  * @return OK if success or an error code which specify the type of error
  */
int fts_disableInterruptNoSync(void)
{
    if (getClient() != NULL) {
        TPD_DEBUG("Number of disable = %d\n", disable_irq_count);
        if (disable_irq_count == 0) {
            TPD_DEBUG("Executing Disable...\n");
            disable_irq_nosync(getClient()->irq);
            disable_irq_count++;
        }
        /* disable_irq is re-entrant so it is required to keep track
         * of the number of calls of this when reenabling */
        TPD_DEBUG("Interrupt No Sync Disabled!\n");
        return OK;
    } else {
        TPD_INFO("%s: Impossible get client irq... ERROR %08X\n",__func__, ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }
}


/**
  * Reset the disable_irq count
  * @return OK
  */
int fts_resetDisableIrqCount(void)
{
    disable_irq_count = 0;
    return OK;
}

/**
  * Enable the interrupt so the ISR of the driver can be called
  * @return OK if success or an error code which specify the type of error
  */
int fts_enableInterrupt(void)
{

    if (getClient() != NULL) {
        TPD_DEBUG("Number of re-enable = %d\n", disable_irq_count);
        while (disable_irq_count > 0) {
            /* loop N times according on the pending number of
             * disable_irq to truly re-enable the int */
            TPD_DEBUG("Executing Enable...\n");
            enable_irq(getClient()->irq);
            disable_irq_count--;
        }
        TPD_DEBUG("Interrupt Enabled!\n");
        return OK;
    } else {
        TPD_INFO("%s: Impossible get client irq... ERROR %08X\n", __func__, ERROR_OP_NOT_ALLOW);
        return ERROR_OP_NOT_ALLOW;
    }
}

/**
  *    Check if there is a crc error in the IC which prevent the fw to run.
  *    @return  OK if no CRC error, or a number >OK according the CRC error
  * found
  */
int fts_crc_check(void)
{
    u8 val;
    u8 crc_status;
    int res;
    u8 error_to_search[6] = { EVT_TYPE_ERROR_CRC_CFG_HEAD,
                  EVT_TYPE_ERROR_CRC_CFG,
                  EVT_TYPE_ERROR_CRC_CX,
                  EVT_TYPE_ERROR_CRC_CX_HEAD,
                  EVT_TYPE_ERROR_CRC_CX_SUB,
                  EVT_TYPE_ERROR_CRC_CX_SUB_HEAD };

    res = fts_writeReadU8UX(FTS_CMD_HW_REG_R, ADDR_SIZE_HW_REG, ADDR_CRC, &val, 1, DUMMY_HW_REG);/* read 2 bytes because the first one is a dummy byte! */
    if (res < OK) {
        TPD_INFO("%s: Cannot read crc status ERROR %08X\n", __func__, res);
        return res;
    }

    crc_status = val & CRC_MASK;
    if (crc_status != OK) {    /* CRC error if crc_status!=0 */
        TPD_INFO("%s:CRC ERROR = %02X\n", __func__, crc_status);
        return CRC_CODE;
    }

    TPD_INFO("%s: Verifying if Config CRC Error...\n", __func__);
    res = fts_system_reset();
    if (res >= OK) {
        res = pollForErrorType(error_to_search, 2);
        if (res < OK) {
            TPD_INFO("%s: No Config CRC Error Found!\n", __func__);
            TPD_INFO("%s: Verifying if Cx CRC Error...\n", __func__);
            res = pollForErrorType(&error_to_search[2], 4);
            if (res < OK) {
                TPD_INFO("%s: No Cx CRC Error Found!\n", __func__);
                return OK;
            } else {
                TPD_INFO("%s: Cx CRC Error found! CRC ERROR = %02X\n", __func__, res);
                return CRC_CX;
            }
        } else {
            TPD_INFO("%s: Config CRC Error found! CRC ERROR = %02X\n", __func__, res);
            return CRC_CONFIG;
        }
    } else {
        TPD_INFO("%s: Error while executing system reset! ERROR %08X\n",__func__, res);
        return res;
    }

    return OK;
}

/**
  * Request a host data and use the sync method to understand when the FW load
  * it
  * @param type the type ID of host data to load (@link load_opt Load Host Data
  * Option  @endlink)
  * @return OK if success or an error code which specify the type of error
  */
int requestSyncFrame(u8 type)
{
    u8 request[3] = { FTS_CMD_SYSTEM, SYS_CMD_LOAD_DATA, type };
    u8 readData[DATA_HEADER] = { 0 };
    int ret, retry = 0, retry2 = 0, time_to_count;
    int count, new_count;

    TPD_DEBUG("%s: Starting to get a sync frame...\n", __func__);

    while (retry2 < RETRY_MAX_REQU_DATA) {
        TPD_DEBUG("%s: Reading count...\n", __func__);

        ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, ADDR_FRAMEBUFFER, readData, DATA_HEADER, DUMMY_FRAMEBUFFER);
        if (ret < OK) {
            TPD_DEBUG("%s: Error while reading count! ERROR %08X\n",__func__, ret | ERROR_REQU_DATA);
            ret |= ERROR_REQU_DATA;
            retry2++;
            continue;
        }

        if (readData[0] != HEADER_SIGNATURE)
            TPD_INFO("%s: Invalid Signature while reading count! ERROR %08X\n",__func__, ret | ERROR_REQU_DATA);

        count = (readData[3] << 8) | readData[2];
        new_count = count;
        TPD_DEBUG("%s: Base count = %d\n", __func__, count);

        TPD_DEBUG("%s: Requesting frame %02X  attempt = %d\n",__func__,  type, retry2 + 1);
        ret = fts_write(request, ARRAY_SIZE(request));
        if (ret >= OK) {
            TPD_DEBUG("%s: Polling for new count...\n", __func__);
            time_to_count = TIMEOUT_REQU_DATA / TIMEOUT_RESOLUTION;
            while (count == new_count && retry < time_to_count) {
                ret = fts_writeReadU8UX(FTS_CMD_FRAMEBUFFER_R, BITS_16, ADDR_FRAMEBUFFER, readData, DATA_HEADER, DUMMY_FRAMEBUFFER);
                if ((ret >= OK) && (readData[0] == HEADER_SIGNATURE) && (readData[1] == type))
                    new_count = ((readData[3] << 8) | readData[2]);
                else
                    TPD_DEBUG("%s: invalid Signature or can not read count... ERROR %08X\n",__func__, ret);
                retry++;
                msleep(TIMEOUT_RESOLUTION);
            }

            if (count == new_count) {
                TPD_INFO("%s: New count not received! ERROR %08X\n", __func__, ERROR_TIMEOUT | ERROR_REQU_DATA);
                ret = ERROR_TIMEOUT | ERROR_REQU_DATA;
            } else {
                TPD_DEBUG("%s: New count found! count = %d! Frame ready!\n", __func__, new_count);
                return OK;
            }
        }
        retry2++;
    }
    TPD_INFO("%s: Request Data failed! ERROR %08X\n", __func__,
         ret);
    return ret;
}


/**
  * Save MP flag value into the flash
  * @param mpflag Value to write in the MP Flag field
  * @return OK if success or an error code which specify the type of error
  */
int saveMpFlag(u8 mpflag)
{
    int ret;

    TPD_INFO("%s: Saving MP Flag = %02X\n", __func__, mpflag);
    ret = writeSysCmd(SYS_CMD_MP_FLAG, &mpflag, 1);
    if (ret < OK) {
        TPD_INFO("%s: Error while writing MP flag on ram... ERROR %08X\n", __func__, ret);
        return ret;
    }

    mpflag = SAVE_PANEL_CONF;
    ret = writeSysCmd(SYS_CMD_SAVE_FLASH, &mpflag, 1);
    if (ret < OK) {
        TPD_INFO("%s: Error while saving MP flag on flash... ERROR %08X\n", __func__, ret);
        return ret;
    }

    ret = readSysInfo(1);
    if (ret < OK) {
        TPD_INFO("%s: Error while refreshing SysInfo... ERROR %08X\n", __func__, ret);
        return ret;
    }

    TPD_INFO("%s: Saving MP Flag OK!\n", __func__);
    return OK;
}
