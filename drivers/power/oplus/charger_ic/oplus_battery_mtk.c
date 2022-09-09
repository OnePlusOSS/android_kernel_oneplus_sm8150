/************************************************************************************
** File:  \\192.168.144.3\Linux_Share\12015\ics2\development\mediatek\custom\oplus77_12015\kernel\battery\battery
** OPLUS_FEATURE_CHG_BASIC
** Copyright (C), 2008-2012, OPLUS Mobile Comm Corp., Ltd
** 
** Description: 
**      for dc-dc sn111008 charg
** 
** Version: 1.0
** Date created: 21:03:46,05/04/2012
** Author: Fanhong.Kong@ProDrv.CHG
** 
** --------------------------- Revision History: ------------------------------------------------------------
** 	<author>	<data>			<desc>
**    1.1          2012-8-08       Fanhong.Kong@ProDrv.CHG   modified charger detect 
************************************************************************************************************/

    

#include <linux/init.h>        /* For init/exit macros */
#include <linux/module.h>      /* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/xlog.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif
#include <linux/suspend.h>

#include <asm/scatterlist.h>
#include <linux/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/system.h>
#include <mach/mt_sleep.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpt.h>
#include <mach/mt_boot.h>


#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>
#include <mach/charging.h>
#include <mach/battery_common.h>
#include <mach/battery_meter.h>
#include "cust_pmic.h"
#include <mach/mt_boot.h>
#include "mach/mtk_rtc.h"
#include <linux/gpio.h>
#include <mach/mt_gpio.h>
#include <linux/compat.h>





#include "../oplus_charger.h"
#include "../oplus_gauge.h"
#include "../oplus_vooc.h"


/* ////////////////////////////////////////////////////////////////////////////// */
/* Battery Logging Entry */
/* ////////////////////////////////////////////////////////////////////////////// */
/* static struct proc_dir_entry *proc_entry; */
//char proc_bat_data[32];
int Enable_BATDRV_LOG = BAT_LOG_FULL;



/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Global Variable */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
CHARGING_CONTROL battery_charging_control;
kal_bool g_bat_init_flag = 0;
int g_platform_boot_mode = 0;

#ifdef MTK_ENABLE_AGING_ALGORITHM
extern U32 suspend_time;
#endif
/* ////////////////////////////////////////////////////////////////////////////// */
/* Integrate with NVRAM */
/* ////////////////////////////////////////////////////////////////////////////// */
#define ADC_CALI_DEVNAME "MT_pmic_adc_cali"
static struct class *adc_cali_class = NULL;
static int adc_cali_major = 0;
static dev_t adc_cali_devno;
static struct cdev *adc_cali_cdev;
static int charger_pretype = 0;

int battery_in_data[1] = { 0 };
int battery_out_data[1] = { 0 };
int charging_level_data[1] = { 0 };

kal_bool g_ADC_Cali = KAL_FALSE;

#define TEST_ADC_CALI_PRINT 			_IO('k', 0)
#define SET_ADC_CALI_Slop 				_IOW('k', 1, int)
#define SET_ADC_CALI_Offset 			_IOW('k', 2, int)
#define SET_ADC_CALI_Cal 				_IOW('k', 3, int)
#define ADC_CHANNEL_READ 				_IOW('k', 4, int)
#define BAT_STATUS_READ 				_IOW('k', 5, int)
#define Set_Charger_Current 			_IOW('k', 6, int)
#define Get_FakeOff_Param 				_IOW('k', 7, int)
#define Get_Notify_Param 				_IOW('k', 8, int)
#define Turn_Off_Charging 				_IOW('k', 9, int)
//add for auto test
#define K_AT_CHG_CHGR_IN   				_IOW('k', 10, int)
#define K_AT_CHG_CHGR_OFF  				_IOW('k', 11, int)
#define K_AT_CHG_ON      				_IOW('k', 12, int)
#define K_AT_CHG_OFF      				_IOW('k', 13, int)
#define K_AT_CHG_INFO         			_IOW('k', 14, int)
#define	SET_SPI_CS_LOW					_IOW('k', 16, int)
//add for meta tool-----------------------------------------
#define Get_META_BAT_VOL 				_IOW('k', 17, int) 
#define Get_META_BAT_SOC 				_IOW('k', 18, int) 
//add for meta tool-----------------------------------------

#ifdef CONFIG_COMPAT
#define COMPAT_TEST_ADC_CALI_PRINT 		_IO('k', 0)
#define COMPAT_SET_ADC_CALI_Slop 		_IOW('k', 1, compat_int_t)
#define COMPAT_SET_ADC_CALI_Offset 		_IOW('k', 2, compat_int_t)
#define COMPAT_SET_ADC_CALI_Cal 		_IOW('k', 3, compat_int_t)
#define COMPAT_ADC_CHANNEL_READ 		_IOW('k', 4, compat_int_t)
#define COMPAT_BAT_STATUS_READ 			_IOW('k', 5, compat_int_t)
#define COMPAT_Set_Charger_Current 		_IOW('k', 6, compat_int_t)
#define COMPAT_Get_FakeOff_Param 		_IOW('k', 7, compat_int_t)
#define COMPAT_Get_Notify_Param 		_IOW('k', 8, compat_int_t)
#define COMPAT_Turn_Off_Charging 		_IOW('k', 9, compat_int_t)
#define COMPAT_K_AT_CHG_CHGR_IN   		_IOW('k', 10, compat_int_t)
#define COMPAT_K_AT_CHG_CHGR_OFF  		_IOW('k', 11, compat_int_t)
#define COMPAT_K_AT_CHG_ON      		_IOW('k', 12, compat_int_t)
#define COMPAT_K_AT_CHG_OFF      		_IOW('k', 13, compat_int_t)
#define COMPAT_K_AT_CHG_INFO         	_IOW('k', 14, compat_int_t)
#define	COMPAT_SET_SPI_CS_LOW			_IOW('k', 16, compat_int_t)
#define COMPAT_Get_META_BAT_VOL 		_IOW('k', 17, compat_int_t) 
#define COMPAT_Get_META_BAT_SOC 		_IOW('k', 18, compat_int_t) 
#endif

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Thread related */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
static kal_bool bat_thread_timeout = KAL_FALSE;
static kal_bool chr_wake_up_bat = KAL_FALSE;	/* charger in/out to wake up battery thread */
static DEFINE_MUTEX(bat_mutex);
static DEFINE_MUTEX(charger_type_mutex);
static DECLARE_WAIT_QUEUE_HEAD(bat_thread_wq);
static struct hrtimer battery_kthread_timer;
kal_bool g_battery_soc_ready = KAL_FALSE;
extern U32 _g_bat_sleep_total_time;

PMU_ChargerStruct BMT_status;


///////////////////////////////////////////////////////////////////////////////////////////
//// fop API 
///////////////////////////////////////////////////////////////////////////////////////////
extern bool mt_usb_is_device(void);


/*mt_sleep*/
void charging_suspend_enable(void)
{
    U32 charging_enable = true;

    battery_charging_control(CHARGING_CMD_ENABLE,&charging_enable);
}

void charging_suspend_disable(void)
{
    U32 charging_enable = false;

    battery_charging_control(CHARGING_CMD_ENABLE,&charging_enable);
}

/*mtk_ts_battery*/
int read_tbat_value(void)
{
	return oplus_chg_get_chg_temperature()/10;
}

int get_charger_detect_status(void)
{
	kal_bool chr_status;

	battery_charging_control(CHARGING_CMD_GET_CHARGER_DET_STATUS, &chr_status);
	return chr_status;
}

/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // PMIC PCHR Related APIs */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
kal_bool upmu_is_chr_det(void)
{
#if !defined(CONFIG_POWER_EXT)
	kal_uint32 tmp32;
#endif

    if(battery_charging_control == NULL)
        battery_charging_control = chr_control_interface;

	tmp32 = get_charger_detect_status();

	if (tmp32 == 0) {
		return KAL_FALSE;
	} else {
		#if !defined(CONFIG_MTK_DUAL_INPUT_CHARGER_SUPPORT)
		if (mt_usb_is_device()) {
			battery_xlog_printk(BAT_LOG_FULL,
					    "[upmu_is_chr_det] Charger exist and USB is not host\n");

			return KAL_TRUE;
		} else {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[upmu_is_chr_det] Charger exist but USB is host\n");

			return KAL_FALSE;
		}
		#else
		return KAL_TRUE;
		#endif
	}
}
EXPORT_SYMBOL(upmu_is_chr_det);


void wake_up_bat(void)
{
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] wake_up_bat. \r\n");

	chr_wake_up_bat = KAL_TRUE;
	bat_thread_timeout = KAL_TRUE;
#ifdef MTK_ENABLE_AGING_ALGORITHM
	suspend_time = 0;
#endif
    _g_bat_sleep_total_time = 0;
	wake_up(&bat_thread_wq);
}
EXPORT_SYMBOL(wake_up_bat);



/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Battery Temprature Parameters and functions */
/* ///////////////////////////////////////////////////////////////////////////////////////// */

kal_bool pmic_chrdet_status(void)
{
    if( upmu_is_chr_det() == KAL_TRUE )    
    {
        return KAL_TRUE;
    }
    else
    {
        if(oplus_vooc_get_fastchg_started() == true){
			//battery_xlog_printk(BAT_LOG_CRTI, "[pmic_chrdet_status] No charger,fast chg started\r\n");
			return KAL_TRUE;
		} else {
			charger_pretype = 0;
			//battery_xlog_printk(BAT_LOG_CRTI, "[pmic_chrdet_status] No charger\r\n");
			return KAL_FALSE;
		}
        
    }
}


/* ///////////////////////////////////////////////////////////////////////////////////////// */
/* // Pulse Charging Algorithm */
/* ///////////////////////////////////////////////////////////////////////////////////////// */
/*battery_meter  kd_flashlight*/
kal_bool bat_is_charger_exist(void)
{
	return get_charger_detect_status();
}

/*battery_meter for qmax*/
kal_bool bat_is_charging_full(void)
{
	if ((oplus_chg_get_batt_full() == KAL_TRUE) && (oplus_chg_get_rechging_status() == KAL_FALSE))
		return KAL_TRUE;
	else
		return KAL_FALSE;
}

/*pmic.c for dvfs*/
kal_uint32 bat_get_ui_percentage(void)
{
	/* for plugging out charger in recharge phase, using SOC as UI_SOC */
	if (chr_wake_up_bat == KAL_TRUE)
		return oplus_chg_get_soc();
	else
		return oplus_chg_get_ui_soc();
}

/*mtk_cooler_bcct.c*/
int get_bat_charging_current_level(void)
{
	CHR_CURRENT_ENUM charging_current;

	battery_charging_control(CHARGING_CMD_GET_CURRENT, &charging_current);

	return charging_current;
}

charger_type mt_get_charger_type(void)
{
	return BMT_status.charger_type;	
}

charger_type mt_charger_type_detection(void)
{
	static charger_type CHR_Type_num = CHARGER_UNKNOWN;
	//chg_debug( "mt_charger_type_detection, CHR_Type_num = %d\r\n",CHR_Type_num);

	mutex_lock(&charger_type_mutex);
	if(pmic_chrdet_status() == KAL_FALSE)
	{
		CHR_Type_num = CHARGER_UNKNOWN;
	}
	else
	{
		if(oplus_chg_get_chg_type() == POWER_SUPPLY_TYPE_UNKNOWN)
		{
			battery_charging_control(CHARGING_CMD_GET_CHARGER_TYPE, &CHR_Type_num);
		}
	}
	
	mutex_unlock(&charger_type_mutex);
	BMT_status.charger_type = CHR_Type_num;
	charger_pretype = CHR_Type_num;
	chg_debug( "mt_charger_type_detection, CHR_Type_num = %d\r\n",CHR_Type_num);

	return CHR_Type_num;
}

int mt_power_supply_type_check(void)
{
	int charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	//chg_debug( "mt_power_supply_type_check, charger_type = %d\r\n",charger_type);

	switch(mt_charger_type_detection()){
		case CHARGER_UNKNOWN:		
			break;
		case CHARGING_HOST:
		case STANDARD_HOST:
			charger_type = POWER_SUPPLY_TYPE_USB;
			break;
		case NONSTANDARD_CHARGER:
		case APPLE_0_5A_CHARGER:		
		case STANDARD_CHARGER:		
		case APPLE_2_1A_CHARGER:
		case APPLE_1_0A_CHARGER:
			charger_type = POWER_SUPPLY_TYPE_USB_DCP;
			break;
		default:
			break;	
	}
	chg_debug( "mt_power_supply_type_check, charger_type = %d\r\n",charger_type);

	return charger_type;	

}


int g_temp_CC_value = 0;
 kal_uint32 g_bcct_flag=0;
 kal_uint32 g_usb_state = USB_UNCONFIGURED;
 /*-------battery_meter.c  get_charging_setting_current()------*/
void BATTERY_SetUSBState(int usb_state_value)
{
#if defined(CONFIG_POWER_EXT)
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY_SetUSBState] in FPGA/EVB, no service\r\n");
#else
    if ( (usb_state_value < USB_SUSPEND) || ((usb_state_value > USB_CONFIGURED))){
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BAT_SetUSBState Fail! Restore to default value\r\n");    
        usb_state_value = USB_UNCONFIGURED;
    } else {
        battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] BAT_SetUSBState Success! Set %d\r\n", usb_state_value);    
        g_usb_state = usb_state_value;    
    }
#endif	
}

//mtk_cooler_bcct.c
kal_uint32 set_bat_charging_current_limit(int current_limit)
{
    battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] set_bat_charging_current_limit (%d)\r\n", current_limit);

    if(current_limit != -1)
    {
        g_bcct_flag=1;
        
        if(current_limit < 70)         g_temp_CC_value=CHARGE_CURRENT_0_00_MA;
        else if(current_limit < 200)   g_temp_CC_value=CHARGE_CURRENT_70_00_MA;
        else if(current_limit < 300)   g_temp_CC_value=CHARGE_CURRENT_200_00_MA;
        else if(current_limit < 400)   g_temp_CC_value=CHARGE_CURRENT_300_00_MA;
        else if(current_limit < 450)   g_temp_CC_value=CHARGE_CURRENT_400_00_MA;
        else if(current_limit < 550)   g_temp_CC_value=CHARGE_CURRENT_450_00_MA;
        else if(current_limit < 650)   g_temp_CC_value=CHARGE_CURRENT_550_00_MA;
        else if(current_limit < 700)   g_temp_CC_value=CHARGE_CURRENT_650_00_MA;
        else if(current_limit < 800)   g_temp_CC_value=CHARGE_CURRENT_700_00_MA;
        else if(current_limit < 900)   g_temp_CC_value=CHARGE_CURRENT_800_00_MA;
        else if(current_limit < 1000)  g_temp_CC_value=CHARGE_CURRENT_900_00_MA;
        else if(current_limit < 1100)  g_temp_CC_value=CHARGE_CURRENT_1000_00_MA;
        else if(current_limit < 1200)  g_temp_CC_value=CHARGE_CURRENT_1100_00_MA;
        else if(current_limit < 1300)  g_temp_CC_value=CHARGE_CURRENT_1200_00_MA;
        else if(current_limit < 1400)  g_temp_CC_value=CHARGE_CURRENT_1300_00_MA;
        else if(current_limit < 1500)  g_temp_CC_value=CHARGE_CURRENT_1400_00_MA;
        else if(current_limit < 1600)  g_temp_CC_value=CHARGE_CURRENT_1500_00_MA;
        else if(current_limit == 1600) g_temp_CC_value=CHARGE_CURRENT_1600_00_MA;
        else                           g_temp_CC_value=CHARGE_CURRENT_450_00_MA;
    }
    else
    {
        //change to default current setting
        g_bcct_flag=0;
    }
    
    wake_up_bat();

    return g_bcct_flag;
}

kal_uint32 set_chr_input_current_limit(int current_limit)
{
	battery_xlog_printk(BAT_LOG_CRTI, "set_chr_input_current_limit _NOT_ supported\n");
	return 0;
}

unsigned long BAT_Get_Battery_Voltage(int polling_mode)
{
	unsigned long ret_val = 0;

	ret_val = battery_meter_get_battery_voltage(KAL_FALSE);

	return ret_val;
}

#ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_340000_V;
#else
BATTERY_VOLTAGE_ENUM cv_voltage = BATTERY_VOLT_04_200000_V;
#endif
BATTERY_VOLTAGE_ENUM battery_get_cv_voltage(void)
{
	return cv_voltage;
}

void battery_set_cv_voltage(BATTERY_VOLTAGE_ENUM cv)
{
	cv_voltage = cv;
}

struct wake_lock battery_fg_lock;
static kal_bool fg_wake_up_bat = KAL_FALSE;
#ifdef FG_BAT_INT
void wake_up_bat2(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] wake_up_bat2. \r\n");

	wake_lock(&battery_fg_lock);
	fg_wake_up_bat = KAL_TRUE;
	bat_thread_timeout = KAL_TRUE;
#ifdef MTK_ENABLE_AGING_ALGORITHM
	suspend_time = 0;
#endif
	_g_bat_sleep_total_time = 0;
	wake_up(&bat_thread_wq);
}
EXPORT_SYMBOL(wake_up_bat2);
#endif	

bool oplus_pmic_check_chip_is_null(void)
{
	if(!g_bat_init_flag) 
		return true;
	else
		return false;
}


void do_chrdet_int_task(void)
{
	kal_bool usb_present = KAL_FALSE;
	
	if(oplus_gauge_check_chip_is_null() || oplus_vooc_check_chip_is_null() || oplus_chg_check_chip_is_null())
	{
		battery_xlog_printk(BAT_LOG_CRTI, "[do_chrdet_int_task] vooc || gauge || charger not ready, will do after bettery init.\n");
		return;
	}

	if(oplus_vooc_get_fastchg_started() == true && oplus_vooc_get_adapter_update_status() != ADAPTER_FW_NEED_UPDATE){
		battery_xlog_printk(BAT_LOG_CRTI, "[do_chrdet_int_task] opchg_get_prop_fast_chg_started = true!\n");
		return;
	}
	
	if(g_bat_init_flag == KAL_TRUE)
	{
		if(pmic_chrdet_status() == KAL_FALSE) {
			oplus_vooc_reset_fastchg_after_usbout();
			usb_present = KAL_FALSE;
		} else {
			usb_present = KAL_TRUE;
		}
		oplus_chg_wake_update_work();
		battery_xlog_printk(BAT_LOG_CRTI, "[do_chrdet_int_task] usb_present:%d\n", usb_present);
	} else {
		battery_xlog_printk(BAT_LOG_CRTI, "[do_chrdet_int_task] battery thread not ready, will do after bettery init.\n");	  
	}	
}

static long adc_cali_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int *user_data_addr;
    int *naram_data_addr;
    int i = 0;
    int j = 0;
    int ret = 0;
	int adc_in_data[2] = {1,1};
	int adc_out_data[2] = {1,1};
	int fakeoff_out_data[4] = {0,0,0,0};
	
    static int at_i_sense=0,at_current_offset=0,at_charger_off_vol = 5000 * 5;
    mutex_lock(&bat_mutex);

    switch(cmd)
    {
        case TEST_ADC_CALI_PRINT :
            break;
        
        case SET_ADC_CALI_Slop:
            battery_xlog_printk(BAT_LOG_FULL, "**** unlocked_ioctl : SET_ADC_CALI_Slop Done!\n");  

            break;    
            
        case SET_ADC_CALI_Offset: 
            battery_xlog_printk(BAT_LOG_FULL, "**** unlocked_ioctl : SET_ADC_CALI_Offset Done!\n");            
            break;
           
        case SET_ADC_CALI_Cal : 

            battery_xlog_printk(BAT_LOG_FULL, "**** unlocked_ioctl : SET_ADC_CALI_Cal Done!\n");            
            break;


        case ADC_CHANNEL_READ: 
		 	user_data_addr = (int *)arg;
            ret = copy_from_user(adc_in_data, user_data_addr, 8); /* 2*int = 2*4 */
			
			battery_out_data[0] = 0;
             ret = copy_to_user(user_data_addr, adc_out_data, 8);;
            battery_xlog_printk(BAT_LOG_CRTI, "**** unlocked_ioctl : ADC_CHANNEL_READ\n"); 
		
            break;

        case BAT_STATUS_READ:  
		
            user_data_addr = (int *)arg;
            ret = copy_from_user(battery_in_data, user_data_addr, 4); 

			battery_out_data[0] = 0;
            ret = copy_to_user(user_data_addr, battery_out_data, 4); 
            battery_xlog_printk(BAT_LOG_CRTI, "**** unlocked_ioctl : BAT_STATUS_READ\n");          		
            break;        

        case Set_Charger_Current: /* For Factory Mode*/

            user_data_addr = (int *)arg;
            ret = copy_from_user(charging_level_data, user_data_addr, 4);   
   
            battery_xlog_printk(BAT_LOG_CRTI, "**** unlocked_ioctl : set_Charger_Current\n");
            break;
	//add for meta tool-------------------------------
	case Get_META_BAT_VOL:

		user_data_addr = (int *)arg;
    		ret = copy_from_user(adc_in_data, user_data_addr, 8);
			
		adc_out_data[0] = oplus_chg_get_batt_volt();
		ret = copy_to_user(user_data_addr, adc_out_data, 8);
		
    	xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "**** unlocked_ioctl : BAT_VOL:%d\n", adc_out_data[0]);   
		break;
	case Get_META_BAT_SOC:

		user_data_addr = (int *)arg;
    		ret = copy_from_user(adc_in_data, user_data_addr, 8);
		adc_out_data[0] = oplus_chg_get_ui_soc();
		ret = copy_to_user(user_data_addr, adc_out_data, 8); 
		
    	xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "**** unlocked_ioctl : SOC:%d\n", adc_out_data[0]);  		
		break;
		//add bing meta tool-------------------------------
	case Get_FakeOff_Param: /* For Factory Mode*/

		user_data_addr = (int *)arg;
		fakeoff_out_data[0] = oplus_chg_get_ui_soc();
		fakeoff_out_data[1] = oplus_chg_get_notify_flag();
		if(pmic_chrdet_status() == KAL_TRUE)
		{
			fakeoff_out_data[2] = 1;
		}
		else
		{
			fakeoff_out_data[2] = 0;
		}
		fakeoff_out_data[3] = oplus_chg_show_vooc_logo_ornot();
		ret = copy_to_user(user_data_addr, fakeoff_out_data, 16);
		chg_err("ioctl : Get_FakeOff_Param:ui_soc:%d, g_NotifyFlag:%d,chr_det:%d,fast_chg = %d\n",fakeoff_out_data[0],fakeoff_out_data[1],fakeoff_out_data[2],fakeoff_out_data[3]);

		break; 
			
	case Get_Notify_Param: /* For Fakeoff Mode*/
		chg_err("ioctl : Get_Notify_Param\n"); 
		break; 
		
	case Turn_Off_Charging: /* For Turnoffcharging Mode*/
		chg_err("ioctl : Turn_Off_Charging\n"); 
		break;

	case K_AT_CHG_CHGR_IN:
		chg_err("ioctl : K_AT_CHG_CHGR_IN\n"); 
		break;
		
	case K_AT_CHG_CHGR_OFF:
		chg_err("ioctl : K_AT_CHG_CHGR_OFF\n");
		break;
		
	case K_AT_CHG_ON:
		chg_err("ioctl : K_AT_CHG_ON\n");
		break;
		
	case K_AT_CHG_OFF:
		chg_err("ioctl : K_AT_CHG_OFF\n");
		break;
		
	case K_AT_CHG_INFO:
		chg_err("ioctl : K_AT_CHG_INFO\n");
		break;
		
	case SET_SPI_CS_LOW:
		chg_err("ioctl : SET_SPI_CS_LOW\n");
		break;
	default:

		break;
    }

    mutex_unlock(&bat_mutex);
    
    return 0;
}

#ifdef CONFIG_COMPAT
static long adc_cali_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long ret = 0;
	
	void __user *arg64 = compat_ptr(arg);

	chg_debug( " cmd = 0x%04x\n", cmd);

	if(!file->f_op || !file->f_op->unlocked_ioctl)
	{
		chg_err(" file->f_op OR file->f_op->unlocked_ioctl is null!\n");
		return -ENOTTY;
	}

    switch(cmd)
    {
        case COMPAT_TEST_ADC_CALI_PRINT :
            ret = file->f_op->unlocked_ioctl(file, COMPAT_TEST_ADC_CALI_PRINT, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_TEST_ADC_CALI_PRINT is failed!\n");
			}
			break;
        
        case COMPAT_SET_ADC_CALI_Slop:            
            ret = file->f_op->unlocked_ioctl(file, COMPAT_SET_ADC_CALI_Slop, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_SET_ADC_CALI_Slop is failed!\n");
			}
			break;   
            
        case COMPAT_SET_ADC_CALI_Offset:            
            ret = file->f_op->unlocked_ioctl(file, COMPAT_SET_ADC_CALI_Offset, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_SET_ADC_CALI_Offset is failed!\n");
			}
			break;
            
        case COMPAT_SET_ADC_CALI_Cal :            
           ret = file->f_op->unlocked_ioctl(file, COMPAT_SET_ADC_CALI_Cal, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_SET_ADC_CALI_Cal is failed!\n");
			}
			break;   

        case COMPAT_ADC_CHANNEL_READ:            
            ret = file->f_op->unlocked_ioctl(file, COMPAT_ADC_CHANNEL_READ, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_ADC_CHANNEL_READ is failed!\n");
			}
			break;
        case COMPAT_BAT_STATUS_READ:            
            ret = file->f_op->unlocked_ioctl(file, COMPAT_BAT_STATUS_READ, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_BAT_STATUS_READ is failed!\n");
			}
			break;   

        case COMPAT_Set_Charger_Current: /* For Factory Mode*/
            ret = file->f_op->unlocked_ioctl(file, COMPAT_Set_Charger_Current, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_Set_Charger_Current is failed!\n");
			}
			break;
	//add for meta tool-------------------------------
		case COMPAT_Get_META_BAT_VOL:
			ret = file->f_op->unlocked_ioctl(file, COMPAT_Get_META_BAT_VOL, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_Get_META_BAT_VOL is failed!\n");
			}
			break;
		case COMPAT_Get_META_BAT_SOC:
			ret = file->f_op->unlocked_ioctl(file, COMPAT_Get_META_BAT_SOC, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_Get_META_BAT_SOC is failed!\n");
			}
			break;
			//add bing meta tool-------------------------------
		case COMPAT_Get_FakeOff_Param: /* For Factory Mode*/
			ret = file->f_op->unlocked_ioctl(file, COMPAT_Get_FakeOff_Param, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_Get_FakeOff_Param is failed!\n");
			}
			break;			
		case COMPAT_Get_Notify_Param: /* For Fakeoff Mode*/
			ret = file->f_op->unlocked_ioctl(file, COMPAT_Get_Notify_Param, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_Get_Notify_Param is failed!\n");
			}
			break;
		case COMPAT_Turn_Off_Charging: /* For Turnoffcharging Mode*/
			ret = file->f_op->unlocked_ioctl(file, COMPAT_Turn_Off_Charging, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_Turn_Off_Charging is failed!\n");
			}
			break;
		case COMPAT_K_AT_CHG_CHGR_IN:
			ret = file->f_op->unlocked_ioctl(file, COMPAT_K_AT_CHG_CHGR_IN, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_K_AT_CHG_CHGR_IN is failed!\n");
			}
			break;
		
		case COMPAT_K_AT_CHG_CHGR_OFF:
			ret = file->f_op->unlocked_ioctl(file, COMPAT_K_AT_CHG_CHGR_OFF, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_K_AT_CHG_CHGR_OFF is failed!\n");
			}
			break;
		case COMPAT_K_AT_CHG_ON:
			ret = file->f_op->unlocked_ioctl(file, COMPAT_K_AT_CHG_ON, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_K_AT_CHG_ON is failed!\n");
			}
			break;
		case COMPAT_K_AT_CHG_OFF:
			ret = file->f_op->unlocked_ioctl(file, COMPAT_K_AT_CHG_OFF, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_K_AT_CHG_OFF is failed!\n");
			}
			break;
		case COMPAT_K_AT_CHG_INFO:
			ret = file->f_op->unlocked_ioctl(file, COMPAT_K_AT_CHG_INFO, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_K_AT_CHG_INFO is failed!\n");
			}
			break;
		case COMPAT_SET_SPI_CS_LOW:
			ret = file->f_op->unlocked_ioctl(file, COMPAT_SET_SPI_CS_LOW, (unsigned long)arg64);
			if(ret < 0)
			{
				chg_err("COMPAT_SET_SPI_CS_LOW is failed!\n");
			}
			break;
		default:
			g_ADC_Cali = KAL_FALSE;
			break;
    }
    
    return 0;
}
#endif   


static int adc_cali_open(struct inode *inode, struct file *file)
{ 
   return 0;
}

static int adc_cali_release(struct inode *inode, struct file *file)
{
    return 0;
}


static struct file_operations adc_cali_fops = {
    .owner        = THIS_MODULE,
    .unlocked_ioctl    = adc_cali_unlocked_ioctl,
    .open        = adc_cali_open,
    .release    = adc_cali_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl = adc_cali_compat_ioctl,
#endif    
};

void check_battery_exist(void)
{
#if defined(CONFIG_DIS_CHECK_BATTERY)
	battery_xlog_printk(BAT_LOG_CRTI, "[BATTERY] Disable check battery exist.\n");
#else
	kal_uint32 baton_count = 0;
	kal_uint32 charging_enable = KAL_FALSE;
	kal_uint32 battery_status;
	kal_uint32 i;

	for (i = 0; i < 3; i++) {
		battery_charging_control(CHARGING_CMD_GET_BATTERY_STATUS, &battery_status);
		baton_count += battery_status;

	}

	if (baton_count >= 3) {
		if ((g_platform_boot_mode == META_BOOT) || (g_platform_boot_mode == ADVMETA_BOOT)
		    || (g_platform_boot_mode == ATE_FACTORY_BOOT)) {
			battery_xlog_printk(BAT_LOG_FULL,
					    "[BATTERY] boot mode = %d, bypass battery check\n",
					    g_platform_boot_mode);
		} else {
			battery_xlog_printk(BAT_LOG_CRTI,
					    "[BATTERY] Battery is not exist, power off FAN5405 and system (%d)\n",
					    baton_count);

			battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);
			battery_charging_control(CHARGING_CMD_SET_POWER_OFF, NULL);
		}
	}
#endif
}





static void get_charging_control(void)
{
	battery_charging_control = chr_control_interface;
}

/*
int get_platform_boot_mode(void)
{
	return g_platform_boot_mode;
}
int get_platform_boot_reason(void)
{
	return g_boot_reason;
}
*/
//--------------------------------------------------------
static int battery_probe(struct platform_device *dev)    
{
	//struct oplus_chg_chip	*chip;
	struct class_device *class_dev = NULL;	
	int ret = 0;
	int ret_device_file=0;
	chg_debug( "battery_probe, begin\n");

	 /* Integrate with NVRAM */
    ret = alloc_chrdev_region(&adc_cali_devno, 0, 1, ADC_CALI_DEVNAME);
    if (ret) 
       xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "Error: Can't Get Major number for adc_cali \n");
    adc_cali_cdev = cdev_alloc();
    adc_cali_cdev->owner = THIS_MODULE;
    adc_cali_cdev->ops = &adc_cali_fops;
    ret = cdev_add(adc_cali_cdev, adc_cali_devno, 1);
    if(ret)
       xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "adc_cali Error: cdev_add\n");
    adc_cali_major = MAJOR(adc_cali_devno);
    adc_cali_class = class_create(THIS_MODULE, ADC_CALI_DEVNAME);
    class_dev = (struct class_device *)device_create(adc_cali_class, 
                                                   NULL, 
                                                   adc_cali_devno, 
                                                   NULL, 
                                                   ADC_CALI_DEVNAME);
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] adc_cali prepare : done !!\n ");
	get_charging_control();
	battery_charging_control(CHARGING_CMD_GET_PLATFORM_BOOT_MODE, &g_platform_boot_mode);
	battery_xlog_printk(BAT_LOG_CRTI, "[BAT_probe] g_platform_boot_mode = %d\n ",
			    g_platform_boot_mode);
	
  
	/*LOG System Set */
	//init_proc_log();
	BMT_status.charger_type = CHARGER_UNKNOWN;
	g_bat_init_flag = KAL_TRUE;
	
    return 0;
	
}

static void battery_timer_pause(void)
{

}

static void battery_timer_resume(void)
{
	


/*
		BMT_status.SOC = opchg_get_prop_batt_capacity();
		if ( BMT_status.SOC < bat_volt_check_point ) 
		{		
			if(g_soc_sync_time>=g_soc_sync_down_times)
			{
				g_soc_sync_time=0;
				bat_volt_check_point--;
			}
			else
			{
				g_soc_sync_time+=1;
			}

		chg_debug( "battery resume NOT by pcm timer,soc = %d,bat_volt_check_point = %d!!\n",BMT_status.SOC,bat_volt_check_point);
		}
		*/

}

static int battery_remove(struct platform_device *dev)    
{
    battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver remove!! ********\n" );

    return 0;
}

static void battery_shutdown(struct platform_device *dev)    
{
    battery_xlog_printk(BAT_LOG_CRTI, "******** battery driver shutdown!! ********\n" );

}




///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API 
///////////////////////////////////////////////////////////////////////////////////////////


static int battery_pm_suspend(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_resume(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_freeze(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_restore(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

static int battery_pm_restore_noirq(struct device *device)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(device);
	BUG_ON(pdev == NULL);

	return ret;
}

int charger_pretype_get(void)
{
	return charger_pretype;
}

static struct dev_pm_ops battery_pm_ops = {
    .suspend = battery_pm_suspend,
    .resume = battery_pm_resume,
    .freeze = battery_pm_freeze,
    .thaw = battery_pm_restore,
    .restore = battery_pm_restore,
    .restore_noirq = battery_pm_restore_noirq,
};

#if 0
static const struct of_device_id mt_battery_of_match[] = {
	{ .compatible = "mediatek,battery", },
	{},
};

MODULE_DEVICE_TABLE(of, mt_battery_of_match);
#endif

static struct platform_driver battery_driver = {
	.probe = battery_probe,
	.remove = battery_remove,
	.shutdown = battery_shutdown,
	.driver = {
		.name = "battery",
		#if 0	
        .of_match_table = mt_battery_of_match,
        #endif
		.pm = &battery_pm_ops,		
		   },
};

static struct platform_device battery_device = {
    .name   = "battery",
    .id        = -1,
};

//----------------------------------------------------------------------------------------------------




//-----------------------------------------------------------------------------------------------------
static int battery_pm_event(struct notifier_block *notifier, unsigned long pm_event, void *unused)
{
    switch(pm_event) {
	case PM_HIBERNATION_PREPARE: /* Going to hibernate */
	case PM_RESTORE_PREPARE: /* Going to restore a saved image */
	case PM_SUSPEND_PREPARE: /* Going to suspend the system */
		pr_warn("[%s] pm_event %lu\n", __func__, pm_event);
		battery_timer_pause();
		return NOTIFY_DONE;
	case PM_POST_HIBERNATION: /* Hibernation finished */
	case PM_POST_SUSPEND: /* Suspend finished */
	case PM_POST_RESTORE: /* Restore failed */
		pr_warn("[%s] pm_event %lu\n", __func__, pm_event);
		battery_timer_resume();
		return NOTIFY_DONE;
    }
    return NOTIFY_OK;
}

static struct notifier_block battery_pm_notifier_block = {
    .notifier_call = battery_pm_event,
    .priority = 0,
};


static int __init battery_init(void)
{
	int ret;

	chg_debug( "battery_init\r\n");
	ret = platform_device_register(&battery_device); 	
	ret = platform_driver_register(&battery_driver);
	if (ret) {
        battery_xlog_printk(BAT_LOG_CRTI, "****[battery_driver] Unable to device register (%d)\n", ret);
        return ret;
    }  


    ret = register_pm_notifier(&battery_pm_notifier_block);
    if (ret)
        chg_err("failed to register PM notifier %d\n", ret);

    battery_xlog_printk(BAT_LOG_CRTI, "****[battery_driver] Initialization : DONE !!\n");

	return 0;

}

static void __exit battery_exit(void)
{
}


module_init(battery_init);
module_exit(battery_exit);
MODULE_AUTHOR("HeHe!");
MODULE_DESCRIPTION("charger pmic Driver");
MODULE_LICENSE("GPL");
