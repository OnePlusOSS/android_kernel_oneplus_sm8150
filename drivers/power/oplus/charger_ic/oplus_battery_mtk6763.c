/**********************************************************************************
* Copyright (c)  2008-2015  Guangdong OPLUS Mobile Comm Corp., Ltd
* OPLUS_FEATURE_CHG_BASIC
* Description: Charger IC management module for charger system framework.
*              Manage all charger IC and define abstarct function flow.
* Version   : 1.0
* Date      : 2015-06-22
* 			: Fanhong.Kong@ProDrv.CHG
* ------------------------------ Revision History: --------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2015-06-22    Fanhong.Kong@ProDrv.CHG   		Created for new architecture
***********************************************************************************/

#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/wait.h>		/* For wait queue*/
#include <linux/sched.h>	/* For wait queue*/
#include <linux/kthread.h>	/* For Kthread_run */
#include <linux/platform_device.h>	/* platform device */
#include <linux/time.h>
#include <linux/wakelock.h>

#include <linux/netlink.h>	/* netlink */
#include <linux/kernel.h>
#include <linux/socket.h>	/* netlink */
#include <linux/skbuff.h>	/* netlink */
#include <net/sock.h>		/* netlink */
#include <linux/cdev.h>		/* cdev */
#include <linux/gpio.h>
#include <mt-plat/mtk_gpio.h>


#include <linux/err.h>	/* IS_ERR, PTR_ERR */
#include <linux/reboot.h>	/*kernel_power_off*/
#include <linux/proc_fs.h>

#include <linux/vmalloc.h>
#include <linux/power_supply.h>
#include <mach/mtk_charging.h>
#include <mt-plat/charging.h>
#include <oplus_bq24196.h>

#include "../oplus_gauge.h"

int mt_power_supply_type_check(void)
{
	int charger_type = POWER_SUPPLY_TYPE_UNKNOWN;
	int charger_type_final = 0;

//	pr_err("mt_power_supply_type_check-----1---------charger_type = %d,charger_type_first = %d\r\n",charger_type,charger_type_first);
	charger_type_final = mt_charger_type_detection();

	switch(charger_type_final) {
	case CHARGER_UNKNOWN:
		break;
	case STANDARD_HOST:
	case CHARGING_HOST:
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
	pr_err("mt_power_supply_type_check-----2---------charger_type = %d,charger_type_final = %d\r\n",charger_type,charger_type_final);

	return charger_type;

}


enum {
    Channel_12 = 2,
    Channel_13,
    Channel_14,
};
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int IMM_IsAdcInitReady(void);
int mt_vadc_read(int times, int Channel)
{
    int ret = 0, data[4], i, ret_value = 0, ret_temp = 0;
    if( IMM_IsAdcInitReady() == 0 )
    {
        return 0;
    }
    i = times ;
    while (i--)
    {
	ret_value = IMM_GetOneChannelValue(Channel, data, &ret_temp);
	if(ret_value != 0)
	{
		i++;
		continue;
	}
	ret += ret_temp;
    }
	ret = ret*1500/4096;
    ret = ret/times;
	chg_debug("[mt_vadc_read] Channel %d: vol_ret=%d\n",Channel,ret);
	return ret;
}
static void set_usbswitch_to_rxtx(struct oplus_chg_chip *chip)
{

	int gpio_status = 0;
	int ret = 0;
	gpio_direction_output(chip->usb_switch_gpio, 1);
	ret = pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.charger_gpio_as_output2);
	if (ret < 0) {
		chg_err("failed to set pinctrl int\n");
		return ;
	}
}
static void set_usbswitch_to_dpdm(struct oplus_chg_chip *chip)
{
	int gpio_status = 0;
	int ret = 0;
	gpio_direction_output(chip->usb_switch_gpio, 0);
	ret = pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.charger_gpio_as_output1);
	if (ret < 0) {
		chg_err("failed to set pinctrl int\n");
		return ;
	}
	chg_err("set_usbswitch_to_dpdm \n");
}
static bool is_support_chargerid_check(void)
{

#ifdef CONFIG_OPLUS_CHECK_CHARGERID_VOLT
	return true;
#else
	return false;
#endif

}
int mt_get_chargerid_volt (struct oplus_chg_chip *chip)
{
	int chargerid_volt = 0;
	if(is_support_chargerid_check() == true)
	{
		chargerid_volt = mt_vadc_read(10,Channel_14);//get the charger id volt
		chg_debug("chargerid_volt = %d \n",
					   chargerid_volt);
	}
		else
		{
		chg_debug("is_support_chargerid_check = false !\n");
		return 0;
	}
	return chargerid_volt;
		}


void mt_set_chargerid_switch_val(struct oplus_chg_chip *chip, int value)
{
	chg_debug("set_value= %d\n",value);
	if(NULL == chip)
		return;
	if(is_support_chargerid_check() == false)
		return;
	if(chip->usb_switch_gpio <= 0) {
		chg_err("usb_shell_ctrl_gpio not exist, return\n");
		return;
	}
	if(IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.charger_gpio_as_output1)
		|| IS_ERR_OR_NULL(chip->normalchg_gpio.charger_gpio_as_output2)) {
		chg_err("pinctrl null, return\n");
		return;
	}
	if(1 == value){
			set_usbswitch_to_rxtx(chip);
	}else if(0 == value){
		set_usbswitch_to_dpdm(chip);
	}else{
		//do nothing
	}
	chg_debug("get_val:%d\n",gpio_get_value(chip->usb_switch_gpio));
}

int mt_get_chargerid_switch_val(struct oplus_chg_chip *chip)
{
	int gpio_status = 0;
	if(NULL == chip)
		return 0;
	if(is_support_chargerid_check() == false)
		return 0;
	gpio_status = gpio_get_value(chip->usb_switch_gpio);

	chg_debug("mt_get_chargerid_switch_val:%d\n",gpio_status);
	return gpio_status;
}




void mt_set_usb_shell_ctrl_val(struct oplus_chg_chip *chip, int value)
{
	if(chip->usb_shell_ctrl_gpio.usb_shell_gpio <= 0) {
		chg_err("usb_shell_ctrl_gpio not exist, return\n");
		return;
	}
	if(IS_ERR_OR_NULL(chip->usb_shell_ctrl_gpio.pinctrl)
		|| IS_ERR_OR_NULL(chip->usb_shell_ctrl_gpio.usb_shell_ctrl_active)
		|| IS_ERR_OR_NULL(chip->usb_shell_ctrl_gpio.usb_shell_ctrl_sleep)) {
		chg_err("pinctrl null, return\n");
		return;
	}
	if(value) {
		gpio_direction_output(chip->usb_shell_ctrl_gpio.usb_shell_gpio, 1);
		pinctrl_select_state(chip->usb_shell_ctrl_gpio.pinctrl,
				chip->usb_shell_ctrl_gpio.usb_shell_ctrl_active);
	} else {
		gpio_direction_output(chip->usb_shell_ctrl_gpio.usb_shell_gpio, 0);
		pinctrl_select_state(chip->usb_shell_ctrl_gpio.pinctrl,
				chip->usb_shell_ctrl_gpio.usb_shell_ctrl_sleep);
	}
	chg_err("set value:%d, gpio_val:%d\n",
		value, gpio_get_value(chip->usb_shell_ctrl_gpio.usb_shell_gpio));
	
}

int mt_get_usb_shell_ctrl_val(struct oplus_chg_chip *chip)
{
	int gpio_status = -1;
	if(chip->usb_shell_ctrl_gpio.usb_shell_gpio <= 0) {
		chg_err("usb_shell_ctrl_gpio not exist, return\n");
		return -1;
	}
	gpio_status = gpio_get_value(chip->usb_shell_ctrl_gpio.usb_shell_gpio);
	chg_debug("get_usb_shell_ctrl:%d\n",gpio_status);
	return gpio_status;
}

int oplus_usb_shell_ctrl_switch_gpio_init(struct oplus_chg_chip *chip)
{

	chg_err("---1-----");
	chip->usb_shell_ctrl_gpio.pinctrl  = devm_pinctrl_get(chip->dev);
	if(IS_ERR_OR_NULL(chip->usb_shell_ctrl_gpio.pinctrl)){
		chg_err("get usb_shell_ctrl_gpio pinctrl falil\n");
		return -EINVAL;
	}
	chg_err("---2-----");
	chip->usb_shell_ctrl_gpio.usb_shell_ctrl_active = 
			pinctrl_lookup_state(chip->usb_shell_ctrl_gpio.pinctrl,"usb_shell_ctrl_output_high");
	if (IS_ERR_OR_NULL(chip->usb_shell_ctrl_gpio.usb_shell_ctrl_active)) {
		chg_err("get chargerid_switch_active fail\n");
		return -EINVAL;
	}

	chip->usb_shell_ctrl_gpio.usb_shell_ctrl_sleep = 
			pinctrl_lookup_state(chip->usb_shell_ctrl_gpio.pinctrl,"usb_shell_ctrl_output_low");
	if (IS_ERR_OR_NULL(chip->usb_shell_ctrl_gpio.usb_shell_ctrl_sleep)) {
		chg_err("get chargerid_switch_active fail\n");
		return -EINVAL;
	}
	pinctrl_select_state(chip->usb_shell_ctrl_gpio.pinctrl,
		chip->usb_shell_ctrl_gpio.usb_shell_ctrl_sleep);

	return 0;
	
}

int oplus_usb_switch_gpio_gpio_init(struct oplus_chg_chip *chip)
{
	int rc;
	chg_err("---1-----");
	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);
    if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)) {
       chg_err("get usb_switch_gpio pinctrl falil\n");
		return -EINVAL;
    }
    chip->normalchg_gpio.charger_gpio_as_output1 = pinctrl_lookup_state(chip->normalchg_gpio.pinctrl,
								"charger_gpio_as_output_low");
    if (IS_ERR_OR_NULL(chip->normalchg_gpio.charger_gpio_as_output1)) {
       	chg_err("get charger_gpio_as_output_low fail\n");
			return -EINVAL;
    }
	chip->normalchg_gpio.charger_gpio_as_output2 = pinctrl_lookup_state(chip->normalchg_gpio.pinctrl,
								"charger_gpio_as_output_high");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.charger_gpio_as_output2)) {
		chg_err("get charger_gpio_as_output_high fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chip->normalchg_gpio.pinctrl,chip->normalchg_gpio.charger_gpio_as_output1);	
	return 0;
}


int charger_pretype_get(void)
{
	int chg_type = STANDARD_HOST;
	chg_type = hw_charging_get_charger_type();
	return chg_type;
}

int oplus_battery_meter_get_battery_voltage(void)
{
	return battery_get_bat_voltage();
}

bool oplus_pmic_check_chip_is_null(void)
{
	if (!oplus_fuelgauged_init_flag) {
		return true;
	} else {
		return false;
	}
}

