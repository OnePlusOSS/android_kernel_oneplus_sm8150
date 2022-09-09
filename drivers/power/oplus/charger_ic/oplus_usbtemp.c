#include <linux/ktime.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "../oplus_charger.h"
#include "../oplus_gauge.h"
#include "../oplus_vooc.h"


#ifdef VENDOR_EDIT
#define USB_20C 20
#define USB_30C 30
#define USB_40C	40
#define USB_57C	57
#define USB_50C 50
#define USB_55C 55
#define USB_100C  100

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
struct wake_lock usbtemp_wakelock;
#else
struct wakeup_source *usbtemp_wakelock;
#endif

#ifdef CONFIG_OPLUS_CHARGER_MTK
#define VBUS_VOLT_THRESHOLD	3000
#else
#define VBUS_VOLT_THRESHOLD	400
#endif
#define USBTEMP_DEFAULT_VOLT_VALUE_MV 950

#define VBUS_MONITOR_INTERVAL	3000//3s

#define MIN_MONITOR_INTERVAL	50//50ms
#define MAX_MONITOR_INTERVAL	50//50ms
#define RETRY_CNT_DELAY         5 //ms
#define HIGH_TEMP_SHORT_CHECK_TIMEOUT 1000 /*ms*/

static int usbtemp_debug = 0;
module_param(usbtemp_debug, int, 0644);
#define OPEN_LOG_BIT BIT(0)
#define TEST_FUNC_BIT BIT(1)
#define TEST_CURRENT_BIT BIT(2)
MODULE_PARM_DESC(usbtemp_debug, "debug usbtemp");
#define USB_TEMP_HIGH 0x01
void oplus_usbtemp_recover_func(struct oplus_chg_chip *chip);
void oplus_set_usb_temp_high(struct oplus_chg_chip *chip)
{
	chip->usb_status |= USB_TEMP_HIGH;
}
void oplus_clear_usb_temp_high(struct oplus_chg_chip *chip)
{
	chip->usb_status = chip->usb_status & (~USB_TEMP_HIGH);
}

static void get_usb_temp(struct oplus_chg_chip *chg)
{

	int i = 0;
	chg->chg_ops->get_usbtemp_volt(chg);
	for (i = chg->len_array- 1; i >= 0; i--) {
		if (chg->con_volt[i] >= chg->usbtemp_volt_l)
			break;
		else if (i == 0)
			break;
	}

	chg->usb_temp_l = chg->con_temp[i];

	for (i = chg->len_array - 1; i >= 0; i--) {
		if (chg->con_volt[i] >= chg->usbtemp_volt_r)
			break;
		else if (i == 0)
			break;
	}

	chg->usb_temp_r = chg->con_temp[i];
	if(usbtemp_debug & TEST_FUNC_BIT){
		chg->usb_temp_r = 60;
	}
	if(usbtemp_debug & TEST_CURRENT_BIT){
		chg->usb_temp_r = 44;
	}
	if(usbtemp_debug & OPEN_LOG_BIT)
		chg_err("usb_temp_l:%d, usb_temp_r:%d\n",chg->usb_temp_l, chg->usb_temp_r);

}

int oplus_usbtemp_dischg_action(struct oplus_chg_chip *chip)
{
	int rc = 0;
#ifndef CONFIG_OPLUS_CHARGER_MTK
	struct smb_charger *chg = NULL;
	chg = &chip->pmic_spmi.smb5_chip->chg;
#endif

	if (get_eng_version() != HIGH_TEMP_AGING) {
		oplus_set_usb_temp_high(chip);
		if (oplus_vooc_get_fastchg_started() == true) {
			oplus_chg_set_chargerid_switch_val(0);
			oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);
			oplus_vooc_reset_mcu();
			//msleep(20);//wait for turn-off fastchg MOS
		}
#ifdef CONFIG_OPLUS_CHARGER_MTK
		chip->chg_ops->charging_disable();
#endif

		usleep_range(10000,10000);///msleep(10);
		vooc_enable_cp_ovp(0);
		chip->chg_ops->charger_suspend();
		usleep_range(10000,10000);
		pr_err("[oplus_usbtemp_dischg_action]:run_action");
		if(chip->chg_ops->set_typec_sinkonly != NULL){
			chip->chg_ops->set_typec_sinkonly();
		} else {
			pr_err("[oplus_usbtemp_dischg_action]: set_typec_sinkonly is null");
		}
	}

#ifndef CONFIG_OPLUS_CHARGER_MTK
	mutex_lock(&chg->pinctrl_mutex);
#endif

	if (get_eng_version() == HIGH_TEMP_AGING) {
		chg_err(" CONFIG_HIGH_TEMP_VERSION enable here,do not set vbus down \n");
		rc = pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.dischg_disable);
	} else {
		pr_err("[oplus_usbtemp_dischg_action]: set vbus down");
		rc = pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.dischg_enable);
	}

#ifndef CONFIG_OPLUS_CHARGER_MTK
		mutex_unlock(&chg->pinctrl_mutex);
#endif

	return 0;
}

void oplus_set_usbtemp_wakelock(bool value) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	if(value) {
		wake_lock(&usbtemp_wakelock);
	} else {
		wake_unlock(&usbtemp_wakelock);
	}
#else
	if(value) {
		__pm_stay_awake(usbtemp_wakelock);
	} else {
		__pm_relax(usbtemp_wakelock);
	}
#endif

}
void oplus_usbtemp_recover_func(struct oplus_chg_chip *chip)
{
	int level;
	int count_time = 0;

#ifndef CONFIG_OPLUS_CHARGER_MTK
	struct smb_charger *chg = NULL;
	chg = &chip->pmic_spmi.smb5_chip->chg;
#endif
	if(gpio_is_valid(chip->normalchg_gpio.dischg_gpio)) {
		level = gpio_get_value(chip->normalchg_gpio.dischg_gpio);
	} else {
		return;
	}

	if (level == 1) {
		oplus_set_usbtemp_wakelock(true);
		do {
			get_usb_temp(chip);
			pr_err("[OPLUS_USBTEMP] recovering......");
			msleep(2000);
			count_time++;
		} while (!(((chip->usb_temp_r < USB_55C || chip->usb_temp_r == USB_100C )
			&& (chip->usb_temp_l < USB_55C ||  chip->usb_temp_l == USB_100C )) || count_time == 30));
		oplus_set_usbtemp_wakelock(false);
		if(count_time == 30) {
			pr_err("[OPLUS_USBTEMP] temp still high");
		} else {
			chip->dischg_flag = false;
			chg_err("dischg disable...[%d]\n", chip->usbtemp_volt);
			oplus_clear_usb_temp_high(chip);
#ifndef CONFIG_OPLUS_CHARGER_MTK
			mutex_lock(&chg->pinctrl_mutex);
#endif
			pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.dischg_disable);
#ifndef CONFIG_OPLUS_CHARGER_MTK
			mutex_unlock(&chg->pinctrl_mutex);
#endif
			pr_err("[OPLUS_USBTEMP] usbtemp recover");
		}
	}
	return;
}

EXPORT_SYMBOL(oplus_usbtemp_recover_func);

#define RETRY_COUNT		3
void oplus_update_usbtemp_current_status(struct oplus_chg_chip *chip)
{
	static int limit_cur_cnt_r = 0;
	static int limit_cur_cnt_l = 0;
	static int recover_cur_cnt_r = 0;
	static int recover_cur_cnt_l = 0;

	if (!chip) {
		return;
	}

	if((chip->usb_temp_l < USB_30C || chip->usb_temp_l > USB_100C)
			&& (chip->usb_temp_r < USB_30C || chip->usb_temp_r > USB_100C)) {
		chip->smart_charge_user = SMART_CHARGE_USER_OTHER;
		chip->usbtemp_cool_down = 0;
		limit_cur_cnt_r = 0;
		recover_cur_cnt_r = 0;
		limit_cur_cnt_l = 0;
		recover_cur_cnt_l = 0;
		return;
	}

	if ((chip->usb_temp_r  - chip->tbatt_temp/10) >= chip->usbtemp_batttemp_gap) {
		limit_cur_cnt_r++;
		recover_cur_cnt_r = 0;
	} else if ((chip->usb_temp_r  - chip->tbatt_temp/10) <= 6)  {
		recover_cur_cnt_r++;
		limit_cur_cnt_r = 0;
	}

	if ((chip->usb_temp_l  - chip->tbatt_temp/10) >= chip->usbtemp_batttemp_gap) {
		limit_cur_cnt_l++;
		recover_cur_cnt_l = 0;
	} else if ((chip->usb_temp_l  - chip->tbatt_temp/10) <= 6)  {
		recover_cur_cnt_l++;
		limit_cur_cnt_l = 0;
	}

	if (RETRY_COUNT <= limit_cur_cnt_r || RETRY_COUNT <= limit_cur_cnt_l) {
		chip->smart_charge_user = SMART_CHARGE_USER_USBTEMP;
		chip->cool_down_done = true;
		limit_cur_cnt_r = 0;
		recover_cur_cnt_r = 0;
		limit_cur_cnt_l = 0;
		recover_cur_cnt_l = 0;
	} else if (RETRY_COUNT <= recover_cur_cnt_r && RETRY_COUNT <= recover_cur_cnt_l) {
		chip->smart_charge_user = SMART_CHARGE_USER_OTHER;
		chip->usbtemp_cool_down = 0;
		limit_cur_cnt_r = 0;
		recover_cur_cnt_r = 0;
		limit_cur_cnt_l = 0;
		recover_cur_cnt_l = 0;
	}

	return;
}


int oplus_usbtemp_monitor_common(void *data)
{
	int delay = 0;
	int vbus_volt = 0;
	static int count = 0;
	static int total_count = 0;
	static int last_usb_temp_l = 25;
	static int current_temp_l = 25;
	static int last_usb_temp_r = 25;
	static int current_temp_r = 25;
	int retry_cnt = 3, i = 0;
	int count_r = 1, count_l = 1;
	bool condition1 = false;
	bool condition2 = false;
	struct oplus_chg_chip *chip = (struct oplus_chg_chip *) data;
#ifndef CONFIG_OPLUS_CHARGER_MTK
	struct smb_charger *chg = NULL;
	chg = &chip->pmic_spmi.smb5_chip->chg;
#endif
	pr_err("[oplus_usbtemp_monitor_main]:run first!");

	while (!kthread_should_stop()) {
		if(chip->chg_ops->oplus_usbtemp_monitor_condition != NULL){
			wait_event_interruptible(chip->oplus_usbtemp_wq,chip->chg_ops->oplus_usbtemp_monitor_condition());
		} else {
			pr_err("[oplus_usbtemp_monitor_main]:condition pointer is NULL");
			return 0;
		}
		if(chip->dischg_flag == true){
			goto dischg;
		}
		if(chip->chg_ops->get_usbtemp_volt == NULL){
			pr_err("[oplus_usbtemp_monitor_main]:get_usbtemp_volt is NULL");
			return 0;
		}
		get_usb_temp(chip);
		if ((chip->usb_temp_l < USB_50C) && (chip->usb_temp_r < USB_50C)){//get vbus when usbtemp < 50C
			vbus_volt = chip->chg_ops->get_charger_volt();
		} else{
			vbus_volt = 0;
		}
		if ((chip->usb_temp_l < USB_40C) && (chip->usb_temp_r < USB_40C)) {
			delay = MAX_MONITOR_INTERVAL;
            		total_count = 10;
        	} else {
            		delay = MIN_MONITOR_INTERVAL;
            		total_count = 30;
       		}
			
		oplus_update_usbtemp_current_status(chip);

		if ((chip->usbtemp_volt_l < USB_50C) && (chip->usbtemp_volt_r < USB_50C) && (vbus_volt < VBUS_VOLT_THRESHOLD))
			delay = VBUS_MONITOR_INTERVAL;
		//condition1  :the temp is higher than 57
		if (chip->tbatt_temp/10 <= USB_50C &&(((chip->usb_temp_l >= USB_57C) && (chip->usb_temp_l < USB_100C))
			|| ((chip->usb_temp_r >= USB_57C) && (chip->usb_temp_r < USB_100C)))) {
			pr_err("in loop 1");
			for (i = 1; i < retry_cnt; i++) {
				mdelay(RETRY_CNT_DELAY);
				get_usb_temp(chip);
				if (chip->usb_temp_r >= USB_57C && chip->usb_temp_r < USB_100C)
					count_r++;
				if (chip->usb_temp_l >= USB_57C && chip->usb_temp_r < USB_100C)
					count_l++;
				pr_err("countl : %d",count_l);
			}
			if (count_r >= retry_cnt || count_l >= retry_cnt) {
				if (!IS_ERR_OR_NULL(chip->normalchg_gpio.dischg_enable)) {
					chip->dischg_flag = true;
					condition1 = true;
					chg_err("dischg enable1...[%d, %d]\n", chip->usb_temp_l, chip->usb_temp_r);
				}
			}
			count_r = 1;
			count_l = 1;
			count = 0;
			last_usb_temp_r = chip->usb_temp_r;
			last_usb_temp_l = chip->usb_temp_l;
		}
		if (chip->tbatt_temp/10 > USB_50C && (((chip->usb_temp_l >= chip->tbatt_temp/10 + 7) && (chip->usb_temp_l < USB_100C))
			|| ((chip->usb_temp_r >= chip->tbatt_temp/10 + 7) && (chip->usb_temp_r < USB_100C)))) {
			pr_err("in loop 1");
			for (i = 1; i <= retry_cnt; i++) {
				mdelay(RETRY_CNT_DELAY);
				get_usb_temp(chip);
				if (chip->usb_temp_r >= chip->tbatt_temp/10 + 7 && chip->usb_temp_r < USB_100C)
					count_r++;
				if (chip->usb_temp_l >= chip->tbatt_temp/10 + 7 && chip->usb_temp_r < USB_100C)
					count_l++;
				pr_err("countl : %d", count_l);
			}
			if (count_r >= retry_cnt || count_l >= retry_cnt) {
				if (!IS_ERR_OR_NULL(chip->normalchg_gpio.dischg_enable)) {
					chip->dischg_flag = true;
					condition1 = true;
					chg_err("dischg enable1...[%d, %d]\n", chip->usb_temp_l, chip->usb_temp_r);
				}
			}
			count_r = 1;
			count_l = 1;
			count = 0;
			last_usb_temp_r = chip->usb_temp_r;
			last_usb_temp_l = chip->usb_temp_l;
		}
		if(condition1 == true){
			pr_err("jump_to_dischg");
			goto dischg;
		}

		//condition2  :the temp uprising to fast
		if (((chip->usb_temp_l - chip->tbatt_temp/10) > chip->usbtemp_batttemp_high_gap && chip->usb_temp_l < USB_100C)
			|| ((chip->usb_temp_r - chip->tbatt_temp/10) > chip->usbtemp_batttemp_high_gap && chip->usb_temp_r < USB_100C)) {
			pr_err("in loop 2");
			if (count == 0) {
				last_usb_temp_r = chip->usb_temp_r;
				last_usb_temp_l = chip->usb_temp_l;
			} else {
				current_temp_r = chip->usb_temp_r;
				current_temp_l = chip->usb_temp_l;
			}
			if (((current_temp_l - last_usb_temp_l) >= 3) || (current_temp_r - last_usb_temp_r) >= 3) {
				for (i = 1; i < retry_cnt; i++) {
					mdelay(RETRY_CNT_DELAY);
					get_usb_temp(chip);
					if ((chip->usb_temp_r - last_usb_temp_r) >= 3 && chip->usb_temp_r < USB_100C)
						count_r++;
					if ((chip->usb_temp_l - last_usb_temp_l) >= 3 && chip->usb_temp_r < USB_100C)
						count_l++;
					pr_err("countl : %d",count_l);
                            	}
				current_temp_l = chip->usb_temp_l;
				current_temp_r = chip->usb_temp_r;
				if (((count_l >= retry_cnt) && (chip->usb_temp_l < USB_100C))
					|| ((count_r >= retry_cnt) && (chip->usb_temp_r < USB_100C))) {
					if (!IS_ERR_OR_NULL(chip->normalchg_gpio.dischg_enable)) {
						chip->dischg_flag = true;
						chg_err("dischg enable3...,current_temp_l=%d,last_usb_temp_l=%d,current_temp_r=%d,last_usb_temp_r =%d\n",
								current_temp_l, last_usb_temp_l, current_temp_r, last_usb_temp_r);
						condition2 = true;
					}
				}
				count_r = 1;
				count_l = 1;
			}
			count++;
			if (count > total_count)
				count = 0;
		} else {
			count = 0;
			last_usb_temp_r = chip->usb_temp_r;
			last_usb_temp_l = chip->usb_temp_l;
		}
	//judge whether to go the action
	dischg:
		if ((chip->usb_temp_l < USB_30C || chip->usb_temp_l > USB_100C)
				&& (chip->usb_temp_r < USB_30C || chip->usb_temp_r > USB_100C)) {
			condition1 = false;
			condition2 = false;
			chip->dischg_flag = false;
		}

		if((condition1==true || condition2==true) && chip->dischg_flag == true){
			oplus_usbtemp_dischg_action(chip);
			condition1 = false;
			condition2 = false;
		}
		msleep(delay);
		if(usbtemp_debug & OPEN_LOG_BIT){
			pr_err("usbtemp: delay %d",delay);
			chg_err("==================usbtemp_volt_l[%d], usb_temp_l[%d], usbtemp_volt_r[%d], usb_temp_r[%d], batt_temperature:%d\n",
				chip->usbtemp_volt_l,chip->usb_temp_l, chip->usbtemp_volt_r, chip->usb_temp_r, chip->tbatt_temp);
		}
	}
	return 0;
}
EXPORT_SYMBOL(oplus_usbtemp_monitor_common);
#endif

