/************************************************************************************
** OPLUS_FEATURE_CHG_BASIC
** Copyright (C), 2018-2019, OPLUS Mobile Comm Corp., Ltd
**
** Description:
**    For P80 charger ic driver
**
** Version: 1.0
** Date created: 2018-11-09
**
** --------------------------- Revision History: ------------------------------------
* <version>       <date>         <author>              			<desc>
*************************************************************************************/
#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
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
#include <linux/pm_wakeup.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/reboot.h>

#include <mt-plat/charger_type.h>
#include <mt-plat/mtk_battery.h>
#include <mt-plat/mtk_boot.h>
#include <pmic.h>
#include <mtk_gauge_time_service.h>


#ifdef OPLUS_FEATURE_CHG_BASIC
//====================================================================//
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include "../oplus_charger.h"
#include "../oplus_gauge.h"
#include "../oplus_vooc.h"
#include "../oplus_adapter.h"
#include "../oplus_short.h"
#include "../gauge_ic/oplus_bq27541.h"
#include "op_charge.h"
#include "../../../misc/mediatek/typec/tcpc/inc/tcpci.h"

struct oplus_chg_chip *g_oplus_chip = NULL;
static int battery_meter_get_charger_voltage(void);
static int oplus_mt6360_reset_charger(void);
static int oplus_mt6360_enable_charging(void);
static int oplus_mt6360_disable_charging(void);
static int oplus_mt6360_float_voltage_write(int vflaot_mv);
static int oplus_mt6360_suspend_charger(void);
static int oplus_mt6360_unsuspend_charger(void);
static int oplus_mt6360_charging_current_write_fast(int chg_curr);
static int oplus_mt6360_set_termchg_current(int term_curr);
static int oplus_mt6360_set_rechg_voltage(int rechg_mv);
int oplus_tbatt_power_off_task_init(struct oplus_chg_chip *chip);

static struct task_struct *oplus_usbtemp_kthread;
static DECLARE_WAIT_QUEUE_HEAD(oplus_usbtemp_wq);
void oplus_set_otg_switch_status(bool value);
void oplus_wake_up_usbtemp_thread(void);
//====================================================================//
#endif /* OPLUS_FEATURE_CHG_BASIC */


//====================================================================//
#ifdef OPLUS_FEATURE_CHG_BASIC
#define USB_TEMP_HIGH		0x01//bit0
#define USB_WATER_DETECT	0x02//bit1
#define USB_RESERVE2		0x04//bit2
#define USB_RESERVE3		0x08//bit3
#define USB_RESERVE4		0x10//bit4
#define USB_DONOT_USE		0x80000000//bit31
static int usb_status = 0;
static void oplus_set_usb_status(int status)
{
	usb_status = usb_status | status;
}

static void oplus_clear_usb_status(int status)
{
	usb_status = usb_status & (~status);
}

static int oplus_get_usb_status(void)
{
	return usb_status;
}
#endif /* OPLUS_FEATURE_CHG_BASIC */
//====================================================================//


static struct charger_manager *pinfo;
static struct list_head consumer_head = LIST_HEAD_INIT(consumer_head);
static DEFINE_MUTEX(consumer_mutex);


bool is_power_path_supported(void)
{
	if (pinfo == NULL)
		return false;

	if (pinfo->data.power_path_support == true)
		return true;

	return false;
}

bool is_disable_charger(void)
{
	if (pinfo == NULL)
		return true;

	if (pinfo->disable_charger == true || IS_ENABLED(CONFIG_POWER_EXT))
		return true;
	else
		return false;
}

void BATTERY_SetUSBState(int usb_state_value)
{
	if (is_disable_charger()) {
		chr_err("[BATTERY_SetUSBState] in FPGA/EVB, no service\n");
	} else {
		if ((usb_state_value < USB_SUSPEND) ||
			((usb_state_value > USB_CONFIGURED))) {
			chr_err("%s Fail! Restore to default value\n",
				__func__);
			usb_state_value = USB_UNCONFIGURED;
		} else {
			chr_err("%s Success! Set %d\n", __func__,
				usb_state_value);
			if (pinfo)
				pinfo->usb_state = usb_state_value;
		}
	}
}

unsigned int set_chr_input_current_limit(int current_limit)
{
	return 500;
}

int get_chr_temperature(int *min_temp, int *max_temp)
{
	*min_temp = 25;
	*max_temp = 30;

	return 0;
}

int set_chr_boost_current_limit(unsigned int current_limit)
{
	return 0;
}

int set_chr_enable_otg(unsigned int enable)
{
	return 0;
}

int mtk_chr_is_charger_exist(unsigned char *exist)
{
	if (mt_get_charger_type() == CHARGER_UNKNOWN)
		*exist = 0;
	else
		*exist = 1;
	return 0;
}

/*=============== fix me==================*/
int chargerlog_level = CHRLOG_ERROR_LEVEL;

int chr_get_debug_level(void)
{
	return chargerlog_level;
}

#ifdef MTK_CHARGER_EXP
#include <linux/string.h>

char chargerlog[1000];
#define LOG_LENGTH 500
int chargerlog_level = 10;
int chargerlogIdx;

int charger_get_debug_level(void)
{
	return chargerlog_level;
}

void charger_log(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vsprintf(chargerlog + chargerlogIdx, fmt, args);
	va_end(args);
	chargerlogIdx = strlen(chargerlog);
	if (chargerlogIdx >= LOG_LENGTH) {
		chr_err("%s", chargerlog);
		chargerlogIdx = 0;
		memset(chargerlog, 0, 1000);
	}
}

void charger_log_flash(const char *fmt, ...)
{
	va_list args;

	va_start(args, fmt);
	vsprintf(chargerlog + chargerlogIdx, fmt, args);
	va_end(args);
	chr_err("%s", chargerlog);
	chargerlogIdx = 0;
	memset(chargerlog, 0, 1000);
}
#endif

void _wake_up_charger(struct charger_manager *info)
{
#ifdef OPLUS_FEATURE_CHG_BASIC
	return;
#else
	unsigned long flags;

	if (info == NULL)
		return;

	spin_lock_irqsave(&info->slock, flags);
	if (!info->charger_wakelock.active)
		__pm_stay_awake(&info->charger_wakelock);
	spin_unlock_irqrestore(&info->slock, flags);
	info->charger_thread_timeout = true;
	wake_up(&info->wait_que);
#endif /* OPLUS_FEATURE_CHG_BASIC */
}

/* charger_manager ops  */
static int _mtk_charger_change_current_setting(struct charger_manager *info)
{
	if (info != NULL && info->change_current_setting)
		return info->change_current_setting(info);

	return 0;
}

static int _mtk_charger_do_charging(struct charger_manager *info, bool en)
{
	if (info != NULL && info->do_charging)
		info->do_charging(info, en);
	return 0;
}
/* charger_manager ops end */


/* user interface */
struct charger_consumer *charger_manager_get_by_name(struct device *dev,
	const char *name)
{
	struct charger_consumer *puser;

	puser = kzalloc(sizeof(struct charger_consumer), GFP_KERNEL);
	if (puser == NULL)
		return NULL;

	mutex_lock(&consumer_mutex);
	puser->dev = dev;

	list_add(&puser->list, &consumer_head);
	if (pinfo != NULL)
		puser->cm = pinfo;

	mutex_unlock(&consumer_mutex);

	return puser;
}
EXPORT_SYMBOL(charger_manager_get_by_name);

int charger_manager_enable_high_voltage_charging(
			struct charger_consumer *consumer, bool en)
{
	struct charger_manager *info = consumer->cm;
	struct list_head *pos;
	struct list_head *phead = &consumer_head;
	struct charger_consumer *ptr;

	if (!info)
		return -EINVAL;

	pr_debug("[%s] %s, %d\n", __func__, dev_name(consumer->dev), en);

	if (!en && consumer->hv_charging_disabled == false)
		consumer->hv_charging_disabled = true;
	else if (en && consumer->hv_charging_disabled == true)
		consumer->hv_charging_disabled = false;
	else {
		pr_info("[%s] already set: %d %d\n", __func__,
			consumer->hv_charging_disabled, en);
		return 0;
	}

	mutex_lock(&consumer_mutex);
	list_for_each(pos, phead) {
		ptr = container_of(pos, struct charger_consumer, list);
		if (ptr->hv_charging_disabled == true) {
			info->enable_hv_charging = false;
			break;
		}
		if (list_is_last(pos, phead))
			info->enable_hv_charging = true;
	}
	mutex_unlock(&consumer_mutex);

	pr_info("%s: user: %s, en = %d\n", __func__, dev_name(consumer->dev),
		info->enable_hv_charging);
	_wake_up_charger(info);

	return 0;
}
EXPORT_SYMBOL(charger_manager_enable_high_voltage_charging);

int charger_manager_enable_power_path(struct charger_consumer *consumer,
	int idx, bool en)
{
	int ret = 0;
	bool is_en = true;
	struct charger_manager *info = consumer->cm;
	struct charger_device *chg_dev;


	if (!info)
		return -EINVAL;

	switch (idx) {
	case MAIN_CHARGER:
		chg_dev = info->chg1_dev;
		break;
	case SLAVE_CHARGER:
		chg_dev = info->chg2_dev;
		break;
	default:
		return -EINVAL;
	}

	ret = charger_dev_is_powerpath_enabled(chg_dev, &is_en);
	if (ret < 0) {
		chr_err("%s: get is power path enabled failed\n", __func__);
		return ret;
	}
	if (is_en == en) {
		chr_err("%s: power path is already en = %d\n", __func__, is_en);
		return 0;
	}

	pr_info("%s: enable power path = %d\n", __func__, en);
	return charger_dev_enable_powerpath(chg_dev, en);
}

static int _charger_manager_enable_charging(struct charger_consumer *consumer,
	int idx, bool en)
{
	struct charger_manager *info = consumer->cm;

	chr_err("%s: dev:%s idx:%d en:%d\n", __func__, dev_name(consumer->dev),
		idx, en);

	if (info != NULL) {
		struct charger_data *pdata;

		if (idx == MAIN_CHARGER)
			pdata = &info->chg1_data;
		else if (idx == SLAVE_CHARGER)
			pdata = &info->chg2_data;
		else
			return -ENOTSUPP;

		if (en == false) {
			_mtk_charger_do_charging(info, en);
			pdata->disable_charging_count++;
		} else {
			if (pdata->disable_charging_count == 1) {
				_mtk_charger_do_charging(info, en);
				pdata->disable_charging_count = 0;
			} else if (pdata->disable_charging_count > 1)
				pdata->disable_charging_count--;
		}
		chr_err("%s: dev:%s idx:%d en:%d cnt:%d\n", __func__,
			dev_name(consumer->dev), idx, en,
			pdata->disable_charging_count);

		return 0;
	}
	return -EBUSY;

}

int charger_manager_enable_charging(struct charger_consumer *consumer,
	int idx, bool en)
{
	struct charger_manager *info = consumer->cm;
	int ret = 0;

	mutex_lock(&info->charger_lock);
	ret = _charger_manager_enable_charging(consumer, idx, en);
	mutex_unlock(&info->charger_lock);
	return ret;
}

int charger_manager_set_input_current_limit(struct charger_consumer *consumer,
	int idx, int input_current)
{
	struct charger_manager *info = consumer->cm;

	if (info != NULL) {
		struct charger_data *pdata;

		if (idx == MAIN_CHARGER)
			pdata = &info->chg1_data;
		else if (idx == SLAVE_CHARGER)
			pdata = &info->chg2_data;
		else
			return -ENOTSUPP;

		pdata->thermal_input_current_limit = input_current;
		chr_err("%s: dev:%s idx:%d en:%d\n", __func__,
			dev_name(consumer->dev), idx, input_current);
		_mtk_charger_change_current_setting(info);
		_wake_up_charger(info);
		return 0;
	}
	return -EBUSY;
}

int charger_manager_set_charging_current_limit(
	struct charger_consumer *consumer, int idx, int charging_current)
{
	struct charger_manager *info = consumer->cm;

	if (info != NULL) {
		struct charger_data *pdata;

		if (idx == MAIN_CHARGER)
			pdata = &info->chg1_data;
		else if (idx == SLAVE_CHARGER)
			pdata = &info->chg2_data;
		else
			return -ENOTSUPP;

		pdata->thermal_charging_current_limit = charging_current;
		chr_err("%s: dev:%s idx:%d en:%d\n", __func__,
			dev_name(consumer->dev), idx, charging_current);
		_mtk_charger_change_current_setting(info);
		_wake_up_charger(info);
		return 0;
	}
	return -EBUSY;
}

int charger_manager_get_charger_temperature(struct charger_consumer *consumer,
	int idx, int *tchg_min,	int *tchg_max)
{
	struct charger_manager *info = consumer->cm;

	if (info != NULL) {
		struct charger_data *pdata;

		if (!upmu_get_rgs_chrdet()) {
			pr_debug("[%s] No cable in, skip it\n", __func__);
			*tchg_min = -127;
			*tchg_max = -127;
			return -EINVAL;
		}

		if (idx == MAIN_CHARGER)
			pdata = &info->chg1_data;
		else if (idx == SLAVE_CHARGER)
			pdata = &info->chg2_data;
		else
			return -ENOTSUPP;

		*tchg_min = pdata->junction_temp_min;
		*tchg_max = pdata->junction_temp_max;

		return 0;
	}
	return -EBUSY;
}

int charger_manager_get_current_charging_type(struct charger_consumer *consumer)
{
	struct charger_manager *info = consumer->cm;

	if (info != NULL) {
		if (mtk_pe20_get_is_connect(info))
			return 2;
	}

	return 0;
}

int charger_manager_get_zcv(struct charger_consumer *consumer, int idx, u32 *uV)
{
	struct charger_manager *info = consumer->cm;
	int ret = 0;
	struct charger_device *pchg;


	if (info != NULL) {
		if (idx == MAIN_CHARGER) {
			pchg = info->chg1_dev;
			ret = charger_dev_get_zcv(pchg, uV);
		} else if (idx == SLAVE_CHARGER) {
			pchg = info->chg2_dev;
			ret = charger_dev_get_zcv(pchg, uV);
		} else
			ret = -1;

	} else {
		chr_err("charger_manager_get_zcv info is null\n");
	}
	chr_err("charger_manager_get_zcv zcv:%d ret:%d\n", *uV, ret);

	return 0;
}

int charger_manager_enable_kpoc_shutdown(struct charger_consumer *consumer,
	bool en)
{
#ifndef OPLUS_FEATURE_CHG_BASIC
	struct charger_manager *info = consumer->cm;

	if (en)
		atomic_set(&info->enable_kpoc_shdn, 1);
	else
		atomic_set(&info->enable_kpoc_shdn, 0);
#endif /* OPLUS_FEATURE_CHG_BASIC */
	return 0;
}

int register_charger_manager_notifier(struct charger_consumer *consumer,
	struct notifier_block *nb)
{
	int ret = 0;
	struct charger_manager *info = consumer->cm;


	mutex_lock(&consumer_mutex);
	if (info != NULL)
	ret = srcu_notifier_chain_register(&info->evt_nh, nb);
	else
		consumer->pnb = nb;
	mutex_unlock(&consumer_mutex);

	return ret;
}

int unregister_charger_manager_notifier(struct charger_consumer *consumer,
				struct notifier_block *nb)
{
	int ret = 0;
	struct charger_manager *info = consumer->cm;

	mutex_lock(&consumer_mutex);
	if (info != NULL)
		ret = srcu_notifier_chain_unregister(&info->evt_nh, nb);
	else
		consumer->pnb = NULL;
	mutex_unlock(&consumer_mutex);

	return ret;
}

/* internal algorithm common function */
bool is_dual_charger_supported(struct charger_manager *info)
{
	if (info->chg2_dev == NULL)
		return false;
	return true;
}

int charger_enable_vbus_ovp(struct charger_manager *pinfo, bool enable)
{
	int ret = 0;
	u32 sw_ovp = 0;

	if (enable)
		sw_ovp = pinfo->data.max_charger_voltage_setting;
	else
		sw_ovp = 15000000;

	/* Enable/Disable SW OVP status */
	pinfo->data.max_charger_voltage = sw_ovp;

	chr_err("[charger_enable_vbus_ovp] en:%d ovp:%d\n",
			    enable, sw_ovp);
	return ret;
}

bool is_typec_adapter(struct charger_manager *info)
{
	if (info->pd_type == PD_CONNECT_TYPEC_ONLY_SNK &&
			tcpm_inquire_typec_remote_rp_curr(info->tcpc) != 500 &&
			info->chr_type != STANDARD_HOST &&
			info->chr_type != CHARGING_HOST &&
			mtk_pe20_get_is_connect(info) == false &&
			mtk_pe_get_is_connect(info) == false &&
			info->enable_type_c == true)
		return true;

	return false;
}

int charger_get_vbus(void)
{
	int ret = 0;
	int vchr = 0;

	if (pinfo == NULL)
		return 0;
	ret = charger_dev_get_vbus(pinfo->chg1_dev, &vchr);
	if (ret < 0)
		return 0;

	vchr = vchr / 1000;
	return vchr;
}

int mtk_get_dynamic_cv(struct charger_manager *info, unsigned int *cv)
{
	int ret = 0;
	u32 _cv, _cv_temp;
	unsigned int vbat_threshold[4] = {3400000, 0, 0, 0};
	u32 vbat_bif = 0, vbat_auxadc = 0, vbat = 0;
	u32 retry_cnt = 0;

	if (pmic_is_bif_exist()) {
		do {
			vbat_auxadc = battery_get_bat_voltage() * 1000;
			ret = pmic_get_bif_battery_voltage(&vbat_bif);
			vbat_bif = vbat_bif * 1000;
			if (ret >= 0 && vbat_bif != 0 &&
			    vbat_bif < vbat_auxadc) {
				vbat = vbat_bif;
				chr_err("%s: use BIF vbat = %duV, dV to auxadc = %duV\n",
					__func__, vbat, vbat_auxadc - vbat_bif);
				break;
			}
			retry_cnt++;
		} while (retry_cnt < 5);

		if (retry_cnt == 5) {
			ret = 0;
			vbat = vbat_auxadc;
			chr_err("%s: use AUXADC vbat = %duV, since BIF vbat = %duV\n",
				__func__, vbat_auxadc, vbat_bif);
		}

		/* Adjust CV according to the obtained vbat */
		vbat_threshold[1] = info->data.bif_threshold1;
		vbat_threshold[2] = info->data.bif_threshold2;
		_cv_temp = info->data.bif_cv_under_threshold2;

		if (!info->enable_dynamic_cv && vbat >= vbat_threshold[2]) {
			_cv = info->data.battery_cv;
			goto out;
		}

		if (vbat < vbat_threshold[1])
			_cv = 4608000;
		else if (vbat >= vbat_threshold[1] && vbat < vbat_threshold[2])
			_cv = _cv_temp;
		else {
			_cv = info->data.battery_cv;
			info->enable_dynamic_cv = false;
		}
out:
		*cv = _cv;
		chr_err("%s: CV = %duV, enable_dynamic_cv = %d\n",
			__func__, _cv, info->enable_dynamic_cv);
	} else
		ret = -ENOTSUPP;

	return ret;
}

int charger_manager_notifier(struct charger_manager *info, int event)
{
	return srcu_notifier_call_chain(&info->evt_nh, event, NULL);
}

void mtk_charger_int_handler(void)
{
#ifdef OPLUS_FEATURE_CHG_BASIC
	chr_err("mtk_charger_int_handler\n");
	if (mt_get_charger_type() != CHARGER_UNKNOWN)
		oplus_wake_up_usbtemp_thread();
#else
	chr_err("mtk_charger_int_handler\n");

	if (pinfo == NULL) {
		chr_err("charger is not rdy ,skip1\n");
		return;
	}

	if (pinfo->init_done != true) {
		chr_err("charger is not rdy ,skip2\n");
		return;
	}

	if (mt_get_charger_type() == CHARGER_UNKNOWN) {
		mutex_lock(&pinfo->cable_out_lock);
		pinfo->cable_out_cnt++;
		chr_err("cable_out_cnt=%d\n", pinfo->cable_out_cnt);
		mutex_unlock(&pinfo->cable_out_lock);
	}

	chr_err("wake_up_charger\n");
	_wake_up_charger(pinfo);
#endif /* OPLUS_FEATURE_CHG_BASIC */
}

static int mtk_chgstat_notify(struct charger_manager *info)
{
	int ret = 0;
	char *env[2] = { "CHGSTAT=1", NULL };

	chr_err("%s: 0x%x\n", __func__, info->notify_code);
	ret = kobject_uevent_env(&info->pdev->dev.kobj, KOBJ_CHANGE, env);
	if (ret)
		chr_err("%s: kobject_uevent_fail, ret=%d", __func__, ret);

	return ret;
}

static int mtk_charger_parse_dt(struct charger_manager *info,
				struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 val;

	chr_err("%s: starts\n", __func__);

	if (!np) {
		chr_err("%s: no device node\n", __func__);
		return -EINVAL;
	}

	info->enable_type_c = of_property_read_bool(np, "enable_type_c");

	info->data.power_path_support =
				of_property_read_bool(np, "power_path_support");
	chr_debug("%s: power_path_support: %d\n",
		__func__, info->data.power_path_support);

	return 0;
}

static ssize_t show_BatNotify(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct charger_manager *pinfo = dev->driver_data;

	pr_debug("[Battery] show_BatteryNotify: 0x%x\n", pinfo->notify_code);

	return sprintf(buf, "%u\n", pinfo->notify_code);
}

static ssize_t store_BatNotify(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct charger_manager *pinfo = dev->driver_data;
	unsigned int reg = 0;
	int ret;

	pr_debug("[Battery] store_BatteryNotify\n");
	if (buf != NULL && size != 0) {
		pr_debug("[Battery] buf is %s and size is %Zu\n", buf, size);
		ret = kstrtouint(buf, 16, &reg);
		pinfo->notify_code = reg;
		pr_debug("[Battery] store code: 0x%x\n", pinfo->notify_code);
		mtk_chgstat_notify(pinfo);
	}
	return size;
}

static DEVICE_ATTR(BatteryNotify, 0664, show_BatNotify, store_BatNotify);

/* Create sysfs and procfs attributes */
static int mtk_charger_setup_files(struct platform_device *pdev)
{
	int ret = 0;
	struct proc_dir_entry *battery_dir = NULL;
	struct charger_manager *info = platform_get_drvdata(pdev);
	/* struct charger_device *chg_dev; */

	/* Battery warning */
	ret = device_create_file(&(pdev->dev), &dev_attr_BatteryNotify);
	if (ret)
		goto _out;
_out:
	return ret;
}

static int pd_tcp_notifier_call(struct notifier_block *pnb,
				unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct charger_manager *pinfo;

	pinfo = container_of(pnb, struct charger_manager, pd_nb);

	chr_err("PD charger event:%d %d\n", (int)event,
		(int)noti->pd_state.connected);
	switch (event) {
	case TCP_NOTIFY_PD_STATE:
		switch (noti->pd_state.connected) {
		case  PD_CONNECT_NONE:
			mutex_lock(&pinfo->charger_pd_lock);
			chr_err("PD Notify Detach\n");
			pinfo->pd_type = PD_CONNECT_NONE;
			mutex_unlock(&pinfo->charger_pd_lock);
			/* reset PE40 */
			break;

		case PD_CONNECT_HARD_RESET:
			mutex_lock(&pinfo->charger_pd_lock);
			chr_err("PD Notify HardReset\n");
			pinfo->pd_type = PD_CONNECT_NONE;
			pinfo->pd_reset = true;
			mutex_unlock(&pinfo->charger_pd_lock);
			_wake_up_charger(pinfo);
			/* reset PE40 */
			break;

		case PD_CONNECT_PE_READY_SNK:
			mutex_lock(&pinfo->charger_pd_lock);
			chr_err("PD Notify fixe voltage ready\n");
			pinfo->pd_type = PD_CONNECT_PE_READY_SNK;
			mutex_unlock(&pinfo->charger_pd_lock);
			/* PD is ready */
			break;

		case PD_CONNECT_PE_READY_SNK_PD30:
			mutex_lock(&pinfo->charger_pd_lock);
			chr_err("PD Notify PD30 ready\r\n");
			pinfo->pd_type = PD_CONNECT_PE_READY_SNK_PD30;
			mutex_unlock(&pinfo->charger_pd_lock);
			/* PD30 is ready */
			break;

		case PD_CONNECT_PE_READY_SNK_APDO:
			mutex_lock(&pinfo->charger_pd_lock);
			chr_err("PD Notify APDO Ready\n");
			pinfo->pd_type = PD_CONNECT_PE_READY_SNK_APDO;
			mutex_unlock(&pinfo->charger_pd_lock);
			/* PE40 is ready */
			_wake_up_charger(pinfo);
			break;

		case PD_CONNECT_TYPEC_ONLY_SNK:
			mutex_lock(&pinfo->charger_pd_lock);
			chr_err("PD Notify Type-C Ready\n");
			pinfo->pd_type = PD_CONNECT_TYPEC_ONLY_SNK;
			mutex_unlock(&pinfo->charger_pd_lock);
			/* type C is ready */
			_wake_up_charger(pinfo);
			break;
		};
		break;

	case TCP_NOTIFY_WD_STATUS:
		chr_err("%s wd status = %d\n", __func__,
			noti->wd_status.water_detected);
		mutex_lock(&pinfo->charger_pd_lock);
		pinfo->water_detected = noti->wd_status.water_detected;
		mutex_unlock(&pinfo->charger_pd_lock);

		if (pinfo->water_detected == true) {
			pinfo->notify_code |= CHG_TYPEC_WD_STATUS;
#ifdef OPLUS_FEATURE_CHG_BASIC
			oplus_set_usb_status(USB_WATER_DETECT);
			oplus_vooc_set_disable_adapter_output(true);
			if (g_oplus_chip && g_oplus_chip->usb_psy)
				power_supply_changed(g_oplus_chip->usb_psy);
#endif
			mtk_chgstat_notify(pinfo);
		} else {
			pinfo->notify_code &= ~CHG_TYPEC_WD_STATUS;
#ifdef OPLUS_FEATURE_CHG_BASIC
			oplus_clear_usb_status(USB_WATER_DETECT);
			oplus_vooc_set_disable_adapter_output(false);
			if (g_oplus_chip && g_oplus_chip->usb_psy)
				power_supply_changed(g_oplus_chip->usb_psy);
#endif
			mtk_chgstat_notify(pinfo);
		}
		break;
	}
	return NOTIFY_OK;
}


#ifdef OPLUS_FEATURE_CHG_BASIC
//====================================================================//
static void oplus_mt6360_dump_registers(void)
{
	struct charger_device *chg = NULL;
	static bool musb_hdrc_release = false;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (musb_hdrc_release == false &&
				g_oplus_chip->unwakelock_chg == 1 &&
				mt_get_charger_type() == NONSTANDARD_CHARGER) {
		musb_hdrc_release = true;
		mt_usb_disconnect();
	} else {
		if (musb_hdrc_release == true &&
				g_oplus_chip->unwakelock_chg == 0 &&
				mt_get_charger_type() == NONSTANDARD_CHARGER) {
			musb_hdrc_release = false;
			mt_usb_connect();
		}
	}

	/*This function runs for more than 400ms, so return when no charger for saving power */
	if (g_oplus_chip->charger_type == POWER_SUPPLY_TYPE_UNKNOWN
			|| oplus_get_chg_powersave() == true) {
		return;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	charger_dev_dump_registers(chg);
	return;
}

static int oplus_mt6360_kick_wdt(void)
{
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	rc = charger_dev_kick_wdt(chg);
	if (rc < 0) {
		chg_debug("charger_dev_kick_wdt fail\n");
	}
	return 0;
}

static int oplus_mt6360_hardware_init(void)
{
	int hw_aicl_point = 4400;
	//int sw_aicl_point = 4500;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;

	oplus_mt6360_reset_charger();

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
		oplus_mt6360_disable_charging();
		oplus_mt6360_float_voltage_write(4400);
		msleep(100);
	}

	oplus_mt6360_float_voltage_write(4385);

	//set_complete_charge_timeout(OVERTIME_DISABLED);
	mt6360_set_register(0x1C, 0xFF, 0xF9);

	//set_prechg_current(300);
	mt6360_set_register(0x18, 0xF, 0x4);

	oplus_mt6360_charging_current_write_fast(512);

	oplus_mt6360_set_termchg_current(150);

	oplus_mt6360_set_rechg_voltage(100);

	charger_dev_set_mivr(chg, hw_aicl_point * 1000);

#ifdef CONFIG_OPLUS_CHARGER_MTK
	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
			|| get_boot_mode() == ADVMETA_BOOT || get_boot_mode() == ATE_FACTORY_BOOT) {
		oplus_mt6360_suspend_charger();
		oplus_mt6360_disable_charging();
	} else {
		oplus_mt6360_unsuspend_charger();
	}
#else /* CONFIG_OPLUS_CHARGER_MTK */
	oplus_mt6360_unsuspend_charger();
#endif /* CONFIG_OPLUS_CHARGER_MTK */

	oplus_mt6360_enable_charging();

	//set_wdt_timer(REG05_BQ25601D_WATCHDOG_TIMER_40S);
	//mt6360_set_register(0x1D, 0x80, 0x80);
	mt6360_set_register(0x1D, 0x30, 0x10);

	return 0;
}

static int oplus_mt6360_charging_current_write_fast(int chg_curr)
{
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	rc = charger_dev_set_charging_current(chg, chg_curr * 1000);
	if (rc < 0) {
		chg_debug("set fast charge current:%d fail\n", chg_curr);
	} else {
		//chg_debug("set fast charge current:%d\n", chg_curr);
	}

	return 0;
}

static void oplus_mt6360_set_aicl_point(int vbatt)
{
	int rc = 0;
	static int hw_aicl_point = 4400;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;

	if (hw_aicl_point == 4400 && vbatt > 4100) {
		hw_aicl_point = 4500;
		//sw_aicl_point = 4550;
	} else if (hw_aicl_point == 4500 && vbatt <= 4100) {
		hw_aicl_point = 4400;
		//sw_aicl_point = 4500;
	}
	rc = charger_dev_set_mivr(chg, hw_aicl_point * 1000);
	if (rc < 0) {
		chg_debug("set aicl point:%d fail\n", hw_aicl_point);
	}
}

static int usb_icl[] = {
	300, 500, 900, 1200, 1350, 1500, 2000, 3000,
};
static int oplus_mt6360_input_current_limit_write(int value)
{
	int rc = 0;
	int i = 0;
	int chg_vol = 0;
	int aicl_point = 0;
	int vbus_mv = 0;
	int ibus_ma = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg_debug("usb input max current limit=%d setting %02x\n", value, i);

	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;

	//aicl_point_temp = g_oplus_chip->sw_aicl_point;
	if (g_oplus_chip->chg_ops->oplus_chg_get_pd_type) {
		if (g_oplus_chip->chg_ops->oplus_chg_get_pd_type() == true) {
			rc = oplus_pdc_get(&vbus_mv, &ibus_ma);
			if (rc >= 0 && ibus_ma >= 500 && ibus_ma < g_oplus_chip->limits.pd_input_current_charger_ma && value > ibus_ma) {
				value = ibus_ma;
				chg_debug("usb input max current limit=%d(pd)\n", value);
			}
		}
	}

	if (g_oplus_chip->batt_volt > 4100 )
		aicl_point = 4550;
	else
		aicl_point = 4500;

	if (value < 500) {
		i = 0;
		goto aicl_end;
	}

	mt6360_aicl_enable(false);

	i = 1; /* 500 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		chg_debug( "use 500 here\n");
		goto aicl_end;
	} else if (value < 900)
		goto aicl_end;

	i = 2; /* 900 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (value < 1200)
		goto aicl_end;

	i = 3; /* 1200 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	}

	i = 4; /* 1350 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 2;
		goto aicl_pre_step;
	}

	i = 5; /* 1500 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 3; //We DO NOT use 1.2A here
		goto aicl_pre_step;
	} else if (value < 1500) {
		i = i - 2; //We use 1.2A here
		goto aicl_end;
	} else if (value < 2000)
		goto aicl_end;

	i = 6; /* 2000 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (value < 3000)
		goto aicl_end;

	i = 7; /* 3000 */
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	msleep(90);
	chg_vol = battery_meter_get_charger_voltage();
	if (chg_vol < aicl_point) {
		i = i - 1;
		goto aicl_pre_step;
	} else if (value >= 3000)
		goto aicl_end;

aicl_pre_step:		
	chg_debug("usb input max current limit aicl chg_vol=%d i[%d]=%d sw_aicl_point:%d aicl_pre_step\n", chg_vol, i, usb_icl[i], aicl_point);
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	goto aicl_rerun;
aicl_end:		
	chg_debug("usb input max current limit aicl chg_vol=%d i[%d]=%d sw_aicl_point:%d aicl_end\n", chg_vol, i, usb_icl[i], aicl_point);
	rc = charger_dev_set_input_current(chg, usb_icl[i] * 1000);
	goto aicl_rerun;
aicl_rerun:
	mt6360_aicl_enable(true);
	return rc;

}

static int oplus_mt6360_float_voltage_write(int vfloat_mv)
{
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	rc = charger_dev_set_constant_voltage(chg, vfloat_mv * 1000);
	if (rc < 0) {
		chg_debug("set float voltage:%d fail\n", vfloat_mv);
	} else {
		//chg_debug("set float voltage:%d\n", vfloat_mv);
	}

	return 0;
}

static int oplus_mt6360_set_termchg_current(int term_curr)
{
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	rc = charger_dev_set_eoc_current(chg, term_curr * 1000);
	if (rc < 0) {
		//chg_debug("set termchg_current fail\n");
	}
	return 0;
}

static int oplus_mt6360_enable_charging(void)
{
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	rc = charger_dev_enable(chg, true);
	if (rc < 0) {
		chg_debug("enable charging fail\n");
	} else {
		//chg_debug("enable charging\n");
	}

	return 0;
}

static int oplus_mt6360_disable_charging(void)
{
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	rc = charger_dev_enable(chg, false);
	if (rc < 0) {
		chg_debug("disable charging fail\n");
	} else {
		//chg_debug("disable charging\n");
	}

	return 0;
}

static int oplus_mt6360_check_charging_enable(void)
{
	return mt6360_check_charging_enable();
}

static int oplus_mt6360_suspend_charger(void)
{
	int rc = 0;

	rc = mt6360_suspend_charger(true);
	if (rc < 0) {
		chg_debug("suspend charger fail\n");
	} else {
		chg_debug("suspend charger\n");
	}
	return 0;
}

static int oplus_mt6360_unsuspend_charger(void)
{
	int rc = 0;

	rc = mt6360_suspend_charger(false);
	if (rc < 0) {
		chg_debug("unsuspend charger fail\n");
	} else {
		chg_debug("unsuspend charger\n");
	}
	return 0;
}

static int oplus_mt6360_set_rechg_voltage(int rechg_mv)
{
	int rc = 0;

	rc = mt6360_set_rechg_voltage(rechg_mv);
	if (rc < 0) {
		chg_debug("set rechg voltage fail:%d\n", rechg_mv);
	}
	return 0;
}

static int oplus_mt6360_reset_charger(void)
{
	int rc = 0;

	rc = mt6360_reset_charger();
	if (rc < 0) {
		chg_debug("reset charger fail\n");
	}
	return 0;
}

static int oplus_mt6360_registers_read_full(void)
{
	bool full = false;
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	rc = charger_dev_is_charging_done(chg, &full);
	if (rc < 0) {
		chg_debug("registers read full  fail\n");
		full = false;
	} else {
		//chg_debug("registers read full\n");
	}

	return full;
}

static int oplus_mt6360_otg_enable(void)
{
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	rc = charger_dev_enable_otg(chg, true);
	if (rc < 0) {
		chg_debug("set enable_otg fail\n");
	} else {
		chg_debug("set enable_otg success\n");
	}

	return 0;
}

static int oplus_mt6360_otg_disable(void)
{
	int rc = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	rc = charger_dev_enable_otg(chg, false);
	if (rc < 0) {
		chg_debug("set disable_otg fail\n");
	} else {
		chg_debug("set disable_otg success\n");
	}

	return 0;
}

static int oplus_mt6360_set_chging_term_disable(void)
{
	int rc = 0;

	rc = mt6360_set_chging_term_disable(true);
	if (rc < 0) {
		chg_debug("disable chging_term fail\n");
	}
	return 0;
}

static bool oplus_mt6360_check_charger_resume(void)
{
	return true;
}

static int oplus_mt6360_get_chg_current_step(void)
{
	return 100;
}

static int mt_power_supply_type_check(void)
{
	int charger_type = POWER_SUPPLY_TYPE_UNKNOWN;

	switch(mt_get_charger_type()) {
	case CHARGER_UNKNOWN:
		break;
	case STANDARD_HOST:
		charger_type = POWER_SUPPLY_TYPE_USB;
		break;
	case CHARGING_HOST:
		charger_type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case NONSTANDARD_CHARGER:
	case STANDARD_CHARGER:	
	case APPLE_2_1A_CHARGER:
	case APPLE_1_0A_CHARGER:
	case APPLE_0_5A_CHARGER:
		charger_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		break;
	}
	chg_debug("charger_type[%d]\n", charger_type);
	return charger_type;
}

static int battery_meter_get_charger_voltage(void)
{
	return battery_get_vbus();
}

static bool oplus_mt6360_get_vbus_status(void)
{
	bool vbus_status = false;
	static bool pre_vbus_status = false;
	int ret = 0;

	ret = mt6360_get_vbus_rising();
	if (ret < 0) {
		if (g_oplus_chip && g_oplus_chip->unwakelock_chg == 1
				&& g_oplus_chip->charger_type != POWER_SUPPLY_TYPE_UNKNOWN) {
			printk(KERN_ERR "[OPLUS_CHG][%s]: unwakelock_chg=1, use pre status\n", __func__);
			return pre_vbus_status;
		} else {
			return false;
		}
	}
	if (ret == 0)
		vbus_status = false;
	else
		vbus_status = true;
	pre_vbus_status = vbus_status;
	return vbus_status;
}

static int oplus_battery_meter_get_battery_voltage(void)
{
	return 4000;
}

static int get_rtc_spare_oplus_fg_value(void)
{
	return 0;
}

static int set_rtc_spare_oplus_fg_value(int soc)
{
	return 0;
}

static void oplus_mt_power_off(void)
{
	struct tcpc_device *tcpc_dev = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (g_oplus_chip->ac_online != true) {
		if (tcpc_dev == NULL) {
			tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
		}
		if (tcpc_dev) {
			if (!(tcpc_dev->pd_wait_hard_reset_complete)
					&& !oplus_mt6360_get_vbus_status()) {
				mt_power_off();
			}
		}
	} else {
		printk(KERN_ERR "[OPLUS_CHG][%s]: ac_online is true, return!\n", __func__);
	}
}

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
/* This function is getting the dynamic aicl result/input limited in mA.
 * If charger was suspended, it must return 0(mA).
 * It meets the requirements in SDM660 platform.
 */
static int oplus_mt6360_chg_get_dyna_aicl_result(void)
{
	int rc = 0;
	int aicl_ma = 0;
	struct charger_device *chg = NULL;

	if (!g_oplus_chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	chg = g_oplus_chip->chgic_mtk.oplus_info->chg1_dev;
	rc = charger_dev_get_input_current(chg, &aicl_ma);
	if (rc < 0) {
		chg_debug("get dyna aicl fail\n");
		return 500;
	}
	return aicl_ma / 1000;
}
#endif

#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
static int rtc_reset_check(void)
{
	int rc = 0;
	struct rtc_time tm;
	struct rtc_device *rtc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return 0;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	if ((tm.tm_year == 70) && (tm.tm_mon == 0) && (tm.tm_mday <= 1)) {
		chg_debug(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  @@@ wday: %d, yday: %d, isdst: %d\n",
			tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
			tm.tm_wday, tm.tm_yday, tm.tm_isdst);
		rtc_class_close(rtc);
		return 1;
	}

	chg_debug(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  ###  wday: %d, yday: %d, isdst: %d\n",
		tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
		tm.tm_wday, tm.tm_yday, tm.tm_isdst);

close_time:
	rtc_class_close(rtc);
	return 0;
}
#endif /* CONFIG_OPLUS_RTC_DET_SUPPORT */
//====================================================================//


//====================================================================//
#ifdef OPLUS_FEATURE_CHG_BASIC
enum {
	Channel_12 = 2,
	Channel_13,
	Channel_14,
	Channel_15,
};

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int IMM_IsAdcInitReady(void);

static int mt_vadc_read(int times, int Channel)
{
	int ret = 0;
	int data[4] = {0};
	int i = 0;
	int ret_value = 0;
	int ret_temp = 0;

	if (IMM_IsAdcInitReady() == 0) {
		return 0;
	}

	i = times ;
	while (i--) {
		ret_value = IMM_GetOneChannelValue(Channel, data, &ret_temp);
		if (ret_value != 0) {
			i++;
			continue;
		}
		ret += ret_temp;
	}

	ret = ret * 1500 / 4096;
	if (times != 0) {
		ret = ret / times;
	}
	//chg_debug("[mt_vadc_read] Channel %d: vol_ret=%d\n", Channel, ret);

	return ret;
}

extern bool ignore_usb;
static void set_usbswitch_to_rxtx(struct oplus_chg_chip *chip)
{
	int ret = 0;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (ignore_usb) {
		chg_err("ignore_usb is true, do not set_usbswitch_to_rxtx\n");
		return;
	}

	gpio_direction_output(chip->normalchg_gpio.chargerid_switch_gpio, 1);
	ret = pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.charger_gpio_as_output2);
	if (ret < 0) {
		chg_err("failed to set pinctrl int\n");
	}
	chg_err("set_usbswitch_to_rxtx\n");
	return;
}

static void set_usbswitch_to_dpdm(struct oplus_chg_chip *chip)
{
	int ret = 0;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	gpio_direction_output(chip->normalchg_gpio.chargerid_switch_gpio, 0);
	ret = pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.charger_gpio_as_output1);
	if (ret < 0) {
		chg_err("failed to set pinctrl int\n");
		return;
	}
	chg_err("set_usbswitch_to_dpdm\n");
}

static bool is_support_chargerid_check(void)
{
#ifdef CONFIG_OPLUS_CHECK_CHARGERID_VOLT
	return true;
#else
	return false;
#endif
}

static int mt_get_chargerid_volt(void)
{
	int chargerid_volt = 0;

	if (is_support_chargerid_check() == true) {
		chargerid_volt = mt_vadc_read(10, Channel_13);//get the charger id volt
		chg_debug("chargerid_volt=%d\n", chargerid_volt);
	} else {
		chg_debug("is_support_chargerid_check=false!\n");
		return 0;
	}

	return chargerid_volt;
}

static void mt_set_chargerid_switch_val(int value)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (is_support_chargerid_check() == false)
		return;
	if (chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("chargerid_switch_gpio not exist, return\n");
		return;
	}
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)
			|| IS_ERR_OR_NULL(chip->normalchg_gpio.charger_gpio_as_output1)
			|| IS_ERR_OR_NULL(chip->normalchg_gpio.charger_gpio_as_output2)) {
		chg_err("pinctrl null, return\n");
		return;
	}

	if (value == 1) {
		set_usbswitch_to_rxtx(chip);
	} else if (value == 0) {
		set_usbswitch_to_dpdm(chip);
	} else {
		//do nothing
	}
	chg_debug("get_val=%d\n", gpio_get_value(chip->normalchg_gpio.chargerid_switch_gpio));

	return;
}

static int mt_get_chargerid_switch_val(void)
{
	int gpio_status = 0;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 0;
	}
	if (is_support_chargerid_check() == false)
		return 0;

	gpio_status = gpio_get_value(chip->normalchg_gpio.chargerid_switch_gpio);

	chg_debug("mt_get_chargerid_switch_val=%d\n", gpio_status);

	return gpio_status;
}

static int oplus_usb_switch_gpio_gpio_init(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return -EINVAL;
	}

	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)) {
		chg_err("get normalchg_gpio.chargerid_switch_gpio pinctrl fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.charger_gpio_as_output1 =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl,
			"charger_gpio_as_output_low");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.charger_gpio_as_output1)) {
		chg_err("get charger_gpio_as_output_low fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.charger_gpio_as_output2 =
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl,
			"charger_gpio_as_output_high");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.charger_gpio_as_output2)) {
		chg_err("get charger_gpio_as_output_high fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chip->normalchg_gpio.pinctrl,
			chip->normalchg_gpio.charger_gpio_as_output1);

	return 0;
}

static int oplus_chg_chargerid_parse_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;	
	struct device_node *node = chip->dev->of_node;
	
	if (chip == NULL || node == NULL) {
		chg_err("oplus_chip or device tree info. missing\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.chargerid_switch_gpio =
			of_get_named_gpio(node, "qcom,chargerid_switch-gpio", 0);
	if (chip->normalchg_gpio.chargerid_switch_gpio <= 0) {
		chg_err("Couldn't read chargerid_switch-gpio rc=%d, chargerid_switch-gpio:%d\n",
				rc, chip->normalchg_gpio.chargerid_switch_gpio);
	} else {
		if (gpio_is_valid(chip->normalchg_gpio.chargerid_switch_gpio)) {
			rc = gpio_request(chip->normalchg_gpio.chargerid_switch_gpio, "charging_switch1-gpio");
			if (rc) {
				chg_err("unable to request chargerid_switch-gpio:%d\n",
						chip->normalchg_gpio.chargerid_switch_gpio);
			} else {
				rc = oplus_usb_switch_gpio_gpio_init();
				if (rc)
					chg_err("unable to init chargerid_switch-gpio:%d\n",
							chip->normalchg_gpio.chargerid_switch_gpio);
			}
		}
		chg_err("chargerid_switch-gpio:%d\n", chip->normalchg_gpio.chargerid_switch_gpio);
	}

	return rc;
}
#endif /*OPLUS_FEATURE_CHG_BASIC*/
//====================================================================//


//====================================================================//
#ifdef OPLUS_FEATURE_CHG_BASIC
static bool oplus_shortc_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return false;
	}

	if (gpio_is_valid(chip->normalchg_gpio.shortc_gpio)) {
		return true;
	}

	return false;
}

static int oplus_shortc_gpio_init(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return -EINVAL;
	}

	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);

	chip->normalchg_gpio.shortc_active =
		pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "shortc_active");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.shortc_active)) {
		chg_err("get shortc_active fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.shortc_active);

	return 0;
}

#ifdef CONFIG_OPLUS_SHORT_HW_CHECK	
static bool oplus_chg_get_shortc_hw_gpio_status(void)
{
	bool shortc_hw_status = 1;
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return shortc_hw_status;
	}

	if (oplus_shortc_check_is_gpio(chip) == true) {	
		shortc_hw_status = !!(gpio_get_value(chip->normalchg_gpio.shortc_gpio));
	}
	return shortc_hw_status;
}
#else /* CONFIG_OPLUS_SHORT_HW_CHECK */
static bool oplus_chg_get_shortc_hw_gpio_status(void)
{
	bool shortc_hw_status = 1;

	return shortc_hw_status;
}
#endif /* CONFIG_OPLUS_SHORT_HW_CHECK */

static int oplus_chg_shortc_hw_parse_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;	
	struct device_node *node = chip->dev->of_node;
	
	if (chip == NULL || node == NULL) {
		chg_err("oplus_chip or device tree info. missing\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.shortc_gpio = of_get_named_gpio(node, "qcom,shortc-gpio", 0);
	if (chip->normalchg_gpio.shortc_gpio <= 0) {
		chg_err("Couldn't read qcom,shortc-gpio rc=%d, qcom,shortc-gpio:%d\n",
				rc, chip->normalchg_gpio.shortc_gpio);
	} else {
		if (oplus_shortc_check_is_gpio(chip) == true) {
			rc = gpio_request(chip->normalchg_gpio.shortc_gpio, "shortc-gpio");
			if (rc) {
				chg_err("unable to request shortc-gpio:%d\n",
						chip->normalchg_gpio.shortc_gpio);
			} else {
				rc = oplus_shortc_gpio_init(chip);
				if (rc)
					chg_err("unable to init shortc-gpio:%d\n", chip->normalchg_gpio.ship_gpio);
			}
		}
		chg_err("shortc-gpio:%d\n", chip->normalchg_gpio.shortc_gpio);
	}

	return rc;
}
#endif /*OPLUS_FEATURE_CHG_BASIC*/
//====================================================================//


//====================================================================//
#ifdef OPLUS_FEATURE_CHG_BASIC
static bool oplus_ship_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return false;
	}

	if (gpio_is_valid(chip->normalchg_gpio.ship_gpio))
		return true;

	return false;
}

static int oplus_ship_gpio_init(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return -EINVAL;
	}

	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);
	chip->normalchg_gpio.ship_active = 
		pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, 
			"ship_active");

	if (IS_ERR_OR_NULL(chip->normalchg_gpio.ship_active)) {
		chg_err("get ship_active fail\n");
		return -EINVAL;
	}
	chip->normalchg_gpio.ship_sleep = 
			pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, 
				"ship_sleep");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.ship_sleep)) {
		chg_err("get ship_sleep fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chip->normalchg_gpio.pinctrl,
		chip->normalchg_gpio.ship_sleep);

	return 0;
}

#define SHIP_MODE_CONFIG		0x40
#define SHIP_MODE_MASK			BIT(0)
#define SHIP_MODE_ENABLE		0
#define PWM_COUNT				5
static void smbchg_enter_shipmode(struct oplus_chg_chip *chip)
{
	int i = 0;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (oplus_ship_check_is_gpio(chip) == true) {
		chg_err("select gpio control\n");
		if (!IS_ERR_OR_NULL(chip->normalchg_gpio.ship_active) && !IS_ERR_OR_NULL(chip->normalchg_gpio.ship_sleep)) {
			pinctrl_select_state(chip->normalchg_gpio.pinctrl,
				chip->normalchg_gpio.ship_sleep);
			for (i = 0; i < PWM_COUNT; i++) {
				//gpio_direction_output(chip->normalchg_gpio.ship_gpio, 1);
				pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.ship_active);
				mdelay(3);
				//gpio_direction_output(chip->normalchg_gpio.ship_gpio, 0);
				pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.ship_sleep);
				mdelay(3);
			}
		}
		chg_err("power off after 15s\n");
	}
}
static void enter_ship_mode_function(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	if (chip->enable_shipmode) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: enter_ship_mode_function\n", __func__);
		smbchg_enter_shipmode(chip);
	}
}

static int oplus_chg_shipmode_parse_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;	
	struct device_node *node = chip->dev->of_node;
	
	if (chip == NULL || node == NULL) {
		chg_err("oplus_chip or device tree info. missing\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.ship_gpio =
			of_get_named_gpio(node, "qcom,ship-gpio", 0);
	if (chip->normalchg_gpio.ship_gpio <= 0) {
		chg_err("Couldn't read qcom,ship-gpio rc = %d, qcom,ship-gpio:%d\n",
				rc, chip->normalchg_gpio.ship_gpio);
	} else {
		if (oplus_ship_check_is_gpio(chip) == true) {
			rc = gpio_request(chip->normalchg_gpio.ship_gpio, "ship-gpio");
			if (rc) {
				chg_err("unable to request ship-gpio:%d\n", chip->normalchg_gpio.ship_gpio);
			} else {
				rc = oplus_ship_gpio_init(chip);
				if (rc)
					chg_err("unable to init ship-gpio:%d\n", chip->normalchg_gpio.ship_gpio);
			}
		}
		chg_err("ship-gpio:%d\n", chip->normalchg_gpio.ship_gpio);
	}

	return rc;
}
#endif /* OPLUS_FEATURE_CHG_BASIC */
//====================================================================//


//====================================================================//
#ifdef OPLUS_FEATURE_CHG_BASIC
static bool oplus_usbtemp_check_is_gpio(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return false;
	}

	if (gpio_is_valid(chip->normalchg_gpio.dischg_gpio))
		return true;

	return false;
}

static bool oplus_usbtemp_check_is_support(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return false;
	}

	if (oplus_usbtemp_check_is_gpio(chip) == true)
		return true;

	chg_err("not support, return false\n");

	return false;
}

static int oplus_dischg_gpio_init(struct oplus_chg_chip *chip)
{
	if (!chip) {
		chg_err("oplus_chip not ready!\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.pinctrl = devm_pinctrl_get(chip->dev);

	if (IS_ERR_OR_NULL(chip->normalchg_gpio.pinctrl)) {
		chg_err("get dischg_pinctrl fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.dischg_enable = pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "dischg_enable");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.dischg_enable)) {
		chg_err("get dischg_enable fail\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.dischg_disable = pinctrl_lookup_state(chip->normalchg_gpio.pinctrl, "dischg_disable");
	if (IS_ERR_OR_NULL(chip->normalchg_gpio.dischg_disable)) {
		chg_err("get dischg_disable fail\n");
		return -EINVAL;
	}

	pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.dischg_disable);

	return 0;
}

#define USB_20C		20//degreeC
#define USB_40C		40
#define USB_50C		50
#define USB_57C		57
#define USB_100C	100
#define USB_25C_VOLT	1192//900//mv
#define USB_50C_VOLT	450
#define USB_55C_VOLT	448
#define USB_60C_VOLT	327
#define VBUS_VOLT_THRESHOLD	3000//3V
#define RETRY_CNT_DELAY		5 //ms
#define MIN_MONITOR_INTERVAL	50//50ms
#define MAX_MONITOR_INTERVAL	200//200ms
#define VBUS_MONITOR_INTERVAL	3000//3s

static void oplus_get_usbtemp_volt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	int usbtemp_volt = 0;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	usbtemp_volt = mt_vadc_read(1, Channel_15);//get the typec tem adc volt
	if (usbtemp_volt <= 0) {
		usbtemp_volt = USB_25C_VOLT;
	}
	chip->usbtemp_volt_r = usbtemp_volt;

	usbtemp_volt = mt_vadc_read(1, Channel_12);//get the typec tem adc2 volt
	if (usbtemp_volt <= 0) {
		usbtemp_volt = USB_25C_VOLT;
	}
	chip->usbtemp_volt_l = usbtemp_volt;

	//chg_err("usbtemp_volt: %d, %d\n", chip->usbtemp_volt_r, chip->usbtemp_volt_l);

	return;
}

static void get_usb_temp(struct oplus_chg_chip *chip)
{
	int i = 0;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return;
	}

	for (i = ARRAY_SIZE(con_volt_18097) - 1; i >= 0; i--) {
		if (con_volt_18097[i] >= chip->usbtemp_volt_r)
			break;
		else if (i == 0)
			break;
	}
	chip->usb_temp_r = con_temp_18097[i];

	for (i = ARRAY_SIZE(con_volt_18097) - 1; i >= 0; i--) {
		if (con_volt_18097[i] >= chip->usbtemp_volt_l)
			break;
		else if (i == 0)
			break;
	}
	chip->usb_temp_l = con_temp_18097[i];

	//chg_err("usbtemp: %d, %d\n", chip->usb_temp_r, chip->usb_temp_l);

	return;
}

static bool oplus_chg_get_vbus_status(struct oplus_chg_chip *chip)
{
	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return false;
	}

	return chip->charger_exist;
}

static int oplus_usbtemp_dischg_action(struct oplus_chg_chip *chip)
{

#ifndef CONFIG_HIGH_TEMP_VERSION
	oplus_set_usb_status(USB_TEMP_HIGH);
	if (oplus_vooc_get_fastchg_started() == true) {
		oplus_chg_set_chargerid_switch_val(0);
		oplus_vooc_switch_mode(NORMAL_CHARGER_MODE);
		oplus_vooc_reset_mcu();
		//msleep(20);//wait for turn-off fastchg MOS
	}
	usleep_range(10000,10000);///msleep(10);
	chip->chg_ops->charger_suspend();
	usleep_range(10000,10000);

	chip->chg_ops->otg_disable();
	oplus_set_otg_switch_status(false);
	usleep_range(12000,12000);
#endif

#ifdef CONFIG_HIGH_TEMP_VERSION
	chg_err(" CONFIG_HIGH_TEMP_VERSION enable here,do not set vbus down \n");
	pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.dischg_disable);
#else
	pinctrl_select_state(chip->normalchg_gpio.pinctrl, chip->normalchg_gpio.dischg_enable);
#endif

	return 0;
}

#define RETRY_COUNT		3
static int oplus_usbtemp_monitor_main(void *data)
{
	int i = 0;
	int delay = 0;
	int vbus_volt = 0;
	int count_r = 0;
	int count_l = 0;
	int time_count = 0;
	static bool dischg_flag = false;
	static int count = 0;
	static int last_usb_temp_r = 25;
	static int last_usb_temp_l = 25;
	static bool init_flag = true;
	struct oplus_chg_chip *chip = g_oplus_chip;

	while (!kthread_should_stop() && dischg_flag == false) {
		if (oplus_get_chg_powersave() == true) {
			delay = 1000 * 30;//30S
			goto check_again;
		}
		oplus_get_usbtemp_volt(chip);
		get_usb_temp(chip);

		if (init_flag == true) {
			init_flag = false;
			if (oplus_chg_get_vbus_status(chip) == false
					&& oplus_chg_get_otg_online() == false) {
				delay = MIN_MONITOR_INTERVAL * 20;
				goto check_again;
			}
		}
		if (chip->usb_temp_r < USB_50C && chip->usb_temp_l < USB_50C)//get vbus when usbtemp < 50C
			vbus_volt = battery_meter_get_charger_voltage();
		else
			vbus_volt = 0;

		if (chip->usb_temp_r < USB_40C && chip->usb_temp_l < USB_40C) {
			delay = MAX_MONITOR_INTERVAL;
			time_count = 8;
		} else {
			delay = MIN_MONITOR_INTERVAL;
			time_count = 30;
		}

		if (chip->usb_temp_r < USB_50C && chip->usb_temp_l < USB_50C && vbus_volt < VBUS_VOLT_THRESHOLD)
			delay = VBUS_MONITOR_INTERVAL;

		if ((chip->usb_temp_r >= USB_57C && chip->usb_temp_r < USB_100C)
				|| (chip->usb_temp_l >= USB_57C && chip->usb_temp_l < USB_100C)) {
			for (i = 0; i < RETRY_COUNT; i++) {
				mdelay(RETRY_CNT_DELAY);
				oplus_get_usbtemp_volt(chip);
				get_usb_temp(chip);
				if (chip->usb_temp_r >= USB_57C)
					count_r++;
				if (chip->usb_temp_l >= USB_57C)
					count_l++;
			}

			if (count_r >= RETRY_COUNT || count_l >= RETRY_COUNT) {
				if (!IS_ERR_OR_NULL(chip->normalchg_gpio.dischg_enable)) {
					dischg_flag = true;
					chg_err("dischg enable...usb_temp[%d,%d], usb_volt[%d,%d]\n",
							chip->usb_temp_r, chip->usb_temp_l, chip->usbtemp_volt_r, chip->usbtemp_volt_l);
					oplus_usbtemp_dischg_action(chip);
				}
			}

			count_r = 0;
			count_l = 0;
			count = 0;
		} else if (((chip->usb_temp_r * 10 - chip->temperature) > chip->limits.usb_high_than_bat_decidegc)
				|| ((chip->usb_temp_l * 10 - chip->temperature) > chip->limits.usb_high_than_bat_decidegc)) {
			if (count <= time_count) {
				if (count == 0) {
					last_usb_temp_r = chip->usb_temp_r;
					last_usb_temp_l = chip->usb_temp_l;
				}

				if (((chip->usb_temp_r - last_usb_temp_r) >= 3 && chip->usb_temp_r >= USB_20C && chip->usb_temp_r < USB_100C)
						|| ((chip->usb_temp_l - last_usb_temp_l) >= 3 && chip->usb_temp_l >= USB_20C && chip->usb_temp_l < USB_100C)) {
					for (i = 0; i < RETRY_COUNT; i++) {
						mdelay(RETRY_CNT_DELAY);
						oplus_get_usbtemp_volt(chip);
						get_usb_temp(chip);
						if ((chip->usb_temp_r - last_usb_temp_r) >= 3)
							count_r++;
						if ((chip->usb_temp_l - last_usb_temp_l) >= 3)
							count_l++;
					}

					if (count_r >= RETRY_COUNT || count_l >= RETRY_COUNT) {
						if (!IS_ERR_OR_NULL(chip->normalchg_gpio.dischg_enable)) {
							dischg_flag = true;
							chg_err("dischg enable...current_usb_temp[%d,%d], last_usb_temp[%d,%d], count[%d]\n",
									chip->usb_temp_r, chip->usb_temp_l, last_usb_temp_r, last_usb_temp_l, count);
							oplus_usbtemp_dischg_action(chip);
						}
					}
					count_r = 0;
					count_l = 0;
				}

				count++;
				if (count > time_count) {
					count = 0;
				}
			}
			msleep(delay);
		} else {
check_again:
			count = 0;
			last_usb_temp_r = chip->usb_temp_r;
			last_usb_temp_l = chip->usb_temp_l;
			msleep(delay);
			wait_event_interruptible(oplus_usbtemp_wq,
				oplus_chg_get_vbus_status(chip) == true || oplus_chg_get_otg_online() == true);
		}
	}

	return 0;
}

static void oplus_usbtemp_thread_init(void)
{
	oplus_usbtemp_kthread =
			kthread_run(oplus_usbtemp_monitor_main, 0, "usbtemp_kthread");
	if (IS_ERR(oplus_usbtemp_kthread)) {
		chg_err("failed to cread oplus_usbtemp_kthread\n");
	}
}

void oplus_wake_up_usbtemp_thread(void)
{
	if (oplus_usbtemp_check_is_support() == true) {
		wake_up_interruptible(&oplus_usbtemp_wq);
	}
}

static int oplus_chg_usbtemp_parse_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;
	struct device_node *node = NULL;

	if (chip)
		node = chip->dev->of_node;
	if (node == NULL) {
		chg_err("oplus_chip or device tree info. missing\n");
		return -EINVAL;
	}

	chip->normalchg_gpio.dischg_gpio = of_get_named_gpio(node, "qcom,dischg-gpio", 0);
	if (chip->normalchg_gpio.dischg_gpio <= 0) {
		chg_err("Couldn't read qcom,dischg-gpio rc=%d, qcom,dischg-gpio:%d\n",
				rc, chip->normalchg_gpio.dischg_gpio);
	} else {
		if (oplus_usbtemp_check_is_support() == true) {
			rc = gpio_request(chip->normalchg_gpio.dischg_gpio, "dischg-gpio");
			if (rc) {
				chg_err("unable to request dischg-gpio:%d\n",
						chip->normalchg_gpio.dischg_gpio);
			} else {
				rc = oplus_dischg_gpio_init(chip);
				if (rc)
					chg_err("unable to init dischg-gpio:%d\n",
							chip->normalchg_gpio.dischg_gpio);
			}
		}
		chg_err("dischg-gpio:%d\n", chip->normalchg_gpio.dischg_gpio);
	}

	return rc;
}
#endif /* OPLUS_FEATURE_CHG_BASIC */
//====================================================================//


//====================================================================//
static int oplus_chg_parse_custom_dt(struct oplus_chg_chip *chip)
{
	int rc = 0;	
	
	if (chip == NULL) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return -EINVAL;
	}

	rc = oplus_chg_chargerid_parse_dt(chip);
	if (rc) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chg_chargerid_parse_dt fail!\n", __func__);
		return -EINVAL;
	}

	rc = oplus_chg_shipmode_parse_dt(chip);
	if (rc) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chg_shipmode_parse_dt fail!\n", __func__);
		return -EINVAL;
	}

	rc = oplus_chg_shortc_hw_parse_dt(chip);
	if (rc) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chg_shortc_hw_parse_dt fail!\n", __func__);
		return -EINVAL;
	}

	rc = oplus_chg_usbtemp_parse_dt(chip);
	if (rc) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chg_usbtemp_parse_dt fail!\n", __func__);
		return -EINVAL;
	}

	return rc;
}
//====================================================================//


//====================================================================//
#ifdef OPLUS_FEATURE_CHG_BASIC
/************************************************/
/* Power Supply Functions
*************************************************/
static int mt_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int rc = 0;

	rc = oplus_ac_get_property(psy, psp, val);
	return rc;
}

static int mt_usb_prop_is_writeable(struct power_supply *psy,
	enum power_supply_property psp)
{
	int rc = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_WATER_DETECT_FEATURE:
			rc = 1;
			break;
		default:
			rc = oplus_usb_property_is_writeable(psy, psp);
	}
	return rc;
}

static int mt_usb_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int rc = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION:
			if (pinfo != NULL && pinfo->tcpc != NULL) {
				if (tcpm_inquire_typec_attach_state(pinfo->tcpc) != TYPEC_UNATTACHED) {
					val->intval = (int)tcpm_inquire_cc_polarity(pinfo->tcpc) + 1;
				} else {
					val->intval = 0;
				}
				if (val->intval != 0)
					printk(KERN_ERR "[OPLUS_CHG][%s]: cc[%d]\n", __func__, val->intval);
			} else {
				val->intval = 0;
			}
			break;
		case POWER_SUPPLY_PROP_TYPEC_SBU_VOLTAGE:
			val->intval = oplus_get_typec_sbu_voltage();
			break;
		case POWER_SUPPLY_PROP_WATER_DETECT_FEATURE:
			val->intval = oplus_get_water_detect();
			break;
		case POWER_SUPPLY_PROP_USB_STATUS:
			val->intval = oplus_get_usb_status();
			break;
		case POWER_SUPPLY_PROP_USBTEMP_VOLT_L:
			if (g_oplus_chip)
				val->intval = g_oplus_chip->usbtemp_volt_l;
			else
				val->intval = -ENODATA;
			break;
		case POWER_SUPPLY_PROP_USBTEMP_VOLT_R:
			if (g_oplus_chip)
				val->intval = g_oplus_chip->usbtemp_volt_r;
			else
				val->intval = -ENODATA;
			break;

		default:
			rc = oplus_usb_get_property(psy, psp, val);
	}
	return rc;
}

static int mt_usb_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	int rc = 0;

	switch (psp) {
		case POWER_SUPPLY_PROP_WATER_DETECT_FEATURE:
			printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_set_water_detect[%d]\n", __func__, val->intval);
			if (val->intval == 0) {
				oplus_set_water_detect(false);
			} else {
				oplus_set_water_detect(true);
			}
			break;
		default:
			rc = oplus_usb_set_property(psy, psp, val);
	}
	return rc;
}

static int battery_prop_is_writeable(struct power_supply *psy,
	enum power_supply_property psp)
{
	int rc = 0;

	rc = oplus_battery_property_is_writeable(psy, psp);
	return rc;
}

static int battery_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	int rc = 0;

	rc = oplus_battery_set_property(psy, psp, val);
	return rc;
}

static int battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int rc = 0;

	rc = oplus_battery_get_property(psy, psp, val);
	return 0;
}

static enum power_supply_property mt_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property mt_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_OTG_SWITCH,
	POWER_SUPPLY_PROP_OTG_ONLINE,
	POWER_SUPPLY_PROP_TYPEC_CC_ORIENTATION,
	POWER_SUPPLY_PROP_TYPEC_SBU_VOLTAGE,
	POWER_SUPPLY_PROP_WATER_DETECT_FEATURE,
	POWER_SUPPLY_PROP_USB_STATUS,
	POWER_SUPPLY_PROP_USBTEMP_VOLT_L,
	POWER_SUPPLY_PROP_USBTEMP_VOLT_R,
};

static enum power_supply_property battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_AUTHENTICATE,
	POWER_SUPPLY_PROP_CHARGE_TIMEOUT,
	POWER_SUPPLY_PROP_CHARGE_TECHNOLOGY,
	POWER_SUPPLY_PROP_FAST_CHARGE,
	POWER_SUPPLY_PROP_MMI_CHARGING_ENABLE,        /*add for MMI_CHG_TEST*/
#ifdef CONFIG_OPLUS_CHARGER_MTK
	POWER_SUPPLY_PROP_STOP_CHARGING_ENABLE,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CURRENT_MAX,
#endif
	POWER_SUPPLY_PROP_BATTERY_FCC,
	POWER_SUPPLY_PROP_BATTERY_SOH,
	POWER_SUPPLY_PROP_BATTERY_CC,
	POWER_SUPPLY_PROP_BATTERY_RM,
	POWER_SUPPLY_PROP_BATTERY_NOTIFY_CODE,
#ifdef CONFIG_OPLUS_SMART_CHARGER_SUPPORT
	POWER_SUPPLY_PROP_COOL_DOWN,
#endif
	POWER_SUPPLY_PROP_ADAPTER_FW_UPDATE,
	POWER_SUPPLY_PROP_VOOCCHG_ING,
#ifdef CONFIG_OPLUS_CHECK_CHARGERID_VOLT
	POWER_SUPPLY_PROP_CHARGERID_VOLT,
#endif
#ifdef CONFIG_OPLUS_SHIP_MODE_SUPPORT
	POWER_SUPPLY_PROP_SHIP_MODE,
#endif
#ifdef CONFIG_OPLUS_CALL_MODE_SUPPORT
	POWER_SUPPLY_PROP_CALL_MODE,
#endif
#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
#ifdef CONFIG_OPLUS_SHORT_USERSPACE
	POWER_SUPPLY_PROP_SHORT_C_LIMIT_CHG,
	POWER_SUPPLY_PROP_SHORT_C_LIMIT_RECHG,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
#else
	POWER_SUPPLY_PROP_SHORT_C_BATT_UPDATE_CHANGE,
	POWER_SUPPLY_PROP_SHORT_C_BATT_IN_IDLE,
	POWER_SUPPLY_PROP_SHORT_C_BATT_CV_STATUS,
#endif /*CONFIG_OPLUS_SHORT_USERSPACE*/
#endif
#ifdef CONFIG_OPLUS_SHORT_HW_CHECK
	POWER_SUPPLY_PROP_SHORT_C_HW_FEATURE,
	POWER_SUPPLY_PROP_SHORT_C_HW_STATUS,
#endif
#ifdef CONFIG_OPLUS_CHIP_SOC_NODE
	POWER_SUPPLY_PROP_CHIP_SOC,
	POWER_SUPPLY_PROP_SMOOTH_SOC,
#endif
#ifdef CONFIG_OPLUS_SHORT_IC_CHECK
	POWER_SUPPLY_PROP_SHORT_C_IC_OTP_STATUS,
	POWER_SUPPLY_PROP_SHORT_C_IC_VOLT_THRESH,
	POWER_SUPPLY_PROP_SHORT_C_IC_OTP_VALUE,
#endif
#ifdef CONFIG_OPLUS_SMART_CHARGER_SUPPORT
	POWER_SUPPLY_PROP_COOL_DOWN,
#endif
};

static int oplus_power_supply_init(struct oplus_chg_chip *chip)
{
	int ret = 0;
	struct oplus_chg_chip *mt_chg = NULL;

	if (chip == NULL) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return -EINVAL;
	}
	mt_chg = chip;

	mt_chg->ac_psd.name = "ac";
	mt_chg->ac_psd.type = POWER_SUPPLY_TYPE_MAINS;
	mt_chg->ac_psd.properties = mt_ac_properties;
	mt_chg->ac_psd.num_properties = ARRAY_SIZE(mt_ac_properties);
	mt_chg->ac_psd.get_property = mt_ac_get_property;
	mt_chg->ac_cfg.drv_data = mt_chg;

	mt_chg->usb_psd.name = "usb";
	mt_chg->usb_psd.type = POWER_SUPPLY_TYPE_USB;
	mt_chg->usb_psd.properties = mt_usb_properties;
	mt_chg->usb_psd.num_properties = ARRAY_SIZE(mt_usb_properties);
	mt_chg->usb_psd.get_property = mt_usb_get_property;
	mt_chg->usb_psd.set_property = mt_usb_set_property;
	mt_chg->usb_psd.property_is_writeable = mt_usb_prop_is_writeable;
	mt_chg->usb_cfg.drv_data = mt_chg;
    
	mt_chg->battery_psd.name = "battery";
	mt_chg->battery_psd.type = POWER_SUPPLY_TYPE_BATTERY;
	mt_chg->battery_psd.properties = battery_properties;
	mt_chg->battery_psd.num_properties = ARRAY_SIZE(battery_properties);
	mt_chg->battery_psd.get_property = battery_get_property;
	mt_chg->battery_psd.set_property = battery_set_property;
	mt_chg->battery_psd.property_is_writeable = battery_prop_is_writeable;

	mt_chg->ac_psy = power_supply_register(mt_chg->dev, &mt_chg->ac_psd,
			&mt_chg->ac_cfg);
	if (IS_ERR(mt_chg->ac_psy)) {
		dev_err(mt_chg->dev, "Failed to register power supply ac: %ld\n",
			PTR_ERR(mt_chg->ac_psy));
		ret = PTR_ERR(mt_chg->ac_psy);
		goto err_ac_psy;
	}

	mt_chg->usb_psy = power_supply_register(mt_chg->dev, &mt_chg->usb_psd,
			&mt_chg->usb_cfg);
	if (IS_ERR(mt_chg->usb_psy)) {
		dev_err(mt_chg->dev, "Failed to register power supply usb: %ld\n",
			PTR_ERR(mt_chg->usb_psy));
		ret = PTR_ERR(mt_chg->usb_psy);
		goto err_usb_psy;
	}

	mt_chg->batt_psy = power_supply_register(mt_chg->dev, &mt_chg->battery_psd,
			NULL);
	if (IS_ERR(mt_chg->batt_psy)) {
		dev_err(mt_chg->dev, "Failed to register power supply battery: %ld\n",
			PTR_ERR(mt_chg->batt_psy));
		ret = PTR_ERR(mt_chg->batt_psy);
		goto err_battery_psy;
	}

	chg_err("%s OK\n", __func__);
	return 0;

err_battery_psy:
	power_supply_unregister(mt_chg->usb_psy);
err_usb_psy:
	power_supply_unregister(mt_chg->ac_psy);
err_ac_psy:

	return ret;
}
#endif /* OPLUS_FEATURE_CHG_BASIC */
//====================================================================//


//====================================================================//
#ifdef OPLUS_FEATURE_CHG_BASIC
void oplus_set_otg_switch_status(bool value)
{
	if (pinfo != NULL && pinfo->tcpc != NULL) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: otg switch[%d]\n", __func__, value);
		tcpm_typec_change_role(pinfo->tcpc, value ? TYPEC_ROLE_DRP : TYPEC_ROLE_SNK);
	}
}
EXPORT_SYMBOL(oplus_set_otg_switch_status);
#endif /* OPLUS_FEATURE_CHG_BASIC */
//====================================================================//


//====================================================================//
#ifdef OPLUS_FEATURE_CHG_BASIC
int oplus_chg_get_mmi_status(void)
{
	struct oplus_chg_chip *chip = g_oplus_chip;

	if (!chip) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: oplus_chip not ready!\n", __func__);
		return 1;
	}
	if (chip->mmi_chg == 0)
		printk(KERN_ERR "[OPLUS_CHG][%s]: mmi_chg[%d]\n", __func__, chip->mmi_chg);
	return chip->mmi_chg;
}
EXPORT_SYMBOL(oplus_chg_get_mmi_status);
#endif /* OPLUS_FEATURE_CHG_BASIC */
//====================================================================//


//====================================================================//
#ifdef OPLUS_FEATURE_CHG_BASIC
static bool oplus_mt6360_get_pd_type(void)
{
	if (pinfo != NULL) {
		return mtk_pdc_check_charger(pinfo);
	}
	return false;
}

static int oplus_mt6360_pd_setup(void)
{
	int vbus_mv = 5000;
	int ibus_ma = 0;
	int ret = 0;

	if(!g_oplus_chip) {
		printk(KERN_ERR "%s:g_oplus_chip not ready\n", __func__);
		return -1;
	}

	ibus_ma = g_oplus_chip->limits.pd_input_current_charger_ma;
	ret = oplus_pdc_setup(vbus_mv, ibus_ma);
	printk(KERN_ERR "%s: vbus[%d], ibus[%d]\n", __func__, vbus_mv, ibus_ma);
	return ret;
}
#endif /* OPLUS_FEATURE_CHG_BASIC */
//====================================================================//


//====================================================================//
struct oplus_chg_operations  mtk6360_chg_ops = {
	.dump_registers = oplus_mt6360_dump_registers,
	.kick_wdt = oplus_mt6360_kick_wdt,
	.hardware_init = oplus_mt6360_hardware_init,
	.charging_current_write_fast = oplus_mt6360_charging_current_write_fast,
	.set_aicl_point = oplus_mt6360_set_aicl_point,
	.input_current_write = oplus_mt6360_input_current_limit_write,
	.float_voltage_write = oplus_mt6360_float_voltage_write,
	.term_current_set = oplus_mt6360_set_termchg_current,
	.charging_enable = oplus_mt6360_enable_charging,
	.charging_disable = oplus_mt6360_disable_charging,
	.get_charging_enable = oplus_mt6360_check_charging_enable,
	.charger_suspend = oplus_mt6360_suspend_charger,
	.charger_unsuspend = oplus_mt6360_unsuspend_charger,
	.set_rechg_vol = oplus_mt6360_set_rechg_voltage,
	.reset_charger = oplus_mt6360_reset_charger,
	.read_full = oplus_mt6360_registers_read_full,
	.otg_enable = oplus_mt6360_otg_enable,
	.otg_disable = oplus_mt6360_otg_disable,
	.set_charging_term_disable = oplus_mt6360_set_chging_term_disable,
	.check_charger_resume = oplus_mt6360_check_charger_resume,
	.get_chg_current_step = oplus_mt6360_get_chg_current_step,
#ifdef CONFIG_OPLUS_CHARGER_MTK
	.get_charger_type = mt_power_supply_type_check,
	.get_charger_volt = battery_meter_get_charger_voltage,
	.check_chrdet_status = oplus_mt6360_get_vbus_status,
	.get_instant_vbatt = oplus_battery_meter_get_battery_voltage,
	.get_boot_mode = (int (*)(void))get_boot_mode,
	.get_boot_reason = (int (*)(void))get_boot_reason,
	.get_chargerid_volt = mt_get_chargerid_volt,
	.set_chargerid_switch_val = mt_set_chargerid_switch_val ,
	.get_chargerid_switch_val  = mt_get_chargerid_switch_val,
	.get_rtc_soc = get_rtc_spare_oplus_fg_value,
	.set_rtc_soc = set_rtc_spare_oplus_fg_value,
	.set_power_off = oplus_mt_power_off,
	//.usb_connect = mt_usb_connect,
	//.usb_disconnect = mt_usb_disconnect,
#else /* CONFIG_OPLUS_CHARGER_MTK */
	.get_charger_type = qpnp_charger_type_get,
	.get_charger_volt = qpnp_get_prop_charger_voltage_now,
	.check_chrdet_status = qpnp_lbc_is_usb_chg_plugged_in,
	.get_instant_vbatt = qpnp_get_prop_battery_voltage_now,
	.get_boot_mode = get_boot_mode,
	.get_rtc_soc = qpnp_get_pmic_soc_memory,
	.set_rtc_soc = qpnp_set_pmic_soc_memory,
#endif /* CONFIG_OPLUS_CHARGER_MTK */

#ifdef CONFIG_OPLUS_SHORT_C_BATT_CHECK
	.get_dyna_aicl_result = oplus_mt6360_chg_get_dyna_aicl_result,
#endif
	.get_shortc_hw_gpio_status = oplus_chg_get_shortc_hw_gpio_status,
#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
	.check_rtc_reset = rtc_reset_check,
#endif
	.oplus_chg_get_pd_type = oplus_mt6360_get_pd_type,
	.oplus_chg_pd_setup = oplus_mt6360_pd_setup,
};
//====================================================================//
#endif /* OPLUS_FEATURE_CHG_BASIC */


static int mtk_charger_probe(struct platform_device *pdev)
{
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct oplus_chg_chip *oplus_chip;
#endif
	struct charger_manager *info = NULL;
	struct list_head *pos;
	struct list_head *phead = &consumer_head;
	struct charger_consumer *ptr;
	int ret;

	chr_err("%s: starts\n", __func__);

#ifdef OPLUS_FEATURE_CHG_BASIC
	oplus_chip = devm_kzalloc(&pdev->dev, sizeof(*oplus_chip), GFP_KERNEL);
	if (!oplus_chip)
		return -ENOMEM;

	oplus_chip->dev = &pdev->dev;
	oplus_chg_parse_svooc_dt(oplus_chip);
	if (oplus_chip->vbatt_num == 1) {
		if (oplus_gauge_check_chip_is_null()) {
			chg_err("[oplus_chg_init] gauge null, will do after bettery init.\n");
			return -EPROBE_DEFER;
		}
		oplus_chip->chg_ops = &mtk6360_chg_ops;
	} else {
		if (oplus_gauge_check_chip_is_null() || oplus_vooc_check_chip_is_null()
				|| oplus_adapter_check_chip_is_null()) {
			chg_err("[oplus_chg_init] gauge || vooc || adapter null, will do after bettery init.\n");
			return -EPROBE_DEFER;
		}
		//oplus_chip->chg_ops = (oplus_get_chg_ops());
	}
#endif /* OPLUS_FEATURE_CHG_BASIC */

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	pinfo = info;

	platform_set_drvdata(pdev, info);
	info->pdev = pdev;

#ifdef OPLUS_FEATURE_CHG_BASIC
	oplus_chip->chgic_mtk.oplus_info = info;

	info->chg1_dev = get_charger_by_name("primary_chg");
	if (info->chg1_dev) {
		chg_err("found primary charger [%s]\n",
			info->chg1_dev->props.alias_name);
	} else {
		chg_err("can't find primary charger!\n");
	}
#endif

	mtk_charger_parse_dt(info, &pdev->dev);

	mutex_init(&info->charger_lock);
	mutex_init(&info->charger_pd_lock);

#ifdef OPLUS_FEATURE_CHG_BASIC
	g_oplus_chip = oplus_chip;
	oplus_power_supply_init(oplus_chip);
	oplus_chg_parse_custom_dt(oplus_chip);
	oplus_chg_parse_charger_dt(oplus_chip);
	oplus_chip->chg_ops->hardware_init();
	oplus_chip->authenticate = oplus_gauge_get_batt_authenticate();
	oplus_chg_init(oplus_chip);
	oplus_chg_wake_update_work();
	oplus_tbatt_power_off_task_init(oplus_chip);
#endif
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (oplus_usbtemp_check_is_support() == true)
		oplus_usbtemp_thread_init();
#endif

	srcu_init_notifier_head(&info->evt_nh);
	ret = mtk_charger_setup_files(pdev);
	if (ret)
		chr_err("Error creating sysfs interface\n");

	pinfo->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (pinfo->tcpc != NULL) {
		pinfo->pd_nb.notifier_call = pd_tcp_notifier_call;
		ret = register_tcp_dev_notifier(pinfo->tcpc, &pinfo->pd_nb,
				TCP_NOTIFY_TYPE_USB | TCP_NOTIFY_TYPE_MISC);
	} else {
		chg_err("get PD dev fail\n");
	}

	mtk_pdc_init(info);

	return 0;
}

static int mtk_charger_remove(struct platform_device *dev)
{
	return 0;
}

static void mtk_charger_shutdown(struct platform_device *dev)
{
	struct charger_manager *info = platform_get_drvdata(dev);
	int ret;

	if (mtk_pe20_get_is_connect(info) || mtk_pe_get_is_connect(info)) {
		if (info->chg2_dev)
			charger_dev_enable(info->chg2_dev, false);
		ret = mtk_pe20_reset_ta_vchr(info);
		if (ret == -ENOTSUPP)
			mtk_pe_reset_ta_vchr(info);
		pr_debug("%s: reset TA before shutdown\n", __func__);
	}
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (g_oplus_chip) {
		enter_ship_mode_function(g_oplus_chip);
	}
#endif
}

static const struct of_device_id mtk_charger_of_match[] = {
	{.compatible = "mediatek,charger",},
	{},
};

MODULE_DEVICE_TABLE(of, mtk_charger_of_match);

struct platform_device charger_device = {
	.name = "charger",
	.id = -1,
};

static struct platform_driver charger_driver = {
	.probe = mtk_charger_probe,
	.remove = mtk_charger_remove,
	.shutdown = mtk_charger_shutdown,
	.driver = {
		   .name = "charger",
		   .of_match_table = mtk_charger_of_match,
	},
};

static int __init mtk_charger_init(void)
{
	return platform_driver_register(&charger_driver);
}
late_initcall(mtk_charger_init);

static void __exit mtk_charger_exit(void)
{
	platform_driver_unregister(&charger_driver);
}
module_exit(mtk_charger_exit);


MODULE_AUTHOR("sjc");
MODULE_DESCRIPTION("OPLUS Charger Driver");
MODULE_LICENSE("GPL");
