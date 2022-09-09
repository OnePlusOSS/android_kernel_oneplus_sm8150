/************************************************************************************
** VENDOR_EDIT
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
#ifndef __OPLUS_BATTERY_MTK6853_H__
#define __OPLUS_BATTERY_MTK6853_H__

#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/wait.h>
#include <linux/alarmtimer.h>
#include <mt-plat/charger_type.h>
#include <mt-plat/mtk_charger.h>
#include <mt-plat/mtk_battery.h>
#include <uapi/linux/sched/types.h>
#include <linux/uaccess.h>

#include <mt-plat/charger_class.h>
#include "../../../../kernel-4.14/drivers/misc/mediatek/typec/tcpc/inc/tcpm.h"
#include "../../../../kernel-4.14/drivers/misc/mediatek/typec/tcpc/inc/mtk_direct_charge_vdm.h"
struct charger_manager;
#include "../../../../kernel-4.14/drivers/power/supply/mediatek/charger/mtk_pe_intf.h"
#include "../../../../kernel-4.14/drivers/power/supply/mediatek/charger/mtk_pe20_intf.h"
#include "../../../../kernel-4.14/drivers/power/supply/mediatek/charger/mtk_pdc_intf.h"
#include "../../../../kernel-4.14/drivers/power/supply/mediatek/charger/mtk_charger_init.h"
#include "../../../../kernel-4.14/drivers/power/supply/mediatek/charger/mtk_charger_intf.h"

typedef enum {
	STEP_CHG_STATUS_STEP1 = 0,	/*16C~44C*/
	STEP_CHG_STATUS_STEP2,
	STEP_CHG_STATUS_STEP3,
	STEP_CHG_STATUS_STEP4,
	STEP_CHG_STATUS_INVALID
} OPLUS_STEP_CHG_STATUS;

#ifdef VENDOR_EDIT

struct mtk_pmic {
	struct charger_manager* oplus_info;
};

//extern int mt_power_supply_type_check(void);
extern enum charger_type mt_get_charger_type(void);
//int battery_meter_get_charger_voltage(void);
extern int mt6360_get_vbus_rising(void);
extern int mt6360_check_charging_enable(void);
extern int mt6360_suspend_charger(bool suspend);
extern int mt6360_set_rechg_voltage(int rechg_mv);
extern int mt6360_reset_charger(void);
extern int mt6360_set_chging_term_disable(bool disable);
extern int mt6360_aicl_enable(bool enable);
extern int mt6360_set_register(u8 addr, u8 mask, u8 data);
extern int mt6360_enter_shipmode(void);

extern void mt_power_off(void);
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);

bool oplus_pmic_check_chip_is_null(void);
extern int oplus_get_typec_sbu_voltage(void);
extern void oplus_set_water_detect(bool enable);
extern int oplus_get_water_detect(void);
extern enum power_supply_type mt6360_get_hvdcp_type(void);
void mt6360_enable_hvdcp_detect(void);
extern void oplus_notify_hvdcp_detect_stat(void);
void oplus_gauge_set_event(int event);
extern bool is_mtksvooc_project;
#endif /* VENDOR_EDIT */
#endif /* __OPLUS_BATTERY_MTK6885_H__ */
