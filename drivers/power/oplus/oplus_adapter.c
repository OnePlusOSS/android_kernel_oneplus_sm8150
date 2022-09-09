/**********************************************************************************
* Copyright (c)  2008-2015  Guangdong OPLUS Mobile Comm Corp., Ltd
* OPLUS_FEATURE_CHG_BASIC
* Description: Charger IC management module for charger system framework.
*                          Manage all charger IC and define abstarct function flow.
**
** Version: 1.0
** Date created: 21:03:46, 05/04/2012
**
** --------------------------- Revision History: ------------------------------------------------------------
* <version>           <date>                <author>                            <desc>
************************************************************************************************************/

#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include "oplus_charger.h"
#include "oplus_vooc.h"
#include "oplus_gauge.h"
#include "oplus_adapter.h"



extern int enable_charger_log;
#define adapter_xlog_printk(num, fmt, ...) \
        do { \
                if (enable_charger_log >= (int)num) { \
                        printk(KERN_NOTICE pr_fmt("[OPLUS_CHG][%s]"fmt), __func__, ##__VA_ARGS__);\
        } \
} while (0)


static struct oplus_adapter_chip *g_adapter_chip = NULL;




static void oplus_adpater_awake_init(struct oplus_adapter_chip *chip)
{
	if (!chip)
		return;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	wake_lock_init(&chip->adapter_wake_lock, WAKE_LOCK_SUSPEND, "adpater_wake_lock");
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 170))
	chip->adapter_ws = wakeup_source_register("adpater_wake_lock");
#else
	chip->adapter_ws = wakeup_source_register(NULL, "adpater_wake_lock");
#endif
}

static void oplus_adapter_set_awake(struct oplus_adapter_chip *chip, bool awake)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0))
	if (awake)
		wake_lock(&chip->adapter_wake_lock);
	else
		wake_unlock(&chip->adapter_wake_lock);
#else
	static bool pm_flag = false;

	if (!chip || !chip->adapter_ws)
		return;

	if (awake && !pm_flag) {
		pm_flag = true;
		__pm_stay_awake(chip->adapter_ws);
	} else if (!awake && pm_flag) {
		__pm_relax(chip->adapter_ws);
		pm_flag = false;
	}
#endif
}


static void adapter_update_work_func(struct work_struct *work)
{
        struct delayed_work *dwork = to_delayed_work(work);
        struct oplus_adapter_chip *chip = container_of(dwork, struct oplus_adapter_chip, adapter_update_work);
        bool update_result = false;
		long tx_gpio = 0, rx_gpio = 0;
        int i = 0;

        if (!chip) {
                chg_err("oplus_adapter_chip NULL\n");
                return;
        }		
        oplus_adapter_set_awake(chip, true);
		
		tx_gpio = oplus_vooc_get_uart_tx();
		rx_gpio = oplus_vooc_get_uart_rx();
		
        adapter_xlog_printk(CHG_LOG_CRTI, " begin\n");
		oplus_vooc_uart_init();
		
        for (i = 0;i < 2;i++) {
                update_result = chip->vops->adapter_update(tx_gpio, rx_gpio);
                if (update_result == true) {
                        break;
                }
                if (i < 1) {
                        msleep(1650);
                }
        }

        if (update_result) {
				oplus_vooc_set_adapter_update_real_status(ADAPTER_FW_UPDATE_SUCCESS);
        } else {
				oplus_vooc_set_adapter_update_real_status(ADAPTER_FW_UPDATE_FAIL);
				oplus_vooc_set_adapter_update_report_status(ADAPTER_FW_UPDATE_FAIL);
				
        }
        msleep(20);
		
		oplus_vooc_uart_reset();
		
        if (update_result) {
                msleep(2000);
				oplus_vooc_set_adapter_update_report_status(ADAPTER_FW_UPDATE_SUCCESS);
        }
        oplus_vooc_battery_update();
        adapter_xlog_printk(CHG_LOG_CRTI, "  end update_result:%d\n", update_result);
        oplus_adapter_set_awake(chip, false);
}

#define ADAPTER_UPDATE_DELAY                          1400
void oplus_adapter_fw_update(void)
{
        struct oplus_adapter_chip *chip = g_adapter_chip ;
		adapter_xlog_printk(CHG_LOG_CRTI, " call \n");
        /*schedule_delayed_work_on(7, &chip->adapter_update_work, */
        /*                        round_jiffies_relative(msecs_to_jiffies(ADAPTER_UPDATE_DELAY)));*/
        schedule_delayed_work(&chip->adapter_update_work,
                                round_jiffies_relative(msecs_to_jiffies(ADAPTER_UPDATE_DELAY)));
}


void oplus_adapter_init(struct oplus_adapter_chip *chip)
{
        g_adapter_chip = chip;
		oplus_adpater_awake_init(chip);
		INIT_DELAYED_WORK(&chip->adapter_update_work, adapter_update_work_func);
}

bool oplus_adapter_check_chip_is_null(void)
{
        if (!g_adapter_chip) {
                return true;
        } else {
                return false;
        }
}

