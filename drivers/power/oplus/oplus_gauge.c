/**********************************************************************************
* Copyright (c)  2008-2015  Guangdong OPLUS Mobile Comm Corp., Ltd
* OPLUS_FEATURE_CHG_BASIC
* Description: Charger IC management module for charger system framework.
*              Manage all charger IC and define abstarct function flow.
* Version    : 1.0
* Date       : 2015-06-22
*            : Fanhong.Kong@ProDrv.CHG
* ------------------------------ Revision History: --------------------------------
* <version>           <date>                <author>                          <desc>
* Revision 1.0        2015-06-22       Fanhong.Kong@ProDrv.CHG       Created for new architecture
***********************************************************************************/

#include "oplus_gauge.h"
#include "oplus_charger.h"
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/debugfs.h>
#include <soc/oplus/system/oplus_project.h>


static struct oplus_gauge_chip *g_gauge_chip = NULL;
static struct oplus_plat_gauge_operations *g_plat_gauge_ops = NULL;
static struct oplus_external_auth_chip *g_external_auth_chip = NULL;

static int gauge_dbg_tbat = 0;
module_param(gauge_dbg_tbat, int, 0644);
MODULE_PARM_DESC(gauge_dbg_tbat, "debug battery temperature");

static int gauge_dbg_vbat = 0;
module_param(gauge_dbg_vbat, int, 0644);
MODULE_PARM_DESC(gauge_dbg_vbat, "debug battery voltage");

static int gauge_dbg_ibat = 0;
module_param(gauge_dbg_ibat, int, 0644);
MODULE_PARM_DESC(gauge_dbg_ibat, "debug battery current");

int oplus_plat_gauge_is_support(void){
	if (!g_plat_gauge_ops) {
		return 0;
	} else {
		return 1;
	}
}

int oplus_gauge_get_plat_batt_mvolts(void) {
	if (!g_plat_gauge_ops) {
		return 0;
	} else {
		return g_plat_gauge_ops->get_plat_battery_mvolts();
	}
	return 0;
}

int oplus_gauge_get_plat_batt_current(void) {
	if (!g_plat_gauge_ops) {
		return 0;
	} else {
		return g_plat_gauge_ops->get_plat_battery_current();
	}
	return 0;
}

int oplus_gauge_get_batt_mvolts(void)
{
	if (!g_gauge_chip) {
		return 3800;
	} else {
	 	if (gauge_dbg_vbat != 0) {
        	printk(KERN_ERR "[OPLUS_CHG]%s:debug enabled,voltage gauge_dbg_vbat[%d] \n",  __func__, gauge_dbg_vbat);
			return gauge_dbg_vbat;
			}
		return g_gauge_chip->gauge_ops->get_battery_mvolts();
	}
}

int oplus_gauge_get_batt_fc(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
		|| !g_gauge_chip->gauge_ops->get_battery_fc) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_fc();
	}
}

int oplus_gauge_get_batt_qm(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
		|| !g_gauge_chip->gauge_ops->get_battery_qm) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_qm();
	}
}

int oplus_gauge_get_batt_pd(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
		|| !g_gauge_chip->gauge_ops->get_battery_pd) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_pd();
	}
}

int oplus_gauge_get_batt_rcu(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
		|| !g_gauge_chip->gauge_ops->get_battery_rcu) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_rcu();
	}
}

int oplus_gauge_get_batt_rcf(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
		|| !g_gauge_chip->gauge_ops->get_battery_rcf) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_rcf();
	}
}

int oplus_gauge_get_batt_fcu(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
		|| !g_gauge_chip->gauge_ops->get_battery_fcu) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_fcu();
	}
}

int oplus_gauge_get_batt_fcf(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
		|| !g_gauge_chip->gauge_ops->get_battery_fcf) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_fcf();
	}
}

int oplus_gauge_get_batt_sou(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
		|| !g_gauge_chip->gauge_ops->get_battery_sou) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_sou();
	}
}

int oplus_gauge_get_batt_do0(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
		|| !g_gauge_chip->gauge_ops->get_battery_do0) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_do0();
	}
}

int oplus_gauge_get_batt_doe(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
                || !g_gauge_chip->gauge_ops->get_battery_doe) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_doe();
	}
}

int oplus_gauge_get_batt_trm(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
                || !g_gauge_chip->gauge_ops->get_battery_trm) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_trm();
	}
}

int oplus_gauge_get_batt_pc(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
		|| !g_gauge_chip->gauge_ops->get_battery_pc) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_pc();
	}
}

int oplus_gauge_get_batt_qs(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
		|| !g_gauge_chip->gauge_ops->get_battery_qs) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_qs();
	}
}

int oplus_gauge_get_batt_mvolts_2cell_max(void)
{
	if(!g_gauge_chip)
		return 3800;
	else
		return g_gauge_chip->gauge_ops->get_battery_mvolts_2cell_max();
}

int oplus_gauge_get_batt_mvolts_2cell_min(void)
{
	if(!g_gauge_chip)
		return 3800;
	else
		return g_gauge_chip->gauge_ops->get_battery_mvolts_2cell_min();
}

int oplus_gauge_get_batt_temperature(void)
{
	int batt_temp = 0;
	if (!g_gauge_chip) {
		return 250;
	} else {
		if (gauge_dbg_tbat != 0) {
			printk(KERN_ERR "[OPLUS_CHG]debug enabled, gauge_dbg_tbat[%d] \n", gauge_dbg_tbat);
			return gauge_dbg_tbat;
			}
		batt_temp = g_gauge_chip->gauge_ops->get_battery_temperature();
/*#ifdef CONFIG_HIGH_TEMP_VERSION*/
		if (get_eng_version() == HIGH_TEMP_AGING) {
			printk(KERN_ERR "[OPLUS_CHG]CONFIG_HIGH_TEMP_VERSION enable here, \
					disable high tbat shutdown \n");
			if (batt_temp > 690)
				batt_temp = 690;
		}
/*#endif*/
		return batt_temp;
	}
}

int oplus_gauge_get_batt_soc(void)
{
	if (!g_gauge_chip) {
		return -1;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_soc();
	}
}

int oplus_gauge_get_batt_current(void)
{
	if (!g_gauge_chip) {
		return 100;
	} else {
		if (gauge_dbg_ibat != 0) {
        	printk(KERN_ERR "[OPLUS_CHG]debug enabled,current gauge_dbg_ibat[%d] \n", gauge_dbg_ibat);
			return gauge_dbg_ibat;
			}
		return g_gauge_chip->gauge_ops->get_average_current();
	}
}

int oplus_gauge_get_remaining_capacity(void)
{
	if (!g_gauge_chip) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_batt_remaining_capacity();
	}
}

int oplus_gauge_get_device_type(void)
{
	if (!g_gauge_chip) {
		return 0;
	} else {
		return g_gauge_chip->device_type;
	}
}

int oplus_gauge_get_device_type_for_vooc(void)
{
	if (!g_gauge_chip) {
		return 0;
	} else {
		return g_gauge_chip->device_type_for_vooc;
	}
}

int oplus_gauge_get_batt_fcc(void)
{
	if (!g_gauge_chip) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_fcc();
	}
}

int oplus_gauge_get_batt_cc(void)
{
	if (!g_gauge_chip) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_cc();
	}
}

int oplus_gauge_get_batt_soh(void)
{
	if (!g_gauge_chip) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_soh();
	}
}

bool oplus_gauge_get_batt_hmac(void)
{
	if (!g_gauge_chip) {
		return false;
	} else if (!g_gauge_chip->gauge_ops->get_battery_hmac) {
		return true;
	} else  {
		return g_gauge_chip->gauge_ops->get_battery_hmac();
	}
}

bool oplus_gauge_get_batt_external_hmac(void)
{
	printk(KERN_ERR "[OPLUS_CHG]%s:oplus_gauge_get_batt_external_hmac \n",  __func__);
	if (!g_external_auth_chip) {
		return false;
	}
	if (!g_external_auth_chip->get_external_auth_hmac) {
		return false;
	} else  {
		return g_external_auth_chip->get_external_auth_hmac();
	}
}

int oplus_gauge_start_test_external_hmac(int count)
{
	printk(KERN_ERR "[OPLUS_CHG]%s:oplus_gauge_start_test_external_hmac \n",  __func__);
	if (!g_external_auth_chip) {
		return false;
	} else  {
		return g_external_auth_chip->start_test_external_hmac(count);
	}
}

int oplus_gauge_get_external_hmac_test_result(int *count_total, int *count_now, int *fail_count)
{
	printk(KERN_ERR "[OPLUS_CHG]%s:oplus_gauge_get_hmac_test_result \n",  __func__);
	if (!g_external_auth_chip) {
		return false;
	} else  {
		return g_external_auth_chip->get_hmac_test_result(count_total, count_now, fail_count);
	}
}

int oplus_gauge_get_external_hmac_status(int *status, int *fail_count, int *total_count,
		int *real_fail_count, int *real_total_count)
{
	printk(KERN_ERR "[OPLUS_CHG]%s:oplus_gauge_get_hmac_status \n",  __func__);
	if (!g_external_auth_chip) {
		return false;
	} else  {
		return g_external_auth_chip->get_hmac_status(status, fail_count,
				total_count, real_fail_count, real_total_count);
	}
}

bool oplus_gauge_get_batt_authenticate(void)
{
	if (!g_gauge_chip) {
		return false;
	} else {
		return g_gauge_chip->gauge_ops->get_battery_authenticate();
	}
}

void oplus_gauge_set_float_uv_ma(int iterm_ma,int float_volt_uv)
{
       if (g_gauge_chip) {
               g_gauge_chip->gauge_ops->set_float_uv_ma(iterm_ma, float_volt_uv);
       }
}

void oplus_gauge_set_batt_full(bool full)
{
	if (g_gauge_chip) {
		g_gauge_chip->gauge_ops->set_battery_full(full);
	}
}

bool oplus_gauge_check_chip_is_null(void)
{
	if (!g_gauge_chip) {
		return true;
	} else {
		return false;
	}
}

void oplus_gauge_init(struct oplus_gauge_chip *chip)
{
	g_gauge_chip = chip;
}

void oplus_plat_gauge_init(struct oplus_plat_gauge_operations *ops)
{
	g_plat_gauge_ops = ops;
}

void oplus_external_auth_init(struct oplus_external_auth_chip *chip)
{
	g_external_auth_chip = chip;
}

int oplus_gauge_get_prev_batt_mvolts(void)
{
	if (!g_gauge_chip)
		return 3800;
	else {
		if (gauge_dbg_vbat != 0) {
			printk(KERN_ERR "[OPLUS_CHG]%s:debug enabled,voltage gauge_dbg_vbat[%d] \n",  __func__, gauge_dbg_vbat);
			return gauge_dbg_vbat;
		}
		return g_gauge_chip->gauge_ops->get_prev_battery_mvolts();
	}
}

int oplus_gauge_get_prev_batt_mvolts_2cell_max(void)
{
	if(!g_gauge_chip)
		return 3800;
	else{
		if (gauge_dbg_vbat != 0) {
		    printk(KERN_ERR "[OPLUS_CHG]%s: debug enabled,voltage gauge_dbg_vbat[%d] \n", __func__, gauge_dbg_vbat);
		    return gauge_dbg_vbat;
		}
		return g_gauge_chip->gauge_ops->get_prev_battery_mvolts_2cell_max();
		}
}

int oplus_gauge_get_prev_batt_mvolts_2cell_min(void)
{
	if(!g_gauge_chip)
		return 3800;
	else {
		if (gauge_dbg_vbat != 0) {
			printk(KERN_ERR "[OPLUS_CHG]%s:debug enabled,voltage gauge_dbg_vbat[%d] \n",  __func__, gauge_dbg_vbat);
			return gauge_dbg_vbat;
			}
		return g_gauge_chip->gauge_ops->get_prev_battery_mvolts_2cell_min();
    }
}

int oplus_gauge_get_prev_batt_temperature(void)
{
	int batt_temp = 0;
	if (!g_gauge_chip)
		return 250;
	else {
		if (gauge_dbg_tbat != 0) {
			printk(KERN_ERR "[OPLUS_CHG]%s: debug enabled, gauge_dbg_tbat[%d] \n", __func__, gauge_dbg_tbat);
			return gauge_dbg_tbat;
		}
		batt_temp = g_gauge_chip->gauge_ops->get_prev_battery_temperature();
/*#ifdef CONFIG_HIGH_TEMP_VERSION*/
	if (get_eng_version() == HIGH_TEMP_AGING) {
		printk(KERN_ERR "[OPLUS_CHG]CONFIG_HIGH_TEMP_VERSION enable here, \
				disable high tbat shutdown \n");
		if (batt_temp > 690)
			batt_temp = 690;
	}
/*#endif*/
		return batt_temp;
	}
}

int oplus_gauge_get_prev_batt_soc(void)
{
	if (!g_gauge_chip)
		return 50;
	else
		return g_gauge_chip->gauge_ops->get_prev_battery_soc();
}

int oplus_gauge_get_prev_batt_current(void)
{
	if (!g_gauge_chip)
		return 100;
	else {
		if (gauge_dbg_ibat != 0) {
			printk(KERN_ERR "[OPLUS_CHG]%s:debug enabled,current gauge_dbg_ibat[%d] \n", __func__, gauge_dbg_ibat);
			return gauge_dbg_ibat;
		}
		return g_gauge_chip->gauge_ops->get_prev_average_current();
		}
}

int oplus_gauge_get_prev_remaining_capacity(void)
{
	if (!g_gauge_chip) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_prev_batt_remaining_capacity();
	}
}

int oplus_gauge_update_battery_dod0(void)
{
	if (!g_gauge_chip)
		return 0;
	else
		return g_gauge_chip->gauge_ops->update_battery_dod0();
}

int oplus_gauge_update_soc_smooth_parameter(void)
{
	if (!g_gauge_chip)
		return 0;
	else
		return g_gauge_chip->gauge_ops->update_soc_smooth_parameter();
}

int oplus_gauge_get_battery_cb_status(void)
{
	if (!g_gauge_chip)
		return 0;
	else
		return g_gauge_chip->gauge_ops->get_battery_cb_status();
}

int oplus_gauge_get_i2c_err(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
		|| !g_gauge_chip->gauge_ops->get_gauge_i2c_err) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_gauge_i2c_err();
	}
}

void oplus_gauge_clear_i2c_err(void)
{
	if (!g_gauge_chip || !g_gauge_chip->gauge_ops
		|| !g_gauge_chip->gauge_ops->clear_gauge_i2c_err) {
		return;
	} else {
		return g_gauge_chip->gauge_ops->clear_gauge_i2c_err();
	}
}

int oplus_gauge_get_prev_batt_fcc(void)
{
	if (!g_gauge_chip) {
		return 0;
	} else {
		return g_gauge_chip->gauge_ops->get_prev_batt_fcc();
	}
}

int oplus_gauge_protect_check(void)
{
	if (!g_gauge_chip) {
		return true;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->protect_check) {
			return g_gauge_chip->gauge_ops->protect_check();
		}
		return true;
	}
}

bool oplus_gauge_afi_update_done(void)
{
	if (!g_gauge_chip) {
		return true;
	} else {
		if (g_gauge_chip->gauge_ops && g_gauge_chip->gauge_ops->afi_update_done) {
			return g_gauge_chip->gauge_ops->afi_update_done();
		}
		return true;
	}
}

