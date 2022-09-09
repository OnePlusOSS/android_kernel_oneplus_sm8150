/************************************************************************************
 ** File:  \\192.168.144.3\Linux_Share\12015\ics2\development\mediatek\custom\oplus77_12015\kernel\battery\battery
 ** VENDOR_EDIT
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
 * <version>       <date>        <author>              			<desc>
 * Revision 1.0    2015-06-22    Fanhong.Kong@ProDrv.CHG   		Created for new architecture
 * Revision 2.0    2018-04-14    Fanhong.Kong@ProDrv.CHG     	Upgrade for SVOOC
 ************************************************************************************************************/

#define pr_fmt(fmt)	"[bq2597x] %s: " fmt, __func__

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>

#include "../oplus_vooc.h"
#include "../oplus_gauge.h"
#include "oplus_bq2597x.h"
/*#include "bq2597x.h"*/

enum {
	ADC_IBUS,
	ADC_VBUS,
	ADC_VAC,
	ADC_VOUT,
	ADC_VBAT,
	ADC_IBAT,
	ADC_TBUS,
	ADC_TBAT,
	ADC_TDIE,
	ADC_MAX_NUM,
};

#define BQ25970_ROLE_STDALONE   0
#define BQ25970_ROLE_SLAVE	1
#define BQ25970_ROLE_MASTER	2

enum {
	BQ25970_STDALONE, BQ25970_SLAVE, BQ25970_MASTER,
};

static int bq2597x_mode_data[] = { [BQ25970_STDALONE] = BQ25970_STDALONE,
		[BQ25970_MASTER] = BQ25970_ROLE_MASTER, [BQ25970_SLAVE
				] = BQ25970_ROLE_SLAVE, };

#define	BAT_OVP_ALARM		BIT(7)
#define BAT_OCP_ALARM		BIT(6)
#define	BUS_OVP_ALARM		BIT(5)
#define	BUS_OCP_ALARM		BIT(4)
#define	BAT_UCP_ALARM		BIT(3)
#define	VBUS_INSERT		BIT(2)
#define VBAT_INSERT		BIT(1)
#define	ADC_DONE		BIT(0)

#define BAT_OVP_FAULT		BIT(7)
#define BAT_OCP_FAULT		BIT(6)
#define BUS_OVP_FAULT		BIT(5)
#define BUS_OCP_FAULT		BIT(4)
#define TBUS_TBAT_ALARM		BIT(3)
#define TS_BAT_FAULT		BIT(2)
#define	TS_BUS_FAULT		BIT(1)
#define	TS_DIE_FAULT		BIT(0)

/*below used for comm with other module*/
#define	BAT_OVP_FAULT_SHIFT			0
#define	BAT_OCP_FAULT_SHIFT			1
#define	BUS_OVP_FAULT_SHIFT			2
#define	BUS_OCP_FAULT_SHIFT			3
#define	BAT_THERM_FAULT_SHIFT			4
#define	BUS_THERM_FAULT_SHIFT			5
#define	DIE_THERM_FAULT_SHIFT			6

#define	BAT_OVP_FAULT_MASK		(1 << BAT_OVP_FAULT_SHIFT)
#define	BAT_OCP_FAULT_MASK		(1 << BAT_OCP_FAULT_SHIFT)
#define	BUS_OVP_FAULT_MASK		(1 << BUS_OVP_FAULT_SHIFT)
#define	BUS_OCP_FAULT_MASK		(1 << BUS_OCP_FAULT_SHIFT)
#define	BAT_THERM_FAULT_MASK		(1 << BAT_THERM_FAULT_SHIFT)
#define	BUS_THERM_FAULT_MASK		(1 << BUS_THERM_FAULT_SHIFT)
#define	DIE_THERM_FAULT_MASK		(1 << DIE_THERM_FAULT_SHIFT)

#define	BAT_OVP_ALARM_SHIFT			0
#define	BAT_OCP_ALARM_SHIFT			1
#define	BUS_OVP_ALARM_SHIFT			2
#define	BUS_OCP_ALARM_SHIFT			3
#define	BAT_THERM_ALARM_SHIFT			4
#define	BUS_THERM_ALARM_SHIFT			5
#define	DIE_THERM_ALARM_SHIFT			6
#define BAT_UCP_ALARM_SHIFT			7

#define	BAT_OVP_ALARM_MASK		(1 << BAT_OVP_ALARM_SHIFT)
#define	BAT_OCP_ALARM_MASK		(1 << BAT_OCP_ALARM_SHIFT)
#define	BUS_OVP_ALARM_MASK		(1 << BUS_OVP_ALARM_SHIFT)
#define	BUS_OCP_ALARM_MASK		(1 << BUS_OCP_ALARM_SHIFT)
#define	BAT_THERM_ALARM_MASK		(1 << BAT_THERM_ALARM_SHIFT)
#define	BUS_THERM_ALARM_MASK		(1 << BUS_THERM_ALARM_SHIFT)
#define	DIE_THERM_ALARM_MASK		(1 << DIE_THERM_ALARM_SHIFT)
#define	BAT_UCP_ALARM_MASK		(1 << BAT_UCP_ALARM_SHIFT)

#define VBAT_REG_STATUS_SHIFT			0
#define IBAT_REG_STATUS_SHIFT			1

#define VBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)
#define IBAT_REG_STATUS_MASK		(1 << VBAT_REG_STATUS_SHIFT)
#define chg_err(fmt, ...) \
        printk(KERN_ERR "[OPLUS_CHG][%s]"fmt, __func__, ##__VA_ARGS__)

#define bq_err(fmt, ...)								\
do {											\
	if (bq->mode == BQ25970_ROLE_MASTER)						\
		printk(KERN_ERR "[bq2597x-MASTER]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else if (bq->mode == BQ25970_ROLE_SLAVE)					\
		printk(KERN_ERR "[bq2597x-SLAVE]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else										\
		printk(KERN_ERR "[bq2597x-STANDALONE]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while (0);

#define bq_info(fmt, ...)								\
do {											\
	if (bq->mode == BQ25970_ROLE_MASTER)						\
		printk(KERN_INFO "[bq2597x-MASTER]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else if (bq->mode == BQ25970_ROLE_SLAVE)					\
		printk(KERN_INFO "[bq2597x-SLAVE]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else										\
		printk(KERN_INFO "[bq2597x-STANDALONE]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while (0);

#define bq_dbg(fmt, ...)								\
do {											\
	if (bq->mode == BQ25970_ROLE_MASTER)						\
		printk(KERN_DEBUG "[bq2597x-MASTER]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else if (bq->mode == BQ25970_ROLE_SLAVE)					\
		printk(KERN_DEBUG "[bq2597x-SLAVE]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else										\
		printk(KERN_DEBUG "[bq2597x-STANDALONE]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while (0);

/*end*/

struct bq2597x_cfg {
	bool bat_ovp_disable;
	bool bat_ocp_disable;
	bool bat_ovp_alm_disable;
	bool bat_ocp_alm_disable;

	int bat_ovp_th;
	int bat_ovp_alm_th;
	int bat_ocp_th;
	int bat_ocp_alm_th;

	bool bus_ovp_alm_disable;
	bool bus_ocp_disable;
	bool bus_ocp_alm_disable;

	int bus_ovp_th;
	int bus_ovp_alm_th;
	int bus_ocp_th;
	int bus_ocp_alm_th;

	bool bat_ucp_alm_disable;

	int bat_ucp_alm_th;
	int ac_ovp_th;

	bool bat_therm_disable;
	bool bus_therm_disable;
	bool die_therm_disable;

	int bat_therm_th; /*in %*/
	int bus_therm_th; /*in %*/
	int die_therm_th; /*in degC*/

	int sense_r_mohm;
};

struct bq2597x {
	struct device *dev;
	struct i2c_client *client;

	int part_no;
	int revision;

	int mode;
	struct mutex pinctrl_mutex;

	struct mutex data_lock;
	struct mutex i2c_rw_lock;
	struct mutex charging_disable_lock;
	struct mutex irq_complete;
	struct delayed_work cp_lc_work;

	int cp_lc_gpio;
	int cp_en;
	int cp_lc_irq;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool batt_present;
	bool vbus_present;

	bool usb_present;
	bool charge_enabled; /* Register bit status */

	/* ADC reading */
	int vbat_volt;
	int vbus_volt;
	int vout_volt;
	int vac_volt;

	int ibat_curr;
	int ibus_curr;

	int bat_temp;
	int bus_temp;
	int die_temp;

	/* alarm/fault status */
	bool bat_ovp_fault;
	bool bat_ocp_fault;
	bool bus_ovp_fault;
	bool bus_ocp_fault;

	bool bat_ovp_alarm;
	bool bat_ocp_alarm;
	bool bus_ovp_alarm;
	bool bus_ocp_alarm;

	bool bat_ucp_alarm;

	bool bat_therm_alarm;
	bool bus_therm_alarm;
	bool die_therm_alarm;

	bool bat_therm_fault;
	bool bus_therm_fault;
	bool die_therm_fault;

	bool therm_shutdown_flag;
	bool therm_shutdown_stat;

	bool vbat_reg;
	bool ibat_reg;

	int prev_alarm;
	int prev_fault;

	int chg_ma;
	int chg_mv;

	int charge_state;

	struct bq2597x_cfg *cfg;

	int skip_writes;
	int skip_reads;

	struct bq2597x_platform_data *platform_data;

	struct delayed_work monitor_work;

	struct dentry *debug_root;

	struct power_supply_desc psy_desc;
	struct power_supply_config psy_cfg;
	struct power_supply *fc2_psy;
};

static struct bq2597x *charger_ic = NULL;
void bq2597x_dump_reg(void);

/************************************************************************/
static int __bq2597x_read_byte(struct bq2597x *bq, u8 reg, u8 *data) {
	s32 ret;

	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		bq_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8) ret;

	return 0;
}

static int __bq2597x_write_byte(struct bq2597x *bq, int reg, u8 val) {
	s32 ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		bq_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n", val,
				reg, ret);
		return ret;
	}
	return 0;
}

static int __bq2597x_read_word(struct bq2597x *bq, u8 reg, u16 *data) {
	s32 ret;

	ret = i2c_smbus_read_word_data(bq->client, reg);
	if (ret < 0) {
		bq_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u16) ret;

	return 0;
}

static int bq2597x_read_byte(struct bq2597x *bq, u8 reg, u8 *data) {
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2597x_read_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2597x_write_byte(struct bq2597x *bq, u8 reg, u8 data) {
	int ret;

	if (bq->skip_writes)
		return 0;

	if (reg == 0xf) {
		bq_err("bq2597x_write_byte reg=%02X, ret=%d\n", reg, data);
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2597x_write_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2597x_read_word(struct bq2597x *bq, u8 reg, u16 *data) {
	int ret;

	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2597x_read_word(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2597x_update_bits(struct bq2597x *bq, u8 reg, u8 mask, u8 data) {
	int ret;
	u8 tmp;

	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2597x_read_byte(bq, reg, &tmp);
	if (ret) {
		bq_err("Failed: reg=%02X, ret=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2597x_write_byte(bq, reg, tmp);
	if (ret)
		bq_err("Failed: reg=%02X, ret=%d\n", reg, ret);

	out: mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

/*********************************************************************/

void bq2597x_hardware_init(void) {
	struct bq2597x *bq = charger_ic;
	int ret;
	if (!charger_ic) {
		bq_err("bq2597x_hardware_init Failed: charger_ic  is null\r\n");
	}
	ret = bq2597x_write_byte(bq, 0x6, 0x64);
	ret = bq2597x_write_byte(bq, 0x0e, 0x6c);
	pr_err(" [OPLUS_CHG][bq2597x_hardware_init_normal_charge]");
}

int bq2597x_hardware_init_pd_qc(void) {
	struct bq2597x *bq = charger_ic;
	int ret;
	bq_err("bq2597x_cp_init_pd_qc\n");
	if (!charger_ic) {
		bq_err("bq2597x_hardware_init_pd_qc Failed: charger_ic  is null\r\n")
	}
	ret = bq2597x_write_byte(bq, 0x6, 0x46);
	ret = bq2597x_write_byte(bq, 0x0e, 0x24);

	return 0;
}

int bq2597x_hardware_init_svooc(void) {
	struct bq2597x *bq = charger_ic;
	int ret;
	if (!charger_ic) {
		bq_err("bq2597x_hardware_init Failed: charger_ic  is null\r\n")
	}
	ret = bq2597x_write_byte(bq, 0x0, 0x78);
	ret = bq2597x_write_byte(bq, 0x1, 0xc6);
	ret = bq2597x_write_byte(bq, 0x2, 0xda);
	ret = bq2597x_write_byte(bq, 0x3, 0xd1);
	ret = bq2597x_write_byte(bq, 0x4, 0xa8);
	ret = bq2597x_write_byte(bq, 0x5, 0x48);
	ret = bq2597x_write_byte(bq, 0x6, 0x5a);
	ret = bq2597x_write_byte(bq, 0x7, 0xda);
	ret = bq2597x_write_byte(bq, 0x8, 0x13);
	ret = bq2597x_write_byte(bq, 0x9, 0x9c);
	ret = bq2597x_write_byte(bq, 0x0a, 0x68);
	ret = bq2597x_write_byte(bq, 0x0b, 0xc8);
	ret = bq2597x_write_byte(bq, 0x0c, 0x66);
	ret = bq2597x_write_byte(bq, 0x0d, 0x66);
	ret = bq2597x_write_byte(bq, 0x0e, 0x48);
	ret = bq2597x_write_byte(bq, 0x0f, 0x02);
	ret = bq2597x_write_byte(bq, 0x10, 0x90);
	ret = bq2597x_write_byte(bq, 0x11, 0x6f);
	ret = bq2597x_write_byte(bq, 0x12, 0x30);
	ret = bq2597x_write_byte(bq, 0x23, 0x90);
	ret = bq2597x_write_byte(bq, 0x24, 0x44);
	//bq2597x_dump_reg();
	pr_info("[OPLUS_CHG][bq2597x_hardware_init_svooc]\r\n");

	return 0;
}

int bq2597x_hardware_init_normal_vooc(void) {
	struct bq2597x *bq = charger_ic;
	int ret;
	if (!charger_ic) {
		bq_err("bq2597x_hardware_init Failed: charger_ic  is null\r\n");
	}

	ret = bq2597x_write_byte(bq, 0x0f, 0x0a);
	ret = bq2597x_write_byte(bq, 0x0, 0x78);
	ret = bq2597x_write_byte(bq, 0x1, 0xc6);
	ret = bq2597x_write_byte(bq, 0x2, 0xda);
	ret = bq2597x_write_byte(bq, 0x3, 0xd1);
	ret = bq2597x_write_byte(bq, 0x4, 0xa8);
	ret = bq2597x_write_byte(bq, 0x5, 0x48);
	ret = bq2597x_write_byte(bq, 0x6, 0x64);
	ret = bq2597x_write_byte(bq, 0x7, 0xe4);
	ret = bq2597x_write_byte(bq, 0x8, 0x17);
	ret = bq2597x_write_byte(bq, 0x9, 0x94);
	ret = bq2597x_write_byte(bq, 0x0a, 0x68);
	ret = bq2597x_write_byte(bq, 0x0b, 0xc8);
	ret = bq2597x_write_byte(bq, 0x0c, 0x66);
	ret = bq2597x_write_byte(bq, 0x0d, 0x66);
	ret = bq2597x_write_byte(bq, 0x0e, 0x00);
	ret = bq2597x_write_byte(bq, 0x10, 0x90);
	ret = bq2597x_write_byte(bq, 0x11, 0x6f);
	ret = bq2597x_write_byte(bq, 0x12, 0x30);
	ret = bq2597x_write_byte(bq, 0x23, 0x90);
	ret = bq2597x_write_byte(bq, 0x24, 0x44);
	//bq2597x_dump_reg();
	pr_info("[OPLUS_CHG][bq2597x_hardware_init_normal_vooc]\r\n");

	return 0;
}

static int bq2597x_enable_charge(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_CHG_ENABLE;
	else
		val = BQ2597X_CHG_DISABLE;

	if (enable) {
		ret = bq2597x_read_byte(bq, 0x0f, &val);
		val |= (1 << 4);
		ret = bq2597x_write_byte(bq, 0x0f, val);
		ret = bq2597x_read_byte(bq, 0x0f, &val);
	} else {
		ret = bq2597x_read_byte(bq, 0x0f, &val);
		val &= (~(1 << 4));
		ret = bq2597x_write_byte(bq, 0x0f, val);
		ret = bq2597x_read_byte(bq, 0x0f, &val);
	}

	bq_err("lkl bq2597x_enable_charge 0X0F = 0x%x enable = %d\n", val, enable);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_charge);

static int bq2597x_check_charge_enabled(struct bq2597x *bq, bool *enabled) {
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0C, &val);
	if (!ret)
		*enabled = !!(val & BQ2597X_CHG_EN_MASK);
	return ret;
}

int bq2597x_reset_cp(struct bq2597x *bq) {
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, 0x0f, &val);
	val |= (1 << 7);
	ret = bq2597x_write_byte(bq, 0x0f, val);
	ret = bq2597x_read_byte(bq, 0x0f, &val);

	bq_err("lkl bq2597x_reset_cp 0X0F = 0x%x\n", val);
	return ret;
}

int oplus_bq2597x_reset_cp(void) {
	struct bq2597x *bq = charger_ic;
	if (!charger_ic) {
		bq_err("bq2597x_hardware_init Failed: charger_ic  is null\r\n");
	}

	bq2597x_reset_cp(bq);
	bq_err("oplus_bq2597x_reset_cp ok \r\n");
	return 0;
}
int oplus_bq2597x_otg_en(int en) {
	struct bq2597x *bq = charger_ic;
	int ret;
	printk("oplus_bq2597x_otg_en ok\n");
	if (!charger_ic) {
		bq_err("bq2597x_hardware_init Failed: charger_ic  is null\r\n");
	}

	if (en) {
		ret = bq2597x_write_byte(bq, 0x10, 0x94);
		ret = bq2597x_write_byte(bq, 0x0f, 0x20);
		ret = bq2597x_write_byte(bq, 0x0f, 0x22);
	} else {
		ret = bq2597x_write_byte(bq, 0x0e, 0x26);
		ret = bq2597x_write_byte(bq, 0x0f, 0x02);
		bq2597x_reset_cp(bq);
		ret = bq2597x_write_byte(bq, 0x0e, 0x24);
	}

	return 0;
}

int oplus_bq2597x_ovp_en(int en) {
	struct bq2597x *bq = charger_ic;
	int ret;
	printk("oplus_bq2597x_ovp_en\n");
	if (!charger_ic) {
		bq_err("oplus_bq2597x_ovp_en Failed: charger_ic  is null\r\n");
	}

	if (en) {
		bq2597x_reset_cp(bq);
	} else {
		ret = bq2597x_write_byte(bq, 0x0f, 0x4);
	}

	return 0;
}

static int bq2597x_enable_wdt(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_WATCHDOG_ENABLE;
	else
		val = BQ2597X_WATCHDOG_DISABLE;

	val <<= BQ2597X_WATCHDOG_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_0B,
	BQ2597X_WATCHDOG_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_wdt);

static int bq2597x_set_wdt(struct bq2597x *bq, int ms) {
	int ret;
	u8 val;

	if (ms == 500)
		val = BQ2597X_WATCHDOG_0P5S;
	else if (ms == 1000)
		val = BQ2597X_WATCHDOG_1S;
	else if (ms == 5000)
		val = BQ2597X_WATCHDOG_5S;
	else if (ms == 30000)
		val = BQ2597X_WATCHDOG_30S;
	else
		val = BQ2597X_WATCHDOG_30S;

	val <<= BQ2597X_WATCHDOG_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_0B,BQ2597X_WATCHDOG_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_wdt);

static int bq2597x_enable_batovp(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_OVP_ENABLE;
	else
		val = BQ2597X_BAT_OVP_DISABLE;

	val <<= BQ2597X_BAT_OVP_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_00,BQ2597X_BAT_OVP_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batovp);

static int bq2597x_set_batovp_th(struct bq2597x *bq, int threshold) {
	int ret;
	u8 val;

	if (threshold < BQ2597X_BAT_OVP_BASE)
		threshold = BQ2597X_BAT_OVP_BASE;

	val = (threshold - BQ2597X_BAT_OVP_BASE) / BQ2597X_BAT_OVP_LSB;

	val <<= BQ2597X_BAT_OVP_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_00,
	BQ2597X_BAT_OVP_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batovp_th);

static int bq2597x_enable_batovp_alarm(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_OVP_ALM_ENABLE;
	else
		val = BQ2597X_BAT_OVP_ALM_DISABLE;

	val <<= BQ2597X_BAT_OVP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_01,
	BQ2597X_BAT_OVP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batovp_alarm);

static int bq2597x_set_batovp_alarm_th(struct bq2597x *bq, int threshold) {
	int ret;
	u8 val;

	if (threshold < BQ2597X_BAT_OVP_ALM_BASE)
		threshold = BQ2597X_BAT_OVP_ALM_BASE;

	val = (threshold - BQ2597X_BAT_OVP_ALM_BASE) / BQ2597X_BAT_OVP_ALM_LSB;

	val <<= BQ2597X_BAT_OVP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_01,
	BQ2597X_BAT_OVP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batovp_alarm_th);

static int bq2597x_enable_batocp(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_OCP_ENABLE;
	else
		val = BQ2597X_BAT_OCP_DISABLE;

	val <<= BQ2597X_BAT_OCP_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_02,
	BQ2597X_BAT_OCP_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batocp);

static int bq2597x_set_batocp_th(struct bq2597x *bq, int threshold) {
	int ret;
	u8 val;

	if (threshold < BQ2597X_BAT_OCP_BASE)
		threshold = BQ2597X_BAT_OCP_BASE;

	val = (threshold - BQ2597X_BAT_OCP_BASE) / BQ2597X_BAT_OCP_LSB;

	val <<= BQ2597X_BAT_OCP_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_02,
	BQ2597X_BAT_OCP_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batocp_th);

static int bq2597x_enable_batocp_alarm(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_OCP_ALM_ENABLE;
	else
		val = BQ2597X_BAT_OCP_ALM_DISABLE;

	val <<= BQ2597X_BAT_OCP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_03,
	BQ2597X_BAT_OCP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batocp_alarm);

static int bq2597x_set_batocp_alarm_th(struct bq2597x *bq, int threshold) {
	int ret;
	u8 val;

	if (threshold < BQ2597X_BAT_OCP_ALM_BASE)
		threshold = BQ2597X_BAT_OCP_ALM_BASE;

	val = (threshold - BQ2597X_BAT_OCP_ALM_BASE) / BQ2597X_BAT_OCP_ALM_LSB;

	val <<= BQ2597X_BAT_OCP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_03,
	BQ2597X_BAT_OCP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batocp_alarm_th);

static int bq2597x_set_busovp_th(struct bq2597x *bq, int threshold) {
	int ret;
	u8 val;

	if (threshold < BQ2597X_BUS_OVP_BASE)
		threshold = BQ2597X_BUS_OVP_BASE;

	val = (threshold - BQ2597X_BUS_OVP_BASE) / BQ2597X_BUS_OVP_LSB;

	val <<= BQ2597X_BUS_OVP_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_06,
	BQ2597X_BUS_OVP_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_busovp_th);

static int bq2597x_enable_busovp_alarm(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BUS_OVP_ALM_ENABLE;
	else
		val = BQ2597X_BUS_OVP_ALM_DISABLE;

	val <<= BQ2597X_BUS_OVP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_07,
	BQ2597X_BUS_OVP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_busovp_alarm);

static int bq2597x_set_busovp_alarm_th(struct bq2597x *bq, int threshold) {
	int ret;
	u8 val;

	if (threshold < BQ2597X_BUS_OVP_ALM_BASE)
		threshold = BQ2597X_BUS_OVP_ALM_BASE;

	val = (threshold - BQ2597X_BUS_OVP_ALM_BASE) / BQ2597X_BUS_OVP_ALM_LSB;

	val <<= BQ2597X_BUS_OVP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_07,
	BQ2597X_BUS_OVP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_busovp_alarm_th);

static int bq2597x_enable_busocp(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BUS_OCP_ENABLE;
	else
		val = BQ2597X_BUS_OCP_DISABLE;

	val <<= BQ2597X_BUS_OCP_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_08,
	BQ2597X_BUS_OCP_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_busocp);

static int bq2597x_set_busocp_th(struct bq2597x *bq, int threshold) {
	int ret;
	u8 val;

	if (threshold < BQ2597X_BUS_OCP_BASE)
		threshold = BQ2597X_BUS_OCP_BASE;

	val = (threshold - BQ2597X_BUS_OCP_BASE) / BQ2597X_BUS_OCP_LSB;

	val <<= BQ2597X_BUS_OCP_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_08,
	BQ2597X_BUS_OCP_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_busocp_th);

static int bq2597x_enable_busocp_alarm(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BUS_OCP_ALM_ENABLE;
	else
		val = BQ2597X_BUS_OCP_ALM_DISABLE;

	val <<= BQ2597X_BUS_OCP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_09,
	BQ2597X_BUS_OCP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_busocp_alarm);

static int bq2597x_set_busocp_alarm_th(struct bq2597x *bq, int threshold) {
	int ret;
	u8 val;

	if (threshold < BQ2597X_BUS_OCP_ALM_BASE)
		threshold = BQ2597X_BUS_OCP_ALM_BASE;

	val = (threshold - BQ2597X_BUS_OCP_ALM_BASE) / BQ2597X_BUS_OCP_ALM_LSB;

	val <<= BQ2597X_BUS_OCP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_09,
	BQ2597X_BUS_OCP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_busocp_alarm_th);

static int bq2597x_enable_batucp_alarm(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_BAT_UCP_ALM_ENABLE;
	else
		val = BQ2597X_BAT_UCP_ALM_DISABLE;

	val <<= BQ2597X_BAT_UCP_ALM_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_04,
	BQ2597X_BAT_UCP_ALM_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_batucp_alarm);

static int bq2597x_set_batucp_alarm_th(struct bq2597x *bq, int threshold) {
	int ret;
	u8 val;

	if (threshold < BQ2597X_BAT_UCP_ALM_BASE)
		threshold = BQ2597X_BAT_UCP_ALM_BASE;

	val = (threshold - BQ2597X_BAT_UCP_ALM_BASE) / BQ2597X_BAT_UCP_ALM_LSB;

	val <<= BQ2597X_BAT_UCP_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_04,
	BQ2597X_BAT_UCP_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_batucp_alarm_th);

static int bq2597x_set_acovp_th(struct bq2597x *bq, int threshold) {
	int ret;
	u8 val;

	if (threshold < BQ2597X_AC_OVP_BASE)
		threshold = BQ2597X_AC_OVP_BASE;

	if (threshold == BQ2597X_AC_OVP_6P5V)
		val = 0x07;
	else
		val = (threshold - BQ2597X_AC_OVP_BASE) / BQ2597X_AC_OVP_LSB;

	val <<= BQ2597X_AC_OVP_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_05,
	BQ2597X_AC_OVP_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2597x_set_acovp_th);

static int bq2597x_set_vdrop_th(struct bq2597x *bq, int threshold) {
	int ret;
	u8 val;

	if (threshold == 300)
		val = BQ2597X_VDROP_THRESHOLD_300MV;
	else
		val = BQ2597X_VDROP_THRESHOLD_400MV;

	val <<= BQ2597X_VDROP_THRESHOLD_SET_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_05,
	BQ2597X_VDROP_THRESHOLD_SET_MASK, val);

	return ret;
}

static int bq2597x_set_vdrop_deglitch(struct bq2597x *bq, int us) {
	int ret;
	u8 val;

	if (us == 8)
		val = BQ2597X_VDROP_DEGLITCH_8US;
	else
		val = BQ2597X_VDROP_DEGLITCH_5MS;

	val <<= BQ2597X_VDROP_DEGLITCH_SET_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_05,
	BQ2597X_VDROP_DEGLITCH_SET_MASK, val);
	return ret;
}

static int bq2597x_enable_bat_therm(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_TSBAT_ENABLE;
	else
		val = BQ2597X_TSBAT_DISABLE;

	val <<= BQ2597X_TSBAT_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_0C,
	BQ2597X_TSBAT_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_bat_therm);

/*
 * the input threshold is the raw value that would write to register directly.
 */
static int bq2597x_set_bat_therm_th(struct bq2597x *bq, u8 threshold) {
	int ret;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_29, threshold);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_bat_therm_th);

static int bq2597x_enable_bus_therm(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_TSBUS_ENABLE;
	else
		val = BQ2597X_TSBUS_DISABLE;

	val <<= BQ2597X_TSBUS_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_0C,
	BQ2597X_TSBUS_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_bus_therm);

/*
 * the input threshold is the raw value that would write to register directly.
 */
static int bq2597x_set_bus_therm_th(struct bq2597x *bq, u8 threshold) {
	int ret;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_28, threshold);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_bus_therm_th);

static int bq2597x_enable_die_therm(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_TDIE_ENABLE;
	else
		val = BQ2597X_TDIE_DISABLE;

	val <<= BQ2597X_TDIE_DIS_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_0C,
	BQ2597X_TDIE_DIS_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_die_therm);

/*
 * please be noted that the unit here is degC
 */
static int bq2597x_set_die_therm_th(struct bq2597x *bq, u8 threshold) {
	int ret;
	u8 val;

	/*BE careful, LSB is here is 1/LSB, so we use multiply here*/
	val = (threshold - BQ2597X_TDIE_ALM_BASE) * BQ2597X_TDIE_ALM_LSB;
	val <<= BQ2597X_TDIE_ALM_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_2A,
	BQ2597X_TDIE_ALM_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_die_therm_th);

static int bq2597x_enable_adc(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_ADC_ENABLE;
	else
		val = BQ2597X_ADC_DISABLE;

	val <<= BQ2597X_ADC_EN_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_14,
	BQ2597X_ADC_EN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_enable_adc);

static int bq2597x_set_adc_average(struct bq2597x *bq, bool avg) {
	int ret;
	u8 val;

	if (avg)
		val = BQ2597X_ADC_AVG_ENABLE;
	else
		val = BQ2597X_ADC_AVG_DISABLE;

	val <<= BQ2597X_ADC_AVG_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_14,
	BQ2597X_ADC_AVG_MASK, val);
	return 0;
}
EXPORT_SYMBOL_GPL(bq2597x_set_adc_average);

static int bq2597x_set_adc_scanrate(struct bq2597x *bq, bool oneshot) {
	int ret;
	u8 val;

	if (oneshot)
		val = BQ2597X_ADC_RATE_ONESHOT;
	else
		val = BQ2597X_ADC_RATE_CONTINOUS;

	val <<= BQ2597X_ADC_RATE_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_14,
	BQ2597X_ADC_EN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_adc_scanrate);

static int bq2597x_set_adc_bits(struct bq2597x *bq, int bits) {
	int ret;
	u8 val;

	if (bits > 15)
		bits = 15;
	if (bits < 12)
		bits = 12;
	val = 15 - bits;

	val <<= BQ2597X_ADC_SAMPLE_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_14,
	BQ2597X_ADC_SAMPLE_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_adc_bits);

#define ADC_REG_BASE 0x16
static int bq2597x_get_adc_data(struct bq2597x *bq, int channel, int *result) {
	int ret;
	u16 val;
	s16 t;

	if (channel > ADC_MAX_NUM)
		return -EINVAL;

	ret = bq2597x_read_word(bq, ADC_REG_BASE + (channel << 1), &val);
	if (ret < 0)
		return ret;
	t = val & 0xFF;
	t <<= 8;
	t |= (val >> 8) & 0xFF;
	*result = t;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2597x_get_adc_data);

static int bq2597x_set_adc_scan(struct bq2597x *bq, int channel, bool enable) {
	int ret;
	u8 reg;
	u8 mask;
	u8 shift;
	u8 val;

	if (channel > ADC_MAX_NUM)
		return -EINVAL;

	if (channel == ADC_IBUS) {
		reg = BQ2597X_REG_14;
		shift = BQ2597X_IBUS_ADC_DIS_SHIFT;
		mask = BQ2597X_IBUS_ADC_DIS_MASK;
	} else {
		reg = BQ2597X_REG_15;
		shift = 8 - channel;
		mask = 1 << shift;
	}

	if (enable)
		val = 0 << shift;
	else
		val = 1 << shift;

	ret = bq2597x_update_bits(bq, reg, mask, val);

	return ret;
}

static int bq2597x_set_alarm_int_mask(struct bq2597x *bq, u8 mask) {
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0F, &val);
	if (ret)
		return ret;

	val |= mask;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_0F, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_alarm_int_mask);

static int bq2597x_clear_alarm_int_mask(struct bq2597x *bq, u8 mask) {
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0F, &val);
	if (ret)
		return ret;

	val &= ~mask;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_0F, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_clear_alarm_int_mask);

static int bq2597x_set_fault_int_mask(struct bq2597x *bq, u8 mask) {
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_12, &val);
	if (ret)
		return ret;

	val |= mask;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_12, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_set_fault_int_mask);

static int bq2597x_clear_fault_int_mask(struct bq2597x *bq, u8 mask) {
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_12, &val);
	if (ret)
		return ret;

	val &= ~mask;

	ret = bq2597x_write_byte(bq, BQ2597X_REG_12, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2597x_clear_fault_int_mask);

static int bq2597x_set_sense_resistor(struct bq2597x *bq, int r_mohm) {
	int ret;
	u8 val;

	if (r_mohm == 2)
		val = BQ2597X_SET_IBAT_SNS_RES_2MHM;
	else if (r_mohm == 5)
		val = BQ2597X_SET_IBAT_SNS_RES_5MHM;
	else
		return -EINVAL;

	val <<= BQ2597X_SET_IBAT_SNS_RES_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_2B,
	BQ2597X_SET_IBAT_SNS_RES_MASK, val);
	return ret;
}

static int bq2597x_enable_regulation(struct bq2597x *bq, bool enable) {
	int ret;
	u8 val;

	if (enable)
		val = BQ2597X_EN_REGULATION_ENABLE;
	else
		val = BQ2597X_EN_REGULATION_DISABLE;

	val <<= BQ2597X_EN_REGULATION_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_2B,
	BQ2597X_EN_REGULATION_MASK, val);

	return ret;

}

static int bq2597x_set_ss_timeout(struct bq2597x *bq, int timeout) {
	int ret;
	u8 val;

	switch (timeout) {
	case 0:
		val = BQ2597X_SS_TIMEOUT_DISABLE;
		break;
	case 12:
		val = BQ2597X_SS_TIMEOUT_12P5MS;
		break;
	case 25:
		val = BQ2597X_SS_TIMEOUT_25MS;
		break;
	case 50:
		val = BQ2597X_SS_TIMEOUT_50MS;
		break;
	case 100:
		val = BQ2597X_SS_TIMEOUT_100MS;
		break;
	case 400:
		val = BQ2597X_SS_TIMEOUT_400MS;
		break;
	case 1500:
		val = BQ2597X_SS_TIMEOUT_1500MS;
		break;
	case 100000:
		val = BQ2597X_SS_TIMEOUT_100000MS;
		break;
	default:
		val = BQ2597X_SS_TIMEOUT_DISABLE;
		break;
	}

	val <<= BQ2597X_SS_TIMEOUT_SET_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_2B,
	BQ2597X_SS_TIMEOUT_SET_MASK, val);

	return ret;
}

static int bq2597x_set_ibat_reg_th(struct bq2597x *bq, int th_ma) {
	int ret;
	u8 val;

	if (th_ma == 200)
		val = BQ2597X_IBAT_REG_200MA;
	else if (th_ma == 300)
		val = BQ2597X_IBAT_REG_300MA;
	else if (th_ma == 400)
		val = BQ2597X_IBAT_REG_400MA;
	else if (th_ma == 500)
		val = BQ2597X_IBAT_REG_500MA;
	else
		val = BQ2597X_IBAT_REG_500MA;

	val <<= BQ2597X_IBAT_REG_SHIFT;
	ret = bq2597x_update_bits(bq, BQ2597X_REG_2C,
	BQ2597X_IBAT_REG_MASK, val);

	return ret;

}

static int bq2597x_set_vbat_reg_th(struct bq2597x *bq, int th_mv) {
	int ret;
	u8 val;

	if (th_mv == 50)
		val = BQ2597X_VBAT_REG_50MV;
	else if (th_mv == 100)
		val = BQ2597X_VBAT_REG_100MV;
	else if (th_mv == 150)
		val = BQ2597X_VBAT_REG_150MV;
	else
		val = BQ2597X_VBAT_REG_200MV;

	val <<= BQ2597X_VBAT_REG_SHIFT;

	ret = bq2597x_update_bits(bq, BQ2597X_REG_2C,
	BQ2597X_VBAT_REG_MASK, val);

	return ret;
}

static int bq2597x_check_reg_status(struct bq2597x *bq) {
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_2C, &val);
	if (!ret) {
		bq->vbat_reg = !!(val & BQ2597X_VBAT_REG_ACTIVE_STAT_MASK);
		bq->ibat_reg = !!(val & BQ2597X_IBAT_REG_ACTIVE_STAT_MASK);
	}

	return ret;
}

static int bq2597x_get_work_mode(struct bq2597x *bq, int *mode) {
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0C, &val);

	if (ret) {
		bq_err("Failed to read operation mode register\n");
		return ret;
	}

	val = (val & BQ2597X_MS_MASK) >> BQ2597X_MS_SHIFT;
	if (val == BQ2597X_MS_MASTER)
		*mode = BQ25970_ROLE_MASTER;
	else if (val == BQ2597X_MS_SLAVE)
		*mode = BQ25970_ROLE_SLAVE;
	else
		*mode = BQ25970_ROLE_STDALONE;

	bq_info("work mode:%s\n",
			*mode == BQ25970_ROLE_STDALONE ? "Standalone" : (*mode == BQ25970_ROLE_SLAVE ? "Slave" : "Master"));
	return ret;
}

static int bq2597x_detect_device(struct bq2597x *bq) {
	int ret;
	u8 data;

	ret = bq2597x_read_byte(bq, BQ2597X_REG_13, &data);
	if (ret == 0) {
		bq->part_no = (data & BQ2597X_DEV_ID_MASK);
		bq->part_no >>= BQ2597X_DEV_ID_SHIFT;
	}

	return ret;
}

static int bq2597x_parse_dt(struct bq2597x *bq, struct device *dev) {
	int ret;
	struct device_node *np = dev->of_node;

	bq->cfg = devm_kzalloc(dev, sizeof(struct bq2597x_cfg),
	GFP_KERNEL);

	if (!bq->cfg)
		return -ENOMEM;

	bq->cfg->bat_ovp_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ovp-disable");
	bq->cfg->bat_ocp_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ocp-disable");
	bq->cfg->bat_ovp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ovp-alarm-disable");
	bq->cfg->bat_ocp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ocp-alarm-disable");
	bq->cfg->bus_ocp_disable = of_property_read_bool(np,
			"ti,bq2597x,bus-ocp-disable");
	bq->cfg->bus_ovp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bus-ovp-alarm-disable");
	bq->cfg->bus_ocp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bus-ocp-alarm-disable");
	bq->cfg->bat_ucp_alm_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-ucp-alarm-disable");
	bq->cfg->bat_therm_disable = of_property_read_bool(np,
			"ti,bq2597x,bat-therm-disable");
	bq->cfg->bus_therm_disable = of_property_read_bool(np,
			"ti,bq2597x,bus-therm-disable");
	bq->cfg->die_therm_disable = of_property_read_bool(np,
			"ti,bq2597x,die-therm-disable");

	ret = of_property_read_u32(np, "ti,bq2597x,bat-ovp-threshold",
			&bq->cfg->bat_ovp_th);
	if (ret) {
		bq_err("failed to read bat-ovp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-ovp-alarm-threshold",
			&bq->cfg->bat_ovp_alm_th);
	if (ret) {
		bq_err("failed to read bat-ovp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-ocp-threshold",
			&bq->cfg->bat_ocp_th);
	if (ret) {
		bq_err("failed to read bat-ocp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-ocp-alarm-threshold",
			&bq->cfg->bat_ocp_alm_th);
	if (ret) {
		bq_err("failed to read bat-ocp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-ovp-threshold",
			&bq->cfg->bus_ovp_th);
	if (ret) {
		bq_err("failed to read bus-ovp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-ovp-alarm-threshold",
			&bq->cfg->bus_ovp_alm_th);
	if (ret) {
		bq_err("failed to read bus-ovp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-ocp-threshold",
			&bq->cfg->bus_ocp_th);
	if (ret) {
		bq_err("failed to read bus-ocp-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-ocp-alarm-threshold",
			&bq->cfg->bus_ocp_alm_th);
	if (ret) {
		bq_err("failed to read bus-ocp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-ucp-alarm-threshold",
			&bq->cfg->bat_ucp_alm_th);
	if (ret) {
		bq_err("failed to read bat-ucp-alarm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bat-therm-threshold",
			&bq->cfg->bat_therm_th);
	if (ret) {
		bq_err("failed to read bat-therm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,bus-therm-threshold",
			&bq->cfg->bus_therm_th);
	if (ret) {
		bq_err("failed to read bus-therm-threshold\n");
		return ret;
	}
	ret = of_property_read_u32(np, "ti,bq2597x,die-therm-threshold",
			&bq->cfg->die_therm_th);
	if (ret) {
		bq_err("failed to read die-therm-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2597x,ac-ovp-threshold",
			&bq->cfg->ac_ovp_th);
	if (ret) {
		bq_err("failed to read ac-ovp-threshold\n");
		return ret;
	}

	ret = of_property_read_u32(np, "ti,bq2597x,sense-resistor-mohm",
			&bq->cfg->sense_r_mohm);
	if (ret) {
		bq_err("failed to read sense-resistor-mohm\n");
		return ret;
	}

	return 0;
}

static int bq2597x_init_protection(struct bq2597x *bq) {
	int ret;

	ret = bq2597x_enable_batovp(bq, !bq->cfg->bat_ovp_disable);
	bq_info("%s bat ovp %s\n", bq->cfg->bat_ovp_disable ? "disable" : "enable",
			!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_batocp(bq, !bq->cfg->bat_ocp_disable);
	bq_info("%s bat ocp %s\n", bq->cfg->bat_ocp_disable ? "disable" : "enable",
			!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_batovp_alarm(bq, !bq->cfg->bat_ovp_alm_disable);
	bq_info("%s bat ovp alarm %s\n",
			bq->cfg->bat_ovp_alm_disable ? "disable" : "enable",
			!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_batocp_alarm(bq, !bq->cfg->bat_ocp_alm_disable);
	bq_info("%s bat ocp alarm %s\n",
			bq->cfg->bat_ocp_alm_disable ? "disable" : "enable",
			!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_batucp_alarm(bq, !bq->cfg->bat_ucp_alm_disable);
	bq_info("%s bat ocp alarm %s\n",
			bq->cfg->bat_ucp_alm_disable ? "disable" : "enable",
			!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_busovp_alarm(bq, !bq->cfg->bus_ovp_alm_disable);
	bq_info("%s bus ovp alarm %s\n",
			bq->cfg->bus_ovp_alm_disable ? "disable" : "enable",
			!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_busocp(bq, !bq->cfg->bus_ocp_disable);
	bq_info("%s bus ocp %s\n", bq->cfg->bus_ocp_disable ? "disable" : "enable",
			!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_busocp_alarm(bq, !bq->cfg->bus_ocp_alm_disable);
	bq_info("%s bus ocp alarm %s\n",
			bq->cfg->bus_ocp_alm_disable ? "disable" : "enable",
			!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_bat_therm(bq, !bq->cfg->bat_therm_disable);
	bq_info("%s bat therm %s\n",
			bq->cfg->bat_therm_disable ? "disable" : "enable",
			!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_bus_therm(bq, !bq->cfg->bus_therm_disable);
	bq_info("%s bus therm %s\n",
			bq->cfg->bus_therm_disable ? "disable" : "enable",
			!ret ? "successfullly" : "failed");

	ret = bq2597x_enable_die_therm(bq, !bq->cfg->die_therm_disable);
	bq_info("%s die therm %s\n",
			bq->cfg->die_therm_disable ? "disable" : "enable",
			!ret ? "successfullly" : "failed");

	ret = bq2597x_set_batovp_th(bq, bq->cfg->bat_ovp_th);
	bq_info("set bat ovp th %d %s\n", bq->cfg->bat_ovp_th,
			!ret ? "successfully" : "failed");

	ret = bq2597x_set_batovp_alarm_th(bq, bq->cfg->bat_ovp_alm_th);
	bq_info("set bat ovp alarm threshold %d %s\n", bq->cfg->bat_ovp_alm_th,
			!ret ? "successfully" : "failed");

	ret = bq2597x_set_batocp_th(bq, bq->cfg->bat_ocp_th);
	bq_info("set bat ocp threshold %d %s\n", bq->cfg->bat_ocp_th,
			!ret ? "successfully" : "failed");

	ret = bq2597x_set_batocp_alarm_th(bq, bq->cfg->bat_ocp_alm_th);
	bq_info("set bat ocp alarm threshold %d %s\n", bq->cfg->bat_ocp_alm_th,
			!ret ? "successfully" : "failed");

	ret = bq2597x_set_busovp_th(bq, bq->cfg->bus_ovp_th);
	bq_info("set bus ovp threshold %d %s\n", bq->cfg->bus_ovp_th,
			!ret ? "successfully" : "failed");

	ret = bq2597x_set_busovp_alarm_th(bq, bq->cfg->bus_ovp_alm_th);
	bq_info("set bus ovp alarm threshold %d %s\n", bq->cfg->bus_ovp_alm_th,
			!ret ? "successfully" : "failed");

	ret = bq2597x_set_busocp_th(bq, bq->cfg->bus_ocp_th);
	bq_info("set bus ocp threshold %d %s\n", bq->cfg->bus_ocp_th,
			!ret ? "successfully" : "failed");

	ret = bq2597x_set_busocp_alarm_th(bq, bq->cfg->bus_ocp_alm_th);
	bq_info("set bus ocp alarm th %d %s\n", bq->cfg->bus_ocp_alm_th,
			!ret ? "successfully" : "failed");

	ret = bq2597x_set_batucp_alarm_th(bq, bq->cfg->bat_ucp_alm_th);
	bq_info("set bat ucp threshold %d %s\n", bq->cfg->bat_ucp_alm_th,
			!ret ? "successfully" : "failed");

	ret = bq2597x_set_bat_therm_th(bq, bq->cfg->bat_therm_th);
	bq_info("set die therm threshold %d %s\n", bq->cfg->bat_therm_th,
			!ret ? "successfully" : "failed");
	ret = bq2597x_set_bus_therm_th(bq, bq->cfg->bus_therm_th);
	bq_info("set bus therm threshold %d %s\n", bq->cfg->bus_therm_th,
			!ret ? "successfully" : "failed");
	ret = bq2597x_set_die_therm_th(bq, bq->cfg->die_therm_th);
	bq_info("set die therm threshold %d %s\n", bq->cfg->die_therm_th,
			!ret ? "successfully" : "failed");

	ret = bq2597x_set_acovp_th(bq, bq->cfg->ac_ovp_th);
	bq_info("set ac ovp threshold %d %s\n", bq->cfg->ac_ovp_th,
			!ret ? "successfully" : "failed");

	return 0;
}

static int bq2597x_init_adc(struct bq2597x *bq) {

	bq2597x_set_adc_scanrate(bq, false);
	bq2597x_set_adc_bits(bq, 13);
	bq2597x_set_adc_average(bq, true);
	bq2597x_set_adc_scan(bq, ADC_IBUS, true);
	bq2597x_set_adc_scan(bq, ADC_VBUS, true);
	bq2597x_set_adc_scan(bq, ADC_VOUT, false);
	bq2597x_set_adc_scan(bq, ADC_VBAT, true);
	bq2597x_set_adc_scan(bq, ADC_IBAT, true);
	bq2597x_set_adc_scan(bq, ADC_TBUS, true);
	bq2597x_set_adc_scan(bq, ADC_TBAT, true);
	bq2597x_set_adc_scan(bq, ADC_TDIE, true);
	bq2597x_set_adc_scan(bq, ADC_VAC, true);

	bq2597x_enable_adc(bq, true);

	return 0;
}

static int bq2597x_init_int_src(struct bq2597x *bq) {
	int ret;
	/*TODO:be careful ts bus and ts bat alarm bit mask is in
	 *	fault mask register, so you need call
	 *	bq2597x_set_fault_int_mask for tsbus and tsbat alarm
	 */
	ret = bq2597x_set_alarm_int_mask(bq, ADC_DONE
	/*			| BAT_UCP_ALARM */
	| BAT_OVP_ALARM);
	if (ret) {
		bq_err("failed to set alarm mask:%d\n", ret);
		return ret;
	}
	return ret;
}

static int bq2597x_init_regulation(struct bq2597x *bq) {
	bq2597x_set_ibat_reg_th(bq, 300);
	bq2597x_set_vbat_reg_th(bq, 100);
	bq2597x_set_vdrop_deglitch(bq, 5000);
	bq2597x_set_vdrop_th(bq, 400);
	bq2597x_enable_regulation(bq, true);

	return 0;
}

static int bq2597x_init_device(struct bq2597x *bq) {
	bq_err("bq2597x_charger_probe\n");
	bq2597x_enable_wdt(bq, false);
	bq2597x_set_ss_timeout(bq, 50);
	bq2597x_set_sense_resistor(bq, bq->cfg->sense_r_mohm);
	bq2597x_init_protection(bq);
	bq2597x_init_adc(bq);
	bq2597x_init_int_src(bq);
	bq2597x_init_regulation(bq);

	return 0;
}

static int bq2597x_set_present(struct bq2597x *bq, bool present) {
	bq->usb_present = present;

	if (present)
		bq2597x_init_device(bq);
	return 0;
}

static ssize_t bq2597x_show_registers(struct device *dev,
		struct device_attribute *attr, char *buf) {
	struct bq2597x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq25970");
	for (addr = 0x0; addr <= 0x2A; addr++) {
		ret = bq2597x_read_byte(bq, addr, &val);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx, "Reg[%.2X] = 0x%.2x\n",
					addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq2597x_store_register(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	struct bq2597x *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf, "%x %x", &reg, &val);
	if (ret == 2 && reg <= 0x2A)
		bq2597x_write_byte(bq, (unsigned char) reg, (unsigned char) val);

	return count;
}

static DEVICE_ATTR(registers, 0660, bq2597x_show_registers, bq2597x_store_register);

static struct attribute *bq2597x_attributes[] = { &dev_attr_registers.attr,
NULL, };

static const struct attribute_group bq2597x_attr_group = { .attrs =
		bq2597x_attributes, };

/*********************************************

 static enum power_supply_property bq2597x_charger_props[] = {
 POWER_SUPPLY_PROP_PRESENT,
 POWER_SUPPLY_PROP_CHARGING_ENABLED,
 POWER_SUPPLY_PROP_STATUS,

 POWER_SUPPLY_PROP_TI_BATTERY_PRESENT,
 POWER_SUPPLY_PROP_TI_VBUS_PRESENT,
 POWER_SUPPLY_PROP_TI_BATTERY_VOLTAGE,
 POWER_SUPPLY_PROP_TI_BATTERY_CURRENT,
 POWER_SUPPLY_PROP_TI_BATTERY_TEMPERATURE,
 POWER_SUPPLY_PROP_TI_BUS_VOLTAGE,
 POWER_SUPPLY_PROP_TI_BUS_CURRENT,
 POWER_SUPPLY_PROP_TI_BUS_TEMPERATURE,
 POWER_SUPPLY_PROP_TI_DIE_TEMPERATURE,
 POWER_SUPPLY_PROP_TI_ALARM_STATUS,
 POWER_SUPPLY_PROP_TI_FAULT_STATUS,
 POWER_SUPPLY_PROP_TI_REG_STATUS,

 };

 static void bq2597x_check_alarm_status(struct bq2597x *bq);
 static void bq2597x_check_fault_status(struct bq2597x *bq);

 static int bq2597x_charger_get_property(struct power_supply *psy,
 enum power_supply_property psp,
 union power_supply_propval *val)
 {
 struct bq2597x *bq = power_supply_get_drvdata(psy);
 int result;
 int ret;
 u8 reg_val;

 switch (psp) {
 case POWER_SUPPLY_PROP_CHARGING_ENABLED:
 bq2597x_check_charge_enabled(bq, &bq->charge_enabled);
 val->intval = bq->charge_enabled;
 break;
 case POWER_SUPPLY_PROP_STATUS:
 val->intval = 0;
 break;
 case POWER_SUPPLY_PROP_PRESENT:
 val->intval = bq->usb_present;
 break;
 case POWER_SUPPLY_PROP_TI_BATTERY_PRESENT:
 ret = bq2597x_read_byte(bq, BQ2597X_REG_0D, &reg_val);
 if (!ret)
 bq->batt_present  = !!(reg_val & VBAT_INSERT);
 val->intval = bq->batt_present;
 break;
 case POWER_SUPPLY_PROP_TI_VBUS_PRESENT:
 ret = bq2597x_read_byte(bq, BQ2597X_REG_0D, &reg_val);
 if (!ret)
 bq->vbus_present  = !!(reg_val & VBUS_INSERT);
 val->intval = bq->vbus_present;
 break;
 case POWER_SUPPLY_PROP_TI_BATTERY_VOLTAGE:
 ret = bq2597x_get_adc_data(bq, ADC_VBAT, &result);
 if (!ret)
 bq->vbat_volt = result;

 val->intval = bq->vbat_volt;
 break;
 case POWER_SUPPLY_PROP_TI_BATTERY_CURRENT:
 ret = bq2597x_get_adc_data(bq, ADC_IBAT, &result);
 if (!ret)
 bq->ibat_curr = result;

 val->intval = bq->ibat_curr;
 break;
 case POWER_SUPPLY_PROP_TI_BATTERY_TEMPERATURE:
 ret = bq2597x_get_adc_data(bq, ADC_TBAT, &result);
 if (!ret)
 bq->bat_temp = result;

 val->intval = bq->bat_temp;
 break;
 case POWER_SUPPLY_PROP_TI_BUS_VOLTAGE:
 ret = bq2597x_get_adc_data(bq, ADC_VBUS, &result);
 if (!ret)
 bq->vbus_volt = result;

 val->intval = bq->vbus_volt;
 break;
 case POWER_SUPPLY_PROP_TI_BUS_CURRENT:
 ret = bq2597x_get_adc_data(bq, ADC_IBUS, &result);
 if (!ret)
 bq->ibus_curr = result;

 val->intval = bq->ibus_curr;
 break;
 case POWER_SUPPLY_PROP_TI_BUS_TEMPERATURE:
 ret = bq2597x_get_adc_data(bq, ADC_TBUS, &result);
 if (!ret)
 bq->bus_temp = result;

 val->intval = bq->bus_temp;
 break;
 case POWER_SUPPLY_PROP_TI_DIE_TEMPERATURE:
 ret = bq2597x_get_adc_data(bq, ADC_TDIE, &result);
 if (!ret)
 bq->die_temp = result;

 val->intval = bq->die_temp;
 break;
 case POWER_SUPPLY_PROP_TI_ALARM_STATUS:

 bq2597x_check_alarm_status(bq);

 val->intval = ((bq->bat_ovp_alarm << BAT_OVP_ALARM_SHIFT)
 | (bq->bat_ocp_alarm << BAT_OCP_ALARM_SHIFT)
 | (bq->bat_ucp_alarm << BAT_UCP_ALARM_SHIFT)
 | (bq->bus_ovp_alarm << BUS_OVP_ALARM_SHIFT)
 | (bq->bus_ocp_alarm << BUS_OCP_ALARM_SHIFT)
 | (bq->bat_therm_alarm << BAT_THERM_ALARM_SHIFT)
 | (bq->bus_therm_alarm << BUS_THERM_ALARM_SHIFT)
 | (bq->die_therm_alarm << DIE_THERM_ALARM_SHIFT));
 break;

 case POWER_SUPPLY_PROP_TI_FAULT_STATUS:
 bq2597x_check_fault_status(bq);

 val->intval = ((bq->bat_ovp_fault << BAT_OVP_FAULT_SHIFT)
 | (bq->bat_ocp_fault << BAT_OCP_FAULT_SHIFT)
 | (bq->bus_ovp_fault << BUS_OVP_FAULT_SHIFT)
 | (bq->bus_ocp_fault << BUS_OCP_FAULT_SHIFT)
 | (bq->bat_therm_fault << BAT_THERM_FAULT_SHIFT)
 | (bq->bus_therm_fault << BUS_THERM_FAULT_SHIFT)
 | (bq->die_therm_fault << DIE_THERM_FAULT_SHIFT));
 break;

 case POWER_SUPPLY_PROP_TI_REG_STATUS:
 bq2597x_check_reg_status(bq);
 val->intval = (bq->vbat_reg << VBAT_REG_STATUS_SHIFT) |
 (bq->ibat_reg << IBAT_REG_STATUS_SHIFT);
 break;
 default:
 return -EINVAL;

 }

 return 0;
 }

 static int bq2597x_charger_set_property(struct power_supply *psy,
 enum power_supply_property prop,
 const union power_supply_propval *val)
 {
 struct bq2597x *bq = power_supply_get_drvdata(psy);

 switch (prop) {
 case POWER_SUPPLY_PROP_CHARGING_ENABLED:
 bq2597x_enable_charge(bq, val->intval);
 bq2597x_check_charge_enabled(bq, &bq->charge_enabled);
 bq_info("POWER_SUPPLY_PROP_CHARGING_ENABLED: %s\n",
 val->intval ? "enable" : "disable");
 break;
 case POWER_SUPPLY_PROP_PRESENT:
 bq2597x_set_present(bq, !!val->intval);
 break;
 default:
 return -EINVAL;
 }

 return 0;
 }


 static int bq2597x_charger_is_writeable(struct power_supply *psy,
 enum power_supply_property prop)
 {
 int ret;

 switch (prop) {
 case POWER_SUPPLY_PROP_CHARGING_ENABLED:
 ret = 1;
 break;
 default:
 ret = 0;
 break;
 }
 return ret;
 }

 static int bq2597x_psy_register(struct bq2597x *bq)
 {
 int ret;

 bq->psy_cfg.drv_data = bq;
 bq->psy_cfg.of_node = bq->dev->of_node;


 if (bq->mode == BQ25970_ROLE_MASTER)
 bq->psy_desc.name = "bq2597x-master";
 else if (bq->mode == BQ25970_ROLE_SLAVE)
 bq->psy_desc.name = "bq2597x-slave";
 else
 bq->psy_desc.name = "bq2597x-standalone";

 bq->psy_desc.type = POWER_SUPPLY_TYPE_MAINS;
 bq->psy_desc.properties = bq2597x_charger_props;
 bq->psy_desc.num_properties = ARRAY_SIZE(bq2597x_charger_props);
 bq->psy_desc.get_property = bq2597x_charger_get_property;
 bq->psy_desc.set_property = bq2597x_charger_set_property;
 bq->psy_desc.property_is_writeable = bq2597x_charger_is_writeable;


 bq->fc2_psy = devm_power_supply_register(bq->dev,
 &bq->psy_desc, &bq->psy_cfg);
 if (IS_ERR(bq->fc2_psy)) {
 bq_err("failed to register fc2_psy:%d\n", ret);
 return PTR_ERR(bq->fc2_psy);
 }

 bq_info("%s power supply register successfully\n", bq->psy_desc.name);

 return 0;
 }
 *********************************************/
void bq2597x_dump_status_reg(void) {

	int ret;
	u8 val;
	u8 addr;
	struct bq2597x *bq = charger_ic;
	bq_err("bq2597x bq2597x_dump_reg\n!");

	if (!bq)
		return;
	for (addr = 0x13; addr < 0x1d; addr++) {
		ret = bq2597x_read_byte(bq, addr, &val);
		if (!ret)
			bq_err("Reg[%02X] = 0x%02X\n", addr, val);
	}

}

void bq2597x_dump_reg(void) {

	int ret;
	u8 val;
	u8 addr;
	struct bq2597x *bq = charger_ic;
	bq_err("bq2597x bq2597x_dump_reg\n!");

	if (!bq)
		return;
	for (addr = 0x00; addr < 0x37; addr++) {
		ret = bq2597x_read_byte(bq, addr, &val);
		if (!ret)
			bq_err("Reg[%02X] = 0x%02X\n", addr, val);
	}

}
EXPORT_SYMBOL_GPL(bq2597x_dump_reg);

static void bq2597x_check_alarm_status(struct bq2597x *bq) {
	int ret;
	u8 flag = 0;
	u8 stat = 0;

	mutex_lock(&bq->data_lock);

	ret = bq2597x_read_byte(bq, BQ2597X_REG_08, &flag);
	if (!ret && (flag & BQ2597X_IBUS_UCP_FALL_FLAG_MASK))
		bq_dbg("UCP_FLAG =0x%02X\n", !!(flag & BQ2597X_IBUS_UCP_FALL_FLAG_MASK));

	ret = bq2597x_read_byte(bq, BQ2597X_REG_2D, &flag);
	if (!ret && (flag & BQ2597X_VDROP_OVP_FLAG_MASK))
		bq_dbg("VDROP_OVP_FLAG =0x%02X\n",
				!!(flag & BQ2597X_VDROP_OVP_FLAG_MASK));

	/*read to clear alarm flag*/
	ret = bq2597x_read_byte(bq, BQ2597X_REG_0E, &flag);
	if (!ret && flag)
		bq_dbg("INT_FLAG =0x%02X\n", flag);

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0D, &stat);
	if (!ret && stat != bq->prev_alarm) {
		bq_dbg("INT_STAT = 0X%02x\n", stat);
		bq->prev_alarm = stat;
		bq->bat_ovp_alarm = !!(stat & BAT_OVP_ALARM);
		bq->bat_ocp_alarm = !!(stat & BAT_OCP_ALARM);
		bq->bus_ovp_alarm = !!(stat & BUS_OVP_ALARM);
		bq->bus_ocp_alarm = !!(stat & BUS_OCP_ALARM);
		bq->batt_present = !!(stat & VBAT_INSERT);
		bq->vbus_present = !!(stat & VBUS_INSERT);
		bq->bat_ucp_alarm = !!(stat & BAT_UCP_ALARM);
	}

	ret = bq2597x_read_byte(bq, BQ2597X_REG_08, &stat);
	if (!ret && (stat & 0x50))
		bq_err("Reg[05]BUS_UCPOVP = 0x%02X\n", stat);

	ret = bq2597x_read_byte(bq, BQ2597X_REG_0A, &stat);
	if (!ret && (stat & 0x02))
		bq_err("Reg[0A]CONV_OCP = 0x%02X\n", stat);

	mutex_unlock(&bq->data_lock);
}

static void bq2597x_check_fault_status(struct bq2597x *bq) {
	int ret;
	u8 flag = 0;
	u8 stat = 0;
	bool changed = false;

	mutex_lock(&bq->data_lock);

	ret = bq2597x_read_byte(bq, BQ2597X_REG_10, &stat);
	if (!ret && stat)
		bq_err("FAULT_STAT = 0x%02X\n", stat);

	ret = bq2597x_read_byte(bq, BQ2597X_REG_11, &flag);
	if (!ret && flag)
		bq_err("FAULT_FLAG = 0x%02X\n", flag);

	if (!ret && flag != bq->prev_fault) {
		changed = true;
		bq->prev_fault = flag;
		bq->bat_ovp_fault = !!(flag & BAT_OVP_FAULT);
		bq->bat_ocp_fault = !!(flag & BAT_OCP_FAULT);
		bq->bus_ovp_fault = !!(flag & BUS_OVP_FAULT);
		bq->bus_ocp_fault = !!(flag & BUS_OCP_FAULT);
		bq->bat_therm_fault = !!(flag & TS_BAT_FAULT);
		bq->bus_therm_fault = !!(flag & TS_BUS_FAULT);

		bq->bat_therm_alarm = !!(flag & TBUS_TBAT_ALARM);
		bq->bus_therm_alarm = !!(flag & TBUS_TBAT_ALARM);
	}

	mutex_unlock(&bq->data_lock);
}

/*
 * interrupt does nothing, just info event chagne, other module could get info
 * through power supply interface
 */
static irqreturn_t bq2597x_charger_interrupt(int irq, void *dev_id) {
	struct bq2597x *bq = dev_id;

	bq_dbg("INT OCCURED\n");
	power_supply_changed(bq->fc2_psy);

	return IRQ_HANDLED;
}

static void determine_initial_status(struct bq2597x *bq) {
	if (bq->client->irq)
		bq2597x_charger_interrupt(bq->client->irq, bq);
}

static int show_registers(struct seq_file *m, void *data) {
	struct bq2597x *bq = m->private;
	u8 addr;
	int ret;
	u8 val;

	for (addr = 0x0; addr <= 0x2B; addr++) {
		ret = bq2597x_read_byte(bq, addr, &val);
		if (!ret)
			seq_printf(m, "Reg[%02X] = 0x%02X\n", addr, val);
	}
	return 0;
}

static int reg_debugfs_open(struct inode *inode, struct file *file) {
	struct bq2597x *bq = inode->i_private;

	return single_open(file, show_registers, bq);
}

static const struct file_operations reg_debugfs_ops = { .owner = THIS_MODULE,
		.open = reg_debugfs_open, .read = seq_read, .llseek = seq_lseek,
		.release = single_release, };

static void create_debugfs_entry(struct bq2597x *bq) {
	if (bq->mode == BQ25970_ROLE_MASTER)
		bq->debug_root = debugfs_create_dir("bq2597x-master", NULL);
	else if (bq->mode == BQ25970_ROLE_SLAVE)
		bq->debug_root = debugfs_create_dir("bq2597x-slave", NULL);
	else
		bq->debug_root = debugfs_create_dir("bq2597x-standalone", NULL);

	if (!bq->debug_root)
		bq_err("Failed to create debug dir\n");

	if (bq->debug_root) {
		debugfs_create_file("registers",
		S_IFREG | S_IRUGO, bq->debug_root, bq, &reg_debugfs_ops);

		debugfs_create_x32("skip_reads",
		S_IFREG | S_IWUSR | S_IRUGO, bq->debug_root, &(bq->skip_reads));
		debugfs_create_x32("skip_writes",
		S_IFREG | S_IWUSR | S_IRUGO, bq->debug_root, &(bq->skip_writes));
	}
}

void voov_enable_bq2597x(void) {
	struct bq2597x *bq = charger_ic;
	int ret;
	u8 val;

	ret = bq2597x_read_byte(bq, 0x0f, &val);
	val |= (1 << 4);
	ret = bq2597x_write_byte(bq, 0x0f, val);
	ret = bq2597x_read_byte(bq, 0x0f, &val);

	bq_err(
			"lkl voov_enable_bq2597x----------------------------- READ 0X0C = 0x%x\n",
			val);

	return;
}

void voov_disable_bq2597x(void) {
	struct bq2597x *bq = charger_ic;

	bq2597x_enable_charge(bq, false);

	return;
}

int bq2597x_ibus_adc1_read(void) {
	u8 val = 0;
	return 0;


	return val;
}

int bq2597x_ibus_adc0_read(void) {
	u8 val = 0;
	return 0;

	return val;
}

static struct of_device_id bq2597x_charger_match_table[] = {
		{ .compatible = "oplus,bq2597x-standalone", .data =
				&bq2597x_mode_data[BQ25970_STDALONE], }, { .compatible =
				"oplus,bq2597x-master", .data =
				&bq2597x_mode_data[BQ25970_MASTER], },

		{ .compatible = "oplus,bq2597x-slave", .data =
				&bq2597x_mode_data[BQ25970_SLAVE], }, { }, };

static void oplus_cp_lc_func(struct work_struct *work) {
	struct bq2597x *bq = charger_ic;
	u8 stat = 0;
	int ret;
	int i = 0;
	ret = bq2597x_read_byte(bq, BQ2597X_REG_08, &stat);
	if (!ret)
		bq_err("Reg[08]BQ2597X_REG_08 = 0x%02X\n", stat);
	for (i = 0; i < 20; i++) {
		if ((stat & BQ2597X_IBUS_UCP_FALL_FLAG_MASK)) {
			voov_enable_bq2597x();
			bq_err("Reg[05]BUS_UCPOVP = 0x%02X, i = %d\n", stat, i);

		}
		ret = bq2597x_read_byte(bq, BQ2597X_REG_08, &stat);
		if (!ret)
			bq_err("Reg[08]BQ2597X_REG_08 = 0x%02X\n", stat);

	}
	bq2597x_dump_reg();
	chg_err("irq_rx_handler------gpio_get_value cp_lc_gpio:%d\n",
			gpio_get_value(bq->cp_lc_gpio));
}

static irqreturn_t irq_rx_handler(int irq, void *dev_id) {
	struct bq2597x *bq = charger_ic;
	if (0)
		schedule_delayed_work(&bq->cp_lc_work, 0);

	return IRQ_HANDLED;
}

void oplus_cp_low_current_irq_init(struct bq2597x *chip) {

#ifdef CONFIG_OPLUS_CHARGER_MTK
	struct device_node *node = NULL;
	struct device_node *node_new = NULL;
	u32 intr[2] = {0, 0};

	node = of_find_compatible_node(NULL, NULL, "mediatek, CP_LOW_CURRENT-eint");
	node_new = of_find_compatible_node(NULL, NULL, "mediatek,  CP_LOW_CURRENT_EINT_NEW_FUNCTION");
	if (node) {
		if (node_new) {
			chip->cp_lc_irq = gpio_to_irq(chip->cp_lc_gpio);
			chg_err("cp_lc_irq:%d\n", chip->cp_lc_irq);
		}
	}
#else
	chip->cp_lc_irq = gpio_to_irq(chip->cp_lc_gpio);
#endif
}

void opchg_set_cp_low_active(struct bq2597x *chip) {
	mutex_lock(&chip->pinctrl_mutex);
	gpio_direction_input(chip->cp_lc_gpio); /* in */
	mutex_unlock(&chip->pinctrl_mutex);
}

void oplus_cp_low_current_eint_register(struct bq2597x *chip) {
	int ret = 0;
	struct device_node *node = NULL;
	node = of_find_compatible_node(NULL, NULL,
			"mediatek,  CP_LOW_CURRENT_EINT_NEW_FUNCTION");
	if (node) {
		opchg_set_cp_low_active(chip);
		ret = request_irq(chip->cp_lc_irq, (irq_handler_t) irq_rx_handler,
		IRQF_TRIGGER_FALLING, "CP_LOW_CURRENT-eint", chip);
		if (ret < 0) {
			chg_err("Unable to request ccdetect-change irq: %d\n", ret);
		}

		if (ret < 0) {
			chg_err(
					"ret = %d, oplus_cp_low_current_eint_register failed to request_irq \n",
					ret);
		} else
			chg_err(
					"ret = %d, oplus_cp_low_current_eint_register OK to request_irq \n",
					ret);

	}

}

void oplus_cp_low_current_eint_unregister(struct bq2597x *chip) {
#ifdef CONFIG_OPLUS_CHARGER_MTK
	struct device_node *node = NULL;
	node = of_find_compatible_node(NULL, NULL, "mediatek,  CP_LOW_CURRENT_EINT_NEW_FUNCTION");
	chg_err("disable_irq_mtk!\r\n");
	if (node) {
		free_irq(chip->cp_lc_gpio, chip);
	} else {
		disable_irq(chip->cp_lc_gpio);
	}
#else
	free_irq(chip->cp_lc_gpio, chip);
#endif
}

extern void bq2597x_hardware_init(void);

extern void voov_enable_bq2597x(void);

extern void voov_disable_bq2597x(void);
extern void bq2597x_dump_reg(void);
extern int bq2597x_hardware_init_svooc(void);
extern int bq2597x_hardware_init_normal_vooc(void);
extern int oplus_bq2597x_reset_cp(void);

struct oplus_vooc_cp oplus_rkbq2597x_ops = {
		.hardware_init_cp =	bq2597x_hardware_init,
		.vooc_enable_cp = voov_enable_bq2597x,
		.vooc_disable_cp = voov_disable_bq2597x,
		.cp_dump_reg = bq2597x_dump_status_reg,
		.cp_hardware_init_svooc = bq2597x_hardware_init_svooc,
		.cp_hardware_init_vooc = bq2597x_hardware_init_normal_vooc,
		.cp_hardware_init_pdqc = bq2597x_hardware_init_pd_qc,
		.oplus_reset_cp = oplus_bq2597x_reset_cp,
		.enalbe_ovp = oplus_bq2597x_ovp_en,
		.enable_cp_for_otg = oplus_bq2597x_otg_en
};

static int bq2597x_charger_probe(struct i2c_client *client,
		const struct i2c_device_id *id) {
	struct bq2597x *bq;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;
	int ret;
	int rc;

	bq_err("bq2597x_charger_probe------------resume dump\n");
	bq = devm_kzalloc(&client->dev, sizeof(struct bq2597x), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->dev = &client->dev;

	bq->client = client;
	i2c_set_clientdata(client, bq);
	charger_ic = bq;

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->irq_complete);
	mutex_init(&bq->pinctrl_mutex);

	bq->resume_completed = true;
	bq->irq_waiting = false;
	INIT_DELAYED_WORK(&bq->cp_lc_work, oplus_cp_lc_func);

	ret = bq2597x_detect_device(bq);
	if (ret) {
		bq_err("No bq2597x device found!\n");
		return -ENODEV;
	}

	match = of_match_node(bq2597x_charger_match_table, node);
	if (match == NULL) {
		bq_err("device tree match not found!\n");
		return -ENODEV;
	}

	bq2597x_get_work_mode(bq, &bq->mode);

	if (bq->mode != *(int *) match->data) {
		bq_err("device operation mode mismatch with dts configuration\n");
		return -EINVAL;
	}

	bq->cp_lc_gpio = of_get_named_gpio(node, "qcom,cp_low_current", 0);
	if (bq->cp_lc_gpio < 0) {
		chg_err("chip->cp_lc_gpio not specified\n");
	} else {
		if (gpio_is_valid(bq->cp_lc_gpio)) {
			rc = gpio_request(bq->cp_lc_gpio, "cp_low_current");
			if (rc) {
				chg_err("unable to request gpio [%d]\n", bq->cp_lc_gpio);
			}
		}
		chg_err("bq->cp_lc_gpio =%d\n", bq->cp_lc_gpio);
	}

	bq->cp_en = of_get_named_gpio(node, "qcom,en_pin", 0);
	if (bq->cp_en < 0) {
		chg_err("chip->cp_en not specified\n");
	} else {
		if (gpio_is_valid(bq->cp_en)) {
			rc = gpio_request(bq->cp_en, "cp_en");
			if (rc) {
				chg_err("unable to request gpio [%d]\n", bq->cp_en);
			} else {
				chg_err("enabled cp_en input [%d]\n", bq->cp_en);
				gpio_direction_input(bq->cp_en);
			}
		}
		chg_err("bq->cp_en =%d\n", bq->cp_en);
	}

	oplus_cp_low_current_irq_init(bq);
	oplus_cp_low_current_eint_register(bq);

/*
	ret = bq2597x_psy_register(bq);
	if (ret)
		return ret;
*/
	if (0) {
		ret = bq2597x_parse_dt(bq, &client->dev);
		if (0)
			return -EIO;

		ret = bq2597x_init_device(bq);
		if (0) {
			bq_err("Failed to init device\n");
			return ret;
		}
		ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, bq2597x_charger_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq2597x charger irq", bq);
		if (ret < 0) {
			bq_err("request irq for irq=%d failed, ret =%d\n",
							client->irq, ret);
			goto err_1;
		}
		enable_irq_wake(client->irq);
		device_init_wakeup(bq->dev, 1);
		create_debugfs_entry(bq);
		ret = sysfs_create_group(&bq->dev->kobj, &bq2597x_attr_group);
		if (ret) {
			bq_err("failed to register sysfs. err: %d\n", ret);
			goto err_1;
		}
		if(0) {
			bq2597x_check_charge_enabled(bq, &bq->charge_enabled);
			bq2597x_check_reg_status(bq);
			bq2597x_set_present(bq, false);
			bq2597x_check_alarm_status(bq);
			bq2597x_check_fault_status(bq);
		}

		determine_initial_status(bq);
	}

	voov_disable_bq2597x();

	oplus_vooc_init_cp(&oplus_rkbq2597x_ops);
	bq_err("bq2597x probe successfully, Part Num:%d\n!", bq->part_no);
	bq2597x_dump_reg();

	return 0;

err_1:
	return ret;
}

static inline bool is_device_suspended(struct bq2597x *bq) {
	return !bq->resume_completed;
}

static int bq2597x_suspend(struct device *dev) {
	return 0;
}

static int bq2597x_suspend_noirq(struct device *dev) {
	return 0;
}

static int bq2597x_resume(struct device *dev) {
	return 0;
}
static int bq2597x_charger_remove(struct i2c_client *client) {
	return 0;
}

static void bq2597x_charger_shutdown(struct i2c_client *client) {
}

static const struct dev_pm_ops bq2597x_pm_ops = { .resume = bq2597x_resume,
		.suspend_noirq = bq2597x_suspend_noirq, .suspend = bq2597x_suspend, };

static const struct i2c_device_id bq2597x_charger_id[] = { {
		"bq2597x-standalone", BQ25970_ROLE_STDALONE }, { "bq2597x-master",
BQ25970_ROLE_MASTER }, { "bq2597x-slave", BQ25970_ROLE_SLAVE }, { }, };

static struct i2c_driver bq2597x_charger_driver = { .driver = { .name =
		"bq2597x-charger", .owner = THIS_MODULE, .of_match_table =
		bq2597x_charger_match_table, .pm = &bq2597x_pm_ops, }, .id_table =
		bq2597x_charger_id,

.probe = bq2597x_charger_probe, .remove = bq2597x_charger_remove, .shutdown =
		bq2597x_charger_shutdown, };

module_i2c_driver( bq2597x_charger_driver);

MODULE_DESCRIPTION("OPLUS BQ2597x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");

