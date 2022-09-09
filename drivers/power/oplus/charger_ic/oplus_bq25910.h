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
************************************************************************************************************/


#ifndef __OPLUS_BQ25910_H__

#define __OPLUS_BQ25910_H__

#define OPLUS_BQ25910

#include <linux/power_supply.h>
#ifdef CONFIG_OPLUS_CHARGER_MTK
#include <mt-plat/charger_type.h>
#else /* CONFIG_OPLUS_CHARGER_MTK */

#endif /* CONFIG_OPLUS_CHARGER_MTK */
#include "../oplus_charger.h"

#define WPC_CHARGE_CURRENT_DEFAULT							500		//500mA
#define WPC_TERMINATION_VOLTAGE_DEFAULT					4370
#define WPC_TERMINATION_CURRENT								200
#define WPC_TERMINATION_VOLTAGE								WPC_TERMINATION_VOLTAGE_DEFAULT
#define WPC_RECHARGE_VOLTAGE_OFFSET							200
#define WPC_PRECHARGE_CURRENT								480
#define WPC_CHARGER_INPUT_CURRENT_LIMIT_DEFAULT			1000

#define BQ25910_FIRST_REG										0x00
#define BQ25910_DUMP_MAX_REG									0x0D
#define BQ25910_LAST_REG										0x0D
#define BQ25910_REG_NUMBER									0x0E


/* Address:00h */
#define REG00_BQ25910_ADDRESS									0x00
#define REG00_BQ25910_CHARGE_FULL_VOL_MASK					(BIT(7) | BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) |BIT(0))
#define REG00_BQ25910_CHARGE_FULL_VOL_SHIFT					0
#define REG00_BQ25910_CHARGE_FULL_VOL_OFFSET				3500
#define REG00_BQ25910_CHARGE_FULL_VOL_STEP					5   //5mV,  default 4.35V

/* Address:01h */
#define REG01_BQ25910_ADDRESS									0x01

#define REG01_BQ25910_CHARGE_CURRENT_SETTING_MASK			(BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG01_BQ25910_CHARGE_CURRENT_SETTING_SHIFT		0
#define REG01_BQ25910_CHARGE_CURRENT_SETTING_OFFSET		0
#define REG01_BQ25910_CHARGE_CURRENT_SETTING_STEP			50    //default 3.5A


/* Address:02h */
#define REG02_BQ25910_ADDRESS                                   			0x02

#define REG02_BQ25910_VINDPM_THRESHOLD_MASK				(BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG02_BQ25910_VINDPM_THRESHOLD_SHIFT				0
#define REG02_BQ25910_VINDPM_THRESHOLD_OFFSET				3900
#define REG02_BQ25910_VINDPM_THRESHOLD_STEP				100    //default 4.3V

/* Address:03h */
#define REG03_BQ25910_ADDRESS									0x03

#define REG03_BQ25910_CURRENT_LIMIT_MASK					(BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG03_BQ25910_CURRENT_LIMIT_SHIFT					0
#define REG03_BQ25910_CURRENT_LIMIT_OFFSET					500
#define REG03_BQ25910_CURRENT_LIMIT_STEP					100    //default 2.4A
#define REG03_BQ25910_CURRENT_LIMIT_500MA					(BIT(2) | BIT(0))
#define REG03_BQ25910_CURRENT_LIMIT_900MA					(BIT(3) | BIT(1))
#define REG03_BQ25910_CURRENT_LIMIT_1200MA					(BIT(3) | BIT(2))
#define REG03_BQ25910_CURRENT_LIMIT_1500MA					(BIT(3) | BIT(2) |BIT(1) | BIT(0))
#define REG03_BQ25910_CURRENT_LIMIT_1700MA					(BIT(4) | BIT(0))
#define REG03_BQ25910_CURRENT_LIMIT_2000MA					(BIT(4) | BIT(2))
#define REG03_BQ25910_CURRENT_LIMIT_3000MA					(BIT(4) |BIT(3) |BIT(2) |BIT(1))


/* Address:05h */
#define REG05_BQ25910_ADDRESS									0x05

#define REG05_BQ25910_TIM2X_EN_MASK							BIT(0)
#define REG05_BQ25910_TIM2X_EN_DISABLE						0        //default  
#define REG05_BQ25910_TIM2X_EN_ENABLE						BIT(0)

#define REG05_BQ25910_FAST_CHARGE_TIMER_MASK				(BIT(2) | BIT(1))
#define REG05_BQ25910_FAST_CHARGE_TIMER_5H					0
#define REG05_BQ25910_FAST_CHARGE_TIMER_8H					BIT(1)
#define REG05_BQ25910_FAST_CHARGE_TIMER_12H					BIT(2)    //default
#define REG05_BQ25910_FAST_CHARGE_TIMER_20H					(BIT(2) | BIT(1))

#define REG05_BQ25910_CHARGE_TIMER_EN_MASK					BIT(3)
#define REG05_BQ25910_CHARGE_TIMER_EN_DISABLE				0    
#define REG05_BQ25910_CHARGE_TIMER_EN_ENABLE				BIT(3)    //default  

#define REG05_BQ25910_WTD_TIMER_MASK						(BIT(5) | BIT(4))
#define REG05_BQ25910_WTD_TIMER_DISABLE						0        //default
#define REG05_BQ25910_WTD_TIMER_40S							BIT(4)
#define REG05_BQ25910_WTD_TIMER_80S							BIT(5)
#define REG05_BQ25910_WTD_TIMER_160S						(BIT(5) | BIT(4))

#define REG05_BQ25910_WTD_RST_MASK							BIT(6)
#define REG05_BQ25910_WTD_RST_NORMAL						0    
#define REG05_BQ25910_WTD_RST_RESET							BIT(6)    //default  

#define REG05_BQ25910_CHARGE_TERMINATION_EN_MASK			BIT(7)
#define REG05_BQ25910_CHARGE_TERMINATION_EN_DISABLE		0    
#define REG05_BQ25910_CHARGE_TERMINATION_EN_ENABLE		BIT(7)    //default  


/* Address:06h */
#define REG06_BQ25910_ADDRESS										0x06

#define REG06_BQ25910_THERMAL_REGULATION_THRESHOLD_MASK		(BIT(5) | BIT(4))
#define REG06_BQ25910_THERMAL_REGULATION_THRESHOLD_SHIFT		4
#define REG06_BQ25910_THERMAL_REGULATION_THRESHOLD_OFFSET	60
#define REG06_BQ25910_THERMAL_REGULATION_THRESHOLD_STEP		20    //default 120

#define REG06_BQ25910_CHG_EN_MASK								BIT(3)
#define REG06_BQ25910_CHG_EN_DISABLE								0       
#define REG06_BQ25910_CHG_EN_ENABLE								BIT(3)    //default

#define REG06_BQ25910_PRECHARGE_THRESHOLD_MASK				(BIT(1) | BIT(0))
#define REG07_BQ25910_PRECHARGE_THRESHOLD_2600MV				0
#define REG07_BQ25910_PRECHARGE_THRESHOLD_2900MV				BIT(0)//default
#define REG07_BQ25910_PRECHARGE_THRESHOLD_3200MV				BIT(1)
#define REG07_BQ25910_PRECHARGE_THRESHOLD_3500MV				(BIT(1) | BIT(0))

/* Address:09h */
#define REG09_BQ25910_ADDRESS										0x09

#define REG09_BQ25910_CHRG_TERM_FLAG_MASK						(BIT(2))
#define REG09_BQ25910_CHARGING_STATUS_CHARGE_TERMINATION		(BIT(2))

/* Address:0Dh */
#define REG0D_BQ25910_ADDRESS							0x0D

#define REG0D_BQ25910_REG_RST_MASK					BIT(7)
#define REG0D_BQ25910_REG_RST_KEEP					0        //default    
#define REG0D_BQ25910_REG_RST_RESET					BIT(7)


enum {
	BQ_OVERTIME_AC = 0,
	BQ_OVERTIME_USB,
	BQ_OVERTIME_DISABLED,
};


struct chip_bq25910 {
        struct i2c_client           *client;
        struct device               *dev;
        int                         hw_aicl_point;
        int                         sw_aicl_point;
        int         pre_current_ma;

        struct pinctrl                       *pinctrl;
        
        int         mps_otg_en_gpio;
        struct pinctrl_state    *mps_otg_en_active;
        struct pinctrl_state    *mps_otg_en_sleep;
        struct pinctrl_state    *mps_otg_en_default;

        atomic_t                    charger_suspended;

#ifdef CONFIG_OPLUS_CHARGER_MTK6853
		int slave_chg_en_gpio;
		struct pinctrl_state *slave_charger_enable;
		struct pinctrl_state *slave_charger_disable;
#endif

};

struct oplus_chg_operations *  oplus_get_chg_ops(void);
bool oplus_charger_ic_chip_is_null(void);
extern int bq25910_input_current_limit_write(int value);
extern int bq25910_charging_current_write_fast(int chg_cur);
extern int bq25910_set_vindpm_vol(int vol);
extern void bq25910_set_aicl_point(int vbatt);
extern int bq25910_set_enable_volatile_writes(void);
extern int bq25910_set_complete_charge_timeout(int val);
extern int bq25910_float_voltage_write(int vfloat_mv);
extern int bq25910_set_prechg_current(int ipre_mA);
extern int bq25910_set_termchg_current(int term_curr);
extern int bq25910_set_rechg_voltage(int recharge_mv);
extern int bq25910_set_wdt_timer(int reg);
extern int bq25910_set_chging_term_disable(void);
extern int bq25910_kick_wdt(void);
extern int bq25910_enable_charging(void);
extern int bq25910_disable_charging(void);
extern int bq25910_check_charging_enable(void);
extern int bq25910_check_learn_mode(void);
extern int bq25910_registers_read_full(void);
extern int bq25910_suspend_charger(void);
extern int bq25910_unsuspend_charger(void);
extern bool bq25910_check_charger_resume(void);
extern int bq25910_reset_charger_for_wired_charge(void);
extern int bq25910_otg_enable(void);
extern int bq25910_otg_disable(void);
extern void bq25910_dump_registers(void);
extern int bq25910_hardware_init(void);
extern bool bq25910_is_detected(void);
#ifdef CONFIG_OPLUS_CHARGER_MTK

#else /* CONFIG_OPLUS_CHARGER_MTK */

#endif/* CONFIG_OPLUS_CHARGER_MTK */
#endif
