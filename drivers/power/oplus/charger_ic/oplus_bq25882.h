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
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2015-06-22    Fanhong.Kong@ProDrv.CHG   		Created for new architecture
************************************************************************************************************/


#ifndef __OPLUS_BQ25882_H__

#define __OPLUS_BQ25882_H__

#include <linux/power_supply.h>
#ifdef CONFIG_OPLUS_CHARGER_MTK
//#include <mt-plat/charging.h>
//#include <mt-plat/battery_meter.h>
#include <mt-plat/charger_type.h>

#else /* CONFIG_OPLUS_CHARGER_MTK */

#endif /* CONFIG_OPLUS_CHARGER_MTK */

#include "../oplus_charger.h"


#define BQ25882_FIRST_REG										0x00
#define BQ25882_LAST_REG										0x25
#define BQ25882_REG_NUMBER										0x26


/* Address:00h */
#define REG00_BQ25882_ADDRESS                                   0x00

#define REG00_BQ25882_BAT_THRESHOLD_MASK                        (BIT(7) | BIT(6) | BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG00_BQ25882_BAT_THRESHOLD_SHIFT                       0
#define REG00_BQ25882_BAT_THRESHOLD_OFFSET                      6800
#define REG00_BQ25882_BAT_THRESHOLD_STEP                        10
#define REG00_BQ25882_BAT_THRESHOLD_8600MV                      (BIT(7) | BIT(5) | BIT(4) | BIT(2))
#define REG00_BQ25882_BAT_THRESHOLD                             REG00_BQ25882_BAT_THRESHOLD_8600MV


/* Address:01h */
#define REG01_BQ25882_ADDRESS                                   0x01

#define REG01_BQ25882_FAST_CURRENT_LIMIT_MASK                   (BIT(5) | BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG01_BQ25882_FAST_CURRENT_LIMIT_SHIFT                  0
#define REG01_BQ25882_FAST_CURRENT_LIMIT_OFFSET                 0
#define REG01_BQ25882_FAST_CURRENT_LIMIT_STEP                   50
#define REG01_BQ25882_FAST_CURRENT_LIMIT_1500MA                 (BIT(4) | BIT(3) | BIT(2) | BIT(1))
#define REG01_BQ25882_FAST_CURRENT_LIMIT                        REG01_BQ25882_FAST_CURRENT_LIMIT_1500MA

#define REG01_BQ25882_EN_ILIM_MASK                              BIT(6)
#define REG01_BQ25882_EN_ILIM_DISABLE                           0
#define REG01_BQ25882_EN_ILIM_ENABLE                            BIT(6)//default
#define REG04_BQ25882_ILIM_SHIFT                         		6


#define REG01_BQ25882_EN_HIZ_MASK                               BIT(7)
#define REG01_BQ25882_EN_HIZ_DISABLE                            0//default
#define REG01_BQ25882_EN_HIZ_ENABLE                             BIT(7)


/* Address:02h */
#define REG02_BQ25882_ADDRESS                                   0x02

#define REG02_BQ25882_INPUT_VOLTAGE_LIMIT_MASK                  (BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG02_BQ25882_INPUT_VOLTAGE_LIMIT_SHIFT                 0
#define REG02_BQ25882_INPUT_VOLTAGE_LIMIT_OFFSET                3900
#define REG02_BQ25882_INPUT_VOLTAGE_LIMIT_STEP                  100
#define REG02_BQ25882_INPUT_VOLTAGE_LIMIT_4400MV                (BIT(2) | BIT(0))//default
#define REG02_BQ25882_INPUT_VOLTAGE_LIMIT                       REG02_BQ25882_INPUT_VOLTAGE_LIMIT_4400MV

//BIT(6),BIT(5) reserved

#define REG02_BQ25882_EN_VINDPM_RST_MASK                        BIT(7)
#define REG02_BQ25882_EN_VINDPM_RST_DISABLE                     0
#define REG02_BQ25882_EN_VINDPM_RST_ENBALE                      BIT(7)


/* Address:03h */
#define REG03_BQ25882_ADDRESS                                   0x03

#define REG03_BQ25882_INPUT_CURRENT_LIMIT_MASK                  (BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG03_BQ25882_INPUT_CURRENT_LIMIT_SHIFT                 0
#define REG03_BQ25882_INPUT_CURRENT_LIMIT_OFFSET                500
#define REG03_BQ25882_INPUT_CURRENT_LIMIT_STEP                  100
#define REG03_BQ25882_INPUT_CURRENT_LIMIT_500MA                 0
#define REG03_BQ25882_INPUT_CURRENT_LIMIT_900MA                 (BIT(2))//(BIT(2) | BIT(0))
#define REG03_BQ25882_INPUT_CURRENT_LIMIT_1200MA                (BIT(2) | BIT(1) | BIT(0))
#define REG03_BQ25882_INPUT_CURRENT_LIMIT_1500MA                (BIT(3) | BIT(1))
#define REG03_BQ25882_INPUT_CURRENT_LIMIT_1700MA                (BIT(3) | BIT(2))
#define REG03_BQ25882_INPUT_CURRENT_LIMIT_2000MA                (BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG03_BQ25882_INPUT_CURRENT_LIMIT_3000MA                (BIT(4) | BIT(3) | BIT(0))


#define REG03_BQ25882_EN_ICO_MASK                               BIT(5)
#define REG03_BQ25882_EN_ICO_DISABLE                            0
#define REG03_BQ25882_EN_ICO_ENABLE                             BIT(5)//default

#define REG03_BQ25882_FORCE_INDET_MASK                          BIT(6)
#define REG03_BQ25882_FORCE_INDET_DISABLE                       0//default
#define REG03_BQ25882_FORCE_INDET_ENABLE                        BIT(6)

#define REG03_BQ25882_FORCE_ICO_MASK                            BIT(7)
#define REG03_BQ25882_FORCE_ICO_DISABLE                         0//default
#define REG03_BQ25882_FORCE_ICO_ENABLE                          BIT(7)


/* Address:04h */
#define REG04_BQ25882_ADDRESS                                   0x04

#define REG04_BQ25882_ITERM_LIMIT_MASK                          (BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG04_BQ25882_ITERM_LIMIT_SHIFT                         0
#define REG04_BQ25882_ITERM_LIMIT_OFFSET                        50
#define REG04_BQ25882_ITERM_LIMIT_STEP                          50
#define REG04_BQ25882_ITERM_LIMIT_100MA                         BIT(0)
#define REG04_BQ25882_ITERM_LIMIT_150MA                         BIT(1)//default
#define REG04_BQ25882_ITERM_LIMIT_200MA                         (BIT(1) | BIT(0))
#define REG04_BQ25882_ITERM_LIMIT_600MA                         (BIT(3) | BIT(1) | BIT(0))
#define REG04_BQ25882_ITERM_LIMIT                               REG04_BQ25882_ITERM_LIMIT_600MA

#define REG04_BQ25882_IPRECHG_LIMIT_MASK                        (BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define REG04_BQ25882_IPRECHG_LIMIT_SHIFT                       4
#define REG04_BQ25882_IPRECHG_LIMIT_OFFSET                      50
#define REG04_BQ25882_IPRECHG_LIMIT_STEP                        50
#define REG04_BQ25882_IPRECHG_LIMIT_150MA                       BIT(5)//default
#define REG04_BQ25882_IPRECHG_LIMIT                             REG04_BQ25882_IPRECHG_LIMIT_150MA


/* Address:05h */
#define REG05_BQ25882_ADDRESS                                   0x05

#define REG05_BQ25882_TMR2X_EN_MASK                             BIT(0)
#define REG05_BQ25882_TMR2X_EN_DISABLE                          0
#define REG05_BQ25882_TMR2X_EN_ENABLE                           BIT(0)//default


#define REG05_BQ25882_CHG_TIMER_MASK                            (BIT(2) | BIT(1))
#define REG05_BQ25882_CHG_TIMER_5H                              0
#define REG05_BQ25882_CHG_TIMER_8H                              BIT(1)
#define REG05_BQ25882_CHG_TIMER_12H                             BIT(2)//default
#define REG05_BQ25882_CHG_TIMER_20H                             (BIT(2) | BIT(1))

#define REG05_BQ25882_EN_TIMER_MASK                             BIT(3)
#define REG05_BQ25882_EN_TIMER_DISABLE                          0
#define REG05_BQ25882_EN_TIMER_ENABLE                           BIT(3)//default

#define REG05_BQ25882_WATCHDOG_MASK                             (BIT(5) | BIT(4))
#define REG05_BQ25882_WATCHDOG_DISABLE                          0
#define REG05_BQ25882_WATCHDOG_40S                              BIT(4)//default
#define REG05_BQ25882_WATCHDOG_80S                              BIT(5)
#define REG05_BQ25882_WATCHDOG_160S                             (BIT(5) | BIT(4))
#define REG05_BQ25882_WATCHDOG                                  REG05_BQ25882_WATCHDOG_DISABLE

//BIT(6) reserved

#define REG05_BQ25882_EN_TERM_MASK                              BIT(7)
#define REG05_BQ25882_EN_TERM_DISABLE                           0
#define REG05_BQ25882_EN_TERM_ENABLE                            BIT(7)//default


/* Address:06h */
#define REG06_BQ25882_ADDRESS                                   0x06

#define REG06_BQ25882_VRECHG_MASK                               (BIT(1) | BIT(0))
#define REG06_BQ25882_VRECHG_SHIFT                         		0
#define REG06_BQ25882_VRECHG_OFFSET                        		100
#define REG06_BQ25882_VRECHG_STEP                          		100
#define REG06_BQ25882_VRECHG_100MV                              0//default
#define REG06_BQ25882_VRECHG_200MV                              BIT(0)
#define REG06_BQ25882_VRECHG_300MV                              BIT(1)
#define REG06_BQ25882_VRECHG_400MV                              (BIT(1) | BIT(0))
#define REG06_BQ25882_VRECHG                                    REG06_BQ25882_VRECHG_100MV

#define REG06_BQ25882_BATLOWV_MASK                              BIT(2)
#define REG06_BQ25882_BATLOWV_5600MV                            0//default
#define REG06_BQ25882_BATLOWV_6000MV                            BIT(2)

#define REG06_BQ25882_EN_CHG_MASK                               BIT(3)
#define REG06_BQ25882_EN_CHG_DISABLE                            0
#define REG06_BQ25882_EN_CHG_ENABLE                             BIT(3)//default

#define REG06_BQ25882_TREG_MASK                                 (BIT(5) | BIT(4))
#define REG06_BQ25882_TREG_60_0C                                0
#define REG06_BQ25882_TREG_80_0C                                BIT(4)
#define REG06_BQ25882_TREG_100_0C                               BIT(5)
#define REG06_BQ25882_TREG_120_0C                               (BIT(5) | BIT(4))//default

#define REG06_BQ25882_AUTO_INDET_EN_MASK                        BIT(6)
#define REG06_BQ25882_AUTO_INDET_EN_DISABLE                     0
#define REG06_BQ25882_AUTO_INDET_EN_ENABLE                      BIT(6)//default

#define REG06_BQ25882_EN_OTG_MASK                               BIT(7)
#define REG06_BQ25882_EN_OTG_DISABLE                            0//default
#define REG06_BQ25882_EN_OTG_ENABLE                             BIT(7)


/* Address:07h */
#define REG07_BQ25882_ADDRESS                                   0x07

#define REG07_BQ25882_SYS_MIN_MASK                              (BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG07_BQ25882_SYS_MIN_SHIFT                             0
#define REG07_BQ25882_SYS_MIN_OFFSET                            6000
#define REG07_BQ25882_SYS_MIN_STEP                              100
#define REG07_BQ25882_SYS_MIN_7000MV                            (BIT(3) | BIT(1))//default 7000MV
#define REG07_BQ25882_SYS_MIN                                   REG07_BQ25882_SYS_MIN_7000MV

#define REG07_BQ25882_TOPOFF_TIMER_MASK                         (BIT(5) | BIT(4))
#define REG07_BQ25882_TOPOFF_TIMER_DISABLE                      0//default
#define REG07_BQ25882_TOPOFF_TIMER_15MIN                        BIT(4)
#define REG07_BQ25882_TOPOFF_TIMER_30MIN                        BIT(5)
#define REG07_BQ25882_TOPOFF_TIMER_45MIN                        (BIT(5) | BIT(4))
#define REG07_BQ25882_TOPOFF                                    REG07_BQ25882_TOPOFF_TIMER_15MIN

#define REG07_BQ25882_WD_RST_MASK                               BIT(6)
#define REG07_BQ25882_WD_RST_NORMAL                             0
#define REG07_BQ25882_WD_RST_RESET                              BIT(6)

#define REG07_BQ25882_PFM_DIS_MASK                              BIT(7)
#define REG07_BQ25882_PFM_DIS_ENABLE                            0//default
#define REG07_BQ25882_PFM_DIS_DISABLE                           BIT(7)


/* Address:08h */
#define REG08_BQ25882_ADDRESS                                   0x08
//Reserved


/* Address:09h */
#define REG09_BQ25882_ADDRESS                                   0x09

#define REG09_BQ25882_OTG_VLIM_MASK                             (BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG09_BQ25882_OTG_VLIM_SHIFT                            0
#define REG09_BQ25882_OTG_VLIM_OFFSET                           4500
#define REG09_BQ25882_OTG_VLIM_STEP                             100
#define REG09_BQ25882_OTG_VLIM_5100MV                           (BIT(2) | BIT(1))//default 5100MV
#define REG09_BQ25882_OTG_VLIM                                  REG09_BQ25882_OTG_VLIM_5100MV

#define REG09_BQ25882_OTG_ILIM_MASK                             (BIT(7) | BIT(6) | BIT(5) | BIT(4))
#define REG09_BQ25882_OTG_ILIM_SHIFT                            4
#define REG09_BQ25882_OTG_ILIM_OFFSET                           500
#define REG09_BQ25882_OTG_ILIM_STEP                             100
#define REG09_BQ25882_OTG_ILIM_500MA                            0//default 500MA
#define REG09_BQ25882_OTG_ILIM_1300MA                           (BIT(7))//(BIT(7) | BIT(6) | BIT(4))//1300ma
#define REG09_BQ25882_OTG_ILIM                                  REG09_BQ25882_OTG_ILIM_500MA


/* Address:0Ah */
#define REG0A_BQ25882_ADDRESS                                   0x0A

#define REG0A_BQ25882_ICO_ILIM_MASK                             (BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0))
#define REG0A_BQ25882_ICO_ILIM_SHIFT                            0
#define REG0A_BQ25882_ICO_ILIM_OFFSET                           500
#define REG0A_BQ25882_ICO_ILIM_STEP                             100
#define REG0A_BQ25882_ICO_ILIM_500MA                            0
#define REG0A_BQ25882_ICO_ILIM                                  REG0A_BQ25882_ICO_ILIM_500MA

//BIT(7),BIT(6),BIT(5) reserved


/* Address:0Bh */
#define REG0B_BQ25882_ADDRESS                                   0x0B

#define REG0B_BQ25882_CHGSTAT_MASK                              (BIT(2) | BIT(1) | BIT(0))
#define REG0B_BQ25882_CHGSTAT_NOCHG                             0
#define REG0B_BQ25882_CHGSTAT_TRICKLE                           BIT(0)
#define REG0B_BQ25882_CHGSTAT_PRECHG                            BIT(1)
#define REG0B_BQ25882_CHGSTAT_CCCHG                             (BIT(1) | BIT(0))
#define REG0B_BQ25882_CHGSTAT_TAPECHG                           BIT(2)
#define REG0B_BQ25882_CHGSTAT_TOPOFFCHG                         (BIT(2) | BIT(0))
#define REG0B_BQ25882_CHGSTAT_CHGDONE                           (BIT(2) | BIT(1))

#define REG0B_BQ25882_WD_STAT_MASK                              BIT(3)
#define REG0B_BQ25882_WD_STAT_NORMAL                            0
#define REG0B_BQ25882_WD_STAT_EXPIRED                           BIT(3)

#define REG0B_BQ25882_TREG_STAT_MASK                            BIT(4)
#define REG0B_BQ25882_TREG_STAT_NORMAL                          0
#define REG0B_BQ25882_TREG_STAT_THERMAL                         BIT(4)

#define REG0B_BQ25882_VINDPM_STAT_MASK                          BIT(5)
#define REG0B_BQ25882_VINDPM_STAT_NORMAL                        0
#define REG0B_BQ25882_VINDPM_STAT_VINDPM                        BIT(5)

#define REG0B_BQ25882_IINDPM_STAT_MASK                          BIT(6)
#define REG0B_BQ25882_IINDPM_STAT_NORMAL                        0
#define REG0B_BQ25882_IINDPM_STAT_IINDPM                        BIT(6)

#define REG0B_BQ25882_ADC_DONE_STAT_MASK                        BIT(7)
#define REG0B_BQ25882_ADC_DONE_STAT_NOT                         0
#define REG0B_BQ25882_ADC_DONE_STAT_DONE                        BIT(7)


/* Address:0Ch */
#define REG0C_BQ25882_ADDRESS                                   0x0C

#define REG0C_BQ25882_VSYS_STAT_MASK                            BIT(0)
#define REG0C_BQ25882_VSYS_STAT_NOT                             0
#define REG0C_BQ25882_VSYS_STAT_IN                              BIT(0)

#define REG0C_BQ25882_ICO_STAT_MASK                             BIT(1)
#define REG0C_BQ25882_ICO_STAT_OPT                              0
#define REG0C_BQ25882_ICO_STAT_MAX                              BIT(1)

//BIT(3),BIT(2) reserved

#define REG0C_BQ25882_VBUS_STAT_MASK                            (BIT(6) | BIT(5) | BIT(4))
#define REG0C_BQ25882_VBUS_STAT_NO_INPUT                        0
#define REG0C_BQ25882_VBUS_STAT_SDP                             BIT(4)
#define REG0C_BQ25882_VBUS_STAT_CDP                             BIT(5)
#define REG0C_BQ25882_VBUS_STAT_DCP                             (BIT(5) | BIT(4))
#define REG0C_BQ25882_VBUS_STAT_POORSRC                         BIT(6)
#define REG0C_BQ25882_VBUS_STAT_UNKOWN                          (BIT(6) | BIT(4))
#define REG0C_BQ25882_VBUS_STAT_NON_STD                         (BIT(6) | BIT(5))
#define REG0C_BQ25882_VBUS_STAT_OTG                             (BIT(6) | BIT(5) | BIT(4))

#define REG0C_BQ25882_PG_STAT_MASK                              BIT(7)
#define REG0C_BQ25882_PG_STAT_NOTGOOD                           0
#define REG0C_BQ25882_PG_STAT_GOOD                              BIT(7)

/* Address:15h */
#define REG15_BQ25882_ADDRESS                                   0x15
#define REG15_BQ25882_ADC_EN_MASK                               BIT(7)
#define REG15_BQ25882_ADC_EN_DISABLE                            0
#define REG15_BQ25882_ADC_EN_ENABLE                             BIT(7)

/* Address:16h */
#define REG16_BQ25882_ADDRESS                                   0x16
#define REG16_BQ25882_IBUS_ADC_DIS_MASK                         BIT(7)
#define REG16_BQ25882_IBUS_ADC_DIS_DISABLE                      0
#define REG16_BQ25882_IBUS_ADC_DIS_ENABLE                       BIT(7)

#define REG16_BQ25882_ICHG_ADC_DIS_MASK                         BIT(6)
#define REG16_BQ25882_ICHG_ADC_DIS_DISABLE                      0
#define REG16_BQ25882_ICHG_ADC_DIS_ENABLE                       BIT(6)




/* Address:25h */
#define REG25_BQ25882_ADDRESS                                   0x25

#define REG25_BQ25882_RESET_MASK                                BIT(7)
#define REG25_BQ25882_RESET_KEEP                                0
#define REG25_BQ25882_RESET_DEFAULT                             BIT(7)



enum {
	OVERTIME_AC = 0,
	OVERTIME_USB,
	OVERTIME_DISABLED,
};

struct chip_bq25882 {
        struct i2c_client           *client;
        struct device               *dev;
        int                         hw_aicl_point;
        int                         sw_aicl_point;
        atomic_t                    charger_suspended;
};

struct oplus_chg_operations *  oplus_get_chg_ops(void);
bool oplus_charger_ic_chip_is_null(void);

extern int bq25882_input_current_limit_write(int value);
extern int bq25882_charging_current_write_fast(int chg_cur);
extern int bq25882_set_vindpm_vol(int vol);
extern void bq25882_set_aicl_point(int vbatt);
extern int bq25882_set_enable_volatile_writes(void);
extern int bq25882_set_complete_charge_timeout(int val);
extern int bq25882_float_voltage_write(int vfloat_mv);
extern int bq25882_set_prechg_current(int ipre_mA);
extern int bq25882_set_topoff_timer(void);
extern int bq25882_set_termchg_current(int term_curr);
extern int bq25882_set_rechg_voltage(int recharge_mv);
extern int bq25882_set_wdt_timer(int reg);
extern int bq25882_set_chging_term_disable(void);
extern int bq25882_kick_wdt(void);
extern int bq25882_enable_charging(void);
extern int bq25882_disable_charging(void);
extern int bq25882_check_charging_enable(void);
extern int bq25882_registers_read_full(void);
extern int bq25882_suspend_charger(void);
extern int bq25882_unsuspend_charger(void);
extern bool bq25882_check_charger_resume(void);
extern int bq25882_reset_charger(void);
extern int bq25882_otg_enable(void);
extern int bq25882_otg_disable(void);
extern int bq25882_ico_disable(void);
extern int bq25882_adc_en_enable(bool enable);
extern int bq25882_ibus_adc_dis_enable(void);
extern int bq25882_ichg_adc_dis_enable(void);
extern void bq25882_dump_registers(void);
extern int bq25882_hardware_init(void);

#ifdef CONFIG_OPLUS_CHARGER_MTK
#else /* CONFIG_OPLUS_CHARGER_MTK */
extern int qpnp_get_battery_voltage(void);
extern int opchg_get_charger_type(void) ;
extern int smbchg_get_chargerid_switch_val(void);
extern void smbchg_set_chargerid_switch_val(int value);
extern int smbchg_get_chargerid_volt(void);
extern int qpnp_get_prop_charger_voltage_now(void);
extern bool oplus_chg_is_usb_present(void);
extern int smbchg_get_boot_reason(void);
extern int oplus_chg_get_shutdown_soc(void);
extern int oplus_chg_backup_soc(int backup_soc);
bool oplus_pmic_check_chip_is_null(void);
#endif/* CONFIG_OPLUS_CHARGER_MTK */
#endif
