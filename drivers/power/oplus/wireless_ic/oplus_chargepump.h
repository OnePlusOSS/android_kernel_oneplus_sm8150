/************************************************************************************
** File:  oplus_chargepump.c
** OPLUS_FEATURE_CHG_BASIC
** Copyright (C), 2008-2012, OPLUS Mobile Comm Corp., Ltd
** 
** Description: 
**   
** 
** Version: 1.0
** Date created: 21:03:46,09/04/2019
** Author: Lin Shangbo
** 
** --------------------------- Revision History: ------------------------------------------------------------
* <version>       <date>        <author>              			<desc>
* Revision 1.0    2019-04-09    Lin Shangbo    		Created for new charger
************************************************************************************************************/


#ifndef __OPLUS_CHARGEPUMP_H__
#define __OPLUS_CHARGEPUMP_H__

#define OP20A

struct chip_chargepump {
	struct i2c_client		*client;
	struct device			*dev;
	struct pinctrl			*pinctrl;   
	int						chargepump_en_gpio;
	struct pinctrl_state	*chargepump_en_active;
	struct pinctrl_state	*chargepump_en_sleep;
	struct pinctrl_state	*chargepump_en_default;
	atomic_t				chargepump_suspended;
	bool					is_chargepump_enable;
};

extern int chargepump_set_for_EPP(void);
extern int chargepump_enable(void);
extern int chargepump_set_for_otg(char enable);
extern int chargepump_set_for_LDO(void);
extern int chargepump_enable_voltage_diff_detect(void);
extern int chargepump_enable_watchdog(void);
extern int chargepump_kick_watchdog(void);
extern int chargepump_disable(void);
extern int chargepump_get_chargepump_en_val(void);
extern void chargepump_print_log(void);
extern int chargepump_check_fastchg_status(void);
extern int chargepump_check_dwp_status(void);
extern int chargepump_dwp_enable(void);
extern int chargepump_dwp_disable(void);
#endif
