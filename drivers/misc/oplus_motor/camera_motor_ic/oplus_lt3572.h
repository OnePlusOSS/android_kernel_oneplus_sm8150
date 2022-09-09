/************************************************************************************
** Copyright (C), 2008-2017, OPLUS Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: oplus_drv8834.h
**
** Description:
**	Definitions for motor driver ic drv8834.
**
** Version: 1.0
** Date created: 2018/01/14,20:27
**
** --------------------------- Revision History: -------------------------------------
* <version>		<date>		<author>		<desc>
**************************************************************************************/
#ifndef __OPLUS_LT3572_H__
#define __OPLUS_LT3572_H__

#define	 RATIO_A 29 /*2.9*/

#define ERROR_NO (-11)

enum {
	GPIO_MODE = 0,
	HIGH_IMPEDANCE_MODE
};
enum DUTY_RATIO {
	RATIO_0_25 = 0,/*up*/
	RATIO_0_75	/*down*/
};

struct oplus_lt_chip {
	struct device	*dev;
	struct pwm_device *pwm_dev;
#ifdef CONFIG_MTK_PLATFORM
	struct pwm_spec_config pwm_setting;
#endif
	struct pinctrl *pctrl;
	struct pinctrl_state *pwm_state;
	struct pinctrl_state *shdn_state;
	struct pinctrl_state *shdna_state;
	struct pinctrl_state *shdnb_state;
	unsigned int step_gpio;
	unsigned int shdn_gpio;
	unsigned int shdna_gpio;
	unsigned int shdnb_gpio;
	int motor_type;
	int duty_ratio;
	int ratio_a;
};

#endif /*__OPLUS_LT3572_H__*/

