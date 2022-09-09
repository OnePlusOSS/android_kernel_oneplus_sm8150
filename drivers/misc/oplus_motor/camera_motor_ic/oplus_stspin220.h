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
#ifndef __OPLUS_STSPIN220_H__
#define __OPLUS_STSPIN220_H__

#define	 RATIO_A 24 /*2.4*/
#define	 RATIO_B 1806 /*18.06*/
#define	 RATIO_B_FI_6 354 /*3.54*/
#define	 RATIO_C 20
#define	 RATIO_D 32

#define ERROR_NO (-11)

enum {
	GPIO_MODE = 0,
	HIGH_IMPEDANCE_MODE
};

struct oplus_sts_chip {
	struct device	*dev;
	struct pwm_device *pwm_dev;
#ifdef CONFIG_MTK_PLATFORM
	struct pwm_spec_config pwm_setting;
#endif
	struct pinctrl *pctrl;
	struct pinctrl_state *pwm_state;
	struct pinctrl_state *boost_state;
	struct pinctrl_state *standby_state;
	struct pinctrl_state *enable_state;
	struct pinctrl_state *dir_state;
	struct pinctrl_state *mode1_state;
	struct pinctrl_state *active_state;
	struct pinctrl_state *sleep_state;
	unsigned int step_gpio;
	unsigned int boost_gpio;
	unsigned int standby_gpio;
	unsigned int enable_gpio;
	unsigned int dir_gpio;
	unsigned int mode1_gpio;
	int md_dir;
	int motor_type;
	int ratio_a;
	int ratio_b;
	int ratio_c;
	int ratio_d;
};

#endif /* __OPLUS_DRV8834_H__*/

