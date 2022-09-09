/************************************************************************************
** Copyright (C), 2008-2017, OPLUS Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: motor.c
**
** Description:
**	Definitions for oplus_motor common software.
**
** Version: 1.0
** Date created: 2018/01/14,20:27
**
** --------------------------- Revision History: ------------------------------------
* <version>		<date>		<author>		<desc>
**************************************************************************************/
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/serio.h>
#include <soc/oplus/boot_mode.h>
#include <soc/oplus/system/oplus_project.h>
#include "oplus_motor/oplus_motor.h"
#include "oplus_motor/oplus_motor_notifier.h"
#include "oplus_motor/digital_hall_ic/oplus_m1120.h"

/*if can not compile success, please update vendor/oplus_motor*/

void oplus_parse_motor_info(struct oplus_motor_chip *chip)
{
	unsigned int pcb_version = 0;

	if (!chip) {
		return;
	}

	MOTOR_ERR("customize %s;%d\n", __func__, __LINE__);

	chip->info.type = MOTOR_FI5;
	chip->info.motor_ic = STSPIN220;
	chip->dir_sign = NEGATIVE;
	chip->is_support_ffd = true;

	pcb_version = get_PCB_Version();

	if (is_project(18115)) {
		if ((pcb_version <= HW_VERSION__13)
				|| (HW_VERSION__16 == pcb_version)) {   /*DVT before and DVT2*/
			chip->pwm_param.normal.up_brake_delay = 50000;
			chip->pwm_param.normal.down_brake_delay = 50000;
			chip->stop.neg[1] = -2;
			chip->stop.pos[1] = 2;

		} else {    /*PVT after*/
			chip->pwm_param.normal.up_brake_delay = 40000;
			chip->pwm_param.normal.down_brake_delay = 25000;
			chip->stop.neg[1] = -2;
			chip->stop.pos[1] = 2;
		}

	} else if (is_project(18501)) {
		if ((pcb_version <= HW_VERSION__13)
				|| (HW_VERSION__16 == pcb_version)) {   /*DVT before and DVT2*/
			chip->pwm_param.normal.up_brake_delay = 50000;
			chip->pwm_param.normal.down_brake_delay = 50000;
			chip->stop.neg[1] = -2;
			chip->stop.pos[1] = 2;

		} else {    /*PVT after*/
			chip->pwm_param.normal.up_brake_delay = 40000;
			chip->pwm_param.normal.down_brake_delay = 25000;
			chip->stop.neg[1] = -2;
			chip->stop.pos[1] = 2;
		}

	} else if (is_project(18503)) {
		if (pcb_version <= HW_VERSION__13) {   /*DVT before*/
			chip->pwm_param.normal.up_brake_delay = 50000;
			chip->pwm_param.normal.down_brake_delay = 50000;
			chip->stop.neg[1] = -2;
			chip->stop.pos[1] = 2;

		} else {    /*PVT after and DVT2*/
			chip->pwm_param.normal.up_brake_delay = 40000;
			chip->pwm_param.normal.down_brake_delay = 25000;
			chip->stop.neg[1] = -2;
			chip->stop.pos[1] = 2;
		}

	} else if (is_project(18119)) {
		chip->pwm_param.normal.up_brake_delay = 40000;
		chip->pwm_param.normal.down_brake_delay = 25000;
		chip->stop.neg[1] = -2;
		chip->stop.pos[1] = 2;

	} else if (is_project(18117)) {
		chip->pwm_param.normal.up_brake_delay = 40000;
		chip->pwm_param.normal.down_brake_delay = 25000;
		chip->stop.neg[1] = -2;
		chip->stop.pos[1] = 2;

	} else {
		MOTOR_ERR("%s: unknow project.\n", __func__);
	}

	MOTOR_ERR("project(%d), pcb_version(%d), up_down(%d, %d).\n",
		  get_project(), pcb_version, chip->pwm_param.normal.up_brake_delay,
		  chip->pwm_param.normal.down_brake_delay);

	MOTOR_LOG("boot_mode is %d.\n", get_boot_mode());

	if ((MSM_BOOT_MODE__RECOVERY == get_boot_mode())
			|| (MSM_BOOT_MODE__FACTORY == get_boot_mode()) ||
			qpnp_is_power_off_charging() || (MSM_BOOT_MODE__SAU == get_boot_mode())) {
		chip->boot_mode = MOTOR_OTHER_MODE;

	} else {
		chip->boot_mode = MOTOR_NORMAL_MODE;
	}
}
EXPORT_SYMBOL(oplus_parse_motor_info);

void oplus_m1120_reconfig(struct oplus_dhall_chip *chip)
{
	if (is_project(18115)) {
		if (get_PCB_Version() <= HW_VERSION__12) {   /*EVT and before*/
			chip->reg.map.range = M1120_VAL_INTSRS_SRS_0_08mT;

		} else {
			chip->reg.map.range = M1120_VAL_INTSRS_SRS_0_04mT;
		}

	} else if (is_project(18501)) {
		if (get_PCB_Version() <= HW_VERSION__11) {	/*T0 and before*/
			chip->reg.map.range = M1120_VAL_INTSRS_SRS_0_08mT;

		} else if (get_PCB_Version() == HW_VERSION__12) {	/*EVT*/
			chip->reg.map.range = M1120_VAL_INTSRS_SRS_0_02mT;

		} else if (get_PCB_Version() >= HW_VERSION__13) {	/*DVT and after*/
			chip->reg.map.range = M1120_VAL_INTSRS_SRS_0_04mT;

		} else {
			MOTOR_ERR("%s: unknow pcb_version(%d).\n", __func__, get_PCB_Version());
		}

	} else if (is_project(18503)) {
		if (get_PCB_Version() <= HW_VERSION__11) {	/*T0 and before*/
			chip->reg.map.range = M1120_VAL_INTSRS_SRS_0_08mT;

		} else if (get_PCB_Version() == HW_VERSION__12) {	/*EVT*/
			chip->reg.map.range = M1120_VAL_INTSRS_SRS_0_02mT;

		} else if (get_PCB_Version() >= HW_VERSION__13) {	/*DVT and after*/
			chip->reg.map.range = M1120_VAL_INTSRS_SRS_0_04mT;

		} else {
			MOTOR_ERR("%s: unknow pcb_version(%d).\n", __func__, get_PCB_Version());
		}

	} else if (is_project(18119)) {
		chip->reg.map.range = M1120_VAL_INTSRS_SRS_0_04mT;

	} else if (is_project(18117)) {
		chip->reg.map.range = M1120_VAL_INTSRS_SRS_0_04mT;

	} else {
		MOTOR_ERR("%s: unknow project.\n", __func__);
	}

	MOTOR_ERR("project(%d), pcb_version(%d), range(%d).\n", get_project(),
		  get_PCB_Version(), chip->reg.map.range);
}
EXPORT_SYMBOL(oplus_m1120_reconfig);

