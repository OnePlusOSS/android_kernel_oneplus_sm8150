/* SPDX-License-Identifier: GPL-2.0-only*/
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 * File: oplus_tri_key.c
 *
 * Description:
 *      Definitions for m1120 tri_state_key data process.
 *
 * Version: 1.0
 */
#ifndef __TRIKEY_H__
#define __TRIKEY_H__

#include <linux/alarmtimer.h>
#include <linux/version.h>
#include "ist_hall_ic/hall_ist8801.h"

enum debug_level {
	LEVEL_BASIC,
	LEVEL_DEBUG,
};

enum dhall_id {
	DHALL_0 = 0,
	DHALL_1,
};

enum dhall_detection_mode {
	DETECTION_MODE_POLLING = 0,
	DETECTION_MODE_INTERRUPT,
	DETECTION_MODE_INVALID,
};

enum motor_direction {
	MOTOR_DOWN = 0,
	MOTOR_UPWARD,
};

enum tri_key_position {
	UP_STATE,
	DOWN_STATE,
	MID_STATE,
};

extern unsigned int tristate_extcon_tab[];
extern unsigned int tri_key_debug;

struct dhall_data_t {
	short data0;
	short data1;
};

struct dhall_operations {
	int (*get_data)(short *data);
	int (*set_detection_mode)(u8 mode);
	int (*enable_irq)(bool enable);
	int (*clear_irq)(void);
	int (*get_irq_state)(void);
	bool (*update_threshold)(int position, short lowthd, short highthd);
	void (*dump_regs)(u8 *buf);
	int (*set_reg)(int reg, int val);
	bool (*is_power_on)(void);
	void (*set_sensitivity)(char *data);
};

struct extcon_dev_data {
	struct work_struct dwork;
	struct work_struct register_work;
	struct extcon_dev *edev;
	struct device *dev;
	struct i2c_client *client;
	struct input_dev     *input_dev;
	struct timer_list s_timer;
	struct pinctrl *key_pinctrl;
	struct pinctrl_state *set_state;
	struct delayed_work	up_work;
	struct delayed_work	down_work;
	struct dhall_operations *dhall_up_ops;
	struct dhall_operations *dhall_down_ops;
	struct mutex mtx;
	const char *d_name;
	const char *m_name;
	int		position;
	int		last_position;
	int		project_info;
	int		interf;
	short		state;
	short		dhall_data0;
	short		dhall_data1;
	short		dnHall_UpV;
	short		dnHall_MdV;
	short		dnHall_DnV;
	short		upHall_UpV;
	short		upHall_MdV;
	short		upHall_DnV;
	int			manual2auto_up_switch;
	int			manual2auto_down_switch;
	int			irq;
};

extern int oplus_register_hall(const char *name,
		struct dhall_operations *ops, struct extcon_dev_data *hall_dev);
extern int oplus_hall_get_data(unsigned int id);
extern int oplus_hall_set_detection_mode(unsigned int id, u8 mode);
extern int oplus_hall_enable_irq(unsigned int id, bool enable);
extern int oplus_hall_clear_irq(unsigned int id);
extern int oplus_hall_irq_handler(unsigned int id);
extern int oplus_hall_get_irq_state(unsigned int id);
extern void oplus_hall_dump_regs(unsigned int id, u8 *buf);
extern int oplus_hall_set_reg(unsigned int id, int reg, int val);
extern bool oplus_hall_update_threshold(unsigned int id,
				int position, short lowthd, short highthd);
extern bool oplus_hall_is_power_on(void);
extern int aw8697_op_haptic_stop(void);

#endif /* __TRIKEY_H__ */
