/************************************************************************************
** Copyright (C), 2008-2018, OPLUS Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: oplus_motor_notifier.H
**
** Description:
**	Definitions for motor  notifier.
**
** Version: 1.0
** Date created: 2017/05/05
**
** --------------------------- Revision History: -------------------------------------------
* <version>	<date>		<author>					<desc>
**************************************************************************************/

#ifndef _OPLUS_MOTOR_NOTIFIER
#define _OPLUS_MOTOR_NOTIFIER

#include <linux/notifier.h>

enum motor_event {
	MOTOR_UP_EVENT = 0,
	MOTOR_DOWN_EVENT,
	MOTOR_BLOCK_EVENT,
};

extern int register_motor_notifier(struct notifier_block *nb);
extern int unregister_motor_notifier(struct notifier_block *nb);
extern int motor_notifier_call_chain(unsigned long val);

#endif /*_OPLUS_MOTOR_NOTIFIER*/
