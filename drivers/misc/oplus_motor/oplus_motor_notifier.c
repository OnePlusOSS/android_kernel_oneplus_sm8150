/************************************************************************************
** Copyright (C), 2008-2018, OPLUS Mobile Comm Corp., Ltd
** VENDOR_EDIT
** File: oplus_motor_notifier.c
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
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/notifier.h>
#include <linux/init.h>
#include <linux/module.h>

static BLOCKING_NOTIFIER_HEAD(motor_chain);

/**
 *	register_motor_notifier - register a client notifier
 *	@nb: notifier block to callback on events
 */
int register_motor_notifier(struct notifier_block *nb)
{
	if (!nb) {
		return -EINVAL;
	}

	return blocking_notifier_chain_register(&motor_chain, nb);
}
EXPORT_SYMBOL(register_motor_notifier);

/**
 *	unregister_motor_notifier - unregister a client notifier
 *	@nb: notifier block to callback on events
 */
int unregister_motor_notifier(struct notifier_block *nb)
{
	if (!nb) {
		return -EINVAL;
	}

	return blocking_notifier_chain_unregister(&motor_chain, nb);
}
EXPORT_SYMBOL(unregister_motor_notifier);

/**
 * motor_notifier_call_chain - notify clients of sensor_events
 *
 */
int motor_notifier_call_chain(unsigned long val)
{
	return blocking_notifier_call_chain(&motor_chain, val, NULL);
}
EXPORT_SYMBOL(motor_notifier_call_chain);
