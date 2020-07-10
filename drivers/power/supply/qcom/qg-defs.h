/* Copyright (c) 2018 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __QG_DEFS_H__
#define __QG_DEFS_H__

#define qg_dbg(chip, reason, fmt, ...)			\
	do {							\
		if (*chip->debug_mask & (reason))		\
			pr_info(fmt, ##__VA_ARGS__);	\
		else						\
			pr_debug(fmt, ##__VA_ARGS__);	\
	} while (0)

#define is_between(left, right, value) \
		(((left) >= (right) && (left) >= (value) \
			&& (value) >= (right)) \
		|| ((left) <= (right) && (left) <= (value) \
			&& (value) <= (right)))

#define UDATA_READY_VOTER		"UDATA_READY_VOTER"
#define FIFO_DONE_VOTER			"FIFO_DONE_VOTER"
#define FIFO_RT_DONE_VOTER		"FIFO_RT_DONE_VOTER"
#define SUSPEND_DATA_VOTER		"SUSPEND_DATA_VOTER"
#define GOOD_OCV_VOTER			"GOOD_OCV_VOTER"
#define PROFILE_IRQ_DISABLE		"NO_PROFILE_IRQ_DISABLE"
#define QG_INIT_STATE_IRQ_DISABLE	"QG_INIT_STATE_IRQ_DISABLE"
#define TTF_AWAKE_VOTER			"TTF_AWAKE_VOTER"

#define V_RAW_TO_UV(V_RAW)		div_u64(194637ULL * (u64)V_RAW, 1000)
#define FIFO_V_RESET_VAL		0x8000
#define FIFO_I_RESET_VAL		0x8000

#define DEGC_SCALE			10
#define UV_TO_DECIUV(a)			(a / 100)
#define DECIUV_TO_UV(a)			(a * 100)

#define QG_MAX_ESR_COUNT		10
#define QG_MIN_ESR_COUNT		2

#define CAP(min, max, value)			\
		((min > value) ? min : ((value > max) ? max : value))

#define QG_SOC_FULL	10000
#define BATT_SOC_32BIT	GENMASK(31, 0)

#endif /* __QG_DEFS_H__ */
