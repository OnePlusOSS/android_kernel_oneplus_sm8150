/* Copyright (c) 2015-2018, The Linux Foundation. All rights reserved.
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

#ifndef _SDE_HWIO_H
#define _SDE_HWIO_H

#include "sde_hw_util.h"

/**
 * MDP TOP block Register and bit fields and defines
 */
#define DISP_INTF_SEL                   0x004
#define INTR_EN                         0x010
#define INTR_STATUS                     0x014
#define INTR_CLEAR                      0x018
#define INTR2_EN                        0x008
#define INTR2_STATUS                    0x00c
#define INTR2_CLEAR                     0x02c
#define HIST_INTR_EN                    0x01c
#define HIST_INTR_STATUS                0x020
#define HIST_INTR_CLEAR                 0x024
#define INTF_INTR_EN                    0x1C0
#define INTF_INTR_STATUS                0x1C4
#define INTF_INTR_CLEAR                 0x1C8
#define SPLIT_DISPLAY_EN                0x2F4
#define SPLIT_DISPLAY_UPPER_PIPE_CTRL   0x2F8
#define DSPP_IGC_COLOR0_RAM_LUTN        0x300
#define DSPP_IGC_COLOR1_RAM_LUTN        0x304
#define DSPP_IGC_COLOR2_RAM_LUTN        0x308
#define PPB0_CNTL                       0x330
#define PPB0_CONFIG                     0x334
#define PPB1_CNTL                       0x338
#define PPB1_CONFIG                     0x33C
#define PPB2_CNTL                       0x370
#define PPB3_CNTL                       0x374
#define HW_EVENTS_CTL                   0x37C
#define CLK_CTRL3                       0x3A8
#define CLK_STATUS3                     0x3AC
#define CLK_CTRL4                       0x3B0
#define CLK_STATUS4                     0x3B4
#define CLK_CTRL5                       0x3B8
#define CLK_STATUS5                     0x3BC
#define CLK_CTRL7                       0x3D0
#define CLK_STATUS7                     0x3D4
#define SPLIT_DISPLAY_LOWER_PIPE_CTRL   0x3F0
#define SPLIT_DISPLAY_TE_LINE_INTERVAL  0x3F4
#define INTF_SW_RESET_MASK              0x3FC
#define HDMI_DP_CORE_SELECT             0x408
#define MDP_OUT_CTL_0                   0x410
#define MDP_VSYNC_SEL                   0x414
#define DCE_SEL                         0x450

/* SDE_SCALER_QSEED3 */
#define QSEED3_COEF_LUT_OFF              0x100
#define QSEED3_FILTERS                     5
#define QSEED3_LUT_REGIONS                 4
#define QSEED3_CIRCULAR_LUTS               9
#define QSEED3_SEPARABLE_LUTS              10
#define QSEED3_LUT_SIZE                    60
#define QSEED3_DIR_LUT_SIZE                (200 * sizeof(u32))
#define QSEED3_COEF_LUT_DIR_BIT            1
#define QSEED3_COEF_LUT_Y_CIR_BIT          2
#define QSEED3_COEF_LUT_UV_CIR_BIT         3
#define QSEED3_COEF_LUT_Y_SEP_BIT          4
#define QSEED3_COEF_LUT_UV_SEP_BIT         5
#define QSEED3_CIR_LUT_SIZE \
	(QSEED3_LUT_SIZE * QSEED3_CIRCULAR_LUTS * sizeof(u32))
#define QSEED3_SEP_LUT_SIZE \
	(QSEED3_LUT_SIZE * QSEED3_SEPARABLE_LUTS * sizeof(u32))

/* SDE_SCALER_QSEED3LITE */
#define QSEED3L_COEF_LUT_OFF                   0x100
#define QSEED3LITE_FILTERS                 2
#define QSEED3L_SEPARABLE_LUTS             10
#define QSEED3L_LUT_SIZE                   33
#define QSEED3L_SEP_LUT_SIZE \
	(QSEED3L_LUT_SIZE * QSEED3L_SEPARABLE_LUTS * sizeof(u32))

#endif /*_SDE_HWIO_H */
