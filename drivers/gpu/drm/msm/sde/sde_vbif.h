/* Copyright (c) 2016-2018, The Linux Foundation. All rights reserved.
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

#ifndef __SDE_VBIF_H__
#define __SDE_VBIF_H__

#include "sde_kms.h"

struct sde_vbif_set_ot_params {
	u32 xin_id;
	u32 num;
	u32 width;
	u32 height;
	u32 frame_rate;
	bool rd;
	bool is_wfd;
	u32 vbif_idx;
	u32 clk_ctrl;
};

struct sde_vbif_set_memtype_params {
	u32 xin_id;
	u32 vbif_idx;
	u32 clk_ctrl;
	bool is_cacheable;
};

/**
 * struct sde_vbif_set_xin_halt_params - xin halt parameters
 * @vbif_idx: vbif identifier
 * @xin_id: client interface identifier
 * @clk_ctrl: clock control identifier of the xin
 * @forced_on: whether or not previous call to xin halt forced the clocks on,
 *	only applicable to xin halt disable calls
 * @enable: whether to enable/disable xin halts
 */
struct sde_vbif_set_xin_halt_params {
	u32 vbif_idx;
	u32 xin_id;
	u32 clk_ctrl;
	bool forced_on;
	bool enable;
};

/**
 * struct sde_vbif_set_qos_params - QoS remapper parameter
 * @vbif_idx: vbif identifier
 * @xin_id: client interface identifier
 * @clk_ctrl: clock control identifier of the xin
 * @num: pipe identifier (debug only)
 * @is_rt: true if pipe is used in real-time use case
 */
struct sde_vbif_set_qos_params {
	u32 vbif_idx;
	u32 xin_id;
	u32 clk_ctrl;
	u32 num;
	bool is_rt;
};

/**
 * sde_vbif_set_ot_limit - set OT limit for vbif client
 * @sde_kms:	SDE handler
 * @params:	Pointer to OT configuration parameters
 */
void sde_vbif_set_ot_limit(struct sde_kms *sde_kms,
		struct sde_vbif_set_ot_params *params);

/**
 * sde_vbif_set_xin_halt - halt one of the xin ports
 *	This function isn't thread safe.
 * @sde_kms:	SDE handler
 * @params:	Pointer to halt configuration parameters
 * Returns:	Whether or not VBIF clocks were forced on
 */
bool sde_vbif_set_xin_halt(struct sde_kms *sde_kms,
		struct sde_vbif_set_xin_halt_params *params);

/**
 * sde_vbif_set_qos_remap - set QoS priority level remap
 * @sde_kms:	SDE handler
 * @params:	Pointer to QoS configuration parameters
 */
void sde_vbif_set_qos_remap(struct sde_kms *sde_kms,
		struct sde_vbif_set_qos_params *params);

/**
 * sde_vbif_clear_errors - clear any vbif errors
 * @sde_kms:	SDE handler
 */
void sde_vbif_clear_errors(struct sde_kms *sde_kms);

/**
 * sde_vbif_init_memtypes - initialize xin memory types for vbif
 * @sde_kms:	SDE handler
 */
void sde_vbif_init_memtypes(struct sde_kms *sde_kms);

/**
 * sde_vbif_halt_plane_xin - halts the xin client for the unused plane
 * On unused plane, check if the vbif for this plane is idle or not.
 * If not then first force_on the planes clock and then send the
 * halt request. Wait for some time then check for the vbif idle
 * or not again.
 * @sde_kms:	SDE handler
 * @xin_id:	xin id of the unused plane
 * @clk_ctrl:	clk ctrl type for the unused plane
 * Returns:	0 on success, error code otherwise
 */
int sde_vbif_halt_plane_xin(struct sde_kms *sde_kms, u32 xin_id,
	       u32 clk_ctrl);

/**
 * sde_vbif_halt_xin_mask - halts/unhalts all the xin clients present in
 * the mask.
 * @sde_kms:	SDE handler
 * @xin_id_mask: Mask of all the xin-ids to be halted/unhalted
 * halt:	boolen to indicate halt/unhalt
 */
int sde_vbif_halt_xin_mask(struct sde_kms *sde_kms, u32 xin_id_mask, bool halt);

#ifdef CONFIG_DEBUG_FS
int sde_debugfs_vbif_init(struct sde_kms *sde_kms, struct dentry *debugfs_root);
void sde_debugfs_vbif_destroy(struct sde_kms *sde_kms);
#else
static inline int sde_debugfs_vbif_init(struct sde_kms *sde_kms,
		struct dentry *debugfs_root)
{
	return 0;
}
static inline void sde_debugfs_vbif_destroy(struct sde_kms *sde_kms)
{
}
#endif
#endif /* __SDE_VBIF_H__ */
