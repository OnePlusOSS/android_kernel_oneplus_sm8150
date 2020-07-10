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
 *
 */

#define pr_fmt(fmt)	"%s:%d: " fmt, __func__, __LINE__

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>
#include <linux/regulator/consumer.h>
#include <linux/dma-direction.h>
#include <soc/qcom/scm.h>
#include <soc/qcom/secure_buffer.h>
#include <asm/cacheflush.h>
#include <uapi/linux/sched/types.h>

#include "sde_rotator_base.h"
#include "sde_rotator_core.h"
#include "sde_rotator_dev.h"
#include "sde_rotator_util.h"
#include "sde_rotator_io_util.h"
#include "sde_rotator_smmu.h"
#include "sde_rotator_r1.h"
#include "sde_rotator_r3.h"
#include "sde_rotator_trace.h"
#include "sde_rotator_debug.h"


/* Rotator device id to be used in SCM call */
#define SDE_ROTATOR_DEVICE	21

/*
 * SCM call function id to be used for switching between secure and non
 * secure context
 */
#define MEM_PROTECT_SD_CTRL_SWITCH 0x18

/* waiting for hw time out, 3 vsync for 30fps*/
#define ROT_HW_ACQUIRE_TIMEOUT_IN_MS 100

/* waiting for inline hw start */
#define ROT_INLINE_START_TIMEOUT_IN_MS	(10000 + 500)

/* default pixel per clock ratio */
#define ROT_PIXEL_PER_CLK_NUMERATOR	36
#define ROT_PIXEL_PER_CLK_DENOMINATOR	10
#define ROT_FUDGE_FACTOR_NUMERATOR	105
#define ROT_FUDGE_FACTOR_DENOMINATOR	100
#define ROT_OVERHEAD_NUMERATOR		27
#define ROT_OVERHEAD_DENOMINATOR	10000

/* Minimum Rotator Clock value */
#define ROT_MIN_ROT_CLK			20000000

/* default minimum bandwidth vote */
#define ROT_ENABLE_BW_VOTE		64000
/*
 * Max rotator hw blocks possible. Used for upper array limits instead of
 * alloc and freeing small array
 */
#define ROT_MAX_HW_BLOCKS 2

#define SDE_REG_BUS_VECTOR_ENTRY(ab_val, ib_val)	\
	{						\
		.src = MSM_BUS_MASTER_AMPSS_M0,		\
		.dst = MSM_BUS_SLAVE_DISPLAY_CFG,	\
		.ab = (ab_val),				\
		.ib = (ib_val),				\
	}

#define BUS_VOTE_19_MHZ 153600000

static struct msm_bus_vectors rot_reg_bus_vectors[] = {
	SDE_REG_BUS_VECTOR_ENTRY(0, 0),
	SDE_REG_BUS_VECTOR_ENTRY(0, BUS_VOTE_19_MHZ),
};
static struct msm_bus_paths rot_reg_bus_usecases[ARRAY_SIZE(
		rot_reg_bus_vectors)];
static struct msm_bus_scale_pdata rot_reg_bus_scale_table = {
	.usecase = rot_reg_bus_usecases,
	.num_usecases = ARRAY_SIZE(rot_reg_bus_usecases),
	.name = "mdss_rot_reg",
	.active_only = 1,
};

/* forward prototype */
static int sde_rotator_update_perf(struct sde_rot_mgr *mgr);

static int sde_rotator_bus_scale_set_quota(struct sde_rot_bus_data_type *bus,
		u64 quota)
{
	int new_uc_idx;
	int ret;

	if (!bus) {
		SDEROT_ERR("null parameter\n");
		return -EINVAL;
	}

	if (!bus->bus_hdl) {
		SDEROT_DBG("bus scaling not enabled\n");
		return 0;
	} else if (bus->bus_hdl < 0) {
		SDEROT_ERR("invalid bus handle %d\n", bus->bus_hdl);
		return -EINVAL;
	}

	if (bus->curr_quota_val == quota) {
		SDEROT_DBG("bw request already requested\n");
		return 0;
	}

	if (!bus->bus_scale_pdata || !bus->bus_scale_pdata->num_usecases) {
		SDEROT_ERR("invalid bus scale data\n");
		return -EINVAL;
	}

	if (!quota) {
		new_uc_idx = 0;
	} else {
		struct msm_bus_vectors *vect = NULL;
		struct msm_bus_scale_pdata *bw_table =
			bus->bus_scale_pdata;
		u64 port_quota = quota;
		u32 total_axi_port_cnt;
		int i;

		new_uc_idx = (bus->curr_bw_uc_idx %
			(bw_table->num_usecases - 1)) + 1;

		total_axi_port_cnt = bw_table->usecase[new_uc_idx].num_paths;
		if (total_axi_port_cnt == 0) {
			SDEROT_ERR("Number of bw paths is 0\n");
			return -ENODEV;
		}
		do_div(port_quota, total_axi_port_cnt);

		for (i = 0; i < total_axi_port_cnt; i++) {
			vect = &bw_table->usecase[new_uc_idx].vectors[i];
			vect->ab = port_quota;
			vect->ib = 0;
		}
	}
	bus->curr_bw_uc_idx = new_uc_idx;
	bus->curr_quota_val = quota;

	SDEROT_EVTLOG(new_uc_idx, quota);
	SDEROT_DBG("uc_idx=%d quota=%llu\n", new_uc_idx, quota);
	ATRACE_BEGIN("msm_bus_scale_req_rot");
	ret = msm_bus_scale_client_update_request(bus->bus_hdl,
		new_uc_idx);
	ATRACE_END("msm_bus_scale_req_rot");

	return ret;
}

static int sde_rotator_enable_reg_bus(struct sde_rot_mgr *mgr, u64 quota)
{
	int ret = 0, changed = 0;
	u32 usecase_ndx = 0;

	if (!mgr || !mgr->reg_bus.bus_hdl)
		return 0;

	if (quota)
		usecase_ndx = 1;

	if (usecase_ndx != mgr->reg_bus.curr_bw_uc_idx) {
		mgr->reg_bus.curr_bw_uc_idx = usecase_ndx;
		changed++;
	}

	SDEROT_DBG("%s, changed=%d register bus %s\n", __func__, changed,
		quota ? "Enable":"Disable");

	if (changed) {
		ATRACE_BEGIN("msm_bus_scale_req_rot_reg");
		ret = msm_bus_scale_client_update_request(mgr->reg_bus.bus_hdl,
			usecase_ndx);
		ATRACE_END("msm_bus_scale_req_rot_reg");
	}

	return ret;
}

/*
 * Clock rate of all open sessions working a particular hw block
 * are added together to get the required rate for that hw block.
 * The max of each hw block becomes the final clock rate voted for
 */
static unsigned long sde_rotator_clk_rate_calc(
	struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private)
{
	struct sde_rot_perf *perf;
	unsigned long clk_rate[ROT_MAX_HW_BLOCKS] = {0};
	unsigned long total_clk_rate = 0;
	int i, wb_idx;

	list_for_each_entry(perf, &private->perf_list, list) {
		bool rate_accounted_for = false;
		/*
		 * If there is one session that has two work items across
		 * different hw blocks rate is accounted for in both blocks.
		 */
		for (i = 0; i < mgr->queue_count; i++) {
			if (perf->work_distribution[i]) {
				clk_rate[i] += perf->clk_rate;
				rate_accounted_for = true;
			}
		}

		/*
		 * Sessions that are open but not distributed on any hw block
		 * Still need to be accounted for. Rate is added to last known
		 * wb idx.
		 */
		wb_idx = perf->last_wb_idx;
		if ((!rate_accounted_for) && (wb_idx >= 0) &&
				(wb_idx < mgr->queue_count))
			clk_rate[wb_idx] += perf->clk_rate;
	}

	for (i = 0; i < mgr->queue_count; i++)
		total_clk_rate = max(clk_rate[i], total_clk_rate);

	SDEROT_DBG("Total clk rate calc=%lu\n", total_clk_rate);
	return total_clk_rate;
}

static struct clk *sde_rotator_get_clk(struct sde_rot_mgr *mgr, u32 clk_idx)
{
	if (clk_idx >= mgr->num_rot_clk) {
		SDEROT_ERR("Invalid clk index:%u", clk_idx);
		return NULL;
	}

	return mgr->rot_clk[clk_idx].clk;
}

static void sde_rotator_set_clk_rate(struct sde_rot_mgr *mgr,
		unsigned long rate, u32 clk_idx)
{
	unsigned long clk_rate;
	struct clk *clk = sde_rotator_get_clk(mgr, clk_idx);
	int ret;

	if (clk) {
		clk_rate = clk_round_rate(clk, rate);
		if (IS_ERR_VALUE(clk_rate)) {
			SDEROT_ERR("unable to round rate err=%ld\n", clk_rate);
		} else {
			ret = clk_set_rate(clk, clk_rate);
			if (ret < 0)
				SDEROT_ERR("clk_set_rate failed, err:%d\n",
						ret);
			else
				SDEROT_DBG("rotator clk rate=%lu\n", clk_rate);
		}
	} else {
		SDEROT_ERR("rotator clk not setup properly\n");
	}
}

/*
 * Update clock according to all open files on rotator block.
 */
static int sde_rotator_update_clk(struct sde_rot_mgr *mgr)
{
	struct sde_rot_file_private *priv;
	unsigned long clk_rate, total_clk_rate;

	total_clk_rate = 0;
	list_for_each_entry(priv, &mgr->file_list, list) {
		clk_rate = sde_rotator_clk_rate_calc(mgr, priv);
		total_clk_rate += clk_rate;
	}

	SDEROT_DBG("core_clk %lu\n", total_clk_rate);
	ATRACE_INT("core_clk", total_clk_rate);
	sde_rotator_set_clk_rate(mgr, total_clk_rate, SDE_ROTATOR_CLK_MDSS_ROT);

	return 0;
}

static int sde_rotator_footswitch_ctrl(struct sde_rot_mgr *mgr, bool on)
{
	int ret = 0;

	if (mgr->regulator_enable == on) {
		SDEROT_DBG("Regulators already in selected mode on=%d\n", on);
		return 0;
	}

	SDEROT_EVTLOG(on);
	SDEROT_DBG("%s: rotator regulators\n", on ? "Enable" : "Disable");

	if (on) {
		mgr->minimum_bw_vote = mgr->enable_bw_vote;
		sde_rotator_update_perf(mgr);
	}

	if (mgr->ops_hw_pre_pmevent)
		mgr->ops_hw_pre_pmevent(mgr, on);

	if (!sde_rot_mgr_pd_enabled(mgr))
		ret = sde_rot_enable_vreg(mgr->module_power.vreg_config,
			mgr->module_power.num_vreg, on);
	if (ret) {
		pr_err("rotator regulator failed to %s ret:%d client:%d\n",
		      on ? "enable" : "disable", ret,
				      sde_rot_mgr_pd_enabled(mgr));
		return ret;
	}

	if (mgr->ops_hw_post_pmevent)
		mgr->ops_hw_post_pmevent(mgr, on);

	if (!on) {
		mgr->minimum_bw_vote = 0;
		sde_rotator_update_perf(mgr);
	}

	mgr->regulator_enable = on;
	return 0;
}

static int sde_rotator_enable_clk(struct sde_rot_mgr *mgr, int clk_idx)
{
	struct clk *clk;
	int ret = 0;

	clk = sde_rotator_get_clk(mgr, clk_idx);
	if (clk) {
		ret = clk_prepare_enable(clk);
		if (ret)
			SDEROT_ERR("enable failed clk_idx %d\n", clk_idx);
	}

	return ret;
}

static void sde_rotator_disable_clk(struct sde_rot_mgr *mgr, int clk_idx)
{
	struct clk *clk;

	clk = sde_rotator_get_clk(mgr, clk_idx);
	if (clk)
		clk_disable_unprepare(clk);
}

int sde_rotator_clk_ctrl(struct sde_rot_mgr *mgr, int enable)
{
	int ret = 0;
	int changed = 0;

	if (enable) {
		if (mgr->rot_enable_clk_cnt == 0)
			changed++;
		mgr->rot_enable_clk_cnt++;
	} else {
		if (mgr->rot_enable_clk_cnt) {
			mgr->rot_enable_clk_cnt--;
			if (mgr->rot_enable_clk_cnt == 0)
				changed++;
		} else {
			SDEROT_ERR("Can not be turned off\n");
		}
	}

	if (changed) {
		SDEROT_EVTLOG(enable);
		SDEROT_DBG("Rotator clk %s\n", enable ? "enable" : "disable");

		if (enable) {
			ret = sde_rotator_enable_clk(mgr,
						SDE_ROTATOR_CLK_MNOC_AHB);
			if (ret)
				goto error_mnoc_ahb;
			ret = sde_rotator_enable_clk(mgr,
						SDE_ROTATOR_CLK_GCC_AHB);
			if (ret)
				goto error_gcc_ahb;
			ret = sde_rotator_enable_clk(mgr,
						SDE_ROTATOR_CLK_GCC_AXI);
			if (ret)
				goto error_gcc_axi;
			ret = sde_rotator_enable_clk(mgr,
						SDE_ROTATOR_CLK_MDSS_AHB);
			if (ret)
				goto error_mdss_ahb;
			ret = sde_rotator_enable_clk(mgr,
						SDE_ROTATOR_CLK_MDSS_AXI);
			if (ret)
				goto error_mdss_axi;
			ret = sde_rotator_enable_clk(mgr,
						SDE_ROTATOR_CLK_MDSS_ROT);
			if (ret)
				goto error_mdss_rot;
			ret = sde_rotator_enable_clk(mgr,
						SDE_ROTATOR_CLK_MDSS_ROT_SUB);
			if (ret)
				goto error_rot_sub;

			/* Active+Sleep */
			msm_bus_scale_client_update_context(
				mgr->data_bus.bus_hdl, false,
				mgr->data_bus.curr_bw_uc_idx);
			trace_rot_bw_ao_as_context(0);
		} else {
			sde_rotator_disable_clk(mgr,
					SDE_ROTATOR_CLK_MDSS_ROT_SUB);
			sde_rotator_disable_clk(mgr, SDE_ROTATOR_CLK_MDSS_ROT);
			sde_rotator_disable_clk(mgr, SDE_ROTATOR_CLK_MDSS_AXI);
			sde_rotator_disable_clk(mgr, SDE_ROTATOR_CLK_MDSS_AHB);
			sde_rotator_disable_clk(mgr, SDE_ROTATOR_CLK_GCC_AXI);
			sde_rotator_disable_clk(mgr, SDE_ROTATOR_CLK_GCC_AHB);
			sde_rotator_disable_clk(mgr, SDE_ROTATOR_CLK_MNOC_AHB);

			/* Active Only */
			msm_bus_scale_client_update_context(
				mgr->data_bus.bus_hdl, true,
				mgr->data_bus.curr_bw_uc_idx);
			trace_rot_bw_ao_as_context(1);
		}
	}

	return ret;
error_rot_sub:
	sde_rotator_disable_clk(mgr, SDE_ROTATOR_CLK_MDSS_ROT);
error_mdss_rot:
	sde_rotator_disable_clk(mgr, SDE_ROTATOR_CLK_MDSS_AXI);
error_mdss_axi:
	sde_rotator_disable_clk(mgr, SDE_ROTATOR_CLK_MDSS_AHB);
error_mdss_ahb:
	sde_rotator_disable_clk(mgr, SDE_ROTATOR_CLK_GCC_AXI);
error_gcc_axi:
	sde_rotator_disable_clk(mgr, SDE_ROTATOR_CLK_GCC_AHB);
error_gcc_ahb:
	sde_rotator_disable_clk(mgr, SDE_ROTATOR_CLK_MNOC_AHB);
error_mnoc_ahb:
	return ret;
}

/* sde_rotator_resource_ctrl - control state of power resource
 * @mgr: Pointer to rotator manager
 * @enable: 1 to enable; 0 to disable
 *
 * This function returns 1 if resource is already in the requested state,
 * return 0 if the state is changed successfully, or negative error code
 * if not successful.
 */
static int sde_rotator_resource_ctrl(struct sde_rot_mgr *mgr, int enable)
{
	int ret;

	if (enable) {
		mgr->res_ref_cnt++;
		ret = pm_runtime_get_sync(&mgr->pdev->dev);
	} else {
		mgr->res_ref_cnt--;
		ret = pm_runtime_put_sync(&mgr->pdev->dev);
	}

	SDEROT_DBG("%s: res_cnt=%d pm=%d enable=%d\n",
		__func__, mgr->res_ref_cnt, ret, enable);
	ATRACE_INT("res_cnt", mgr->res_ref_cnt);

	return ret;
}

/* caller is expected to hold perf->work_dis_lock lock */
static bool sde_rotator_is_work_pending(struct sde_rot_mgr *mgr,
	struct sde_rot_perf *perf)
{
	int i;

	for (i = 0; i < mgr->queue_count; i++) {
		if (perf->work_distribution[i]) {
			SDEROT_DBG("Work is still scheduled to complete\n");
			return true;
		}
	}
	return false;
}

static void sde_rotator_clear_fence(struct sde_rot_entry *entry)
{
	if (entry->input_fence) {
		SDEROT_EVTLOG(entry->input_fence, 1111);
		SDEROT_DBG("sys_fence_put i:%p\n", entry->input_fence);
		sde_rotator_put_sync_fence(entry->input_fence);
		entry->input_fence = NULL;
	}

	/* fence failed to copy to user space */
	if (entry->output_fence) {
		if (entry->fenceq && entry->fenceq->timeline)
			sde_rotator_resync_timeline(entry->fenceq->timeline);

		SDEROT_EVTLOG(entry->output_fence, 2222);
		SDEROT_DBG("sys_fence_put o:%p\n", entry->output_fence);
		sde_rotator_put_sync_fence(entry->output_fence);
		entry->output_fence = NULL;
	}
}

static int sde_rotator_signal_output(struct sde_rot_entry *entry)
{
	struct sde_rot_timeline *rot_timeline;

	if (!entry->fenceq)
		return -EINVAL;

	rot_timeline = entry->fenceq->timeline;

	if (entry->output_signaled) {
		SDEROT_DBG("output already signaled\n");
		return 0;
	}

	SDEROT_DBG("signal fence s:%d.%d\n", entry->item.session_id,
			entry->item.sequence_id);

	sde_rotator_inc_timeline(rot_timeline, 1);

	entry->output_signaled = true;

	return 0;
}

static int sde_rotator_import_buffer(struct sde_layer_buffer *buffer,
	struct sde_mdp_data *data, u32 flags, struct device *dev, bool input)
{
	int i, ret = 0;
	struct sde_fb_data planes[SDE_ROT_MAX_PLANES];
	int dir = DMA_TO_DEVICE;

	if (!input)
		dir = DMA_FROM_DEVICE;

	if (buffer->plane_count > SDE_ROT_MAX_PLANES) {
		SDEROT_ERR("buffer plane_count exceeds MAX_PLANE limit:%d\n",
				buffer->plane_count);
		return -EINVAL;
	}

	data->sbuf = buffer->sbuf;
	data->scid = buffer->scid;
	data->writeback = buffer->writeback;

	memset(planes, 0, sizeof(planes));

	for (i = 0; i < buffer->plane_count; i++) {
		planes[i].memory_id = buffer->planes[i].fd;
		planes[i].offset = buffer->planes[i].offset;
		planes[i].buffer = buffer->planes[i].buffer;
		planes[i].addr = buffer->planes[i].addr;
		planes[i].len = buffer->planes[i].len;
	}

	ret =  sde_mdp_data_get_and_validate_size(data, planes,
			buffer->plane_count, flags, dev, true, dir, buffer);

	return ret;
}

static int sde_rotator_secure_session_ctrl(bool enable)
{
	struct sde_rot_data_type *mdata = sde_rot_get_mdata();
	uint32_t *sid_info = NULL;
	struct scm_desc desc = {0};
	unsigned int resp = 0;
	int ret = 0;

	if (test_bit(SDE_CAPS_SEC_ATTACH_DETACH_SMMU, mdata->sde_caps_map)) {

		sid_info = kzalloc(sizeof(uint32_t), GFP_KERNEL);
		if (!sid_info)
			return -ENOMEM;

		sid_info[0] = mdata->sde_smmu[SDE_IOMMU_DOMAIN_ROT_SECURE].sid;
		desc.arginfo = SCM_ARGS(4, SCM_VAL, SCM_RW, SCM_VAL, SCM_VAL);
		desc.args[0] = SDE_ROTATOR_DEVICE;
		desc.args[1] = SCM_BUFFER_PHYS(sid_info);
		desc.args[2] = sizeof(uint32_t);

		if (!mdata->sec_cam_en && enable) {
			/*
			 * Enable secure camera operation
			 * Send SCM call to hypervisor to switch the
			 * secure_vmid to secure context
			 */
			desc.args[3] = VMID_CP_CAMERA_PREVIEW;

			mdata->sec_cam_en = 1;
			sde_smmu_secure_ctrl(0);

			dmac_flush_range(sid_info, sid_info + 1);
			ret = scm_call2(SCM_SIP_FNID(SCM_SVC_MP,
					MEM_PROTECT_SD_CTRL_SWITCH), &desc);
			resp = desc.ret[0];
			if (ret) {
				SDEROT_ERR("scm_call(1) ret=%d, resp=%x\n",
					ret, resp);
				/* failure, attach smmu */
				mdata->sec_cam_en = 0;
				sde_smmu_secure_ctrl(1);
				ret = -EINVAL;
				goto end;
			}

			SDEROT_DBG(
			  "scm(1) sid0x%x dev0x%llx vmid0x%llx ret%d resp%x\n",
				sid_info[0], desc.args[0], desc.args[3],
				ret, resp);
			SDEROT_EVTLOG(1, sid_info, sid_info[0],
					desc.args[0], desc.args[3],
					ret, resp);
		} else if (mdata->sec_cam_en && !enable) {
			/*
			 * Disable secure camera operation
			 * Send SCM call to hypervisor to switch the
			 * secure_vmid to non-secure context
			 */
			desc.args[3] = VMID_CP_PIXEL;
			mdata->sec_cam_en = 0;

			dmac_flush_range(sid_info, sid_info + 1);
			ret = scm_call2(SCM_SIP_FNID(SCM_SVC_MP,
				MEM_PROTECT_SD_CTRL_SWITCH), &desc);
			resp = desc.ret[0];

			SDEROT_DBG(
			  "scm(0) sid0x%x dev0x%llx vmid0x%llx ret%d resp%d\n",
				sid_info[0], desc.args[0], desc.args[3],
				ret, resp);

			/* force smmu to reattach */
			sde_smmu_secure_ctrl(1);

			SDEROT_EVTLOG(0, sid_info, sid_info[0],
					desc.args[0], desc.args[3],
					ret, resp);
		}
	} else {
		return 0;
	}

end:
	kfree(sid_info);

	if (ret)
		return ret;

	return resp;
}


static int sde_rotator_map_and_check_data(struct sde_rot_entry *entry)
{
	int ret;
	struct sde_layer_buffer *input;
	struct sde_layer_buffer *output;
	struct sde_mdp_format_params *in_fmt, *out_fmt;
	struct sde_mdp_plane_sizes ps;
	bool rotation;
	bool secure;

	input = &entry->item.input;
	output = &entry->item.output;

	rotation = (entry->item.flags &  SDE_ROTATION_90) ? true : false;

	ret = sde_smmu_ctrl(1);
	if (ret < 0)
		return ret;

	secure = (entry->item.flags & SDE_ROTATION_SECURE_CAMERA) ?
			true : false;
	ret = sde_rotator_secure_session_ctrl(secure);
	if (ret) {
		SDEROT_ERR("failed secure session enabling/disabling %d\n",
			ret);
		goto end;
	}

	in_fmt = sde_get_format_params(input->format);
	if (!in_fmt) {
		SDEROT_ERR("invalid input format:%d\n", input->format);
		ret = -EINVAL;
		goto end;
	}

	out_fmt = sde_get_format_params(output->format);
	if (!out_fmt) {
		SDEROT_ERR("invalid output format:%d\n", output->format);
		ret = -EINVAL;
		goto end;
	}

	/* if error during map, the caller will release the data */
	ret = sde_mdp_data_map(&entry->src_buf, true, DMA_TO_DEVICE);
	if (ret) {
		SDEROT_ERR("source buffer mapping failed ret:%d\n", ret);
		goto end;
	}

	ret = sde_mdp_data_map(&entry->dst_buf, true, DMA_FROM_DEVICE);
	if (ret) {
		SDEROT_ERR("destination buffer mapping failed ret:%d\n", ret);
		goto end;
	}

	ret = sde_mdp_get_plane_sizes(
			in_fmt, input->width, input->height, &ps, 0, rotation);
	if (ret) {
		SDEROT_ERR("fail to get input plane size ret=%d\n", ret);
		goto end;
	}

	ret = sde_mdp_data_check(&entry->src_buf, &ps, in_fmt);
	if (ret) {
		SDEROT_ERR("fail to check input data ret=%d\n", ret);
		goto end;
	}

	ret = sde_mdp_get_plane_sizes(out_fmt, output->width, output->height,
			&ps, 0, rotation);
	if (ret) {
		SDEROT_ERR("fail to get output plane size ret=%d\n", ret);
		goto end;
	}

	ret = sde_mdp_data_check(&entry->dst_buf, &ps, out_fmt);
	if (ret) {
		SDEROT_ERR("fail to check output data ret=%d\n", ret);
		goto end;
	}

end:
	sde_smmu_ctrl(0);

	return ret;
}

static struct sde_rot_perf *__sde_rotator_find_session(
	struct sde_rot_file_private *private,
	u32 session_id)
{
	struct sde_rot_perf *perf, *perf_next;
	bool found = false;

	list_for_each_entry_safe(perf, perf_next, &private->perf_list, list) {
		if (perf->config.session_id == session_id) {
			found = true;
			break;
		}
	}
	if (!found)
		perf = NULL;
	return perf;
}

static struct sde_rot_perf *sde_rotator_find_session(
	struct sde_rot_file_private *private,
	u32 session_id)
{
	struct sde_rot_perf *perf;

	perf = __sde_rotator_find_session(private, session_id);
	return perf;
}

static void sde_rotator_release_data(struct sde_rot_entry *entry)
{
	SDEROT_EVTLOG(entry->src_buf.p[0].addr, entry->dst_buf.p[0].addr);
	sde_mdp_data_free(&entry->src_buf, true, DMA_TO_DEVICE);
	sde_mdp_data_free(&entry->dst_buf, true, DMA_FROM_DEVICE);
}

static int sde_rotator_import_data(struct sde_rot_mgr *mgr,
	struct sde_rot_entry *entry)
{
	int ret;
	struct sde_layer_buffer *input;
	struct sde_layer_buffer *output;
	u32 flag = 0;

	input = &entry->item.input;
	output = &entry->item.output;

	if (entry->item.flags & SDE_ROTATION_SECURE)
		flag = SDE_SECURE_OVERLAY_SESSION;

	if (entry->item.flags & SDE_ROTATION_EXT_DMA_BUF)
		flag |= SDE_ROT_EXT_DMA_BUF;

	if (entry->item.flags & SDE_ROTATION_EXT_IOVA)
		flag |= SDE_ROT_EXT_IOVA;

	if (entry->item.flags & SDE_ROTATION_SECURE_CAMERA)
		flag |= SDE_SECURE_CAMERA_SESSION;

	ret = sde_rotator_import_buffer(input, &entry->src_buf, flag,
				&mgr->pdev->dev, true);
	if (ret) {
		SDEROT_ERR("fail to import input buffer ret=%d\n", ret);
		return ret;
	}

	/*
	 * driver assumes output buffer is ready to be written
	 * immediately
	 */
	ret = sde_rotator_import_buffer(output, &entry->dst_buf, flag,
				&mgr->pdev->dev, false);
	if (ret) {
		SDEROT_ERR("fail to import output buffer ret=%d\n", ret);
		return ret;
	}

	return ret;
}

/*
 * sde_rotator_require_reconfiguration - check if reconfiguration is required
 * @mgr: Pointer to rotator manager
 * @hw: Pointer to rotator hw resource
 * @entry: Pointer to next rotation entry
 *
 * Parameters are validated by caller.
 */
static int sde_rotator_require_reconfiguration(struct sde_rot_mgr *mgr,
		struct sde_rot_hw_resource *hw, struct sde_rot_entry *entry)
{
	/* OT setting change may impact queued entries */
	if (entry->perf && (entry->perf->rdot_limit != mgr->rdot_limit ||
			entry->perf->wrot_limit != mgr->wrot_limit))
		return true;

	/* sbuf mode is exclusive and may impact queued entries */
	if (!mgr->sbuf_ctx && entry->perf && entry->perf->config.output.sbuf)
		return true;

	return false;
}

/*
 * sde_rotator_is_hw_idle - check if hw block is not processing request
 * @mgr: Pointer to rotator manager
 * @hw: Pointer to rotator hw resource
 *
 * Parameters are validated by caller.
 */
static int sde_rotator_is_hw_idle(struct sde_rot_mgr *mgr,
		struct sde_rot_hw_resource *hw)
{
	int i;

	/*
	 * Wait until all queues are idle in order to update global
	 * setting such as VBIF QoS.  This check can be relaxed if global
	 * settings can be updated individually by entries already
	 * queued in hw queue, i.e. REGDMA can update VBIF directly.
	 */
	for (i = 0; i < mgr->queue_count; i++) {
		struct sde_rot_hw_resource *hw_res = mgr->commitq[i].hw;

		if (hw_res && atomic_read(&hw_res->num_active))
			return false;
	}

	return true;
}

/*
 * sde_rotator_is_hw_available - check if hw is available for the given entry
 * @mgr: Pointer to rotator manager
 * @hw: Pointer to rotator hw resource
 * @entry: Pointer to rotation entry
 *
 * Parameters are validated by caller.
 */
static int sde_rotator_is_hw_available(struct sde_rot_mgr *mgr,
		struct sde_rot_hw_resource *hw, struct sde_rot_entry *entry)
{
	/*
	 * Wait until hw is idle if reconfiguration is required; otherwise,
	 * wait until free queue entry is available
	 */
	if (sde_rotator_require_reconfiguration(mgr, hw, entry)) {
		SDEROT_DBG(
			"wait4idle active=%d pending=%d rdot:%u/%u wrot:%u/%u s:%d.%d\n",
				atomic_read(&hw->num_active), hw->pending_count,
				mgr->rdot_limit, entry->perf->rdot_limit,
				mgr->wrot_limit, entry->perf->wrot_limit,
				entry->item.session_id,
				entry->item.sequence_id);
		return sde_rotator_is_hw_idle(mgr, hw);
	} else if (mgr->sbuf_ctx && mgr->sbuf_ctx != entry->private) {
		SDEROT_DBG("wait until sbuf mode is off\n");
		return false;
	} else {
		return (atomic_read(&hw->num_active) < hw->max_active);
	}
}

/*
 * sde_rotator_req_wait_for_idle - wait for hw for a request to be idle
 * @mgr: Pointer to rotator manager
 * @req: Pointer to rotation request
 */
static void sde_rotator_req_wait_for_idle(struct sde_rot_mgr *mgr,
		struct sde_rot_entry_container *req)
{
	struct sde_rot_queue *queue;
	struct sde_rot_hw_resource *hw;
	int i, ret;

	if (!mgr || !req) {
		SDEROT_ERR("invalid params\n");
		return;
	}

	for (i = 0; i < req->count; i++) {
		queue = req->entries[i].commitq;
		if (!queue || !queue->hw)
			continue;
		hw = queue->hw;
		while (atomic_read(&hw->num_active) > 1) {
			sde_rot_mgr_unlock(mgr);
			ret = wait_event_timeout(hw->wait_queue,
				atomic_read(&hw->num_active) <= 1,
				msecs_to_jiffies(mgr->hwacquire_timeout));
			sde_rot_mgr_lock(mgr);
			if (!ret) {
				SDEROT_ERR(
					"timeout waiting for hw idle, a:%d\n",
					atomic_read(&hw->num_active));
				return;
			}
		}
	}
}

/*
 * sde_rotator_get_hw_resource - block waiting for hw availability or timeout
 * @queue: Pointer to rotator queue
 * @entry: Pointer to rotation entry
 */
static struct sde_rot_hw_resource *sde_rotator_get_hw_resource(
	struct sde_rot_queue *queue, struct sde_rot_entry *entry)
{
	struct sde_rot_hw_resource *hw;
	struct sde_rot_mgr *mgr;
	int ret;

	if (!queue || !entry || !queue->hw) {
		SDEROT_ERR("null parameters\n");
		return NULL;
	}

	hw = queue->hw;
	mgr = entry->private->mgr;

	WARN_ON(atomic_read(&hw->num_active) > hw->max_active);
	while (!sde_rotator_is_hw_available(mgr, hw, entry)) {
		sde_rot_mgr_unlock(mgr);
		ret = wait_event_timeout(hw->wait_queue,
			sde_rotator_is_hw_available(mgr, hw, entry),
			msecs_to_jiffies(mgr->hwacquire_timeout));
		sde_rot_mgr_lock(mgr);
		if (!ret) {
			SDEROT_ERR(
				"timeout waiting for hw resource, a:%d p:%d\n",
				atomic_read(&hw->num_active),
				hw->pending_count);
			SDEROT_EVTLOG(entry->item.session_id,
					entry->item.sequence_id,
					atomic_read(&hw->num_active),
					hw->pending_count,
					SDE_ROT_EVTLOG_ERROR);
			return NULL;
		}
	}
	atomic_inc(&hw->num_active);
	SDEROT_EVTLOG(atomic_read(&hw->num_active), hw->pending_count,
			mgr->rdot_limit, entry->perf->rdot_limit,
			mgr->wrot_limit, entry->perf->wrot_limit,
			entry->item.session_id, entry->item.sequence_id);
	SDEROT_DBG("active=%d pending=%d rdot=%u/%u wrot=%u/%u s:%d.%d\n",
			atomic_read(&hw->num_active), hw->pending_count,
			mgr->rdot_limit, entry->perf->rdot_limit,
			mgr->wrot_limit, entry->perf->wrot_limit,
			entry->item.session_id, entry->item.sequence_id);
	mgr->rdot_limit = entry->perf->rdot_limit;
	mgr->wrot_limit = entry->perf->wrot_limit;

	if (!mgr->sbuf_ctx && entry->perf->config.output.sbuf) {
		SDEROT_DBG("acquire sbuf s:%d.%d\n", entry->item.session_id,
				entry->item.sequence_id);
		SDEROT_EVTLOG(entry->item.session_id, entry->item.sequence_id);
		mgr->sbuf_ctx = entry->private;
	}

	return hw;
}

/*
 * sde_rotator_put_hw_resource - return hw resource and wake up waiting clients
 * @queue: Pointer to rotator queue
 * @entry: Pointer to rotation entry
 * @hw: Pointer to hw resource to be returned
 */
static void sde_rotator_put_hw_resource(struct sde_rot_queue *queue,
		struct sde_rot_entry *entry, struct sde_rot_hw_resource *hw)
{
	struct sde_rot_mgr *mgr;
	int i;

	if (!queue || !entry || !hw) {
		SDEROT_ERR("null parameters\n");
		return;
	}

	mgr = entry->private->mgr;

	WARN_ON(atomic_read(&hw->num_active) < 1);
	if (!atomic_add_unless(&hw->num_active, -1, 0))
		SDEROT_ERR("underflow active=%d pending=%d s:%d.%d\n",
			atomic_read(&hw->num_active), hw->pending_count,
			entry->item.session_id, entry->item.sequence_id);
	/*
	 * Wake up all queues in case any entry is waiting for hw idle,
	 * in order to update global settings, such as VBIF QoS.
	 * This can be relaxed to the given hw resource if global
	 * settings can be updated individually by entries already
	 * queued in hw queue.
	 */
	for (i = 0; i < mgr->queue_count; i++) {
		struct sde_rot_hw_resource *hw_res = mgr->commitq[i].hw;

		if (hw_res)
			wake_up(&hw_res->wait_queue);
	}
	SDEROT_EVTLOG(atomic_read(&hw->num_active), hw->pending_count,
			entry->item.session_id, entry->item.sequence_id);
	SDEROT_DBG("active=%d pending=%d s:%d.%d\n",
			atomic_read(&hw->num_active), hw->pending_count,
			entry->item.session_id, entry->item.sequence_id);
}

/*
 * caller will need to call sde_rotator_deinit_queue when
 * the function returns error
 */
static int sde_rotator_init_queue(struct sde_rot_mgr *mgr)
{
	int i, size, ret = 0;
	char name[32];
	struct sched_param param = { .sched_priority = 5 };

	size = sizeof(struct sde_rot_queue) * mgr->queue_count;
	mgr->commitq = devm_kzalloc(mgr->device, size, GFP_KERNEL);
	if (!mgr->commitq)
		return -ENOMEM;

	for (i = 0; i < mgr->queue_count; i++) {
		snprintf(name, sizeof(name), "rot_commitq_%d_%d",
				mgr->device->id, i);
		SDEROT_DBG("work queue name=%s\n", name);
		kthread_init_worker(&mgr->commitq[i].rot_kw);
		mgr->commitq[i].rot_thread = kthread_run(kthread_worker_fn,
				&mgr->commitq[i].rot_kw, name);
		if (IS_ERR(mgr->commitq[i].rot_thread)) {
			ret = -EPERM;
			mgr->commitq[i].rot_thread = NULL;
			break;
		}

		ret = sched_setscheduler(mgr->commitq[i].rot_thread,
			SCHED_FIFO, &param);
		if (ret) {
			SDEROT_ERR(
				"failed to set kthread priority for commitq %d\n",
				ret);
			break;
		}

		/* timeline not used */
		mgr->commitq[i].timeline = NULL;
	}

	size = sizeof(struct sde_rot_queue) * mgr->queue_count;
	mgr->doneq = devm_kzalloc(mgr->device, size, GFP_KERNEL);
	if (!mgr->doneq)
		return -ENOMEM;

	for (i = 0; i < mgr->queue_count; i++) {
		snprintf(name, sizeof(name), "rot_doneq_%d_%d",
				mgr->device->id, i);
		SDEROT_DBG("work queue name=%s\n", name);
		kthread_init_worker(&mgr->doneq[i].rot_kw);
		mgr->doneq[i].rot_thread = kthread_run(kthread_worker_fn,
				&mgr->doneq[i].rot_kw, name);
		if (IS_ERR(mgr->doneq[i].rot_thread)) {
			ret = -EPERM;
			mgr->doneq[i].rot_thread = NULL;
			break;
		}

		ret = sched_setscheduler(mgr->doneq[i].rot_thread,
			SCHED_FIFO, &param);
		if (ret) {
			SDEROT_ERR(
				"failed to set kthread priority for doneq %d\n",
				ret);
			break;
		}

		/* timeline not used */
		mgr->doneq[i].timeline = NULL;
	}
	return ret;
}

static void sde_rotator_deinit_queue(struct sde_rot_mgr *mgr)
{
	int i;

	if (mgr->commitq) {
		for (i = 0; i < mgr->queue_count; i++) {
			if (mgr->commitq[i].rot_thread) {
				kthread_flush_worker(&mgr->commitq[i].rot_kw);
				kthread_stop(mgr->commitq[i].rot_thread);
			}
		}
		devm_kfree(mgr->device, mgr->commitq);
		mgr->commitq = NULL;
	}
	if (mgr->doneq) {
		for (i = 0; i < mgr->queue_count; i++) {
			if (mgr->doneq[i].rot_thread) {
				kthread_flush_worker(&mgr->doneq[i].rot_kw);
				kthread_stop(mgr->doneq[i].rot_thread);
			}
		}
		devm_kfree(mgr->device, mgr->doneq);
		mgr->doneq = NULL;
	}
	mgr->queue_count = 0;
}

/*
 * sde_rotator_assign_queue() - Function assign rotation work onto hw
 * @mgr:	Rotator manager.
 * @entry:	Contains details on rotator work item being requested
 * @private:	Private struct used for access rot session performance struct
 *
 * This Function allocates hw required to complete rotation work item
 * requested.
 *
 * Caller is responsible for calling cleanup function if error is returned
 */
static int sde_rotator_assign_queue(struct sde_rot_mgr *mgr,
	struct sde_rot_entry *entry,
	struct sde_rot_file_private *private)
{
	struct sde_rot_perf *perf;
	struct sde_rot_queue *queue;
	struct sde_rot_hw_resource *hw;
	struct sde_rotation_item *item = &entry->item;
	u32 wb_idx = item->wb_idx;
	u32 pipe_idx = item->pipe_idx;
	int ret = 0;

	if (wb_idx >= mgr->queue_count) {
		/* assign to the lowest priority queue */
		wb_idx = mgr->queue_count - 1;
	}

	entry->doneq = &mgr->doneq[wb_idx];
	entry->commitq = &mgr->commitq[wb_idx];
	queue = mgr->commitq;

	if (!queue->hw) {
		hw = mgr->ops_hw_alloc(mgr, pipe_idx, wb_idx);
		if (IS_ERR_OR_NULL(hw)) {
			SDEROT_ERR("fail to allocate hw\n");
			ret = PTR_ERR(hw);
		} else {
			queue->hw = hw;
		}
	}

	if (queue->hw) {
		entry->commitq = queue;
		queue->hw->pending_count++;
	}

	perf = sde_rotator_find_session(private, item->session_id);
	if (!perf) {
		SDEROT_ERR(
			"Could not find session based on rotation work item\n");
		return -EINVAL;
	}

	entry->perf = perf;
	perf->last_wb_idx = wb_idx;

	return ret;
}

static void sde_rotator_unassign_queue(struct sde_rot_mgr *mgr,
	struct sde_rot_entry *entry)
{
	struct sde_rot_queue *queue = entry->commitq;

	if (!queue)
		return;

	entry->fenceq = NULL;
	entry->commitq = NULL;
	entry->doneq = NULL;

	if (!queue->hw) {
		SDEROT_ERR("entry assigned a queue with no hw\n");
		return;
	}

	queue->hw->pending_count--;
	if (queue->hw->pending_count == 0) {
		mgr->ops_hw_free(mgr, queue->hw);
		queue->hw = NULL;
	}
}

void sde_rotator_queue_request(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private,
	struct sde_rot_entry_container *req)
{
	struct sde_rot_entry *entry;
	struct sde_rot_queue *queue;
	u32 wb_idx;
	int i;

	if (!mgr || !private || !req) {
		SDEROT_ERR("null parameters\n");
		return;
	}

	if (!req->entries) {
		SDEROT_DBG("no entries in request\n");
		return;
	}

	for (i = 0; i < req->count; i++) {
		entry = req->entries + i;
		queue = entry->commitq;
		wb_idx = queue->hw->wb_id;
		entry->perf->work_distribution[wb_idx]++;
		entry->work_assigned = true;
	}

	for (i = 0; i < req->count; i++) {
		entry = req->entries + i;
		queue = entry->commitq;
		entry->output_fence = NULL;

		if (entry->item.ts)
			entry->item.ts[SDE_ROTATOR_TS_QUEUE] = ktime_get();
		kthread_queue_work(&queue->rot_kw, &entry->commit_work);
	}
}

static u32 sde_rotator_calc_buf_bw(struct sde_mdp_format_params *fmt,
		uint32_t width, uint32_t height, uint32_t frame_rate)
{
	u32 bw;

	bw = width * height * frame_rate;

	if (sde_mdp_is_tp10_format(fmt))
		bw *= 2;
	else if (sde_mdp_is_p010_format(fmt))
		bw *= 3;
	else if (fmt->chroma_sample == SDE_MDP_CHROMA_420)
		bw = (bw * 3) / 2;
	else
		bw *= fmt->bpp;
	SDEROT_EVTLOG(bw, width, height, frame_rate, fmt->format);
	return bw;
}

static int sde_rotator_find_max_fps(struct sde_rot_mgr *mgr)
{
	struct sde_rot_file_private *priv;
	struct sde_rot_perf *perf;
	int max_fps = 0;

	list_for_each_entry(priv, &mgr->file_list, list) {
		list_for_each_entry(perf, &priv->perf_list, list) {
			if (perf->config.frame_rate > max_fps)
				max_fps = perf->config.frame_rate;
		}
	}

	SDEROT_DBG("Max fps:%d\n", max_fps);
	return max_fps;
}

static int sde_rotator_calc_perf(struct sde_rot_mgr *mgr,
		struct sde_rot_perf *perf)
{
	struct sde_rotation_config *config = &perf->config;
	u32 read_bw, write_bw;
	struct sde_mdp_format_params *in_fmt, *out_fmt;
	struct sde_rotator_device *rot_dev;
	int max_fps;

	rot_dev = platform_get_drvdata(mgr->pdev);

	in_fmt = sde_get_format_params(config->input.format);
	if (!in_fmt) {
		SDEROT_ERR("invalid input format %d\n", config->input.format);
		return -EINVAL;
	}
	out_fmt = sde_get_format_params(config->output.format);
	if (!out_fmt) {
		SDEROT_ERR("invalid output format %d\n", config->output.format);
		return -EINVAL;
	}

	/*
	 * rotator processes 4 pixels per clock, but the actual throughtput
	 * is 3.6. We also need to take into account for overhead time. Final
	 * equation is:
	 *        W x H / throughput / (1/fps - overhead) * fudge_factor
	 */
	max_fps = sde_rotator_find_max_fps(mgr);
	perf->clk_rate = config->input.width * config->input.height;
	perf->clk_rate = (perf->clk_rate * mgr->pixel_per_clk.denom) /
			mgr->pixel_per_clk.numer;
	perf->clk_rate *= max_fps;
	perf->clk_rate = (perf->clk_rate * mgr->fudge_factor.numer) /
			mgr->fudge_factor.denom;
	perf->clk_rate *= mgr->overhead.denom;

	/*
	 * check for override overhead default value
	 */
	if (rot_dev->min_overhead_us > (mgr->overhead.numer * 100))
		perf->clk_rate = DIV_ROUND_UP_ULL(perf->clk_rate,
				(mgr->overhead.denom - max_fps *
				(rot_dev->min_overhead_us / 100)));
	else
		perf->clk_rate = DIV_ROUND_UP_ULL(perf->clk_rate,
				(mgr->overhead.denom - max_fps *
				mgr->overhead.numer));

	/* use client provided clock if specified */
	if (config->flags & SDE_ROTATION_EXT_PERF)
		perf->clk_rate = config->clk_rate;

	/*
	 * check for Override clock calculation
	 */
	if (rot_dev->min_rot_clk > perf->clk_rate)
		perf->clk_rate = rot_dev->min_rot_clk;

	if (mgr->min_rot_clk > perf->clk_rate)
		perf->clk_rate = mgr->min_rot_clk;

	if (mgr->max_rot_clk && (perf->clk_rate > mgr->max_rot_clk)) {
		SDEROT_ERR("invalid clock:%ld exceeds max:%ld allowed\n",
				perf->clk_rate, mgr->max_rot_clk);
		return -EINVAL;
	}

	read_bw =  sde_rotator_calc_buf_bw(in_fmt, config->input.width,
				config->input.height, max_fps);

	write_bw = sde_rotator_calc_buf_bw(out_fmt, config->output.width,
				config->output.height, max_fps);

	read_bw = sde_apply_comp_ratio_factor(read_bw, in_fmt,
			&config->input.comp_ratio);
	write_bw = sde_apply_comp_ratio_factor(write_bw, out_fmt,
			&config->output.comp_ratio);

	perf->bw = read_bw + write_bw;

	/*
	 * check for override bw calculation
	 */
	if (rot_dev->min_bw > perf->bw)
		perf->bw = rot_dev->min_bw;

	/* use client provided bandwidth if specified */
	if (config->flags & SDE_ROTATION_EXT_PERF)
		perf->bw = config->data_bw;

	perf->rdot_limit = sde_mdp_get_ot_limit(
			config->input.width, config->input.height,
			config->input.format, config->frame_rate, true);
	perf->wrot_limit = sde_mdp_get_ot_limit(
			config->input.width, config->input.height,
			config->input.format, config->frame_rate, false);

	SDEROT_DBG("clk:%lu, rdBW:%d, wrBW:%d, rdOT:%d, wrOT:%d\n",
			perf->clk_rate, read_bw, write_bw, perf->rdot_limit,
			perf->wrot_limit);
	SDEROT_EVTLOG(perf->clk_rate, read_bw, write_bw, perf->rdot_limit,
			perf->wrot_limit);
	return 0;
}

static int sde_rotator_update_perf(struct sde_rot_mgr *mgr)
{
	struct sde_rot_file_private *priv;
	struct sde_rot_perf *perf;
	int not_in_suspend_mode;
	u64 total_bw = 0;

	not_in_suspend_mode = !atomic_read(&mgr->device_suspended);

	if (not_in_suspend_mode) {
		list_for_each_entry(priv, &mgr->file_list, list) {
			list_for_each_entry(perf, &priv->perf_list, list) {
				total_bw += perf->bw;
			}
		}
	}

	total_bw += mgr->pending_close_bw_vote;
	total_bw = max_t(u64, total_bw, mgr->minimum_bw_vote);
	sde_rotator_enable_reg_bus(mgr, total_bw);
	ATRACE_INT("bus_quota", total_bw);
	sde_rotator_bus_scale_set_quota(&mgr->data_bus, total_bw);

	return 0;
}

static void sde_rotator_release_from_work_distribution(
		struct sde_rot_mgr *mgr,
		struct sde_rot_entry *entry)
{
	if (entry->work_assigned) {
		bool free_perf = false;
		u32 wb_idx = entry->commitq->hw->wb_id;

		if (entry->perf->work_distribution[wb_idx])
			entry->perf->work_distribution[wb_idx]--;

		if (!entry->perf->work_distribution[wb_idx]
				&& list_empty(&entry->perf->list)) {
			/* close session has offloaded perf free to us */
			free_perf = true;
		}

		entry->work_assigned = false;
		if (free_perf) {
			if (mgr->pending_close_bw_vote < entry->perf->bw) {
				SDEROT_ERR(
					"close bw vote underflow %llu / %llu\n",
						mgr->pending_close_bw_vote,
						entry->perf->bw);
				mgr->pending_close_bw_vote = 0;
			} else {
				mgr->pending_close_bw_vote -= entry->perf->bw;
			}
			devm_kfree(&mgr->pdev->dev,
				entry->perf->work_distribution);
			devm_kfree(&mgr->pdev->dev, entry->perf);
			sde_rotator_update_perf(mgr);
			sde_rotator_clk_ctrl(mgr, false);
			sde_rotator_resource_ctrl(mgr, false);
			entry->perf = NULL;
		}
	}
}

static void sde_rotator_release_entry(struct sde_rot_mgr *mgr,
	struct sde_rot_entry *entry)
{
	sde_rotator_release_from_work_distribution(mgr, entry);
	sde_rotator_clear_fence(entry);
	sde_rotator_release_data(entry);
	sde_rotator_unassign_queue(mgr, entry);
}

/*
 * sde_rotator_commit_handler - Commit workqueue handler.
 * @file: Pointer to work struct.
 *
 * This handler is responsible for commit the job to h/w.
 * Once the job is committed, the job entry is added to the done queue.
 *
 * Note this asynchronous handler is protected by hal lock.
 */
static void sde_rotator_commit_handler(struct kthread_work *work)
{
	struct sde_rot_entry *entry;
	struct sde_rot_entry_container *request;
	struct sde_rot_hw_resource *hw;
	struct sde_rot_mgr *mgr;
	struct sched_param param = { .sched_priority = 5 };
	int ret;

	entry = container_of(work, struct sde_rot_entry, commit_work);
	request = entry->request;

	if (!request || !entry->private || !entry->private->mgr) {
		SDEROT_ERR("fatal error, null request/context/device\n");
		return;
	}

	ret = sched_setscheduler(entry->fenceq->rot_thread, SCHED_FIFO, &param);
	if (ret) {
		SDEROT_WARN("Fail to set kthread priority for fenceq: %d\n",
				ret);
	}

	mgr = entry->private->mgr;

	SDEROT_EVTLOG(
		entry->item.session_id, entry->item.sequence_id,
		entry->item.src_rect.x, entry->item.src_rect.y,
		entry->item.src_rect.w, entry->item.src_rect.h,
		entry->item.dst_rect.x, entry->item.dst_rect.y,
		entry->item.dst_rect.w, entry->item.dst_rect.h,
		entry->item.flags,
		entry->dnsc_factor_w, entry->dnsc_factor_h);

	SDEDEV_DBG(mgr->device,
		"commit handler s:%d.%u src:(%d,%d,%d,%d) dst:(%d,%d,%d,%d) f:0x%x dnsc:%u/%u\n",
		entry->item.session_id, entry->item.sequence_id,
		entry->item.src_rect.x, entry->item.src_rect.y,
		entry->item.src_rect.w, entry->item.src_rect.h,
		entry->item.dst_rect.x, entry->item.dst_rect.y,
		entry->item.dst_rect.w, entry->item.dst_rect.h,
		entry->item.flags,
		entry->dnsc_factor_w, entry->dnsc_factor_h);

	sde_rot_mgr_lock(mgr);

	hw = sde_rotator_get_hw_resource(entry->commitq, entry);
	if (!hw) {
		SDEROT_ERR("no hw for the queue\n");
		goto get_hw_res_err;
	}

	if (entry->item.ts)
		entry->item.ts[SDE_ROTATOR_TS_COMMIT] = ktime_get();

	trace_rot_entry_commit(
		entry->item.session_id, entry->item.sequence_id,
		entry->item.wb_idx, entry->item.flags,
		entry->item.input.format,
		entry->item.input.width, entry->item.input.height,
		entry->item.src_rect.x, entry->item.src_rect.y,
		entry->item.src_rect.w, entry->item.src_rect.h,
		entry->item.output.format,
		entry->item.output.width, entry->item.output.height,
		entry->item.dst_rect.x, entry->item.dst_rect.y,
		entry->item.dst_rect.w, entry->item.dst_rect.h);

	ATRACE_INT("sde_smmu_ctrl", 0);
	ret = sde_smmu_ctrl(1);
	if (ret < 0) {
		SDEROT_ERR("IOMMU attach failed\n");
		goto smmu_error;
	}
	ATRACE_INT("sde_smmu_ctrl", 1);

	ret = sde_rotator_map_and_check_data(entry);
	if (ret) {
		SDEROT_ERR("fail to prepare input/output data %d\n", ret);
		goto error;
	}

	ret = mgr->ops_config_hw(hw, entry);
	if (ret) {
		SDEROT_ERR("fail to configure hw resource %d\n", ret);
		goto error;
	}

	if (entry->item.ts)
		entry->item.ts[SDE_ROTATOR_TS_START] = ktime_get();

	ret = sde_rotator_req_wait_start(mgr, request);
	if (ret) {
		SDEROT_WARN("timeout waiting for inline start\n");
		SDEROT_EVTLOG(entry->item.session_id, entry->item.sequence_id,
				SDE_ROT_EVTLOG_ERROR);
		goto kickoff_error;
	}

	ret = mgr->ops_kickoff_entry(hw, entry);
	if (ret) {
		SDEROT_ERR("fail to do kickoff %d\n", ret);
		SDEROT_EVTLOG(entry->item.session_id, entry->item.sequence_id,
				SDE_ROT_EVTLOG_ERROR);
		goto kickoff_error;
	}

	if (entry->item.ts)
		entry->item.ts[SDE_ROTATOR_TS_FLUSH] = ktime_get();

	SDEROT_EVTLOG(entry->item.session_id, 1);

	kthread_queue_work(&entry->doneq->rot_kw, &entry->done_work);
	sde_rot_mgr_unlock(mgr);
	return;
kickoff_error:
	/*
	 * Wait for any pending operations to complete before cancelling this
	 * one so that the system is left in a consistent state.
	 */
	sde_rotator_req_wait_for_idle(mgr, request);
	mgr->ops_cancel_hw(hw, entry);
error:
	sde_smmu_ctrl(0);
smmu_error:
	sde_rotator_put_hw_resource(entry->commitq, entry, hw);
get_hw_res_err:
	sde_rotator_signal_output(entry);
	sde_rotator_release_entry(mgr, entry);
	atomic_dec(&request->pending_count);
	atomic_inc(&request->failed_count);
	if (request->retire_kw && request->retire_work)
		kthread_queue_work(request->retire_kw, request->retire_work);
	sde_rot_mgr_unlock(mgr);
}

/*
 * sde_rotator_done_handler - Done workqueue handler.
 * @file: Pointer to work struct.
 *
 * This handler is responsible for waiting for h/w done event.
 * Once the job is done, the output fence will be signaled and the job entry
 * will be retired.
 *
 * Note this asynchronous handler is protected by hal lock.
 */
static void sde_rotator_done_handler(struct kthread_work *work)
{
	struct sde_rot_entry *entry;
	struct sde_rot_entry_container *request;
	struct sde_rot_hw_resource *hw;
	struct sde_rot_mgr *mgr;
	int ret;

	entry = container_of(work, struct sde_rot_entry, done_work);
	request = entry->request;

	if (!request || !entry->private || !entry->private->mgr) {
		SDEROT_ERR("fatal error, null request/context/device\n");
		return;
	}

	mgr = entry->private->mgr;
	hw = entry->commitq->hw;

	SDEDEV_DBG(mgr->device,
		"done handler s:%d.%u src:(%d,%d,%d,%d) dst:(%d,%d,%d,%d) f:0x%x dsnc:%u/%u\n",
		entry->item.session_id, entry->item.sequence_id,
		entry->item.src_rect.x, entry->item.src_rect.y,
		entry->item.src_rect.w, entry->item.src_rect.h,
		entry->item.dst_rect.x, entry->item.dst_rect.y,
		entry->item.dst_rect.w, entry->item.dst_rect.h,
		entry->item.flags,
		entry->dnsc_factor_w, entry->dnsc_factor_h);

	SDEROT_EVTLOG(entry->item.session_id, 0);
	ret = mgr->ops_wait_for_entry(hw, entry);
	if (ret) {
		SDEROT_ERR("fail to wait for completion %d\n", ret);
		atomic_inc(&request->failed_count);
	}
	SDEROT_EVTLOG(entry->item.session_id, 1);

	if (entry->item.ts)
		entry->item.ts[SDE_ROTATOR_TS_DONE] = ktime_get();

	trace_rot_entry_done(
		entry->item.session_id, entry->item.sequence_id,
		entry->item.wb_idx, entry->item.flags,
		entry->item.input.format,
		entry->item.input.width, entry->item.input.height,
		entry->item.src_rect.x, entry->item.src_rect.y,
		entry->item.src_rect.w, entry->item.src_rect.h,
		entry->item.output.format,
		entry->item.output.width, entry->item.output.height,
		entry->item.dst_rect.x, entry->item.dst_rect.y,
		entry->item.dst_rect.w, entry->item.dst_rect.h);

	sde_rot_mgr_lock(mgr);
	sde_rotator_put_hw_resource(entry->commitq, entry, entry->commitq->hw);
	sde_rotator_signal_output(entry);
	ATRACE_INT("sde_rot_done", 1);
	sde_rotator_release_entry(mgr, entry);
	atomic_dec(&request->pending_count);
	if (request->retire_kw && request->retire_work)
		kthread_queue_work(request->retire_kw, request->retire_work);
	if (entry->item.ts)
		entry->item.ts[SDE_ROTATOR_TS_RETIRE] = ktime_get();
	sde_rot_mgr_unlock(mgr);

	ATRACE_INT("sde_smmu_ctrl", 3);
	sde_smmu_ctrl(0);
	ATRACE_INT("sde_smmu_ctrl", 4);
}

static bool sde_rotator_verify_format(struct sde_rot_mgr *mgr,
	struct sde_mdp_format_params *in_fmt,
	struct sde_mdp_format_params *out_fmt, bool rotation, u32 mode)
{
	u8 in_v_subsample, in_h_subsample;
	u8 out_v_subsample, out_h_subsample;

	if (!sde_rotator_is_valid_pixfmt(mgr, in_fmt->format, true, mode)) {
		SDEROT_ERR("Invalid input format 0x%x (%4.4s)\n",
				in_fmt->format, (char *)&in_fmt->format);
		goto verify_error;
	}

	if (!sde_rotator_is_valid_pixfmt(mgr, out_fmt->format, false, mode)) {
		SDEROT_ERR("Invalid output format 0x%x (%4.4s)\n",
				out_fmt->format, (char *)&out_fmt->format);
		goto verify_error;
	}

	if ((in_fmt->is_yuv != out_fmt->is_yuv) ||
		(in_fmt->pixel_mode != out_fmt->pixel_mode) ||
		(in_fmt->unpack_tight != out_fmt->unpack_tight)) {
		SDEROT_ERR(
			"Rotator does not support CSC yuv:%d/%d pm:%d/%d ut:%d/%d\n",
			in_fmt->is_yuv, out_fmt->is_yuv,
			in_fmt->pixel_mode, out_fmt->pixel_mode,
			in_fmt->unpack_tight, out_fmt->unpack_tight);
		goto verify_error;
	}

	/* Forcing same pixel depth */
	if (memcmp(in_fmt->bits, out_fmt->bits, sizeof(in_fmt->bits))) {
		/* Exception is that RGB can drop alpha or add X */
		if (in_fmt->is_yuv || out_fmt->alpha_enable ||
			(in_fmt->bits[C2_R_Cr] != out_fmt->bits[C2_R_Cr]) ||
			(in_fmt->bits[C0_G_Y] != out_fmt->bits[C0_G_Y]) ||
			(in_fmt->bits[C1_B_Cb] != out_fmt->bits[C1_B_Cb])) {
			SDEROT_ERR("Bit format does not match\n");
			goto verify_error;
		}
	}

	/* Need to make sure that sub-sampling persists through rotation */
	if (rotation) {
		sde_mdp_get_v_h_subsample_rate(in_fmt->chroma_sample,
			&in_v_subsample, &in_h_subsample);
		sde_mdp_get_v_h_subsample_rate(out_fmt->chroma_sample,
			&out_v_subsample, &out_h_subsample);

		if ((in_v_subsample != out_h_subsample) ||
				(in_h_subsample != out_v_subsample)) {
			SDEROT_ERR("Rotation has invalid subsampling\n");
			goto verify_error;
		}
	} else {
		if (in_fmt->chroma_sample != out_fmt->chroma_sample) {
			SDEROT_ERR("Format subsampling mismatch\n");
			goto verify_error;
		}
	}

	return true;

verify_error:
	SDEROT_ERR("in_fmt=0x%x (%4.4s), out_fmt=0x%x (%4.4s), mode=%d\n",
			in_fmt->format, (char *)&in_fmt->format,
			out_fmt->format, (char *)&out_fmt->format,
			mode);
	return false;
}

static struct sde_mdp_format_params *__verify_input_config(
		struct sde_rot_mgr *mgr,
		struct sde_rotation_config *config)
{
	struct sde_mdp_format_params *in_fmt;
	u8 in_v_subsample, in_h_subsample;
	u32 input;
	int verify_input_only;

	if (!mgr || !config) {
		SDEROT_ERR("null parameters\n");
		return NULL;
	}

	input = config->input.format;
	verify_input_only =
		(config->flags & SDE_ROTATION_VERIFY_INPUT_ONLY) ? 1 : 0;

	in_fmt = sde_get_format_params(input);
	if (!in_fmt) {
		if (!verify_input_only)
			SDEROT_ERR("Unrecognized input format:0x%x\n", input);
		return NULL;
	}

	sde_mdp_get_v_h_subsample_rate(in_fmt->chroma_sample,
		&in_v_subsample, &in_h_subsample);

	/* Dimension of image needs to be divisible by subsample rate  */
	if ((config->input.height % in_v_subsample) ||
			(config->input.width % in_h_subsample)) {
		if (!verify_input_only)
			SDEROT_ERR(
				"In ROI, subsample mismatch, w=%d, h=%d, vss%d, hss%d\n",
					config->input.width,
					config->input.height,
					in_v_subsample, in_h_subsample);
		return NULL;
	}

	return in_fmt;
}

static struct sde_mdp_format_params *__verify_output_config(
		struct sde_rot_mgr *mgr,
		struct sde_rotation_config *config)
{
	struct sde_mdp_format_params *out_fmt;
	u8 out_v_subsample, out_h_subsample;
	u32 output;
	int verify_input_only;

	if (!mgr || !config) {
		SDEROT_ERR("null parameters\n");
		return NULL;
	}

	output = config->output.format;
	verify_input_only =
		(config->flags & SDE_ROTATION_VERIFY_INPUT_ONLY) ? 1 : 0;

	out_fmt = sde_get_format_params(output);
	if (!out_fmt) {
		if (!verify_input_only)
			SDEROT_ERR("Unrecognized output format:0x%x\n", output);
		return NULL;
	}

	sde_mdp_get_v_h_subsample_rate(out_fmt->chroma_sample,
		&out_v_subsample, &out_h_subsample);

	/* Dimension of image needs to be divisible by subsample rate  */
	if ((config->output.height % out_v_subsample) ||
			(config->output.width % out_h_subsample)) {
		if (!verify_input_only)
			SDEROT_ERR(
				"Out ROI, subsample mismatch, w=%d, h=%d, vss%d, hss%d\n",
					config->output.width,
					config->output.height,
					out_v_subsample, out_h_subsample);
		return NULL;
	}

	return out_fmt;
}

int sde_rotator_verify_config_input(struct sde_rot_mgr *mgr,
		struct sde_rotation_config *config)
{
	struct sde_mdp_format_params *in_fmt;

	in_fmt = __verify_input_config(mgr, config);
	if (!in_fmt)
		return -EINVAL;

	return 0;
}

int sde_rotator_verify_config_output(struct sde_rot_mgr *mgr,
		struct sde_rotation_config *config)
{
	struct sde_mdp_format_params *out_fmt;

	out_fmt = __verify_output_config(mgr, config);
	if (!out_fmt)
		return -EINVAL;

	return 0;
}

int sde_rotator_verify_config_all(struct sde_rot_mgr *mgr,
	struct sde_rotation_config *config)
{
	struct sde_mdp_format_params *in_fmt, *out_fmt;
	bool rotation;
	u32 mode;

	if (!mgr || !config) {
		SDEROT_ERR("null parameters\n");
		return -EINVAL;
	}

	rotation = (config->flags & SDE_ROTATION_90) ? true : false;

	mode = config->output.sbuf ? SDE_ROTATOR_MODE_SBUF :
				SDE_ROTATOR_MODE_OFFLINE;

	in_fmt = __verify_input_config(mgr, config);
	if (!in_fmt)
		return -EINVAL;

	out_fmt = __verify_output_config(mgr, config);
	if (!out_fmt)
		return -EINVAL;

	if (!sde_rotator_verify_format(mgr, in_fmt, out_fmt, rotation, mode)) {
		SDEROT_ERR(
			"Rot format pairing invalid, in_fmt:0x%x, out_fmt:0x%x\n",
					config->input.format,
					config->output.format);
		return -EINVAL;
	}

	return 0;
}

static int sde_rotator_validate_item_matches_session(
	struct sde_rotation_config *config, struct sde_rotation_item *item)
{
	int ret;

	ret = __compare_session_item_rect(&config->input,
		&item->src_rect, item->input.format, true);
	if (ret)
		return ret;

	ret = __compare_session_item_rect(&config->output,
		&item->dst_rect, item->output.format, false);
	if (ret)
		return ret;

	ret = __compare_session_rotations(config->flags, item->flags);
	if (ret)
		return ret;

	return 0;
}

/* Only need to validate x and y offset for ubwc dst fmt */
static int sde_rotator_validate_img_roi(struct sde_rotation_item *item)
{
	struct sde_mdp_format_params *fmt;
	int ret = 0;

	fmt = sde_get_format_params(item->output.format);
	if (!fmt) {
		SDEROT_DBG("invalid output format:%d\n",
					item->output.format);
		return -EINVAL;
	}

	if (sde_mdp_is_ubwc_format(fmt))
		ret = sde_validate_offset_for_ubwc_format(fmt,
			item->dst_rect.x, item->dst_rect.y);

	return ret;
}

static int sde_rotator_validate_fmt_and_item_flags(
	struct sde_rotation_config *config, struct sde_rotation_item *item)
{
	struct sde_mdp_format_params *fmt;

	fmt = sde_get_format_params(item->input.format);
	if ((item->flags & SDE_ROTATION_DEINTERLACE) &&
			sde_mdp_is_ubwc_format(fmt)) {
		SDEROT_DBG("cannot perform deinterlace on tiled formats\n");
		return -EINVAL;
	}
	return 0;
}

static int sde_rotator_validate_entry(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private,
	struct sde_rot_entry *entry)
{
	int ret;
	struct sde_rotation_item *item;
	struct sde_rot_perf *perf;

	item = &entry->item;

	if (item->wb_idx >= mgr->queue_count)
		item->wb_idx = mgr->queue_count - 1;

	perf = sde_rotator_find_session(private, item->session_id);
	if (!perf) {
		SDEROT_DBG("Could not find session:%u\n", item->session_id);
		return -EINVAL;
	}

	ret = sde_rotator_validate_item_matches_session(&perf->config, item);
	if (ret) {
		SDEROT_DBG("Work item does not match session:%u\n",
					item->session_id);
		return ret;
	}

	ret = sde_rotator_validate_img_roi(item);
	if (ret) {
		SDEROT_DBG("Image roi is invalid\n");
		return ret;
	}

	ret = sde_rotator_validate_fmt_and_item_flags(&perf->config, item);
	if (ret)
		return ret;

	ret = mgr->ops_hw_validate_entry(mgr, entry);
	if (ret) {
		SDEROT_DBG("fail to configure downscale factor\n");
		return ret;
	}
	return ret;
}

/*
 * Upon failure from the function, caller needs to make sure
 * to call sde_rotator_remove_request to clean up resources.
 */
static int sde_rotator_add_request(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private,
	struct sde_rot_entry_container *req)
{
	struct sde_rot_entry *entry;
	struct sde_rotation_item *item;
	int i, ret;

	for (i = 0; i < req->count; i++) {
		entry = req->entries + i;
		item = &entry->item;
		entry->fenceq = private->fenceq;

		ret = sde_rotator_validate_entry(mgr, private, entry);
		if (ret) {
			SDEROT_ERR("fail to validate the entry\n");
			return ret;
		}

		ret = sde_rotator_import_data(mgr, entry);
		if (ret) {
			SDEROT_ERR("fail to import the data\n");
			return ret;
		}

		entry->input_fence = item->input.fence;
		entry->output_fence = item->output.fence;

		ret = sde_rotator_assign_queue(mgr, entry, private);
		if (ret) {
			SDEROT_ERR("fail to assign queue to entry\n");
			return ret;
		}

		entry->request = req;

		kthread_init_work(&entry->commit_work,
				sde_rotator_commit_handler);
		kthread_init_work(&entry->done_work,
				sde_rotator_done_handler);
		SDEROT_DBG(
			"Entry added. wbidx=%u, src{%u,%u,%u,%u}f=%x dst{%u,%u,%u,%u}f=%x session_id=%u\n",
			item->wb_idx,
			item->src_rect.x, item->src_rect.y,
			item->src_rect.w, item->src_rect.h, item->input.format,
			item->dst_rect.x, item->dst_rect.y,
			item->dst_rect.w, item->dst_rect.h, item->output.format,
			item->session_id);
	}

	list_add(&req->list, &private->req_list);

	return 0;
}

void sde_rotator_remove_request(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private,
	struct sde_rot_entry_container *req)
{
	int i;

	if (!mgr || !private || !req) {
		SDEROT_ERR("null parameters\n");
		return;
	}

	for (i = 0; i < req->count; i++)
		sde_rotator_release_entry(mgr, req->entries + i);
	list_del_init(&req->list);
}

/* This function should be called with req_lock */
static void sde_rotator_cancel_request(struct sde_rot_mgr *mgr,
	struct sde_rot_entry_container *req)
{
	struct sde_rot_entry *entry;
	int i;

	if (atomic_read(&req->pending_count)) {
		/*
		 * To avoid signal the rotation entry output fence in the wrong
		 * order, all the entries in the same request needs to be
		 * canceled first, before signaling the output fence.
		 */
		SDEROT_DBG("cancel work start\n");
		sde_rot_mgr_unlock(mgr);
		for (i = req->count - 1; i >= 0; i--) {
			entry = req->entries + i;
			kthread_cancel_work_sync(&entry->commit_work);
			kthread_cancel_work_sync(&entry->done_work);
		}
		sde_rot_mgr_lock(mgr);
		SDEROT_DBG("cancel work done\n");
		for (i = req->count - 1; i >= 0; i--) {
			entry = req->entries + i;
			sde_rotator_signal_output(entry);
			sde_rotator_release_entry(mgr, entry);
		}
	}

	list_del_init(&req->list);
	devm_kfree(&mgr->pdev->dev, req);
}

void sde_rotator_cancel_all_requests(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private)
{
	struct sde_rot_entry_container *req, *req_next;

	SDEROT_DBG("Canceling all rotator requests\n");

	list_for_each_entry_safe(req, req_next, &private->req_list, list)
		sde_rotator_cancel_request(mgr, req);
}

static void sde_rotator_free_completed_request(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private)
{
	struct sde_rot_entry_container *req, *req_next;

	list_for_each_entry_safe(req, req_next, &private->req_list, list) {
		if ((atomic_read(&req->pending_count) == 0) && req->finished) {
			list_del_init(&req->list);
			devm_kfree(&mgr->pdev->dev, req);
		}
	}
}

static void sde_rotator_release_rotator_perf_session(
	struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private)
{
	struct sde_rot_perf *perf, *perf_next;

	SDEROT_DBG("Releasing all rotator request\n");
	sde_rotator_cancel_all_requests(mgr, private);

	list_for_each_entry_safe(perf, perf_next, &private->perf_list, list) {
		list_del_init(&perf->list);
		devm_kfree(&mgr->pdev->dev, perf->work_distribution);
		devm_kfree(&mgr->pdev->dev, perf);
	}
}

static void sde_rotator_release_all(struct sde_rot_mgr *mgr)
{
	struct sde_rot_file_private *priv, *priv_next;

	list_for_each_entry_safe(priv, priv_next, &mgr->file_list, list) {
		sde_rotator_release_rotator_perf_session(mgr, priv);
		sde_rotator_resource_ctrl(mgr, false);
		list_del_init(&priv->list);
	}

	sde_rotator_update_perf(mgr);
}

int sde_rotator_validate_request(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private,
	struct sde_rot_entry_container *req)
{
	int i, ret = 0;
	struct sde_rot_entry *entry;

	if (!mgr || !private || !req) {
		SDEROT_ERR("null parameters\n");
		return -EINVAL;
	}

	for (i = 0; i < req->count; i++) {
		entry = req->entries + i;
		ret = sde_rotator_validate_entry(mgr, private,
			entry);
		if (ret) {
			SDEROT_DBG("invalid entry\n");
			return ret;
		}
	}

	return ret;
}

static int sde_rotator_open_session(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private, u32 session_id)
{
	struct sde_rotation_config config;
	struct sde_rot_perf *perf;
	int ret;

	if (!mgr || !private)
		return -EINVAL;

	memset(&config, 0, sizeof(struct sde_rotation_config));

	/* initialize with default parameters */
	config.frame_rate = 30;
	config.input.comp_ratio.numer = 1;
	config.input.comp_ratio.denom = 1;
	config.input.format = SDE_PIX_FMT_Y_CBCR_H2V2;
	config.input.width = 640;
	config.input.height = 480;
	config.output.comp_ratio.numer = 1;
	config.output.comp_ratio.denom = 1;
	config.output.format = SDE_PIX_FMT_Y_CBCR_H2V2;
	config.output.width = 640;
	config.output.height = 480;

	perf = devm_kzalloc(&mgr->pdev->dev, sizeof(*perf), GFP_KERNEL);
	if (!perf)
		return -ENOMEM;

	perf->work_distribution = devm_kzalloc(&mgr->pdev->dev,
		sizeof(u32) * mgr->queue_count, GFP_KERNEL);
	if (!perf->work_distribution) {
		ret = -ENOMEM;
		goto alloc_err;
	}

	config.session_id = session_id;
	perf->config = config;
	perf->last_wb_idx = 0;

	INIT_LIST_HEAD(&perf->list);
	list_add(&perf->list, &private->perf_list);

	ret = sde_rotator_resource_ctrl(mgr, true);
	if (ret < 0) {
		SDEROT_ERR("Failed to acquire rotator resources\n");
		goto resource_err;
	}

	ret = sde_rotator_update_clk(mgr);
	if (ret) {
		SDEROT_ERR("failed to update clk %d\n", ret);
		goto update_clk_err;
	}

	ret = sde_rotator_clk_ctrl(mgr, true);
	if (ret) {
		SDEROT_ERR("failed to enable clk %d\n", ret);
		goto enable_clk_err;
	}

	SDEROT_DBG("open session id=%u in{%u,%u}f:%u out{%u,%u}f:%u\n",
		config.session_id, config.input.width, config.input.height,
		config.input.format, config.output.width, config.output.height,
		config.output.format);

	goto done;
enable_clk_err:
update_clk_err:
	sde_rotator_resource_ctrl(mgr, false);
resource_err:
	list_del_init(&perf->list);
	devm_kfree(&mgr->pdev->dev, perf->work_distribution);
alloc_err:
	devm_kfree(&mgr->pdev->dev, perf);
done:
	return ret;
}

static int sde_rotator_close_session(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private, u32 session_id)
{
	struct sde_rot_perf *perf;
	bool offload_release_work = false;
	u32 id;

	id = (u32)session_id;
	perf = __sde_rotator_find_session(private, id);
	if (!perf) {
		SDEROT_ERR("Trying to close session that does not exist\n");
		return -EINVAL;
	}

	if (sde_rotator_is_work_pending(mgr, perf)) {
		SDEROT_DBG("Work is still pending, offload free to wq\n");
		mgr->pending_close_bw_vote += perf->bw;
		offload_release_work = true;
	}
	list_del_init(&perf->list);

	if (offload_release_work)
		goto done;

	devm_kfree(&mgr->pdev->dev, perf->work_distribution);
	devm_kfree(&mgr->pdev->dev, perf);
	sde_rotator_update_perf(mgr);
	sde_rotator_clk_ctrl(mgr, false);
	sde_rotator_update_clk(mgr);
	sde_rotator_resource_ctrl(mgr, false);
done:
	if (mgr->sbuf_ctx == private) {
		SDEROT_DBG("release sbuf session id:%u\n", id);
		SDEROT_EVTLOG(id);
		mgr->sbuf_ctx = NULL;
	}

	SDEROT_DBG("Closed session id:%u\n", id);
	return 0;
}

static int sde_rotator_config_session(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private,
	struct sde_rotation_config *config)
{
	int ret = 0;
	struct sde_rot_perf *perf;

	ret = sde_rotator_verify_config_all(mgr, config);
	if (ret) {
		SDEROT_ERR("Rotator verify format failed\n");
		return ret;
	}

	perf = sde_rotator_find_session(private, config->session_id);
	if (!perf) {
		SDEROT_ERR("No session with id=%u could be found\n",
			config->session_id);
		return -EINVAL;
	}

	perf->config = *config;
	ret = sde_rotator_calc_perf(mgr, perf);

	if (ret) {
		SDEROT_ERR("error in configuring the session %d\n", ret);
		goto done;
	}

	ret = sde_rotator_update_perf(mgr);
	if (ret) {
		SDEROT_ERR("error in updating perf: %d\n", ret);
		goto done;
	}

	ret = sde_rotator_update_clk(mgr);
	if (ret) {
		SDEROT_ERR("error in updating the rotator clk: %d\n", ret);
		goto done;
	}

	if (config->output.sbuf && mgr->sbuf_ctx != private && mgr->sbuf_ctx) {
		SDEROT_ERR("too many sbuf sessions\n");
		ret = -EBUSY;
		goto done;
	}

	SDEROT_DBG(
		"reconfig session id=%u in{%u,%u}f:%x out{%u,%u}f:%x fps:%d clk:%lu bw:%llu\n",
		config->session_id, config->input.width, config->input.height,
		config->input.format, config->output.width,
		config->output.height, config->output.format,
		config->frame_rate, perf->clk_rate, perf->bw);
	SDEROT_EVTLOG(config->session_id, config->input.width,
			config->input.height, config->input.format,
			config->output.width, config->output.height,
			config->output.format, config->frame_rate);
done:
	return ret;
}

struct sde_rot_entry_container *sde_rotator_req_init(
	struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private,
	struct sde_rotation_item *items,
	u32 count, u32 flags)
{
	struct sde_rot_entry_container *req;
	int size, i;

	if (!mgr || !private || !items) {
		SDEROT_ERR("null parameters\n");
		return ERR_PTR(-EINVAL);
	}

	size = sizeof(struct sde_rot_entry_container);
	size += sizeof(struct sde_rot_entry) * count;
	req = devm_kzalloc(&mgr->pdev->dev, size, GFP_KERNEL);

	if (!req)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&req->list);
	req->count = count;
	req->entries = (struct sde_rot_entry *)
		((void *)req + sizeof(struct sde_rot_entry_container));
	req->flags = flags;
	atomic_set(&req->pending_count, count);
	atomic_set(&req->failed_count, 0);

	for (i = 0; i < count; i++) {
		req->entries[i].item = items[i];
		req->entries[i].private = private;

		init_completion(&req->entries[i].item.inline_start);
		complete_all(&req->entries[i].item.inline_start);
	}

	return req;
}

void sde_rotator_req_reset_start(struct sde_rot_mgr *mgr,
		struct sde_rot_entry_container *req)
{
	int i;

	if (!mgr || !req)
		return;

	for (i = 0; i < req->count; i++)
		reinit_completion(&req->entries[i].item.inline_start);
}

void sde_rotator_req_set_start(struct sde_rot_mgr *mgr,
		struct sde_rot_entry_container *req)
{
	struct kthread_work *commit_work;
	int i;

	if (!mgr || !req || !req->entries)
		return;

	/* signal ready to start */
	for (i = 0; i < req->count; i++)
		complete_all(&req->entries[i].item.inline_start);

	for (i = 0; i < req->count; i++) {
		commit_work = &req->entries[i].commit_work;

		SDEROT_EVTLOG(i, req->count);

		sde_rot_mgr_unlock(mgr);
		kthread_flush_work(commit_work);
		sde_rot_mgr_lock(mgr);
	}
}

int sde_rotator_req_wait_start(struct sde_rot_mgr *mgr,
		struct sde_rot_entry_container *req)
{
	struct completion *inline_start;
	int i, ret;

	if (!mgr || !req || !req->entries)
		return -EINVAL;

	/* only wait for sbuf mode */
	if (!mgr->sbuf_ctx || !req->count ||
			mgr->sbuf_ctx != req->entries[0].private)
		return 0;

	for (i = 0; i < req->count; i++) {
		inline_start = &req->entries[i].item.inline_start;

		sde_rot_mgr_unlock(mgr);
		ret = wait_for_completion_timeout(inline_start,
			msecs_to_jiffies(ROT_INLINE_START_TIMEOUT_IN_MS));
		sde_rot_mgr_lock(mgr);
	}

	/* wait call returns zero on timeout */
	return ret ? 0 : -EBUSY;
}

void sde_rotator_req_finish(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private,
	struct sde_rot_entry_container *req)
{
	if (!mgr || !private || !req) {
		SDEROT_ERR("null parameters\n");
		return;
	}

	req->finished = true;
}

void sde_rotator_abort_inline_request(struct sde_rot_mgr *mgr,
		struct sde_rot_file_private *private,
		struct sde_rot_entry_container *req)
{
	struct kthread_work *commit_work;
	struct kthread_work *done_work;
	struct sde_rot_entry *entry;
	struct sde_rot_hw_resource *hw;
	int i;

	if (!mgr || !private || !req || !req->entries)
		return;

	for (i = 0; i < req->count; i++) {
		entry = &req->entries[i];
		if (!entry)
			continue;

		commit_work = &entry->commit_work;
		done_work = &entry->done_work;

		hw = sde_rotator_get_hw_resource(entry->commitq, entry);
		if (!hw) {
			SDEROT_ERR("no hw for the queue\n");
			SDEROT_EVTLOG(i, req->count, SDE_ROT_EVTLOG_ERROR);
			continue;
		}

		SDEROT_EVTLOG(i, req->count);

		mgr->ops_abort_hw(hw, entry);

		sde_rot_mgr_unlock(mgr);
		kthread_flush_work(commit_work);
		kthread_flush_work(done_work);
		sde_rot_mgr_lock(mgr);
	}
}

int sde_rotator_handle_request_common(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private,
	struct sde_rot_entry_container *req)
{
	int ret;

	if (!mgr || !private || !req) {
		SDEROT_ERR("null parameters\n");
		return -EINVAL;
	}

	sde_rotator_free_completed_request(mgr, private);

	ret = sde_rotator_add_request(mgr, private, req);
	if (ret) {
		SDEROT_ERR("fail to add rotation request\n");
		sde_rotator_remove_request(mgr, private, req);
		return ret;
	}
	return ret;
}

static int sde_rotator_open(struct sde_rot_mgr *mgr,
		struct sde_rot_file_private **pprivate)
{
	struct sde_rot_file_private *private;

	if (!mgr || !pprivate)
		return -ENODEV;

	if (atomic_read(&mgr->device_suspended))
		return -EPERM;

	private = devm_kzalloc(&mgr->pdev->dev, sizeof(*private),
		GFP_KERNEL);
	if (!private)
		return -ENOMEM;

	INIT_LIST_HEAD(&private->req_list);
	INIT_LIST_HEAD(&private->perf_list);
	INIT_LIST_HEAD(&private->list);

	list_add(&private->list, &mgr->file_list);

	*pprivate = private;

	return 0;
}

static bool sde_rotator_file_priv_allowed(struct sde_rot_mgr *mgr,
		struct sde_rot_file_private *priv)
{
	struct sde_rot_file_private *_priv, *_priv_next;
	bool ret = false;

	list_for_each_entry_safe(_priv, _priv_next, &mgr->file_list, list) {
		if (_priv == priv) {
			ret = true;
			break;
		}
	}
	return ret;
}

static int sde_rotator_close(struct sde_rot_mgr *mgr,
		struct sde_rot_file_private *private)
{
	if (!mgr || !private)
		return -ENODEV;

	if (!(sde_rotator_file_priv_allowed(mgr, private))) {
		SDEROT_ERR(
			"Calling close with unrecognized rot_file_private\n");
		return -EINVAL;
	}

	/*
	 * if secure camera session was enabled
	 * go back to non secure state
	 */
	sde_rotator_secure_session_ctrl(false);
	sde_rotator_release_rotator_perf_session(mgr, private);

	list_del_init(&private->list);
	devm_kfree(&mgr->pdev->dev, private);

	sde_rotator_update_perf(mgr);
	return 0;
}

static ssize_t sde_rotator_show_caps(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t len = PAGE_SIZE;
	int cnt = 0;
	struct sde_rot_mgr *mgr = sde_rot_mgr_from_device(dev);

	if (!mgr)
		return cnt;

#define SPRINT(fmt, ...) \
		(cnt += scnprintf(buf + cnt, len - cnt, fmt, ##__VA_ARGS__))

	SPRINT("queue_count=%d\n", mgr->queue_count);
	SPRINT("downscale=1\n");
	SPRINT("ubwc=1\n");

	if (mgr->ops_hw_show_caps)
		cnt += mgr->ops_hw_show_caps(mgr, attr, buf + cnt, len - cnt);

	return cnt;
}

static ssize_t sde_rotator_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t len = PAGE_SIZE;
	int cnt = 0;
	struct sde_rot_mgr *mgr = sde_rot_mgr_from_device(dev);
	int i;

	if (!mgr)
		return cnt;

#define SPRINT(fmt, ...) \
		(cnt += scnprintf(buf + cnt, len - cnt, fmt, ##__VA_ARGS__))

	SPRINT("reg_bus_bw=%llu\n", mgr->reg_bus.curr_quota_val);
	SPRINT("data_bus_bw=%llu\n", mgr->data_bus.curr_quota_val);
	SPRINT("pending_close_bw_vote=%llu\n", mgr->pending_close_bw_vote);
	SPRINT("device_suspended=%d\n", atomic_read(&mgr->device_suspended));
	SPRINT("footswitch_cnt=%d\n", mgr->res_ref_cnt);
	SPRINT("regulator_enable=%d\n", mgr->regulator_enable);
	SPRINT("enable_clk_cnt=%d\n", mgr->rot_enable_clk_cnt);
	for (i = 0; i < mgr->num_rot_clk; i++)
		if (mgr->rot_clk[i].clk)
			SPRINT("%s=%lu\n", mgr->rot_clk[i].clk_name,
					clk_get_rate(mgr->rot_clk[i].clk));

	if (mgr->ops_hw_show_state)
		cnt += mgr->ops_hw_show_state(mgr, attr, buf + cnt, len - cnt);

	return cnt;
}

static DEVICE_ATTR(caps, 0444, sde_rotator_show_caps, NULL);
static DEVICE_ATTR(state, 0444, sde_rotator_show_state, NULL);

static struct attribute *sde_rotator_fs_attrs[] = {
	&dev_attr_caps.attr,
	&dev_attr_state.attr,
	NULL
};

static struct attribute_group sde_rotator_fs_attr_group = {
	.attrs = sde_rotator_fs_attrs
};

static int sde_rotator_parse_dt_bus(struct sde_rot_mgr *mgr,
	struct platform_device *dev)
{
	int ret = 0, i;
	int usecases;
	struct device_node *node;

	mgr->data_bus.bus_scale_pdata = msm_bus_cl_get_pdata(dev);
	if (IS_ERR_OR_NULL(mgr->data_bus.bus_scale_pdata)) {
		ret = PTR_ERR(mgr->data_bus.bus_scale_pdata);
		if (ret) {
			SDEROT_ERR("msm_bus_cl_get_pdata failed. ret=%d\n",
					ret);
			mgr->data_bus.bus_scale_pdata = NULL;
		}
	}

	node = of_get_child_by_name(dev->dev.of_node, "qcom,rot-reg-bus");
	if (node) {
		mgr->reg_bus.bus_scale_pdata
				= msm_bus_pdata_from_node(dev, node);
		if (IS_ERR_OR_NULL(mgr->reg_bus.bus_scale_pdata)) {
			SDEROT_ERR("reg bus pdata parsing failed\n");
			ret = PTR_ERR(mgr->reg_bus.bus_scale_pdata);
			if (!mgr->reg_bus.bus_scale_pdata)
				ret = -EINVAL;
			mgr->reg_bus.bus_scale_pdata = NULL;
		}
	} else {
		SDEROT_DBG(
			"no DT entries, configuring default reg bus table\n");
		mgr->reg_bus.bus_scale_pdata = &rot_reg_bus_scale_table;
		usecases = mgr->reg_bus.bus_scale_pdata->num_usecases;
		for (i = 0; i < usecases; i++) {
			rot_reg_bus_usecases[i].num_paths = 1;
			rot_reg_bus_usecases[i].vectors =
				&rot_reg_bus_vectors[i];
		}
	}

	return ret;
}

static int sde_rotator_parse_dt(struct sde_rot_mgr *mgr,
	struct platform_device *dev)
{
	int ret = 0;
	u32 data;

	ret = of_property_read_u32(dev->dev.of_node,
		"qcom,mdss-wb-count", &data);
	if (!ret) {
		if (data > ROT_MAX_HW_BLOCKS) {
			SDEROT_ERR(
				"Err, num of wb block (%d) larger than sw max %d\n",
				data, ROT_MAX_HW_BLOCKS);
			return -EINVAL;
		}

		mgr->queue_count = data;
	}

	ret = sde_rotator_parse_dt_bus(mgr, dev);
	if (ret)
		SDEROT_ERR("Failed to parse bus data\n");

	return ret;
}

static void sde_rotator_put_dt_vreg_data(struct device *dev,
	struct sde_module_power *mp)
{
	if (!mp) {
		SDEROT_ERR("%s: invalid input\n", __func__);
		return;
	}

	sde_rot_config_vreg(dev, mp->vreg_config, mp->num_vreg, 0);
	if (mp->vreg_config) {
		devm_kfree(dev, mp->vreg_config);
		mp->vreg_config = NULL;
	}
	mp->num_vreg = 0;
}

static int sde_rotator_get_dt_vreg_data(struct device *dev,
	struct sde_module_power *mp)
{
	const char *st = NULL;
	struct device_node *of_node = NULL;
	int dt_vreg_total = 0;
	int i;
	int rc;

	if (!dev || !mp) {
		SDEROT_ERR("%s: invalid input\n", __func__);
		return -EINVAL;
	}

	of_node = dev->of_node;

	dt_vreg_total = of_property_count_strings(of_node, "qcom,supply-names");
	if (dt_vreg_total < 0) {
		SDEROT_ERR("%s: vreg not found. rc=%d\n", __func__,
			dt_vreg_total);
		return 0;
	}
	mp->num_vreg = dt_vreg_total;
	mp->vreg_config = devm_kzalloc(dev, sizeof(struct sde_vreg) *
		dt_vreg_total, GFP_KERNEL);
	if (!mp->vreg_config)
		return -ENOMEM;

	/* vreg-name */
	for (i = 0; i < dt_vreg_total; i++) {
		rc = of_property_read_string_index(of_node,
			"qcom,supply-names", i, &st);
		if (rc) {
			SDEROT_ERR("%s: error reading name. i=%d, rc=%d\n",
				__func__, i, rc);
			goto error;
		}
		snprintf(mp->vreg_config[i].vreg_name, 32, "%s", st);
	}
	sde_rot_config_vreg(dev, mp->vreg_config, mp->num_vreg, 1);

	for (i = 0; i < dt_vreg_total; i++) {
		SDEROT_DBG("%s: %s min=%d, max=%d, enable=%d disable=%d\n",
			__func__,
			mp->vreg_config[i].vreg_name,
			mp->vreg_config[i].min_voltage,
			mp->vreg_config[i].max_voltage,
			mp->vreg_config[i].enable_load,
			mp->vreg_config[i].disable_load);
	}
	return rc;

error:
	if (mp->vreg_config) {
		devm_kfree(dev, mp->vreg_config);
		mp->vreg_config = NULL;
	}
	mp->num_vreg = 0;
	return rc;
}

static void sde_rotator_bus_scale_unregister(struct sde_rot_mgr *mgr)
{
	SDEROT_DBG("unregister bus_hdl=%x, reg_bus_hdl=%x\n",
		mgr->data_bus.bus_hdl, mgr->reg_bus.bus_hdl);

	if (mgr->data_bus.bus_hdl)
		msm_bus_scale_unregister_client(mgr->data_bus.bus_hdl);

	if (mgr->reg_bus.bus_hdl)
		msm_bus_scale_unregister_client(mgr->reg_bus.bus_hdl);
}

static int sde_rotator_bus_scale_register(struct sde_rot_mgr *mgr)
{
	if (!mgr->data_bus.bus_scale_pdata) {
		SDEROT_DBG("Bus scaling is not enabled\n");
		return 0;
	}

	mgr->data_bus.bus_hdl =
		msm_bus_scale_register_client(
		mgr->data_bus.bus_scale_pdata);
	if (!mgr->data_bus.bus_hdl) {
		SDEROT_ERR("bus_client register failed\n");
		return -EINVAL;
	}
	SDEROT_DBG("registered bus_hdl=%x\n", mgr->data_bus.bus_hdl);

	if (mgr->reg_bus.bus_scale_pdata) {
		mgr->reg_bus.bus_hdl =
			msm_bus_scale_register_client(
			mgr->reg_bus.bus_scale_pdata);
		if (!mgr->reg_bus.bus_hdl) {
			SDEROT_ERR("register bus_client register failed\n");
			sde_rotator_bus_scale_unregister(mgr);
		} else {
			SDEROT_DBG("registered register bus_hdl=%x\n",
					mgr->reg_bus.bus_hdl);
		}
	}

	return 0;
}

static inline int sde_rotator_search_dt_clk(struct platform_device *pdev,
		struct sde_rot_mgr *mgr, char *clk_name, int clk_idx,
		bool mandatory)
{
	struct clk *tmp;
	int rc = 0;

	if (clk_idx >= SDE_ROTATOR_CLK_MAX) {
		SDEROT_ERR("invalid clk index %d\n", clk_idx);
		return -EINVAL;
	}

	tmp = devm_clk_get(&pdev->dev, clk_name);
	if (IS_ERR(tmp)) {
		if (mandatory)
			SDEROT_ERR("unable to get clk: %s\n", clk_name);
		else
			tmp = NULL;
		rc = PTR_ERR(tmp);
	}

	strlcpy(mgr->rot_clk[clk_idx].clk_name, clk_name,
			sizeof(mgr->rot_clk[clk_idx].clk_name));

	mgr->rot_clk[clk_idx].clk = tmp;
	return mandatory ? rc : 0;
}

static int sde_rotator_parse_dt_clk(struct platform_device *pdev,
		struct sde_rot_mgr *mgr)
{
	u32 rc = 0;
	int num_clk;

	num_clk = of_property_count_strings(pdev->dev.of_node,
			"clock-names");
	if ((num_clk <= 0) || (num_clk > SDE_ROTATOR_CLK_MAX)) {
		SDEROT_ERR("Number of clocks are out of range: %d\n", num_clk);
		goto clk_err;
	}

	mgr->num_rot_clk = SDE_ROTATOR_CLK_MAX;
	mgr->rot_clk = devm_kzalloc(&pdev->dev,
			sizeof(struct sde_rot_clk) * mgr->num_rot_clk,
			GFP_KERNEL);
	if (!mgr->rot_clk) {
		rc = -ENOMEM;
		mgr->num_rot_clk = 0;
		goto clk_err;
	}

	if (sde_rotator_search_dt_clk(pdev, mgr, "mnoc_clk",
			SDE_ROTATOR_CLK_MNOC_AHB, false) ||
			sde_rotator_search_dt_clk(pdev, mgr, "gcc_iface",
				SDE_ROTATOR_CLK_GCC_AHB, false) ||
			sde_rotator_search_dt_clk(pdev, mgr, "gcc_bus",
				SDE_ROTATOR_CLK_GCC_AXI, false) ||
			sde_rotator_search_dt_clk(pdev, mgr, "iface_clk",
				SDE_ROTATOR_CLK_MDSS_AHB, true) ||
			sde_rotator_search_dt_clk(pdev, mgr, "axi_clk",
				SDE_ROTATOR_CLK_MDSS_AXI, false) ||
			sde_rotator_search_dt_clk(pdev, mgr, "rot_core_clk",
				SDE_ROTATOR_CLK_MDSS_ROT, false)) {
		rc = -EINVAL;
		goto clk_err;
	}

	/*
	 * If 'MDSS_ROT' is already present, place 'rot_clk' under
	 * MDSS_ROT_SUB. Otherwise, place it directly into MDSS_ROT.
	 */
	if (sde_rotator_get_clk(mgr, SDE_ROTATOR_CLK_MDSS_ROT))
		rc = sde_rotator_search_dt_clk(pdev, mgr, "rot_clk",
				SDE_ROTATOR_CLK_MDSS_ROT_SUB, true);
	else
		rc = sde_rotator_search_dt_clk(pdev, mgr, "rot_clk",
				SDE_ROTATOR_CLK_MDSS_ROT, true);
clk_err:
	return rc;
}

static int sde_rotator_register_clk(struct platform_device *pdev,
		struct sde_rot_mgr *mgr)
{
	int ret;

	ret = sde_rotator_parse_dt_clk(pdev, mgr);
	if (ret) {
		SDEROT_ERR("unable to parse clocks\n");
		return -EINVAL;
	}

	return 0;
}

static void sde_rotator_unregister_clk(struct sde_rot_mgr *mgr)
{
	devm_kfree(mgr->device, mgr->rot_clk);
	mgr->rot_clk = NULL;
	mgr->num_rot_clk = 0;
}

static int sde_rotator_res_init(struct platform_device *pdev,
	struct sde_rot_mgr *mgr)
{
	int ret;

	if (!sde_rot_mgr_pd_enabled(mgr)) {
		ret = sde_rotator_get_dt_vreg_data(
				&pdev->dev, &mgr->module_power);
		if (ret)
			return ret;
	}

	ret = sde_rotator_register_clk(pdev, mgr);
	if (ret)
		goto error;

	ret = sde_rotator_bus_scale_register(mgr);
	if (ret)
		goto error;

	return 0;
error:
	sde_rotator_put_dt_vreg_data(&pdev->dev, &mgr->module_power);
	return ret;
}

static void sde_rotator_res_destroy(struct sde_rot_mgr *mgr)
{
	struct platform_device *pdev = mgr->pdev;

	sde_rotator_unregister_clk(mgr);
	sde_rotator_bus_scale_unregister(mgr);

	if (!sde_rot_mgr_pd_enabled(mgr))
		sde_rotator_put_dt_vreg_data(&pdev->dev, &mgr->module_power);
}

int sde_rotator_core_init(struct sde_rot_mgr **pmgr,
		struct platform_device *pdev)
{
	struct sde_rot_data_type *mdata = sde_rot_get_mdata();
	struct sde_rot_mgr *mgr;
	int ret;

	if (!pmgr || !pdev) {
		SDEROT_ERR("null parameters\n");
		return -EINVAL;
	}

	mgr = devm_kzalloc(&pdev->dev, sizeof(struct sde_rot_mgr),
		GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;

	mgr->pdev = pdev;
	mgr->device = &pdev->dev;
	mgr->pending_close_bw_vote = 0;
	mgr->enable_bw_vote = ROT_ENABLE_BW_VOTE;
	mgr->hwacquire_timeout = ROT_HW_ACQUIRE_TIMEOUT_IN_MS;
	mgr->queue_count = 1;
	mgr->pixel_per_clk.numer = ROT_PIXEL_PER_CLK_NUMERATOR;
	mgr->pixel_per_clk.denom = ROT_PIXEL_PER_CLK_DENOMINATOR;
	mgr->fudge_factor.numer = ROT_FUDGE_FACTOR_NUMERATOR;
	mgr->fudge_factor.denom = ROT_FUDGE_FACTOR_DENOMINATOR;
	mgr->overhead.numer = ROT_OVERHEAD_NUMERATOR;
	mgr->overhead.denom = ROT_OVERHEAD_DENOMINATOR;

	mutex_init(&mgr->lock);
	atomic_set(&mgr->device_suspended, 0);
	INIT_LIST_HEAD(&mgr->file_list);

	ret = sysfs_create_group(&mgr->device->kobj,
			&sde_rotator_fs_attr_group);
	if (ret) {
		SDEROT_ERR("unable to register rotator sysfs nodes\n");
		goto error_create_sysfs;
	}

	ret = sde_rotator_parse_dt(mgr, pdev);
	if (ret) {
		SDEROT_ERR("fail to parse the dt\n");
		goto error_parse_dt;
	}

	ret = sde_rotator_res_init(pdev, mgr);
	if (ret) {
		SDEROT_ERR("res_init failed %d\n", ret);
		goto error_res_init;
	}

	*pmgr = mgr;
	ret = sde_rotator_footswitch_ctrl(mgr, true);
	if (ret) {
		SDEROT_INFO("res_init failed %d, use probe defer\n", ret);
		ret = -EPROBE_DEFER;
		goto error_fs_en_fail;
	}

	/* enable power and clock before h/w initialization/query */
	sde_rotator_update_clk(mgr);
	sde_rotator_resource_ctrl(mgr, true);
	sde_rotator_clk_ctrl(mgr, true);

	mdata->mdss_version = SDE_REG_READ(mdata, SDE_REG_HW_VERSION);
	SDEROT_DBG("mdss revision %x\n", mdata->mdss_version);

	if (IS_SDE_MAJOR_MINOR_SAME(mdata->mdss_version,
			SDE_MDP_HW_REV_107)) {
		mgr->ops_hw_init = sde_rotator_r1_init;
	} else if (IS_SDE_MAJOR_MINOR_SAME(mdata->mdss_version,
			SDE_MDP_HW_REV_300) ||
		IS_SDE_MAJOR_MINOR_SAME(mdata->mdss_version,
			SDE_MDP_HW_REV_400) ||
		IS_SDE_MAJOR_MINOR_SAME(mdata->mdss_version,
			SDE_MDP_HW_REV_410) ||
		IS_SDE_MAJOR_SAME(mdata->mdss_version,
			SDE_MDP_HW_REV_500)) {
		mgr->ops_hw_init = sde_rotator_r3_init;
		mgr->min_rot_clk = ROT_MIN_ROT_CLK;

		/*
		 * on platforms where the maxlinewidth is greater than
		 * default we need to have a max clock rate check to
		 * ensure we do not cross the max allowed clock for rotator
		 */
		if (IS_SDE_MAJOR_SAME(mdata->mdss_version,
			SDE_MDP_HW_REV_500))
			mgr->max_rot_clk = ROT_R3_MAX_ROT_CLK;

		if (!IS_SDE_MAJOR_SAME(mdata->mdss_version,
					SDE_MDP_HW_REV_500) &&
				!sde_rotator_get_clk(mgr,
					SDE_ROTATOR_CLK_MDSS_AXI)) {
			SDEROT_ERR("unable to get mdss_axi_clk\n");
			ret = -EINVAL;
			goto error_map_hw_ops;
		}
	} else {
		ret = -ENODEV;
		SDEROT_ERR("unsupported sde version %x\n",
				mdata->mdss_version);
		goto error_map_hw_ops;
	}

	ret = mgr->ops_hw_init(mgr);
	if (ret) {
		SDEROT_ERR("hw init failed %d\n", ret);
		goto error_hw_init;
	}

	sde_rotator_pm_qos_add(mdata);

	ret = sde_rotator_init_queue(mgr);
	if (ret) {
		SDEROT_ERR("fail to init queue\n");
		goto error_init_queue;
	}

	/* disable power and clock after h/w initialization/query */
	sde_rotator_clk_ctrl(mgr, false);
	sde_rotator_resource_ctrl(mgr, false);
	sde_rotator_footswitch_ctrl(mgr, false);
	pm_runtime_set_suspended(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	return 0;

error_init_queue:
	mgr->ops_hw_destroy(mgr);
error_hw_init:
error_map_hw_ops:
	sde_rotator_clk_ctrl(mgr, false);
	sde_rotator_resource_ctrl(mgr, false);
	sde_rotator_footswitch_ctrl(mgr, false);
error_fs_en_fail:
	sde_rotator_res_destroy(mgr);
error_res_init:
error_parse_dt:
	sysfs_remove_group(&mgr->device->kobj, &sde_rotator_fs_attr_group);
error_create_sysfs:
	devm_kfree(&pdev->dev, mgr);
	*pmgr = NULL;
	return ret;
}

void sde_rotator_core_destroy(struct sde_rot_mgr *mgr)
{
	struct device *dev;

	if (!mgr) {
		SDEROT_ERR("null parameters\n");
		return;
	}

	dev = mgr->device;
	sde_rotator_deinit_queue(mgr);
	mgr->ops_hw_destroy(mgr);
	sde_rotator_release_all(mgr);
	pm_runtime_disable(mgr->device);
	sde_rotator_res_destroy(mgr);
	sysfs_remove_group(&mgr->device->kobj, &sde_rotator_fs_attr_group);
	devm_kfree(dev, mgr);
}

void sde_rotator_core_dump(struct sde_rot_mgr *mgr)
{
	if (!mgr) {
		SDEROT_ERR("null parameters\n");
		return;
	}

	sde_rotator_resource_ctrl(mgr, true);

	if (mgr->ops_hw_dump_status)
		mgr->ops_hw_dump_status(mgr);

	SDEROT_EVTLOG_TOUT_HANDLER("rot", "rot_dbg_bus", "vbif_dbg_bus");

	sde_rotator_resource_ctrl(mgr, false);
}

static void sde_rotator_suspend_cancel_rot_work(struct sde_rot_mgr *mgr)
{
	struct sde_rot_file_private *priv, *priv_next;

	list_for_each_entry_safe(priv, priv_next, &mgr->file_list, list) {
		sde_rotator_cancel_all_requests(mgr, priv);
	}
}

#if defined(CONFIG_PM)
/*
 * sde_rotator_runtime_suspend - Turn off power upon runtime suspend event
 * @dev: Pointer to device structure
 */
int sde_rotator_runtime_suspend(struct device *dev)
{
	struct sde_rot_mgr *mgr;

	mgr = sde_rot_mgr_from_device(dev);

	if (!mgr) {
		SDEROT_ERR("null parameters\n");
		return -ENODEV;
	}

	if (mgr->rot_enable_clk_cnt) {
		SDEROT_ERR("invalid runtime suspend request %d\n",
				mgr->rot_enable_clk_cnt);
		return -EBUSY;
	}

	sde_rotator_footswitch_ctrl(mgr, false);
	ATRACE_END("runtime_active");
	SDEROT_DBG("exit runtime_active\n");
	return 0;
}

/*
 * sde_rotator_runtime_resume - Turn on power upon runtime resume event
 * @dev: Pointer to device structure
 */
int sde_rotator_runtime_resume(struct device *dev)
{
	struct sde_rot_mgr *mgr;

	mgr = sde_rot_mgr_from_device(dev);

	if (!mgr) {
		SDEROT_ERR("null parameters\n");
		return -ENODEV;
	}

	SDEROT_DBG("begin runtime_active\n");
	ATRACE_BEGIN("runtime_active");
	return sde_rotator_footswitch_ctrl(mgr, true);
}

/*
 * sde_rotator_runtime_idle - check if device is idling
 * @dev: Pointer to device structure
 */
int sde_rotator_runtime_idle(struct device *dev)
{
	struct sde_rot_mgr *mgr;

	mgr = sde_rot_mgr_from_device(dev);

	if (!mgr) {
		SDEROT_ERR("null parameters\n");
		return -ENODEV;
	}

	/* add check for any busy status, if any */
	SDEROT_DBG("idling ...\n");
	return 0;
}

#endif

#ifdef CONFIG_PM_SLEEP
/*
 * sde_rotator_pm_suspend - put the device in pm suspend state by cancelling
 *							 all active requests
 * @dev: Pointer to device structure
 */
int sde_rotator_pm_suspend(struct device *dev)
{
	struct sde_rot_mgr *mgr;

	mgr = sde_rot_mgr_from_device(dev);

	if (!mgr) {
		SDEROT_ERR("null parameters\n");
		return -ENODEV;
	}


	sde_rot_mgr_lock(mgr);
	atomic_inc(&mgr->device_suspended);
	sde_rotator_suspend_cancel_rot_work(mgr);
	mgr->minimum_bw_vote = 0;
	sde_rotator_update_perf(mgr);
	ATRACE_END("pm_active");
	SDEROT_DBG("end pm active %d\n", atomic_read(&mgr->device_suspended));
	sde_rot_mgr_unlock(mgr);
	return 0;
}

/*
 * sde_rotator_pm_resume - put the device in pm active state
 * @dev: Pointer to device structure
 */
int sde_rotator_pm_resume(struct device *dev)
{
	struct sde_rot_mgr *mgr;

	mgr = sde_rot_mgr_from_device(dev);

	if (!mgr) {
		SDEROT_ERR("null parameters\n");
		return -ENODEV;
	}

	/*
	 * It is possible that the runtime status of the device may
	 * have been active when the system was suspended. Reset the runtime
	 * status to suspended state after a complete system resume.
	 */
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	sde_rot_mgr_lock(mgr);
	SDEROT_DBG("begin pm active %d\n", atomic_read(&mgr->device_suspended));
	ATRACE_BEGIN("pm_active");
	atomic_dec(&mgr->device_suspended);
	sde_rotator_update_perf(mgr);
	sde_rot_mgr_unlock(mgr);
	return 0;
}
#endif

#if defined(CONFIG_PM) && !defined(CONFIG_PM_SLEEP)
int sde_rotator_suspend(struct platform_device *dev, pm_message_t state)
{
	struct sde_rot_mgr *mgr;

	mgr = sde_rot_mgr_from_pdevice(dev);

	if (!mgr) {
		SDEROT_ERR("null_parameters\n");
		return -ENODEV;
	}

	sde_rot_mgr_lock(mgr);
	atomic_inc(&mgr->device_suspended);
	sde_rotator_suspend_cancel_rot_work(mgr);
	sde_rotator_update_perf(mgr);
	sde_rot_mgr_unlock(mgr);
	return 0;
}

int sde_rotator_resume(struct platform_device *dev)
{
	struct sde_rot_mgr *mgr;

	mgr = sde_rot_mgr_from_pdevice(dev);

	if (!mgr) {
		SDEROT_ERR("null parameters\n");
		return -ENODEV;
	}

	sde_rot_mgr_lock(mgr);
	atomic_dec(&mgr->device_suspended);
	sde_rotator_update_perf(mgr);
	sde_rot_mgr_unlock(mgr);
	return 0;
}
#endif

/*
 * sde_rotator_session_open - external wrapper for open function
 *
 * Note each file open (sde_rot_file_private) is mapped to one session only.
 */
int sde_rotator_session_open(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private **pprivate, int session_id,
	struct sde_rot_queue_v1 *queue)
{
	int ret;
	struct sde_rot_file_private *private;

	if (!mgr || !pprivate || !queue) {
		SDEROT_ERR("null parameters\n");
		return -EINVAL;
	}

	ret = sde_rotator_open(mgr, &private);
	if (ret)
		goto error_open;

	private->mgr = mgr;
	private->fenceq = queue;

	ret = sde_rotator_open_session(mgr, private, session_id);
	if (ret)
		goto error_open_session;

	*pprivate = private;

	return 0;
error_open_session:
	sde_rotator_close(mgr, private);
error_open:
	return ret;
}

/*
 * sde_rotator_session_close - external wrapper for close function
 */
void sde_rotator_session_close(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private, int session_id)
{
	if (!mgr || !private) {
		SDEROT_ERR("null parameters\n");
		return;
	}

	sde_rotator_close_session(mgr, private, session_id);
	sde_rotator_close(mgr, private);

	SDEROT_DBG("session closed s:%d\n", session_id);
}

/*
 * sde_rotator_session_config - external wrapper for config function
 */
int sde_rotator_session_config(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private,
	struct sde_rotation_config *config)
{
	if (!mgr || !private || !config) {
		SDEROT_ERR("null parameters\n");
		return -EINVAL;
	}

	return sde_rotator_config_session(mgr, private, config);
}

/*
 * sde_rotator_session_validate - validate session
 */
int sde_rotator_session_validate(struct sde_rot_mgr *mgr,
	struct sde_rot_file_private *private,
	struct sde_rotation_config *config)
{
	int ret;

	if (!mgr || !private || !config) {
		SDEROT_ERR("null parameters\n");
		return -EINVAL;
	}

	SDEROT_DBG(
		"validate session id=%u in{%u,%u}f:%x out{%u,%u}f:%x fps:%d\n",
		config->session_id, config->input.width, config->input.height,
		config->input.format, config->output.width,
		config->output.height, config->output.format,
		config->frame_rate);

	ret = sde_rotator_verify_config_all(mgr, config);
	if (ret) {
		SDEROT_WARN("rotator verify format failed %d\n", ret);
		return ret;
	}

	if (config->output.sbuf && mgr->sbuf_ctx != private && mgr->sbuf_ctx) {
		SDEROT_WARN("too many sbuf sessions\n");
		return -EBUSY;
	}

	return 0;
}
