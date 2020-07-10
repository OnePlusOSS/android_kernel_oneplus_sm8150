/* Copyright (c) 2013-2019, The Linux Foundation. All rights reserved.
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

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/debugfs.h>
#include <linux/videodev2.h>
#include <linux/of_device.h>
#include <linux/sched_clock.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>

#include "msm_isp.h"
#include "msm_isp_util.h"
#include "msm_isp_axi_util.h"
#include "msm_isp_stats_util.h"
#include "msm_sd.h"
#include "msm_isp48.h"
#include "msm_isp47.h"
#include "msm_isp46.h"
#include "msm_isp44.h"
#include "msm_isp40.h"
#include "msm_isp32.h"
#include "msm_cam_cx_ipeak.h"

static struct msm_sd_req_vb2_q vfe_vb2_ops;
static struct msm_isp_buf_mgr vfe_buf_mgr;
static struct msm_vfe_common_dev_data vfe_common_data;
static struct dual_vfe_resource dualvfe;

static const struct of_device_id msm_vfe_dt_match[] = {
	{
		.compatible = "qcom,vfe",
	},
	{}
};

MODULE_DEVICE_TABLE(of, msm_vfe_dt_match);

#define MAX_OVERFLOW_COUNTERS  29
#define OVERFLOW_LENGTH 1024
#define OVERFLOW_BUFFER_LENGTH 64
static char stat_line[OVERFLOW_LENGTH];

static int msm_isp_enable_debugfs(struct vfe_device *vfe_dev,
	  struct msm_isp_bw_req_info *isp_req_hist);

static char *stats_str[MAX_OVERFLOW_COUNTERS] = {
	"imgmaster0_overflow_cnt",
	"imgmaster1_overflow_cnt",
	"imgmaster2_overflow_cnt",
	"imgmaster3_overflow_cnt",
	"imgmaster4_overflow_cnt",
	"imgmaster5_overflow_cnt",
	"imgmaster6_overflow_cnt",
	"be_overflow_cnt",
	"bg_overflow_cnt",
	"bf_overflow_cnt",
	"awb_overflow_cnt",
	"rs_overflow_cnt",
	"cs_overflow_cnt",
	"ihist_overflow_cnt",
	"skinbhist_overflow_cnt",
	"bfscale_overflow_cnt",
	"ISP_VFE0_client_info.active",
	"ISP_VFE0_client_info.ab",
	"ISP_VFE0_client_info.ib",
	"ISP_VFE1_client_info.active",
	"ISP_VFE1_client_info.ab",
	"ISP_VFE1_client_info.ib",
	"ISP_CPP_client_info.active",
	"ISP_CPP_client_info.ab",
	"ISP_CPP_client_info.ib",
	"ISP_last_overflow.ab",
	"ISP_last_overflow.ib",
	"ISP_VFE_CLK_RATE",
	"ISP_CPP_CLK_RATE",
};

#define MAX_DEPTH_BW_REQ_HISTORY 25
#define MAX_BW_HISTORY_BUFF_LEN  6144
#define MAX_BW_HISTORY_LINE_BUFF_LEN 512

#define MAX_UB_INFO_BUFF_LEN  1024
#define MAX_UB_INFO_LINE_BUFF_LEN 256

static struct msm_isp_bw_req_info
	msm_isp_bw_request_history[MAX_DEPTH_BW_REQ_HISTORY];
static int msm_isp_bw_request_history_idx;
static char bw_request_history_buff[MAX_BW_HISTORY_BUFF_LEN];
static char ub_info_buffer[MAX_UB_INFO_BUFF_LEN];
static spinlock_t req_history_lock;

static int vfe_debugfs_statistics_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t vfe_debugfs_statistics_read(struct file *t_file,
	char __user *t_char, size_t t_size_t, loff_t *t_loff_t)
{
	int i;
	uint64_t *ptr;
	char buffer[OVERFLOW_BUFFER_LENGTH] = {0};
	struct vfe_device *vfe_dev = (struct vfe_device *)
		t_file->private_data;
	struct msm_isp_statistics *stats = vfe_dev->stats;

	memset(stat_line, 0, sizeof(stat_line));
	msm_isp_util_get_bandwidth_stats(vfe_dev, stats);
	ptr = (uint64_t *)(stats);
	for (i = 0; i < MAX_OVERFLOW_COUNTERS; i++) {
		strlcat(stat_line, stats_str[i], sizeof(stat_line));
		strlcat(stat_line, "     ", sizeof(stat_line));
		snprintf(buffer, sizeof(buffer), "%llu", ptr[i]);
		strlcat(stat_line, buffer, sizeof(stat_line));
		strlcat(stat_line, "\r\n", sizeof(stat_line));
	}
	return simple_read_from_buffer(t_char, t_size_t,
		t_loff_t, stat_line, strlen(stat_line));
}

static ssize_t vfe_debugfs_statistics_write(struct file *t_file,
	const char __user *t_char, size_t t_size_t, loff_t *t_loff_t)
{
	struct vfe_device *vfe_dev = (struct vfe_device *)
		t_file->private_data;
	struct msm_isp_statistics *stats = vfe_dev->stats;

	memset(stats, 0, sizeof(struct msm_isp_statistics));

	return sizeof(struct msm_isp_statistics);
}

static int bw_history_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t bw_history_read(struct file *t_file, char __user *t_char,
	size_t t_size_t, loff_t *t_loff_t)
{
	int i;
	char *out_buffer = bw_request_history_buff;
	char line_buffer[MAX_BW_HISTORY_LINE_BUFF_LEN] = {0};
	struct msm_isp_bw_req_info *isp_req_hist =
		(struct msm_isp_bw_req_info *) t_file->private_data;

	memset(out_buffer, 0, MAX_BW_HISTORY_BUFF_LEN);

	snprintf(line_buffer, sizeof(line_buffer),
		"Bus bandwidth request history in chronological order:\n");
	strlcat(out_buffer, line_buffer, sizeof(bw_request_history_buff));

	snprintf(line_buffer, sizeof(line_buffer),
		"MSM_ISP_MIN_AB = %u, MSM_ISP_MIN_IB = %u\n\n",
		MSM_ISP_MIN_AB, MSM_ISP_MIN_IB);
	strlcat(out_buffer, line_buffer, sizeof(bw_request_history_buff));

	for (i = 0; i < MAX_DEPTH_BW_REQ_HISTORY; i++) {
		snprintf(line_buffer, sizeof(line_buffer),
		 "idx = %d, client = %u, timestamp = %llu, ab = %llu, ib = %llu\n"
		 "ISP0.active = %x, ISP0.ab = %llu, ISP0.ib = %llu\n"
		 "ISP1.active = %x, ISP1.ab = %llu, ISP1.ib = %llu\n"
		 "CPP.active = %x, CPP.ab = %llu, CPP.ib = %llu\n\n",
		 i, isp_req_hist[i].client, isp_req_hist[i].timestamp,
		 isp_req_hist[i].total_ab, isp_req_hist[i].total_ib,
		 isp_req_hist[i].client_info[0].active,
		 isp_req_hist[i].client_info[0].ab,
		 isp_req_hist[i].client_info[0].ib,
		 isp_req_hist[i].client_info[1].active,
		 isp_req_hist[i].client_info[1].ab,
		 isp_req_hist[i].client_info[1].ib,
		 isp_req_hist[i].client_info[2].active,
		 isp_req_hist[i].client_info[2].ab,
		 isp_req_hist[i].client_info[2].ib);
		strlcat(out_buffer, line_buffer,
		sizeof(bw_request_history_buff));
	}
	return simple_read_from_buffer(t_char, t_size_t,
		t_loff_t, out_buffer, strlen(out_buffer));
}

static ssize_t bw_history_write(struct file *t_file,
	const char __user *t_char, size_t t_size_t, loff_t *t_loff_t)
{
	struct msm_isp_bw_req_info *isp_req_hist =
		(struct msm_isp_bw_req_info *) t_file->private_data;

	memset(isp_req_hist, 0, sizeof(msm_isp_bw_request_history));
	msm_isp_bw_request_history_idx = 0;
	return sizeof(msm_isp_bw_request_history);
}

static int ub_info_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t ub_info_read(struct file *t_file, char __user *t_char,
	size_t t_size_t, loff_t *t_loff_t)
{
	int i;
	char *out_buffer = ub_info_buffer;
	char line_buffer[MAX_UB_INFO_LINE_BUFF_LEN] = {0};
	struct vfe_device *vfe_dev =
		(struct vfe_device *) t_file->private_data;
	struct msm_isp_ub_info *ub_info = vfe_dev->ub_info;

	memset(out_buffer, 0, MAX_UB_INFO_LINE_BUFF_LEN);
	snprintf(line_buffer, sizeof(line_buffer),
		"wm_ub_policy_type = %d\n"
		"num_wm = %d\n"
		"wm_ub = %d\n",
		ub_info->policy, ub_info->num_wm, ub_info->wm_ub);
	strlcat(out_buffer, line_buffer,
	    sizeof(ub_info_buffer));
	for (i = 0; i < ub_info->num_wm; i++) {
		snprintf(line_buffer, sizeof(line_buffer),
			"data[%d] = 0x%x, addr[%d] = 0x%llx\n",
			i, ub_info->data[i], i, ub_info->addr[i]);
		strlcat(out_buffer, line_buffer,
			sizeof(ub_info_buffer));
	}

	return simple_read_from_buffer(t_char, t_size_t,
		t_loff_t, out_buffer, strlen(out_buffer));
}

static ssize_t ub_info_write(struct file *t_file,
	const char __user *t_char, size_t t_size_t, loff_t *t_loff_t)
{
	struct vfe_device *vfe_dev =
		(struct vfe_device *) t_file->private_data;
	struct msm_isp_ub_info *ub_info = vfe_dev->ub_info;

	memset(ub_info, 0, sizeof(struct msm_isp_ub_info));

	return sizeof(struct msm_isp_ub_info);
}

static const struct file_operations vfe_debugfs_error = {
	.open = vfe_debugfs_statistics_open,
	.read = vfe_debugfs_statistics_read,
	.write = vfe_debugfs_statistics_write,
};

static const struct file_operations bw_history_ops = {
	.open = bw_history_open,
	.read = bw_history_read,
	.write = bw_history_write,
};

static const struct file_operations ub_info_ops = {
	.open = ub_info_open,
	.read = ub_info_read,
	.write = ub_info_write,
};

static int msm_isp_enable_debugfs(struct vfe_device *vfe_dev,
	struct msm_isp_bw_req_info *isp_req_hist)
{
	struct dentry *debugfs_base;
	char dirname[32] = {0};

	snprintf(dirname, sizeof(dirname), "msm_isp%d", vfe_dev->pdev->id);
	debugfs_base = debugfs_create_dir(dirname, NULL);
	if (!debugfs_base)
		return -ENOMEM;
	if (!debugfs_create_file("stats", 0644, debugfs_base,
		vfe_dev, &vfe_debugfs_error))
		return -ENOMEM;

	if (!debugfs_create_file("bw_req_history", 0644,
		debugfs_base, isp_req_hist, &bw_history_ops))
		return -ENOMEM;

	if (!debugfs_create_file("ub_info", 0644,
		debugfs_base, vfe_dev, &ub_info_ops))
		return -ENOMEM;

	return 0;
}

void msm_isp_update_req_history(uint32_t client, uint64_t ab,
				 uint64_t ib,
				 struct msm_isp_bandwidth_info *client_info,
				 unsigned long long ts)
{
	int i;

	spin_lock(&req_history_lock);
	msm_isp_bw_request_history[msm_isp_bw_request_history_idx].client =
		client;
	msm_isp_bw_request_history[msm_isp_bw_request_history_idx].timestamp =
		ts;
	msm_isp_bw_request_history[msm_isp_bw_request_history_idx].total_ab =
		ab;
	msm_isp_bw_request_history[msm_isp_bw_request_history_idx].total_ib =
		ib;

	for (i = 0; i < MAX_ISP_CLIENT; i++) {
		msm_isp_bw_request_history[msm_isp_bw_request_history_idx]
			.client_info[i].active = client_info[i].active;
		msm_isp_bw_request_history[msm_isp_bw_request_history_idx]
			.client_info[i].ab = client_info[i].ab;
		msm_isp_bw_request_history[msm_isp_bw_request_history_idx]
			.client_info[i].ib = client_info[i].ib;
	}

	msm_isp_bw_request_history_idx = (msm_isp_bw_request_history_idx + 1)
			 % MAX_DEPTH_BW_REQ_HISTORY;
	spin_unlock(&req_history_lock);
}

void msm_isp_update_last_overflow_ab_ib(struct vfe_device *vfe_dev)
{
	spin_lock(&req_history_lock);
	vfe_dev->msm_isp_last_overflow_ab =
	msm_isp_bw_request_history[msm_isp_bw_request_history_idx].total_ab;
	vfe_dev->msm_isp_last_overflow_ib =
	msm_isp_bw_request_history[msm_isp_bw_request_history_idx].total_ib;
	spin_unlock(&req_history_lock);
}


#ifdef CONFIG_COMPAT
static long msm_isp_dqevent(struct file *file, struct v4l2_fh *vfh, void *arg)
{
	long rc;

	if (is_compat_task()) {
		struct msm_isp_event_data32 *event_data32;
		struct msm_isp_event_data  *event_data;
		struct v4l2_event isp_event;
		struct v4l2_event *isp_event_user;

		memset(&isp_event, 0, sizeof(isp_event));
		rc = v4l2_event_dequeue(vfh, &isp_event,
				file->f_flags & O_NONBLOCK);
		if (rc)
			return rc;
		event_data = (struct msm_isp_event_data *)
				isp_event.u.data;
		isp_event_user = (struct v4l2_event *)arg;
		memcpy(isp_event_user, &isp_event,
				sizeof(*isp_event_user));
		event_data32 = (struct msm_isp_event_data32 *)
			isp_event_user->u.data;
		memset(event_data32, 0,
				sizeof(struct msm_isp_event_data32));
		event_data32->timestamp.tv_sec =
				event_data->timestamp.tv_sec;
		event_data32->timestamp.tv_usec =
				event_data->timestamp.tv_usec;
		event_data32->mono_timestamp.tv_sec =
				event_data->mono_timestamp.tv_sec;
		event_data32->mono_timestamp.tv_usec =
				event_data->mono_timestamp.tv_usec;
		event_data32->frame_id = event_data->frame_id;
		memcpy(&(event_data32->u), &(event_data->u),
					sizeof(event_data32->u));
	} else {
		rc = v4l2_event_dequeue(vfh, arg,
				file->f_flags & O_NONBLOCK);
	}
	return rc;
}
#else
static long msm_isp_dqevent(struct file *file, struct v4l2_fh *vfh, void *arg)
{
	return v4l2_event_dequeue(vfh, arg,
			file->f_flags & O_NONBLOCK);
}
#endif

static long msm_isp_subdev_do_ioctl(
	struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct v4l2_fh *vfh = file->private_data;

	switch (cmd) {
	case VIDIOC_DQEVENT: {
		if (!(sd->flags & V4L2_SUBDEV_FL_HAS_EVENTS))
			return -ENOIOCTLCMD;
		return msm_isp_dqevent(file, vfh, arg);
	}
	break;
	case VIDIOC_SUBSCRIBE_EVENT:
		return v4l2_subdev_call(sd, core, subscribe_event, vfh, arg);

	case VIDIOC_UNSUBSCRIBE_EVENT:
		return v4l2_subdev_call(sd, core, unsubscribe_event, vfh, arg);

	default:
		return v4l2_subdev_call(sd, core, ioctl, cmd, arg);
	}
}

static struct v4l2_subdev_core_ops msm_vfe_v4l2_subdev_core_ops = {
	.ioctl = msm_isp_ioctl,
	.subscribe_event = msm_isp_subscribe_event,
	.unsubscribe_event = msm_isp_unsubscribe_event,
};

static struct v4l2_subdev_ops msm_vfe_v4l2_subdev_ops = {
	.core = &msm_vfe_v4l2_subdev_core_ops,
};

static struct v4l2_subdev_internal_ops msm_vfe_subdev_internal_ops = {
	.open = msm_isp_open_node,
	.close = msm_isp_close_node,
};

static long msm_isp_v4l2_fops_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_isp_subdev_do_ioctl);
}

static void isp_vma_open(struct vm_area_struct *vma)
{
	pr_debug("%s: open called\n", __func__);
}

static void isp_vma_close(struct vm_area_struct *vma)
{
	pr_debug("%s: close called\n", __func__);
}

static int isp_vma_fault(struct vm_fault *vmf)
{
	struct page *page;
	struct vfe_device *vfe_dev = vmf->vma->vm_private_data;
	struct isp_kstate *isp_page = NULL;

	isp_page = vfe_dev->isp_page;

	pr_debug("%s: vfeid:%d u_virt_addr:0x%lx k_virt_addr:%pK\n",
		__func__, vfe_dev->pdev->id, vmf->vma->vm_start,
		(void *)isp_page);
	if (isp_page != NULL) {
		page = virt_to_page(isp_page);
		get_page(page);
		vmf->page = page;
		isp_page->kernel_sofid =
			vfe_dev->axi_data.src_info[VFE_PIX_0].frame_id;
		isp_page->vfeid = vfe_dev->pdev->id;
	}
	return 0;
}

static const struct vm_operations_struct isp_vm_ops = {
	.open = isp_vma_open,
	.close = isp_vma_close,
	.fault = isp_vma_fault,
};

static int msm_isp_v4l2_fops_mmap(struct file *filep,
	struct vm_area_struct *vma)
{
	int ret =  -EINVAL;
	struct video_device *vdev = video_devdata(filep);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct vfe_device *vfe_dev = v4l2_get_subdevdata(sd);

	vma->vm_ops = &isp_vm_ops;
	vma->vm_flags |=
		(unsigned long)(VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_private_data = vfe_dev;
	isp_vma_open(vma);
	ret = 0;
	pr_debug("%s: isp mmap is called vm_start: 0x%lx\n",
		__func__, vma->vm_start);
	return ret;
}

static struct v4l2_file_operations msm_isp_v4l2_fops = {
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = msm_isp_v4l2_fops_ioctl,
#endif
	.unlocked_ioctl = msm_isp_v4l2_fops_ioctl,
	.mmap = msm_isp_v4l2_fops_mmap
};

static int vfe_set_common_data(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = NULL;
	struct vfe_device *vfe_dev = NULL;

	sd = (struct v4l2_subdev *)platform_get_drvdata(pdev);
	if (!sd) {
		pr_err("%s: Error! Cannot find subdev\n", __func__);
		return -EPERM;
	}
	vfe_dev = (struct vfe_device *)v4l2_get_subdevdata(sd);
	if (!vfe_dev) {
		pr_err("%s: Error! Cannot find vfe_dev\n", __func__);
		return -EPERM;
	}

	vfe_dev->common_data = (struct msm_vfe_common_dev_data *)
		pdev->dev.platform_data;

	vfe_dev->common_data->dual_vfe_res = &dualvfe;
	vfe_dev->common_data->dual_vfe_res->axi_data[vfe_dev->pdev->id] =
		&vfe_dev->axi_data;
	vfe_dev->common_data->dual_vfe_res->stats_data[vfe_dev->pdev->id] =
		&vfe_dev->stats_data;
	vfe_dev->common_data->dual_vfe_res->vfe_dev[vfe_dev->pdev->id] =
		vfe_dev;
	return 0;
}

static int vfe_probe(struct platform_device *pdev)
{
	struct vfe_parent_device *vfe_parent_dev;
	int rc = 0;
	struct device_node *node;
	struct platform_device *new_dev = NULL;
	uint32_t i = 0;
	char name[10] = "\0";

	vfe_parent_dev = kzalloc(sizeof(struct vfe_parent_device),
		GFP_KERNEL);
	if (!vfe_parent_dev) {
		rc = -ENOMEM;
		goto end;
	}

	vfe_parent_dev->common_sd = kzalloc(
		sizeof(struct msm_vfe_common_subdev), GFP_KERNEL);
	if (!vfe_parent_dev->common_sd) {
		rc = -ENOMEM;
		goto probe_fail1;
	}

	vfe_parent_dev->common_sd->common_data = &vfe_common_data;
	mutex_init(&vfe_common_data.vfe_common_mutex);
	spin_lock_init(&vfe_common_data.common_dev_data_lock);
	spin_lock_init(&vfe_common_data.vfe_irq_dump.common_dev_irq_dump_lock);
	spin_lock_init(
		&vfe_common_data.vfe_irq_dump.common_dev_tasklet_dump_lock);
	for (i = 0; i < (VFE_AXI_SRC_MAX * MAX_VFE); i++)
		spin_lock_init(&(vfe_common_data.streams[i].lock));
	for (i = 0; i < (MSM_ISP_STATS_MAX * MAX_VFE); i++)
		spin_lock_init(&(vfe_common_data.stats_streams[i].lock));

	for (i = 0; i <= MAX_VFE; i++) {
		INIT_LIST_HEAD(&vfe_common_data.tasklets[i].tasklet_q);
		tasklet_init(&vfe_common_data.tasklets[i].tasklet,
			msm_isp_do_tasklet,
			(unsigned long)(&vfe_common_data.tasklets[i]));
		spin_lock_init(&vfe_common_data.tasklets[i].tasklet_lock);
	}

	of_property_read_u32(pdev->dev.of_node,
		"num_child", &vfe_parent_dev->num_hw_sd);

	for (i = 0; i < vfe_parent_dev->num_hw_sd; i++) {
		node = NULL;
		snprintf(name, sizeof(name), "qcom,vfe%d", i);
		node = of_find_node_by_name(NULL, name);
		if (!node) {
			pr_err("%s: Error! Cannot find node in dtsi %s\n",
				__func__, name);
			goto probe_fail2;
		}
		new_dev = of_find_device_by_node(node);
		if (!new_dev) {
			pr_err("%s: Failed to find device on bus %s\n",
				__func__, node->name);
			goto probe_fail2;
		}
		vfe_parent_dev->child_list[i] = new_dev;
		new_dev->dev.platform_data =
			(void *)vfe_parent_dev->common_sd->common_data;
		rc = vfe_set_common_data(new_dev);
		if (rc < 0)
			goto probe_fail2;
	}

	vfe_parent_dev->num_sd = vfe_parent_dev->num_hw_sd;
	vfe_parent_dev->pdev = pdev;

	return rc;

probe_fail2:
	kfree(vfe_parent_dev->common_sd);
probe_fail1:
	kfree(vfe_parent_dev);
end:
	return rc;
}

int vfe_hw_probe(struct platform_device *pdev)
{
	struct vfe_device *vfe_dev;
	/*struct msm_cam_subdev_info sd_info;*/
	const struct of_device_id *match_dev;
	int rc = 0;
	struct msm_vfe_hardware_info *hw_info;

	vfe_dev = kzalloc(sizeof(struct vfe_device), GFP_KERNEL);
	if (!vfe_dev) {
		rc = -ENOMEM;
		goto end;
	}
	vfe_dev->stats = kzalloc(sizeof(struct msm_isp_statistics), GFP_KERNEL);
	if (!vfe_dev->stats) {
		rc = -ENOMEM;
		goto probe_fail1;
	}

	vfe_dev->ub_info = kzalloc(sizeof(struct msm_isp_ub_info), GFP_KERNEL);
	if (!vfe_dev->ub_info) {
		rc = -ENOMEM;
		goto probe_fail2;
	}

	if (pdev->dev.of_node) {
		of_property_read_u32(pdev->dev.of_node,
			"cell-index", &pdev->id);

		match_dev = of_match_device(pdev->dev.driver->of_match_table,
			&pdev->dev);
		if (!match_dev) {
			pr_err("%s: No vfe hardware info\n", __func__);
			rc = -EINVAL;
			goto probe_fail3;
		}
		vfe_dev->hw_info =
			(struct msm_vfe_hardware_info *) match_dev->data;
		/* Cx ipeak support */
		if (of_find_property(pdev->dev.of_node,
			"qcom,vfe-cx-ipeak", NULL)) {
			vfe_dev->vfe_cx_ipeak = cx_ipeak_register(
				pdev->dev.of_node, "qcom,vfe-cx-ipeak");
			if (vfe_dev->vfe_cx_ipeak)
				cam_cx_ipeak_register_cx_ipeak(
				vfe_dev->vfe_cx_ipeak, &vfe_dev->cx_ipeak_bit);
			pr_debug("%s: register cx_ipeak received bit %d\n",
				__func__, vfe_dev->cx_ipeak_bit);
		}
	} else {
		vfe_dev->hw_info = (struct msm_vfe_hardware_info *)
			platform_get_device_id(pdev)->driver_data;
	}

	if (!vfe_dev->hw_info) {
		pr_err("%s: No vfe hardware info\n", __func__);
		rc = -EINVAL;
		goto probe_fail3;
	}
	ISP_DBG("%s: device id = %d\n", __func__, pdev->id);

	vfe_dev->pdev = pdev;
	hw_info = vfe_dev->hw_info;

	rc = vfe_dev->hw_info->vfe_ops.platform_ops.get_platform_data(vfe_dev);
	if (rc < 0) {
		pr_err("%s: failed to get platform resources\n", __func__);
		rc = -ENOMEM;
		goto probe_fail3;
	}

	if (
	hw_info->vfe_ops.platform_ops.get_dual_sync_platform_data) {
		rc =
		hw_info->vfe_ops.platform_ops.get_dual_sync_platform_data(
			vfe_dev);
			if (rc < 0)
				pr_err("%s:fail get dual_sync\n", __func__);
	}

	v4l2_subdev_init(&vfe_dev->subdev.sd, &msm_vfe_v4l2_subdev_ops);
	vfe_dev->subdev.sd.internal_ops =
		&msm_vfe_subdev_internal_ops;
	snprintf(vfe_dev->subdev.sd.name,
		ARRAY_SIZE(vfe_dev->subdev.sd.name),
		"vfe");
	vfe_dev->subdev.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	vfe_dev->subdev.sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;
	v4l2_set_subdevdata(&vfe_dev->subdev.sd, vfe_dev);
	platform_set_drvdata(pdev, &vfe_dev->subdev.sd);
	mutex_init(&vfe_dev->realtime_mutex);
	mutex_init(&vfe_dev->core_mutex);
	spin_lock_init(&vfe_dev->shared_data_lock);
	spin_lock_init(&vfe_dev->reg_update_lock);
	spin_lock_init(&req_history_lock);
	spin_lock_init(&vfe_dev->reset_completion_lock);
	spin_lock_init(&vfe_dev->halt_completion_lock);
	media_entity_pads_init(&vfe_dev->subdev.sd.entity, 0, NULL);
	vfe_dev->subdev.sd.entity.function = MSM_CAMERA_SUBDEV_VFE;
	//vfe_dev->subdev.sd.entity.group_id = MSM_CAMERA_SUBDEV_VFE;
	vfe_dev->subdev.sd.entity.name = pdev->name;
	vfe_dev->subdev.close_seq = MSM_SD_CLOSE_1ST_CATEGORY | 0x2;
	rc = msm_sd_register(&vfe_dev->subdev);
	if (rc != 0) {
		pr_err("%s: msm_sd_register error = %d\n", __func__, rc);
		goto probe_fail3;
	}
	msm_cam_copy_v4l2_subdev_fops(&msm_isp_v4l2_fops);
	msm_isp_v4l2_fops.unlocked_ioctl = msm_isp_v4l2_fops_ioctl;
#ifdef CONFIG_COMPAT
	msm_isp_v4l2_fops.compat_ioctl32 =
		msm_isp_v4l2_fops_ioctl;
#endif
	msm_isp_v4l2_fops.mmap = msm_isp_v4l2_fops_mmap;

	vfe_dev->subdev.sd.devnode->fops = &msm_isp_v4l2_fops;

	vfe_dev->buf_mgr = &vfe_buf_mgr;
	v4l2_subdev_notify(&vfe_dev->subdev.sd,
		MSM_SD_NOTIFY_REQ_CB, &vfe_vb2_ops);
	rc = msm_isp_create_isp_buf_mgr(vfe_dev->buf_mgr,
		&vfe_vb2_ops, &pdev->dev,
		vfe_dev->hw_info->axi_hw_info->scratch_buf_range);
	if (rc < 0) {
		pr_err("%s: Unable to create buffer manager\n", __func__);
		rc = -EINVAL;
		goto probe_fail3;
	}
	msm_isp_enable_debugfs(vfe_dev, msm_isp_bw_request_history);
	vfe_dev->buf_mgr->init_done = 1;
	vfe_dev->vfe_open_cnt = 0;
	/*Allocate a page in kernel and map it to camera user process*/
	vfe_dev->isp_page = (struct isp_kstate *)get_zeroed_page(GFP_KERNEL);
	if (vfe_dev->isp_page == NULL) {
		pr_err("%s: no enough memory\n", __func__);
		rc = -ENOMEM;
		goto probe_fail3;
	}
	vfe_dev->isp_page->vfeid = vfe_dev->pdev->id;
	return rc;

probe_fail3:
	kfree(vfe_dev->ub_info);
probe_fail2:
	kfree(vfe_dev->stats);
probe_fail1:
	kfree(vfe_dev);
end:
	return rc;
}

static struct platform_driver vfe_driver = {
	.probe = vfe_probe,
	.driver = {
		.name = "msm_vfe",
		.owner = THIS_MODULE,
		.of_match_table = msm_vfe_dt_match,
	},
};

static int __init msm_vfe_init_module(void)
{
	return platform_driver_register(&vfe_driver);
}

static void __exit msm_vfe_exit_module(void)
{
	platform_driver_unregister(&vfe_driver);
}

late_initcall(msm_vfe_init_module);
module_exit(msm_vfe_exit_module);
MODULE_DESCRIPTION("MSM VFE driver");
MODULE_LICENSE("GPL v2");
