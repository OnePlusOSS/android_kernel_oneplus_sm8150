/* Copyright (c) 2013-2018, The Linux Foundation. All rights reserved.
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

#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/ioctl.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <linux/proc_fs.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>


#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-device.h>
#include <media/videobuf2-core.h>
#include <media/msmb_generic_buf_mgr.h>

#include "msm.h"
#include "msm_buf_mgr.h"
#include "cam_smmu_api.h"

#undef CDBG
#define CDBG(fmt, args...) pr_debug(fmt, ##args)

#define BUF_DEBUG_FULL 0
#define MAX_LIST_COUNT 100

static int msm_buf_check_head_sanity(struct msm_isp_bufq *bufq)
{
	int rc = 0;
	struct list_head *prev = NULL;
	struct list_head *next = NULL;

	if (!bufq) {
		pr_err("%s: Error! Invalid bufq\n", __func__);
		return -EINVAL;
	}

	prev = bufq->head.prev;
	next = bufq->head.next;

	if (!prev) {
		pr_err("%s: Error! bufq->head.prev is NULL\n", __func__);
		return -EINVAL;
	}

	if (!next) {
		pr_err("%s: Error! bufq->head.next is NULL\n", __func__);
		return -EINVAL;
	}

	if (prev->next != &bufq->head) {
		pr_err("%s: Error! head prev->next is %pK should be %pK\n",
			__func__, prev->next, &bufq->head);
		return -EINVAL;
	}

	if (next->prev != &bufq->head) {
		pr_err("%s: Error! head next->prev is %pK should be %pK\n",
			__func__, next->prev, &bufq->head);
		return -EINVAL;
	}

	return rc;
}

static struct msm_isp_bufq *msm_isp_get_bufq(
	struct msm_isp_buf_mgr *buf_mgr,
	uint32_t bufq_handle)
{
	struct msm_isp_bufq *bufq = NULL;
	uint32_t bufq_index = bufq_handle & 0xFF;

	/* bufq_handle cannot be 0 */
	if ((bufq_handle == 0) ||
		bufq_index >= BUF_MGR_NUM_BUF_Q ||
		(bufq_index >= buf_mgr->num_buf_q))
		return NULL;

	bufq = &buf_mgr->bufq[bufq_index];
	if (bufq->bufq_handle == bufq_handle)
		return bufq;

	return NULL;
}

static struct msm_isp_buffer *msm_isp_get_buf_ptr(
	struct msm_isp_buf_mgr *buf_mgr,
	uint32_t bufq_handle, uint32_t buf_index)
{
	struct msm_isp_bufq *bufq = NULL;
	struct msm_isp_buffer *buf_info = NULL;

	bufq = msm_isp_get_bufq(buf_mgr, bufq_handle);
	if (!bufq) {
		pr_err("%s: Invalid bufq\n", __func__);
		return buf_info;
	}

	if (bufq->num_bufs <= buf_index) {
		pr_err("%s: Invalid buf index\n", __func__);
		return buf_info;
	}
	buf_info = &bufq->bufs[buf_index];
	return buf_info;
}

static uint32_t msm_isp_get_buf_handle(
	struct msm_isp_buf_mgr *buf_mgr,
	uint32_t session_id, uint32_t stream_id)
{
	int i;
	uint32_t embedded_stream_id = 0;

	for (i = 0; i < buf_mgr->num_buf_q; i++) {
		if (buf_mgr->bufq[i].session_id == session_id &&
			buf_mgr->bufq[i].stream_id == stream_id)
			return 0;
	}

	/* put stream id in handle, if its stats, use FFFF */
	if (stream_id & (1 << 31))
		embedded_stream_id = 0xFFFF;
	else
		embedded_stream_id = stream_id;

	for (i = 0; i < buf_mgr->num_buf_q; i++) {
		if (buf_mgr->bufq[i].bufq_handle == 0) {
			buf_mgr->bufq[i].bufq_handle =
				embedded_stream_id << 8 | i;
			return buf_mgr->bufq[i].bufq_handle;
		}
	}
	return 0;
}

static int msm_isp_free_bufq_handle(struct msm_isp_buf_mgr *buf_mgr,
	uint32_t bufq_handle)
{
	struct msm_isp_bufq *bufq =
		msm_isp_get_bufq(buf_mgr, bufq_handle);
	if (!bufq)
		return -EINVAL;

	/* Set everything except lock to 0 */
	bufq->bufq_handle = 0;
	bufq->bufs = NULL;
	bufq->session_id = 0;
	bufq->stream_id = 0;
	bufq->num_bufs = 0;
	bufq->buf_type = 0;
	INIT_LIST_HEAD(&bufq->head);

	return 0;
}

static void msm_isp_copy_planes_from_v4l2_buffer(
	struct msm_isp_qbuf_buffer *qbuf_buf,
	const struct vb2_buffer *vb2_buf)
{
	int i;

	qbuf_buf->num_planes = vb2_buf->num_planes;
	for (i = 0; i < qbuf_buf->num_planes; i++) {
		qbuf_buf->planes[i].addr = vb2_buf->planes[i].m.userptr;
		qbuf_buf->planes[i].offset = vb2_buf->planes[i].data_offset;
		qbuf_buf->planes[i].length = vb2_buf->planes[i].length;
	}
}

static int msm_isp_prepare_v4l2_buf(struct msm_isp_buf_mgr *buf_mgr,
	struct msm_isp_buffer *buf_info,
	struct msm_isp_qbuf_buffer *qbuf_buf,
	uint32_t stream_id)
{
	int i, rc = -1;
	int ret;
	struct msm_isp_buffer_mapped_info *mapped_info;
#ifndef CONFIG_MSM_ISP_V1
	uint32_t accu_length = 0;
#endif
	struct msm_isp_bufq *bufq = NULL;

	bufq = msm_isp_get_bufq(buf_mgr, buf_info->bufq_handle);
	if (!bufq) {
		pr_err("%s: Invalid bufq, stream id %x\n",
			__func__, stream_id);
		return -EINVAL;
	}

	if (qbuf_buf->num_planes > MAX_PLANES_PER_STREAM) {
		pr_err("%s: Invalid num_planes %d , stream id %x\n",
			__func__, qbuf_buf->num_planes, stream_id);
		return -EINVAL;
	}

	for (i = 0; i < qbuf_buf->num_planes; i++) {
		mapped_info = &buf_info->mapped_info[i];
		mapped_info->buf_fd = qbuf_buf->planes[i].addr;

		if (bufq->security_mode == SECURE_MODE)
			ret = cam_smmu_get_stage2_phy_addr(buf_mgr->iommu_hdl,
					mapped_info->buf_fd,
					CAM_SMMU_MAP_RW,
					&(mapped_info->paddr),
					&(mapped_info->len));
		else
			ret = cam_smmu_get_phy_addr(buf_mgr->iommu_hdl,
					mapped_info->buf_fd,
					CAM_SMMU_MAP_RW,
					&(mapped_info->paddr),
					&(mapped_info->len));
		if (ret) {
			rc = -EINVAL;
			pr_err_ratelimited("%s: cannot map address", __func__);
			goto get_phy_err;
		}

#ifdef CONFIG_MSM_ISP_V1
		mapped_info->paddr += qbuf_buf->planes[i].offset;
#else
		mapped_info->paddr += accu_length;
		accu_length += qbuf_buf->planes[i].length;
#endif

		CDBG("%s: plane: %d addr:%pK\n",
			__func__, i, (void *)mapped_info->paddr);

	}
	buf_info->num_planes = qbuf_buf->num_planes;
	return 0;
get_phy_err:
	i--;

	return rc;
}

static void msm_isp_unprepare_v4l2_buf(
	struct msm_isp_buf_mgr *buf_mgr,
	struct msm_isp_buffer *buf_info,
	uint32_t stream_id)
{
	int i;
	struct msm_isp_buffer_mapped_info *mapped_info;
	struct msm_isp_bufq *bufq = NULL;

	if (!buf_mgr || !buf_info) {
		pr_err("%s: NULL ptr %pK %pK\n", __func__,
			buf_mgr, buf_info);
		return;
	}

	if (buf_info->num_planes > VIDEO_MAX_PLANES) {
		pr_err("%s: Invalid num_planes %d , stream id %x\n",
			__func__, buf_info->num_planes, stream_id);
		return;
	}

	bufq = msm_isp_get_bufq(buf_mgr, buf_info->bufq_handle);
	if (!bufq) {
		pr_err("%s: Invalid bufq, stream id %x\n",
			__func__, stream_id);
		return;
	}

	for (i = 0; i < buf_info->num_planes; i++) {
		mapped_info = &buf_info->mapped_info[i];
		/* SEC_CAM: check any change is needed for secure_mode */
		if (bufq->security_mode == SECURE_MODE)
			cam_smmu_put_stage2_phy_addr(buf_mgr->iommu_hdl,
					mapped_info->buf_fd);
		else
			cam_smmu_put_phy_addr(buf_mgr->iommu_hdl,
					mapped_info->buf_fd);
	}
}

static int msm_isp_map_buf(struct msm_isp_buf_mgr *buf_mgr,
	struct msm_isp_buffer_mapped_info *mapped_info, uint32_t fd)
{
	int rc = 0;
	int ret;

	if (!buf_mgr || !mapped_info) {
		pr_err_ratelimited("%s: %d] NULL ptr buf_mgr %pK mapped_info %pK\n",
			__func__, __LINE__, buf_mgr, mapped_info);
		return -EINVAL;
	}
	if (buf_mgr->secure_enable == SECURE_MODE)
		ret = cam_smmu_get_stage2_phy_addr(buf_mgr->iommu_hdl,
				fd,
				CAM_SMMU_MAP_RW,
				&(mapped_info->paddr),
				&(mapped_info->len));
	else
		ret = cam_smmu_get_phy_addr(buf_mgr->iommu_hdl,
				fd,
				CAM_SMMU_MAP_RW,
				&(mapped_info->paddr),
				&(mapped_info->len));

	if (ret) {
		rc = -EINVAL;
		pr_err_ratelimited("%s: cannot map address", __func__);
		goto smmu_map_error;
	}
	CDBG("%s: addr:%pK\n",
		__func__, (void *)mapped_info->paddr);

	return rc;
smmu_map_error:
	if (buf_mgr->secure_enable == SECURE_MODE)
		cam_smmu_put_stage2_phy_addr(buf_mgr->iommu_hdl,
					fd);
	else
		cam_smmu_put_phy_addr(buf_mgr->iommu_hdl,
			fd);
	return rc;
}

static int msm_isp_unmap_buf(struct msm_isp_buf_mgr *buf_mgr,
	uint32_t fd)
{
	if (!buf_mgr) {
		pr_err_ratelimited("%s: %d] NULL ptr buf_mgr\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	/* SEC_CAMERA: recheck Put part for stats */
	if (buf_mgr->secure_enable == SECURE_MODE)
		cam_smmu_put_stage2_phy_addr(buf_mgr->iommu_hdl,
					fd);
	else
		cam_smmu_put_phy_addr(buf_mgr->iommu_hdl,
			fd);

	return 0;
}

static int msm_isp_buf_prepare(struct msm_isp_buf_mgr *buf_mgr,
	struct msm_isp_qbuf_info *info, struct vb2_v4l2_buffer *vb2_v4l2_buf)
{
	int rc = -1;
	unsigned long flags;
	struct msm_isp_bufq *bufq = NULL;
	struct msm_isp_buffer *buf_info = NULL;
	struct msm_isp_qbuf_buffer buf;

	buf_info = msm_isp_get_buf_ptr(buf_mgr,
		info->handle, info->buf_idx);
	if (!buf_info) {
		pr_err("Invalid buffer prepare\n");
		return rc;
	}

	bufq = msm_isp_get_bufq(buf_mgr, buf_info->bufq_handle);
	if (!bufq) {
		pr_err("%s: Invalid bufq\n",
			__func__);
		return rc;
	}

	spin_lock_irqsave(&bufq->bufq_lock, flags);
	if (buf_info->state == MSM_ISP_BUFFER_STATE_DIVERTED) {
		rc = buf_info->state;
		spin_unlock_irqrestore(&bufq->bufq_lock, flags);
		return rc;
	}

	if (buf_info->state != MSM_ISP_BUFFER_STATE_INITIALIZED) {
		pr_err("%s: Invalid buffer state: %d bufq %x buf-id %d\n",
			__func__, buf_info->state, bufq->bufq_handle,
			buf_info->buf_idx);
		spin_unlock_irqrestore(&bufq->bufq_lock, flags);
		return rc;
	}
	spin_unlock_irqrestore(&bufq->bufq_lock, flags);

	if (vb2_v4l2_buf) {
		msm_isp_copy_planes_from_v4l2_buffer(&buf,
			&vb2_v4l2_buf->vb2_buf);
		buf_info->vb2_v4l2_buf = vb2_v4l2_buf;
	} else {
		buf = info->buffer;
	}

	rc = msm_isp_prepare_v4l2_buf(buf_mgr, buf_info, &buf, bufq->stream_id);
	if (rc < 0) {
		pr_err_ratelimited("%s: Prepare buffer error\n", __func__);
		return rc;
	}

	spin_lock_irqsave(&bufq->bufq_lock, flags);
	buf_info->state = MSM_ISP_BUFFER_STATE_PREPARED;
	spin_unlock_irqrestore(&bufq->bufq_lock, flags);
	return rc;
}

static int msm_isp_buf_unprepare_all(struct msm_isp_buf_mgr *buf_mgr,
	uint32_t buf_handle)
{
	int rc = -1, i;
	struct msm_isp_bufq *bufq = NULL;
	struct msm_isp_buffer *buf_info = NULL;

	bufq = msm_isp_get_bufq(buf_mgr, buf_handle);
	if (!bufq) {
		pr_err("%s: Invalid bufq\n", __func__);
		return rc;
	}

	for (i = 0; i < bufq->num_bufs; i++) {
		buf_info = msm_isp_get_buf_ptr(buf_mgr, buf_handle, i);
		if (!buf_info) {
			pr_err("%s: buf not found\n", __func__);
			return rc;
		}
		if (buf_info->state == MSM_ISP_BUFFER_STATE_UNUSED ||
				buf_info->state ==
					MSM_ISP_BUFFER_STATE_INITIALIZED)
			continue;

		if (BUF_SRC(bufq->stream_id) == MSM_ISP_BUFFER_SRC_HAL) {
			if (buf_info->state == MSM_ISP_BUFFER_STATE_DEQUEUED ||
			buf_info->state == MSM_ISP_BUFFER_STATE_DIVERTED)
				buf_mgr->vb2_ops->put_buf(
					buf_info->vb2_v4l2_buf,
					bufq->session_id, bufq->stream_id);
		}
		msm_isp_unprepare_v4l2_buf(buf_mgr, buf_info, bufq->stream_id);
	}
	return 0;
}

static int msm_isp_get_buf_by_index(struct msm_isp_buf_mgr *buf_mgr,
	uint32_t bufq_handle, uint32_t buf_index,
	struct msm_isp_buffer **buf_info)
{
	int rc = -EINVAL;
	unsigned long flags;
	struct msm_isp_bufq *bufq = NULL;
	struct msm_isp_buffer *temp_buf_info;
	uint32_t i = 0;

	bufq = msm_isp_get_bufq(buf_mgr, bufq_handle);
	if (!bufq) {
		pr_err("%s: Invalid bufq\n", __func__);
		return rc;
	}

	spin_lock_irqsave(&bufq->bufq_lock, flags);
	if (buf_index >= bufq->num_bufs) {
		pr_err("%s: Invalid buf index: %d max: %d\n", __func__,
			buf_index, bufq->num_bufs);
		spin_unlock_irqrestore(&bufq->bufq_lock, flags);
		return rc;
	}

	*buf_info = NULL;
	for (i = 0; bufq->num_bufs; i++) {
		temp_buf_info = &bufq->bufs[i];
		if (temp_buf_info && temp_buf_info->buf_idx == buf_index) {
			*buf_info = temp_buf_info;
			break;
		}
	}

	if (*buf_info) {
		pr_debug("Found buf in isp buf mgr");
		rc = 0;
	}
	spin_unlock_irqrestore(&bufq->bufq_lock, flags);
	return rc;
}

static int msm_isp_buf_unprepare(struct msm_isp_buf_mgr *buf_mgr,
	uint32_t buf_handle, int32_t buf_idx)
{
	struct msm_isp_bufq *bufq = NULL;
	struct msm_isp_buffer *buf_info = NULL;

	bufq = msm_isp_get_bufq(buf_mgr, buf_handle);
	if (!bufq) {
		pr_err("%s: Invalid bufq\n", __func__);
		return -EINVAL;
	}

	buf_info = msm_isp_get_buf_ptr(buf_mgr, buf_handle, buf_idx);
	if (!buf_info) {
		pr_err("%s: buf not found\n", __func__);
		return -EINVAL;
	}
	if (buf_info->state == MSM_ISP_BUFFER_STATE_UNUSED ||
			buf_info->state == MSM_ISP_BUFFER_STATE_INITIALIZED)
		return 0;

	if (BUF_SRC(bufq->stream_id) == MSM_ISP_BUFFER_SRC_HAL) {
		if (buf_info->state == MSM_ISP_BUFFER_STATE_DEQUEUED ||
		buf_info->state == MSM_ISP_BUFFER_STATE_DIVERTED)
			buf_mgr->vb2_ops->put_buf(buf_info->vb2_v4l2_buf,
				bufq->session_id, bufq->stream_id);
	}
	msm_isp_unprepare_v4l2_buf(buf_mgr, buf_info, bufq->stream_id);

	return 0;
}


static int msm_isp_get_buf(struct msm_isp_buf_mgr *buf_mgr, uint32_t id,
	uint32_t bufq_handle, uint32_t buf_index,
	struct msm_isp_buffer **buf_info)
{
	int rc = -1;
	unsigned long flags;
	struct msm_isp_buffer *temp_buf_info = NULL;
	struct msm_isp_bufq *bufq = NULL;
	struct vb2_v4l2_buffer *vb2_v4l2_buf = NULL;

	if (buf_mgr->open_count == 0) {
		pr_err_ratelimited("%s: bug mgr open cnt = 0\n",
			__func__);
		return 0;
	}

	bufq = msm_isp_get_bufq(buf_mgr, bufq_handle);
	if (!bufq) {
		pr_err_ratelimited("%s: Invalid bufq\n", __func__);
		return rc;
	}

	spin_lock_irqsave(&bufq->bufq_lock, flags);
	if (!bufq->bufq_handle) {
		pr_err_ratelimited("%s: Invalid bufq handle\n", __func__);
		spin_unlock_irqrestore(&bufq->bufq_lock, flags);
		return rc;
	}

	*buf_info = NULL;

	switch (BUF_SRC(bufq->stream_id)) {
	case MSM_ISP_BUFFER_SRC_NATIVE:
		list_for_each_entry(temp_buf_info, &bufq->head, list) {
			if (temp_buf_info->state ==
					MSM_ISP_BUFFER_STATE_QUEUED) {
				list_del_init(&temp_buf_info->list);
				if (msm_buf_check_head_sanity(bufq) < 0) {
					spin_unlock_irqrestore(
						&bufq->bufq_lock, flags);
					WARN(1, "%s buf_handle 0x%x buf_idx %d\n",
						__func__,
						bufq->bufq_handle,
						temp_buf_info->buf_idx);
					return -EFAULT;
				}
				*buf_info = temp_buf_info;
				break;
			}
		}
		break;
	case MSM_ISP_BUFFER_SRC_HAL:
		if (buf_index == MSM_ISP_INVALID_BUF_INDEX)
			vb2_v4l2_buf = buf_mgr->vb2_ops->get_buf(
				bufq->session_id, bufq->stream_id);
		else
			vb2_v4l2_buf = buf_mgr->vb2_ops->get_buf_by_idx(
				bufq->session_id, bufq->stream_id,  buf_index);
		if (vb2_v4l2_buf) {
			if (vb2_v4l2_buf->vb2_buf.index < bufq->num_bufs) {
				*buf_info = &bufq->bufs[vb2_v4l2_buf
						->vb2_buf.index];
				(*buf_info)->vb2_v4l2_buf = vb2_v4l2_buf;
			} else {
				pr_err("%s: Incorrect buf index %d\n",
					__func__, vb2_v4l2_buf->vb2_buf.index);
				rc = -EINVAL;
			}
			if ((*buf_info) == NULL) {
				buf_mgr->vb2_ops->put_buf(vb2_v4l2_buf,
					bufq->session_id, bufq->stream_id);
				pr_err("%s: buf index %d not found!\n",
					__func__, vb2_v4l2_buf->vb2_buf.index);
				rc = -EINVAL;

			}
		} else {
			CDBG("%s: No HAL Buffer session_id: %d stream_id: %d\n",
				__func__, bufq->session_id, bufq->stream_id);
			rc = -EINVAL;
		}
		break;
	case MSM_ISP_BUFFER_SRC_SCRATCH:
		/* In scratch buf case we have only on buffer in queue.
		 * We return every time same buffer.
		 */
		*buf_info = list_entry(bufq->head.next, typeof(**buf_info),
				list);
		break;
	default:
		pr_err("%s: Incorrect buf source.\n", __func__);
		rc = -EINVAL;
		spin_unlock_irqrestore(&bufq->bufq_lock, flags);
		return rc;
	}

	if (!(*buf_info)) {
		rc = -ENOMEM;
	} else {
		(*buf_info)->state = MSM_ISP_BUFFER_STATE_DEQUEUED;
		rc = 0;
	}
	spin_unlock_irqrestore(&bufq->bufq_lock, flags);
	return rc;
}

static int msm_isp_put_buf_unsafe(struct msm_isp_buf_mgr *buf_mgr,
	uint32_t bufq_handle, uint32_t buf_index)
{
	int rc = -1;
	struct msm_isp_bufq *bufq = NULL;
	struct msm_isp_buffer *buf_info = NULL;

	bufq = msm_isp_get_bufq(buf_mgr, bufq_handle);
	if (!bufq) {
		pr_err("%s: Invalid bufq\n", __func__);
		return rc;
	}

	buf_info = msm_isp_get_buf_ptr(buf_mgr, bufq_handle, buf_index);
	if (!buf_info) {
		pr_err("%s: buf not found\n", __func__);
		return rc;
	}

	switch (buf_info->state) {
	case MSM_ISP_BUFFER_STATE_PREPARED:
	case MSM_ISP_BUFFER_STATE_DEQUEUED:
		if (BUF_SRC(bufq->stream_id)) {
			if (!list_empty(&buf_info->list)) {
				WARN(1, "%s: buf %x/%x double add\n",
					__func__, bufq_handle, buf_index);
				return -EFAULT;
			}
			list_add_tail(&buf_info->list, &bufq->head);
			if (msm_buf_check_head_sanity(bufq) < 0) {
				WARN(1, "%s buf_handle 0x%x buf_idx %d\n",
					__func__,
					bufq->bufq_handle,
					buf_info->buf_idx);
				return -EFAULT;
			}
		} else {
			buf_mgr->vb2_ops->put_buf(buf_info->vb2_v4l2_buf,
				bufq->session_id, bufq->stream_id);
		}
		buf_info->state = MSM_ISP_BUFFER_STATE_QUEUED;
		rc = 0;
		break;
	case MSM_ISP_BUFFER_STATE_DISPATCHED:
		buf_info->state = MSM_ISP_BUFFER_STATE_QUEUED;
		rc = 0;
		break;
	case MSM_ISP_BUFFER_STATE_QUEUED:
		if (IS_ENABLED(CONFIG_MSM_ISP_V1)) {
			rc = 0;
			break;
		}
	case MSM_ISP_BUFFER_STATE_DIVERTED:
	default:
		WARN(1, "%s: bufq 0x%x, buf idx 0x%x, incorrect state = %d",
			__func__, bufq_handle, buf_index, buf_info->state);
		return -EFAULT;
	}

	return rc;
}

static int msm_isp_put_buf(struct msm_isp_buf_mgr *buf_mgr,
	uint32_t bufq_handle, uint32_t buf_index)
{
	int rc = -1;
	unsigned long flags;
	struct msm_isp_bufq *bufq = NULL;
	struct msm_isp_buffer *buf_info = NULL;

	bufq = msm_isp_get_bufq(buf_mgr, bufq_handle);
	if (!bufq) {
		pr_err("%s: Invalid bufq\n", __func__);
		return rc;
	}

	buf_info = msm_isp_get_buf_ptr(buf_mgr, bufq_handle, buf_index);
	if (!buf_info) {
		pr_err("%s: buf not found\n", __func__);
		return rc;
	}

	spin_lock_irqsave(&bufq->bufq_lock, flags);

	rc = msm_isp_put_buf_unsafe(buf_mgr, bufq_handle, buf_index);

	spin_unlock_irqrestore(&bufq->bufq_lock, flags);

	return rc;
}

static int msm_isp_buf_divert(struct msm_isp_buf_mgr *buf_mgr,
	uint32_t bufq_handle, uint32_t buf_index,
	struct timeval *tv, uint32_t frame_id)
{
	unsigned long flags;
	struct msm_isp_bufq *bufq = NULL;
	struct msm_isp_buffer *buf_info = NULL;

	bufq = msm_isp_get_bufq(buf_mgr, bufq_handle);
	if (!bufq) {
		pr_err("Invalid bufq\n");
		return -EINVAL;
	}

	buf_info = msm_isp_get_buf_ptr(buf_mgr, bufq_handle, buf_index);
	if (!buf_info) {
		pr_err("%s: buf not found\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&bufq->bufq_lock, flags);

	buf_info->frame_id = frame_id;
#ifdef CONFIG_MSM_ISP_V1
	if (buf_info->state == MSM_ISP_BUFFER_STATE_DEQUEUED) {
		buf_info->state = MSM_ISP_BUFFER_STATE_DIVERTED;
		buf_info->tv = tv;
	}
#else
	if (BUF_SRC(bufq->stream_id) == MSM_ISP_BUFFER_SRC_NATIVE) {
		buf_info->state = MSM_ISP_BUFFER_STATE_DIVERTED;
		buf_info->tv = tv;
	}
#endif
	spin_unlock_irqrestore(&bufq->bufq_lock, flags);
	return 0;
}

static int msm_isp_buf_done(struct msm_isp_buf_mgr *buf_mgr,
	uint32_t bufq_handle, uint32_t buf_index,
	struct timeval *tv, uint32_t frame_id, uint32_t output_format)
{
	int rc = 0;
	unsigned long flags;
	struct msm_isp_bufq *bufq = NULL;
	struct msm_isp_buffer *buf_info = NULL;
	enum msm_isp_buffer_state state;

	bufq = msm_isp_get_bufq(buf_mgr, bufq_handle);
	if (!bufq) {
		pr_err("Invalid bufq\n");
		return -EINVAL;
	}

	buf_info = msm_isp_get_buf_ptr(buf_mgr, bufq_handle, buf_index);
	if (!buf_info) {
		pr_err("%s: buf not found\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&bufq->bufq_lock, flags);
	state = buf_info->state;

	if (BUF_SRC(bufq->stream_id) == MSM_ISP_BUFFER_SRC_HAL) {
		if (state == MSM_ISP_BUFFER_STATE_DEQUEUED) {
			buf_info->state = MSM_ISP_BUFFER_STATE_DISPATCHED;
			spin_unlock_irqrestore(&bufq->bufq_lock, flags);
			buf_mgr->vb2_ops->buf_done(buf_info->vb2_v4l2_buf,
				bufq->session_id, bufq->stream_id,
				frame_id, tv, output_format);
		} else {
			spin_unlock_irqrestore(&bufq->bufq_lock, flags);
		}
		goto done;
	}

	/*
	 * For native buffer put the diverted buffer back to queue since caller
	 * is not going to send it to CPP, this is error case like
	 * drop_frame/empty_buffer
	 */
	if (state == MSM_ISP_BUFFER_STATE_DIVERTED) {
		buf_info->state = MSM_ISP_BUFFER_STATE_PREPARED;
		rc = msm_isp_put_buf_unsafe(buf_mgr, buf_info->bufq_handle,
			buf_info->buf_idx);
		if (rc < 0)
			pr_err("%s: Buf put failed\n", __func__);
	}
	spin_unlock_irqrestore(&bufq->bufq_lock, flags);
done:
	return rc;
}

static int msm_isp_flush_buf(struct msm_isp_buf_mgr *buf_mgr,
	uint32_t bufq_handle, enum msm_isp_buffer_flush_t flush_type,
	struct timeval *tv, uint32_t frame_id)
{
	int i;
	struct msm_isp_bufq *bufq = NULL;
	struct msm_isp_buffer *buf_info = NULL;
	unsigned long flags;

	bufq = msm_isp_get_bufq(buf_mgr, bufq_handle);
	if (!bufq) {
		pr_err("Invalid bufq\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&bufq->bufq_lock, flags);
	for (i = 0; i < bufq->num_bufs; i++) {
		buf_info = msm_isp_get_buf_ptr(buf_mgr, bufq_handle, i);
		if (!buf_info) {
			pr_err("%s: buf not found\n", __func__);
			continue;
		}
		switch (flush_type) {
		case MSM_ISP_BUFFER_FLUSH_DIVERTED:
			if (buf_info->state !=
				MSM_ISP_BUFFER_STATE_DIVERTED)
				continue;
			buf_info->state = MSM_ISP_BUFFER_STATE_PREPARED;
			msm_isp_put_buf_unsafe(buf_mgr,
				bufq_handle, buf_info->buf_idx);
			break;
		case MSM_ISP_BUFFER_FLUSH_ALL:
			if (buf_info->state ==
				MSM_ISP_BUFFER_STATE_DIVERTED)
				continue;
			if (buf_info->state !=
				MSM_ISP_BUFFER_STATE_DEQUEUED)
				continue;
			msm_isp_put_buf_unsafe(buf_mgr,
				bufq_handle, buf_info->buf_idx);
			break;
		default:
			WARN(1, "Invalid flush type %d\n", flush_type);
		}
	}

	spin_unlock_irqrestore(&bufq->bufq_lock, flags);
	return 0;
}

static int msm_isp_buf_enqueue(struct msm_isp_buf_mgr *buf_mgr,
	struct msm_isp_qbuf_info *info)
{
	int rc = 0, buf_state;
	struct msm_isp_bufq *bufq = NULL;
	struct msm_isp_buffer *buf_info = NULL;

	bufq = msm_isp_get_bufq(buf_mgr, info->handle);
	if (!bufq) {
		pr_err("%s: Invalid bufq, handle 0x%x, stream id %x num_plane %d\n"
			, __func__, info->handle, (info->handle >> 8),
			info->buffer.num_planes);
		return -EINVAL;
	}

	buf_state = msm_isp_buf_prepare(buf_mgr, info, NULL);
	if (buf_state < 0) {
		pr_err_ratelimited("%s: Buf prepare failed\n", __func__);
		return -EINVAL;
	}

	if (buf_state == MSM_ISP_BUFFER_STATE_DIVERTED) {
		buf_info = msm_isp_get_buf_ptr(buf_mgr,
						info->handle, info->buf_idx);
		if (!buf_info) {
			pr_err("%s: buf not found\n", __func__);
			return -EINVAL;
		}
		if (info->dirty_buf) {
			buf_info->buf_debug.put_state[
				buf_info->buf_debug.put_state_last]
				= MSM_ISP_BUFFER_STATE_PUT_BUF;
			buf_info->buf_debug.put_state_last ^= 1;
			buf_info->state = MSM_ISP_BUFFER_STATE_PREPARED;
			rc = msm_isp_put_buf(buf_mgr,
				info->handle, info->buf_idx);
		} else {
			if (BUF_SRC(bufq->stream_id))
				pr_err("%s: Invalid native buffer state\n",
					__func__);
			else {
				buf_info->buf_debug.put_state[
					buf_info->buf_debug.put_state_last] =
					MSM_ISP_BUFFER_STATE_PUT_BUF;
				buf_info->buf_debug.put_state_last ^= 1;
				rc = msm_isp_buf_done(buf_mgr,
					info->handle, info->buf_idx,
					buf_info->tv, buf_info->frame_id, 0);
			}
		}
	} else {
		if (BUF_SRC(bufq->stream_id) != MSM_ISP_BUFFER_SRC_HAL) {
			buf_info = msm_isp_get_buf_ptr(buf_mgr,
				info->handle, info->buf_idx);
			if (!buf_info) {
				pr_err("%s: buf not found\n", __func__);
				return -EINVAL;
			}

			buf_info->buf_debug.put_state[
				buf_info->buf_debug.put_state_last] =
				MSM_ISP_BUFFER_STATE_PUT_PREPARED;
			buf_info->buf_debug.put_state_last ^= 1;
			rc = msm_isp_put_buf(buf_mgr,
					info->handle, info->buf_idx);
			if (rc < 0) {
				pr_err("%s: Buf put failed stream %x\n",
					__func__, bufq->stream_id);
				return rc;
			}
		}
	}
	return 0;
}

static int msm_isp_buf_dequeue(struct msm_isp_buf_mgr *buf_mgr,
	struct msm_isp_qbuf_info *info)
{
	struct msm_isp_buffer *buf_info = NULL;
	int rc = 0;

	buf_info = msm_isp_get_buf_ptr(buf_mgr, info->handle, info->buf_idx);
	if (!buf_info) {
		pr_err("Invalid buffer dequeue\n");
		return -EINVAL;
	}

	if (buf_info->state == MSM_ISP_BUFFER_STATE_DEQUEUED ||
		buf_info->state == MSM_ISP_BUFFER_STATE_DIVERTED) {
		pr_err("%s: Invalid state %d\n", __func__, buf_info->state);
		return -EINVAL;
	}
	msm_isp_buf_unprepare(buf_mgr, info->handle, info->buf_idx);

	buf_info->state = MSM_ISP_BUFFER_STATE_INITIALIZED;

	return rc;
}

static int msm_isp_get_bufq_handle(struct msm_isp_buf_mgr *buf_mgr,
	uint32_t session_id, uint32_t stream_id)
{
	int i;

	for (i = 0; i < buf_mgr->num_buf_q; i++) {
		if (buf_mgr->bufq[i].session_id == session_id &&
			buf_mgr->bufq[i].stream_id == stream_id) {
			return buf_mgr->bufq[i].bufq_handle;
		}
	}
	pr_err("%s: No match found 0x%x 0x%x\n", __func__,
			session_id, stream_id);
	return 0;
}

static int msm_isp_get_buf_src(struct msm_isp_buf_mgr *buf_mgr,
	uint32_t bufq_handle, uint32_t *buf_src)
{
	struct msm_isp_bufq *bufq = NULL;

	bufq = msm_isp_get_bufq(buf_mgr, bufq_handle);
	if (!bufq) {
		pr_err("%s: Invalid bufq\n",
			__func__);
		return -EINVAL;
	}
	*buf_src = BUF_SRC(bufq->stream_id);

	return 0;
}

static int msm_isp_request_bufq(struct msm_isp_buf_mgr *buf_mgr,
	struct msm_isp_buf_request_ver2 *buf_request)
{
	int i;
	struct msm_isp_bufq *bufq = NULL;

	CDBG("%s: E\n", __func__);

	if (!buf_request->num_buf || buf_request->num_buf > VB2_MAX_FRAME) {
		pr_err("Invalid buffer request\n");
		return -EINVAL;
	}

	buf_request->handle = msm_isp_get_buf_handle(buf_mgr,
		buf_request->session_id, buf_request->stream_id);
	if (!buf_request->handle) {
		pr_err("Invalid buffer handle\n");
		return -EINVAL;
	}

	bufq = msm_isp_get_bufq(buf_mgr, buf_request->handle);
	if (!bufq) {
		pr_err("%s: Invalid bufq stream_id %x\n",
			__func__, buf_request->stream_id);

		return -EINVAL;
	}

	bufq->bufs = kzalloc(sizeof(struct msm_isp_buffer) *
		buf_request->num_buf, GFP_KERNEL);
	if (!bufq->bufs) {
		msm_isp_free_bufq_handle(buf_mgr, buf_request->handle);
		return -ENOMEM;
	}
	spin_lock_init(&bufq->bufq_lock);
	bufq->bufq_handle = buf_request->handle;
	bufq->session_id = buf_request->session_id;
	bufq->stream_id = buf_request->stream_id;
	bufq->num_bufs = buf_request->num_buf;
	bufq->buf_type = buf_request->buf_type;
	INIT_LIST_HEAD(&bufq->head);
	bufq->security_mode = buf_request->security_mode;

	for (i = 0; i < buf_request->num_buf; i++) {
		bufq->bufs[i].state = MSM_ISP_BUFFER_STATE_INITIALIZED;
		bufq->bufs[i].buf_debug.put_state[0] =
			MSM_ISP_BUFFER_STATE_PUT_PREPARED;
		bufq->bufs[i].buf_debug.put_state[1] =
			MSM_ISP_BUFFER_STATE_PUT_PREPARED;
		bufq->bufs[i].buf_debug.put_state_last = 0;
		bufq->bufs[i].bufq_handle = bufq->bufq_handle;
		bufq->bufs[i].buf_idx = i;
		INIT_LIST_HEAD(&bufq->bufs[i].list);
	}

	return 0;
}

static int msm_isp_release_bufq(struct msm_isp_buf_mgr *buf_mgr,
	uint32_t bufq_handle)
{
	struct msm_isp_bufq *bufq = NULL;
	unsigned long flags;

	bufq = msm_isp_get_bufq(buf_mgr, bufq_handle);
	if (!bufq) {
		pr_err("Invalid bufq release\n");
		return -EINVAL;
	}

	msm_isp_buf_unprepare_all(buf_mgr, bufq_handle);

	spin_lock_irqsave(&bufq->bufq_lock, flags);
	kfree(bufq->bufs);
	msm_isp_free_bufq_handle(buf_mgr, bufq_handle);
	spin_unlock_irqrestore(&bufq->bufq_lock, flags);

	return 0;
}

static void msm_isp_release_all_bufq(
	struct msm_isp_buf_mgr *buf_mgr)
{
	struct msm_isp_bufq *bufq = NULL;
	unsigned long flags;
	int i;

	for (i = 0; i < buf_mgr->num_buf_q; i++) {
		bufq = &buf_mgr->bufq[i];
		if (!bufq->bufq_handle)
			continue;

		msm_isp_buf_unprepare_all(buf_mgr, bufq->bufq_handle);

		spin_lock_irqsave(&bufq->bufq_lock, flags);
		kfree(bufq->bufs);
		msm_isp_free_bufq_handle(buf_mgr, bufq->bufq_handle);
		spin_unlock_irqrestore(&bufq->bufq_lock, flags);
	}
}

/**
 * msm_isp_buf_put_scratch() - Release scratch buffers
 * @buf_mgr: The buffer structure for h/w
 *
 * Returns 0 on success else error code
 */
static int msm_isp_buf_put_scratch(struct msm_isp_buf_mgr *buf_mgr)
{
	int rc;

	if (!buf_mgr->scratch_buf_addr)
		return 0;

	if (buf_mgr->secure_enable == SECURE_MODE) {
		rc = cam_smmu_free_stage2_scratch_mem(buf_mgr->iommu_hdl,
				buf_mgr->dmabuf);
		if (buf_mgr->scratch_buf_stats_addr)
			rc = cam_smmu_put_phy_addr_scratch(buf_mgr->iommu_hdl,
				buf_mgr->scratch_buf_stats_addr);
	} else {
		rc = cam_smmu_put_phy_addr_scratch(buf_mgr->iommu_hdl,
				buf_mgr->scratch_buf_addr);
	}
	if (rc)
		pr_err("%s: failed to put scratch buffer to img iommu: %d\n",
			__func__, rc);


	if (!rc) {
		buf_mgr->scratch_buf_addr = 0;
		buf_mgr->scratch_buf_stats_addr = 0;
	}

	return rc;
}

/**
 * msm_isp_buf_get_scratch() - Create scratch buffers
 * @buf_mgr: The buffer structure for h/w
 *
 * Create and map scratch buffers for all IOMMU's under the buffer
 * manager.
 *
 * Returns 0 on success else error code
 */
static int msm_isp_buf_get_scratch(struct msm_isp_buf_mgr *buf_mgr)
{
	int rc;
	size_t range = buf_mgr->scratch_buf_range;

	if (buf_mgr->scratch_buf_addr || !buf_mgr->scratch_buf_range)
		/* already mapped or not supported */
		return 0;

	if (buf_mgr->secure_enable == SECURE_MODE) {
		rc = cam_smmu_alloc_get_stage2_scratch_mem(buf_mgr->iommu_hdl,
				CAM_SMMU_MAP_RW,
				&buf_mgr->dmabuf,
				&buf_mgr->scratch_buf_addr,
				&range);
		if (rc)
			goto done;

		rc = cam_smmu_get_phy_addr_scratch(
				buf_mgr->iommu_hdl,
				CAM_SMMU_MAP_RW,
				&buf_mgr->scratch_buf_stats_addr,
				buf_mgr->scratch_buf_range,
				SZ_4K);
		if (rc)
			msm_isp_buf_put_scratch(buf_mgr);
	} else {
		rc = cam_smmu_get_phy_addr_scratch(
				buf_mgr->iommu_hdl,
				CAM_SMMU_MAP_RW,
				&buf_mgr->scratch_buf_addr,
				buf_mgr->scratch_buf_range,
				SZ_4K);
		buf_mgr->scratch_buf_stats_addr = buf_mgr->scratch_buf_addr;
	}
done:
	if (rc) {
		pr_err("%s: failed to map scratch buffer to img iommu: %d\n",
			__func__, rc);
		return rc;
	}
	return rc;
}

int msm_isp_smmu_attach(struct msm_isp_buf_mgr *buf_mgr,
	void *arg)
{
	struct msm_vfe_smmu_attach_cmd *cmd = arg;
	int rc = 0;
	int32_t stall_disable = 1;

	pr_debug("%s: cmd->security_mode : %d\n", __func__, cmd->security_mode);

	mutex_lock(&buf_mgr->lock);
	if (cmd->iommu_attach_mode == IOMMU_ATTACH) {
		/* disable smmu stall on fault */
		cam_smmu_set_attr(buf_mgr->iommu_hdl,
			DOMAIN_ATTR_CB_STALL_DISABLE, &stall_disable);
		/*
		 * Call hypervisor thru scm call to notify secure or
		 * non-secure mode
		 */
		if (buf_mgr->attach_ref_cnt == 0) {
			if (cmd->security_mode == SECURE_MODE)
				rc = cam_smmu_ops(buf_mgr->iommu_hdl,
					CAM_SMMU_ATTACH_SEC_VFE_NS_STATS);
			else
				rc = cam_smmu_ops(buf_mgr->iommu_hdl,
					CAM_SMMU_ATTACH);
			if (rc < 0) {
				pr_err("%s: img smmu attach error, rc :%d\n",
					__func__, rc);
				goto err1;
			}
			buf_mgr->secure_enable = cmd->security_mode;
		}
		buf_mgr->attach_ref_cnt++;
		rc = msm_isp_buf_get_scratch(buf_mgr);
		if (rc)
			goto err2;
	}

	mutex_unlock(&buf_mgr->lock);
	return rc;

err2:
	if (buf_mgr->secure_enable == SECURE_MODE)
		cam_smmu_ops(buf_mgr->iommu_hdl,
				CAM_SMMU_DETACH_SEC_VFE_NS_STATS);
	else
		cam_smmu_ops(buf_mgr->iommu_hdl, CAM_SMMU_DETACH);
err1:
	mutex_unlock(&buf_mgr->lock);
	return rc;
}

static int msm_isp_init_isp_buf_mgr(struct msm_isp_buf_mgr *buf_mgr,
	const char *ctx_name)
{
	int rc = -1;
	int i = 0;

	mutex_lock(&buf_mgr->lock);
	if (buf_mgr->open_count++) {
		mutex_unlock(&buf_mgr->lock);
		return 0;
	}

	CDBG("%s: E\n", __func__);
	buf_mgr->attach_ref_cnt = 0;

	buf_mgr->num_buf_q = BUF_MGR_NUM_BUF_Q;
	memset(buf_mgr->bufq, 0, sizeof(buf_mgr->bufq));

	rc = cam_smmu_get_handle("vfe", &buf_mgr->iommu_hdl);
	if (rc < 0) {
		pr_err("vfe get handle failed\n");
		goto get_handle_error;
	}

	for (i = 0; i < BUF_MGR_NUM_BUF_Q; i++)
		spin_lock_init(&buf_mgr->bufq[i].bufq_lock);

	buf_mgr->pagefault_debug_disable = 0;
	buf_mgr->frameId_mismatch_recovery = 0;
get_handle_error:
	mutex_unlock(&buf_mgr->lock);
	return 0;
}

static int msm_isp_deinit_isp_buf_mgr(
	struct msm_isp_buf_mgr *buf_mgr)
{
	mutex_lock(&buf_mgr->lock);
	if (buf_mgr->open_count > 0)
		buf_mgr->open_count--;

	if (buf_mgr->open_count) {
		mutex_unlock(&buf_mgr->lock);
		return 0;
	}
	msm_isp_release_all_bufq(buf_mgr);
	buf_mgr->num_buf_q = 0;
	buf_mgr->pagefault_debug_disable = 0;

	msm_isp_buf_put_scratch(buf_mgr);
	if (buf_mgr->attach_ref_cnt != 0) {
		if (buf_mgr->secure_enable == SECURE_MODE)
			cam_smmu_ops(buf_mgr->iommu_hdl,
				CAM_SMMU_DETACH_SEC_VFE_NS_STATS);
		else
			cam_smmu_ops(buf_mgr->iommu_hdl, CAM_SMMU_DETACH);
	}
	cam_smmu_destroy_handle(buf_mgr->iommu_hdl);
	buf_mgr->attach_ref_cnt = 0;
	buf_mgr->secure_enable = 0;
	buf_mgr->attach_ref_cnt = 0;
	mutex_unlock(&buf_mgr->lock);
	return 0;
}

int msm_isp_proc_buf_cmd(struct msm_isp_buf_mgr *buf_mgr,
	unsigned int cmd, void *arg)
{
	int rc = -EINVAL;

	switch (cmd) {
	case VIDIOC_MSM_ISP_REQUEST_BUF: {
		struct msm_isp_buf_request *buf_req = arg;
		struct msm_isp_buf_request_ver2 buf_req_ver2;

		memcpy(&buf_req_ver2, buf_req,
			sizeof(struct msm_isp_buf_request));
		buf_req_ver2.security_mode = NON_SECURE_MODE;
		rc = buf_mgr->ops->request_buf(buf_mgr, &buf_req_ver2);
		memcpy(buf_req, &buf_req_ver2,
			sizeof(struct msm_isp_buf_request));
		break;
	}
	case VIDIOC_MSM_ISP_REQUEST_BUF_VER2: {
		struct msm_isp_buf_request_ver2 *buf_req_ver2 = arg;

		rc = buf_mgr->ops->request_buf(buf_mgr, buf_req_ver2);
		break;
	}
	case VIDIOC_MSM_ISP_ENQUEUE_BUF: {
		struct msm_isp_qbuf_info *qbuf_info = arg;

		rc = buf_mgr->ops->enqueue_buf(buf_mgr, qbuf_info);
		break;
	}
	case VIDIOC_MSM_ISP_DEQUEUE_BUF: {
		struct msm_isp_qbuf_info *qbuf_info = arg;

		rc = buf_mgr->ops->dequeue_buf(buf_mgr, qbuf_info);
		break;
	}
	case VIDIOC_MSM_ISP_RELEASE_BUF: {
		struct msm_isp_buf_request *buf_req = arg;

		rc = buf_mgr->ops->release_buf(buf_mgr, buf_req->handle);
		break;
	}
	case VIDIOC_MSM_ISP_UNMAP_BUF: {
		struct msm_isp_unmap_buf_req *unmap_req = arg;

		rc = buf_mgr->ops->unmap_buf(buf_mgr, unmap_req->fd);
		break;
	}
	}
	return rc;
}

static int msm_isp_buf_mgr_debug(struct msm_isp_buf_mgr *buf_mgr,
	unsigned long fault_addr)
{
	struct msm_isp_buffer *bufs = NULL;
	uint32_t i = 0, j = 0, k = 0, rc = 0;
	char *print_buf = NULL, temp_buf[100];
	uint32_t print_buf_size = 2000;
	unsigned long start_addr = 0, end_addr = 0;
	int buf_addr_delta = -1;
	int temp_delta = 0;
	uint32_t debug_stream_id = 0;
	uint32_t debug_buf_idx = 0;
	uint32_t debug_buf_plane = 0;
	unsigned long debug_start_addr = 0;
	unsigned long debug_end_addr = 0;
	uint32_t debug_frame_id = 0;
	enum msm_isp_buffer_state debug_state = MSM_ISP_BUFFER_STATE_UNUSED;
	unsigned long flags;
	struct msm_isp_bufq *bufq = NULL;

	if (!buf_mgr) {
		pr_err_ratelimited("%s: %d] NULL buf_mgr\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	for (i = 0; i < BUF_MGR_NUM_BUF_Q; i++) {
		bufq = &buf_mgr->bufq[i];

		spin_lock_irqsave(&bufq->bufq_lock, flags);
		if (!bufq->bufq_handle) {
			spin_unlock_irqrestore(&bufq->bufq_lock, flags);
			continue;
		}

		for (j = 0; j < bufq->num_bufs; j++) {
			bufs = &bufq->bufs[j];
			if (!bufs)
				continue;

			for (k = 0; k < bufs->num_planes; k++) {
				start_addr =
					bufs->mapped_info[k].paddr;
				end_addr = bufs->mapped_info[k].paddr +
					bufs->mapped_info[k].len - 1;
				temp_delta = fault_addr - start_addr;
				if (temp_delta < 0)
					continue;

				if (buf_addr_delta == -1 ||
					temp_delta < buf_addr_delta) {
					buf_addr_delta = temp_delta;
					debug_stream_id = bufq->stream_id;
					debug_buf_idx = j;
					debug_buf_plane = k;
					debug_start_addr = start_addr;
					debug_end_addr = end_addr;
					debug_frame_id = bufs->frame_id;
					debug_state = bufs->state;
				}
			}
		}
		start_addr = 0;
		end_addr = 0;
		spin_unlock_irqrestore(&bufq->bufq_lock, flags);
	}

	pr_err("%s: ==== SMMU page fault addr %lx ====\n", __func__,
		fault_addr);
	pr_err("%s: nearby stream id %x, frame_id %d\n", __func__,
		debug_stream_id, debug_frame_id);
	pr_err("%s: nearby buf index %d, plane %d, state %d\n", __func__,
		debug_buf_idx, debug_buf_plane, debug_state);
	pr_err("%s: buf address %pK -- %pK\n", __func__,
		(void *)debug_start_addr, (void *)debug_end_addr);

	if (BUF_DEBUG_FULL) {
		print_buf = kzalloc(print_buf_size, GFP_ATOMIC);
		if (!print_buf)
			return -ENOMEM;
		snprintf(print_buf, print_buf_size, "%s\n", __func__);
		for (i = 0; i < BUF_MGR_NUM_BUF_Q; i++) {
			if (i % 2 == 0 && i > 0) {
				pr_err("%s\n", print_buf);
				print_buf[0] = 0;
			}
			if (buf_mgr->bufq[i].bufq_handle != 0) {
				snprintf(temp_buf, sizeof(temp_buf),
					"handle %x stream %x num_bufs %d\n",
					buf_mgr->bufq[i].bufq_handle,
					buf_mgr->bufq[i].stream_id,
					buf_mgr->bufq[i].num_bufs);
				strlcat(print_buf, temp_buf, print_buf_size);
				for (j = 0; j < buf_mgr->bufq[i].num_bufs;
					j++) {
					bufs =
						&buf_mgr->bufq[i].bufs[j];
					if (!bufs)
						break;

					for (k = 0; k < bufs->num_planes; k++) {
						start_addr =
							bufs->mapped_info[
							k].paddr;
						end_addr =
							bufs->mapped_info[
							k].paddr +
							bufs->mapped_info[
							k].len;
						snprintf(temp_buf,
							sizeof(temp_buf),
							"buf%d plane%d", j, k);
						snprintf(temp_buf,
							sizeof(temp_buf),
							"start_addr %pK",
							(void *)start_addr);
						snprintf(temp_buf,
							sizeof(temp_buf),
							"end_addr %pK\n",
							(void *)end_addr);
						strlcat(print_buf, temp_buf,
							print_buf_size);
					}
				}
				start_addr = 0;
				end_addr = 0;
			}
		}
		pr_err("%s\n", print_buf);
		kfree(print_buf);
	}
	return rc;
}

static struct msm_isp_buf_ops isp_buf_ops = {
	.request_buf = msm_isp_request_bufq,
	.enqueue_buf = msm_isp_buf_enqueue,
	.dequeue_buf = msm_isp_buf_dequeue,
	.release_buf = msm_isp_release_bufq,
	.get_bufq_handle = msm_isp_get_bufq_handle,
	.get_buf_src = msm_isp_get_buf_src,
	.get_buf = msm_isp_get_buf,
	.get_buf_by_index = msm_isp_get_buf_by_index,
	.map_buf = msm_isp_map_buf,
	.unmap_buf = msm_isp_unmap_buf,
	.put_buf = msm_isp_put_buf,
	.flush_buf = msm_isp_flush_buf,
	.buf_done = msm_isp_buf_done,
	.buf_mgr_init = msm_isp_init_isp_buf_mgr,
	.buf_mgr_deinit = msm_isp_deinit_isp_buf_mgr,
	.buf_mgr_debug = msm_isp_buf_mgr_debug,
	.get_bufq = msm_isp_get_bufq,
	.buf_divert = msm_isp_buf_divert,
};

int msm_isp_create_isp_buf_mgr(
	struct msm_isp_buf_mgr *buf_mgr,
	struct msm_sd_req_vb2_q *vb2_ops,
	struct device *dev,
	uint32_t scratch_buf_range)
{
	int rc = 0;

	if (buf_mgr->init_done)
		return rc;

	buf_mgr->ops = &isp_buf_ops;
	buf_mgr->vb2_ops = vb2_ops;
	buf_mgr->open_count = 0;
	buf_mgr->pagefault_debug_disable = 0;
	buf_mgr->secure_enable = NON_SECURE_MODE;
	buf_mgr->scratch_buf_range = scratch_buf_range;
	mutex_init(&buf_mgr->lock);

	return 0;
}
