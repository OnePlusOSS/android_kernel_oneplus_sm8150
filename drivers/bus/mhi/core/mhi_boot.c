/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/mhi.h>
#include "mhi_internal.h"


/* setup rddm vector table for rddm transfer and program rxvec */
void mhi_rddm_prepare(struct mhi_controller *mhi_cntrl,
			     struct image_info *img_info)
{
	struct mhi_buf *mhi_buf = img_info->mhi_buf;
	struct bhi_vec_entry *bhi_vec = img_info->bhi_vec;
	void __iomem *base = mhi_cntrl->bhie;
	u32 sequence_id;
	int i = 0;

	for (i = 0; i < img_info->entries - 1; i++, mhi_buf++, bhi_vec++) {
		MHI_VERB("Setting vector:%pad size:%zu\n",
			 &mhi_buf->dma_addr, mhi_buf->len);
		bhi_vec->dma_addr = mhi_buf->dma_addr;
		bhi_vec->size = mhi_buf->len;
	}

	MHI_LOG("BHIe programming for RDDM\n");

	mhi_cntrl->write_reg(mhi_cntrl, base, BHIE_RXVECADDR_HIGH_OFFS,
			upper_32_bits(mhi_buf->dma_addr));

	mhi_cntrl->write_reg(mhi_cntrl, base, BHIE_RXVECADDR_LOW_OFFS,
			lower_32_bits(mhi_buf->dma_addr));

	mhi_cntrl->write_reg(mhi_cntrl, base, BHIE_RXVECSIZE_OFFS,
			mhi_buf->len);
	sequence_id = prandom_u32() & BHIE_RXVECSTATUS_SEQNUM_BMSK;

	if (unlikely(!sequence_id))
		sequence_id = 1;

	mhi_write_reg_field(mhi_cntrl, base, BHIE_RXVECDB_OFFS,
			    BHIE_RXVECDB_SEQNUM_BMSK, BHIE_RXVECDB_SEQNUM_SHFT,
			    sequence_id);

	MHI_LOG("address:%pad len:0x%lx sequence:%u\n",
		&mhi_buf->dma_addr, mhi_buf->len, sequence_id);
}

/* collect rddm during kernel panic */
static int __mhi_download_rddm_in_panic(struct mhi_controller *mhi_cntrl)
{
	int ret;
	u32 rx_status;
	enum mhi_ee ee;
	const u32 delayus = 2000;
	u32 retry = (mhi_cntrl->timeout_ms * 1000) / delayus;
	const u32 rddm_timeout_us = 200000;
	int rddm_retry = rddm_timeout_us / delayus; /* time to enter rddm */
	void __iomem *base = mhi_cntrl->bhie;

	MHI_LOG("Entered with pm_state:%s dev_state:%s ee:%s\n",
		to_mhi_pm_state_str(mhi_cntrl->pm_state),
		TO_MHI_STATE_STR(mhi_cntrl->dev_state),
		TO_MHI_EXEC_STR(mhi_cntrl->ee));

	/*
	 * This should only be executing during a kernel panic, we expect all
	 * other cores to shutdown while we're collecting rddm buffer. After
	 * returning from this function, we expect device to reset.
	 *
	 * Normaly, we would read/write pm_state only after grabbing
	 * pm_lock, since we're in a panic, skipping it. Also there is no
	 * gurantee this state change would take effect since
	 * we're setting it w/o grabbing pmlock, it's best effort
	 */
	mhi_cntrl->pm_state = MHI_PM_LD_ERR_FATAL_DETECT;
	/* update should take the effect immediately */
	smp_wmb();

	/*
	 * Make sure device is not already in RDDM.
	 * In case device asserts and a kernel panic follows, device will
	 * already be in RDDM. Do not trigger SYS ERR again and proceed with
	 * waiting for image download completion.
	 */
	ee = mhi_get_exec_env(mhi_cntrl);
	if (ee != MHI_EE_RDDM) {

		MHI_LOG("Trigger device into RDDM mode using SYSERR\n");
		mhi_set_mhi_state(mhi_cntrl, MHI_STATE_SYS_ERR);

		MHI_LOG("Waiting for device to enter RDDM\n");
		while (rddm_retry--) {
			ee = mhi_get_exec_env(mhi_cntrl);
			if (ee == MHI_EE_RDDM)
				break;

			udelay(delayus);
		}

		if (rddm_retry <= 0) {
			/* Hardware reset; force device to enter rddm */
			MHI_LOG(
				"Did not enter RDDM, do a host req. reset\n");
			mhi_cntrl->write_reg(mhi_cntrl, mhi_cntrl->regs,
				      MHI_SOC_RESET_REQ_OFFSET,
				      MHI_SOC_RESET_REQ);
			udelay(delayus);
		}

		ee = mhi_get_exec_env(mhi_cntrl);
	}

	MHI_LOG("Waiting for image download completion, current EE:%s\n",
		TO_MHI_EXEC_STR(ee));
	while (retry--) {
		ret = mhi_read_reg_field(mhi_cntrl, base, BHIE_RXVECSTATUS_OFFS,
					 BHIE_RXVECSTATUS_STATUS_BMSK,
					 BHIE_RXVECSTATUS_STATUS_SHFT,
					 &rx_status);
		if (ret)
			return -EIO;

		if (rx_status == BHIE_RXVECSTATUS_STATUS_XFER_COMPL) {
			MHI_LOG("RDDM successfully collected\n");
			return 0;
		}

		udelay(delayus);
	}

	ee = mhi_get_exec_env(mhi_cntrl);
	ret = mhi_read_reg(mhi_cntrl, base, BHIE_RXVECSTATUS_OFFS, &rx_status);

	MHI_ERR("Did not complete RDDM transfer\n");
	MHI_ERR("Current EE:%s\n", TO_MHI_EXEC_STR(ee));
	MHI_ERR("RXVEC_STATUS:0x%x, ret:%d\n", rx_status, ret);

	return -EIO;
}

/* download ramdump image from device */
int mhi_download_rddm_img(struct mhi_controller *mhi_cntrl, bool in_panic)
{
	void __iomem *base = mhi_cntrl->bhie;
	u32 rx_status;

	if (in_panic)
		return __mhi_download_rddm_in_panic(mhi_cntrl);

	MHI_LOG("Waiting for image download completion\n");

	/* waiting for image download completion */
	wait_event_timeout(mhi_cntrl->state_event,
			   mhi_read_reg_field(mhi_cntrl, base,
					      BHIE_RXVECSTATUS_OFFS,
					      BHIE_RXVECSTATUS_STATUS_BMSK,
					      BHIE_RXVECSTATUS_STATUS_SHFT,
					      &rx_status) || rx_status,
			   msecs_to_jiffies(mhi_cntrl->timeout_ms));

	return (rx_status == BHIE_RXVECSTATUS_STATUS_XFER_COMPL) ? 0 : -EIO;
}
EXPORT_SYMBOL(mhi_download_rddm_img);

static int mhi_fw_load_amss(struct mhi_controller *mhi_cntrl,
			    const struct mhi_buf *mhi_buf)
{
	void __iomem *base = mhi_cntrl->bhie;
	rwlock_t *pm_lock = &mhi_cntrl->pm_lock;
	u32 tx_status;

	read_lock_bh(pm_lock);
	if (!MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state)) {
		read_unlock_bh(pm_lock);
		return -EIO;
	}

	MHI_LOG("Starting BHIe Programming\n");

	mhi_cntrl->write_reg(mhi_cntrl, base, BHIE_TXVECADDR_HIGH_OFFS,
			upper_32_bits(mhi_buf->dma_addr));

	mhi_cntrl->write_reg(mhi_cntrl, base, BHIE_TXVECADDR_LOW_OFFS,
			lower_32_bits(mhi_buf->dma_addr));

	mhi_cntrl->write_reg(mhi_cntrl, base, BHIE_TXVECSIZE_OFFS,
			mhi_buf->len);

	mhi_cntrl->sequence_id = prandom_u32() & BHIE_TXVECSTATUS_SEQNUM_BMSK;
	mhi_write_reg_field(mhi_cntrl, base, BHIE_TXVECDB_OFFS,
			    BHIE_TXVECDB_SEQNUM_BMSK, BHIE_TXVECDB_SEQNUM_SHFT,
			    mhi_cntrl->sequence_id);
	read_unlock_bh(pm_lock);

	MHI_LOG("Upper:0x%x Lower:0x%x len:0x%lx sequence:%u\n",
		upper_32_bits(mhi_buf->dma_addr),
		lower_32_bits(mhi_buf->dma_addr),
		mhi_buf->len, mhi_cntrl->sequence_id);
	MHI_LOG("Waiting for image transfer completion\n");

	/* waiting for image download completion */
	wait_event_timeout(mhi_cntrl->state_event,
			   MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state) ||
			   mhi_read_reg_field(mhi_cntrl, base,
					      BHIE_TXVECSTATUS_OFFS,
					      BHIE_TXVECSTATUS_STATUS_BMSK,
					      BHIE_TXVECSTATUS_STATUS_SHFT,
					      &tx_status) || tx_status,
			   msecs_to_jiffies(mhi_cntrl->timeout_ms));

	if (MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state))
		return -EIO;

	return (tx_status == BHIE_TXVECSTATUS_STATUS_XFER_COMPL) ? 0 : -EIO;
}

static int mhi_fw_load_sbl(struct mhi_controller *mhi_cntrl,
			   dma_addr_t dma_addr,
			   size_t size)
{
	u32 tx_status, val;
	int i, ret;
	void __iomem *base = mhi_cntrl->bhi;
	rwlock_t *pm_lock = &mhi_cntrl->pm_lock;
	struct {
		char *name;
		u32 offset;
	} error_reg[] = {
		{ "ERROR_CODE", BHI_ERRCODE },
		{ "ERROR_DBG1", BHI_ERRDBG1 },
		{ "ERROR_DBG2", BHI_ERRDBG2 },
		{ "ERROR_DBG3", BHI_ERRDBG3 },
		{ NULL },
	};

	MHI_LOG("Starting BHI programming\n");

	/* program start sbl download via  bhi protocol */
	read_lock_bh(pm_lock);
	if (!MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state)) {
		read_unlock_bh(pm_lock);
		goto invalid_pm_state;
	}

	mhi_cntrl->write_reg(mhi_cntrl, base, BHI_STATUS, 0);
	mhi_cntrl->write_reg(mhi_cntrl, base, BHI_IMGADDR_HIGH,
		      upper_32_bits(dma_addr));
	mhi_cntrl->write_reg(mhi_cntrl, base, BHI_IMGADDR_LOW,
		      lower_32_bits(dma_addr));
	mhi_cntrl->write_reg(mhi_cntrl, base, BHI_IMGSIZE, size);
	mhi_cntrl->session_id = prandom_u32() & BHI_TXDB_SEQNUM_BMSK;
	mhi_cntrl->write_reg(mhi_cntrl, base, BHI_IMGTXDB,
			mhi_cntrl->session_id);
	read_unlock_bh(pm_lock);

	MHI_LOG("Waiting for image transfer completion\n");

	/* waiting for image download completion */
	wait_event_timeout(mhi_cntrl->state_event,
			   MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state) ||
			   mhi_read_reg_field(mhi_cntrl, base, BHI_STATUS,
					      BHI_STATUS_MASK, BHI_STATUS_SHIFT,
					      &tx_status) || tx_status,
			   msecs_to_jiffies(mhi_cntrl->timeout_ms));
	if (MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state))
		goto invalid_pm_state;

	if (tx_status == BHI_STATUS_ERROR) {
		MHI_ERR("Image transfer failed\n");
		read_lock_bh(pm_lock);
		if (MHI_REG_ACCESS_VALID(mhi_cntrl->pm_state)) {
			for (i = 0; error_reg[i].name; i++) {
				ret = mhi_read_reg(mhi_cntrl, base,
						   error_reg[i].offset, &val);
				if (ret)
					break;
				MHI_ERR("reg:%s value:0x%x\n",
					error_reg[i].name, val);
			}
		}
		read_unlock_bh(pm_lock);
		goto invalid_pm_state;
	}

	return (tx_status == BHI_STATUS_SUCCESS) ? 0 : -ETIMEDOUT;

invalid_pm_state:

	return -EIO;
}

void mhi_free_bhie_table(struct mhi_controller *mhi_cntrl,
			 struct image_info *image_info)
{
	int i;
	struct mhi_buf *mhi_buf = image_info->mhi_buf;

	for (i = 0; i < image_info->entries; i++, mhi_buf++)
		mhi_free_coherent(mhi_cntrl, mhi_buf->len, mhi_buf->buf,
				  mhi_buf->dma_addr);

	kfree(image_info->mhi_buf);
	kfree(image_info);
}

int mhi_alloc_bhie_table(struct mhi_controller *mhi_cntrl,
			 struct image_info **image_info,
			 size_t alloc_size)
{
	size_t seg_size = mhi_cntrl->seg_len;
	/* requier additional entry for vec table */
	int segments = DIV_ROUND_UP(alloc_size, seg_size) + 1;
	int i;
	struct image_info *img_info;
	struct mhi_buf *mhi_buf;

	MHI_LOG("Allocating bytes:%zu seg_size:%zu total_seg:%u\n",
		alloc_size, seg_size, segments);

	img_info = kzalloc(sizeof(*img_info), GFP_KERNEL);
	if (!img_info)
		return -ENOMEM;

	/* allocate memory for entries */
	img_info->mhi_buf = kcalloc(segments, sizeof(*img_info->mhi_buf),
				    GFP_KERNEL);
	if (!img_info->mhi_buf)
		goto error_alloc_mhi_buf;

	/* allocate and populate vector table */
	mhi_buf = img_info->mhi_buf;
	for (i = 0; i < segments; i++, mhi_buf++) {
		size_t vec_size = seg_size;

		/* last entry is for vector table */
		if (i == segments - 1)
			vec_size = sizeof(struct bhi_vec_entry) * i;

		mhi_buf->len = vec_size;
		mhi_buf->buf = mhi_alloc_coherent(mhi_cntrl, vec_size,
					&mhi_buf->dma_addr, GFP_KERNEL);
		if (!mhi_buf->buf)
			goto error_alloc_segment;

		MHI_LOG("Entry:%d Address:0x%llx size:%lu\n", i,
			mhi_buf->dma_addr, mhi_buf->len);
	}

	img_info->bhi_vec = img_info->mhi_buf[segments - 1].buf;
	img_info->entries = segments;
	*image_info = img_info;

	MHI_LOG("Successfully allocated bhi vec table\n");

	return 0;

error_alloc_segment:
	for (--i, --mhi_buf; i >= 0; i--, mhi_buf--)
		mhi_free_coherent(mhi_cntrl, mhi_buf->len, mhi_buf->buf,
				  mhi_buf->dma_addr);

error_alloc_mhi_buf:
	kfree(img_info);

	return -ENOMEM;
}

static void mhi_firmware_copy(struct mhi_controller *mhi_cntrl,
			      const struct firmware *firmware,
			      struct image_info *img_info)
{
	size_t remainder = firmware->size;
	size_t to_cpy;
	const u8 *buf = firmware->data;
	int i = 0;
	struct mhi_buf *mhi_buf = img_info->mhi_buf;
	struct bhi_vec_entry *bhi_vec = img_info->bhi_vec;

	while (remainder) {
		MHI_ASSERT(i >= img_info->entries, "malformed vector table");

		to_cpy = min(remainder, mhi_buf->len);
		memcpy(mhi_buf->buf, buf, to_cpy);
		bhi_vec->dma_addr = mhi_buf->dma_addr;
		bhi_vec->size = to_cpy;

		MHI_VERB("Setting Vector:0x%llx size: %llu\n",
			 bhi_vec->dma_addr, bhi_vec->size);
		buf += to_cpy;
		remainder -= to_cpy;
		i++;
		bhi_vec++;
		mhi_buf++;
	}
}

void mhi_fw_load_worker(struct work_struct *work)
{
	int ret;
	struct mhi_controller *mhi_cntrl;
	const char *fw_name;
	const struct firmware *firmware;
	struct image_info *image_info;
	void *buf;
	dma_addr_t dma_addr;
	size_t size;

	mhi_cntrl = container_of(work, struct mhi_controller, fw_worker);

	MHI_LOG("Waiting for device to enter PBL from EE:%s\n",
		TO_MHI_EXEC_STR(mhi_cntrl->ee));

	ret = wait_event_timeout(mhi_cntrl->state_event,
				 MHI_IN_PBL(mhi_cntrl->ee) ||
				 MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state),
				 msecs_to_jiffies(mhi_cntrl->timeout_ms));

	if (!ret || MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state)) {
		MHI_ERR("MHI is not in valid state\n");
		return;
	}

	MHI_LOG("Device current EE:%s\n", TO_MHI_EXEC_STR(mhi_cntrl->ee));

	/* if device in pthru, do reset to ready state transition */
	if (mhi_cntrl->ee == MHI_EE_PTHRU)
		goto fw_load_ee_pthru;

	fw_name = (mhi_cntrl->ee == MHI_EE_EDL) ?
		mhi_cntrl->edl_image : mhi_cntrl->fw_image;

	if (!fw_name || (mhi_cntrl->fbc_download && (!mhi_cntrl->sbl_size ||
						     !mhi_cntrl->seg_len))) {
		MHI_ERR("No firmware image defined or !sbl_size || !seg_len\n");
		return;
	}

	ret = request_firmware(&firmware, fw_name, mhi_cntrl->dev);
	if (ret) {
		MHI_ERR("Error loading firmware, ret:%d\n", ret);
		return;
	}

	size = (mhi_cntrl->fbc_download) ? mhi_cntrl->sbl_size : firmware->size;

	/* the sbl size provided is maximum size, not necessarily image size */
	if (size > firmware->size)
		size = firmware->size;

	buf = mhi_alloc_coherent(mhi_cntrl, size, &dma_addr, GFP_KERNEL);
	if (!buf) {
		MHI_ERR("Could not allocate memory for image\n");
		release_firmware(firmware);
		return;
	}

	/* load sbl image */
	memcpy(buf, firmware->data, size);
	ret = mhi_fw_load_sbl(mhi_cntrl, dma_addr, size);
	mhi_free_coherent(mhi_cntrl, size, buf, dma_addr);

	if (!mhi_cntrl->fbc_download || ret || mhi_cntrl->ee == MHI_EE_EDL)
		release_firmware(firmware);

	/* error or in edl, we're done */
	if (ret || mhi_cntrl->ee == MHI_EE_EDL)
		return;

	write_lock_irq(&mhi_cntrl->pm_lock);
	mhi_cntrl->dev_state = MHI_STATE_RESET;
	write_unlock_irq(&mhi_cntrl->pm_lock);

	/*
	 * if we're doing fbc, populate vector tables while
	 * device transitioning into MHI READY state
	 */
	if (mhi_cntrl->fbc_download) {
		ret = mhi_alloc_bhie_table(mhi_cntrl, &mhi_cntrl->fbc_image,
					   firmware->size);
		if (ret) {
			MHI_ERR("Error alloc size of %zu\n", firmware->size);
			goto error_alloc_fw_table;
		}

		MHI_LOG("Copying firmware image into vector table\n");

		/* load the firmware into BHIE vec table */
		mhi_firmware_copy(mhi_cntrl, firmware, mhi_cntrl->fbc_image);
	}

fw_load_ee_pthru:
	/* transitioning into MHI RESET->READY state */
	ret = mhi_ready_state_transition(mhi_cntrl);

	MHI_LOG("To Reset->Ready PM_STATE:%s MHI_STATE:%s EE:%s, ret:%d\n",
		to_mhi_pm_state_str(mhi_cntrl->pm_state),
		TO_MHI_STATE_STR(mhi_cntrl->dev_state),
		TO_MHI_EXEC_STR(mhi_cntrl->ee), ret);

	if (!mhi_cntrl->fbc_download)
		return;

	if (ret) {
		MHI_ERR("Did not transition to READY state\n");
		goto error_read;
	}

	/* wait for SBL event */
	ret = wait_event_timeout(mhi_cntrl->state_event,
				 mhi_cntrl->ee == MHI_EE_SBL ||
				 MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state),
				 msecs_to_jiffies(mhi_cntrl->timeout_ms));

	if (!ret || MHI_PM_IN_ERROR_STATE(mhi_cntrl->pm_state)) {
		MHI_ERR("MHI did not enter BHIE\n");
		goto error_read;
	}

	/* start full firmware image download */
	image_info = mhi_cntrl->fbc_image;
	ret = mhi_fw_load_amss(mhi_cntrl,
			       /* last entry is vec table */
			       &image_info->mhi_buf[image_info->entries - 1]);

	MHI_LOG("amss fw_load, ret:%d\n", ret);

	release_firmware(firmware);

	return;

error_read:
	mhi_free_bhie_table(mhi_cntrl, mhi_cntrl->fbc_image);
	mhi_cntrl->fbc_image = NULL;

error_alloc_fw_table:
	release_firmware(firmware);
}
