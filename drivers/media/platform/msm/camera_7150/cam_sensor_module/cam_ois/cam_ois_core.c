/* Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/firmware.h>
#include <cam_sensor_cmn_header.h>
#include "cam_ois_core.h"
#include "cam_ois_soc.h"
#include "cam_sensor_util.h"
#include "cam_debug_util.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"

#ifdef VENDOR_EDIT
#define MODE_NOCONTINUE 1
#define MODE_CONTINUE 0
//#define MAX_LENGTH 128
#define MAX_LENGTH_MAIN 160
struct cam_sensor_i2c_reg_setting_array {
	struct cam_sensor_i2c_reg_array reg_setting[512];
	unsigned short size;
	enum camera_sensor_i2c_type addr_type;
	enum camera_sensor_i2c_type data_type;
	unsigned short delay;
};
//0xBF03 is used for clock 24Mhz \ 0xFF02 is for clock 19.2Mhz
struct cam_sensor_i2c_reg_setting_array bu63169_pll_settings = {
    .reg_setting =
	{
		{.reg_addr = 0x8262, .reg_data = 0xBF03, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x8263, .reg_data = 0x9F05, .delay = 0x01, .data_mask = 0x00}, \
		{.reg_addr = 0x8264, .reg_data = 0x6040, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x8260, .reg_data = 0x1130, .delay = 0x00, .data_mask = 0X00}, \
		{.reg_addr = 0x8265, .reg_data = 0x8000, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x8261, .reg_data = 0x0280, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x8261, .reg_data = 0x0380, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x8261, .reg_data = 0x0988, .delay = 0x00, .data_mask = 0X00}, \
	},
    .size = 8,
    .addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
    .data_type = CAMERA_SENSOR_I2C_TYPE_WORD,
    .delay = 1,
};

struct cam_sensor_i2c_reg_setting_array bu63169_gyroOn_settings = {
	.reg_setting =
	{
		{.reg_addr = 0x8218, .reg_data = 0x0F00, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x821B, .reg_data = 0x230B, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x821C, .reg_data = 0x230B, .delay = 0x01, .data_mask = 0x00}, \
		{.reg_addr = 0x821B, .reg_data = 0x081F, .delay = 0x00, .data_mask = 0X00}, \
		{.reg_addr = 0x821C, .reg_data = 0x081F, .delay = 0x01, .data_mask = 0x00}, \
		{.reg_addr = 0x821B, .reg_data = 0x800C, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x821C, .reg_data = 0x800C, .delay = 0x01, .data_mask = 0x00}, \
		{.reg_addr = 0x821B, .reg_data = 0x000D, .delay = 0x00, .data_mask = 0X00}, \
		{.reg_addr = 0x821C, .reg_data = 0x000D, .delay = 0x01, .data_mask = 0X00}, \
		{.reg_addr = 0x8218, .reg_data = 0x2700, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x847F, .reg_data = 0x0D0D, .delay = 0x00, .data_mask = 0x00}, \
	},
	.size = 11,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
	.data_type = CAMERA_SENSOR_I2C_TYPE_WORD,
	.delay = 1,
};

struct cam_sensor_i2c_reg_setting_array bu63169_gyroOff_settings = {
	.reg_setting =
	{
		{.reg_addr = 0x847F, .reg_data = 0x8D0D, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x8218, .reg_data = 0x0F00, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x821B, .reg_data = 0x000B, .delay = 0x00, .data_mask = 0x00}, \
		{.reg_addr = 0x821C, .reg_data = 0x000B, .delay = 0x00, .data_mask = 0x00}, \
	},
	.size = 4,
	.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
	.data_type = CAMERA_SENSOR_I2C_TYPE_WORD,
	.delay = 1,
};

static int RamWriteByte(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t addr, uint32_t data, unsigned short mdelay)
{
	int32_t rc = 0;
	int retry = 3;
        int i;
	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = mdelay,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.delay = mdelay,
	};
	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	for( i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "write 0x%04x failed, retry:%d", addr, i+1);
		} else {
			return rc;
		}
	}
	return rc;
}

static int RamWriteWord(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t addr, uint32_t data)
{
	int32_t rc = 0;
	int retry = 3;
        int i; 
	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = 0x00,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.data_type = CAMERA_SENSOR_I2C_TYPE_WORD,
		.delay = 0x00,
	};
	if (addr == 0x8c) {
		i2c_write .addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
		i2c_write .data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	}
	if (o_ctrl == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	for( i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write(&(o_ctrl->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "write 0x%04x failed, retry:%d", addr, i+1);
		} else {
			return rc;
		}
	}
	return rc;
}

static int RamMultiWrite(struct cam_ois_ctrl_t *o_ctrl,
	struct cam_sensor_i2c_reg_setting *write_setting) {
	int rc = 0;
        int i;
	for (i = 0; i < write_setting->size; i++) {
		rc = RamWriteWord(o_ctrl, write_setting->reg_setting[i].reg_addr,
			write_setting->reg_setting[i].reg_data);
	}
	return rc;
}
#endif


int32_t cam_ois_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 1;
	power_info->power_setting =
		(struct cam_sensor_power_setting *)
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VAF;
	power_info->power_setting[0].seq_val = CAM_VAF;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 2;

	power_info->power_down_setting_size = 1;
	power_info->power_down_setting =
		(struct cam_sensor_power_setting *)
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}


/**
 * cam_ois_get_dev_handle - get device handle
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_get_dev_handle(struct cam_ois_ctrl_t *o_ctrl,
	void *arg)
{
	struct cam_sensor_acquire_dev    ois_acq_dev;
	struct cam_create_dev_hdl        bridge_params;
	struct cam_control              *cmd = (struct cam_control *)arg;

	if (o_ctrl->bridge_intf.device_hdl != -1) {
		CAM_ERR(CAM_OIS, "Device is already acquired");
		return -EFAULT;
	}
	if (copy_from_user(&ois_acq_dev, u64_to_user_ptr(cmd->handle),
		sizeof(ois_acq_dev)))
		return -EFAULT;

	bridge_params.session_hdl = ois_acq_dev.session_handle;
	bridge_params.ops = &o_ctrl->bridge_intf.ops;
	bridge_params.v4l2_sub_dev_flag = 0;
	bridge_params.media_entity_flag = 0;
	bridge_params.priv = o_ctrl;
	bridge_params.dev_id = CAM_OIS;
	ois_acq_dev.device_handle =
		cam_create_device_hdl(&bridge_params);
	o_ctrl->bridge_intf.device_hdl = ois_acq_dev.device_handle;
	o_ctrl->bridge_intf.session_hdl = ois_acq_dev.session_handle;

	CAM_DBG(CAM_OIS, "Device Handle: %d", ois_acq_dev.device_handle);
	if (copy_to_user(u64_to_user_ptr(cmd->handle), &ois_acq_dev,
		sizeof(struct cam_sensor_acquire_dev))) {
		CAM_ERR(CAM_OIS, "ACQUIRE_DEV: copy to user failed");
		return -EFAULT;
	}
	return 0;
}

static int cam_ois_power_up(struct cam_ois_ctrl_t *o_ctrl)
{
	int                             rc = 0;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t  *power_info;

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_OIS,
			"Using default power settings");
		rc = cam_ois_construct_default_power_setting(power_info);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Construct default ois power setting failed.");
			return rc;
		}
	}

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"failed to fill vreg params for power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "failed in ois power up rc %d", rc);
		return rc;
	}

	rc = camera_io_init(&o_ctrl->io_master_info);
	if (rc)
		CAM_ERR(CAM_OIS, "cci_init failed: rc: %d", rc);

	return rc;
}

/**
 * cam_ois_power_down - power down OIS device
 * @o_ctrl:     ctrl structure
 *
 * Returns success or failure
 */
static int cam_ois_power_down(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t                         rc = 0;
	struct cam_sensor_power_ctrl_t  *power_info;
	struct cam_hw_soc_info          *soc_info =
		&o_ctrl->soc_info;
	struct cam_ois_soc_private *soc_private;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "failed: o_ctrl %pK", o_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &o_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_OIS, "failed: power_info %pK", power_info);
		return -EINVAL;
	}

	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_OIS, "power down the core is failed:%d", rc);
		return rc;
	}

	camera_io_release(&o_ctrl->io_master_info);

	return rc;
}

static int cam_ois_apply_settings(struct cam_ois_ctrl_t *o_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;
	uint32_t i, size;
	#ifdef VENDOR_EDIT
	int mode = MODE_CONTINUE;
	#endif
	if (o_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_OIS, " Invalid settings");
		return -EINVAL;
	}

	#ifdef VENDOR_EDIT
	if (MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master) {
		mode = MODE_NOCONTINUE;
	}
	#endif

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		if (i2c_list->op_code ==  CAM_SENSOR_I2C_WRITE_RANDOM) {
			#ifdef VENDOR_EDIT
			if (mode == MODE_CONTINUE) {
				rc = camera_io_dev_write(&(o_ctrl->io_master_info),
					&(i2c_list->i2c_settings));


			} else {
				rc = RamMultiWrite(o_ctrl, &(i2c_list->i2c_settings));
			}
			#else
			rc = camera_io_dev_write(&(o_ctrl->io_master_info),
				&(i2c_list->i2c_settings));
			#endif
			if (rc < 0) {
				CAM_ERR(CAM_OIS,
					"Failed in Applying i2c wrt settings");
				return rc;
			}
		} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
			size = i2c_list->i2c_settings.size;
			for (i = 0; i < size; i++) {
				rc = camera_io_dev_poll(
				&(o_ctrl->io_master_info),
				i2c_list->i2c_settings.reg_setting[i].reg_addr,
				i2c_list->i2c_settings.reg_setting[i].reg_data,
				i2c_list->i2c_settings.reg_setting[i].data_mask,
				i2c_list->i2c_settings.addr_type,
				i2c_list->i2c_settings.data_type,
				i2c_list->i2c_settings.reg_setting[i].delay);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"i2c poll apply setting Fail");
					return rc;
				}
			}
		}
	}

	return rc;
}

static int cam_ois_slaveInfo_pkt_parser(struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_ois_info *ois_info;

	if (!o_ctrl || !cmd_buf || len < sizeof(struct cam_cmd_ois_info)) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	ois_info = (struct cam_cmd_ois_info *)cmd_buf;
	if (o_ctrl->io_master_info.master_type == CCI_MASTER) {
		o_ctrl->io_master_info.cci_client->i2c_freq_mode =
			ois_info->i2c_freq_mode;
		o_ctrl->io_master_info.cci_client->sid =
			ois_info->slave_addr >> 1;
		o_ctrl->ois_fw_flag = ois_info->ois_fw_flag;
		o_ctrl->is_ois_calib = ois_info->is_ois_calib;
		memcpy(o_ctrl->ois_name, ois_info->ois_name, OIS_NAME_LEN);
		o_ctrl->ois_name[OIS_NAME_LEN - 1] = '\0';
		o_ctrl->io_master_info.cci_client->retries = 3;
		o_ctrl->io_master_info.cci_client->id_map = 0;
		memcpy(&(o_ctrl->opcode), &(ois_info->opcode),
			sizeof(struct cam_ois_opcode));
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x Freq Mode: %d",
			ois_info->slave_addr, ois_info->i2c_freq_mode);
	} else if (o_ctrl->io_master_info.master_type == I2C_MASTER) {
		o_ctrl->io_master_info.client->addr = ois_info->slave_addr;
		CAM_DBG(CAM_OIS, "Slave addr: 0x%x", ois_info->slave_addr);
	} else {
		CAM_ERR(CAM_OIS, "Invalid Master type : %d",
			o_ctrl->io_master_info.master_type);
		rc = -EINVAL;
	}

	return rc;
}

#ifndef VENDOR_EDIT
#define PRJ_VERSION_PATH  "/proc/oppoVersion/prjVersion"
#define PCB_VERSION_PATH  "/proc/oppoVersion/pcbVersion"
static int getfileData(char *filename, char *context)
{
	struct file *mfile = NULL;
	ssize_t size = 0;
	loff_t offsize = 0;
	mm_segment_t old_fs;
	char project[10] = {0};

	memset(project, 0, sizeof(project));
	mfile = filp_open(filename, O_RDONLY, 0644);
	if (IS_ERR(mfile))
	{
		CAM_ERR(CAM_OIS, "%s fopen file %s failed !", __func__, filename);
		return (-1);
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	offsize = 0;
	size = vfs_read(mfile, project, sizeof(project), &offsize);
	if (size < 0) {
		CAM_ERR(CAM_OIS, "fread file %s error size:%s", __func__, filename);
		set_fs(old_fs);
		filp_close(mfile, NULL);
		return (-1);
	}
	set_fs(old_fs);
	filp_close(mfile, NULL);

	CAM_ERR(CAM_OIS, "%s project:%s", __func__, project);
	memcpy(context, project, size);
	return 0;
}

static int getProject(char *project)
{
	int rc = 0;
	rc = getfileData(PRJ_VERSION_PATH, project);
	return rc;
}

static int getPcbVersion(char *pcbVersion)
{
	int rc = 0;
	rc = getfileData(PCB_VERSION_PATH, pcbVersion);
	return rc;
}

#define VERSION_OFFSET              0x7FF4
#define FW_UPDATE_VERSION           0x300D
#define FW_UPDATE_MIDDLE_VERSION    0x000C
#define FW_DVT4G_UPDATE_VERSION     0x600B
#define FW_DVT5G_UPDATE_VERSION     0x610B

static int cam_sem1815s_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                            *ptr = NULL;
	int32_t                            rc = 0, cnt;
	uint32_t                           fw_size, data;
	uint16_t                           check_sum = 0x0;
	const struct firmware              *fw = NULL;
	const char                         *fw_name_bin = NULL;
	char                               name_bin[32] = {0};
	struct device                      *dev = &(o_ctrl->pdev->dev);
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                        *page = NULL;
	int                                reg_double = 1;
	int 				   i;
	char                               mPrjname[10];
	char                               mPcbVer[4];
	uint8_t                            fw_ver[4] = {0};
	uint32_t                           ois_fw_version = FW_UPDATE_VERSION;
	uint32_t                           ois_middle_version = FW_UPDATE_MIDDLE_VERSION;
	uint32_t                           ois_dvt_version = FW_DVT4G_UPDATE_VERSION;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	CAM_ERR(CAM_OIS, "entry:%s ", __func__);

	memset(mPrjname, 0, sizeof(mPrjname));
	memset(mPcbVer, 0, sizeof(mPcbVer));

	rc = getProject(mPrjname);
	rc |= getPcbVersion(mPcbVer);

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x1008, &data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x1008 fail");
		return 0;
	}
	data = ((data >> 8) & 0x0FF) | ((data & 0xFF) << 8);
	ois_middle_version = ((ois_middle_version >> 8) & 0x0FF) | ((ois_middle_version & 0xFF) << 8);
	ois_dvt_version = ((ois_dvt_version >> 8) & 0x0FF) | ((ois_dvt_version & 0xFF) << 8);
	ois_fw_version = ((ois_fw_version >> 8) & 0x0FF) | ((ois_fw_version & 0xFF) << 8);

	if (data >= ois_fw_version) {
		CAM_ERR(CAM_OIS, "project is:%s mPcbVer:%s fw version:0x%0x ois_fw_version:0x%0x no need to update !!!", mPrjname, mPcbVer, data, ois_fw_version);
		return 0;
	} else {
		if (data >= ois_middle_version) //PVT module need update to 0x000D
		{
			snprintf(name_bin, 32, "%s.bin", o_ctrl->ois_name);
		}
		else if (data < ois_dvt_version) //DVT module need to update 0x600B(0x610B)
		{
			if ((strcmp(mPrjname, "18503") == 0 || strcmp(mPrjname, "18501") == 0)
				&& (strcmp(mPcbVer, "4") == 0 || strcmp(mPcbVer, "3") == 0)){
				snprintf(name_bin, 32, "%s_dvt5g.bin", o_ctrl->ois_name);
			} else {
				snprintf(name_bin, 32, "%s_dvt4g.bin", o_ctrl->ois_name);
			}
		}
		else
		{
			CAM_ERR(CAM_OIS, "project is:%s mPcbVer:%s fw version:0x%0x ois_fw_version:0x%0x no need to update !!!", mPrjname, mPcbVer, data, ois_fw_version);
			return 0;
		}
		CAM_ERR(CAM_OIS, "project is:%s mPcbVer:%s fw version:0x%0x need to update ois_fw_version:0x%0x !!!", mPrjname, mPcbVer, data, ois_fw_version);
	}
	/* cast pointer as const pointer*/
	fw_name_bin = name_bin;

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x0001, &data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x0001 fail");
	}
	if (data != 0x01) {
		RamWriteByte(o_ctrl, 0x0000, 0x0, 50);
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x0201, &data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x0201 fail");
	}
	if (data != 0x01) {
		RamWriteByte(o_ctrl, 0x0200, 0x0, 10);
		rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x0201, &data,
			CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "read 0x0201 fail");
		}
	}

	RamWriteByte(o_ctrl, 0x1000, 0x05, 60);

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x0001, &data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x0001 fail");
	}
	if (data != 0x02) {
		return 0;
	}

	/* Load FW */
	rc = request_firmware(&fw, fw_name_bin, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_bin);
		return 0;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		page_address(page));

	CAM_DBG(CAM_OIS, "total_bytes:%d", total_bytes);

	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;) {
		i2c_reg_setting.size = 0;
		for (i = 0; (i < MAX_LENGTH && cnt < total_bytes); i++,ptr++) {
			if (cnt >= VERSION_OFFSET && cnt < (VERSION_OFFSET + 4)) {
				fw_ver[cnt-VERSION_OFFSET] = *ptr;
				CAM_ERR(CAM_OIS, "get fw version:0x%0x", fw_ver[cnt-VERSION_OFFSET]);
			}
			i2c_reg_setting.reg_setting[i].reg_addr =
				o_ctrl->opcode.prog;
			i2c_reg_setting.reg_setting[i].reg_data = *ptr;
			i2c_reg_setting.reg_setting[i].delay = 0;
			i2c_reg_setting.reg_setting[i].data_mask = 0;
			i2c_reg_setting.size++;
			cnt++;
			if (reg_double == 0) {
				reg_double = 1;
			} else {
				check_sum += ((*(ptr+1) << 8) | *ptr) & 0xFFFF;
				reg_double = 0;
			}
		}
		i2c_reg_setting.delay = 0;

		if (i2c_reg_setting.size > 0) {
			rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
				&i2c_reg_setting, 1);
		    msleep(1);
		}
	}
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
		goto release_firmware;
	}
	CAM_ERR(CAM_OIS, "check sum:0x%0x", check_sum);

	RamWriteWord(o_ctrl, 0x1002, ((check_sum&0x0FF) << 8) | ((check_sum&0xFF00) >> 8));
	msleep(10);

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x1001, &data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x1001 fail");
	} else {
		CAM_ERR(CAM_OIS, "get 0x1001 = 0x%0x", data);
	}

	RamWriteByte(o_ctrl, 0x1000, 0x80, 200);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "write 0x1000 fail");
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x1008, &data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x1008 fail");
	}
	CAM_ERR(CAM_OIS, "get 0x1008 = 0x%0x", data);

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x100A, &data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x100A fail");
	}
	CAM_ERR(CAM_OIS, "get 0x100A = 0x%0x", data);

release_firmware:
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, fw_size);
	release_firmware(fw);

	return rc;
}
#endif

static int cam_ois_fw_download(struct cam_ois_ctrl_t *o_ctrl)
{
	uint16_t                           total_bytes = 0;
	uint8_t                           *ptr = NULL;
	int32_t                            rc = 0, cnt;
	int 				   i;
	uint32_t                           fw_size;
	const struct firmware             *fw = NULL;
	const char                        *fw_name_prog = NULL;
	const char                        *fw_name_coeff = NULL;
	char                               name_prog[32] = {0};
	char                               name_coeff[32] = {0};
	struct device                     *dev = &(o_ctrl->pdev->dev);
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct page                       *page = NULL;
#ifdef VENDOR_EDIT
	uint32_t ee_vcmid = 0 ;
	struct cam_sensor_cci_client ee_cci_client ;
	const uint8_t IMX586_EEPROM_SID = (0xA0 >> 1);
	const uint8_t IMX586_EEPROM_VCMID_ADDR = 0x0A;
	const uint8_t IMX586_SECOND_SOURCE_VCMID = 0x3A;
#endif
	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	snprintf(name_coeff, 32, "%s.coeff", o_ctrl->ois_name);

	snprintf(name_prog, 32, "%s.prog", o_ctrl->ois_name);
#ifdef VENDOR_EDIT
	memcpy(&ee_cci_client, o_ctrl->io_master_info.cci_client,sizeof(struct cam_sensor_cci_client));
	ee_cci_client.sid = IMX586_EEPROM_SID;
	rc = cam_cci_i2c_read(&ee_cci_client,
				IMX586_EEPROM_VCMID_ADDR,
				&ee_vcmid, CAMERA_SENSOR_I2C_TYPE_WORD,
				CAMERA_SENSOR_I2C_TYPE_BYTE);
	CAM_ERR(CAM_SENSOR, "distinguish imx586 camera module ois fw, vcm id : 0x%x ",ee_vcmid);
	if (IMX586_SECOND_SOURCE_VCMID == ee_vcmid) {
		snprintf(name_coeff, 32, "%s_sec.coeff", o_ctrl->ois_name);
		snprintf(name_prog, 32, "%s_sec.prog", o_ctrl->ois_name);
	}
#endif
	/* cast pointer as const pointer*/
	fw_name_prog = name_prog;
	fw_name_coeff = name_coeff;

	/* Load FW */
	rc = request_firmware(&fw, fw_name_prog, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_prog);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		page_address(page));

	#ifndef VENDOR_EDIT
	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.prog;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, 1);
	#else
	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;) {
		i2c_reg_setting.size = 0;
                for (i = 0; (i < MAX_LENGTH_MAIN && cnt < total_bytes); i++,ptr++) {
			i2c_reg_setting.reg_setting[i].reg_addr =
				o_ctrl->opcode.prog;
			i2c_reg_setting.reg_setting[i].reg_data = *ptr;
			i2c_reg_setting.reg_setting[i].delay = 0;
			i2c_reg_setting.reg_setting[i].data_mask = 0;
			i2c_reg_setting.size++;
			cnt++;
		}
		i2c_reg_setting.delay = 0;
		if (i2c_reg_setting.size > 0) {
			rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
				&i2c_reg_setting, 1);
		}
	}
	#endif
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);
		goto release_firmware;
	}
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, fw_size);
	page = NULL;
	fw_size = 0;
	release_firmware(fw);

	rc = request_firmware(&fw, fw_name_coeff, dev);
	if (rc) {
		CAM_ERR(CAM_OIS, "Failed to locate %s", fw_name_coeff);
		return rc;
	}

	total_bytes = fw->size;
	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = total_bytes;
	i2c_reg_setting.delay = 0;
	fw_size = PAGE_ALIGN(sizeof(struct cam_sensor_i2c_reg_array) *
		total_bytes) >> PAGE_SHIFT;
	page = cma_alloc(dev_get_cma_area((o_ctrl->soc_info.dev)),
		fw_size, 0, GFP_KERNEL);
	if (!page) {
		CAM_ERR(CAM_OIS, "Failed in allocating i2c_array");
		release_firmware(fw);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		page_address(page));

	#ifndef VENDOR_EDIT
	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;
		cnt++, ptr++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr =
			o_ctrl->opcode.coeff;
		i2c_reg_setting.reg_setting[cnt].reg_data = *ptr;
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
		&i2c_reg_setting, 1);
	#else
	for (cnt = 0, ptr = (uint8_t *)fw->data; cnt < total_bytes;) {
		i2c_reg_setting.size = 0;
                for (i = 0; (i < MAX_LENGTH_MAIN && cnt < total_bytes); i++,ptr++) {
			i2c_reg_setting.reg_setting[i].reg_addr =
				o_ctrl->opcode.coeff;
			i2c_reg_setting.reg_setting[i].reg_data = *ptr;
			i2c_reg_setting.reg_setting[i].delay = 0;
			i2c_reg_setting.reg_setting[i].data_mask = 0;
			i2c_reg_setting.size++;
			cnt++;
		}
		i2c_reg_setting.delay = 0;
		rc = camera_io_dev_write_continuous(&(o_ctrl->io_master_info),
			&i2c_reg_setting, 1);
	}
	#endif
	if (rc < 0)
		CAM_ERR(CAM_OIS, "OIS FW download failed %d", rc);

release_firmware:
	cma_release(dev_get_cma_area((o_ctrl->soc_info.dev)),
		page, fw_size);
	release_firmware(fw);

	return rc;
}
#ifdef VENDOR_EDIT
static int cam_ois_read_gyrodata(struct cam_ois_ctrl_t *o_ctrl, uint32_t gyro_x_addr,
	uint32_t gyro_y_addr, uint32_t *gyro_data, bool need_convert)
{
	uint32_t gyro_offset = 0x00;
	uint32_t gyro_offset_x,  gyro_offset_y = 0x00;
	int rc = 0;

	CAM_INFO(CAM_OIS, "i2c_master:%d read gyro offset ", o_ctrl->io_master_info.cci_client->cci_i2c_master);
	rc = camera_io_dev_read(&(o_ctrl->io_master_info), gyro_x_addr, &gyro_offset_x,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "read gyro offset_x fail");
		}
	if (need_convert) {
		gyro_offset_x = (gyro_offset_x & 0xFF) << 8 | (gyro_offset_x & 0xFF00) >> 8;
	}
	rc = camera_io_dev_read(&(o_ctrl->io_master_info), gyro_y_addr, &gyro_offset_y,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "read gyro offset_y fail");
	}

	if (need_convert) {
		gyro_offset_y = (gyro_offset_y & 0xFF) << 8 | (gyro_offset_y & 0xFF00) >> 8;
	}

	gyro_offset = ((gyro_offset_y & 0xFFFF) << 16) | (gyro_offset_x & 0xFFFF);
	CAM_ERR(CAM_OIS, "final gyro_offset = 0x%x; gyro_x=0x%x, gyro_y=0x%x",
		gyro_offset, gyro_offset_x, gyro_offset_y);
	*gyro_data = gyro_offset;
	return rc;
}
static int cam_ois_sem1215s_calibration(struct cam_ois_ctrl_t *o_ctrl)
{
	int32_t                            rc = 0;
	uint32_t                           data;
	uint32_t                           calib_data = 0x0;
	int                                calib_ret = 0;
	int				   i;
	uint32_t                           gyro_offset = 0;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x0001, &data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x0001 fail");
	}
	if (data != 0x01) {
		RamWriteByte(o_ctrl, 0x0000, 0x0, 50);
    }

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x0201, &data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x0201 fail");
	}
	if (data != 0x01) {
		RamWriteByte(o_ctrl, 0x0200, 0x0, 10);
	}

	RamWriteByte(o_ctrl, 0x0600, 0x1, 100);
	for (i = 0; i < 5; i++) {
		rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x0600, &data,
			CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (data == 0x00) {
        	break;
		}

		if((data != 0x00) && (i >= 5))
		{
			CAM_ERR(CAM_OIS, "Gyro Offset Cal FAIL ");

		}
	}

	cam_ois_read_gyrodata(o_ctrl, 0x0604, 0x0606, &gyro_offset, 1);

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x0004, &calib_data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if ((calib_data & 0x0100) == 0x0100) {
		CAM_ERR(CAM_OIS, "Gyro X Axis Offset Cal ERROR ");
		calib_ret = (0x1 << 16);
	}
	if ((calib_data & 0x0200) == 0x0200) {
		CAM_ERR(CAM_OIS, "Gyro Y Axis Offset Cal ERROR ");
		calib_ret |= (0x1);
	}

	if ((calib_data & (0x0100 | 0x0200)) == 0x0000)
	{
		RamWriteByte(o_ctrl, 0x300, 0x1, 100);
		for (i = 0; i < 5; i++) {
			rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x0300, &data,
				CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (data == 0x00) {
				break;
			} else if((data != 0x00) && (i >= 5)) {
				CAM_ERR(CAM_OIS, "Flash Save FAIL ");
			}
		}

		rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x0004, &calib_data,
			CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
		if ((calib_data & 0x0040) != 0x00)
		{
			CAM_ERR(CAM_OIS, "Gyro Offset Cal ERROR ");
			calib_ret = (0x1 << 15);
		}
	}

	return calib_ret;
}

static int cam_ois_bu63169_calibration(
	struct cam_ois_ctrl_t *o_ctrl) {
	int32_t   rc = 0;
	uint32_t  data = 0xFFFF;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x8406, &data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x8406 fail");
	} else {
		CAM_ERR(CAM_OIS, "get 0x8406 = 0x%0x", data);
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x8486, &data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x8486 fail");
	} else {
		CAM_ERR(CAM_OIS, "get 0x8486 = 0x%0x", data);
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x8407, &data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x8407 fail");
	} else {
		CAM_ERR(CAM_OIS, "get 0x8407 = 0x%0x", data);
	}

	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x8487, &data,
		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "read 0x8487 fail");
	} else {
		CAM_ERR(CAM_OIS, "get 0x8487 = 0x%0x", data);
	}

	return rc;
}

#define OIS_HALL_DATA_SIZE   52
static int cam_ois_bu63169_getmultiHall(
	struct cam_ois_ctrl_t *o_ctrl,
	struct ois_hall_type *o_hall)
{
	int32_t        rc = 0;
	uint8_t        data[OIS_HALL_DATA_SIZE] = {0x00};
	int            offset = 0;
	int            i = 0;
	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	memset(o_hall, 0x00, sizeof(struct ois_hall_type));
	rc = camera_io_dev_read_seq(&(o_ctrl->io_master_info), 0x8A, data,
		CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE, OIS_HALL_DATA_SIZE);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "get mutil hall data fail");
		return -EINVAL;
	}

	o_hall->dataNum = data[0];
	offset++;
	if (o_hall->dataNum <= 0 || o_hall->dataNum > HALL_MAX_NUMBER) {
		CAM_ERR(CAM_OIS, "get a wrong number of hall data:%d",	 o_hall->dataNum);
		return -EINVAL;
	}

	for (i = 0; i < o_hall->dataNum; i++) {
		o_hall->mdata[i] = ((data[offset+3] & 0x00FF)
			| ((data[offset+2] << 8) & 0xFF00)
			| ((data[offset+1] << 16) & 0x00FF0000)
			| ((data[offset] << 24) & 0xFF000000));
		offset += 4;
	}

	o_hall->timeStamp = ((uint32_t)(data[offset] << 8) | data[offset+1]);

	return rc;
}
static int cam_ois_bu63169_GyroPower(
	struct cam_ois_ctrl_t *o_ctrl,
	int poweron)
{
	int32_t        rc = 0;
	#ifdef VENDOR_EDIT
	struct cam_sensor_i2c_reg_setting sensor_setting;
	#endif

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}

	CAM_ERR(CAM_OIS, "set bu63169 Gyro power stats:%d", poweron);
	if (MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master) {
		sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
		if (poweron == 1) {
			sensor_setting.size = bu63169_gyroOn_settings.size;
			sensor_setting.delay = bu63169_gyroOn_settings.delay;
			sensor_setting.reg_setting = bu63169_gyroOn_settings.reg_setting;
		} else {
			sensor_setting.size = bu63169_gyroOff_settings.size;
			sensor_setting.delay = bu63169_gyroOff_settings.delay;
			sensor_setting.reg_setting = bu63169_gyroOff_settings.reg_setting;
		}
		rc = RamMultiWrite(o_ctrl, &sensor_setting);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "write power settings error");
		}
	}
	return 0;
}

static int cam_ois_sem1215s_getGyroNoise(
	struct cam_ois_ctrl_t *o_ctrl,
	uint32_t *gyro_noise)
{
	int32_t        rc = 0;
	uint32_t       gyro_noise_xaddr = 0x0B04;
	uint32_t       gyro_noise_yaddr = 0x0B06;
	int32_t        m_convert = 1;

	if (!o_ctrl) {
		CAM_ERR(CAM_OIS, "Invalid Args");
		return -EINVAL;
	}
	*gyro_noise = 0x00;
	rc = RamWriteByte(o_ctrl, 0x0B00, 0x01, 1);
	if (rc < 0) {
		CAM_ERR(CAM_OIS, "write power settings error");
	}
	m_convert = 1;
	cam_ois_read_gyrodata(o_ctrl, gyro_noise_xaddr, gyro_noise_yaddr, gyro_noise, m_convert);

	return 0;
}

#endif
/**
 * cam_ois_pkt_parse - Parse csl packet
 * @o_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
static int cam_ois_pkt_parse(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int32_t                         rc = 0;
	int32_t                         i = 0;
	uint32_t                        total_cmd_buf_in_bytes = 0;
	struct common_header           *cmm_hdr = NULL;
	uintptr_t                       generic_ptr;
	struct cam_control             *ioctl_ctrl = NULL;
	struct cam_config_dev_cmd       dev_config;
	struct i2c_settings_array      *i2c_reg_settings = NULL;
	struct cam_cmd_buf_desc        *cmd_desc = NULL;
	uintptr_t                       generic_pkt_addr;
	size_t                          pkt_len;
	size_t                          remain_len = 0;
	struct cam_packet              *csl_packet = NULL;
	size_t                          len_of_buff = 0;
	uint32_t                       *offset = NULL, *cmd_buf;
	#ifdef VENDOR_EDIT
	struct cam_sensor_i2c_reg_setting sensor_setting;
	#endif
	struct cam_ois_soc_private     *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t  *power_info = &soc_private->power_info;

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&dev_config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(dev_config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(dev_config.packet_handle,
		&generic_pkt_addr, &pkt_len);
	if (rc) {
		CAM_ERR(CAM_OIS,
			"error in converting command Handle Error: %d", rc);
		return rc;
	}

	remain_len = pkt_len;
	if ((sizeof(struct cam_packet) > pkt_len) ||
		((size_t)dev_config.offset >= pkt_len -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_OIS,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), pkt_len);
		rc = -EINVAL;
		goto rel_pkt;
	}

	remain_len -= (size_t)dev_config.offset;
	csl_packet = (struct cam_packet *)
		(generic_pkt_addr + (uint32_t)dev_config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_OIS, "Invalid packet params");
		rc = -EINVAL;
		goto rel_pkt;
	}


	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_OIS_PACKET_OPCODE_INIT:
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);

		/* Loop through multiple command buffers */
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			total_cmd_buf_in_bytes = cmd_desc[i].length;
			if (!total_cmd_buf_in_bytes)
				continue;

			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
				&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "Failed to get cpu buf : 0x%x",
					cmd_desc[i].mem_handle);
				goto rel_pkt;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_OIS, "invalid cmd buf");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}

			if ((len_of_buff < sizeof(struct common_header)) ||
				(cmd_desc[i].offset > (len_of_buff -
				sizeof(struct common_header)))) {
				CAM_ERR(CAM_OIS,
					"Invalid length for sensor cmd");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}
			remain_len = len_of_buff - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmm_hdr = (struct common_header *)cmd_buf;

			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				rc = cam_ois_slaveInfo_pkt_parser(
					o_ctrl, cmd_buf, remain_len);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"Failed in parsing slave info");
					goto rel_cmd_buf;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_OIS,
					"Received power settings buffer");
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					power_info, remain_len);
				if (rc) {
					CAM_ERR(CAM_OIS,
					"Failed: parse power settings");
					goto rel_cmd_buf;
				}
				break;
			default:
			if (o_ctrl->i2c_init_data.is_settings_valid == 0) {
				CAM_DBG(CAM_OIS,
				"Received init settings");
				i2c_reg_settings =
					&(o_ctrl->i2c_init_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
					"init parsing failed: %d", rc);
					goto rel_cmd_buf;
				}
			} else if ((o_ctrl->is_ois_calib != 0) &&
				(o_ctrl->i2c_calib_data.is_settings_valid ==
				0)) {
				CAM_DBG(CAM_OIS,
					"Received calib settings");
				i2c_reg_settings = &(o_ctrl->i2c_calib_data);
				i2c_reg_settings->is_settings_valid = 1;
				i2c_reg_settings->request_id = 0;
				rc = cam_sensor_i2c_command_parser(
					&o_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1);
				if (rc < 0) {
					CAM_ERR(CAM_OIS,
						"Calib parsing failed: %d", rc);
					goto rel_cmd_buf;
				}
			}
			break;
			}
			if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
				CAM_WARN(CAM_OIS, "Failed to put cpu buf: 0x%x",
					cmd_desc[i].mem_handle);
		}

		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = cam_ois_power_up(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, " OIS Power up failed");
				goto rel_pkt;
			}
			o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		}

		#ifdef VENDOR_EDIT
		if (MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master) {
			CAM_ERR(CAM_OIS, "need to write pll0 settings");
			sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
			sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
			sensor_setting.size = bu63169_pll_settings.size;
			sensor_setting.delay = bu63169_pll_settings.delay;
			sensor_setting.reg_setting = bu63169_pll_settings.reg_setting;
			rc = RamMultiWrite(o_ctrl, &sensor_setting);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "write pll settings error");
				goto pwr_dwn;
			}
		}
		if (o_ctrl->ois_fw_flag) {
			if (MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master) {
				rc = cam_ois_fw_download(o_ctrl);
			}
			#if 0
			else {
				rc = cam_sem1815s_ois_fw_download(o_ctrl);
			}
			#endif
			if (rc) {
				CAM_ERR(CAM_OIS, "Failed OIS FW Download");
				goto pwr_dwn;
			}
		}
		#else
		if (o_ctrl->ois_fw_flag) {
			rc = cam_ois_fw_download(o_ctrl);
			if (rc) {
				CAM_ERR(CAM_OIS, "Failed OIS FW Download");
				goto pwr_dwn;
			}
		}
		#endif

		#ifdef VENDOR_EDIT
		if (MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master) {
			uint32_t sum_check = 0;
        	rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x84F7, &sum_check,
        		CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "read 0x84F7 fail");
			} else {
				CAM_INFO(CAM_OIS, "0x84F7 = 0x%x", sum_check);
			}
			rc = camera_io_dev_read(&(o_ctrl->io_master_info), 0x84F6, &sum_check,
				CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "read 0x84F6 fail");
			} else {
				CAM_INFO(CAM_OIS, "0x84F6 = 0x%x", sum_check);
			}

		}
		#endif

		rc = cam_ois_apply_settings(o_ctrl, &o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply Init settings");
			goto pwr_dwn;
		}
		if (o_ctrl->is_ois_calib) {
			rc = cam_ois_apply_settings(o_ctrl,
				&o_ctrl->i2c_calib_data);
			if (rc) {
				CAM_ERR(CAM_OIS, "Cannot apply calib data");
				goto pwr_dwn;
			}
		}
		rc = delete_request(&o_ctrl->i2c_init_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Init data: rc: %d", rc);
			rc = 0;
		}
		rc = delete_request(&o_ctrl->i2c_calib_data);
		if (rc < 0) {
			CAM_WARN(CAM_OIS,
				"Fail deleting Calibration data: rc: %d", rc);
			rc = 0;
		}
		break;
	case CAM_OIS_PACKET_OPCODE_OIS_CONTROL:
		if (o_ctrl->cam_ois_state < CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Not in right state to control OIS: %d",
				o_ctrl->cam_ois_state);
			goto rel_pkt;
		}
		offset = (uint32_t *)&csl_packet->payload;
		offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		i2c_reg_settings = &(o_ctrl->i2c_mode_data);
		i2c_reg_settings->is_settings_valid = 1;
		i2c_reg_settings->request_id = 0;
		rc = cam_sensor_i2c_command_parser(&o_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "OIS pkt parsing failed: %d", rc);
			goto rel_pkt;
		}

		rc = cam_ois_apply_settings(o_ctrl, i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "Cannot apply mode settings");
			goto rel_pkt;
		}

		rc = delete_request(i2c_reg_settings);
		if (rc < 0) {
			CAM_ERR(CAM_OIS,
				"Fail deleting Mode data: rc: %d", rc);
			goto rel_pkt;
		}
		break;
	default:
		CAM_ERR(CAM_OIS, "Invalid Opcode: %d",
			(csl_packet->header.op_code & 0xFFFFFF));
		rc = -EINVAL;
		goto rel_pkt;
	}

	if (!rc)
		goto rel_pkt;

rel_cmd_buf:
	if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
		CAM_WARN(CAM_OIS, "Failed to put cpu buf: 0x%x",
			cmd_desc[i].mem_handle);
pwr_dwn:
	cam_ois_power_down(o_ctrl);
rel_pkt:
	if (cam_mem_put_cpu_buf(dev_config.packet_handle))
		CAM_WARN(CAM_OIS, "Fail in put buffer: 0x%x",
			dev_config.packet_handle);

	return rc;
}

void cam_ois_shutdown(struct cam_ois_ctrl_t *o_ctrl)
{
	int rc = 0;
	struct cam_ois_soc_private *soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info = &soc_private->power_info;

	if (o_ctrl->cam_ois_state == CAM_OIS_INIT)
		return;

	if (o_ctrl->cam_ois_state >= CAM_OIS_CONFIG) {
		rc = cam_ois_power_down(o_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "OIS Power down failed");
		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
	}

	if (o_ctrl->cam_ois_state >= CAM_OIS_ACQUIRE) {
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
	}

	if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_mode_data);

	if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_calib_data);

	if (o_ctrl->i2c_init_data.is_settings_valid == 1)
		delete_request(&o_ctrl->i2c_init_data);

	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_down_setting_size = 0;
	power_info->power_setting_size = 0;

	o_ctrl->cam_ois_state = CAM_OIS_INIT;
}

/**
 * cam_ois_driver_cmd - Handle ois cmds
 * @e_ctrl:     ctrl structure
 * @arg:        Camera control command argument
 *
 * Returns success or failure
 */
int cam_ois_driver_cmd(struct cam_ois_ctrl_t *o_ctrl, void *arg)
{
	int                              rc = 0;
	struct cam_ois_query_cap_t       ois_cap = {0};
	struct cam_control              *cmd = (struct cam_control *)arg;
	struct cam_ois_soc_private      *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;
	#ifdef VENDOR_EDIT
	uint32_t gyro_x_addr = 0;
	uint32_t gyro_y_addr = 0;
	uint32_t hall_x_addr = 0;
	uint32_t hall_y_addr = 0;
	#endif

	if (!o_ctrl || !cmd) {
		CAM_ERR(CAM_OIS, "Invalid arguments");
		return -EINVAL;
	}

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_OIS, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	soc_private =
		(struct cam_ois_soc_private *)o_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	mutex_lock(&(o_ctrl->ois_mutex));
	switch (cmd->op_code) {
	case CAM_QUERY_CAP:
		ois_cap.slot_info = o_ctrl->soc_info.index;

		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&ois_cap,
			sizeof(struct cam_ois_query_cap_t))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		CAM_DBG(CAM_OIS, "ois_cap: ID: %d", ois_cap.slot_info);
		break;
	case CAM_ACQUIRE_DEV:
		rc = cam_ois_get_dev_handle(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed to acquire dev");
			goto release_mutex;
		}

		o_ctrl->cam_ois_state = CAM_OIS_ACQUIRE;
		break;
	case CAM_START_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for start : %d",
			o_ctrl->cam_ois_state);
			goto release_mutex;
		}
		o_ctrl->cam_ois_state = CAM_OIS_START;
		break;
	case CAM_CONFIG_DEV:
		rc = cam_ois_pkt_parse(o_ctrl, arg);
		if (rc) {
			CAM_ERR(CAM_OIS, "Failed in ois pkt Parsing");
			#ifdef VENDOR_EDIT
			rc = 0;
			#endif
			goto release_mutex;
		}
		break;
	case CAM_RELEASE_DEV:
		if (o_ctrl->cam_ois_state == CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
				"Cant release ois: in start state");
			goto release_mutex;
		}

		if (o_ctrl->cam_ois_state == CAM_OIS_CONFIG) {
			rc = cam_ois_power_down(o_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_OIS, "OIS Power down failed");
				goto release_mutex;
			}
		}

		if (o_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_OIS, "link hdl: %d device hdl: %d",
				o_ctrl->bridge_intf.device_hdl,
				o_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = cam_destroy_device_hdl(o_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_OIS, "destroying the device hdl");
		o_ctrl->bridge_intf.device_hdl = -1;
		o_ctrl->bridge_intf.link_hdl = -1;
		o_ctrl->bridge_intf.session_hdl = -1;
		o_ctrl->cam_ois_state = CAM_OIS_INIT;

		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_down_setting_size = 0;
		power_info->power_setting_size = 0;

		if (o_ctrl->i2c_mode_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_mode_data);

		if (o_ctrl->i2c_calib_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_calib_data);

		if (o_ctrl->i2c_init_data.is_settings_valid == 1)
			delete_request(&o_ctrl->i2c_init_data);

		break;
	case CAM_STOP_DEV:
		if (o_ctrl->cam_ois_state != CAM_OIS_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_OIS,
			"Not in right state for stop : %d",
			o_ctrl->cam_ois_state);
		}
		o_ctrl->cam_ois_state = CAM_OIS_CONFIG;
		break;

	#ifdef VENDOR_EDIT
	case CAM_GET_OIS_GYRO_OFFSET: {
		uint32_t gyro_offset = 0;
		bool m_convert = 0;
		if (MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master) {
			gyro_x_addr = 0x8455;
			gyro_y_addr = 0x8456;
			m_convert = 0;
		} else { //we need to change these later for tele camera
			gyro_x_addr = 0x0604;
			gyro_y_addr = 0x0606;
			m_convert = 1;
		}
		cam_ois_read_gyrodata(o_ctrl, gyro_x_addr, gyro_y_addr, &gyro_offset, m_convert);
		#if 0
		CAM_INFO(CAM_OIS, "i2c_master:%d read gyro offset ", o_ctrl->io_master_info.cci_client->cci_i2c_master);
		rc = camera_io_dev_read(&(o_ctrl->io_master_info), gyro_x_addr, &gyro_offset_x,
			CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
        	if (rc < 0) {
			CAM_ERR(CAM_OIS, "read gyro offset_x fail");
		}
		gyro_offset_x = (gyro_offset_x & 0xFF) << 8 | (gyro_offset_x & 0xFF00) >> 8;

		rc = camera_io_dev_read(&(o_ctrl->io_master_info), gyro_y_addr, &gyro_offset_y,
			CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
        	if (rc < 0) {
			CAM_ERR(CAM_OIS, "read gyro offset_y fail");
		}
		gyro_offset_y = (gyro_offset_y & 0xFF) << 8 | (gyro_offset_y & 0xFF00) >> 8;

		gyro_offset = ((gyro_offset_y & 0xFFFF) << 16) | (gyro_offset_x & 0xFFFF);
		CAM_INFO(CAM_OIS, "final gyro_offset = 0x%x; gyro_x=0x%x, gyro_y=0x%x",
			gyro_offset, gyro_offset_x, gyro_offset_y);
		#endif

		if (copy_to_user((void __user *) cmd->handle, &gyro_offset,
			sizeof(gyro_offset))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		break;
	}

	case CAM_GET_OIS_HALL_POSITION: {
		uint32_t hall_position = 0;
		uint32_t hall_position_x = 0;
		uint32_t hall_position_y = 0;
		if (MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master) {
			hall_x_addr = 0x843f;
			hall_y_addr = 0x84bf;
		} else {  //we need to change these later for tele camera
			hall_x_addr = 0x0B10;
			hall_y_addr = 0x0B12;
		}

		rc = camera_io_dev_read(&(o_ctrl->io_master_info), hall_x_addr, &hall_position_x,
			CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "read hall_position_x fail");
		}

		rc = camera_io_dev_read(&(o_ctrl->io_master_info), hall_y_addr, &hall_position_y,
			CAMERA_SENSOR_I2C_TYPE_WORD, CAMERA_SENSOR_I2C_TYPE_WORD);
		if (rc < 0) {
			CAM_ERR(CAM_OIS, "read hall_position_y fail");
		}

		hall_position = ((hall_position_y & 0xFFFF) << 16) | (hall_position_x & 0xFFFF);
		CAM_INFO(CAM_OIS, "final hall_position = 0x%x; hall_position_x=0x%x, hall_position_y=0x%x",
			hall_position, hall_position_x, hall_position_y);

		if (copy_to_user((void __user *) cmd->handle, &hall_position,
			sizeof(hall_position))) {
			CAM_ERR(CAM_OIS, "Failed Copy hall data to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		break;
	}
	case CAM_OIS_GYRO_OFFSET_CALIBRATION: {
		uint32_t result = 0;
		if (MASTER_1 == o_ctrl->io_master_info.cci_client->cci_i2c_master) {
			result = cam_ois_sem1215s_calibration(o_ctrl);
		} else {
			result = cam_ois_bu63169_calibration(o_ctrl);
		}
		if (copy_to_user((void __user *) cmd->handle, &result,
			sizeof(result))) {
			CAM_ERR(CAM_OIS, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
		break;
	}
	case CAM_GET_OIS_EIS_HALL: {
		struct ois_hall_type halldata;
		int m_result = 0;
		if (MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master) {
			cam_ois_bu63169_getmultiHall(o_ctrl, &halldata);

			m_result = copy_to_user((void __user *) cmd->handle, &halldata, sizeof(halldata));
			if (m_result != 0) {
				CAM_ERR(CAM_OIS, "Failed Copy multi hall data to User:%d !!!", m_result);
				rc = -EFAULT;
				goto release_mutex;
			}
		}
		break;
	}
	case CAM_SET_GYRO_POWER_STATUS: {
		int m_result = 0;
		int power = (int)cmd->reserved;
		if (MASTER_0 == o_ctrl->io_master_info.cci_client->cci_i2c_master) {
			m_result = cam_ois_bu63169_GyroPower(o_ctrl, power);

			m_result = copy_to_user((void __user *) cmd->handle, &m_result, sizeof(m_result));
			if (m_result != 0) {
				CAM_ERR(CAM_OIS, "Failed set power status:%d !!!", power);
				rc = -EFAULT;
				goto release_mutex;
			}
		}
		break;
	}

	case CAM_GET_GYRO_NOISE: {
		int m_result = 0;
		uint32_t gyro_noise = 0x00;
		if (MASTER_1 == o_ctrl->io_master_info.cci_client->cci_i2c_master) {
			m_result = cam_ois_sem1215s_getGyroNoise(o_ctrl, &gyro_noise);
			if (copy_to_user((void __user *) cmd->handle, &gyro_noise,
				sizeof(gyro_noise))) {
				CAM_ERR(CAM_OIS, "Failed Copy to User");
				rc = -EFAULT;
				goto release_mutex;
			}
		}
		break;
	}
	#endif

	default:
		CAM_ERR(CAM_OIS, "invalid opcode");
		goto release_mutex;
	}
release_mutex:
	mutex_unlock(&(o_ctrl->ois_mutex));
	return rc;
}
