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
#include <cam_sensor_cmn_header.h>
#include "cam_actuator_core.h"
#include "cam_sensor_util.h"
#include "cam_trace.h"
#include "cam_res_mgr_api.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"

#include "cam_actuator_dev.h"

#define ak7375c_write_slave 0xE8
#define ak7375c_read_slave 0xE9
#define check_store_ak7375c_add 0x4B
#define check_ak7375c_PID_size 24
#define update_ak7375c_PID_size 22

static uint32_t change_ak7375c_setting_mode[4] = {0xAE, 0x3B, 0x00, 0x0};

static uint32_t release_ak7375c_setting_mode[4] = {0xAE, 0x00, 0x00, 0x0};

static uint32_t store_ak7375c_arr[4] = {0x03, 0x02, 0xD8, 0x0}; // store delay 216ms

static uint32_t update_ak7375c_PID_arr[update_ak7375c_PID_size][4] = {
	{0x10, 0x24, 0x00, 0x0},
	{0x11, 0x3C, 0x00, 0x0},
	{0x12, 0x54, 0x00, 0x0},
	{0x13, 0x56, 0x00, 0x0},
	{0x14, 0x21, 0x00, 0x0},
	{0x15, 0x00, 0x00, 0x0},
	{0x16, 0x21, 0x00, 0x0},
	{0x17, 0x40, 0x00, 0x0},
	{0x18, 0xDB, 0x00, 0x0},
	{0x1A, 0x00, 0x00, 0x0},
	{0x1B, 0x5C, 0x00, 0x0},
	{0x1C, 0xB0, 0x00, 0x0},
	{0x1D, 0x92, 0x00, 0x0},
	{0x1E, 0x50, 0x00, 0x0},
	{0x1F, 0x52, 0x00, 0x0},
	{0x20, 0x01, 0x00, 0x0},
	{0x21, 0x02, 0x00, 0x0},
	{0x22, 0x04, 0x00, 0x0},
	{0x23, 0x28, 0x00, 0x0},
	{0x24, 0xFF, 0x00, 0x0},
	{0x25, 0x1E, 0x00, 0x0},
	{0x26, 0x58, 0x00, 0x0}
};

static uint32_t check_ak7375c_PID_arr[check_ak7375c_PID_size][4] = {
	{0x0A, 0x89, 0x00, 0x0},
	{0x0B, 0x88, 0x00, 0x0},
	{0x10, 0x24, 0x00, 0x0},
	{0x11, 0x3C, 0x00, 0x0},
	{0x12, 0x54, 0x00, 0x0},
	{0x13, 0x56, 0x00, 0x0},
	{0x14, 0x21, 0x00, 0x0},
	{0x15, 0x00, 0x00, 0x0},
	{0x16, 0x21, 0x00, 0x0},
	{0x17, 0x40, 0x00, 0x0},
	{0x18, 0xDB, 0x00, 0x0},
	{0x1A, 0x00, 0x00, 0x0},
	{0x1B, 0x5C, 0x00, 0x0},
	{0x1C, 0xB0, 0x00, 0x0},
	{0x1D, 0x92, 0x00, 0x0},
	{0x1E, 0x50, 0x00, 0x0},
	{0x1F, 0x52, 0x00, 0x0},
	{0x20, 0x01, 0x00, 0x0},
	{0x21, 0x02, 0x00, 0x0},
	{0x22, 0x04, 0x00, 0x0},
	{0x23, 0x28, 0x00, 0x0},
	{0x24, 0xFF, 0x00, 0x0},
	{0x25, 0x1E, 0x00, 0x0},
	{0x26, 0x58, 0x00, 0x0}
};

#define UPDATE_REG_SIZE 64
static uint32_t update_reg_arr[64][4] = {
	{0x00, 0x33, 0x00, 0x0},
	{0x01, 0x08, 0x00, 0x0},
	{0x02, 0x48, 0x00, 0x0},
	{0x03, 0x40, 0x00, 0x0},
	{0x04, 0x4B, 0x00, 0x0},
	{0x05, 0x60, 0x00, 0x0},
	{0x06, 0xEE, 0x00, 0x0},
	{0x07, 0x81, 0x00, 0x0},

	{0x08, 0x6F, 0x00, 0x0},
	{0x09, 0x8D, 0x00, 0x0},
	{0x0A, 0x14, 0x00, 0x0},
	{0x0B, 0x60, 0x00, 0x0},
	{0x0C, 0x56, 0x00, 0x0},
	{0x0D, 0x52, 0x00, 0x0},
	{0x0E, 0x51, 0x00, 0x0},
	{0x0F, 0x48, 0x00, 0x0},

	{0x10, 0x79, 0x00, 0x0},
	{0x11, 0xD0, 0x00, 0x0},
	{0x12, 0x5F, 0x00, 0x0},
	{0x13, 0x18, 0x00, 0x0},
	{0x14, 0x81, 0x00, 0x0},
	{0x15, 0xFF, 0x00, 0x0},
	{0x16, 0x78, 0x00, 0x0},
	{0x17, 0x10, 0x00, 0x0},

	{0x18, 0x00, 0x00, 0x0},
	{0x19, 0x07, 0x00, 0x0},
	{0x1A, 0x06, 0x00, 0x0},
	{0x1B, 0x31, 0x00, 0x0},
	{0x1C, 0xCB, 0x00, 0x0},
	{0x1D, 0x68, 0x00, 0x0},
	{0x1E, 0x7F, 0x00, 0x0},
	{0x1F, 0x7F, 0x00, 0x0},

	{0x20, 0x40, 0x00, 0x0},
	{0x21, 0x0C, 0x00, 0x0},
	{0x22, 0x00, 0x00, 0x0},
	{0x23, 0x22, 0x00, 0x0},
	{0x24, 0xE1, 0x00, 0x0},
	{0x25, 0x80, 0x00, 0x0},
	{0x26, 0x00, 0x00, 0x0},
	{0x27, 0x5B, 0x00, 0x0},

	{0x28, 0x45, 0x00, 0x0},
	{0x29, 0xF7, 0x00, 0x0},
	{0x2A, 0x2F, 0x00, 0x0},
	{0x2B, 0x51, 0x00, 0x0},
	{0x2C, 0xFE, 0x00, 0x0},
	{0x2D, 0xDD, 0x00, 0x0},
	{0x2E, 0xDE, 0x00, 0x0},
	{0x2F, 0xEE, 0x00, 0x0},

	{0x30, 0xFF, 0x00, 0x0},
	{0x31, 0xFF, 0x00, 0x0},
	{0x32, 0xFF, 0x00, 0x0},
	{0x33, 0xFF, 0x00, 0x0},
	{0x34, 0xFF, 0x00, 0x0},
	{0x35, 0xFF, 0x00, 0x0},
	{0x36, 0xFF, 0x00, 0x0},
	{0x37, 0xFF, 0x00, 0x0},

	{0x38, 0x7F, 0x00, 0x0},
	{0x39, 0xFF, 0x00, 0x0},
	{0x3A, 0xFF, 0x00, 0x0},
	{0x3B, 0xFF, 0x00, 0x0},
	{0x3C, 0x20, 0x00, 0x0},
	{0x3D, 0x11, 0x00, 0x0},
	{0x3E, 0x18, 0x00, 0x0},
	{0x3F, 0x01, 0x00, 0x0}

};

int32_t cam_actuator_construct_default_power_setting(
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

int RamWriteByte(struct cam_actuator_ctrl_t *a_ctrl,
	uint32_t addr, uint32_t data, unsigned short mdelay)
{
	int32_t rc = 0;
	int retry = 1;
	int i = 0;
	uint32_t read_data = 0;
	struct cam_sensor_i2c_reg_array i2c_write_setting = {
		.reg_addr = addr,
		.reg_data = data,
		.delay = mdelay,
		.data_mask = 0x00,
	};
	struct cam_sensor_i2c_reg_setting i2c_write = {
		.reg_setting = &i2c_write_setting,
		.size = 1,
		.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE,
		.delay = mdelay,
	};
	if (a_ctrl == NULL) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	read_data = 0;
	rc = camera_io_dev_read(
		&(a_ctrl->io_master_info),
		addr, &read_data,
		CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0){
		CAM_ERR(CAM_ACTUATOR, "read error");
		//break;
	}
	CAM_ERR(CAM_ACTUATOR, "before write data addr:%x data:%x ,will write data is %x",addr,read_data,data);

	for(i = 0; i < retry; i++)
	{
		rc = camera_io_dev_write(&(a_ctrl->io_master_info), &i2c_write);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "write 0x%04x failed, retry:%d", addr, i+1);
		} else {
			break;
		}
	}

	read_data = 0;
	rc = camera_io_dev_read(
		&(a_ctrl->io_master_info),
		addr, &read_data,
		CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
	if (rc < 0){
		CAM_ERR(CAM_ACTUATOR, "write error");
		//break;
	}

	CAM_ERR(CAM_ACTUATOR, "after write data addr:%x data:%x",addr,read_data);
	return rc;
}

int32_t cam_actuator_update_pid(struct cam_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	uint32_t IDSEL;
	uint32_t HallCal[5];
	uint32_t Linearity[8];
	int i;
	uint32_t temp = 0;
	uint32_t read_data = 0;

	CAM_ERR(CAM_ACTUATOR, "entry cam_actuator_update_pid");
	if (a_ctrl->need_check_pid &&
		0xE4 >> 1 == a_ctrl->io_master_info.cci_client->sid) {

		a_ctrl->io_master_info.cci_client->sid = 0xE6 >> 1;//read from eeporm
		msleep(10);

		for(i=0; i< UPDATE_REG_SIZE; i++)
		{
			rc = camera_io_dev_read(
					&(a_ctrl->io_master_info),
					update_reg_arr[i][0], &read_data,
					CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0){
				CAM_ERR(CAM_ACTUATOR, "camera_io_dev_read error");
			}
			CAM_ERR(CAM_ACTUATOR, " original addr:%x data:%x",update_reg_arr[i][0],read_data);
		}

		// backup Hall calibration data
		for(i=0; i<5; i++)
		{
			temp = i+0x1E;
			rc = camera_io_dev_read(
				&(a_ctrl->io_master_info),
				temp, &HallCal[i],
				CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0){
				CAM_ERR(CAM_ACTUATOR, "camera_io_dev_read error");
			}
			CAM_ERR(CAM_ACTUATOR, "backup Hall calibration data addr:%x data:%x",temp,HallCal[i]);
		}


		// backup Linearity data
		rc = camera_io_dev_read(
			&(a_ctrl->io_master_info),
			0x06, &IDSEL,
			CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
		if (rc < 0){
			CAM_ERR(CAM_ACTUATOR, "camera_io_dev_read error");
		}
		CAM_ERR(CAM_ACTUATOR, "read IDSEL data addr:0x06 data:%x",IDSEL);


		if (IDSEL == 0xC3) // check bit1-2 = 0110b?
		{
			// Linearity correction area(128h - 12Fh)
			for(i=0; i<8; i++)
			{
				temp = i+0x28;
				rc = camera_io_dev_read(
					&(a_ctrl->io_master_info),
					temp, &Linearity[i],
					CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
				if (rc < 0){
					CAM_ERR(CAM_ACTUATOR, "camera_io_dev_read error");
				}
				CAM_ERR(CAM_ACTUATOR, "read data addr:%x data:%x",temp,Linearity[i]);
			}

		}else{
			// Linearity correction area(130h - 137h)   IDSEL == 0xC2 || IDSEL == 0xE0
			for(i=0; i<8; i++)
			{
				temp = i+0x30;
				rc = camera_io_dev_read(
					&(a_ctrl->io_master_info),
					temp, &Linearity[i],
					CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
				if (rc < 0){
					CAM_ERR(CAM_ACTUATOR, "camera_io_dev_read error");
				}
				CAM_ERR(CAM_ACTUATOR, "read data addr:%x data:%x",temp,Linearity[i]);
			}
		}

		a_ctrl->io_master_info.cci_client->sid = 0xE4 >> 1;
		// Write all data to EEPROM
		RamWriteByte(a_ctrl, 0x98, 0xE2, 0);		// Release Write Protect
		RamWriteByte(a_ctrl, 0x99, 0xAF, 0);		// Release Write Protect


		a_ctrl->io_master_info.cci_client->sid = 0xE6 >> 1;

		for(i=0; i< UPDATE_REG_SIZE; i++)
		{
			RamWriteByte(a_ctrl, update_reg_arr[i][0], update_reg_arr[i][1], 20);
			// wait 20 msec
		}

		// recover Hall calibration data
		for(i=0; i<5; i++)
		{
			RamWriteByte(a_ctrl, i+0x1E, HallCal[i], 20);
			// wait 20 msec
		}

		// Linearity correction area(130h - 137h)
		for(i=0; i<8; i++)
		{
			RamWriteByte(a_ctrl, i+0x30, Linearity[i], 20);
			// wait 20 msec
		}

		if(IDSEL == 0XC3){
			RamWriteByte(a_ctrl, 0X06, 0xEF, 20);
		}else{
			RamWriteByte(a_ctrl, 0X06, 0xEE, 20);
		}

		a_ctrl->io_master_info.cci_client->sid = 0xE4 >> 1;
		RamWriteByte(a_ctrl, 0x98, 0x00, 0);
		RamWriteByte(a_ctrl, 0x99, 0x00, 0);

		RamWriteByte(a_ctrl, 0xE0, 0x01, 0);

		a_ctrl->io_master_info.cci_client->sid = 0xE6 >> 1;

		for(i=0; i< UPDATE_REG_SIZE; i++)
		{
			rc = camera_io_dev_read(
				&(a_ctrl->io_master_info),
				update_reg_arr[i][0], &read_data,
				CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0){
				CAM_ERR(CAM_ACTUATOR, "camera_io_dev_read error");
			}
			CAM_ERR(CAM_ACTUATOR, " final addr:%x data:%x",update_reg_arr[i][0],read_data);
		}
	}

		a_ctrl->io_master_info.cci_client->sid = 0xE4 >> 1;

	return rc;
}

int32_t cam_actuator_check_firmware(struct cam_actuator_ctrl_t *a_ctrl)
 {
	int32_t cnt = 0;
	int32_t rc = 0;
	uint32_t reg_data = 0;
	if (0xE4 >> 1 == a_ctrl->io_master_info.cci_client->sid) {
		a_ctrl->io_master_info.cci_client->sid = 0xE6 >> 1;
		for(cnt = 0; cnt < UPDATE_REG_SIZE; cnt++)
		{
		   rc = camera_io_dev_read(
				&(a_ctrl->io_master_info),
				update_reg_arr[cnt][0], &reg_data,
				CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0){
				CAM_ERR(CAM_ACTUATOR, "read PID data error in step %d:rc %d", cnt, rc);
				break;
			}
			if(update_reg_arr[cnt][0] >= 0x07 && update_reg_arr[cnt][0] <= 0x1d){
				if (reg_data != update_reg_arr[cnt][1]){
					CAM_ERR(CAM_ACTUATOR, "new PID data wrong in step %d:rc %d, reg_data is %x", cnt, rc,reg_data);
					rc = -1;
					break;
				}
			}else if(update_reg_arr[cnt][0] >= 0x23 && update_reg_arr[cnt][0] <= 0x27){
				if (reg_data != update_reg_arr[cnt][1]){
					CAM_ERR(CAM_ACTUATOR, "new PID data wrong in step %d:rc %d, reg_data is %x", cnt, rc,reg_data);
					rc = -1;
					break;
				}
			}else if(update_reg_arr[cnt][0] >= 0x38 && update_reg_arr[cnt][0] <= 0x3f){
				if (reg_data != update_reg_arr[cnt][1]){
					CAM_ERR(CAM_ACTUATOR, "new PID data wrong in step %d:rc %d, reg_data is %x", cnt, rc,reg_data);
					rc = -1;
					break;
				}
			}
		}
	}

	a_ctrl->io_master_info.cci_client->sid = 0xE4 >> 1;

	return rc;
 }

int32_t cam_AK7375c_actuator_check_pid(struct cam_actuator_ctrl_t *a_ctrl) {
	int32_t cnt = 0;
	int32_t rc = 0;
	uint32_t reg_data = 0;
	if (ak7375c_write_slave >> 1 == a_ctrl->io_master_info.cci_client->sid) {
		a_ctrl->io_master_info.cci_client->sid = ak7375c_read_slave >> 1;
		for (cnt = 0; cnt < check_ak7375c_PID_size; cnt++)
		{
			rc = camera_io_dev_read(
				&(a_ctrl->io_master_info),
				check_ak7375c_PID_arr[cnt][0], &reg_data,
				CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
			if (rc < 0){
				CAM_ERR(CAM_ACTUATOR, "read PID data error in step %d:rc %d", cnt, rc);
				break;
			}
			if (reg_data != check_ak7375c_PID_arr[cnt][1])
			{
				CAM_ERR(CAM_ACTUATOR, "new PID data wrong in step %d:rc %d, des_data=0x%x,curr_data=0x%x", cnt, rc,check_ak7375c_PID_arr[cnt][1],reg_data);
				rc = -1;
				break;
			}
		}
		a_ctrl->io_master_info.cci_client->sid = ak7375c_write_slave >> 1;
	}

	return rc;
}

int32_t cam_AK7375c_actuator_update_pid(struct cam_actuator_ctrl_t *a_ctrl) {
	int32_t cnt = 0;
	int32_t i = 0;
	int32_t rc = 0;
	int32_t retry = 2;
	uint32_t reg_data = 0xff;

	CAM_INFO(CAM_ACTUATOR, "entry cam_actuator_update_pid");

/*change to Setting mode*/
	rc = RamWriteByte(a_ctrl, change_ak7375c_setting_mode[0], change_ak7375c_setting_mode[1], change_ak7375c_setting_mode[2]);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "change to Setting mode failed,rc = %d",rc);
		return rc;
	}

/*write PID */
	//msleep(20);
	for (cnt = 0; cnt < update_ak7375c_PID_size; cnt++)
	{
		rc = RamWriteByte(a_ctrl, update_ak7375c_PID_arr[cnt][0], update_ak7375c_PID_arr[cnt][1], update_ak7375c_PID_arr[cnt][2]);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "write PID data error addr0x%x,data0x%x",update_ak7375c_PID_arr[cnt][0],update_ak7375c_PID_arr[cnt][1]);
			break;
		}
	}
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "write PID failed,rc = %d", rc);
		return rc;
	}

/* store instruction for max 2 times*/
	for (i = 0; i < retry; i++)
	{
		rc = RamWriteByte(a_ctrl, store_ak7375c_arr[0], store_ak7375c_arr[1], store_ak7375c_arr[2]);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "store PID failed %d times", i);
			continue;
		}
		//msleep(200);

		/*check store instruction*/
		a_ctrl->io_master_info.cci_client->sid = ak7375c_read_slave >> 1;
		rc = camera_io_dev_read(
			&(a_ctrl->io_master_info),
			check_store_ak7375c_add, &reg_data,
			CAMERA_SENSOR_I2C_TYPE_BYTE, CAMERA_SENSOR_I2C_TYPE_BYTE);
		a_ctrl->io_master_info.cci_client->sid = ak7375c_write_slave >> 1;

		if (rc < 0){
			CAM_ERR(CAM_ACTUATOR, "read data error");
			continue;
		}
		/*the 2st bit of data is 0? */
		if (reg_data&0x04) {
			CAM_ERR(CAM_ACTUATOR, "stroe failed %d times ,read data=0x%x",i,reg_data);	//	stroe failed, try 2 tiams to write addr03h data 02h
			rc = -1;
			continue;
		} else {
			break;
		}

	}
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "store PID failed,rc = %d", rc);
		return rc;
	}

/* release setting mode*/
	rc = RamWriteByte(a_ctrl, release_ak7375c_setting_mode[0], release_ak7375c_setting_mode[1], release_ak7375c_setting_mode[2]);
	if (rc < 0){
		CAM_ERR(CAM_ACTUATOR, "release setting mode failed");
	}

	return rc;
}

static int32_t cam_actuator_power_up(struct cam_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;
	int retry = 2;
	int re = 0;

	struct cam_hw_soc_info  *soc_info =
		&a_ctrl->soc_info;
	struct cam_actuator_soc_private  *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_ACTUATOR,
			"Using default power settings");
		rc = cam_actuator_construct_default_power_setting(power_info);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Construct default actuator power setting failed.");
			return rc;
		}
	}

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		&a_ctrl->soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		&a_ctrl->soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR,
			"failed to fill vreg params power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR,
			"failed in actuator power up rc %d", rc);
		return rc;
	}

	rc = camera_io_init(&a_ctrl->io_master_info);
	if (rc < 0)
		CAM_ERR(CAM_ACTUATOR, "cci init failed: rc: %d", rc);

	CAM_INFO(CAM_ACTUATOR, "a_ctrl->io_master_info.cci_client->sid = %x",a_ctrl->io_master_info.cci_client->sid << 1);
	CAM_INFO(CAM_ACTUATOR, "need_check_pid = %d, pid_data_updated = %d",a_ctrl->need_check_pid,a_ctrl->pid_data_updated);

	if (a_ctrl->need_check_pid &&
		0xE4 >> 1 == a_ctrl->io_master_info.cci_client->sid) {
		for (re = 0;re < retry; re++){
			rc = cam_actuator_check_firmware(a_ctrl);
			if (rc < 0){
				//if rc is error ,update the pid eeprom
				CAM_ERR(CAM_ACTUATOR, "start store the pid data!");
				rc = cam_actuator_update_pid(a_ctrl);
			}
			if (rc < 0){
				CAM_ERR(CAM_ACTUATOR, "update the pid data error,check the io ctrl!");
			}else {
				break;
			}
		}
	}
	else if (a_ctrl->need_check_pid && !a_ctrl->pid_data_updated &&
		ak7375c_write_slave >> 1 == a_ctrl->io_master_info.cci_client->sid) {
		for (re = 0; re < retry; re++){
			rc = cam_AK7375c_actuator_check_pid(a_ctrl);
			if (rc < 0){
				//if rc is error ,update the pid eeprom
				CAM_INFO(CAM_ACTUATOR, "start to update the pid data!");
				rc = cam_AK7375c_actuator_update_pid(a_ctrl);
				if (rc < 0){
					CAM_ERR(CAM_ACTUATOR, "updated the pid data error!");
				}else {
					CAM_INFO(CAM_ACTUATOR, "updated the pid data succeed!");
					a_ctrl->pid_data_updated = true;
					break;
				}
			} else {
				CAM_INFO(CAM_ACTUATOR, "had updated the pid data succeed!");
				a_ctrl->pid_data_updated = true;
				break;
			}
		}
	}
	return rc;
}

static int32_t cam_actuator_power_down(struct cam_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_hw_soc_info *soc_info = &a_ctrl->soc_info;
	struct cam_actuator_soc_private  *soc_private;

	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "failed: a_ctrl %pK", a_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &a_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_ACTUATOR, "failed: power_info %pK", power_info);
		return -EINVAL;
	}
	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR, "power down the core is failed:%d", rc);
		return rc;
	}

	camera_io_release(&a_ctrl->io_master_info);

	return rc;
}

static int32_t cam_actuator_i2c_modes_util(
	struct camera_io_master *io_master_info,
	struct i2c_settings_list *i2c_list)
{
	int32_t rc = 0;
	uint32_t i, size;

	if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_RANDOM) {
		rc = camera_io_dev_write(io_master_info,
			&(i2c_list->i2c_settings));
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed to random write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_SEQ) {
		rc = camera_io_dev_write_continuous(
			io_master_info,
			&(i2c_list->i2c_settings),
			0);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed to seq write I2C settings: %d",
				rc);
			return rc;
			}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_BURST) {
		rc = camera_io_dev_write_continuous(
			io_master_info,
			&(i2c_list->i2c_settings),
			1);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed to burst write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
		size = i2c_list->i2c_settings.size;
		for (i = 0; i < size; i++) {
			rc = camera_io_dev_poll(
			io_master_info,
			i2c_list->i2c_settings.reg_setting[i].reg_addr,
			i2c_list->i2c_settings.reg_setting[i].reg_data,
			i2c_list->i2c_settings.reg_setting[i].data_mask,
			i2c_list->i2c_settings.addr_type,
			i2c_list->i2c_settings.data_type,
			i2c_list->i2c_settings.reg_setting[i].delay);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR,
					"i2c poll apply setting Fail: %d", rc);
				return rc;
			}
		}
	}

	return rc;
}

int32_t cam_actuator_slaveInfo_pkt_parser(struct cam_actuator_ctrl_t *a_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_i2c_info *i2c_info;

	if (!a_ctrl || !cmd_buf || (len < sizeof(struct cam_cmd_i2c_info))) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	i2c_info = (struct cam_cmd_i2c_info *)cmd_buf;
	if (a_ctrl->io_master_info.master_type == CCI_MASTER) {
		a_ctrl->io_master_info.cci_client->cci_i2c_master =
			a_ctrl->cci_i2c_master;
		a_ctrl->io_master_info.cci_client->i2c_freq_mode =
			i2c_info->i2c_freq_mode;
		a_ctrl->io_master_info.cci_client->sid =
			i2c_info->slave_addr >> 1;
		CAM_DBG(CAM_ACTUATOR, "Slave addr: 0x%x Freq Mode: %d",
			i2c_info->slave_addr, i2c_info->i2c_freq_mode);
	} else if (a_ctrl->io_master_info.master_type == I2C_MASTER) {
		a_ctrl->io_master_info.client->addr = i2c_info->slave_addr;
		CAM_DBG(CAM_ACTUATOR, "Slave addr: 0x%x", i2c_info->slave_addr);
	} else {
		CAM_ERR(CAM_ACTUATOR, "Invalid Master type: %d",
			a_ctrl->io_master_info.master_type);
		 rc = -EINVAL;
	}

	return rc;
}

int32_t cam_actuator_apply_settings(struct cam_actuator_ctrl_t *a_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;

	if (a_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_ACTUATOR, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		rc = cam_actuator_i2c_modes_util(
			&(a_ctrl->io_master_info),
			i2c_list);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed to apply settings: %d",
				rc);
		} else {
			CAM_DBG(CAM_ACTUATOR,
				"Success:request ID: %d",
				i2c_set->request_id);
		}
	}

	return rc;
}

int32_t cam_actuator_apply_request(struct cam_req_mgr_apply_request *apply)
{
	int32_t rc = 0, request_id, del_req_id;
	struct cam_actuator_ctrl_t *a_ctrl = NULL;

	if (!apply) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Input Args");
		return -EINVAL;
	}

	a_ctrl = (struct cam_actuator_ctrl_t *)
		cam_get_device_priv(apply->dev_hdl);
	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "Device data is NULL");
		return -EINVAL;
	}
	request_id = apply->request_id % MAX_PER_FRAME_ARRAY;

	trace_cam_apply_req("Actuator", apply->request_id);

	CAM_DBG(CAM_ACTUATOR, "Request Id: %lld", apply->request_id);
	mutex_lock(&(a_ctrl->actuator_mutex));
	if ((apply->request_id ==
		a_ctrl->i2c_data.per_frame[request_id].request_id) &&
		(a_ctrl->i2c_data.per_frame[request_id].is_settings_valid)
		== 1) {
		rc = cam_actuator_apply_settings(a_ctrl,
			&a_ctrl->i2c_data.per_frame[request_id]);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed in applying the request: %lld\n",
				apply->request_id);
			goto release_mutex;
		}
	}
	del_req_id = (request_id +
		MAX_PER_FRAME_ARRAY - MAX_SYSTEM_PIPELINE_DELAY) %
		MAX_PER_FRAME_ARRAY;

	if (apply->request_id >
		a_ctrl->i2c_data.per_frame[del_req_id].request_id) {
		a_ctrl->i2c_data.per_frame[del_req_id].request_id = 0;
		rc = delete_request(&a_ctrl->i2c_data.per_frame[del_req_id]);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Fail deleting the req: %d err: %d\n",
				del_req_id, rc);
			goto release_mutex;
		}
	} else {
		CAM_DBG(CAM_ACTUATOR, "No Valid Req to clean Up");
	}

release_mutex:
	mutex_unlock(&(a_ctrl->actuator_mutex));
	return rc;
}

int32_t cam_actuator_establish_link(
	struct cam_req_mgr_core_dev_link_setup *link)
{
	struct cam_actuator_ctrl_t *a_ctrl = NULL;

	if (!link) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	a_ctrl = (struct cam_actuator_ctrl_t *)
		cam_get_device_priv(link->dev_hdl);
	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "Device data is NULL");
		return -EINVAL;
	}

	mutex_lock(&(a_ctrl->actuator_mutex));
	if (link->link_enable) {
		a_ctrl->bridge_intf.link_hdl = link->link_hdl;
		a_ctrl->bridge_intf.crm_cb = link->crm_cb;
	} else {
		a_ctrl->bridge_intf.link_hdl = -1;
		a_ctrl->bridge_intf.crm_cb = NULL;
	}
	mutex_unlock(&(a_ctrl->actuator_mutex));

	return 0;
}

static void cam_actuator_update_req_mgr(
	struct cam_actuator_ctrl_t *a_ctrl,
	struct cam_packet *csl_packet)
{
	struct cam_req_mgr_add_request add_req;

	add_req.link_hdl = a_ctrl->bridge_intf.link_hdl;
	add_req.req_id = csl_packet->header.request_id;
	add_req.dev_hdl = a_ctrl->bridge_intf.device_hdl;
	add_req.skip_before_applying = 0;

	if (a_ctrl->bridge_intf.crm_cb &&
		a_ctrl->bridge_intf.crm_cb->add_req) {
		a_ctrl->bridge_intf.crm_cb->add_req(&add_req);
		CAM_DBG(CAM_ACTUATOR, "Request Id: %lld added to CRM",
			add_req.req_id);
	} else {
		CAM_ERR(CAM_ACTUATOR, "Can't add Request ID: %lld to CRM",
			csl_packet->header.request_id);
	}
}

int32_t cam_actuator_publish_dev_info(struct cam_req_mgr_device_info *info)
{
	if (!info) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	info->dev_id = CAM_REQ_MGR_DEVICE_ACTUATOR;
	strlcpy(info->name, CAM_ACTUATOR_NAME, sizeof(info->name));
	info->p_delay = 1;
	info->trigger = CAM_TRIGGER_POINT_SOF;

	return 0;
}

int32_t cam_actuator_i2c_pkt_parse(struct cam_actuator_ctrl_t *a_ctrl,
	void *arg)
{
	int32_t  rc = 0;
	int32_t  i = 0;
	uint32_t total_cmd_buf_in_bytes = 0;
	size_t   len_of_buff = 0;
	size_t   remain_len = 0;
	uint32_t *offset = NULL;
	uint32_t *cmd_buf = NULL;
	uintptr_t generic_ptr;
	uintptr_t generic_pkt_ptr;
	struct common_header      *cmm_hdr = NULL;
	struct cam_control        *ioctl_ctrl = NULL;
	struct cam_packet         *csl_packet = NULL;
	struct cam_config_dev_cmd config;
	struct i2c_data_settings  *i2c_data = NULL;
	struct i2c_settings_array *i2c_reg_settings = NULL;
	struct cam_cmd_buf_desc   *cmd_desc = NULL;
	struct cam_actuator_soc_private *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;

	if (!a_ctrl || !arg) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;

	power_info = &soc_private->power_info;

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(config.packet_handle,
		&generic_pkt_ptr, &len_of_buff);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "Error in converting command Handle %d",
			rc);
		return rc;
	}

	remain_len = len_of_buff;
	if ((sizeof(struct cam_packet) > len_of_buff) ||
		((size_t)config.offset >= len_of_buff -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_ACTUATOR,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), len_of_buff);
		rc = -EINVAL;
		goto rel_pkt_buf;
	}

	remain_len -= (size_t)config.offset;
	csl_packet = (struct cam_packet *)
			(generic_pkt_ptr + (uint32_t)config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_ACTUATOR, "Invalid packet params");
		rc = -EINVAL;
		goto rel_pkt_buf;
	}

	CAM_DBG(CAM_ACTUATOR, "Pkt opcode: %d",	csl_packet->header.op_code);

	if ((csl_packet->header.op_code & 0xFFFFFF) !=
		CAM_ACTUATOR_PACKET_OPCODE_INIT &&
		csl_packet->header.request_id <= a_ctrl->last_flush_req
		&& a_ctrl->last_flush_req != 0) {
		CAM_DBG(CAM_ACTUATOR,
			"reject request %lld, last request to flush %lld",
			csl_packet->header.request_id, a_ctrl->last_flush_req);
		rc = -EINVAL;
		goto rel_pkt_buf;
	}

	if (csl_packet->header.request_id > a_ctrl->last_flush_req)
		a_ctrl->last_flush_req = 0;

	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_ACTUATOR_PACKET_OPCODE_INIT:
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
				CAM_ERR(CAM_ACTUATOR, "Failed to get cpu buf");
				goto rel_pkt_buf;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_ACTUATOR, "invalid cmd buf");
				goto rel_cmd_buf;
			}
			if ((len_of_buff < sizeof(struct common_header)) ||
				(cmd_desc[i].offset > (len_of_buff -
				sizeof(struct common_header)))) {
				CAM_ERR(CAM_ACTUATOR,
					"Invalid length for sensor cmd");
				rc = -EINVAL;
				goto rel_cmd_buf;
			}
			remain_len = len_of_buff - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmm_hdr = (struct common_header *)cmd_buf;

			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				CAM_DBG(CAM_ACTUATOR,
					"Received slave info buffer");
				rc = cam_actuator_slaveInfo_pkt_parser(
					a_ctrl, cmd_buf, remain_len);
				if (rc < 0) {
					CAM_ERR(CAM_ACTUATOR,
					"Failed to parse slave info: %d", rc);
					goto rel_cmd_buf;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_ACTUATOR,
					"Received power settings buffer");
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					power_info, remain_len);
				if (rc) {
					CAM_ERR(CAM_ACTUATOR,
					"Failed:parse power settings: %d",
					rc);
					goto rel_cmd_buf;
				}
				break;
			default:
				CAM_DBG(CAM_ACTUATOR,
					"Received initSettings buffer");
				i2c_data = &(a_ctrl->i2c_data);
				i2c_reg_settings =
					&i2c_data->init_settings;

				i2c_reg_settings->request_id = 0;
				i2c_reg_settings->is_settings_valid = 1;
				rc = cam_sensor_i2c_command_parser(
					&a_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1);
				if (rc < 0) {
					CAM_ERR(CAM_ACTUATOR,
					"Failed:parse init settings: %d",
					rc);
					goto rel_cmd_buf;
				}
				break;
			}
			if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
				CAM_WARN(CAM_ACTUATOR,
					"Failed to put cpu buf: 0x%x",
					cmd_desc[i].mem_handle);
		}

		if (a_ctrl->cam_act_state == CAM_ACTUATOR_ACQUIRE) {
			rc = cam_actuator_power_up(a_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR,
					" Actuator Power up failed");
				goto rel_pkt_buf;
			}
			a_ctrl->cam_act_state = CAM_ACTUATOR_CONFIG;
		}

		rc = cam_actuator_apply_settings(a_ctrl,
			&a_ctrl->i2c_data.init_settings);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "Cannot apply Init settings");
			goto rel_pkt_buf;
		}

		/* Delete the request even if the apply is failed */
		rc = delete_request(&a_ctrl->i2c_data.init_settings);
		if (rc < 0) {
			CAM_WARN(CAM_ACTUATOR,
				"Fail in deleting the Init settings");
			rc = 0;
		}
		break;
	case CAM_ACTUATOR_PACKET_AUTO_MOVE_LENS:
		if (a_ctrl->cam_act_state < CAM_ACTUATOR_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_ACTUATOR,
				"Not in right state to move lens: %d",
				a_ctrl->cam_act_state);
			goto rel_pkt_buf;
		}
		a_ctrl->setting_apply_state = ACT_APPLY_SETTINGS_NOW;

		i2c_data = &(a_ctrl->i2c_data);
		i2c_reg_settings = &i2c_data->init_settings;

		i2c_data->init_settings.request_id =
			csl_packet->header.request_id;
		i2c_reg_settings->is_settings_valid = 1;
		offset = (uint32_t *)&csl_packet->payload;
		offset += csl_packet->cmd_buf_offset / sizeof(uint32_t);
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		rc = cam_sensor_i2c_command_parser(
			&a_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Auto move lens parsing failed: %d", rc);
			goto rel_pkt_buf;
		}
		cam_actuator_update_req_mgr(a_ctrl, csl_packet);
		break;
	case CAM_ACTUATOR_PACKET_MANUAL_MOVE_LENS:
		if (a_ctrl->cam_act_state < CAM_ACTUATOR_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_ACTUATOR,
				"Not in right state to move lens: %d",
				a_ctrl->cam_act_state);
			goto rel_pkt_buf;
		}

		a_ctrl->setting_apply_state = ACT_APPLY_SETTINGS_LATER;
		i2c_data = &(a_ctrl->i2c_data);
		i2c_reg_settings = &i2c_data->per_frame[
			csl_packet->header.request_id % MAX_PER_FRAME_ARRAY];

		 i2c_reg_settings->request_id =
			csl_packet->header.request_id;
		i2c_reg_settings->is_settings_valid = 1;
		offset = (uint32_t *)&csl_packet->payload;
		offset += csl_packet->cmd_buf_offset / sizeof(uint32_t);
		cmd_desc = (struct cam_cmd_buf_desc *)(offset);
		rc = cam_sensor_i2c_command_parser(
			&a_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Manual move lens parsing failed: %d", rc);
			goto rel_pkt_buf;
		}

		cam_actuator_update_req_mgr(a_ctrl, csl_packet);
		break;
	case CAM_PKT_NOP_OPCODE:
		if (a_ctrl->cam_act_state < CAM_ACTUATOR_CONFIG) {
			CAM_WARN(CAM_ACTUATOR,
				"Received NOP packets in invalid state: %d",
				a_ctrl->cam_act_state);
			goto rel_pkt_buf;
		}
		cam_actuator_update_req_mgr(a_ctrl, csl_packet);
		break;
	default:
		CAM_ERR(CAM_ACTUATOR, "Wrong Opcode: %d",
			csl_packet->header.op_code & 0xFFFFFF);
		rc = -EINVAL;
		goto rel_pkt_buf;
	}

	if (cam_mem_put_cpu_buf(config.packet_handle))
		CAM_WARN(CAM_ACTUATOR, "Fail to put cmd buffer: 0x%x",
			config.packet_handle);

	return rc;

rel_cmd_buf:
	if (cam_mem_put_cpu_buf(cmd_desc[i].mem_handle))
		CAM_WARN(CAM_ACTUATOR, "Fail to put cmd buffer: 0x%x",
			cmd_desc[i].mem_handle);
rel_pkt_buf:
	if (cam_mem_put_cpu_buf(config.packet_handle))
		CAM_WARN(CAM_ACTUATOR, "Fail to put cmd buffer: 0x%x",
			config.packet_handle);

	return rc;
}

void cam_actuator_shutdown(struct cam_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;
	struct cam_actuator_soc_private  *soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info =
		&soc_private->power_info;

	if (a_ctrl->cam_act_state == CAM_ACTUATOR_INIT)
		return;

	if (a_ctrl->cam_act_state >= CAM_ACTUATOR_CONFIG) {
		rc = cam_actuator_power_down(a_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_ACTUATOR, "Actuator Power down failed");
		a_ctrl->cam_act_state = CAM_ACTUATOR_ACQUIRE;
	}

	if (a_ctrl->cam_act_state >= CAM_ACTUATOR_ACQUIRE) {
		rc = cam_destroy_device_hdl(a_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_ACTUATOR, "destroying  dhdl failed");
		a_ctrl->bridge_intf.device_hdl = -1;
		a_ctrl->bridge_intf.link_hdl = -1;
		a_ctrl->bridge_intf.session_hdl = -1;
	}

	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_setting_size = 0;
	power_info->power_down_setting_size = 0;

	a_ctrl->cam_act_state = CAM_ACTUATOR_INIT;
}

int32_t cam_actuator_driver_cmd(struct cam_actuator_ctrl_t *a_ctrl,
	void *arg)
{
	int rc = 0;
	struct cam_control *cmd = (struct cam_control *)arg;
	struct cam_actuator_soc_private *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;

	if (!a_ctrl || !cmd) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;

	power_info = &soc_private->power_info;

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_ACTUATOR, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	CAM_DBG(CAM_ACTUATOR, "Opcode to Actuator: %d", cmd->op_code);

	mutex_lock(&(a_ctrl->actuator_mutex));
	switch (cmd->op_code) {
	case CAM_ACQUIRE_DEV: {
		struct cam_sensor_acquire_dev actuator_acq_dev;
		struct cam_create_dev_hdl bridge_params;

		if (a_ctrl->bridge_intf.device_hdl != -1) {
			CAM_ERR(CAM_ACTUATOR, "Device is already acquired");
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = copy_from_user(&actuator_acq_dev,
			u64_to_user_ptr(cmd->handle),
			sizeof(actuator_acq_dev));
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "Failed Copying from user\n");
			goto release_mutex;
		}

		bridge_params.session_hdl = actuator_acq_dev.session_handle;
		bridge_params.ops = &a_ctrl->bridge_intf.ops;
		bridge_params.v4l2_sub_dev_flag = 0;
		bridge_params.media_entity_flag = 0;
		bridge_params.priv = a_ctrl;
		bridge_params.dev_id = CAM_ACTUATOR;
		actuator_acq_dev.device_handle =
			cam_create_device_hdl(&bridge_params);
		a_ctrl->bridge_intf.device_hdl = actuator_acq_dev.device_handle;
		a_ctrl->bridge_intf.session_hdl =
			actuator_acq_dev.session_handle;

		CAM_DBG(CAM_ACTUATOR, "Device Handle: %d",
			actuator_acq_dev.device_handle);
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&actuator_acq_dev,
			sizeof(struct cam_sensor_acquire_dev))) {
			CAM_ERR(CAM_ACTUATOR, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}

		a_ctrl->cam_act_state = CAM_ACTUATOR_ACQUIRE;
	}
		break;
	case CAM_RELEASE_DEV: {
		if (a_ctrl->cam_act_state == CAM_ACTUATOR_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_ACTUATOR,
				"Cant release actuator: in start state");
			goto release_mutex;
		}

		if (a_ctrl->cam_act_state == CAM_ACTUATOR_CONFIG) {
			rc = cam_actuator_power_down(a_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR,
					"Actuator Power down failed");
				goto release_mutex;
			}
		}

		if (a_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_ACTUATOR, "link hdl: %d device hdl: %d",
				a_ctrl->bridge_intf.device_hdl,
				a_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}

		if (a_ctrl->bridge_intf.link_hdl != -1) {
			CAM_ERR(CAM_ACTUATOR,
				"Device [%d] still active on link 0x%x",
				a_ctrl->cam_act_state,
				a_ctrl->bridge_intf.link_hdl);
			rc = -EAGAIN;
			goto release_mutex;
		}

		rc = cam_destroy_device_hdl(a_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_ACTUATOR, "destroying the device hdl");
		a_ctrl->bridge_intf.device_hdl = -1;
		a_ctrl->bridge_intf.link_hdl = -1;
		a_ctrl->bridge_intf.session_hdl = -1;
		a_ctrl->cam_act_state = CAM_ACTUATOR_INIT;
		a_ctrl->last_flush_req = 0;
		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_down_setting_size = 0;
		power_info->power_setting_size = 0;
	}
		break;
	case CAM_QUERY_CAP: {
		struct cam_actuator_query_cap actuator_cap = {0};

		actuator_cap.slot_info = a_ctrl->soc_info.index;
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&actuator_cap,
			sizeof(struct cam_actuator_query_cap))) {
			CAM_ERR(CAM_ACTUATOR, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
	}
		break;
	case CAM_START_DEV: {
		if (a_ctrl->cam_act_state != CAM_ACTUATOR_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_ACTUATOR,
			"Not in right state to start : %d",
			a_ctrl->cam_act_state);
			goto release_mutex;
		}
		a_ctrl->cam_act_state = CAM_ACTUATOR_START;
		a_ctrl->last_flush_req = 0;
	}
		break;
	case CAM_STOP_DEV: {
		struct i2c_settings_array *i2c_set = NULL;
		int i;

		if (a_ctrl->cam_act_state != CAM_ACTUATOR_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_ACTUATOR,
			"Not in right state to stop : %d",
			a_ctrl->cam_act_state);
			goto release_mutex;
		}

		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			i2c_set = &(a_ctrl->i2c_data.per_frame[i]);

			if (i2c_set->is_settings_valid == 1) {
				rc = delete_request(i2c_set);
				if (rc < 0)
					CAM_ERR(CAM_SENSOR,
						"delete request: %lld rc: %d",
						i2c_set->request_id, rc);
			}
		}
		a_ctrl->last_flush_req = 0;
		a_ctrl->cam_act_state = CAM_ACTUATOR_CONFIG;
	}
		break;
	case CAM_CONFIG_DEV: {
		a_ctrl->setting_apply_state =
			ACT_APPLY_SETTINGS_LATER;
		rc = cam_actuator_i2c_pkt_parse(a_ctrl, arg);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "Failed in actuator Parsing");
			goto release_mutex;
		}

		if (a_ctrl->setting_apply_state ==
			ACT_APPLY_SETTINGS_NOW) {
			rc = cam_actuator_apply_settings(a_ctrl,
				&a_ctrl->i2c_data.init_settings);
			if (rc < 0)
				CAM_ERR(CAM_ACTUATOR,
					"Cannot apply Update settings");

			/* Delete the request even if the apply is failed */
			rc = delete_request(&a_ctrl->i2c_data.init_settings);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR,
					"Failed in Deleting the Init Pkt: %d",
					rc);
				goto release_mutex;
			}
		}
	}
		break;
	default:
		CAM_ERR(CAM_ACTUATOR, "Invalid Opcode %d", cmd->op_code);
	}

release_mutex:
	mutex_unlock(&(a_ctrl->actuator_mutex));

	return rc;
}

int32_t cam_actuator_flush_request(struct cam_req_mgr_flush_request *flush_req)
{
	int32_t rc = 0, i;
	uint32_t cancel_req_id_found = 0;
	struct cam_actuator_ctrl_t *a_ctrl = NULL;
	struct i2c_settings_array *i2c_set = NULL;

	if (!flush_req)
		return -EINVAL;

	a_ctrl = (struct cam_actuator_ctrl_t *)
		cam_get_device_priv(flush_req->dev_hdl);
	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "Device data is NULL");
		return -EINVAL;
	}

	if (a_ctrl->i2c_data.per_frame == NULL) {
		CAM_ERR(CAM_ACTUATOR, "i2c frame data is NULL");
		return -EINVAL;
	}

	mutex_lock(&(a_ctrl->actuator_mutex));
	if (flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_ALL) {
		a_ctrl->last_flush_req = flush_req->req_id;
		CAM_DBG(CAM_ACTUATOR, "last reqest to flush is %lld",
			flush_req->req_id);
	}

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
		i2c_set = &(a_ctrl->i2c_data.per_frame[i]);

		if ((flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ)
				&& (i2c_set->request_id != flush_req->req_id))
			continue;

		if (i2c_set->is_settings_valid == 1) {
			rc = delete_request(i2c_set);
			if (rc < 0)
				CAM_ERR(CAM_ACTUATOR,
					"delete request: %lld rc: %d",
					i2c_set->request_id, rc);

			if (flush_req->type ==
				CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ) {
				cancel_req_id_found = 1;
				break;
			}
		}
	}

	if (flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ &&
		!cancel_req_id_found)
		CAM_DBG(CAM_ACTUATOR,
			"Flush request id:%lld not found in the pending list",
			flush_req->req_id);
	mutex_unlock(&(a_ctrl->actuator_mutex));
	return rc;
}
