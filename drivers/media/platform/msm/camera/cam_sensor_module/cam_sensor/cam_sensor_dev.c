/* Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.
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

#include "cam_sensor_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_sensor_soc.h"
#include "cam_sensor_core.h"

struct cam_sensor_i2c_reg_setting_array {
	struct cam_sensor_i2c_reg_array reg_setting[1024];
	unsigned short size;
	enum camera_sensor_i2c_type addr_type;
	enum camera_sensor_i2c_type data_type;
	unsigned short delay;
};

struct cam_sensor_settings {
    struct cam_sensor_i2c_reg_setting_array imx586_setting0;
    struct cam_sensor_i2c_reg_setting_array imx586_setting1;
    struct cam_sensor_i2c_reg_setting_array streamoff;
    struct cam_sensor_i2c_reg_setting_array s5k3m5_setting;
    struct cam_sensor_i2c_reg_setting_array imx471_setting;
    struct cam_sensor_i2c_reg_setting_array imx481_setting;
	struct cam_sensor_i2c_reg_setting_array gc5035_setting;
};

struct cam_sensor_settings sensor_settings = {
#include "CAM_SENSOR_SETTINGS.h"
};
static bool is_ftm_current_test = false;
static struct wakeup_source *cam_clk_ws;
#define CAM_CLK_WS_TIMEOUT 2000

void cam_request_timeout_ws(void)
{
	__pm_wakeup_event(cam_clk_ws, CAM_CLK_WS_TIMEOUT);
}

static long cam_sensor_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int rc = 0;
	struct cam_sensor_ctrl_t *s_ctrl =
		v4l2_get_subdevdata(sd);
/* Add for AT camera test */
	struct cam_sensor_i2c_reg_setting sensor_setting;
	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_sensor_driver_cmd(s_ctrl, arg);
		break;
/* Add for AT camera test */
	case VIDIOC_CAM_FTM_POWNER_DOWN:
		CAM_ERR(CAM_SENSOR, "FTM stream off");
        if (s_ctrl->sensordata->slave_info.sensor_id == 0x586
                ||s_ctrl->sensordata->slave_info.sensor_id == 0x30d5
                ||s_ctrl->sensordata->slave_info.sensor_id == 0x471
                ||s_ctrl->sensordata->slave_info.sensor_id == 0x481
                ||s_ctrl->sensordata->slave_info.sensor_id == 0x5035) {
			          sensor_setting.reg_setting = sensor_settings.streamoff.reg_setting;
			          sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
			          sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
			          sensor_setting.size = sensor_settings.streamoff.size;
			          sensor_setting.delay = sensor_settings.streamoff.delay;
            rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);

            if (rc < 0) {
    			/* If the I2C reg write failed for the first section reg, send
                the result instead of keeping writing the next section of reg. */
                CAM_ERR(CAM_SENSOR, "FTM Failed to stream off setting,rc=%d.",rc);
            } else {
                    CAM_ERR(CAM_SENSOR, "FTM successfully to stream off");
            }
        }
		rc = cam_sensor_power_down(s_ctrl);
        CAM_ERR(CAM_SENSOR, "FTM power down.rc=%d",rc);
		break;
	case VIDIOC_CAM_FTM_POWNER_UP:
		rc = cam_sensor_power_up(s_ctrl);
		CAM_ERR(CAM_SENSOR, "FTM power up sensor id 0x%x,result %d",s_ctrl->sensordata->slave_info.sensor_id,rc);
		if (rc < 0) {
			CAM_ERR(CAM_SENSOR, "FTM power up failed!");
			break;
		}
        is_ftm_current_test = true;
		if (s_ctrl->sensordata->slave_info.sensor_id == 0x586) {
			CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
			sensor_setting.reg_setting = sensor_settings.imx586_setting0.reg_setting;
			sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
			sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
			sensor_setting.size = sensor_settings.imx586_setting0.size;
			sensor_setting.delay = sensor_settings.imx586_setting0.delay;

            rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);

            if (rc < 0) {
    			/* If the I2C reg write failed for the first section reg, send
                the result instead of keeping writing the next section of reg. */
                CAM_ERR(CAM_SENSOR, "FTM Failed to write sensor setting 1/2");
				break;
            } else {
                    CAM_ERR(CAM_SENSOR, "FTM successfully to write sensor setting 1/2");
            }

            sensor_setting.reg_setting = sensor_settings.imx586_setting1.reg_setting;
            sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
            sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
            sensor_setting.size = sensor_settings.imx586_setting1.size;
            sensor_setting.delay = sensor_settings.imx586_setting1.delay;

            rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);

            if (rc < 0) {
                    CAM_ERR(CAM_SENSOR, "FTM Failed to write sensor setting 2/2");
            } else {
                    CAM_ERR(CAM_SENSOR, "FTM successfully to write sensor setting 2/2");
            }
		} else {
			if (s_ctrl->sensordata->slave_info.sensor_id == 0x30d5) {
				CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
				sensor_setting.reg_setting = sensor_settings.s5k3m5_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.size = sensor_settings.s5k3m5_setting.size;
				sensor_setting.delay = sensor_settings.s5k3m5_setting.delay;
                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x5035) {
				CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
				sensor_setting.reg_setting = sensor_settings.gc5035_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.gc5035_setting.size;
				sensor_setting.delay = sensor_settings.gc5035_setting.delay;
                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
			}else if (s_ctrl->sensordata->slave_info.sensor_id == 0x471) {
				CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
				sensor_setting.reg_setting = sensor_settings.imx471_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.imx471_setting.size;
				sensor_setting.delay = sensor_settings.imx471_setting.delay;
                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
			} else if (s_ctrl->sensordata->slave_info.sensor_id == 0x481) {
				CAM_ERR(CAM_SENSOR, "FTM sensor setting 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
				sensor_setting.reg_setting = sensor_settings.imx481_setting.reg_setting;
				sensor_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
				sensor_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
				sensor_setting.size = sensor_settings.imx481_setting.size;
				sensor_setting.delay = sensor_settings.imx481_setting.delay;
                rc = camera_io_dev_write(&(s_ctrl->io_master_info), &sensor_setting);
			}else{
                CAM_ERR(CAM_SENSOR, "FTM unknown sensor id 0x%x",s_ctrl->sensordata->slave_info.sensor_id);
                rc = -1;
            }

			if (rc < 0) {
				CAM_ERR(CAM_SENSOR, "FTM Failed to write sensor setting");
			} else {
				CAM_ERR(CAM_SENSOR, "FTM successfully to write sensor setting");
			}
		}
		break;
	default:
		CAM_ERR(CAM_SENSOR, "Invalid ioctl cmd: %d", cmd);
		rc = -EINVAL;
		break;
	}
	return rc;
}

static int cam_sensor_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_sensor_ctrl_t *s_ctrl =
		v4l2_get_subdevdata(sd);

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "s_ctrl ptr is NULL");
		return -EINVAL;
	}

	mutex_lock(&(s_ctrl->cam_sensor_mutex));
    if(!is_ftm_current_test)
    	cam_sensor_shutdown(s_ctrl);
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));

	return 0;
}

#ifdef CONFIG_COMPAT
static long cam_sensor_init_subdev_do_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct cam_control cmd_data;
	int32_t rc = 0;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_SENSOR, "Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_sensor_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc < 0)
			CAM_ERR(CAM_SENSOR, "cam_sensor_subdev_ioctl failed");
			break;
	default:
		CAM_ERR(CAM_SENSOR, "Invalid compat ioctl cmd_type: %d", cmd);
		rc = -EINVAL;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_SENSOR,
				"Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}

	return rc;
}

#endif
static struct v4l2_subdev_core_ops cam_sensor_subdev_core_ops = {
	.ioctl = cam_sensor_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_sensor_init_subdev_do_ioctl,
#endif
	.s_power = cam_sensor_power,
};

static struct v4l2_subdev_ops cam_sensor_subdev_ops = {
	.core = &cam_sensor_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops cam_sensor_internal_ops = {
	.close = cam_sensor_subdev_close,
};

static int cam_sensor_init_subdev_params(struct cam_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;

	s_ctrl->v4l2_dev_str.internal_ops =
		&cam_sensor_internal_ops;
	s_ctrl->v4l2_dev_str.ops =
		&cam_sensor_subdev_ops;
	strlcpy(s_ctrl->device_name, CAMX_SENSOR_DEV_NAME,
		sizeof(s_ctrl->device_name));
	s_ctrl->v4l2_dev_str.name =
		s_ctrl->device_name;
	s_ctrl->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	s_ctrl->v4l2_dev_str.ent_function =
		CAM_SENSOR_DEVICE_TYPE;
	s_ctrl->v4l2_dev_str.token = s_ctrl;

	rc = cam_register_subdev(&(s_ctrl->v4l2_dev_str));
	if (rc)
		CAM_ERR(CAM_SENSOR, "Fail with cam_register_subdev rc: %d", rc);

	return rc;
}

static int32_t cam_sensor_driver_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int32_t rc = 0;
	int i = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;
	struct cam_hw_soc_info   *soc_info = NULL;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CAM_ERR(CAM_SENSOR,
			"%s :i2c_check_functionality failed", client->name);
		return -EFAULT;
	}

	/* Create sensor control structure */
	s_ctrl = kzalloc(sizeof(*s_ctrl), GFP_KERNEL);
	if (!s_ctrl)
		return -ENOMEM;

	i2c_set_clientdata(client, s_ctrl);

	s_ctrl->io_master_info.client = client;
	soc_info = &s_ctrl->soc_info;
	soc_info->dev = &client->dev;
	soc_info->dev_name = client->name;

	/* Initialize sensor device type */
	s_ctrl->of_node = client->dev.of_node;
	s_ctrl->io_master_info.master_type = I2C_MASTER;
	s_ctrl->is_probe_succeed = 0;
	s_ctrl->last_flush_req = 0;

	rc = cam_sensor_parse_dt(s_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "cam_sensor_parse_dt rc %d", rc);
		goto free_s_ctrl;
	}

	rc = cam_sensor_init_subdev_params(s_ctrl);
	if (rc)
		goto free_s_ctrl;

	s_ctrl->i2c_data.per_frame =
		(struct i2c_settings_array *)
		kzalloc(sizeof(struct i2c_settings_array) *
		MAX_PER_FRAME_ARRAY, GFP_KERNEL);
	if (s_ctrl->i2c_data.per_frame == NULL) {
		rc = -ENOMEM;
		goto unreg_subdev;
	}

	INIT_LIST_HEAD(&(s_ctrl->i2c_data.init_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.config_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamon_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamoff_settings.list_head));

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++)
		INIT_LIST_HEAD(&(s_ctrl->i2c_data.per_frame[i].list_head));

	s_ctrl->bridge_intf.device_hdl = -1;
	s_ctrl->bridge_intf.link_hdl = -1;
	s_ctrl->bridge_intf.ops.get_dev_info = cam_sensor_publish_dev_info;
	s_ctrl->bridge_intf.ops.link_setup = cam_sensor_establish_link;
	s_ctrl->bridge_intf.ops.apply_req = cam_sensor_apply_request;
	s_ctrl->bridge_intf.ops.flush_req = cam_sensor_flush_request;

	s_ctrl->sensordata->power_info.dev = soc_info->dev;

	return rc;
unreg_subdev:
	cam_unregister_subdev(&(s_ctrl->v4l2_dev_str));
free_s_ctrl:
	kfree(s_ctrl);
	return rc;
}

static int cam_sensor_platform_remove(struct platform_device *pdev)
{
	int                        i;
	struct cam_sensor_ctrl_t  *s_ctrl;
	struct cam_hw_soc_info    *soc_info;

	s_ctrl = platform_get_drvdata(pdev);
	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "sensor device is NULL");
		return 0;
	}

	CAM_INFO(CAM_SENSOR, "platform remove invoked");
	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	cam_sensor_shutdown(s_ctrl);
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	cam_unregister_subdev(&(s_ctrl->v4l2_dev_str));
	soc_info = &s_ctrl->soc_info;
	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	kfree(s_ctrl->i2c_data.per_frame);
	platform_set_drvdata(pdev, NULL);
	v4l2_set_subdevdata(&(s_ctrl->v4l2_dev_str.sd), NULL);
	devm_kfree(&pdev->dev, s_ctrl);

	return 0;
}

static int cam_sensor_driver_i2c_remove(struct i2c_client *client)
{
	int                        i;
	struct cam_sensor_ctrl_t  *s_ctrl = i2c_get_clientdata(client);
	struct cam_hw_soc_info    *soc_info;

	if (!s_ctrl) {
		CAM_ERR(CAM_SENSOR, "sensor device is NULL");
		return 0;
	}

	CAM_INFO(CAM_SENSOR, "i2c remove invoked");
	mutex_lock(&(s_ctrl->cam_sensor_mutex));
	cam_sensor_shutdown(s_ctrl);
	mutex_unlock(&(s_ctrl->cam_sensor_mutex));
	cam_unregister_subdev(&(s_ctrl->v4l2_dev_str));
	soc_info = &s_ctrl->soc_info;
	for (i = 0; i < soc_info->num_clk; i++)
		devm_clk_put(soc_info->dev, soc_info->clk[i]);

	kfree(s_ctrl->i2c_data.per_frame);
	v4l2_set_subdevdata(&(s_ctrl->v4l2_dev_str.sd), NULL);
	kfree(s_ctrl);

	return 0;
}

static const struct of_device_id cam_sensor_driver_dt_match[] = {
	{.compatible = "qcom,cam-sensor"},
	{}
};

static int32_t cam_sensor_driver_platform_probe(
	struct platform_device *pdev)
{
	int32_t rc = 0, i = 0;
	struct cam_sensor_ctrl_t *s_ctrl = NULL;
	struct cam_hw_soc_info *soc_info = NULL;
	static bool cam_clk_ws_flag = true;

	/* Create sensor control structure */
	s_ctrl = devm_kzalloc(&pdev->dev,
		sizeof(struct cam_sensor_ctrl_t), GFP_KERNEL);
	if (!s_ctrl)
		return -ENOMEM;

	soc_info = &s_ctrl->soc_info;
	soc_info->pdev = pdev;
	soc_info->dev = &pdev->dev;
	soc_info->dev_name = pdev->name;

	/* Initialize sensor device type */
	s_ctrl->of_node = pdev->dev.of_node;
	s_ctrl->is_probe_succeed = 0;
	s_ctrl->last_flush_req = 0;

	/*fill in platform device*/
	s_ctrl->pdev = pdev;

	s_ctrl->io_master_info.master_type = CCI_MASTER;

	rc = cam_sensor_parse_dt(s_ctrl);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "failed: cam_sensor_parse_dt rc %d", rc);
		goto free_s_ctrl;
	}

	/* Fill platform device id*/
	pdev->id = soc_info->index;

	rc = cam_sensor_init_subdev_params(s_ctrl);
	if (rc)
		goto free_s_ctrl;

	s_ctrl->i2c_data.per_frame =
		(struct i2c_settings_array *)
		kzalloc(sizeof(struct i2c_settings_array) *
		MAX_PER_FRAME_ARRAY, GFP_KERNEL);
	if (s_ctrl->i2c_data.per_frame == NULL) {
		rc = -ENOMEM;
		goto unreg_subdev;
	}

	INIT_LIST_HEAD(&(s_ctrl->i2c_data.init_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.config_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamon_settings.list_head));
	INIT_LIST_HEAD(&(s_ctrl->i2c_data.streamoff_settings.list_head));

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++)
		INIT_LIST_HEAD(&(s_ctrl->i2c_data.per_frame[i].list_head));

	s_ctrl->bridge_intf.device_hdl = -1;
	s_ctrl->bridge_intf.link_hdl = -1;
	s_ctrl->bridge_intf.ops.get_dev_info = cam_sensor_publish_dev_info;
	s_ctrl->bridge_intf.ops.link_setup = cam_sensor_establish_link;
	s_ctrl->bridge_intf.ops.apply_req = cam_sensor_apply_request;
	s_ctrl->bridge_intf.ops.flush_req = cam_sensor_flush_request;

	s_ctrl->sensordata->power_info.dev = &pdev->dev;
	platform_set_drvdata(pdev, s_ctrl);
	s_ctrl->sensor_state = CAM_SENSOR_INIT;

	if (cam_clk_ws_flag) {
		cam_clk_ws = wakeup_source_register("cam_clk");
		cam_clk_ws_flag = false;
	}

	return rc;
unreg_subdev:
	cam_unregister_subdev(&(s_ctrl->v4l2_dev_str));
free_s_ctrl:
	devm_kfree(&pdev->dev, s_ctrl);
	return rc;
}

MODULE_DEVICE_TABLE(of, cam_sensor_driver_dt_match);

static struct platform_driver cam_sensor_platform_driver = {
	.probe = cam_sensor_driver_platform_probe,
	.driver = {
		.name = "qcom,camera",
		.owner = THIS_MODULE,
		.of_match_table = cam_sensor_driver_dt_match,
		.suppress_bind_attrs = true,
	},
	.remove = cam_sensor_platform_remove,
};

static const struct i2c_device_id i2c_id[] = {
	{SENSOR_DRIVER_I2C, (kernel_ulong_t)NULL},
	{ }
};

static struct i2c_driver cam_sensor_driver_i2c = {
	.id_table = i2c_id,
	.probe = cam_sensor_driver_i2c_probe,
	.remove = cam_sensor_driver_i2c_remove,
	.driver = {
		.name = SENSOR_DRIVER_I2C,
	},
};

static int __init cam_sensor_driver_init(void)
{
	int32_t rc = 0;

	rc = platform_driver_register(&cam_sensor_platform_driver);
	if (rc < 0) {
		CAM_ERR(CAM_SENSOR, "platform_driver_register Failed: rc = %d",
			rc);
		return rc;
	}

	rc = i2c_add_driver(&cam_sensor_driver_i2c);
	if (rc)
		CAM_ERR(CAM_SENSOR, "i2c_add_driver failed rc = %d", rc);

	return rc;
}

static void __exit cam_sensor_driver_exit(void)
{
	platform_driver_unregister(&cam_sensor_platform_driver);
	i2c_del_driver(&cam_sensor_driver_i2c);
}

module_init(cam_sensor_driver_init);
module_exit(cam_sensor_driver_exit);
MODULE_DESCRIPTION("cam_sensor_driver");
MODULE_LICENSE("GPL v2");
