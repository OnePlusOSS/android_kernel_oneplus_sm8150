/***************************************************************
** Copyright (C),  2020,  OPLUS Mobile Comm Corp.,  Ltd
** VENDOR_EDIT
** File : oplus_display_panel.c
** Description : oplus display panel char dev  /dev/oplus_panel
** Version : 1.0
** Date : 2020/06/13
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**  Li.Sheng       2020/06/13        1.0           Build this moudle
******************************************************************/
#include "oplus_display_panel.h"

int oplus_display_panel_set_hecate_info(void *buf);

#define PANEL_IOCTL_DEF(ioctl, _func) \
	[PANEL_IOCTL_NR(ioctl)] = {		\
		.cmd = ioctl,			\
		.func = _func,			\
		.name = #ioctl,			\
	}
extern int oplus_display_set_aod_area(void *buf);
extern int oplus_display_panel_set_dimlayer_enable(void *data);
extern int oplus_display_panel_get_brightness(void *buf);

static const struct panel_ioctl_desc panel_ioctls[] = {
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_POWER, oplus_display_panel_set_pwr),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_POWER, oplus_display_panel_get_pwr),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_SEED, oplus_display_panel_set_seed),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_SEED, oplus_display_panel_get_seed),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PANELID, oplus_display_panel_get_id),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_FFL, oplus_display_panel_set_ffl),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_FFL, oplus_display_panel_get_ffl),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_AOD, oplus_panel_set_aod_light_mode),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_AOD, oplus_panel_get_aod_light_mode),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_MAX_BRIGHTNESS, oplus_display_panel_set_max_brightness),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_MAX_BRIGHTNESS, oplus_display_panel_get_max_brightness),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_PANELINFO, oplus_display_panel_get_vendor),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_CCD, oplus_display_panel_get_ccd_check),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_SERIAL_NUMBER, oplus_display_panel_get_serial_number),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_HBM, oplus_display_panel_set_hbm),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_HBM, oplus_display_panel_get_hbm),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DIM_ALPHA, oplus_display_panel_set_dim_alpha),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DIM_ALPHA, oplus_display_panel_get_dim_alpha),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DIM_DC_ALPHA, oplus_display_panel_set_dim_alpha),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DIM_DC_ALPHA, oplus_display_panel_get_dim_dc_alpha),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_POWER_STATUS, oplus_display_panel_set_power_status),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_POWER_STATUS, oplus_display_panel_get_power_status),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_CLOSEBL_FLAG, oplus_display_panel_set_closebl_flag),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_CLOSEBL_FLAG, oplus_display_panel_get_closebl_flag),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DIMLAYER_HBM, oplus_display_panel_set_dimlayer_hbm),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_DIMLAYER_HBM, oplus_display_panel_get_dimlayer_hbm),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_DIMLAYER_BL_EN, oplus_display_panel_set_dimlayer_enable),
	PANEL_IOCTL_DEF(PANEL_IOCTL_GET_OPLUS_BRIGHTNESS, oplus_display_panel_get_brightness),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_AOD_AREA,  oplus_display_set_aod_area),
	PANEL_IOCTL_DEF(PANEL_IOCTL_SET_HECATE_INFO,  oplus_display_panel_set_hecate_info),
};

static struct hecate_info mhecate_info[2] = {{ 0 }, { 0 }};
int oplus_display_panel_set_hecate_info(void *buf)
{
	struct hecate_info *h_info = buf;

	uint32_t display_id = h_info->display_id;
	mhecate_info[display_id].display_id = h_info->display_id;
	mhecate_info[display_id].kgsl_dump = h_info->kgsl_dump;
	pr_err("%s, displayid = %d, kgsl dump = %d", __func__,
		h_info->display_id, mhecate_info[display_id].kgsl_dump);

	return 0;
}

static int panel_open(struct inode *inode, struct file *filp)
{
	if (panel_ref > 3) {
		pr_err("%s panel has already open\n", __func__);
		return -1;
	}

	if (panel_ref == 1) {
		try_module_get(THIS_MODULE);
	}

	++panel_ref;

	return 0;
}

extern ssize_t oplus_sde_evtlog_dump_read(struct file *file, char __user *buff,
		size_t count, loff_t *ppos);

static ssize_t panel_read(struct file *filp, char __user *buffer,
		size_t count, loff_t *offset)
{
	ssize_t lens = 0;

	if (mhecate_info[0].kgsl_dump) {
		lens += oplus_sde_evtlog_dump_read(filp, buffer, count, offset);
		if (lens < 0) {
			lens = 0;
		}
	}
	/*other dump add here, as for lens add*/

	return lens;
}

static ssize_t panel_write(struct file *file, const char __user *buffer,
		size_t count, loff_t *f_pos)
{
	pr_err("%s\n", __func__);
	return count;
}

long panel_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned int in_size, out_size, drv_size, ksize;
	unsigned int nr = PANEL_IOCTL_NR(cmd);
	char static_data[128];
	char *kdata = NULL;
	const struct panel_ioctl_desc *ioctl = NULL;
	oplus_panel_feature *func = NULL;
	int retcode = -EINVAL;

	if ((nr >= PANEL_COMMOND_MAX) || (nr <= PANEL_COMMOND_BASE)) {
		pr_err("%s invalid cmd\n", __func__);
		return retcode;
	}

	ioctl = &panel_ioctls[nr];
	func = ioctl->func;
	if (unlikely(!func)) {
		pr_err("%s no function  nr = 0x%x\n", __func__ , nr);
		retcode = -EINVAL;
		return retcode;
	}

	in_size = out_size = drv_size = PANEL_IOCTL_SIZE(cmd);
	if ((cmd & ioctl->cmd & IOC_IN) == 0) {
		in_size = 0;
	}
	if ((cmd & ioctl->cmd & IOC_OUT) == 0) {
		out_size = 0;
	}
	ksize = max(max(in_size, out_size), drv_size);

	pr_debug("%s pid = %d, cmd = %s\n", __func__, task_pid_nr(current), ioctl->name);

	if (ksize <= sizeof(static_data)) {
		kdata = static_data;
	} else {
		kdata = kmalloc(ksize, GFP_KERNEL);
		if (!kdata) {
			retcode = -ENOMEM;
			goto err_panel;
		}
	}

	if (copy_from_user(kdata, (void __user *)arg, in_size) != 0) {
		retcode = -EFAULT;
		goto err_panel;
	}

	if (ksize > in_size) {
		memset(kdata+in_size, 0, ksize-in_size);
	}
	retcode = func(kdata);  /*any lock here?*/

	if (copy_to_user((void __user *)arg, kdata, out_size) != 0) {
		retcode = -EFAULT;
		goto err_panel;
	}

err_panel:
	if (!ioctl) {
		pr_err("%s invalid ioctl\n", __func__);
	}
	if (kdata != static_data) {
		kfree(kdata);
	}
	if (retcode) {
		pr_err("%s pid = %d, retcode = %d\n", __func__, task_pid_nr(current), retcode);
	}
	return retcode;
}

int panel_release(struct inode *inode, struct file *filp)
{
	--panel_ref;
	module_put(THIS_MODULE);
	pr_err("%s\n", __func__);

	return 0;
}

static const struct file_operations panel_ops =
{
	.owner              = THIS_MODULE,
	.open               = panel_open,
	.release            = panel_release,
	.unlocked_ioctl     = panel_ioctl,
	.compat_ioctl       = panel_ioctl,
	.read               = panel_read,
	.write              = panel_write,
};

static int __init oplus_display_panel_init()
{
	int rc = 0;

	printk("%s\n", __func__);

	rc = alloc_chrdev_region(&dev_num, 0, 1, OPLUS_PANEL_NAME);
	if (rc < 0) {
		pr_err("%s: failed to alloc chrdev region\n", __func__);
		return rc;
	}

	panel_class = class_create(THIS_MODULE, OPLUS_PANEL_CLASS_NAME);
	if (IS_ERR(panel_class)) {
		pr_err("%s class create error\n", __func__);
		goto err_class_create;
	}

	cdev_init(&panel_cdev, &panel_ops);
	rc = cdev_add(&panel_cdev, dev_num, 1);
	if (rc < 0) {
		pr_err("%s: failed to add cdev\n", __func__);
		goto err_cdev_add;
	}

	panel_dev = device_create(panel_class, NULL, dev_num, NULL, OPLUS_PANEL_NAME);
	if (IS_ERR(panel_dev)) {
		pr_err("%s device create error\n", __func__);
		goto err_device_create;
	}
	return 0;

err_device_create:
	cdev_del(&panel_cdev);
err_cdev_add:
	class_destroy(panel_class);
err_class_create:
	unregister_chrdev_region(dev_num, 1);

	return rc;
}

void __exit oplus_display_panel_exit()
{
	pr_err("%s\n", __func__);

	cdev_del(&panel_cdev);
	device_destroy(panel_class, dev_num);
	class_destroy(panel_class);
	unregister_chrdev_region(dev_num, 1);
}

module_init(oplus_display_panel_init);
module_exit(oplus_display_panel_exit);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lisheng");
