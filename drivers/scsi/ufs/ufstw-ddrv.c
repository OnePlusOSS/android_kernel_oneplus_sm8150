#include <linux/blkdev.h>
#include <scsi/scsi_cmnd.h>
#include <scsi/scsi_driver.h>
#include <linux/sysfs.h>

#include "ufs.h"
#include "ufshcd.h"

#define UFSTWD_DD_VER				0xFF00

#define __QUERY_DESC_UNIT_MAX_SIZE		0x2D

#define __UNIT_DESC_TW_LU_MAX_BUF_SIZE		0x29
#define __UNIT_DESC_PARAM_LU_ENABLE		0x3

#define __QUERY_FLAG_IDN_TW_EN			0x0E
#define __QUERY_FLAG_IDN_TW_BUF_FLUSH_EN	0x0F
#define __QUERY_FLAG_IDN_TW_FLUSH_DURING_HIBERN 0x10
#define __QUERY_ATTR_IDN_TW_FLUSH_STATUS	0x1C
#define __QUERY_ATTR_IDN_TW_BUF_SIZE		0x1D
#define __QUERY_ATTR_IDN_TW_BUF_LIFETIME_EST	0x1E

#define __QUERY_REQ_TIMEOUT			1000

static const int selector = 1;

extern int ufsf_query_flag(struct ufs_hba *hba, enum query_opcode opcode,
		enum flag_idn idn, u8 index, bool *flag_res);

struct ufstwd_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct ufstwd_dev_info *ufstwd, char *buf);
	ssize_t (*store)(struct ufstwd_dev_info *ufstwd,
			const char *buf, size_t count);
};

static ssize_t ufstwd_sysfs_store_tw_enable(struct ufstwd_dev_info *ufstwd,
		const char *buf, size_t count)
{
	unsigned long val;
	ssize_t ret = count;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val) {
		if (ufsf_query_flag(ufstwd->hba, UPIU_QUERY_OPCODE_SET_FLAG,
					__QUERY_FLAG_IDN_TW_EN,
					ufstwd->lun, NULL)) {
			pr_err("%s:%d tw set failed..\n", __func__, __LINE__);
			ret = -EINVAL;
			goto failed;
		}
	} else {
		if (ufsf_query_flag(ufstwd->hba, UPIU_QUERY_OPCODE_CLEAR_FLAG,
					__QUERY_FLAG_IDN_TW_EN,
					ufstwd->lun, NULL)) {
			pr_err("%s:%d tw clear failed..\n", __func__, __LINE__);
			ret = -EINVAL;
			goto failed;
		}
	}

	pr_info("%s:%d set tw_enable=%d success..\n",
			__func__, __LINE__, !!val);
failed:
	return ret;
}

static ssize_t ufstwd_sysfs_show_tw_enable(
		struct ufstwd_dev_info *ufstwd, char *buf)
{
	bool tw_enable;

	if (ufsf_query_flag(ufstwd->hba, UPIU_QUERY_OPCODE_READ_FLAG,
				__QUERY_FLAG_IDN_TW_EN,
				ufstwd->lun, &tw_enable)) {
		pr_err("%s:%d read flag tw_enable err..\n",
				__func__, __LINE__);
		return -EINVAL;
	}

	pr_info("%s:%d tw_enable %d\n", __func__, __LINE__, tw_enable);
	return snprintf(buf, PAGE_SIZE, "%d", tw_enable);
}

static ssize_t ufstwd_sysfs_store_flush_enable(struct ufstwd_dev_info *ufstwd,
		const char *buf, size_t count)
{
	unsigned long val;
	ssize_t ret = count;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val) {
		if (ufsf_query_flag(ufstwd->hba, UPIU_QUERY_OPCODE_SET_FLAG,
					__QUERY_FLAG_IDN_TW_BUF_FLUSH_EN,
					ufstwd->lun, NULL)) {
			pr_err("%s:%d set flag flush_enable err..\n",
					__func__, __LINE__);
			ret = -EINVAL;
			goto failed;
		}
	} else {
		if (ufsf_query_flag(ufstwd->hba, UPIU_QUERY_OPCODE_CLEAR_FLAG,
					__QUERY_FLAG_IDN_TW_BUF_FLUSH_EN,
					ufstwd->lun, NULL)) {
			pr_err("%s:%d clear flag flush_enable err..\n",
					__func__, __LINE__);
			ret = -EINVAL;
			goto failed;
		}
	}

	pr_info("%s:%d set flush_enable=%d success..\n",
			__func__, __LINE__, !!val);
failed:
	return ret;
}

static ssize_t ufstwd_sysfs_show_flush_enable(
		struct ufstwd_dev_info *ufstwd, char *buf)
{
	bool flush_enable;

	if (ufsf_query_flag(ufstwd->hba, UPIU_QUERY_OPCODE_READ_FLAG,
				__QUERY_FLAG_IDN_TW_BUF_FLUSH_EN,
				ufstwd->lun, &flush_enable)) {
		pr_err("%s:%d read flag flush_enable err..\n", __func__, __LINE__);
		return -EINVAL;
	}

	pr_info("%s:%d flush_enable %d\n", __func__, __LINE__, flush_enable);
	return snprintf(buf, PAGE_SIZE, "%d", flush_enable);
}

static ssize_t ufstwd_sysfs_store_flush_during(struct ufstwd_dev_info *ufstwd,
		const char *buf, size_t count)
{
	unsigned long val;
	ssize_t ret = count;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val) {
		if (ufsf_query_flag(ufstwd->hba, UPIU_QUERY_OPCODE_SET_FLAG,
					__QUERY_FLAG_IDN_TW_FLUSH_DURING_HIBERN,
					ufstwd->lun, NULL)) {
			pr_err("%s:%d set flag flush_during_hibern_enter err..\n",
					__func__, __LINE__);
			ret = -EINVAL;
			goto failed;
		}
	} else {
		if (ufsf_query_flag(ufstwd->hba, UPIU_QUERY_OPCODE_CLEAR_FLAG,
					__QUERY_FLAG_IDN_TW_FLUSH_DURING_HIBERN,
					ufstwd->lun, NULL)) {
			pr_err("%s:%d clear flag flush_during_hibern_enter err..\n",
					__func__, __LINE__);
			ret = -EINVAL;
			goto failed;
		}
	}

	pr_info("%s:%d set flush_enable_during_hibern=%d success..\n",
			__func__, __LINE__, !!val);
failed:
	return ret;
}

static ssize_t ufstwd_sysfs_show_flush_during(
		struct ufstwd_dev_info *ufstwd, char *buf)
{
	bool flush_during;

	if (ufsf_query_flag(ufstwd->hba, UPIU_QUERY_OPCODE_READ_FLAG,
				__QUERY_FLAG_IDN_TW_FLUSH_DURING_HIBERN,
				ufstwd->lun, &flush_during)) {
		pr_err("%s:%d read flag flush_during_hibern_enter err..\n",
				__func__, __LINE__);
		return -EINVAL;
	}

	pr_info("%s:%d flush_during_hibern_enter %d\n",
			__func__, __LINE__, flush_during);
	return snprintf(buf, PAGE_SIZE, "%d", flush_during);
}

static ssize_t ufstwd_sysfs_show_flush_status(
		struct ufstwd_dev_info *ufstwd, char *buf)
{
	u32 val;

	if (ufshcd_query_attr(ufstwd->hba, UPIU_QUERY_OPCODE_READ_ATTR,
			__QUERY_ATTR_IDN_TW_FLUSH_STATUS, ufstwd->lun,
			selector, &val))
		return -EINVAL;

	pr_info("%s:%d flush_status %u (0x%X)\n", __func__, __LINE__, val, val);
	return snprintf(buf, PAGE_SIZE, "%u", val);
}

static ssize_t ufstwd_sysfs_show_abs(struct ufstwd_dev_info *ufstwd, char *buf)
{
	u32 val;

	if (ufshcd_query_attr(ufstwd->hba, UPIU_QUERY_OPCODE_READ_ATTR,
			__QUERY_ATTR_IDN_TW_BUF_SIZE, ufstwd->lun,
			selector, &val))
		return -EINVAL;

	pr_info("%s:%d buf_size %u (0x%X)\n", __func__, __LINE__, val, val);
	return snprintf(buf, PAGE_SIZE, "%u", val);
}

static ssize_t ufstwd_sysfs_show_lifetime(
		struct ufstwd_dev_info *ufstwd, char *buf)
{
	u32 val;

	if (ufshcd_query_attr(ufstwd->hba, UPIU_QUERY_OPCODE_READ_ATTR,
			__QUERY_ATTR_IDN_TW_BUF_LIFETIME_EST, ufstwd->lun,
			selector, &val))
		return -EINVAL;

	pr_info("%s:%d lifetime_est %u (0x%X)\n", __func__, __LINE__, val, val);
	return snprintf(buf, PAGE_SIZE, "%u", val);
}

static struct ufstwd_sysfs_entry ufstwd_sysfs_entries[] = {
	__ATTR(tw_enable, 0644, ufstwd_sysfs_show_tw_enable,
			ufstwd_sysfs_store_tw_enable),
	__ATTR(flush_enable, 0644, ufstwd_sysfs_show_flush_enable,
			ufstwd_sysfs_store_flush_enable),
	__ATTR(flush_during_hiber_enter, 0644, ufstwd_sysfs_show_flush_during,
			ufstwd_sysfs_store_flush_during),
	__ATTR(flush_status, 0444, ufstwd_sysfs_show_flush_status, NULL),
	__ATTR(available_buffer_size, 0444, ufstwd_sysfs_show_abs, NULL),
	__ATTR(lifetime_est, 0444, ufstwd_sysfs_show_lifetime, NULL),
};

static ssize_t ufstwd_attr_show(struct kobject *kobj,
		struct attribute *attr, char *page)
{
	struct ufstwd_sysfs_entry *entry;
	struct ufstwd_dev_info *ufstwd;
	ssize_t error;

	entry = container_of(attr, struct ufstwd_sysfs_entry, attr);
	if (!entry->show)
		return -EIO;
	ufstwd = container_of(kobj, struct ufstwd_dev_info, kobj);

	pm_runtime_get_sync(ufstwd->hba->dev);
	mutex_lock(&ufstwd->sysfs_lock);
	error = entry->show(ufstwd, page);
	mutex_unlock(&ufstwd->sysfs_lock);
	pm_runtime_put_sync(ufstwd->hba->dev);

	return error;
}

static ssize_t ufstwd_attr_store(struct kobject *kobj, struct attribute *attr,
				const char *page, size_t length)
{
	struct ufstwd_sysfs_entry *entry;
	struct ufstwd_dev_info *ufstwd;
	ssize_t error;

	entry = container_of(attr, struct ufstwd_sysfs_entry, attr);
	if (!entry->store)
		return -EIO;
	ufstwd = container_of(kobj, struct ufstwd_dev_info, kobj);

	pm_runtime_get_sync(ufstwd->hba->dev);
	mutex_lock(&ufstwd->sysfs_lock);
	error = entry->store(ufstwd, page, length);
	mutex_unlock(&ufstwd->sysfs_lock);
	pm_runtime_put_sync(ufstwd->hba->dev);

	return error;
}

static const struct sysfs_ops ufstwd_sysfs_ops = {
	.show = ufstwd_attr_show,
	.store = ufstwd_attr_store,
};

static struct kobj_type ufstwd_ktype = {
	.sysfs_ops = &ufstwd_sysfs_ops,
	.release = NULL,
};

static int ufstwd_init_sysfs(struct ufs_hba *hba,
		struct ufstwd_dev_info *ufstwd)
{
	struct ufstwd_sysfs_entry *entry;
	int err = 0;

	ufstwd->sysfs_entries = ufstwd_sysfs_entries;

	kobject_init(&ufstwd->kobj, &ufstwd_ktype);
	mutex_init(&ufstwd->sysfs_lock);

	err = kobject_add(&ufstwd->kobj, kobject_get(&hba->dev->kobj),
			  "ufstwd%d", ufstwd->lun);
	if (!err) {
		for (entry = ufstwd->sysfs_entries; entry->attr.name != NULL;
				entry++) {
			pr_info("ufstwd%d sysfs attr creates: %s",
				  ufstwd->lun, entry->attr.name);
			if (sysfs_create_file(&ufstwd->kobj, &entry->attr))
				break;
		}
		pr_info("ufstwd%d sysfs adds uevent", ufstwd->lun);
		kobject_uevent(&ufstwd->kobj, KOBJ_ADD);
	}
	return err;
}

int ufstw_enable_tw(struct ufstwd_dev_info *ufstwd, bool enable)
{
	ssize_t ret = 0;
	if (enable) {
		if (ufsf_query_flag(ufstwd->hba, UPIU_QUERY_OPCODE_SET_FLAG,
					__QUERY_FLAG_IDN_TW_EN,
					ufstwd->lun, NULL)) {
			pr_err("%s:%d tw set failed..\n", __func__, __LINE__);
			ret = -EINVAL;
			goto failed;
		}
	} else {
		if (ufsf_query_flag(ufstwd->hba, UPIU_QUERY_OPCODE_CLEAR_FLAG,
					__QUERY_FLAG_IDN_TW_EN,
					ufstwd->lun, NULL)) {
			pr_err("%s:%d tw clear failed..\n", __func__, __LINE__);
			ret = -EINVAL;
			goto failed;
		}
	}

	pr_info("%s:%d set tw_enable=%d success..\n",
			__func__, __LINE__, enable);
failed:
	return ret;
}

int ufstw_enable_auto_flush(struct ufstwd_dev_info *ufstwd, bool enable)
{
	ssize_t ret = 0;
	if (enable) {
		if (ufsf_query_flag(ufstwd->hba, UPIU_QUERY_OPCODE_SET_FLAG,
					__QUERY_FLAG_IDN_TW_FLUSH_DURING_HIBERN,
					ufstwd->lun, NULL)) {
			pr_err("%s:%d set flag flush_during_hibern_enter err..\n",
					__func__, __LINE__);
			ret = -EINVAL;
			goto failed;
		}
	} else {
		if (ufsf_query_flag(ufstwd->hba, UPIU_QUERY_OPCODE_CLEAR_FLAG,
					__QUERY_FLAG_IDN_TW_FLUSH_DURING_HIBERN,
					ufstwd->lun, NULL)) {
			pr_err("%s:%d clear flag flush_during_hibern_enter err..\n",
					__func__, __LINE__);
			ret = -EINVAL;
			goto failed;
		}
	}

	pr_info("%s:%d set flush_enable_during_hibern=%d success..\n",
			__func__, __LINE__, enable);
failed:
	return ret;
}

extern int ufsplus_tw_status;
static bool ufstwd_lu_init(struct ufs_hba *hba, int lun, u8 *unit_buf)
{
	struct ufstwd_dev_info *ufstwd;
	u8 lu_enable;
	u32 tw_lu_buf_size;

	printk("%s:%d lu desc size %d\n", __func__, __LINE__, unit_buf[0]);

	lu_enable = unit_buf[__UNIT_DESC_PARAM_LU_ENABLE];
	if (!lu_enable) {
		pr_info("%s:%d lun%d not enabled\n", __func__, __LINE__, lun);
		return false;
	}

	tw_lu_buf_size = LI_EN_32(&unit_buf[
			__UNIT_DESC_TW_LU_MAX_BUF_SIZE]);
	if (!tw_lu_buf_size) {
		pr_info("%s:%d lun%d has not tw-buffer\n",
				__func__, __LINE__, lun);
		return false;
	}

	ufstwd = kzalloc(sizeof(struct ufstwd_dev_info), GFP_KERNEL);
	if (!ufstwd) {
		pr_err("%s:%d lun%d memory allocation failed..\n",
				__func__, __LINE__, lun);
		return false;
	}

	hba->ufstwd = ufstwd;
	ufstwd->hba = hba;
	ufstwd->lun = lun;
	ufstw_enable_tw(ufstwd,true);
	ufstw_enable_auto_flush(ufstwd,true);
	ufsplus_tw_status = 1;
	ufstwd_init_sysfs(hba, ufstwd);

	pr_info("%s:%d ufstw%d lu init success.. buffersize %u\n",
		__func__, __LINE__, lun, tw_lu_buf_size);
	return true;
}

void ufstwd_dev_init(struct ufs_hba *hba)
{
	bool success;
	int size;
	int lun, err = 0;
	u8 *unit_buf;

	pr_info("%s:%d init start..\n", __func__, __LINE__);
	pr_info("%s:%d UFSTW-DEBUG Driver version 0x%X\n",
			__func__, __LINE__, UFSTWD_DD_VER);
	unit_buf = kzalloc(255, GFP_KERNEL);

	size = __QUERY_DESC_UNIT_MAX_SIZE;
	pm_runtime_get_sync(hba->dev);
	for (lun = 0; lun < 8; lun++) {
		printk("%s:%d read desc.. lun%d\n",
				__func__, __LINE__, lun);
		err = ufshcd_query_descriptor_retry(hba,
				UPIU_QUERY_OPCODE_READ_DESC,
				QUERY_DESC_IDN_UNIT, lun,
				selector, unit_buf, &size);
		if (err) {
			pr_warn("%s:%d lun%d read descriptor error.."
				"size %d err %d len %d\n",
				__func__, __LINE__, lun, size,
				err, unit_buf[0]);
			continue;
		}

		printk("%s:%d init start.. lun%d\n",
				__func__, __LINE__, lun);
		success = ufstwd_lu_init(hba, lun, unit_buf);
		if (success) {
			pr_info("%s:%d ufstwd lu%d init complete..\n",
					__func__, __LINE__, lun);
			break;
		}
	}
	pm_runtime_put_sync(hba->dev);

	if (!success)
		pr_info("%s:%d cannot find ufstw lu\n",
				__func__, __LINE__);

	pr_info("%s:%d init end..\n", __func__, __LINE__);
	kfree(unit_buf);
}
