#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/soc/qcom/smem.h>
#include <linux/seq_file.h>
#include <linux/platform_device.h>
#include <soc/oplus/system/oplus_project.h>
#include <linux/io.h>
#include <stdbool.h>

#include "oplus_nfc.h"

#define NFC_CHIPSET_VERSION (0x1)

static char current_chipset[32];
static bool support_nfc = false;

bool is_nfc_support()
{
	return support_nfc;
}

bool is_support_chip(chip_type chip)
{
	bool ret = false;
	const char* target_chipset;

	if (!support_nfc)
	{
		pr_err("%s, nfc not supported, or oplus_nfc has not started", __func__);
		return false;
	}

	switch(chip) {
		case NQ330:
			target_chipset = "NQ330";
			break;
		case SN100T:
			target_chipset = "SN100T";
			break;
		case SN100F:
			target_chipset = "SN100F";
			break;
		case ST21H:
			target_chipset = "ST21H";
			break;
		case ST54H:
			target_chipset = "ST54H";
			break;
		case PN557:
			target_chipset = "PN557|NQ330";
			break;
		default:
			target_chipset = "UNKNOWN";
			break;
	}

	if (strstr(target_chipset, current_chipset) != NULL)
	{
		ret = true;
	}

	pr_err("oplus_nfc target_chipset = %s, current_chipset = %s \n", target_chipset, current_chipset);
	return ret;
}

static int nfc_read_func(struct seq_file *s, void *v)
{
	void *p = s->private;

	switch((uint32_t)(p)) {
	case NFC_CHIPSET_VERSION:
		seq_printf(s, "%s", current_chipset);
		break;
	default:
		seq_printf(s, "not support\n");
		break;
	}

	return 0;
}

static int nfc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nfc_read_func, PDE_DATA(inode));
}

static const struct file_operations nfc_info_fops = {
	.owner = THIS_MODULE,
	.open  = nfc_open,
	.read  = seq_read,
	.release = single_release,
};

static int oplus_nfc_probe(struct platform_device *pdev)
{
	struct device_node *np;
	struct device* dev;
	unsigned int project;
	unsigned int operator;
	int readRet;
	char prop_project[32];
	char prop_project_operator[32];
	const char *chipset_node;
	struct proc_dir_entry *p_entry;
	static struct proc_dir_entry *nfc_info = NULL;

	pr_err("enter %s", __func__);
	dev = &pdev->dev;
	if (!dev)
	{
		pr_err("%s, no device", __func__);
		goto error_init;
	}
	project = get_project();
	operator = get_Operator_Version();
	sprintf(prop_project_operator, "chipset-%d-%d", project, operator);
	pr_err("%s, prop_project_operator to be read = %s", __func__, prop_project_operator);
	readRet = of_property_read_string(dev->of_node, prop_project_operator, &chipset_node);
	if (readRet != 0) {
		sprintf(prop_project, "chipset-%d", project);
		pr_err("%s, prop_project to be read = %s", __func__, prop_project);
		readRet = of_property_read_string(dev->of_node, prop_project, &chipset_node);
	}
	np = dev->of_node;
	if(readRet != 0) {
		sprintf(current_chipset, "NULL");
	} else {
		pr_err("%s, get chipset_node content = %s", __func__, chipset_node);
		strcpy(current_chipset, chipset_node);
		support_nfc = true;
	}

	nfc_info = proc_mkdir("oplus_nfc", NULL);
	if (!nfc_info)
	{
		pr_err("%s, make oplus_nfc dir fail", __func__);
		goto error_init;
	}

	p_entry = proc_create_data("chipset", S_IRUGO, nfc_info, &nfc_info_fops, (uint32_t *)(NFC_CHIPSET_VERSION));
	if (!p_entry)
	{
		pr_err("%s, make chipset node fail", __func__);
		goto error_init;
	}

	return 0;

error_init:
	pr_err("%s error_init", __func__);
	remove_proc_entry("oplus_nfc", NULL);
	return -ENOENT;
}

static int oplus_nfc_remove(struct platform_device *pdev)
{
	remove_proc_entry("oplus_nfc", NULL);
	return 0;
}

static const struct of_device_id onc[] = {
	{.compatible = "oplus-nfc-chipset", },
	{},
};

MODULE_DEVICE_TABLE(of, onc);

static struct platform_driver oplus_nfc_driver = {
	.probe  = oplus_nfc_probe,
	.remove = oplus_nfc_remove,
	.driver = {
		.name = "oplus-nfc-chipset",
		.of_match_table = of_match_ptr(onc),
	},
};

static int __init oplus_nfc_init(void)
{
	pr_err("enter %s", __func__);
	return platform_driver_register(&oplus_nfc_driver);
}

subsys_initcall(oplus_nfc_init);

static void __exit oplus_nfc_exit(void)
{
	platform_driver_unregister(&oplus_nfc_driver);
}
module_exit(oplus_nfc_exit);

MODULE_DESCRIPTION("OPLUS nfc chipset version");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Yukun Wang");
