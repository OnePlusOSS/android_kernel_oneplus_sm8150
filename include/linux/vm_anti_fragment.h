/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#ifndef VM_ANTI_FRAGMENT_H
#define VM_ANTI_FRAGMENT_H
static ssize_t vm_fra_op_read(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	return -ENOSYS;
}

static ssize_t vm_fra_op_write(struct file *file, const char __user *buff, size_t len, loff_t *ppos)
{
	return -ENOSYS;
}

ssize_t vm_search_two_way_op_read(struct file *filp, char __user *buff, size_t count, loff_t *off)
{
	return -ENOSYS;
}

static ssize_t vm_search_two_way_op_write(struct file *file, const char __user *buff, size_t len, loff_t *ppos)
{
	return -ENOSYS;
}

static const struct file_operations vm_fra_op_fops = {
        .read = vm_fra_op_read,
        .write = vm_fra_op_write,
};

static const struct file_operations vm_search_two_way_fops = {
        .read = vm_search_two_way_op_read,
        .write = vm_search_two_way_op_write,
};
#endif
