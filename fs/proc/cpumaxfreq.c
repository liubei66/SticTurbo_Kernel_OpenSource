/* 
 * Copyright (C) 2017-2020, The Linux Foundation. All rights reserved.
 * Copyright (C) 2020 Amktiao.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 */

#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int cpumaxfreq_show(struct seq_file *m, void *v)
{
	seq_printf(m, "10");

	return 0;
}

static int cpumaxfreq_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpumaxfreq_show, NULL);
}

static const struct file_operations proc_cpumaxfreq_operations = {
	.open       = cpumaxfreq_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = seq_release,
};

static int __init proc_cpumaxfreq_init(void)
{
	proc_create("cpumaxfreq", 0, NULL, &proc_cpumaxfreq_operations);
	return 0;
}
fs_initcall(proc_cpumaxfreq_init);
