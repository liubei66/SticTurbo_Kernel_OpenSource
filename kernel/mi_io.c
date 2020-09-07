/* 
 * Copyright (C) 2017-2020, The Linux Foundation. All rights reserved.
 * Copyright (C) 2019 XiaoMi, Inc.
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

#include <linux/mi_io.h>

#define IO_SYSCALL_DEFAULT	300
#define IO_BLK_DEFAULT		300
#define IO_BLK_DRIVER_DEFAULT	50
#define IO_BLK_SUBMIT_BIO_DEFAULT	200
#define IO_JBD2_DEFAULT		290
#define IO_ELV_DEFAULT		260
#define IO_DETAIL_SHOW_DEFAULT		0
#define IO_SHOW_LOG_DEFAULT		1

int show_io_level[level_max] = {
	IO_SYSCALL_DEFAULT,
	IO_BLK_DEFAULT,
	IO_BLK_DRIVER_DEFAULT,
	IO_BLK_SUBMIT_BIO_DEFAULT,
	IO_JBD2_DEFAULT,
	IO_ELV_DEFAULT,
	IO_DETAIL_SHOW_DEFAULT,
	IO_SHOW_LOG_DEFAULT,
};