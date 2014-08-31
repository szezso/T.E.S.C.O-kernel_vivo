/*
 * include/linux/ion.h
 *
 * Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_MSM_ION_H
#define _LINUX_MSM_ION_H

#include <linux/ion.h>

static inline struct ion_handle *ion_import_dma_buf(struct ion_client *client, int fd)
{
	return ERR_PTR(-ENODEV);
}

struct sg_table *ion_sg_table(struct ion_client *client,
			      struct ion_handle *handle);

#endif
