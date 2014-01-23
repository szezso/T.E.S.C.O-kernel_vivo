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

/**
 * Flag for clients to force contiguous memort allocation
 *
 * Use of this flag is carefully monitored!
 */
#define ION_FORCE_CONTIGUOUS (1 << 30)

enum cp_mem_usage {
	VIDEO_BITSTREAM = 0x1,
	VIDEO_PIXEL = 0x2,
	VIDEO_NONPIXEL = 0x3,
	MAX_USAGE = 0x4,
	UNKNOWN = 0x7FFFFFFF,
};

/**
 * ion_import_dma_buf() - given an dma-buf fd from the ion exporter get handle
 * @client:	the client
 * @fd:		the dma-buf fd
 *
 * Given an dma-buf fd that was allocated through ion via ion_share_dma_buf,
 * import that fd and return a handle representing it.  If a dma-buf from
 * another exporter is passed in this function will return ERR_PTR(-EINVAL)
 */
struct ion_handle *ion_import_dma_buf(struct ion_client *client, int fd);

#endif
