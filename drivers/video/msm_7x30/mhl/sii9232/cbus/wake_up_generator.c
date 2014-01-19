/***************************************************************************
 *
 *   SiI9232 - MHL Transmitter Driver
 *
 * Copyright (C) 2011 SiliconImage, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <mach/debug_display.h>
#include <mach/htc_mhl.h>
#include "../mhl_tpi.h"
#include "../mhl_access.h"
#include "../mhl_hdcp.h"
#include "../inc/tdsm.h"
#include "../inc/defs.h"

#define DelayMS(a) msleep(a)

extern void I2C_ReadModifyWriteByte(uint8_t deviceID, uint8_t offset, uint8_t mask, uint8_t value);
unsigned char toggleCbus = 0;

void CbusWakeUpGenerator(int addr)
{

	TPI_DEBUG_PRINT(("CbusWakeUpGenerator: I2C method\n"));

    if((toggleCbus == 0))
    {
    	TPI_DEBUG_PRINT(("toggleCbus -> %02X \n", (int)toggleCbus));

        I2C_ReadModifyWriteByte(addr, 0x96, (0x80 | 0x40), (0x80 | 0x40));
        DelayMS(18);
        I2C_ReadModifyWriteByte(addr, 0x96, (0x80 | 0x40), 0x00);

        DelayMS(18);
        I2C_ReadModifyWriteByte(addr, 0x96, (0x80 | 0x40), (0x80 | 0x40));
        DelayMS(18);
        I2C_ReadModifyWriteByte(addr, 0x96, (0x80 | 0x40), 0x00);

        DelayMS(54);

        I2C_ReadModifyWriteByte(addr, 0x96, (0x80 | 0x40), (0x80 | 0x40));
        DelayMS(18);
        I2C_ReadModifyWriteByte(addr, 0x96, (0x80 | 0x40), 0x00);

        DelayMS(18);

        I2C_ReadModifyWriteByte(addr, 0x96, (0x80 | 0x40), (0x80 | 0x40));
        DelayMS(18);
        I2C_ReadModifyWriteByte(addr, 0x96, (0x80 | 0x40), 0x00);

        DelayMS(500);

    	toggleCbus = 1;

    }
}

