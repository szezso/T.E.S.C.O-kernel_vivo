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

#ifndef __SI_REGIO_H__
#define __SI_REGIO_H__
#include <linux/kernel.h>
#include <linux/types.h>
uint8_t SiIRegioCbusRead ( uint16_t regAddr, uint8_t channel );
void SiIRegioCbusWrite ( uint16_t regAddr, uint8_t channel, uint8_t value );

#endif // __SI_REGIO_H__

