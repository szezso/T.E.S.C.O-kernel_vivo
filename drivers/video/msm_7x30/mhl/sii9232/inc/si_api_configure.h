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
 

#ifndef __SI_APICONFIGURE_H__
#define __SI_APICONFIGURE_H__

#define ROM     static        // 8051 type of ROM memory
#define XDATA   //xdata       // 8051 type of external memory

#define BIT0                    0x01
#define BIT1                    0x02
#define BIT2                    0x04
#define BIT3                    0x08
#define BIT4                    0x10
#define BIT5                    0x20
#define BIT6                    0x40
#define BIT7                    0x80

#define MSG_ALWAYS              0x00
#define MSG_STAT                0x01
#define MSG_DBG                 0x02
#define DEBUG_PRINT(l,x)      if (l<=0) printf x
extern int CBUS_SLAVE_ADDR;
#define SET_BITS    0xFF
#define CLEAR_BITS  0x00

#define MHD_MAX_CHANNELS                1   // Number of MDHI channels

#define CBUS_CMD_RESPONSE_TIMEOUT       10      // In 100ms increments


#endif  // __SI_APICONFIGURE_H__

