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

#define RCP_ENABLE     1
#define MSC_TESTER     0

#define DISABLE 0x00
#define ENABLE  0xFF

#define DEV_SUPPORT_EDID


#define CONF__TPI_DEBUG_PRINT   (ENABLE)
#define CONF__TPI_EDID_PRINT    (ENABLE)
#define CONF__TPI_TRACE_PRINT	(DISABLE)

#if (CONF__TPI_DEBUG_PRINT == ENABLE)
    #define TPI_DEBUG_PRINT(x...) PR_DISP_DEBUG x
#else
    #define TPI_DEBUG_PRINT(x)
#endif

enum
{
    MHL_MSC_MSG_RCP             = 0x10,
    MHL_MSC_MSG_RCPK            = 0x11,
    MHL_MSC_MSG_RCPE            = 0x12,
    MHL_MSC_MSG_RAP             = 0x20,
    MHL_MSC_MSG_RAPK            = 0x21,
};

enum
{
	MHL_ACK						= 0x33,
	MHL_NACK					= 0x34,
	MHL_ABORT					= 0x35,
	MHL_WRITE_STAT				= 0x60 | 0x80,
	MHL_SET_INT					= 0x60,
	MHL_READ_DEVCAP				= 0x61,
	MHL_GET_STATE				= 0x62,
	MHL_GET_VENDOR_ID			= 0x63,
	MHL_SET_HPD					= 0x64,
	MHL_CLR_HPD					= 0x65,
	MHL_SET_CAP_ID				= 0x66,
	MHL_GET_CAP_ID				= 0x67,
	MHL_MSC_MSG					= 0x68,
	MHL_GET_SC1_ERRORCODE		= 0x69,
	MHL_GET_DDC_ERRORCODE		= 0x6A,
	MHL_GET_MSC_ERRORCODE		= 0x6B,
	MHL_WRITE_BURST				= 0x6C,
	MHL_GET_SC3_ERRORCODE		= 0x6D,
};
