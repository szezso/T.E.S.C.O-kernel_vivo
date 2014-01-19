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

#ifndef __SI_CBUS_DEFS_H__
#define	__SI_CBUS_DEFS_H__

#define		MHD_INTERRUPT_SIZE			 	0x44 //4	
#define		MHD_SCRATCHPAD_SIZE				16
#define		MHD_MAX_BUFFER_SIZE				MHD_SCRATCHPAD_SIZE	// manually define highest number
//------------------------------------------------------------------------------
//
// MHD Specs defined registers in device capability set
//
//
typedef struct {
	unsigned char	mhd_devcap_version;					// 0x00
	unsigned char	mhd_devcap_cbus_version;			// 0x01
	unsigned char	mhd_devcap_device_category;			// 0x02
	unsigned char	mhd_devcap_power_supply_capacity;	// 0x03
	unsigned char	mhd_devcap_power_supply_provided;	// 0x04
	unsigned char	mhd_devcap_video_link_mode_support;	// 0x05
	unsigned char	mhd_devcap_audio_link_mode_support;	// 0x06
	unsigned char	mhd_devcap_hdcp_status;				// 0x07
	unsigned char	mhd_devcap_logical_device_map;		// 0x08
	unsigned char	mhd_devcap_link_bandwidth_limit;	// 0x09
	unsigned char	mhd_devcap_reserved_1;				// 0x0a
	unsigned char	mhd_devcap_reserved_2;				// 0x0b
	unsigned char	mhd_devcap_reserved_3;				// 0x0c
	unsigned char	mhd_devcap_scratchpad_size;			// 0x0d
	unsigned char	mhd_devcap_interrupt_size;			// 0x0e
	unsigned char	mhd_devcap_devcap_size;				// 0x0f

} mhd_devcap_t;
//------------------------------------------------------------------------------
//
// MHD Specs defined registers for interrupts
//
//
typedef struct {

	unsigned char	mhd_intr_0;		// 0x00
	unsigned char	mhd_intr_1;		// 0x01
	unsigned char	mhd_intr_2;		// 0x02
	unsigned char	mhd_intr_3;		// 0x03

} mhd_interrupt_t;
//------------------------------------------------------------------------------
//
// MHD Specs defined registers for status
//
//
typedef struct {

	unsigned char	mhd_status_0;	// 0x00
	unsigned char	mhd_status_1;	// 0x01
	unsigned char	mhd_status_2;	// 0x02
	unsigned char	mhd_status_3;	// 0x03

} mhd_status_t;
//------------------------------------------------------------------------------
//
// MHD Specs defined registers for local scratchpad registers
//
//
typedef struct {

	unsigned char	mhd_scratchpad[16];

} mhd_scratchpad_t;
//------------------------------------------------------------------------------

//------- END OF DEFINES -------------
#endif	// __SI_CBUS_DEFS_H__
