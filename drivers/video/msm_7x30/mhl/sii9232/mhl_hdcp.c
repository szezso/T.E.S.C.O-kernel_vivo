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
 
#ifndef _MHL_HDCP_C_
#define _MHL_HDCP_C_
 
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <mach/debug_display.h>
#include <mach/htc_mhl.h>
#include "mhl_tpi.h"
#include "mhl_access.h"
#include "mhl_hdcp.h"
#include "inc/defs.h"
#include "inc/tdsm.h"

#if 1
#define HDCP_DBG(fmt, arg...) PR_DISP_DEBUG( "[hdmi/hdcp]%s: " fmt, __func__, ##arg)
#else
#define HDCP_DBG(fmt...) do {} while (0)
#endif

#define hdcp_err(fmt, arg...) pr_err( "[hdmi/hdcp]%s: " fmt, __func__, ##arg)
#define AKSV_SIZE              5
#define NUM_OF_ONES_IN_KSV    20
#define HDCP_WORKAROUND_INTERVAL 5000


bool HDCP_TxSupports;
bool HDCP_Started;
bool HDCP_AksvValid;
#ifndef HTC_DISABLE_HDCP_WORKAROUND
bool HDCP_Success = false;
#endif //HTC_DISABLE_HDCP_WORKAROUND
u8 HDCP_LinkProtectionLevel;

static struct mhl_info *mhl_hdcp_info;

extern int PutNextTxEvent(TxPowerStateEvent_e event);
extern void HdcpTimerStart(unsigned long m_sec);

bool hdcp_check_support()
{
	u8 HDCP_Rev;
	bool HDCP_Supported;

	HDCP_Supported = true;
	/* Check Device ID */
	HDCP_Rev = ReadByteTPI(0x30);
	if (HDCP_Rev !=	(0x10 | 0x02))
		HDCP_Supported = false;

	HDCP_DBG("ret=%d\n", HDCP_Supported);
	return HDCP_Supported;
}

bool hdcp_check_aksv()
{
	int ret;
	u8 B_Data[AKSV_SIZE];
	u8 i, j, NumOfOnes = 0;

	memset(B_Data, 0, AKSV_SIZE);

	for (i = 0; i < 5; i++) {
		B_Data[i] = ReadByteTPI(0x36 + i);
	}

	HDCP_DBG(" askv={%02x, %02x, %02x, %02x, %02x}\n",
		B_Data[0], B_Data[1], B_Data[2], B_Data[3], B_Data[4]);
	for (i=0; i < AKSV_SIZE; i++)
		for (j=0; j < BYTE_SIZE; j++) {
			if (B_Data[i] & 0x01)
				NumOfOnes++;
			B_Data[i] >>= 1;
		}
	if (NumOfOnes != NUM_OF_ONES_IN_KSV)
		ret = false;
	else ret = true;

	HDCP_DBG(":ret=%s\n", ret ? "true" : "false");

	return true;
}

void hdcp_on(const char *caller)
{
	HDCP_DBG(", caller=%s\n", caller);
	WriteByteTPI(0x2A, 0x01);
	HdcpTimerStart(HDCP_WORKAROUND_INTERVAL);  //7.89j
	HDCP_Started = true;
	#ifndef HTC_DISABLE_HDCP_WORKAROUND
	HDCP_Success = false;
	#endif //HTC_DISABLE_HDCP_WORKAROUND
}

void hdcp_off()
{
	HDCP_DBG("\n");
	ReadModifyWriteTPI(0x1A, BIT_3, 0x08);
	//PutNextTxEvent(txpseHDCPDeAuthenticated); //7.89j
	WriteByteTPI(0x2A, 0x00);
	HDCP_Started = false;
	#ifndef HTC_DISABLE_HDCP_WORKAROUND
	HDCP_Success = false;
	#endif //HTC_DISABLE_HDCP_WORKAROUND
	HDCP_LinkProtectionLevel = 0x00;
}

void hdcp_init(struct mhl_info* info)
{

	HDCP_DBG("\n");

	HDCP_TxSupports = false;
	HDCP_Started = false;
	HDCP_AksvValid = false;
	HDCP_LinkProtectionLevel = 0x00;

	mhl_hdcp_info = info;

	/* TX-related... need only be done once. */
	if (!hdcp_check_support()) {
		hdcp_err("TX does not support HDCP\n");
		return;
	}
	if (!hdcp_check_aksv()) {
		hdcp_err("Illegal AKSV\n");
		return;
	}
	HDCP_TxSupports = true;
	HDCP_AksvValid = true;
	// Enable the KSV Forwarding feature and the KSV FIFO Intererrupt
	ReadModifyWriteTPI(0x2A, BIT_4, 0x10);
	ReadModifyWriteTPI(0x3F, BIT_1, 0x02);
}

void hdcp_restart()
{
	HDCP_DBG("\n");
	DisableTMDS();
	hdcp_off();
	EnableTMDS();
	HdcpTimerStart(HDCP_WORKAROUND_INTERVAL); //7.89j
}

void hdcp_check_status(u8 InterruptStatusImage)
{
	u8 QueryData, LinkStatus, RegImage, NewLinkProtectionLevel, ksv;

	if (HDCP_TxSupports == false || HDCP_AksvValid == false)
		return;

	if ((HDCP_LinkProtectionLevel == 0x00) &&
		(HDCP_Started == false)) {
			QueryData = ReadByteTPI(0x29);
		/* Is HDCP avaialable */
		if (QueryData & BIT_1) {
			hdcp_on(__func__);
		}
	}
	/* Check if Link Status has changed: */
	if (InterruptStatusImage & 0x20) {
		HDCP_DBG("SECURITY_CHANGE_EVENT\n");
		LinkStatus = ReadByteTPI(0x29);
		LinkStatus &= (BIT_5 | BIT_4);
		tpi_clear_interrupt(0x20);
		switch (LinkStatus) {
		case 0x00:
			HDCP_DBG("Link = Normal\n");
			break;
		case 0x10:
			HDCP_DBG("Link = Lost\n");
			hdcp_restart();
			break;
		case 0x20:
			HDCP_DBG("Link = Renegotiation Required\n");
			hdcp_off();
			hdcp_on(__func__);
			break;
		case 0x30:
			HDCP_DBG("Link = Suspended\n");
			hdcp_on(__func__);
			break;
		}
	}
	/* Check if HDCP state has changed: */
	if (InterruptStatusImage & 0x80) {
		HDCP_DBG("HDCP_CHANGE_EVENT\n");
		RegImage = ReadByteTPI(0x29);
		NewLinkProtectionLevel = RegImage &
			(BIT_7 | BIT_6);
		if (NewLinkProtectionLevel != HDCP_LinkProtectionLevel) {
			HDCP_LinkProtectionLevel = NewLinkProtectionLevel;
			switch (HDCP_LinkProtectionLevel) {
			case 0x00:
				HDCP_DBG("Protection = None\n");
				hdcp_restart();
				break;
			case 0x40:
				if (mhl_hdcp_info->check_hdmi_sink()) {
					ReadModifyWriteTPI(0x26, BIT_4, 0);
				}
				PutNextTxEvent(txpseHDCPAuthenticated);  //7.98j
				ReadModifyWriteTPI(0x1A, BIT_3, 0);
				#ifndef HTC_DISABLE_HDCP_WORKAROUND
				HDCP_Success = true;
				#endif //HTC_DISABLE_HDCP_WORKAROUND
				HDCP_DBG("Protection = Local, Video Unmuted\n");
				break;
			case (0x80 | 0x40):
				HDCP_DBG("Protection = Extended\n");
				break;
			default:
				HDCP_DBG("Protection = Extended but not Local?\n");
				hdcp_restart();
				break;
			}
		}

		if ((ReadByteTPI(0x3E) & BIT_1) == 0x02) {
			ReadModifyWriteTPI(0x3E, BIT_1, 0x02);
			TPI_DEBUG_PRINT (("KSV Fwd: KSV FIFO has data...\n"));
			do {
				ksv = ReadByteTPI(0x41);
				if (ksv & (BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0)) {
					TPI_DEBUG_PRINT (("KSV Fwd: KSV FIFO Count = %d, ", (
					    int)(ksv & (BIT_4 | BIT_3 | BIT_2 | BIT_1 | BIT_0))));
					ksv = ReadByteTPI(0x42);	// Forward or store for revocation check
					TPI_DEBUG_PRINT (("Value = %d\n", (int)ksv));
				}
			} while ((ksv & BIT_7) == 0x00);
				TPI_DEBUG_PRINT (("KSV Fwd: Last KSV FIFO forward complete\n"));
		}
		tpi_clear_interrupt(0x80);
	}
}

#endif
