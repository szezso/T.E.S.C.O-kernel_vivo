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
 
#ifndef _MHL_TPI_C_
#define _MHL_TPI_C_

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <mach/htc_mhl.h>
#include <mach/debug_display.h>
#include "mhl_tpi.h"
#include "mhl_access.h"
#include "mhl_hdcp.h"
#include "./inc/tdsm.h"
#include "./inc/defs.h"

static void TxHW_Reset(void);
static bool StartTPI(void);

static void WakeUpFromD3(void);
static void InitializeStateVariables(void);
static void WriteInitialRegisterValues(void);
static void InitCBusRegs(void);

static void InitForceUsbIdSwitchOpen (void);
static void InitReleaseUsbIdSwitchOpen (void);

void ForceUsbIdSwitchOpen (void);
void ReleaseUsbIdSwitchOpen (void);

static void EnableInterrupts (uint8_t pattern);
static void DisableInterrupts (uint8_t pattern);

static void HotPlugService (void);

static void OnMHLCableConnected (void);

static void OnHdmiCableConnected (void);
static void OnHdmiCableDisconnected (void);

static void OnDownstreamRxPoweredDown (void);
static void OnDownstreamRxPoweredUp (void);

static void TxPowerStateD0 (void);
static void TxPowerStateD3 (void);

static void CheckTxFifoStable (void);


static bool tclkStable;
static bool MHLCableConnected;
static bool hdmiCableConnected;
static bool dsRxPoweredUp;
static uint8_t tmdsPoweredUp;
uint8_t txPowerState;
static bool checkTclkStable;
static struct mhl_info *mhl_tpi_info;
bool bInTpiMode = false;
static unsigned long rsenCheckTimeout = 0;
static unsigned long deglitchTimeout = 0;
static unsigned long hdcp_timeout_jiffies = 0;
#ifndef HTC_DISABLE_HDCP_WORKAROUND
static unsigned long hdcpCheckTimeout = 0;
#endif //HTC_DISABLE_HDCP_WORKAROUND

extern void hdcp_restart(void);
extern void CpCbusInitialize(struct mhl_info *mhl_cbus_info);
extern void CpCbusHandler(void);
extern int toggleCbus;
extern void CbusWakeUpGenerator(int addr);
#ifndef HTC_DISABLE_HDCP_WORKAROUND
extern bool HDCP_Success;
#endif //HTC_DISABLE_HDCP_WORKAROUND

#define NON_MASKABLE_INT		0xFF
#define ENABLE_AUTO_SOFT_RESET			0x04
#define DISABLE_AUTO_SOFT_RESET			0x00
#define ASR_VALUE						ENABLE_AUTO_SOFT_RESET
#define DDC_XLTN_TIMEOUT_MAX_VAL		0x30
#define CLEAR_CBUS_TOGGLE()     (toggleCbus = 0)
#define TPI_SLAVE_ADDR		((uint8_t)(mhl_tpi_info->i2c_addr_tpi))
#define DelayMS(a)		msleep(a)
#define CALL_CBUS_WAKEUP_GENERATOR(addr)	CbusWakeUpGenerator(addr) 
#define INTERVAL_RSEN_DEGLITCH 150
#define TXSF_D0_NO_CONNECTION_DOWNSTREAM_POWER 0
#define TXSF_CHECK_VIDEO_MODE_CHANGE 0
#define TXSF_CALL_CEC_HANDLER 0
#define TXSF_D0_NO_CONNECTION  ( txsfExaminePinTxInt )

void tpi_clear_interrupt(uint8_t pattern)
{
	WriteByteTPI(0x3D, pattern);
}

bool tpi_enable_interrupts(uint8_t Interrupt_Pattern)
{
	TPI_DEBUG_PRINT(("%s, reg=%02x, pat=%02x\n", __func__, 0x3C, Interrupt_Pattern));
	ReadSetWriteTPI(0x3C, Interrupt_Pattern);
	return true;
}

void tpi_disable_interrupts(uint8_t pattern)
{
	ReadClearWriteTPI(0x3C, pattern);
}

void tpi_clear_pending_event(void)
{
	int retry = 100;

	while (retry--) {
		WriteByteTPI(0x3c, 1);
		WriteByteTPI(0x3d, 1);
		if (ReadByteTPI(0x3d) & 0x01)
			DelayMS(1);
		else
			break;
	}
	if (retry < 19) TPI_DEBUG_PRINT(("%s: retry=%d\n", __func__, 19 - retry));
}


void SetAudioMute(uint8_t audioMute)
{
	ReadModifyWriteTPI(0x26, BIT_4, audioMute);
}

void SetInputColorSpace(uint8_t inputColorSpace)
{
	ReadModifyWriteTPI(0x09, (BIT_1 | BIT_0), inputColorSpace);
	ReadModifyWriteTPI(0x09, (BIT_1 | BIT_0), inputColorSpace);
	ReadModifyWriteTPI(0x19, 0x00, 0x00);
}


uint8_t Status_Query(void)
{
	return txPowerState;
}

static bool ProductID_Read(void)
{
	uint8_t devID = 0x00;
	uint16_t wID = 0x0000;

	devID = ReadIndexedRegister(0x01, 0x03);
	wID = devID;
	wID <<= 8;
	devID = ReadIndexedRegister(0x01, 0x02);
	wID |= devID;
	TPI_DEBUG_PRINT(("SiI %04X\n", (int) wID));

	if (wID == SiI9232_PRODUCT_ID) {
		return true;
	}	
	TPI_DEBUG_PRINT(("Unsupported TX\n"));
	return false;
}

static void GoToD3 (void)
{
	printk("%s\n", __func__);
	mhl_tpi_info->change_hdmi_state(false);
	hdcp_off();

	TxPowerStateD3();

	MHLCableConnected = false;
	hdmiCableConnected = false;
	dsRxPoweredUp = false;
	bInTpiMode = false;
	mhl_tpi_info->cable_status_update(false);
}

bool TPI_Init(void)
{
	TPI_DEBUG_PRINT(("9232 SK "));
	TPI_DEBUG_PRINT(("(X04)"));
	TPI_DEBUG_PRINT((" FW 0.22\n"));

	txPowerState = TX_POWER_STATE_D0;

	InitializeStateVariables();
	TxHW_Reset();
	WriteInitialRegisterValues();
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x7A, 0x14);
	if (!StartTPI()) {
		return false;
	}

	if (!ProductID_Read()) {
		return false;
	}

       hdcp_init(mhl_tpi_info);

	MHLCableConnected = FALSE;
	hdmiCableConnected = false;
	OnDownstreamRxPoweredDown();
	WriteIndexedRegister(0x01, 0xA0, 0xD0);

        TPI_DEBUG_PRINT(("\ncalling CpCbusInitialize\n\n"));
        CpCbusInitialize(mhl_tpi_info);

	hdcp_off();

	TxPowerStateD3();

	MHLCableConnected = false;
	hdmiCableConnected = false;
	dsRxPoweredUp = false;
   	bInTpiMode =false;	

        txPowerState = txpsD3;
	return true;
}

int tpi_hw_init(struct mhl_info* info) 
{
        bool rv;
    
        mhl_tpi_info = info;
        mhl_access_init(info);
	
	mhl_tpi_info->video_switch(1); 
        rv = TPI_Init();

        if (rv != true) {
                pr_err("%s: can't init\n", __func__);
                return -1;
        }
        return 0;
}

static void WakeUpFromD3(void) 
{
	TPI_DEBUG_PRINT(("Waking up...\n"));

	InitializeStateVariables();
	WriteInitialRegisterValues();
}

static void InitializeStateVariables (void)
{
	tclkStable = false;
	checkTclkStable = true;
	tmdsPoweredUp = false;
	MHLCableConnected = false;
	hdmiCableConnected = false;
	dsRxPoweredUp = false;
}

static void WriteInitialRegisterValues(void)
{
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x08, 0x37);

	I2C_WriteByte(TPI_SLAVE_ADDR, 0xA0, 0xD0);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0xA1, 0xFC);

	I2C_WriteByte(TPI_SLAVE_ADDR, 0xA3, 0xC0);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0xA6, 0x0C);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x2B, 0x01);

	ReadModifyWriteTPI(0x90, BIT_3 | BIT_2, BIT_2);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x91, 0xA5);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x94, 0x75);


	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x31, I2C_ReadByte(CBUS_SLAVE_ADDR, 0x31) | 0x0c);

	I2C_WriteByte(TPI_SLAVE_ADDR, 0xA5, 0xA0);
	TPI_DEBUG_PRINT(("1x Mode\n"));
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x95, 0x31);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x96, 0x20);

	ReadModifyWriteTPI(0x97,  BIT_1, 0);

	ReadModifyWriteTPI(0x95, BIT_6, BIT_6);
	WriteByteTPI(0x92, 0x86);
	WriteByteTPI(0x93, 0xCC);
	if (txPowerState != TX_POWER_STATE_D3) {
		ReadModifyWriteTPI(0x79, BIT_5 | BIT_4, BIT_4);
	}

	DelayMS(25);
	ReadModifyWriteTPI(0x95, BIT_6, 0x00);
	ReadModifyWriteTPI(0x78, BIT_5, 0);

	I2C_WriteByte(TPI_SLAVE_ADDR, 0x90, 0x27);

	I2C_WriteByte(TPI_SLAVE_ADDR, 0x05, 0x08);
	DelayMS(2);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x05, 0x00);

	InitCBusRegs();

	I2C_WriteByte(TPI_SLAVE_ADDR, 0x05, ASR_VALUE);
	}


static void InitCBusRegs(void) 
{
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x1F, 0x02);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x07, 0x30 | 0x06);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x40, 0x03);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x42, 0x06);
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x36, 0x0C);

	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x3D, 0xFD);	
	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x1C, 0x00);

	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x44, 0x02);

	I2C_WriteByte(CBUS_SLAVE_ADDR, 0x09, 0x60);	
}


void ForceUsbIdSwitchOpen (void)
{
	ReadModifyWriteIndexedRegister(0x01, 0x90, BIT_0, 0x00);
	ReadModifyWriteIndexedRegister(0x01, 0x95, BIT_6, BIT_6);
	WriteIndexedRegister(0x01, 0x92, 0x86);
	ReadModifyWriteIndexedRegister(0x01, 0x79, BIT_5 | BIT_4, BIT_4);	
}


void ReleaseUsbIdSwitchOpen(void)
{
	DelayMS(25);
	ReadModifyWriteIndexedRegister(0x01, 0x95, BIT_6, 0x00);
	ReadModifyWriteIndexedRegister(0x01, 0x90, BIT_0, BIT_0);
}


static void InitForceUsbIdSwitchOpen(void){
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x90, 0x26);
	ReadModifyWriteTPI(0x95, BIT_6, BIT_6);
	ReadModifyWriteTPI(0x95, BIT_6, BIT_6);
	ReadModifyWriteTPI(0x79, BIT_5 | BIT_4, BIT_4);
}


static void InitReleaseUsbIdSwitchOpen(void){
	DelayMS(25);
	ReadModifyWriteTPI(0x95, BIT_6, 0x00);
	ReadModifyWriteTPI(0x90, BIT_0, BIT_0);
}

enum hd_res {
        hd_720p = 0,    /* 1280 * 720 */
        svga,           /* 800 * 600 */
        pal,            /* 720 * 576 */
        edtv,           /* 720 * 480 */
        vga,            /* 640 * 480 */
};

static uint8_t video_param[][8] = {
        [hd_720p] = {0x01, 0x1d, 0x70, 0x17, 0x72, 0x06, 0xee, 0x02},
        [svga] = {0xa0, 0x0f, 0x70, 0x17, 0x20, 0x04, 0x74, 0x02},
        [pal] = {0x8c, 0x0a, 0x88, 0x13, 0x60, 0x03, 0x71, 0x02}, /* 576p50 */
        [edtv] = {0x8c, 0x0a, 0x70, 0x17, 0x5a, 0x03, 0x0d, 0x02},/* 480p60 */
        [vga] = {0x8c, 0x0a, 0x70, 0x17, 0x20, 0x03, 0x0d, 0x02},
};
static uint8_t avi_info_frame[][14] = {
        [hd_720p] = {0x43, 0x00, 0x28, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
                     0x00, 0x00, 0x00, 0x00, 0x00},
        [svga] = {0x6f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                  0x00, 0x00, 0x00, 0x00},
        [pal] = {0x46, 0x00, 0x18, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00}, /* 576p50 */
        [edtv] = {0x55, 0x00, 0x18, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00,
                  0x00, 0x00, 0x00, 0x00},
        [vga] = {0x5E, 0x00, 0x10, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
                 0x00, 0x00, 0x00, 0x00},
};
static uint8_t audio_info_frame[] =
        { 0xc2, 0x84, 0x01, 0x0a, 0x71, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


void Active9232 (void) {
	uint8_t i;
	uint8_t *video_parm = &video_param[hd_720p][0];

        /* choose video mode */
	if (mhl_tpi_info->check_hdmi_sink()) {
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x08, 0x70);
		I2C_WriteBlock(TPI_SLAVE_ADDR, 0x0, video_parm, 8);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x0, 0x60);
	}

	if (!mhl_tpi_info->check_hdmi_sink()) {
    		ReadModifyWriteTPI(0x1A, BIT_0, 0x01);
	}
	
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x09, 0x00);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x19, 0x00);

	if (!mhl_tpi_info->check_hdmi_sink()) {
    		ReadModifyWriteTPI(0x1A, BIT_0, 0x00);
	}
	
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x60, 0x00);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0xbf, 0x00);

	for (i = 0; i < 14 ;i++)
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x0C + i, avi_info_frame[hd_720p][i]);

	I2C_WriteByte(TPI_SLAVE_ADDR, 0xbf, 0xc6);
	
	if (mhl_tpi_info->check_hdmi_sink()) {
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x26, 0x91);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x25, 0x03);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x27, 0x59);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x28, 0x00);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x1f, 0x80);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x20, 0x90);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x21, 0x00);
	
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x24, 0x02);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x25, 0x00);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0xbc, 0x02);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0xbd, 0x24);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0xbe, 0x92);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0xbc, 0x02);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0xbd, 0x2f);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0xbe, 0x0);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x26, 0x81);
	
		I2C_WriteBlock(TPI_SLAVE_ADDR, 0xbf, audio_info_frame, 15);
		I2C_WriteByte(TPI_SLAVE_ADDR, 0x0a, 0x0);
	}
}

void HotPlugService (void) {
	DisableInterrupts(0xFF);
	Active9232();
	if ((HDCP_TxSupports == true)) {
		if (HDCP_AksvValid == true) {
			// AV MUTE
			TPI_DEBUG_PRINT (("TMDS -> Enabled (Video Muted)\n"));

			ReadModifyWriteTPI(0x1A, BIT_6 | BIT_4	| BIT_3,
				0x40 | 0x00 | 0x08);

			tmdsPoweredUp = true;

			EnableInterrupts(BIT_0 | 0x02 | 0x10 | 0x20 | 0x40 | 0x80);
		}

	} else {
		TPI_DEBUG_PRINT (("TMDS -> Enabled\n"));
		ReadModifyWriteTPI(0x1A, 
			BIT_6 | BIT_4 | BIT_3,
				0x40 | 0x00 | 0x08);
		tmdsPoweredUp = TRUE;
		EnableInterrupts(0x01 | 0x02 | 0x10 | 0x40);
	}
	TxPowerStateD0();

	I2C_WriteByte(TPI_SLAVE_ADDR, 0xcd, 0x0);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x19, 0x0);
}

static void TxHW_Reset(void) {
	mhl_tpi_info->reset_chip();
}

static bool StartTPI(void)
{

	WriteByteTPI(TPI_ENABLE, 0x00);
	DelayMS(100);
    	bInTpiMode = true;	
	return true;
}

static void EnableInterrupts(uint8_t Interrupt_Pattern) 
{
    ReadModifyWriteTPI(0x3D,Interrupt_Pattern,Interrupt_Pattern);
    ReadModifyWriteTPI(0x3C,Interrupt_Pattern,Interrupt_Pattern);
}

static void DisableInterrupts(uint8_t Interrupt_Pattern) 
{
	ReadClearWriteTPI(0x3C, Interrupt_Pattern);
	ReadModifyWriteIndexedRegister(0x01, 0x75, BIT_5, 0);
}

static void TxPowerStateD0(void) 
{
	ReadModifyWriteTPI(0x1E, BIT_1 | BIT_0, 0x00);
	TPI_DEBUG_PRINT(("TX Power State D0\n"));
	ReadModifyWriteIndexedRegister(0x01, 0x94, BIT_0 | BIT_1, 0x01);
	ReadModifyWriteIndexedRegister(0x01, 0x93, 0xF0, 0x80);

}


static void TxPowerStateD3(void)
{
	ReadModifyWriteIndexedRegister(0x01, 0x94, BIT_0 | BIT_1, 0x00);
	ReadModifyWriteIndexedRegister(0x01, 0x93, BIT_3 | BIT_4 | BIT_5 | BIT_7, BIT_3);
	ReadModifyWriteIndexedRegister(0x02, 0x3D, BIT_0, 0x00);
	bInTpiMode = false;
	TPI_DEBUG_PRINT(("TX Power State D3\n"));
    	CLEAR_CBUS_TOGGLE(); 
}

void EnableTMDS(void) 
{
	TPI_DEBUG_PRINT(("TMDS -> Enabled\n"));
	ReadModifyWriteTPI(0x1A, BIT_4, 
		0x00);
	tmdsPoweredUp = true;
}


void DisableTMDS(void)
{
	TPI_DEBUG_PRINT(("TMDS -> Disabled\n"));
	ReadModifyWriteTPI(0x1A, BIT_4 | BIT_3,
		 0x10 | 0x08);
	tmdsPoweredUp = false;
}

void RAPContentOn (void)
{
    ReadModifyWriteTPI(0x1A, BIT_3, 0x00);
}
void RAPContentOff (void)
{
    ReadModifyWriteTPI(0x1A, BIT_3, 0x08);
}

void CheckTxFifoStable(void)
{
	uint8_t bTemp;

	bTemp = ReadIndexedRegister(0x01, 0x3E);
	if ((bTemp & (BIT_7 | BIT_6)) != 0x00) {
		TPI_DEBUG_PRINT(("FIFO Overrun / Underrun\n"));
		WriteIndexedRegister(0x01, 0x05, BIT_4 | ASR_VALUE);
		DelayMS(1);
		WriteIndexedRegister(0x01, 0x05, ASR_VALUE);
	}
}


bool GetDDC_Access(uint8_t* SysCtrlRegVal)
{
        uint8_t sysCtrl, TPI_ControlImage, DDCReqTimeout = T_DDC_ACCESS;

	sysCtrl = I2C_ReadByte(TPI_SLAVE_ADDR, 0x1A);
        *SysCtrlRegVal = sysCtrl;

        sysCtrl |= BIT_2;
        I2C_WriteByte(TPI_SLAVE_ADDR, 0x1A, sysCtrl);


        while (DDCReqTimeout--) {
                TPI_ControlImage = I2C_ReadByte(TPI_SLAVE_ADDR, 0x1A);

                if (TPI_ControlImage & BIT_1) {
                        sysCtrl |= BIT_1;
                        I2C_WriteByte(TPI_SLAVE_ADDR, 0x1A, sysCtrl);
                        return true;
                }
                I2C_WriteByte(TPI_SLAVE_ADDR, 0x1A, sysCtrl);
                DelayMS(200);
        }
        I2C_WriteByte(TPI_SLAVE_ADDR, 0x1A, sysCtrl);
        return false;
}

bool ReleaseDDC(uint8_t SysCtrlRegVal)
{
        uint8_t DDCReqTimeout = T_DDC_ACCESS, TPI_ControlImage;
        SysCtrlRegVal &= ~(0x6);
        while (DDCReqTimeout--) {
        	I2C_WriteByte(TPI_SLAVE_ADDR, 0x1A, SysCtrlRegVal);
                TPI_ControlImage = I2C_ReadByte(TPI_SLAVE_ADDR, 0x1A); 
                if (!(TPI_ControlImage & 0x6))
                        return true;
        }
        return false;
}
#if 1 

static unsigned char edid_buf[512];

int tpi_read_edid(void)
{
	int i;
        uint8_t SysCtrlReg;
        int ret, edid_blocks = 0;
        struct i2c_msg msg;
        uint8_t i2c_buff[2];
        uint8_t pbuf[] = {1, 0, 1, 128} ;

        struct i2c_msg paging_msg[] = {
                {
                        .addr = 0x30, .flags = 0, .len = 1, .buf = &pbuf[0],
                },
                {
                        .addr = 0x50, .flags = 0, .len = 1, .buf = &pbuf[1],
                },
                {       //Block-2
                        .addr = 0x50, .flags = I2C_M_RD, .len = 128, .buf = &edid_buf[256],
                },
                {
                        .addr = 0x30, .flags = 0, .len = 1, .buf = &pbuf[2],
                },
                {
                        .addr = 0x50, .flags = 0, .len = 1, .buf = &pbuf[3],
                },
                {       //Block-3
                        .addr = 0x50, .flags = I2C_M_RD, .len = 128, .buf = &edid_buf[384],
                },
        };

#if 0
        DisableTMDS(hdmi);
#else
        uint8_t val;
        val = I2C_ReadByte(TPI_SLAVE_ADDR, 0x1A);
	I2C_WriteByte(TPI_SLAVE_ADDR, 0x1A, val|BIT_4);
#endif

        if (!GetDDC_Access(&SysCtrlReg)) {
                pr_err("%s: DDC bus request failed\n", __func__);
                return false;
        }

        // Block-0
        memset(edid_buf, 0, 512);
	
	for (i=0; i < 512 ; i++) {
        	if (i%16 == 0 ) pr_debug("\n");
	        pr_debug(" %x", edid_buf[i]);
	}

        msg.addr = 0x50;
        msg.flags = 0;
        msg.len = 1;
        msg.buf = edid_buf;
        ret = chip_I2C_TxTransferData(&msg, 1);
        if (ret < 0)
                pr_err("%s: i2c transfer error\n", __func__);
	
	for (i=0; i < 512 ; i++) {
	        if (i%16 == 0 ) pr_debug("\n");
	        	pr_debug(" %x", edid_buf[i]);
	}
        msg.addr = 0x50;
        msg.flags = I2C_M_RD;
        msg.len = 128;
        msg.buf = edid_buf;
        ret = chip_I2C_TxTransferData(&msg, 1);
	for (i=0; i < 512 ; i++) {
	        if (i%16 == 0 ) pr_debug("\n");
        		pr_debug(" %x", edid_buf[i]);
	}

        if (ret < 0) {
                pr_err("%s: i2c transfer error\n", __func__);
                goto end_read_edid;
        } else {
                if (edid_buf[0x7e] <= 3)
                        edid_blocks = edid_buf[0x7e] ;

                pr_debug("EDID blocks = %d\n", edid_blocks);

                if (edid_blocks == 0 ) {
                        goto end_read_edid;
                }
                // Block-1
                msg.addr = 0x50;
                msg.flags = 0;
                msg.len = 1;
                i2c_buff[0] = 128;
                msg.buf = i2c_buff;
                ret = chip_I2C_TxTransferData(&msg, 1);

                msg.addr = 0x50;
                msg.flags = I2C_M_RD;
                msg.len = 128;
                msg.buf = edid_buf + 128;
                ret = chip_I2C_TxTransferData(&msg, 1);
		
		for (i=0; i < 512 ; i++) {
		        if (i % 16 == 0 ) pr_debug("\n");
		        pr_debug(" %x", edid_buf[i]);
		}
        }

        if (edid_blocks > 1) {
                // block 2/3
                chip_I2C_TxTransferData(paging_msg, 3);
                chip_I2C_TxTransferData(&paging_msg[3], 3);
        }

end_read_edid:
        if (!ReleaseDDC(SysCtrlReg)) {
                pr_err("%s: DDC bus release failed\n", __func__);
                return false;
        }

	for (i = 0; i < 512; i += 16)
		PR_DISP_INFO("EDID[%02x-%02x] %02x %02x %02x %02x  "
		"%02x %02x %02x %02x    %02x %02x %02x %02x  "
		"%02x %02x %02x %02x\n", i, i+15,
		edid_buf[i+0], edid_buf[i+1], edid_buf[i+2], edid_buf[i+3],
		edid_buf[i+4], edid_buf[i+5], edid_buf[i+6], edid_buf[i+7],
		edid_buf[i+8], edid_buf[i+9], edid_buf[i+10], edid_buf[i+11],
		edid_buf[i+12], edid_buf[i+13], edid_buf[i+14], edid_buf[i+15]);

        return 0;
}
#endif



StateTableEntry_t  txsteD0_NoConnection[] =
{
    {
          txpseHDMI_CableConnected
        , txpsD0_HDMICableConnectedReadyToSampleRSEN
        , txtfHDMICableConnected
    }
    ,
    {   txpseMHL_Established
        ,txpsD0_Connected
        , txtfMHL_Established|txtfSetDeferRSEN_SamplingTimer
    }
    ,
    {txpseUSB_Established
        , txpsD3
        , txtfUSB_Established
    }
    ,
    {
          txpseCBUS_LockOut
        , txpsD3
        , txtfCBUS_LockOut | txtfGoToD3 | txtfLastEntryThisRow
    }
};

#define TXSF_D0_COMMON     ( txsfExamineCBUSIntrStatus               \
                              | txsfExamineTpiIntr                      \
                              | txsfCheckMSCRequesterAbortHotPlug       \
                              | txsfProcessCBusEvents                   \
                              | TXSF_D0_NO_CONNECTION_DOWNSTREAM_POWER  \
                              | TXSF_CHECK_VIDEO_MODE_CHANGE            \
                              | TXSF_CALL_CEC_HANDLER                   \
                           )

#define TXSF_D0_CONNECTED    ( TXSF_D0_COMMON \
                              | txsfCheckDeferRSEN_SamplingTimerExpired \
                              )

StateTableEntry_t  txsteD0_Connected[] =
{
    {
          txpseUSB_Established
        , txpsD3
        , txtfUSB_Established
    }
    ,
    {
          txpseCBUS_LockOut
        , txpsD3
        , txtfCBUS_LockOut | txtfGoToD3
    }
    ,
    {
        txpseDeferRSEN_SamplingTimerExpired
        ,txpsD0_ConnectedReadyToSampleRSEN
        ,txtfSetDeGlitchTimer | txtfLastEntryThisRow
    }
};
//7.89j  stert
#define TXSF_D0_CONNECTED_READY_TO_SAMPLE_RSEN (TXSF_D0_COMMON | txsfSampleRSEN)
StateTableEntry_t  txsteD0_ConnectedReadyToSampleRSEN[] =
{
{
          txpseHDMI_CableConnected
        , txpsD0_HDMICableConnectedReadyToSampleRSEN
        , txtfHDMICableConnected
    }
    ,
    {
        txpseUSB_Established
        , txpsD3
        , txtfUSB_Established
    }
    ,
    {
          txpseCBUS_LockOut
        , txpsD3
        , txtfCBUS_LockOut | txtfGoToD3
    }
    ,
    {
          txpseRSEN_SampledLow
        , txpsD3
        , txtfGoToD3 | txtfLastEntryThisRow
    }
};

#define TXSF_D3_REGISTERS_INACCESSIBLE ( txsfExaminePinTxInt | txsfCheckForRGND_Rdy)
StateTableEntry_t  txsteD3[] =
{
    {
          txpseRGND_Ready
        , txpsD0_NoConnection
        , txtfInitRGND_Ready
    }
    ,
    {     txpseMHL_Established
        , txpsD0_Connected
        , txtfMHL_Established|txtfSetDeferRSEN_SamplingTimer|txtfLastEntryThisRow
    }
};

#define TXSF_D0_HDMI_CABLE_CONNECTED_COMMON (TXSF_D0_COMMON |  txsfCheckHDCPStatus )
#define TXSF_D0_HDMI_CABLE_CONNECTED        (TXSF_D0_HDMI_CABLE_CONNECTED_COMMON |txsfCheckHDCPTimer| txsfCheckDeferRSEN_SamplingTimerExpired )
StateTableEntry_t txsteD0_HDMICableConnected[]=
{
    {
        txpseHDCPAuthenticated
        ,txpsD0_HDCPAuthenticated
        ,txtfHDCPAuthenticated
    }
    ,
    {
          txpseHDMI_CableDisconnected
        , txpsD0_Connected
        , txtfHDMICableDisconnected
    }
    ,
    {
        txpseDeferRSEN_SamplingTimerExpired
        ,txpsD0_HDMICableConnectedReadyToSampleRSEN
        ,txtfSetDeGlitchTimer
    }
    ,
    {
        txpseUSB_Established
        , txpsD3
        , txtfUSB_Established
    }
    ,
    {
          txpseCBUS_LockOut
        , txpsD3
        , txtfCBUS_LockOut | txtfGoToD3
    }
    ,
    {
          txpseRSEN_SampledLow
        , txpsD3
        , txtfGoToD3 | txtfLastEntryThisRow
    }

};

#define TXSF_D0_HDMI_CABLE_CONNECTED_READY_TO_SAMPLE_RSEN (TXSF_D0_HDMI_CABLE_CONNECTED_COMMON |txsfCheckHDCPTimer| txsfSampleRSEN)
StateTableEntry_t txsteD0_HDMICableConnectedReadyToSampleRSEN[]=
{
    {
        txpseHDCPAuthenticated
        ,txpsD0_HDCPAuthenticatedReadyToSampleRSEN
        ,txtfHDCPAuthenticated
    }
    ,
    {
          txpseHDMI_CableDisconnected
        , txpsD0_ConnectedReadyToSampleRSEN
        , txtfHDMICableDisconnected
    }
    ,
    {
        txpseUSB_Established
        , txpsD3
        , txtfUSB_Established
    }
    ,
    {
          txpseCBUS_LockOut
        , txpsD3
        , txtfCBUS_LockOut | txtfGoToD3
    }
    ,
    {
          txpseRSEN_SampledLow
        , txpsD3
        , txtfGoToD3 | txtfLastEntryThisRow
    }

};

#define TXSF_D0_HDCP_AUTHENTICATED_COMMON (TXSF_D0_HDMI_CABLE_CONNECTED_COMMON )
#define TXSF_D0_HDCP_AUTHENTICATED        (TXSF_D0_HDCP_AUTHENTICATED_COMMON | txsfCheckDeferRSEN_SamplingTimerExpired)
StateTableEntry_t txsteD0_HDCPAuthenticated[]=
{
    {
        txpseHDCPDeAuthenticated
        ,txpsD0_HDMICableConnected
        ,txtfHDCPDeAuthenticated
    }
    ,
    {
          txpseHDMI_CableDisconnected
        , txpsD0_ConnectedReadyToSampleRSEN
        , txtfHDMICableDisconnected
    }
    ,
    {
        txpseDeferRSEN_SamplingTimerExpired
        ,txpsD0_HDCPAuthenticatedReadyToSampleRSEN
        ,txtfSetDeGlitchTimer
    }
    ,
    {
        txpseUSB_Established
        , txpsD3
        , txtfUSB_Established
    }
    ,
    {
          txpseCBUS_LockOut
        , txpsD3
        , txtfCBUS_LockOut | txtfGoToD3
    }
    ,
    {
          txpseRSEN_SampledLow
        , txpsD3
        , txtfGoToD3 | txtfLastEntryThisRow
    }

};

#define TXSF_D0_HDCP_AUTHENTICATED_READY_TO_SAMPLE_RSEN (TXSF_D0_HDCP_AUTHENTICATED_COMMON| txsfSampleRSEN)
StateTableEntry_t txsteD0_HDCPAuthenticatedReadyToSampleRSEN[]=
{
    {
        txpseHDCPDeAuthenticated
        ,txpsD0_HDMICableConnectedReadyToSampleRSEN
        ,txtfHDCPDeAuthenticated
    }
    ,
    {
          txpseHDMI_CableDisconnected
        , txpsD0_ConnectedReadyToSampleRSEN
        , txtfHDMICableDisconnected
    }
    ,
    {
        txpseUSB_Established
        , txpsD3
        , txtfUSB_Established
    }
    ,
    {
          txpseCBUS_LockOut
        , txpsD3
        , txtfCBUS_LockOut | txtfGoToD3
    }
    ,
    {
          txpseRSEN_SampledLow
        , txpsD3
        , txtfGoToD3 | txtfLastEntryThisRow
    }

};

// The order of the following table is CRITICAL.
//  Do NOT insert anything without a corresponding insertion into TxPowerState_e
//   See tdsm.h for the definition of TxPowerState_e
StateTableRowHeader_t TxPowerStateTransitionAndResponseTable[txps_NUM_STATES]=
{
      { TXSF_D0_NO_CONNECTION           ,txsteD0_NoConnection               }
    , { TXSF_D0_CONNECTED               ,txsteD0_Connected                  }
    , { TXSF_D0_CONNECTED_READY_TO_SAMPLE_RSEN              ,txsteD0_ConnectedReadyToSampleRSEN         }
    , { TXSF_D3_REGISTERS_INACCESSIBLE  ,txsteD3                            }
    , { TXSF_D0_HDMI_CABLE_CONNECTED                        ,txsteD0_HDMICableConnected                 }
    , { TXSF_D0_HDMI_CABLE_CONNECTED_READY_TO_SAMPLE_RSEN   ,txsteD0_HDMICableConnectedReadyToSampleRSEN}
    , { TXSF_D0_HDCP_AUTHENTICATED                          ,txsteD0_HDCPAuthenticated                  }
    , { TXSF_D0_HDCP_AUTHENTICATED_READY_TO_SAMPLE_RSEN     ,txsteD0_HDCPAuthenticatedReadyToSampleRSEN }
};
//7.89j  end


ByteQueue_t txEventQueue={0,0,{0}};

PStateTableEntry_t LookupNextState(TxPowerState_e state, uint8_t event)
{
PStateTableEntry_t pStateTableEntry = TxPowerStateTransitionAndResponseTable[state].pStateRow;
uint16_t transitionActionFlags;
    do
    {
        transitionActionFlags = pStateTableEntry->transitionActionFlags;
        if (event == pStateTableEntry->event)
        {
            break;
        }
        pStateTableEntry++;
    }while (!(txtfLastEntryThisRow & transitionActionFlags));
    if (event != pStateTableEntry->event)
    {
        TPI_DEBUG_PRINT(("failure event:%d\n",(int)event));
        return NULL;
    }
    TPI_DEBUG_PRINT(("lookup state:%d event:%d\n",(int)state,(int)event));
    return pStateTableEntry;
}

TxPowerStateEvent_e GetNextTxEvent(void)
{
    if (txEventQueue.head == txEventQueue.tail)
    {
        return txpseNoOp;
    }
    else
    {
    TxPowerStateEvent_e retVal;
        retVal = txEventQueue.queue[txEventQueue.head];
        ADVANCE_QUEUE_PTR(txEventQueue.head)
        return retVal;
    }
}

int PutNextTxEvent(TxPowerStateEvent_e event)
{
    if (txEventQueue.head != txEventQueue.tail)
    {
        // non-empty case
        if (txEventQueue.head < txEventQueue.tail)
        {

            if (txEventQueue.head == RELATIVE_QUEUE_PTR(txEventQueue.tail - 1))
            {
                // full case
                return 0;
            }
        }
        else
        {
            // wrap-around case
            if (RELATIVE_QUEUE_PTR(txEventQueue.head + 1) == txEventQueue.tail)
            {
                // full case
                TPI_DEBUG_PRINT(("queue full\n"));
                return 0;
            }
        }
    }
    // at least one slot available
    txEventQueue.queue[txEventQueue.tail] = event;
    ADVANCE_QUEUE_PTR(txEventQueue.tail)
    return 1;
}

uint8_t ExamineIntr4(void)   //7.89j
{
    uint8_t intr4Image;
    uint8_t intr4IntsHandled=0;
    intr4Image = ReadTxPage0Register(0x74);

	if (intr4Image & BIT_2)
	{
		// MHL Mode Established
        PutNextTxEvent(txpseMHL_Established);
        intr4IntsHandled |= BIT_2;
	}

	if (intr4Image & BIT_3)
	{
		// usb mode established
        PutNextTxEvent(txpseUSB_Established);
        intr4IntsHandled |= BIT_3;
	}

	if (intr4Image & BIT_4)
	{
		// CBus Lockout
        TPI_DEBUG_PRINT(("CBus Lockout\n"));
        PutNextTxEvent(txpseCBUS_LockOut);
        intr4IntsHandled |= BIT_4;
	}

   	if (intr4Image & BIT_6)
   	{
   		// RGND Detection
        PutNextTxEvent(txpseRGND_Ready);
        intr4IntsHandled |= BIT_6;
   	}

    return intr4IntsHandled;
}

void	SiiMhlTxGotMhlIntr( uint8_t intr_0, uint8_t intr_1 )
{
	TPI_DEBUG_PRINT(("MhlTx: INTERRUPT Arrived. %02X, %02X\n", (int) intr_0, (int) intr_1));
}

void	SiiMhlTxGotMhlStatus( uint8_t status_0, uint8_t status_1 )
{
	TPI_DEBUG_PRINT(("MhlTx: STATUS Arrived. %02X, %02X\n", (int) status_0, (int) status_1));
}

void HdcpTimerStart(unsigned long m_sec)
{
    hdcp_timeout_jiffies = jiffies + msecs_to_jiffies(m_sec);
}

bool HdcpTimerExpired(void)
{
    if( time_after(jiffies, hdcp_timeout_jiffies) ) return true;
    return false;
}

void GatherTxEvents(TxPowerState_e txPowerState, uint8_t *pTpiIntsHandled, uint8_t *pCBusIntsHandled, uint8_t *pIntr4IntsHandled)
{
PStateTableRowHeader_t   pStateHeader = &TxPowerStateTransitionAndResponseTable[txPowerState];
    if (pStateHeader->stateActionFlags & txsfCheckDeferRSEN_SamplingTimerExpired)
    {
        if (time_after(jiffies, rsenCheckTimeout))	
        {
            TPI_DEBUG_PRINT(("DTX\n"));
            if (!PutNextTxEvent(txpseDeferRSEN_SamplingTimerExpired))
            {
            }
        }
    }
    if (pStateHeader->stateActionFlags & txsfExamineCBUSIntrStatus)
    {
    byte cBusInt;
    uint8_t cBusIntsHandled=0;
        cBusInt = ReadByteCBUS(0x08);
        {
            if ((BIT_5 | BIT_6) & cBusInt)
            {
   				WriteByteCBUS(0x0D, 0xFF);
   				WriteByteCBUS(0x0E, 0xFF);

   				WriteByteCBUS(0x13, 0x62);
   				WriteByteCBUS(0x12, 0x01);

   				TPI_DEBUG_PRINT(("PVC Abort\n"));
                cBusIntsHandled |= (BIT_5 | BIT_6);
            }

            if (pStateHeader->stateActionFlags & txsfSampleRSEN)
            {
            uint8_t RSEN;
            static uint8_t prevRSEN = BIT_2;
            uint8_t sysStat;
                sysStat = ReadIndexedRegister(0x01, 0x09);
                RSEN = BIT_2 & sysStat;
                if (0 == RSEN)
                {
			        if (time_after(jiffies, rsenCheckTimeout))					
                    {
                        if (0 == prevRSEN)
                        {
                            TPI_DEBUG_PRINT(("RSEN Low sysStat:0x%x\n",(unsigned int)sysStat));
                            if (!PutNextTxEvent(txpseRSEN_SampledLow))
                            {
                            }
                        }
                        else
                        {

					deglitchTimeout = jiffies + HZ/8;
                        }
                    }
                }
                else
                {

					deglitchTimeout = jiffies + HZ/8;
                }
                prevRSEN = RSEN;
            }
        }
        *pCBusIntsHandled = cBusIntsHandled;
        {
        uint8_t cbusInt,i;

        	cbusInt = ReadByteCBUS(0x1E);
        	if( cbusInt )
        	{
        	    TPI_DEBUG_PRINT(("Drv: CBUS INTR_2: %d\n", (int) cbusInt));
        	}
        	if(cbusInt & 0x04)
        	{
        	    TPI_DEBUG_PRINT(("Drv: INT Received: %d\n", (int) cbusInt));

        		SiiMhlTxGotMhlIntr( ReadByteCBUS( 0xA0 ), ReadByteCBUS( 0xA1 ) );

           		for(i = 0xA0; i <= 0xA3; i++)
        		{
        			WriteByteCBUS( i, ReadByteCBUS( i ));
        		}
        	}
        	if(cbusInt & 0x08)
        	{
        	    TPI_DEBUG_PRINT(("Drv: STATUS Received: %d\n", (int) cbusInt));

           		SiiMhlTxGotMhlStatus( ReadByteCBUS( 0xB0), ReadByteCBUS( 0xB1) );

        		for(i = 0xB0; i <= 0xB3; i++)
        		{
        			WriteByteCBUS( i, ReadByteCBUS(  i ));
        		}
        	}
        	if(cbusInt)
        	{
        		WriteByteCBUS(0x1E, cbusInt);

        	    TPI_DEBUG_PRINT(("Drv: Clear CBUS INTR_2: %02X\n", (int) cbusInt));
        	}
        }
    }

    if (pStateHeader->stateActionFlags & txsfExamineTpiIntr)
    {
    uint8_t InterruptStatusImage;
    uint8_t intsHandled=0;

    	InterruptStatusImage = ReadByteTPI(0x3D);

		if (InterruptStatusImage == 0xFF)
		{

			TPI_DEBUG_PRINT(("TP -> NMI Detected\n"));
			TPI_Init();
			HotPlugService();
            intsHandled |= 0xFF;
		}
		if (InterruptStatusImage & BIT_0)
		{

            *pIntr4IntsHandled = ExamineIntr4();  //7.89j
            intsHandled |= InterruptStatusImage & ~BIT_0;
			if( HdcpTimerExpired() ) hdcp_restart();
		}


        if (InterruptStatusImage & 0x10)
        {

            intsHandled |= 0x10;
        }

		if ((hdmiCableConnected == TRUE)
			&& (dsRxPoweredUp == TRUE))

		{
			hdcp_check_status(InterruptStatusImage);
		}
        *pTpiIntsHandled = intsHandled;

    }
    if (pStateHeader->stateActionFlags & txsfCheckMSCRequesterAbortHotPlug)
    {
    uint8_t abortReason;
		abortReason = ReadByteCBUS(0x0D);
		if (((abortReason & BIT_6)
		            >> 6)
		            != hdmiCableConnected)
		{
			if (hdmiCableConnected == TRUE)
			{
		                TPI_DEBUG_PRINT(("\n"));
				OnHdmiCableDisconnected();
			}
			else
			{
				if (MHLCableConnected == TRUE)
				{
					OnHdmiCableConnected();
				}
			}
		}

		if (txsfCheckHDCPTimer & pStateHeader->stateActionFlags)  //7.89j
		{
			if( HdcpTimerExpired() )
			{
				hdcp_restart();
			}
		}

		CheckTxFifoStable();
    }
    if (pStateHeader->stateActionFlags & txsfProcessCBusEvents)
    {
		if(MHLCableConnected)
        {
			CpCbusHandler();
        }
    }

    if (pStateHeader->stateActionFlags & txsfExaminePinTxInt)
    {
        if (mhl_tpi_info->get_int_status() == 0)

		{
            *pIntr4IntsHandled = ExamineIntr4( );    //7.89j
        }
    }

}


void TPI_Poll (void)
{

uint8_t tpiIntsHandled   =0;
uint8_t CBusIntsHandled  =0;
uint8_t intr4IntsHandled =0;

TxPowerStateEvent_e event;
    GatherTxEvents(txPowerState,&tpiIntsHandled,&CBusIntsHandled, &intr4IntsHandled);

	#ifndef HTC_DISABLE_HDCP_WORKAROUND
    if (HDCP_Success)
	hdcpCheckTimeout = 0;

    if (hdmiCableConnected && hdcpCheckTimeout &&
	time_after(jiffies, hdcpCheckTimeout) && !HDCP_Success ) {
        TPI_DEBUG_PRINT(("\nHDCP not success in 5 secs\n\n"));
	hdcpCheckTimeout = 0;
	GoToD3 ();
    }
	#endif //HTC_DISABLE_HDCP_WORKAROUND

    while (txpseNoOp != (event = GetNextTxEvent()))
    {
    PStateTableEntry_t psteTemp;
        psteTemp = LookupNextState(txPowerState, event);
        if (NULL == psteTemp)
        {
            TPI_DEBUG_PRINT(("invalid event:%d for state: %d\n",(int)event,(int)txPowerState));
        }
        else
        {
        uint16_t transitionActionFlags;

            transitionActionFlags = psteTemp->transitionActionFlags;
            if (transitionActionFlags & txtfInitRGND_Ready)
            {
            uint8_t bTemp;
            	TPI_DEBUG_PRINT(("RGND Detection (D3)\n"));
            	WakeUpFromD3();
                bTemp = ReadTxPage0Register(0x99);
            	TPI_DEBUG_PRINT(("[99] -> %02X\n", (int)bTemp));

            	bTemp &= 0x03;
            	WriteTxPage0Register(0x90
            	    , 0x20
            	    | 0x04
            	    | 0x00
            	    | 0x01
            	    );
                if (bTemp == 0x02 || bTemp == 0x01)
            	{
            		ReadModifyWriteTxPage0Register(0x95, 0x20, 0x20);
                    ReadModifyWriteTxPage0Register(0x93, 0xC0 | 0x30, 0x80  | 0x00);
                    CALL_CBUS_WAKEUP_GENERATOR(TPI_SLAVE_ADDR);
            	}
                else if (bTemp == 0x00)
                {
                }
            	else
            	{
            		ReadModifyWriteTxPage0Register(0x95, 0x20, 0x00);
            	}

            }
            if (transitionActionFlags & txtfMHL_Established)
            {
				TPI_DEBUG_PRINT(("MHL Mode Est\n"));
				OnMHLCableConnected(); // this starts TPI Mode
            }
            if (transitionActionFlags & txtfSetDeferRSEN_SamplingTimer)
            {
				rsenCheckTimeout = jiffies + HZ/2;
            }
            if (transitionActionFlags & txtfSetDeGlitchTimer)
            {
				  deglitchTimeout = jiffies + HZ/8;
                TPI_DEBUG_PRINT(("\ncalling CpCbusInitialize\n\n"));
                CpCbusInitialize(mhl_tpi_info);
				hdcp_restart();
            }
            if (transitionActionFlags & txtfUSB_Established)
            {
				TPI_DEBUG_PRINT(("USB Mode Established\n"));
				DelayMS(100);
				GoToD3 ();
            }
            if (transitionActionFlags & txtfCBUS_LockOut)
            {
                if (!bInTpiMode)
                {
                	InitForceUsbIdSwitchOpen();
                	InitReleaseUsbIdSwitchOpen();
                }
                else
                {
                	ForceUsbIdSwitchOpen();
                	ReleaseUsbIdSwitchOpen();
                }
            }
            if (transitionActionFlags & txtfGoToD3)
            {
				GoToD3 ();
            }
            TPI_DEBUG_PRINT(("old state: %d new state: %d\n",(int)txPowerState,(int)psteTemp->newState));
            txPowerState = psteTemp->newState;
        }
    }

    if (CBusIntsHandled)
    {
		WriteByteCBUS(0x08, CBusIntsHandled);		// Clear interrupt(s)
    }

	if (intr4IntsHandled)
	{
    	WriteTxPage0Register( 0x74, intr4IntsHandled);			// Clear interrupts that were handled
	}

    if (tpiIntsHandled)
    {
		WriteByteTPI (0x3D, tpiIntsHandled);	// Clear this interrupt.
    }
}

static void OnHdmiCableConnected (void) {

	TPI_DEBUG_PRINT (("HDMI Connected\n"));

	DelayMS(500);
	hdmiCableConnected = true;
	tpi_read_edid();
	mhl_tpi_info->edid_buf = edid_buf;
	mhl_tpi_info->read_edid();
	mhl_tpi_info->change_hdmi_state(true);

	if (mhl_tpi_info->check_hdmi_sink()) {
		ReadModifyWriteTPI(0x1A, BIT_0, 0x01 );
	} else {
		ReadModifyWriteTPI(0x1A, BIT_0, 0x00 );
	}

	OnDownstreamRxPoweredUp();
	#ifndef HTC_DISABLE_HDCP_WORKAROUND
	hdcpCheckTimeout = jiffies + HZ * 5;
	#endif //HTC_DISABLE_HDCP_WORKAROUND
}

static void OnHdmiCableDisconnected (void) {

	TPI_DEBUG_PRINT (("HDMI Disconnected\n"));

	hdmiCableConnected = false;
	#ifndef HTC_DISABLE_HDCP_WORKAROUND
	hdcpCheckTimeout = 0;
	#endif //HTC_DISABLE_HDCP_WORKAROUND
	mhl_tpi_info->change_hdmi_state(false);
	OnDownstreamRxPoweredDown();
}


static void OnMHLCableConnected(void)
{

	TPI_DEBUG_PRINT(("MHL Connected\n"));
	MHLCableConnected = TRUE;

	if( !bInTpiMode )
	{
	    StartTPI();
	    TxPowerStateD0();
	}

	WriteIndexedRegister(0x01, 0xA0, 0x10);

	ReadModifyWriteTxPage0Register( 0x79, 0x10, 0x00);

	ReadModifyWriteCBUS(0x08, BIT_4, 0x00);	

	TPI_DEBUG_PRINT(("Setting DDC Byte Mode\n"));
	WriteByteCBUS(0x07, DDC_XLTN_TIMEOUT_MAX_VAL | 0x02);
	ReadModifyWriteCBUS(0x44, BIT_1, BIT_1);

	mhl_tpi_info->cable_status_update(true);
}

static void OnDownstreamRxPoweredDown (void) {

	TPI_DEBUG_PRINT (("DSRX -> Powered Down\n"));

	dsRxPoweredUp = false;

	if (HDCP_Started == TRUE)
 	{
		hdcp_off();
 	}

	DisableTMDS();
	ReadModifyWriteTPI(0x1A, BIT_0, 0x00);
}

static void OnDownstreamRxPoweredUp (void) {

	TPI_DEBUG_PRINT(("DSRX -> Powered Up\n"));

	dsRxPoweredUp = true;
	HotPlugService();
}

#undef _MHL_TPI_C_
#endif
