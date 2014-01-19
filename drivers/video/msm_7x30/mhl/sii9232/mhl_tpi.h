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
#ifdef _MHL_TPI_C_

struct mhl_info;
void TPI_Poll(void);
uint8_t Status_Query(void);
void EnableTMDS(void);
void DisableTMDS(void);

#else
extern int tpi_hw_init(struct mhl_info* info);
extern void EnableTMDS(void);
extern void DisableTMDS(void);
extern void RestartHDCP(void);

extern void SetAudioMute(uint8_t audioMute);
extern void SetInputColorSpace(uint8_t inputColorSpace);
extern void tpi_clear_interrupt(uint8_t pattern);

extern void TPI_Poll(void);

#endif

#define byte unsigned char

#define TRUE true
#define FALSE false

#define TABLE_DRIVEN_STATE_MACHINE
#define DEV_SUPPORT_HDCP
#define DEV_SUPPORT_EDID

#define T_MONITORING_PERIOD		10
#define SiI9232_PRODUCT_ID      0x9232
#define SiI_DEVICE_ID			0xB0

#define TX_HW_RESET_PERIOD		10

#define HDMI_SLAVE_ADDR 0x96
#define CBUS_SLAVE_ADDR 0xC8

#define INDEXED_PAGE_0		0x01
#define INDEXED_PAGE_1		0x02
#define INDEXED_PAGE_2		0x03
#define T_DDC_ACCESS		50

#define TX_POWER_STATE_MASK					(BIT_1 | BIT_0)
#define TX_POWER_STATE_D0					(0x00)
#define TX_POWER_STATE_D1					(0x01)
#define TX_POWER_STATE_D2					(0x02)
#define TX_POWER_STATE_D3					(0x03)

#define TPI_ENABLE							(0xC7)

#define LOW_BYTE                0x00FF

#define LOW_NIBBLE              0x0F
#define HI_NIBBLE               0xF0

#define MSBIT                   0x80
#define LSBIT                   0x01

#define BIT_0                   0x01
#define BIT_1                   0x02
#define BIT_2                   0x04
#define BIT_3                   0x08
#define BIT_4                   0x10
#define BIT_5                   0x20
#define BIT_6                   0x40
#define BIT_7                   0x80

#define TWO_LSBITS              0x03
#define THREE_LSBITS            0x07
#define FOUR_LSBITS             0x0F
#define FIVE_LSBITS             0x1F
#define SEVEN_LSBITS            0x7F
#define TWO_MSBITS              0xC0
#define EIGHT_BITS              0xFF
#define BYTE_SIZE               0x08
#define BITS_1_0                0x03
#define BITS_2_1                0x06
#define BITS_2_1_0              0x07
#define BITS_3_2                0x0C
#define BITS_4_3_2              0x1C  
#define BITS_5_4                0x30
#define BITS_5_4_3							0x38
#define BITS_6_5                0x60
#define BITS_6_5_4              0x70
#define BITS_7_6                0xC0

#define HOT_PLUG_EVENT          0x01
#define RX_SENSE_EVENT          0x02
#define HOT_PLUG_STATE          0x04
#define RX_SENSE_STATE          0x08

#define AUDIO_ERROR_EVENT       0x10
#define SECURITY_CHANGE_EVENT   0x20
#define V_READY_EVENT           0x40
#define HDCP_CHANGE_EVENT       0x80

#define NON_MASKABLE_INT		0xFF

#define SET_BITS    0xFF
#define CLEAR_BITS  0x00

#define MHD_MAX_CHANNELS                1   // Number of MDHI channels

#define CBUS_CMD_RESPONSE_TIMEOUT       10      // In 100ms increments



