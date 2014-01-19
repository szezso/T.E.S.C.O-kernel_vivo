/* drivers/video/msm/mhl/sii9232/mhl_access.h - sii9232 MHL driver
 *
 * Copyright (C) 2010 HTC, Inc.
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
 
#ifdef _MHL_ACCESS_C_

#define INDEXED_PAGE_0		0x01
#define INDEXED_PAGE_1		0x02
#define INDEXED_PAGE_2		0x03

#define T_DDC_ACCESS		50

// Indexed access defines
#define TPI_INDEXED_PAGE_REG		0xBC
#define TPI_INDEXED_OFFSET_REG		0xBD
#define TPI_INDEXED_VALUE_REG		0xBE
#define T_DDC_ACCESS    50

#define MHL_I2C_RETRY_COUNT	2

struct mhl_info* mhl_access_info;
extern bool bInTpiMode;

#else //_MHL_ACCESS_C_

extern void mhl_access_init(struct mhl_info* info);
extern uint8_t I2C_ReadByte(uint8_t deviceID, uint8_t offset);
extern void I2C_WriteByte(uint8_t deviceID, uint8_t offset, uint8_t value);
extern uint8_t I2C_WriteBlock(uint8_t deviceID, uint8_t offset, uint8_t *buffer, uint32_t length);
extern uint8_t ReadByteTPI(uint8_t Offset);
extern void WriteByteTPI (uint8_t Offset, uint8_t Data);
extern void ReadModifyWriteCBUS(uint8_t Offset, uint8_t Mask, uint8_t Value);
extern uint8_t ReadByteCBUS (uint8_t Offset);
extern void WriteByteCBUS(uint8_t Offset, uint8_t Data);
extern void ReadModifyWriteIndexedRegister (uint8_t PageNum, uint8_t Offset, uint8_t Mask, uint8_t Data);
extern void WriteIndexedRegister (uint8_t PageNum, uint8_t Offset, uint8_t Data);
extern void ReadSetWriteTPI(uint8_t Offset, uint8_t Pattern);
extern void ReadModifyWriteTPI(uint8_t Offset, uint8_t Mask, uint8_t Data);
extern void ReadClearWriteTPI(uint8_t Offset, uint8_t Pattern);
extern int chip_I2C_TxTransferData(struct i2c_msg *msg, uint32_t length);
extern uint8_t ReadIndexedRegister (uint8_t PageNum, uint8_t Offset);
extern uint8_t ReadTxPage0Register(uint8_t RegOffset);
extern void WriteTxPage0Register(uint8_t RegOffset, uint8_t Value);
extern void ReadModifyWriteTxPage0Register(uint8_t RegOffset, uint8_t Mask, uint8_t Value);


#endif //_MHL_ACCESS_C_
