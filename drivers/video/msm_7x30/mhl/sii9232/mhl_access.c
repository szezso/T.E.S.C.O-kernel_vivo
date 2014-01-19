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
 
 
#ifndef _MHL_ACCESS_C_
#define _MHL_ACCESS_C_
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <mach/htc_mhl.h>
#include <mach/debug_display.h>
#include "mhl_access.h"


// -----------------------------------------------------------------------------
//                             Constant value define
// -----------------------------------------------------------------------------
#define TPI_SLAVE_ADDR		(mhl_access_info->i2c_addr_tpi)
#define CBUS_SLAVE_ADDR		(mhl_access_info->i2c_addr_cbus)


// -----------------------------------------------------------------------------
//                          Global variable declaration
// -----------------------------------------------------------------------------
bool g_bI2CBusError = false;


// -----------------------------------------------------------------------------
//                         Common Routine Implementation
// -----------------------------------------------------------------------------
void mhl_access_init(struct mhl_info* info)
{
	mhl_access_info = info;
}

int chip_I2C_RxData(uint8_t deviceID, char *rxData, uint32_t length)
{
	int err;
	uint8_t loop_i;
	uint8_t slave_addr = deviceID >> 1;
	struct i2c_msg msgs[] = {
		{
		 .addr = slave_addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		 },
		{
		 .addr = slave_addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	if( g_bI2CBusError ){
		// Return directly if met the I2C bus hold problem...
		PR_DISP_WARN("%s() ignore I2C request!\n", __func__ );
		return -EIO;
	}

	mhl_access_info->i2c_client->addr = slave_addr;
	for (loop_i = 0; loop_i < MHL_I2C_RETRY_COUNT; loop_i++) {
		err = i2c_transfer(mhl_access_info->i2c_client->adapter, msgs, 2);

		if (err > 0)
			break;
		if(err == -ETIMEDOUT){
			// This return value means that I2C bus hold by the Sii9232, we need to reset it!
			g_bI2CBusError = true;
			break;
		}

		mdelay(10);
}

	if (loop_i >= MHL_I2C_RETRY_COUNT) {
		PR_DISP_ERR("%s retry over %d\n",
			__func__, MHL_I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

int chip_I2C_TxData(uint8_t deviceID, char *txData, uint32_t length)
{
	int err;
	uint8_t loop_i;
	uint8_t slave_addr = deviceID >> 1;
	struct i2c_msg msg[] = {
		{
		 .addr = slave_addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	if( g_bI2CBusError ){
		// Return directly if met the I2C bus hold problem...
		PR_DISP_WARN("%s() ignore I2C request!\n", __func__ );
		return -EIO;
	}

	mhl_access_info->i2c_client->addr = slave_addr;
	for (loop_i = 0; loop_i < MHL_I2C_RETRY_COUNT; loop_i++) {
		err = i2c_transfer(mhl_access_info->i2c_client->adapter, msg, 1);

		if (err > 0)
			break;
		if(err == -ETIMEDOUT){
			// This return value means that I2C bus hold by the Sii9232, we need to reset it!
			g_bI2CBusError = true;
			break;
		}

		mdelay(10);
	}

	if (loop_i >= MHL_I2C_RETRY_COUNT) {
		PR_DISP_ERR("%s retry over %d\n",
			__func__, MHL_I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

int chip_I2C_TxTransferData(struct i2c_msg *msg, uint32_t length)
{
	int err;
	uint8_t loop_i;

	if( g_bI2CBusError ){
		// Return directly if met the I2C bus hold problem...
		PR_DISP_WARN("%s() ignore I2C request!\n", __func__ );
		return -EIO;
	}

	mhl_access_info->i2c_client->addr = msg->addr;
	for (loop_i = 0; loop_i < MHL_I2C_RETRY_COUNT; loop_i++) {
		err = i2c_transfer(mhl_access_info->i2c_client->adapter, msg, length);

		if (err > 0)
			break;
		if(err == -ETIMEDOUT){
			// This return value means that I2C bus hold by the Sii9232, we need to reset it!
			g_bI2CBusError = true;
			break;
		}

		mdelay(10);
	}

	if (loop_i >= MHL_I2C_RETRY_COUNT) {
		PR_DISP_ERR("%s retry over %d\n",
			__func__, MHL_I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

int chip_I2C_TxBlockData(uint8_t deviceID, uint32_t offset, char *txData, uint32_t length) {
	return i2c_smbus_write_i2c_block_data(mhl_access_info->i2c_client, offset, length, txData);
}



void I2C_WriteByte(uint8_t deviceID, uint8_t offset, uint8_t value)
{
	char buffer[2];

	buffer[0] = offset;
	buffer[1] = value;
	chip_I2C_TxData(deviceID, buffer, 2);
}

uint8_t I2C_ReadByte(uint8_t deviceID, uint8_t offset)
{
	char buffer[2];

	buffer[0] = offset;
	chip_I2C_RxData(deviceID, buffer, 1);
    return buffer[0];
}


uint8_t I2C_WriteBlock(uint8_t deviceID, uint8_t offset, uint8_t *buffer, uint32_t length)
{

	chip_I2C_TxBlockData(deviceID, offset, buffer, length);
	return (0);
}

uint8_t I2C_TransferData(struct i2c_msg *msg, uint32_t length)
{

	chip_I2C_TxTransferData(msg, length);
	return (0);
}

void I2C_ReadModifyWriteByte(uint8_t deviceID, uint8_t offset, uint8_t mask, uint8_t value)
{
    uint8_t tmp;

    tmp = I2C_ReadByte(deviceID, offset);
    tmp &= ~mask;
	tmp |= (value & mask);
    I2C_WriteByte(deviceID, offset, tmp);
}


uint8_t ReadByteTPI (uint8_t Offset) 
{
	return I2C_ReadByte(TPI_SLAVE_ADDR, Offset);
}

void WriteByteTPI (uint8_t Offset, uint8_t Data) 
{
	I2C_WriteByte(TPI_SLAVE_ADDR, Offset, Data);
}

void ReadModifyWriteTPI(uint8_t Offset, uint8_t Mask, uint8_t Data)
{

	uint8_t Temp;

	Temp = ReadByteTPI(Offset);
	Temp &= ~Mask;
	Temp |= (Data & Mask);
	WriteByteTPI(Offset, Temp);
}

void WriteBlockTPI(uint8_t TPI_Offset, uint32_t NBytes, uint8_t * pData)
{
    I2C_WriteBlock(TPI_SLAVE_ADDR, TPI_Offset, pData, NBytes);
}

uint8_t ReadByteCBUS (uint8_t Offset) 
{
	return I2C_ReadByte(CBUS_SLAVE_ADDR, Offset);
}

void WriteByteCBUS(uint8_t Offset, uint8_t Data) 
{
	I2C_WriteByte(CBUS_SLAVE_ADDR, Offset, Data);
}

void ReadModifyWriteCBUS(uint8_t Offset, uint8_t Mask, uint8_t Value) 
{

    uint8_t Temp;

    Temp = ReadByteCBUS(Offset);
    Temp &= ~Mask;
	Temp |= (Value & Mask);
    WriteByteCBUS(Offset, Temp);
}

uint8_t ReadIndexedRegister (uint8_t PageNum, uint8_t Offset) 
{
	WriteByteTPI(TPI_INDEXED_PAGE_REG, PageNum);
	WriteByteTPI(TPI_INDEXED_OFFSET_REG, Offset);
	return ReadByteTPI(TPI_INDEXED_VALUE_REG);
}


void WriteIndexedRegister (uint8_t PageNum, uint8_t Offset, uint8_t Data) 
{
	WriteByteTPI(TPI_INDEXED_PAGE_REG, PageNum);
	WriteByteTPI(TPI_INDEXED_OFFSET_REG, Offset);
	WriteByteTPI(TPI_INDEXED_VALUE_REG, Data);
}

void ReadModifyWriteIndexedRegister (uint8_t PageNum, uint8_t Offset, uint8_t Mask, uint8_t Data) 
{

	uint8_t Temp;

	Temp = ReadIndexedRegister (PageNum, Offset);
	Temp &= ~Mask;
	Temp |= (Data & Mask);
	WriteByteTPI(TPI_INDEXED_VALUE_REG, Temp);
}

uint8_t ReadTxPage0Register(uint8_t RegOffset)
{
	uint8_t retVal;
    if (bInTpiMode)
    {
        retVal = ReadIndexedRegister(INDEXED_PAGE_0, RegOffset);
    }
    else
    {
        retVal = I2C_ReadByte(TPI_SLAVE_ADDR, RegOffset);
    }
    return retVal;
}

void WriteTxPage0Register(uint8_t RegOffset, uint8_t Value)
{
    if (bInTpiMode)
    {
        WriteIndexedRegister(INDEXED_PAGE_0, RegOffset, Value);
    }
    else
    {
        I2C_WriteByte(TPI_SLAVE_ADDR, RegOffset, Value);
    }
}

void ReadModifyWriteTxPage0Register(uint8_t RegOffset, uint8_t Mask, uint8_t Value)
{
    if (bInTpiMode)
    {
        ReadModifyWriteIndexedRegister(INDEXED_PAGE_0, RegOffset, Mask, Value);
    }
    else
    {
        I2C_ReadModifyWriteByte(TPI_SLAVE_ADDR, RegOffset, Mask, Value);
    }
}



void ReadClearWriteTPI(uint8_t Offset, uint8_t Pattern)
{
	uint8_t Tmp;

	Tmp = ReadByteTPI(Offset);
	Tmp &= ~Pattern;
	WriteByteTPI(Offset, Tmp);
}

void ReadSetWriteTPI(uint8_t Offset, uint8_t Pattern)
{
	uint8_t Tmp;

	Tmp = ReadByteTPI(Offset);
	Tmp |= Pattern;
	WriteByteTPI(Offset, Tmp);
}
#undef _MHL_ACCESS_C_
#endif
