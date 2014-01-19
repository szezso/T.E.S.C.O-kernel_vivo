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
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <mach/debug_display.h>
#include <mach/htc_mhl.h>
#include "../inc/defs.h"
#include "../inc/type_defs.h"
#include "si_api_cbus.h"

#define TPI_SLAVE_ADDR		(mhl_cbus_info->i2c_addr_tpi)
#define CBUS_SLAVE_ADDR		(mhl_cbus_info->i2c_addr_cbus)

static uint8_t msc_return_cmd;
static uint8_t msc_return_value;

static cbusChannelState_t l_cbus[1];
bool dev_cap_regs_ready_bit;
extern struct mhl_info* mhl_cbus_info;
extern uint8_t I2C_ReadByte(uint8_t deviceID, uint8_t offset);
extern void I2C_WriteByte(uint8_t deviceID, uint8_t offset, uint8_t value);
extern void ForceUsbIdSwitchOpen(void);
extern void ReleaseUsbIdSwitchOpen(void);

static uint8_t l_cbusPortOffsets [1] = { 0x00 };

uint8_t SiIRegioCbusRead ( uint16_t regAddr, uint8_t channel )
{
    return(I2C_ReadByte(CBUS_SLAVE_ADDR + l_cbusPortOffsets[channel], regAddr));
}

void SiIRegioCbusWrite ( uint16_t regAddr, uint8_t channel, uint8_t value )
{

    I2C_WriteByte(CBUS_SLAVE_ADDR + l_cbusPortOffsets[channel], regAddr, value);
}

uint8_t SI_CbusRequestStatus ( uint8_t channel )
{
    return( l_cbus[ channel].request[ l_cbus[ channel].activeIndex ].reqStatus );
}

void SI_CbusRequestSetIdle ( uint8_t channel, uint8_t newState )
{
    l_cbus[ channel].request[ l_cbus[ channel].activeIndex ].reqStatus = newState;
}

cbus_req_t *SI_CbusRequestData ( uint8_t channel )
{

    return( &l_cbus[ channel].request[ l_cbus[ channel].activeIndex ] );
}

bool SI_CbusChannelConnected (uint8_t channel)
{
    return( l_cbus[ channel].connected );
}

void cbus_display_registers(int startfrom, int howmany)
{
	int	regnum, regval, i;
	int end_at;

	TPI_DEBUG_PRINT(("\nInternal Registers are:\n"));
	end_at = startfrom + howmany;

	for(regnum = startfrom; regnum <= end_at;)
	{
		for (i = 0 ; i <= 7; i++, regnum++)
		{
			regval = SiIRegioCbusRead(regnum, 0);
			TPI_DEBUG_PRINT(("R%02X = %02X; ", regnum, regval));
		}
		TPI_DEBUG_PRINT(("\n"));
	}

	TPI_DEBUG_PRINT(("\n\n"));
}

static uint8_t CBusProcessConnectionChange ( int channel )
{
    channel = 0;

    TPI_DEBUG_PRINT(("CBUS:: ----Connection Change----\n"));

    return( ERROR_CBUS_TIMEOUT );
}

static uint8_t CBusProcessFailureInterrupts ( uint8_t channel, uint8_t intStatus, uint8_t inResult )
{

    uint8_t result          = inResult;
    uint8_t mscAbortReason  = SUCCESS;
	uint8_t ddcAbortReason  = SUCCESS;

	TPI_DEBUG_PRINT(("CBUS:: CBusProcessFailureInterrupts !!\n"));

    intStatus &=  0x40 | 0x20;

    if ( intStatus )
    {
        result = ERROR_CBUS_ABORT;

		if( intStatus & 0x01 )
		{
			TPI_DEBUG_PRINT(("CBUS Connection Change Detected\n"));
		}
		if( intStatus & 0x04 )
		{
			ddcAbortReason = SiIRegioCbusRead( 0xC0C, channel );
			TPI_DEBUG_PRINT(("CBUS DDC ABORT happened, reason:: %02X\n", (int)(ddcAbortReason)));
                        ForceUsbIdSwitchOpen();
                        ReleaseUsbIdSwitchOpen();
		}

        if ( intStatus & 0x20 )
        {
            mscAbortReason = SiIRegioCbusRead( 0xC0D, channel );

            TPI_DEBUG_PRINT(("CBUS:: MSC Transfer ABORTED. Clearing 0x0D\n"));
            SiIRegioCbusWrite( 0xC0D, channel, 0xFF );
            ForceUsbIdSwitchOpen();
            ReleaseUsbIdSwitchOpen();
        }
        if ( intStatus & 0x40 )
        {
            TPI_DEBUG_PRINT(("CBUS:: MSC Peer sent an ABORT. Clearing 0x0E\n"));
            SiIRegioCbusWrite( 0xC0E, channel, 0xFF );
        }

        // Now display the abort reason.

        if ( mscAbortReason != 0 )
        {
            TPI_DEBUG_PRINT(("CBUS:: Reason for ABORT is ....0x%02X = ", (int)mscAbortReason ));

            if ( mscAbortReason & (0x01 << 0))
            {
                TPI_DEBUG_PRINT(("Requestor MAXFAIL - retry threshold exceeded\n"));
            }
            if ( mscAbortReason & (0x01 << 1))
            {
                TPI_DEBUG_PRINT(("Protocol Error\n"));
            }
            if ( mscAbortReason & (0x01 << 2))
            {
                TPI_DEBUG_PRINT(("Requestor translation layer timeout\n"));
            }
            if ( mscAbortReason & (0x01 << 7))
            {
                TPI_DEBUG_PRINT(("Peer sent an abort\n"));
            }
            if ( mscAbortReason & (0x01 << 3))
            {
                TPI_DEBUG_PRINT(("Undefined opcode\n"));
            }
        }

        /* Clear any failure interrupt that we received.    */

        SiIRegioCbusWrite( 0xC08, channel, intStatus );
    }

    return( result );
}

static uint8_t CBusProcessSubCommand ( int channel, uint8_t vs_cmd, uint8_t vs_data )
{

    l_cbus[ channel].request[ l_cbus[ channel].activeIndex ].command    = vs_cmd;
    l_cbus[ channel].request[ l_cbus[ channel].activeIndex ].offsetData = vs_data;

    switch (vs_cmd)
    {
        case MHD_MSC_MSG_RAP:
				TPI_DEBUG_PRINT(("CBUS:: Received <-- MHL_MSC_MSG_RAP:: cbus state = %02X\n", (int)(l_cbus[ channel].state)));

        case MHD_MSC_MSG_RCP:

            TPI_DEBUG_PRINT(("CBUS:: MHD_MSC_MSG_RCP:: cbus state = %02X\n", (int)(l_cbus[ channel].state)));

            switch ( l_cbus[ channel].state )
            {
                case CBUS_IDLE:
                case CBUS_SENT:
                    l_cbus[ channel].request[ l_cbus[ channel].activeIndex ].reqStatus = CBUS_REQ_RECEIVED;
                    l_cbus[ channel].state = CBUS_RECEIVED;
                    break;
                default:
                    l_cbus[ channel].state = CBUS_IDLE;
                    break;
            }

            break;
        case MHD_MSC_MSG_RCPK:
            TPI_DEBUG_PRINT(("CBUS:: Received <-- MHL_MSC_MSG_RCPK\n"));
            break;

        case MHD_MSC_MSG_RCPE:
            TPI_DEBUG_PRINT(("CBUS:: Illegal Message: MHD_MSC_MSG_ERROR\n"));
            break;
        case MHD_MSC_MSG_RAPK:
            break;
		default:
			break;
    }

    TPI_DEBUG_PRINT(("CBUS:: MSG_MSC CMD:  0x%02X\n", (int)vs_cmd ));
    TPI_DEBUG_PRINT(("CBUS:: MSG_MSC Data: 0x%02X\n", (int)vs_data ));

    return( SUCCESS );
}

static bool CBusWriteCommand ( int channel, cbus_req_t *pReq  )
{
    uint8_t i, startbit;
    bool  success = true;

    TPI_DEBUG_PRINT(("CBUS:: Sending MSC command %02X, %02X, %02X\n", (int)pReq->command, (int)pReq->msgData[0], (int)pReq->msgData[1]));

	SiIRegioCbusWrite( 0xC13, channel, pReq->offsetData);
	SiIRegioCbusWrite( 0xC14, channel, pReq->msgData[0] );

    startbit = 0x00;
    switch ( pReq->command )
    {
		case MHD_SET_INT:
			SiIRegioCbusWrite( 0xC13, channel, pReq->offsetData + 0x20 );
			startbit = (0x01 << 3);
			break;

        case MHD_WRITE_STAT:
            SiIRegioCbusWrite( 0xC13, channel, pReq->offsetData + 0x30 );
            startbit = (0x01 << 3);
            break;

        case MHD_READ_DEVCAP:
            startbit = (0x01 << 2);
            break;

		case MHD_GET_STATE:			// 0x62 - Used for heartbeat
		case MHD_GET_VENDOR_ID:		// 0x63 - for vendor id
		case MHD_SET_HPD:			// 0x64	- Set Hot Plug Detect in follower
		case MHD_CLR_HPD:			// 0x65	- Clear Hot Plug Detect in follower
		case GET_SC1_ERRORCODE:		// 0x69	- Get channel 1 command error code
		case GET_DDC_ERRORCODE:		// 0x6A	- Get DDC channel command error code.
		case GET_MSC_ERRORCODE:		// 0x6B	- Get MSC command error code.
		case GET_SC3_ERRORCODE:		// 0x6D	- Get channel 3 command error code.
			SiIRegioCbusWrite( 0xC13, channel, pReq->command );
            startbit = (0x01 << 0);
            break;

        case MHD_MSC_MSG:
			SiIRegioCbusWrite( 0xC15, channel, pReq->msgData[1] );
			SiIRegioCbusWrite( 0xC13, channel, pReq->command );
            startbit = (0x01 << 1);
            break;

        case MHD_WRITE_BURST:
            SiIRegioCbusWrite( 0xC13, channel, pReq->offsetData + 0x40 );
            SiIRegioCbusWrite( 0xC20, channel, pReq->length );

            for ( i = 0; i < pReq->length; i++ )
            {
                SiIRegioCbusWrite( 0xCC0 + i, channel, pReq->msgData[i] );
            }
            startbit = (0x01 << 4);
            break;

        default:
            success = false;
            break;
    }

    if ( success )
    {
        SiIRegioCbusWrite( 0xC12, channel, startbit );
    }

    return( success );
}

static uint8_t CBusCommandGetNextInQueue( uint8_t channel )
{
	uint8_t   result = STATUS_SUCCESS;

	uint8_t nextIndex = (l_cbus[ channel].activeIndex == (CBUS_MAX_COMMAND_QUEUE - 1)) ?
		 0 : (l_cbus[ channel].activeIndex + 1)	;


	while ( l_cbus[ channel].request[ nextIndex].reqStatus != CBUS_REQ_PENDING )
	{
		if ( nextIndex == l_cbus[ channel].activeIndex )   //searched whole queue, no pending
			return 0;

	    nextIndex = ( nextIndex == (CBUS_MAX_COMMAND_QUEUE - 1)) ?
			 		0 : (nextIndex + 1);
	}

    if ( CBusWriteCommand( channel, &l_cbus[ channel].request[ nextIndex] ) )
    {
        l_cbus[ channel].request[ nextIndex].reqStatus = CBUS_REQ_SENT;
        l_cbus[ channel].activeIndex = nextIndex;
		l_cbus[ channel].state = CBUS_SENT;
    }
    else
    {
        TPI_DEBUG_PRINT(("CBUS:: CBusWriteCommand failed\n" ));
		result = ERROR_WRITE_FAILED;
    }

	return result;

}

static uint8_t CBusCheckInterruptStatus ( uint8_t channel )
{
    uint8_t 	intStatus, result;
	uint8_t     vs_cmd, vs_data;
	uint8_t 	writeBurstLen 	= 0;

    intStatus = SiIRegioCbusRead( 0xC08, channel );
	if ( intStatus & 0x08 )
    {
	    vs_cmd  = SiIRegioCbusRead( 0xC18, channel );
	    vs_data = SiIRegioCbusRead( 0xC19, channel );
	}
	SiIRegioCbusWrite( 0xC08, channel, intStatus );

    result = SUCCESS;
	intStatus &= (~0x80);
    if ( intStatus != 0 )
    {
		if ( intStatus & 0x01 )
        {
            result = CBusProcessConnectionChange( channel );
			SiIRegioCbusWrite( 0xC0A, channel, 0x01 );
        }
        if ( intStatus & 0x10 )
        {

            l_cbus[ channel].state = CBUS_XFR_DONE;

            msc_return_cmd = l_cbus[ channel].request[ l_cbus[ channel].activeIndex ].msgData[0] =
                SiIRegioCbusRead( 0xC16, channel );
			msc_return_value = l_cbus[ channel].request[ l_cbus[ channel].activeIndex ].msgData[1] =
                SiIRegioCbusRead( 0xC17, channel );

            TPI_DEBUG_PRINT(
                ( "CBUS:: Transfer Done - Data returned: %02X %02X\n",
                (int)l_cbus[ channel].request[ l_cbus[ channel].activeIndex ].msgData[0],
				(int)l_cbus[ channel].request[ l_cbus[ channel].activeIndex ].msgData[1]
                ));

            result = SUCCESS;

			writeBurstLen = SiIRegioCbusRead( 0xC20, channel );
			if( writeBurstLen & (0x01 << 6) )
			{
				result = ERROR_NACK_RECEIVED;
			}
			result = CBusProcessFailureInterrupts( channel, intStatus, result );
        }

		if ( intStatus & 0x08 )
        {
            result = CBusProcessSubCommand( channel, vs_cmd, vs_data );
        }
    }
    return( result );
}

bool SI_CbusMscMsgSubCmdSend ( uint8_t channel, uint8_t vsCommand, uint8_t cmdData )
{
	cbus_req_t	req;

	req.command     = MHD_MSC_MSG;
	req.msgData[0]  = vsCommand;
	req.msgData[1]  = cmdData;
    return( SI_CbusWriteCommand( channel, &req  ));
}

bool SI_CbusRcpMessageAck ( uint8_t channel, uint8_t cmdStatus, uint8_t keyCode )
{

    SI_CbusRequestSetIdle( channel, CBUS_REQ_IDLE );
	if(cmdStatus != MHD_MSC_MSG_NO_ERROR)
	{
        SI_CbusMscMsgSubCmdSend( channel, MHD_MSC_MSG_RCPE, cmdStatus );
	}
    return( SI_CbusMscMsgSubCmdSend( channel, MHD_MSC_MSG_RCPK, keyCode ));
}

bool SI_CbusRapMessageAck ( uint8_t channel, uint8_t cmdStatus )
{

    SI_CbusRequestSetIdle( channel, CBUS_REQ_IDLE );
    return( SI_CbusMscMsgSubCmdSend( channel, MHD_MSC_MSG_RAPK, cmdStatus ));
}

bool SI_CbusSendDcapRdyMsg ( uint8_t channel )
{
	cbus_req_t Req;
	bool result = true;

	if( l_cbus[ channel].connected )
	{
		TPI_DEBUG_PRINT(("SI_CbusSendDcapRdyMsg Called!!\n"));

		Req.command = MHL_WRITE_STAT;
		Req.offsetData = 0x00;
		Req.msgData[0] = BIT0;
		result = SI_CbusWriteCommand(0, &Req);

		Req.command = MHL_SET_INT;
		Req.offsetData = 0x00;
		Req.msgData[0] = BIT0;
		result = SI_CbusWriteCommand(0, &Req);

		dev_cap_regs_ready_bit = true;
	}

	return result;
}

uint8_t SI_CbusHandler ( uint8_t channel )
{
    uint8_t result = SUCCESS;


	if ( !dev_cap_regs_ready_bit )
	{
		SI_CbusSendDcapRdyMsg( channel );
	}
    result = CBusCheckInterruptStatus( channel );


    if ( (result == ERROR_NO_HEARTBEAT) || (result == ERROR_NACK_RECEIVED) )
    {
        TPI_DEBUG_PRINT(("SI_CbusHandler:: CBusCheckInterruptStatus returned -->> %02X\n", (int)result));
        return( result );
    }

    switch ( l_cbus[ channel].state )
    {
    case CBUS_IDLE:
		result = CBusCommandGetNextInQueue( channel );				
        break;

    case CBUS_SENT:
        break;

    case CBUS_XFR_DONE:
		l_cbus[ channel].state      = CBUS_IDLE;

        l_cbus[ channel].request[ l_cbus[channel].activeIndex].reqStatus = CBUS_REQ_IDLE;
        break;

    case CBUS_WAIT_RESPONSE:
        break;

    case CBUS_RECEIVED:

        break;

    default:

        l_cbus[ channel].state = CBUS_IDLE;
        result = ERROR_INVALID;
        break;
    }

    return( result );
}

bool SI_CbusWriteCommand ( uint8_t channel, cbus_req_t *pReq  )
{
    uint8_t queueIndex;
    bool  success = false;

	if ( l_cbus[ channel].connected )
	{

    for ( queueIndex = 0; queueIndex < CBUS_MAX_COMMAND_QUEUE; queueIndex++ )
    {
        if ( l_cbus[ channel].request[ queueIndex].reqStatus == CBUS_REQ_IDLE )
        {
            memcpy( &l_cbus[ channel].request[ queueIndex], pReq, sizeof( cbus_req_t ));
            l_cbus[ channel].request[ queueIndex].reqStatus = CBUS_REQ_PENDING;
            success = true;
            break;
        }
    }

    if ( success )
    {
        switch ( l_cbus[ channel].state )
        {
            case CBUS_IDLE:
            case CBUS_RECEIVED:	   
				success = CBusCommandGetNextInQueue( channel );
                break;

            case CBUS_WAIT_RESPONSE:
            case CBUS_SENT:
            case CBUS_XFR_DONE:

                TPI_DEBUG_PRINT(( "CBUS:: Channel State: %02X\n", (int)l_cbus[ channel].state ));
                break;

            default:
                TPI_DEBUG_PRINT(( "CBUS:: Channel State: %02X (illegal)\n", (int)l_cbus[ channel].state ));
                l_cbus[ channel].state = CBUS_IDLE;
                l_cbus[ channel].request[ queueIndex].reqStatus = CBUS_REQ_IDLE;
                success = false;
                break;
        }
    }
    else
    {
        TPI_DEBUG_PRINT(
            ( "CBUS:: Queue full - Request0: %02X Request1: %02X\n",
            (int)l_cbus[ channel].request[ 0].reqStatus,
            (int)l_cbus[ channel].request[ 1].reqStatus
            ));
    }
	}

    return( success );
}

bool SI_CbusUpdateBusStatus ( uint8_t channel )
{
    uint8_t busStatus;

    busStatus = SiIRegioCbusRead( 0xC0A, channel );

    l_cbus[ channel].connected = (busStatus & 0x01) != 0;

    SiIRegioCbusWrite( 0xC0A, channel, busStatus );

    return( l_cbus[ channel].connected );
}

bool SI_CbusInitialize ( void )
{
	uint8_t     channel;
	int	result = SUCCESS;
	uint16_t	devcap_reg;
	int 		regval;

   	memset( &l_cbus, 0, sizeof( l_cbus ));
	dev_cap_regs_ready_bit = false;

	channel = 0;

	devcap_reg = 0xC80;
	SiIRegioCbusWrite(devcap_reg++, channel, 0x04);
	SiIRegioCbusWrite(devcap_reg++, channel, (0x01 << 4) | 0x00);
	SiIRegioCbusWrite(devcap_reg++, channel, 0x02);
	SiIRegioCbusWrite(devcap_reg++, channel, 0);
	SiIRegioCbusWrite(devcap_reg++, channel, 0);
	SiIRegioCbusWrite(devcap_reg++, channel, 0x01 |	0x10);
	SiIRegioCbusWrite(devcap_reg++, channel, 0x01);
	SiIRegioCbusWrite(devcap_reg++, channel, 0);
	SiIRegioCbusWrite(devcap_reg++, channel, (0x01 << 7));
	SiIRegioCbusWrite(devcap_reg++, channel, 0x0F);
	SiIRegioCbusWrite(devcap_reg++, channel, 0x01 | 0x02 | 0x04);
	SiIRegioCbusWrite(devcap_reg++, channel, 0);
	SiIRegioCbusWrite(devcap_reg++, channel, 0);
	SiIRegioCbusWrite(devcap_reg++, channel, 16);
	SiIRegioCbusWrite(devcap_reg++, channel, 0x44);
	SiIRegioCbusWrite(devcap_reg++, channel, 0);

	if(SiIRegioCbusRead(0xC07, channel) == 0xff)
	{
		cbus_display_registers(0, 0x30);
		TPI_DEBUG_PRINT(("cbus initialization failed\n"));
		return ERROR_INIT;
	}

	TPI_DEBUG_PRINT(("cbus_initialize. Poll interval = 50ms. CBUS Connected = %d\n", (int)SI_CbusChannelConnected(channel)));

	SiIRegioCbusWrite(0xC09, channel, (0x01 | 0x08 | 0x10	| 0x20 | 0x40 | 0x80 ));

	regval = SiIRegioCbusRead(0xC31, channel);
	regval = (regval | 0x0C);
	SiIRegioCbusWrite(0xC31, channel, regval);

    regval = SiIRegioCbusRead( 0xC22, channel );
    SiIRegioCbusWrite( 0xC22, channel, (regval & 0x0F));

	SiIRegioCbusWrite(0xC30, channel, 0x01);

	return result;
}

