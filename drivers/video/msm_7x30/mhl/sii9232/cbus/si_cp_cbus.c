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

#include <linux/input.h>
#include <mach/debug_display.h>
#include <mach/htc_mhl.h>
#include "../inc/defs.h"
#include "si_api_cbus.h"

struct mhl_info* mhl_cbus_info;
typedef struct
{
    uint8_t rcpKeyCode;
    char   rcpName[30];
} SI_Rc5RcpConversion_t;

ROM SI_Rc5RcpConversion_t RcpSourceToSink[] =
{
    { MHD_RCP_CMD_SELECT,			"Select"},
    { MHD_RCP_CMD_UP,				"Up"},
    { MHD_RCP_CMD_DOWN,				"Down"},
    { MHD_RCP_CMD_LEFT,				"Left"},
    { MHD_RCP_CMD_RIGHT, 			"Right"},

    { MHD_RCP_CMD_ROOT_MENU,		"Root Menu"},

    { MHD_RCP_CMD_EXIT,				"Exit"},

    { MHD_RCP_CMD_NUM_0,			"Num 0"},
    { MHD_RCP_CMD_NUM_1,			"Num 1"},
    { MHD_RCP_CMD_NUM_2,			"Num 2"},
    { MHD_RCP_CMD_NUM_3,			"Num 3"},
    { MHD_RCP_CMD_NUM_4,			"Num 4"},
    { MHD_RCP_CMD_NUM_5,			"Num 5"},
    { MHD_RCP_CMD_NUM_6,			"Num 6"},
    { MHD_RCP_CMD_NUM_7,			"Num 7"},
    { MHD_RCP_CMD_NUM_8,			"Num 8"},
    { MHD_RCP_CMD_NUM_9,			"Num 9"},

    { MHD_RCP_CMD_ENTER,			"Enter"},
    { MHD_RCP_CMD_CLEAR,			"Clear"},

    { MHD_RCP_CMD_CH_UP,			"Channel Up"},
    { MHD_RCP_CMD_CH_DOWN,			"Channel Down"},
    { MHD_RCP_CMD_PRE_CH,			"Previous Channel"},
    { MHD_RCP_CMD_SOUND_SELECT,		"Sound Select"},

    { MHD_RCP_CMD_VOL_UP,			"Volume Up"},
    { MHD_RCP_CMD_VOL_DOWN,			"Volume Down"},
    { MHD_RCP_CMD_MUTE,				"Mute"},
    { MHD_RCP_CMD_PLAY,				"Play"},
    { MHD_RCP_CMD_STOP,			 	"Stop"},
    { MHD_RCP_CMD_PAUSE,			"Pause"},
    { MHD_RCP_CMD_RECORD,			"Record"},
    { MHD_RCP_CMD_REWIND,			"Rewind"},
    { MHD_RCP_CMD_FAST_FWD,			"Fast Fwd"},
    { MHD_RCP_CMD_EJECT,			"Eject"},
    { MHD_RCP_CMD_FWD,				"Forward"},
    { MHD_RCP_CMD_BKWD,				"Backward"},

    { MHD_RCP_CMD_PLAY_FUNC,		"Play Function"},
    { MHD_RCP_CMD_PAUSE_PLAY_FUNC,	"Pause Play Function"},
    { MHD_RCP_CMD_RECORD_FUNC,		"Record Function"},
    { MHD_RCP_CMD_PAUSE_REC_FUNC,	"Pause Record Function"},
    { MHD_RCP_CMD_STOP_FUNC,		"Stop Function"},
    { MHD_RCP_CMD_MUTE_FUNC,		"Mute Function"},
    { MHD_RCP_CMD_UN_MUTE_FUNC,		"Un-Mute Function"},

};


static uint8_t CbusRc5toRcpConvert ( uint8_t keyCode )
 {
    uint8_t i;
    uint8_t retVal = 0xFF;
    uint8_t length = sizeof(RcpSourceToSink)/sizeof(SI_Rc5RcpConversion_t);

    for ( i = 0; i < length ; i++ )
    {
        if ( keyCode == RcpSourceToSink[i].rcpKeyCode )
        {
            retVal = RcpSourceToSink[i].rcpKeyCode;
            TPI_DEBUG_PRINT(("CPCBUS:: Send ----> %s\n", RcpSourceToSink[i].rcpName ));
            break;
        }
    }

    /* Return the new code or 0xFF if not found.    */

    return( ( i == length ) ? 0xFF : retVal );
 }

bool CpCbusSendRcpMessage ( uint8_t channel, uint8_t keyCode )
{
    bool  success;

    success = false;
    for ( ;; )
    {
        TPI_DEBUG_PRINT(( "CPCBUS:: Sending RCP Msg:: %02X keycode to channel %d CBUS\n", (int)keyCode, (int)channel ));
        if ( channel == 0xFF)
        {
            TPI_DEBUG_PRINT(( "\n::::::: Bad channel -- " ));
            break;
        }

        keyCode = CbusRc5toRcpConvert( keyCode );
        if (keyCode == 0xFF)
        {
            TPI_DEBUG_PRINT(( "\n::::::: Bad KeyCode -- " ));
            break;
        }

        success = SI_CbusMscMsgSubCmdSend( channel, MHD_MSC_MSG_RCP, keyCode );
        break;
    }

    if ( !success )
    {
        TPI_DEBUG_PRINT(( "Unable to send command :::::::\n" ));
    }

    return( success );
}

bool CpCbusSendRapMessage ( uint8_t channel, uint8_t actCode )
{
    bool  success;

    success = false;
    for ( ;; )
    {
        TPI_DEBUG_PRINT(( "CPCBUS:: Sending RAP Msg:: %02X action code to channel %d CBUS\n", (int)actCode, (int)channel ));
        if ( channel == 0xFF)
        {
            TPI_DEBUG_PRINT(( "\n::::::: Bad channel -- " ));
            break;
        }

		if ((actCode == MHD_RAP_CMD_POLL) || (actCode == MHD_RAP_CMD_CHG_QUIET) || (actCode != MHD_RAP_CMD_CHG_ACTIVE_PWR))
        {
            success = SI_CbusMscMsgSubCmdSend( channel, MHD_MSC_MSG_RAP, actCode );
            break;
        }
		else
		{
			TPI_DEBUG_PRINT(( "\n::::::: Bad action code -- " ));
            break;
		}
    }

    if ( !success )
    {
        TPI_DEBUG_PRINT(( "Unable to send action command :::::::\n" ));
    }

    return( success );
}

static uint8_t CpProcessRcpMessage ( uint8_t channel, uint8_t rcpData )
{
    uint8_t rcpkStatus = MHD_MSC_MSG_NO_ERROR;

    TPI_DEBUG_PRINT(("RCP Key Code: 0x%02X, channel: 0x%02X\n", (int)rcpData, (int)channel ));

    switch ( rcpData )
    {
        case MHD_RCP_CMD_SELECT:
			TPI_DEBUG_PRINT(( "\n MHD_RCP_CMD_SELECT received %d\n\n", (int)rcpData ));
			mhl_cbus_info->send_keyevent(KEY_ENTER, 0);
			break;
        case MHD_RCP_CMD_UP:
			TPI_DEBUG_PRINT(( "\n MHD_RCP_CMD_UP received %d\n\n", (int)rcpData ));
			mhl_cbus_info->send_keyevent(KEY_UP, 0);
			break;
         case MHD_RCP_CMD_DOWN:
			TPI_DEBUG_PRINT(( "\n MHD_RCP_CMD_DOWN received %d\n\n", (int)rcpData ));
			mhl_cbus_info->send_keyevent(KEY_DOWN, 0);
			break;
        case MHD_RCP_CMD_LEFT:
			TPI_DEBUG_PRINT(( "\n MHD_RCP_CMD_LEFT received %d\n\n", (int)rcpData ));
			mhl_cbus_info->send_keyevent(KEY_LEFT, 0);
			break;
         case MHD_RCP_CMD_RIGHT:
			TPI_DEBUG_PRINT(( "\n MHD_RCP_CMD_RIGHT received %d\n\n", (int)rcpData ));
			mhl_cbus_info->send_keyevent(KEY_RIGHT, 0);
			break;
         case MHD_RCP_CMD_ROOT_MENU:
			TPI_DEBUG_PRINT(( "\n MHD_RCP_CMD_ROOT_MENU received %d\n\n", (int)rcpData ));
			mhl_cbus_info->send_keyevent(KEY_HOME, 0);
			break;
         case MHD_RCP_CMD_EXIT:
			TPI_DEBUG_PRINT(( "\n MHD_RCP_CMD_EXIT received %d\n\n", (int)rcpData ));
			mhl_cbus_info->send_keyevent(KEY_BACK, 0);
			break;
        default:
            rcpkStatus = MHD_MSC_MSG_INEFFECTIVE_KEY_CODE;
            break;
    }

    if ( rcpkStatus == MHD_MSC_MSG_INEFFECTIVE_KEY_CODE )
    {
        TPI_DEBUG_PRINT(("\nKeyCode not recognized or supported.\n\n" ));
    }

    return( rcpkStatus );
}

extern void RAPContentOn (void);
extern void RAPContentOff (void);

static uint8_t CpProcessRapMessage ( uint8_t channel, uint8_t rcpData )
{
    uint8_t rapkStatus = MHD_MSC_MSG_NO_ERROR;

    TPI_DEBUG_PRINT(("RAP Key Code: 0x%02X, channel: 0x%02X\n", (int)rcpData, (int)channel ));

    switch ( rcpData )
    {
        case MHD_RAP_CMD_POLL:
			TPI_DEBUG_PRINT(( "\nPOLL received %d\n\n", (int)rcpData ));
			break;
        case MHD_RAP_CMD_CHG_ACTIVE_PWR:
			TPI_DEBUG_PRINT(( "\nCHANGE TO ACTIVE POWER STATE received %d\n\n", (int)rcpData ));
			RAPContentOn();
			break;
		case MHD_RAP_CMD_CHG_QUIET:
			TPI_DEBUG_PRINT(( "\nCHANGE TO QUIET STATE received %d\n\n", (int)rcpData ));
			RAPContentOff();
			break;
        default:
            rapkStatus = MHD_MSC_MSG_RAP_UNRECOGNIZED_ACT_CODE;
            break;
    }

    if ( rapkStatus == MHD_MSC_MSG_RAP_UNRECOGNIZED_ACT_CODE )
    {
        TPI_DEBUG_PRINT(("\nKeyCode not recognized !! \n\n" ));
    }

    return( rapkStatus );
}

static void CbusConnectionCheck ( uint8_t channel )
{
    static uint8_t busConnected[ 1 ] = {0};


    if ( busConnected[ channel ] != SI_CbusChannelConnected( channel ))
    {
        busConnected[ channel ] = SI_CbusChannelConnected( channel );

        TPI_DEBUG_PRINT(("CPCBUS:: ***Channel: %d,  CBUS %s ****\n", (int)channel , busConnected[ channel ] ? "Connected" : "Unconnected"));
    }
}


static void CpCbusProcessPrivateMessage ( uint8_t channel )
{
    uint8_t     status;
    cbus_req_t  *pCmdRequest;

    pCmdRequest = SI_CbusRequestData( channel );

    switch ( pCmdRequest->command )
    {
        case MHD_MSC_MSG_RCP:

            status = CpProcessRcpMessage( channel, pCmdRequest->offsetData );
            SI_CbusRcpMessageAck( channel, status, pCmdRequest->offsetData );
            break;

        case MHD_MSC_MSG_RCPK:
            break;
        case MHD_MSC_MSG_RCPE:
            break;
        case MHD_MSC_MSG_RAP:
		    status = CpProcessRapMessage( channel, pCmdRequest->offsetData );
            SI_CbusRapMessageAck( channel, status );
            break;
        case MHD_MSC_MSG_RAPK:
            break;
    }
}

void CpCbusHandler ( void )
{
    uint8_t channel, status;

    for ( channel = 0; channel < 1; channel++ )
    {
        SI_CbusUpdateBusStatus( channel );
        CbusConnectionCheck( channel );

        status = SI_CbusHandler( channel );
        if ( status == SUCCESS )
        {

            status = SI_CbusRequestStatus( channel );
            switch ( status )
            {
            case CBUS_REQ_IDLE:
                break;
            case CBUS_REQ_PENDING:
                break;
            case CBUS_REQ_SENT:
                break;

            case CBUS_REQ_RECEIVED:
                CpCbusProcessPrivateMessage( channel );
                break;
            default:
                break;
            }
        }
        else if ( status == ERROR_NO_HEARTBEAT )
        {
            TPI_DEBUG_PRINT(( "Lost CBUS channel %d heartbeat\n", (int)channel ));
        }
        else
        {
            //Lee: Only thing that comes here is interrupt timeout -- is this bad?
        }
    }
}

void CpCbusInitialize ( struct mhl_info* mhl_info )
{

	mhl_cbus_info = mhl_info;
    	SI_CbusInitialize();
}
