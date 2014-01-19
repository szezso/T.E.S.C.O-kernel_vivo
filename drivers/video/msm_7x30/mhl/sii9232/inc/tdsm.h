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
 
#ifndef _TDSM_H_ //{
#define _TDSM_H_
typedef enum
{
      txpsD0_NoConnection                   = 0
    , txpsD0_Connected                      = 1
    , txpsD0_ConnectedReadyToSampleRSEN     = 2
    , txpsD3                                = TX_POWER_STATE_D3
    , txpsD0_HDMICableConnected                          //7.89j
    , txpsD0_HDMICableConnectedReadyToSampleRSEN         //7.89j
    , txpsD0_HDCPAuthenticated                           //7.89j
    , txpsD0_HDCPAuthenticatedReadyToSampleRSEN          //7.89j

    ,txps_NUM_STATES  // this entry must be last
}TxPowerState_e;

typedef enum
{
      txpseNoOp                             = 0
    , txpseRGND_Ready
    , txpseMHL_Established
    , txpseUSB_Established
    , txpseCBUS_LockOut
    , txpseDeferRSEN_SamplingTimerExpired
    , txpseDeGlitchTimerExpired
    , txpseRSEN_SampledLow
    , txpseDDC_Abort
    , txpseSCDT_Change
    , txpseHDMI_CableConnected                 //7.89j
    , txpseHDMI_CableDisconnected              //7.89j
    , txpseNMI_Occurred
    , txpseD2D0_Transition
    , txpseHDCPAuthenticated                  //7.89j
    , txpseHDCPDeAuthenticated                //7.89j

    , txpseNUM_EVENTS // this entry must be last
}TxPowerStateEvent_e;



typedef enum
{
      txtfNull                          = 0x0000
    , txtfLastEntryThisRow              = 0x0001
    , txtfInitRGND_Ready                = 0x0002
    , txtfMHL_Established               = 0x0004
    , txtfUSB_Established               = 0x0008
    , txtfCBUS_LockOut                  = 0x0010
    , txtfSetDeferRSEN_SamplingTimer    = 0x0020    //7.89j
    , txtfSetDeGlitchTimer              = 0x0040
    , txtfGoToD3                        = 0x0080
    , txtfHDMICableConnected            = 0x0100
    , txtfHDMICableDisconnected         = 0x0200
    , txtfHDCPAuthenticated             = 0x0400
    , txtfHDCPDeAuthenticated           = 0x0800
}TxPowerActionFlags_e;

typedef enum
{
      txsfNull                                  = 0x0000

    , txsfExamineCBUSIntrStatus                 = 0x0001
    , txsfExamineTpiIntr                        = 0x0002
    , txsfSampleRSEN                            = 0x0004
    , txsfCheckMHLConnected9290                 = 0x0008
    , txsfCheckHotPlugEventMask                 = 0x0010
    , txsfCheckMSCRequesterAbortHotPlug         = 0x0020
    , txsfProcessCBusEvents                     = 0x0040
    , txsfCheckDownStreamRxPower                = 0x0080
    , txsfCheckForAudioErrorEvent               = 0x0100
    , txsfCheckHDCPStatus                       = 0x0200
    , txsfCheckForVideoModeChange               = 0x0400
    , txsfCallCECHandler                        = 0x0800
    , txsfExaminePinTxInt                       = 0x1000
    , txsfCheckForRGND_Rdy                      = 0x2000
    , txsfCheckDeferRSEN_SamplingTimerExpired   = 0x4000
    , txsfCheckHDCPTimer                        = 0x8000    //7.89j

}TxPowerStateFlags_e;

typedef struct _StateTableEntry_t
{
    TxPowerStateEvent_e   event;
    TxPowerState_e        newState;
    uint16_t              transitionActionFlags;  // note that the size of this structure, on an 8051, should be exactly 4 bytes.
}StateTableEntry_t,*PStateTableEntry_t;

typedef struct _StateTableRowHeader_t
{
    uint16_t stateActionFlags;
    PStateTableEntry_t  pStateRow;
}StateTableRowHeader_t,*PStateTableRowHeader_t;

#define NUM_TX_EVENT_QUEUE_EVENTS 8 /* must be a power of 2*/
#define MAX_TX_EVENT_QUEUE_INDEX ( NUM_TX_EVENT_QUEUE_EVENTS - 1)
#define NUM_TX_EVENT_QUEUE_MODULO_MASK 0x7
typedef struct _ByteQueue_t
{
    uint8_t head;   // queue empty condition head == tail
    uint8_t tail;
    TxPowerStateEvent_e queue[NUM_TX_EVENT_QUEUE_EVENTS];
}ByteQueue_t,*PByteQueue_t;

#define RELATIVE_QUEUE_PTR(ptr) ((ptr) & NUM_TX_EVENT_QUEUE_MODULO_MASK)
#define ADVANCE_QUEUE_PTR(ptr) { ptr++; ptr &= NUM_TX_EVENT_QUEUE_MODULO_MASK; }

#endif //}
