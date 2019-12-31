/******************************************************************************
 * Filename:       common.h
 *
 * Description:    This file contains definitions common to multiple source files, source include control
 *              for labs, debug logging level definitions
 *
 * Copyright (c) 2018, Ekko Tech Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Ekko Tech Limited nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
#ifndef APPLICATION_COMMON_H_
#define APPLICATION_COMMON_H_

/*********************************************************************
 * INCLUDES
 */
#include "labs.h"

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/PIN.h>

#include "icall_ble_api.h"
#include <icall.h>

#include "project_zero.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */
extern snv_config_t gSnvState;
extern uint8_t gIsSnvDirty;
extern uint8_t gShouldIlluminate;
extern uint8_t gPairingState;

/*********************************************************************
 * CONSTANTS
 */

// Global event IDs
#define APP_LUMINANCE_CHANGE_EVT    Event_Id_29

//
// Debug logging levels
//
#define DEBUG_LEVEL_NONE            0
#define DEBUG_LEVEL_INFO            1
#define DEBUG_LEVEL_WARN            2
#define DEBUG_LEVEL_ERROR           3
#define DEBUG_LEVEL                 DEBUG_LEVEL_NONE

//
// General definitions
//
#define OFF                         0
#define ON                          1
#define MSEC_PER_SEC                1000
#define USEC_PER_SEC                1000000
#define USEC_PER_MSEC               1000

/*********************************************************************
 * MACROS
 */
#define member_size(type, member) sizeof(((type *)0)->member)

/*********************************************************************
 * TYPEDEFS
 */
// Types of messages that can be sent to the user application task from other
// tasks or interrupts. Note: Messages from BLE Stack are sent differently.
typedef enum
{
  APP_MSG_SERVICE_WRITE = 0,   /* A characteristic value has been written     */
  APP_MSG_SERVICE_CFG,         /* A characteristic configuration has changed  */
  APP_MSG_UPDATE_CHARVAL,      /* Request from ourselves to update a value    */
  APP_MSG_GAP_STATE_CHANGE,    /* The GAP / connection state has changed      */
  APP_MSG_BUTTON_DEBOUNCED,    /* A button has been debounced with new value  */
  APP_MSG_SEND_PASSCODE,       /* A pass-code/PIN is requested during pairing */
  APP_MSG_PRZ_CONN_EVT,        /* Connection Event finished report            */
} app_msg_types_t;

// Struct for messages sent to the application task
typedef struct
{
  Queue_Elem       _elem;
  app_msg_types_t  type;
  uint8_t          pdu[];
} app_msg_t;

// Struct for messages about characteristic data
typedef struct
{
  uint16_t svcUUID; // UUID of the service
  uint16_t dataLen; //
  uint8_t  paramID; // Index of the characteristic
  uint8_t  data[];  // Flexible array member, extended to malloc - sizeof(.)
} char_data_t;

// Struct for message about sending/requesting passcode from peer.
typedef struct
{
  uint16_t connHandle;
  uint8_t  uiInputs;
  uint8_t  uiOutputs;
  uint32_t   numComparison;
} passcode_req_t;

// Struct for message about button state
typedef struct
{
  PIN_Id   pinId;
  uint8_t  state;
} button_state_t;


/*********************************************************************
 * FUNCTIONS
 */


/*
 * api_function_name - purpose
 *
 *    parameters
 */


#endif /* APPLICATION_COMMON_H_ */
