/******************************************************************************

 @file  project_zero.c

 @brief This file contains the Project Zero sample application implementation

 Group: CMCU, LPRF
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2013-2019, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
 its contributors may be used to endorse or promote products derived
 from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include <string.h>

#include "labs.h"
#include "common.h"

//#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Clock.h>

#include <ti/drivers/PIN.h>
#include <ti/display/Display.h>

#include <xdc/runtime/Diags.h>
#include <uartlog/UartLog.h>

/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"
#include <icall.h>

#include <osal_snv.h>
#include <peripheral.h>
#include <devinfoservice.h>

#include "util.h"

#include "Board.h"
#include "project_zero.h"

// TI provided services
#include "led_service.h"
#include "button_service.h"
#include "data_service.h"

// Custom services
#include "lss_service.h"
#include "lss_handler.h"
#include "als_service.h"
#include "als_handler.h"

/*********************************************************************
 * CONSTANTS
 */

//
// Periodic clock timeout definitions
// Timeout for the first clock tick (in msec)
#define STARTUP_EVT_PERIOD                  500
// How often to perform periodic event (in msec)
#define PERIODIC_EVT_PERIOD                 5000

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL        160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE           GAP_ADTYPE_FLAGS_GENERAL

// Default pass-code used for pairing.
#define DEFAULT_PASSCODE                    000000

// Task configuration
#define PRZ_TASK_PRIORITY                     1

#ifndef PRZ_TASK_STACK_SIZE
#define PRZ_TASK_STACK_SIZE                   800
#endif

// Internal Events for RTOS application
#define PRZ_ICALL_EVT                         ICALL_MSG_EVENT_ID  // Event_Id_31
#define PRZ_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define PRZ_STATE_CHANGE_EVT                  Event_Id_00
#define PRZ_CHAR_CHANGE_EVT                   Event_Id_01
#define PRZ_PERIODIC_EVT                      Event_Id_02
#define PRZ_APP_MSG_EVT                       Event_Id_03

#define PRZ_ALL_EVENTS                       (PRZ_ICALL_EVT        | \
                                              PRZ_QUEUE_EVT        | \
                                              PRZ_STATE_CHANGE_EVT | \
                                              PRZ_CHAR_CHANGE_EVT  | \
                                              PRZ_PERIODIC_EVT     | \
                                              PRZ_APP_MSG_EVT)

// Set the register cause to the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_SET(registerCause) (connectionEventRegisterCauseBitMap |= registerCause )

// Remove the register cause from the registration bit-mask
#define CONNECTION_EVENT_REGISTER_BIT_REMOVE(registerCause) (connectionEventRegisterCauseBitMap &= (~registerCause) )

// Gets whether the current App is registered to the receive connection events
#define CONNECTION_EVENT_IS_REGISTERED (connectionEventRegisterCauseBitMap > 0)

// Gets whether the registerCause was registered to recieve connection event
#define CONNECTION_EVENT_REGISTRATION_CAUSE(registerCause) (connectionEventRegisterCauseBitMap & registerCause )

/*********************************************************************
 * GLOBAL VARIABLES
 */

#ifdef LAB_4      // LAB_4 - Non-Volatile Memory
//
// SNV state
//
uint8_t gIsSnvDirty = false;
snv_config_t gSnvState = { .offOn = 0,
                           .colour = { .red = 0, .green = 0, .blue = 0},
                           // Add other vars here as needed
                         };

#endif /* LAB_4 */

#ifdef LAB_5        // LAB_5 - Analogue Input
uint8_t gShouldIlluminate = false;
#endif /* LAB_5 */

#ifdef LAB_6        // LAB_6 - Pairing and Bonding
// Current pair/bond state of the central-peripheral connection
// All states use the GAP Bond Manager pre-defined states (defined in gapbondmgr.h)
uint8_t gPairingState = GAPBOND_PAIRING_STATE_STARTED;
#endif

//
// Original Project Zero code has these declared as local variables
// Need to promote up to global in order to access across the app
//
// Entity ID globally used to check for source and/or destination of messages
ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
ICall_SyncHandle syncEvent;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Queue object used for application messages.
static Queue_Struct applicationMsgQ;
static Queue_Handle hApplicationMsgQ;

// Task configuration
Task_Struct przTask;
Char przTaskStack[PRZ_TASK_STACK_SIZE];

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
// PRZ task periodic clock
static Clock_Params periodicClockParams;
static Clock_Struct periodicClock;

#endif /* LAB_4 */

// LAB_1_TODO_3
// Modify advertData and scanRspData here
// Tag for TX power is GAP_ADTYPE_POWER_LEVEL
// Tag for Primary Service is GAP_ADTYPE_128BIT_MORE



// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // No scan response data provided.
  0x00 // Placeholder to keep the compiler happy.

};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) or general
  // discoverable mode (advertises indefinitely), depending
  // on the DEFAULT_DISCOVERY_MODE define.
  2,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // Local name
 16,
 GAP_ADTYPE_LOCAL_NAME_COMPLETE,
 'P', 'r', 'o', 'j', 'e', 'c', 't', ' ', 'Z', 'e', 'r', 'o', ' ','R','2',

};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Project Zero R2";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

/* Pin driver handles */
static PIN_Handle buttonPinHandle;
static PIN_Handle ledPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State buttonPinState;
static PIN_State ledPinState;

/*
 * Initial LED pin configuration table
 *   - LEDs Board_LED0 & Board_LED1 are off.
 */
PIN_Config ledPinTable[] = {
  Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    Board_BUTTON0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    Board_BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};

// Clock objects for debouncing the buttons
static Clock_Struct button0DebounceClock;
static Clock_Struct button1DebounceClock;

// State of the buttons
static uint8_t button0State = 0;
static uint8_t button1State = 0;

// Global display handle
Display_Handle dispHandle;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void ProjectZero_init( void );
static void ProjectZero_taskFxn( UArg a0, UArg a1 );

static void user_processApplicationMessage( app_msg_t *pMsg );
static uint8_t ProjectZero_processStackMsg( ICall_Hdr *pMsg );
static uint8_t ProjectZero_processGATTMsg( gattMsgEvent_t *pMsg );

static void ProjectZero_sendAttRsp( void );
static uint8_t ProjectZero_processGATTMsg( gattMsgEvent_t *pMsg );
static void ProjectZero_freeAttRsp( uint8_t status );

static void ProjectZero_connEvtCB( Gap_ConnEventRpt_t *pReport );
static void ProjectZero_processConnEvt( Gap_ConnEventRpt_t *pReport );

static void user_processGapStateChangeEvt( gaprole_States_t newState );
static void user_gapStateChangeCB( gaprole_States_t newState );
static void user_gapBondMgr_passcodeCB( uint8_t *deviceAddr,
                                        uint16_t connHandle, uint8_t uiInputs,
                                        uint8_t uiOutputs,
                                        uint32 numComparison );
static void user_gapBondMgr_pairStateCB( uint16_t connHandle, uint8_t state,
                                         uint8_t status );

static void buttonDebounceSwiFxn( UArg buttonId );
static void user_handleButtonPress( button_state_t *pState );

// Generic callback handlers for value changes in services.
static void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                        uint8_t paramID, uint8_t *pValue,
                                        uint16_t len );
static void user_service_CfgChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                      uint8_t paramID, uint8_t *pValue,
                                      uint16_t len );

// Task context handlers for generated services.
static void user_LedService_ValueChangeHandler( char_data_t *pCharData );
static void user_ButtonService_CfgChangeHandler( char_data_t *pCharData );
static void user_DataService_ValueChangeHandler( char_data_t *pCharData );
static void user_DataService_CfgChangeHandler( char_data_t *pCharData );
//
#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
static void initSnv( uint8_t appId, snv_config_t *pSnvState );
static void saveSnvState( uint8_t appId, snv_config_t *pState );
static void periodicClockTimeoutSwiFxn();
#endif /* LAB_4 */

// Task handler for sending notifications.
static void user_updateCharVal( char_data_t *pCharData );

// Utility functions
static void user_enqueueRawAppMsg( app_msg_types_t appMsgType, uint8_t *pData,
                                   uint16_t len );
static void user_enqueueCharDataMsg( app_msg_types_t appMsgType,
                                     uint16_t connHandle, uint16_t serviceUUID,
                                     uint8_t paramID, uint8_t *pValue,
                                     uint16_t len );
static void buttonCallbackFxn( PIN_Handle handle, PIN_Id pinId );

#if defined(UARTLOG_ENABLE)
static char *Util_getLocalNameStr( const uint8_t *data );
#endif
/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler( uint8 assertCause, uint8 assertSubcause );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t user_gapRoleCBs =
{
  user_gapStateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t user_bondMgrCBs =
{
  (pfnPasscodeCB_t)user_gapBondMgr_passcodeCB, // Passcode callback
  user_gapBondMgr_pairStateCB // Pairing / Bonding state Callback
};

/*
 * Callbacks in the user application for events originating from BLE services.
 */
// LED Service callback handler.
// The type LED_ServiceCBs_t is defined in led_service.h
static LedServiceCBs_t user_LED_ServiceCBs =
{
  .pfnChangeCb    = user_service_ValueChangeCB, // Characteristic value change callback handler
  .pfnCfgChangeCb = NULL, // No notification-/indication enabled chars in LED Service
};

// Button Service callback handler.
// The type Button_ServiceCBs_t is defined in button_service.h
static ButtonServiceCBs_t user_Button_ServiceCBs =
{
  .pfnChangeCb    = NULL, // No writable chars in Button Service, so no change handler.
  .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
};

// Data Service callback handler.
// The type Data_ServiceCBs_t is defined in data_service.h
static DataServiceCBs_t user_Data_ServiceCBs =
{
  .pfnChangeCb    = user_service_ValueChangeCB, // Characteristic value change callback handler
  .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
};

#ifdef LAB_2        // LAB_2 - Service configuration
// LED String Service callback handler.
// The type LssServiceCBs_t is defined in lss_service.h
static LssServiceCBs_t user_Lss_ServiceCBs =
{
 .pfnChangeCb    = user_service_ValueChangeCB, // Characteristic value change callback handler
 .pfnCfgChangeCb = NULL, // No notification-/indication enabled chars in LED Service
};

// Ambient Light Service callback handler.
// The type AlsServiceCBs_t is defined in als_service.h
static AlsServiceCBs_t user_Als_ServiceCBs =
{
 .pfnChangeCb    = user_service_ValueChangeCB, // Characteristic value change callback handler
 .pfnCfgChangeCb = user_service_CfgChangeCB, // Noti/ind configuration callback handler
};

#endif /* LAB_2 */

/*********************************************************************
 * The following typedef and global handle the registration to connection event
 */
typedef enum
{
   NONE_REGISTERED    = 0,
   FOR_ATT_RSP        = 1,
} connectionEventRegisterCause_u;

// Handle the registration and un-registration for the connection event, since only one can be registered.
uint32_t connectionEventRegisterCauseBitMap = NONE_REGISTERED; // See connectionEventRegisterCause_u

/*
 * @brief  Register to receive connection event reports for all the connections
 *
 * @param  connectionEventRegisterCause  Represents the reason for registration
 *
 * @return @ref SUCCESS
 */
bStatus_t ProjectZero_RegistertToAllConnectionEvent(connectionEventRegisterCause_u connectionEventRegisterCause)
{
    bStatus_t status = SUCCESS;

  // In case  there is no registration for the connection event, register for report
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    status = GAP_RegisterConnEventCb(ProjectZero_connEvtCB, GAP_CB_REGISTER, LINKDB_CONNHANDLE_ALL);
  }
  
  if(status == SUCCESS)
  {
    // Add the reason bit to the bitamap.
    CONNECTION_EVENT_REGISTER_BIT_SET(connectionEventRegisterCause);
  }

  return(status);
}

/*
 * @brief   Unregister connection events
 *
 * @param connectionEventRegisterCause represents the reason for registration
 *
 * @return @ref SUCCESS
 *
 */
bStatus_t ProjectZero_UnRegistertToAllConnectionEvent (connectionEventRegisterCause_u connectionEventRegisterCause)
{
  bStatus_t status = SUCCESS;

  CONNECTION_EVENT_REGISTER_BIT_REMOVE(connectionEventRegisterCause);
  
  // In case there are no more subscribers for the connection event then unregister for report
  if (!CONNECTION_EVENT_IS_REGISTERED)
  {
    GAP_RegisterConnEventCb(ProjectZero_connEvtCB, GAP_CB_UNREGISTER, LINKDB_CONNHANDLE_ALL);
  }

  return(status);
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * @brief   Task creation function for the user task.
 *
 * @param   None.
 *
 * @return  None.
 */
void ProjectZero_createTask( void )
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = przTaskStack;
    taskParams.stackSize = PRZ_TASK_STACK_SIZE;
    taskParams.priority = PRZ_TASK_PRIORITY;

    Task_construct(&przTask, ProjectZero_taskFxn, &taskParams, NULL);
}

/*
 * @brief   Called before the task loop and contains application-specific
 *          initialization of the BLE stack, hardware setup, power-state
 *          notification if used, and BLE profile/service initialization.
 *
 * @param   None.
 *
 * @return  None.
 */
static void ProjectZero_init( void )
{
    // ******************************************************************
    // NO STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages via ICall to Stack.
    ICall_registerApp(&selfEntity, &syncEvent);

    Log_info0("Initializing the user task, hardware, BLE stack and services.");

    // Open display. By default this is disabled via the predefined symbol Display_DISABLE_ALL.
    dispHandle = Display_open(Display_Type_UART, NULL);

    // Initialize queue for application messages.
    // Note: Used to transfer control to application thread from e.g. interrupts.
    Queue_construct(&applicationMsgQ, NULL);
    hApplicationMsgQ = Queue_handle(&applicationMsgQ);

    // ******************************************************************
    // Hardware initialization
    // ******************************************************************

    // Open LED pins
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if (!ledPinHandle)
    {
        Log_error0("Error initializing board LED pins");
        Task_exit();
    }

    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if (!buttonPinHandle)
    {
        Log_error0("Error initializing button pins");
        Task_exit();
    }

    // Setup callback for button pins
    if (PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0)
    {
        Log_error0("Error registering button callback function");
        Task_exit();
    }

    // Create the debounce clock objects for Button 0 and Button 1
    Clock_Params clockParams;
    Clock_Params_init(&clockParams);

    // Both clock objects use the same callback, so differentiate on argument
    // given to the callback in Swi context
    clockParams.arg = Board_BUTTON0;

    // Initialize to 50 ms timeout when Clock_start is called.
    // Timeout argument is in ticks, so convert from ms to ticks via tickPeriod.
    Clock_construct(&button0DebounceClock, buttonDebounceSwiFxn,
                    50 * (1000 / Clock_tickPeriod), &clockParams);

    // Second button
    clockParams.arg = Board_BUTTON1;
    Clock_construct(&button1DebounceClock, buttonDebounceSwiFxn,
                    50 * (1000 / Clock_tickPeriod), &clockParams);
#ifdef LAB_3        // LED String Driver Implementation
    lss_Hardware_Init();
#endif

#ifdef LAB_5        // LAB_5 - Analogue Input
    als_Hardware_Init();
#endif

    // ******************************************************************
    // BLE Stack initialization
    // ******************************************************************

    // Setup the GAP Peripheral Role Profile
    uint8_t initialAdvertEnable = TRUE;  // Advertise on power-up

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable. Otherwise wait this long [ms] before advertising again.
    uint16_t advertOffTime = 0; // miliseconds

    // Set advertisement enabled.
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);

    // Configure the wait-time before restarting advertisement automatically
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    // Initialize Scan Response data
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);

    // Initialize Advertisement data
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    Log_info1("Name in advertData array: \x1b[33m%s\x1b[0m",
              (IArg )Util_getLocalNameStr(advertData));

#ifdef LAB_1        // Lab 1 - Apple Inter-operability
    
    // LAB_1_TODO_1
    // Modify code here

    // Override default GAP Role parameters set in peripheral.c
    // GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS: will wait for a parameter update
    // request from the remote before sending
    // GAPROLE_LINK_PARAM_UPDATE_INITIATE_APP_PARAMS: will send desired parameters as
    // soon as a link has been established with the remote
    uint8_t enableUpdateRequest = aaa;
    uint16_t desiredConnTimeout = bbb;    // Units of 10ms - 2000ms / 10ms = 200
    uint16_t desiredSlaveLatency = c;     // No units = 0
    uint16_t desiredMinInterval = dd;    // Units of 1.25ms - 75ms / 1.25ms = 60
    uint16_t desiredMaxInterval = eee; // Units of 1.25ms - 125ms / 1.25ms = 100

    // // Set the Peripheral GAPRole Parameters
    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);

    // Set advertising interval
    uint16_t advInt = fff;     // Units of 625us - 152.5ms / 0.625ms = 244


#else
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;
#endif /* LAB_1 */

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);

    // Set duration of advertisement before stopping in Limited adv mode.
    GAP_SetParamValue(TGAP_LIM_ADV_TIMEOUT, 30); // Seconds

    // ******************************************************************
    // BLE Bond Manager initialization
    // ******************************************************************
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;

#ifdef LAB_6        // LAB_6 - Pairing and Bonding
    uint8_t mitm = xxx
    uint8_t ioCap = yyy
    uint8_t bonding = zzz
#else
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = FALSE;
#endif /* LAB_6 */

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);

    // ******************************************************************
    // BLE Service initialization
    // ******************************************************************

    // Add services to GATT server
    GGS_AddService(GATT_ALL_SERVICES);// GAP
    GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
    DevInfo_AddService();                        // Device Information Service

    // Set the device name characteristic in the GAP Profile
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

#ifdef LAB_1        // Lab 1 - Apple Inter-operability
    
    // LAB_1_TODO_2 - Insert device name write permission here
    

#endif /* LAB_1 */

    // Add services to GATT server and give ID of this task for Indication acks.
    LedService_AddService(selfEntity);
    ButtonService_AddService(selfEntity);
    DataService_AddService(selfEntity);
#ifdef LAB_2        // LAB_2 - Service configuration
 
    // LAB_2_TODO_1 - Add services here

#endif /* LAB_2 */

    // Register callbacks with the generated services that
    // can generate events (writes received) to the application
    LedService_RegisterAppCBs(&user_LED_ServiceCBs);
    ButtonService_RegisterAppCBs(&user_Button_ServiceCBs);
    DataService_RegisterAppCBs(&user_Data_ServiceCBs);
#ifdef LAB_2        // LAB_2 - Service configuration

    // LAB_2_TODO_2 - Register App callbacks here

#endif /* LAB_2 */

    // Placeholder variable for characteristic initialization
    uint8_t initVal[40] = { 0 };
    uint8_t initString[] = "This is a pretty long string, isn't it!";

    // Initialization of characteristics in LED_Service that can provide data.
    LedService_SetParameter(LS_LED0_ID, LS_LED0_LEN, initVal);
    LedService_SetParameter(LS_LED1_ID, LS_LED1_LEN, initVal);

    // Initialization of characteristics in Button_Service that can provide data.
    ButtonService_SetParameter(BS_BUTTON0_ID, BS_BUTTON0_LEN, initVal);
    ButtonService_SetParameter(BS_BUTTON1_ID, BS_BUTTON1_LEN, initVal);

    // Initalization of characteristics in Data_Service that can provide data.
    DataService_SetParameter(DS_STRING_ID, sizeof(initString), initString);
    DataService_SetParameter(DS_STREAM_ID, DS_STREAM_LEN, initVal);

    // Start the stack in Peripheral mode.
    VOID GAPRole_StartDevice(&user_gapRoleCBs);

    // Start Bond Manager
    VOID GAPBondMgr_Register(&user_bondMgrCBs);

    // Register with GAP for HCI/Host messages
    GAP_RegisterForMsgs(selfEntity);

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    // ******************************************************************
    // Application resource initialisation
    // ******************************************************************
    //
    // Service specific resource initialisation
    //
#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
    lss_Resource_Init();
#endif /* LAB_3 */

#ifdef LAB_5        // LAB_5 - Analogue Input
    als_Resource_Init();
#endif /* LAB_5 */

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
     initSnv(SNV_APP_ID, &gSnvState);

    // LAB_4_TODO_2

    // Insert clock construct code here

    // Construct the periodic clock
    // Clock fires after an initial STARTUP_EVT_PERIOD milliseconds and every
    // PERIODIC_EVT_PERIOD milliseconds after that
    // Convert milliseconds to clock ticks by multiplying by (USEC_PER_MSEC / Clock_tickPeriod)

#endif /* LAB_4 */

}

/*
 * @brief   Application task entry point.
 *
 *          Invoked by TI-RTOS when BIOS_start is called. Calls an init function
 *          and enters an infinite loop waiting for messages.
 *
 *          Messages can be either directly from the BLE stack or from user code
 *          like Hardware Interrupt (Hwi) or a callback function.
 *
 *          The reason for sending messages to this task from e.g. Hwi's is that
 *          some RTOS and Stack APIs are not available in callbacks and so the
 *          actions that may need to be taken is dispatched to this Task.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void ProjectZero_taskFxn( UArg a0, UArg a1 )
{
    // Initialize application
    ProjectZero_init();

    // Application main loop
    for (;;)
    {
        uint32_t events;

        // Waits for an event to be posted associated with the calling thread.
        // Note that an event associated with a thread is posted when a
        // message is queued to the message receive queue of the thread
        events = Event_pend(syncEvent, Event_Id_NONE, PRZ_ALL_EVENTS,
        ICALL_TIMEOUT_FOREVER);

        if (events)
        {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            // Check if we got a signal because of a stack message
            if (ICall_fetchServiceMsg(&src, &dest,
                                      (void **) &pMsg) == ICALL_ERRNO_SUCCESS)
            {
                uint8 safeToDealloc = TRUE;

                if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
                {
                    ICall_Stack_Event *pEvt = (ICall_Stack_Event *) pMsg;

                    if (pEvt->signature != 0xffff)
                    {
                        // Process inter-task message
                        safeToDealloc = ProjectZero_processStackMsg(
                                (ICall_Hdr *) pMsg);
                    }
                }

                if (pMsg && safeToDealloc)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            // Process messages sent from another task or another context.
            while (!Queue_empty(hApplicationMsgQ))
            {
                app_msg_t *pMsg = Queue_dequeue(hApplicationMsgQ);

                // Process application-layer message probably sent from ourselves.
                user_processApplicationMessage(pMsg);

                // Free the received message.
                ICall_free(pMsg);
            }

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
            // LAB_4_TODO_2
            // LAB_5_TODO

            // Insert event handling code here

#endif /* LAB_4 */

#ifdef LAB_5        // LAB_5 - Analogue Input
            
            // LAB_5_TODO
            // Insert light level handling code here

#endif /* LAB_5 */

        }
    }
}

/*
 * @brief   Handle application messages
 *
 *          These are messages not from the BLE stack, but from the
 *          application itself.
 *
 *          For example, in a Software Interrupt (Swi) it is not possible to
 *          call any BLE APIs, so instead the Swi function must send a message
 *          to the application Task for processing in Task context.
 *
 * @param   pMsg  Pointer to the message of type app_msg_t.
 *
 * @return  None.
 */
static void user_processApplicationMessage( app_msg_t *pMsg )
{
    char_data_t *pCharData = (char_data_t *) pMsg->pdu;

    switch (pMsg->type)
    {
        case APP_MSG_SERVICE_WRITE: /* Message about received value write */
            /* Call different handler per service */
            switch (pCharData->svcUUID)
            {
#ifdef LAB_2        // LAB_2 - Service configuration
                case LSS_SERVICE_SERV_UUID:
                    user_LssService_ValueChangeHandler(pCharData);
                    break;
                case ALS_SERVICE_SERV_UUID:
                    user_AlsService_ValueChangeHandler(pCharData);
                    break;
#endif /* LAB_2 */
                case LED_SERVICE_SERV_UUID:
                    user_LedService_ValueChangeHandler(pCharData);
                    break;
                case DATA_SERVICE_SERV_UUID:
                    user_DataService_ValueChangeHandler(pCharData);
                    break;

            }
            break;

        case APP_MSG_SERVICE_CFG: /* Message about received CCCD write */
            /* Call different handler per service */
            switch (pCharData->svcUUID)
            {
#ifdef LAB_2    // LAB_2 - Service configuration
                case ALS_SERVICE_SERV_UUID:
                    user_AlsService_CfgChangeHandler(pCharData);
                    break;
#endif /* LAB_2 */
                case BUTTON_SERVICE_SERV_UUID:
                    user_ButtonService_CfgChangeHandler(pCharData);
                    break;
                case DATA_SERVICE_SERV_UUID:
                    user_DataService_CfgChangeHandler(pCharData);
                    break;
            }
            break;

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR, 0);
            break;

        case APP_MSG_UPDATE_CHARVAL: /* Message from ourselves to send  */
            user_updateCharVal(pCharData);
            break;

        case APP_MSG_GAP_STATE_CHANGE: /* Message that GAP state changed  */
            user_processGapStateChangeEvt(*(gaprole_States_t *) pMsg->pdu);
            break;

        case APP_MSG_SEND_PASSCODE: /* Message about pairing PIN request */
        {
            passcode_req_t *pReq = (passcode_req_t *) pMsg->pdu;
            Log_info2("BondMgr Requested passcode. We are %s passcode %06d",
                      (IArg )(pReq->uiInputs ? "Sending" : "Displaying"),
                      DEFAULT_PASSCODE);
            // Send passcode response.
            GAPBondMgr_PasscodeRsp(pReq->connHandle, SUCCESS, DEFAULT_PASSCODE);
        }
            break;

        case APP_MSG_BUTTON_DEBOUNCED: /* Message from swi about pin change */
        {
            button_state_t *pButtonState = (button_state_t *) pMsg->pdu;
            user_handleButtonPress(pButtonState);
        }
            break;

        case APP_MSG_PRZ_CONN_EVT:
        {
            ProjectZero_processConnEvt((Gap_ConnEventRpt_t *) pMsg->pdu);
            break;
        }
    }
}

/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of system/application events deferred to the user Task context.
 *  Invoked from the application Task function above.
 *
 *  Further down you can find the callback handler section containing the
 *  functions that defer their actions via messages to the application task.
 *
 ****************************************************************************
 *****************************************************************************/

/*
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void user_processGapStateChangeEvt( gaprole_States_t newState )
{
    switch (newState)
    {
        case GAPROLE_STARTED:
        {
            uint8_t ownAddress[B_ADDR_LEN];
            uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

            GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

            // use 6 bytes of device address for 8 bytes of system ID value
            systemId[0] = ownAddress[0];
            systemId[1] = ownAddress[1];
            systemId[2] = ownAddress[2];

            // set middle bytes to zero
            systemId[4] = 0x00;
            systemId[3] = 0x00;

            // shift three bytes up
            systemId[7] = ownAddress[5];
            systemId[6] = ownAddress[4];
            systemId[5] = ownAddress[3];

            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN,
                                 systemId);

            // Display device address
            char *cstr_ownAddress = Util_convertBdAddr2Str(ownAddress);
            Log_info1("GAP is started. Our address: \x1b[32m%s\x1b[0m",
                      (IArg )cstr_ownAddress);
        }
            break;

        case GAPROLE_ADVERTISING:
            Log_info0("Advertising");
            break;

        case GAPROLE_CONNECTED:
        {
            uint8_t peerAddress[B_ADDR_LEN];

            GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

            char *cstr_peerAddress = Util_convertBdAddr2Str(peerAddress);
            Log_info1("Connected. Peer address: \x1b[32m%s\x1b[0m",
                      (IArg )cstr_peerAddress);
        }
            break;

        case GAPROLE_CONNECTED_ADV:
            Log_info0("Connected and advertising");
            break;

        case GAPROLE_WAITING:
            Log_info0("Disconnected / Idle");
            break;

        case GAPROLE_WAITING_AFTER_TIMEOUT:
            Log_info0("Connection timed out");
            break;

        case GAPROLE_ERROR:
            Log_info0("Error");
            break;

        default:
            break;
    }
}

/*
 * @brief   Handle a debounced button press or release in Task context.
 *          Invoked by the taskFxn based on a message received from a callback.
 *
 * @see     buttonDebounceSwiFxn
 * @see     buttonCallbackFxn
 *
 * @param   pState  pointer to button_state_t message sent from debounce Swi.
 *
 * @return  None.
 */
static void user_handleButtonPress( button_state_t *pState )
{
    Log_info2(
            "%s %s",
            (IArg)(pState->pinId == Board_BUTTON0?"Button 0":"Button 1"),
            (IArg )(pState->state ?
                    "\x1b[32mpressed\x1b[0m" : "\x1b[33mreleased\x1b[0m"));

    // Update the service with the new value.
    // Will automatically send notification/indication if enabled.
    switch (pState->pinId)
    {
        case Board_BUTTON0:
            ButtonService_SetParameter(BS_BUTTON0_ID, sizeof(pState->state),
                                       &pState->state);
            break;
        case Board_BUTTON1:
            ButtonService_SetParameter(BS_BUTTON1_ID, sizeof(pState->state),
                                       &pState->state);
            break;
    }
}

/*
 * @brief   Handle a write request sent from a peer device.
 *
 *          Invoked by the Task based on a message received from a callback.
 *
 *          When we get here, the request has already been accepted by the
 *          service and is valid from a BLE protocol perspective as well as
 *          having the correct length as defined in the service implementation.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void user_LedService_ValueChangeHandler( char_data_t *pCharData )
{
    static uint8_t pretty_data_holder[16]; // 5 bytes as hex string "AA:BB:CC:DD:EE"
    Util_convertArrayToHexString(pCharData->data, pCharData->dataLen,
                                 pretty_data_holder,
                                 sizeof(pretty_data_holder));

    switch (pCharData->paramID)
    {
        case LS_LED0_ID:
            Log_info3("Value Change msg: %s %s: %s", (IArg )"LED Service",
                      (IArg )"LED0", (IArg )pretty_data_holder);

            // Do something useful with pCharData->data here
            // -------------------------
            // Set the output value equal to the received value. 0 is off, not 0 is on
            PIN_setOutputValue(ledPinHandle, Board_RLED, pCharData->data[0]);
            Log_info2("Turning %s %s", (IArg )"\x1b[31mLED0\x1b[0m",
                      (IArg )(pCharData->data[0] ? "on" : "off"));
            break;

        case LS_LED1_ID:
            Log_info3("Value Change msg: %s %s: %s", (IArg )"LED Service",
                      (IArg )"LED1", (IArg )pretty_data_holder);

            // Do something useful with pCharData->data here
            // -------------------------
            // Set the output value equal to the received value. 0 is off, not 0 is on
            PIN_setOutputValue(ledPinHandle, Board_GLED, pCharData->data[0]);
            Log_info2("Turning %s %s", (IArg )"\x1b[32mLED1\x1b[0m",
                      (IArg )(pCharData->data[0] ? "on" : "off"));
            break;

        default:
            return;
    }
}

/*
 * @brief   Handle a CCCD (configuration change) write received from a peer
 *          device. This tells us whether the peer device wants us to send
 *          Notifications or Indications.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void user_ButtonService_CfgChangeHandler( char_data_t *pCharData )
{
#if defined(UARTLOG_ENABLE)
    // Cast received data to uint16, as that's the format for CCCD writes.
    uint16_t configValue = *(uint16_t *) pCharData->data;
    char *configValString;

    // Determine what to tell the user
    switch (configValue)
    {
        case GATT_CFG_NO_OPERATION:
            configValString = "Noti/Ind disabled";
            break;
        case GATT_CLIENT_CFG_NOTIFY:
            configValString = "Notifications enabled";
            break;
        case GATT_CLIENT_CFG_INDICATE:
            configValString = "Indications enabled";
            break;
    }
#endif
    switch (pCharData->paramID)
    {
        case BS_BUTTON0_ID:
            Log_info3("CCCD Change msg: %s %s: %s", (IArg )"Button Service",
                      (IArg )"BUTTON0", (IArg )configValString);
            // -------------------------
            // Do something useful with configValue here. It tells you whether someone
            // wants to know the state of this characteristic.
            // ...
            break;

        case BS_BUTTON1_ID:
            Log_info3("CCCD Change msg: %s %s: %s", (IArg )"Button Service",
                      (IArg )"BUTTON1", (IArg )configValString);
            // -------------------------
            // Do something useful with configValue here. It tells you whether someone
            // wants to know the state of this characteristic.
            // ...
            break;
    }
}

/*
 * @brief   Handle a write request sent from a peer device.
 *
 *          Invoked by the Task based on a message received from a callback.
 *
 *          When we get here, the request has already been accepted by the
 *          service and is valid from a BLE protocol perspective as well as
 *          having the correct length as defined in the service implementation.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void user_DataService_ValueChangeHandler( char_data_t *pCharData )
{
    // Value to hold the received string for printing via Log, as Log printouts
    // happen in the Idle task, and so need to refer to a global/static variable.
    static uint8_t received_string[DS_STRING_LEN] = { 0 };

    switch (pCharData->paramID)
    {
        case DS_STRING_ID:
            // Do something useful with pCharData->data here
            // -------------------------
            // Copy received data to holder array, ensuring NULL termination.
            memset(received_string, 0, DS_STRING_LEN);
            memcpy(received_string, pCharData->data, DS_STRING_LEN - 1);
            // Needed to copy before log statement, as the holder array remains after
            // the pCharData message has been freed and reused for something else.
            Log_info3("Value Change msg: %s %s: %s", (IArg )"Data Service",
                      (IArg )"String", (IArg )received_string);
            break;

        case DS_STREAM_ID:
            Log_info3(
                    "Value Change msg: Data Service Stream: %02x:%02x:%02x...",
                    (IArg )pCharData->data[0], (IArg )pCharData->data[1],
                    (IArg )pCharData->data[2]);
            // -------------------------
            // Do something useful with pCharData->data here
            break;

        default:
            return;
    }
}

/*
 * @brief   Handle a CCCD (configuration change) write received from a peer
 *          device. This tells us whether the peer device wants us to send
 *          Notifications or Indications.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void user_DataService_CfgChangeHandler( char_data_t *pCharData )
{
#if defined(UARTLOG_ENABLE)
    // Cast received data to uint16, as that's the format for CCCD writes.
    uint16_t configValue = *(uint16_t *) pCharData->data;
    char *configValString;

    // Determine what to tell the user
    switch (configValue)
    {
        case GATT_CFG_NO_OPERATION:
            configValString = "Noti/Ind disabled";
            break;
        case GATT_CLIENT_CFG_NOTIFY:
            configValString = "Notifications enabled";
            break;
        case GATT_CLIENT_CFG_INDICATE:
            configValString = "Indications enabled";
            break;
    }
#endif
    switch (pCharData->paramID)
    {
        case DS_STREAM_ID:
            Log_info3("CCCD Change msg: %s %s: %s", (IArg )"Data Service",
                      (IArg )"Stream", (IArg )configValString);
            // -------------------------
            // Do something useful with configValue here. It tells you whether someone
            // wants to know the state of this characteristic.
            // ...
            break;
    }
}

/*
 * @brief   Process an incoming BLE stack message.
 *
 *          This could be a GATT message from a peer device like acknowledgement
 *          of an Indication we sent, or it could be a response from the stack
 *          to an HCI message that the user application sent.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processStackMsg( ICall_Hdr *pMsg )
{
    uint8_t safeToDealloc = TRUE;

    switch (pMsg->event)
    {
        case GATT_MSG_EVENT:
            // Process GATT message
            safeToDealloc = ProjectZero_processGATTMsg((gattMsgEvent_t *) pMsg);
            break;

        case HCI_GAP_EVENT_EVENT:
        {
            // Process HCI message
            switch (pMsg->status)
            {
                case HCI_COMMAND_COMPLETE_EVENT_CODE:
                    // Process HCI Command Complete Event
                    Log_info0("HCI Command Complete Event received");
                    break;

                default:
                    break;
            }
        }
            break;

        default:
            // do nothing
            break;
    }

    return (safeToDealloc);
}

/*
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t ProjectZero_processGATTMsg( gattMsgEvent_t *pMsg )
{
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
        Log_warning1(
                "Outgoing RF FIFO full. Re-schedule transmission of msg with opcode 0x%02x",
                pMsg->method);

        // No HCI buffer was available. Let's try to retransmit the response
        // on the next connection event.
        if (ProjectZero_RegistertToAllConnectionEvent(FOR_ATT_RSP) == SUCCESS)
        {
            // First free any pending response
            ProjectZero_freeAttRsp(FAILURE);

            // Hold on to the response message for retransmission
            pAttRsp = pMsg;

            // Don't free the response message yet
            return (FALSE);
        }
    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
        // ATT request-response or indication-confirmation flow control is
        // violated. All subsequent ATT requests or indications will be dropped.
        // The app is informed in case it wants to drop the connection.

        // Log the opcode of the message that caused the violation.
        Log_error1("Flow control violated. Opcode of offending ATT msg: 0x%02x",
                   pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
        // MTU size updated
        Log_info1("MTU Size change: %d bytes", pMsg->msg.mtuEvt.MTU);
    }
    else
    {
        // Got an expected GATT message from a peer.
        Log_info1("Recevied GATT Message. Opcode: 0x%02x", pMsg->method);
    }

    // Free message payload. Needed only for ATT Protocol messages
    GATT_bm_free(&pMsg->msg, pMsg->method);

    // It's safe to free the incoming message
    return (TRUE);
}

/*********************************************************************
 * @fn      ProjectZero_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void ProjectZero_processConnEvt( Gap_ConnEventRpt_t *pReport )
{

    if (CONNECTION_EVENT_REGISTRATION_CAUSE(FOR_ATT_RSP))
    {
        // The GATT server might have returned a blePending as it was trying
        // to process an ATT Response. Now that we finished with this
        // connection event, let's try sending any remaining ATT Responses
        // on the next connection event.
        // Try to retransmit pending ATT Response (if any)
        ProjectZero_sendAttRsp();
    }

}

/*
 *  Application error handling functions
 *****************************************************************************/

/*
 * @brief   Send a pending ATT response message.
 *
 *          The message is one that the stack was trying to send based on a
 *          peer request, but the response couldn't be sent because the
 *          user application had filled the TX queue with other data.
 *
 * @param   none
 *
 * @return  none
 */
static void ProjectZero_sendAttRsp( void )
{
    // See if there's a pending ATT Response to be transmitted
    if (pAttRsp != NULL)
    {
        uint8_t status;

        // Increment retransmission count
        rspTxRetry++;

        // Try to retransmit ATT response till either we're successful or
        // the ATT Client times out (after 30s) and drops the connection.
        status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method,
                              &(pAttRsp->msg));
        if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
        {
            // Disable connection event end notice
            ProjectZero_UnRegistertToAllConnectionEvent(FOR_ATT_RSP);

            // We're done with the response message
            ProjectZero_freeAttRsp(status);
        }
        else
        {
            // Continue retrying
            Log_warning2("Retrying message with opcode 0x%02x. Attempt %d",
                         pAttRsp->method, rspTxRetry);
        }
    }
}

/*
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void ProjectZero_freeAttRsp( uint8_t status )
{
    // See if there's a pending ATT response message
    if (pAttRsp != NULL)
    {
        // See if the response was sent out successfully
        if (status == SUCCESS)
        {
            Log_info2("Sent message with opcode 0x%02x. Attempt %d",
                      pAttRsp->method, rspTxRetry);
        }
        else
        {
            Log_error2("Gave up message with opcode 0x%02x. Status: %d",
                       pAttRsp->method, status);

            // Free response payload
            GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
        }

        // Free response message
        ICall_freeMsg(pAttRsp);

        // Reset our globals
        pAttRsp = NULL;
        rspTxRetry = 0;
    }
}

/******************************************************************************
 *****************************************************************************
 *
 *  Handlers of direct system callbacks.
 *
 *  Typically enqueue the information or request as a message for the
 *  application Task for handling.
 *
 ****************************************************************************
 *****************************************************************************/

/*********************************************************************
 * @fn      ProjectZero_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void ProjectZero_connEvtCB( Gap_ConnEventRpt_t *pReport )
{
    // Enqueue the event for processing in the app context.
    user_enqueueRawAppMsg(APP_MSG_PRZ_CONN_EVT, (uint8_t *) pReport,
                          sizeof(pReport));
    ICall_free(pReport);
}

/*
 *  Callbacks from the Stack Task context (GAP or Service changes)
 *****************************************************************************/

/**
 * Callback from GAP Role indicating a role state change.
 */
static void user_gapStateChangeCB( gaprole_States_t newState )
{
    Log_info1("(CB) GAP State change: %d, Sending msg to app.",
              (IArg )newState);
    user_enqueueRawAppMsg(APP_MSG_GAP_STATE_CHANGE, (uint8_t *) &newState,
                          sizeof(newState));
}

/*
 * @brief   Passcode callback.
 *
 * @param   connHandle - connection handle
 * @param   uiInputs   - input passcode?
 * @param   uiOutputs  - display passcode?
 * @param   numComparison - numeric comparison value
 *
 * @return  none
 */
static void user_gapBondMgr_passcodeCB( uint8_t *deviceAddr,
                                        uint16_t connHandle, uint8_t uiInputs,
                                        uint8_t uiOutputs,
                                        uint32 numComparison )
{
    passcode_req_t req = { .connHandle = connHandle, .uiInputs = uiInputs,
                           .uiOutputs = uiOutputs, .numComparison =
                                   numComparison };

    // Defer handling of the passcode request to the application, in case
    // user input is required, and because a BLE API must be used from Task.
    user_enqueueRawAppMsg(APP_MSG_SEND_PASSCODE, (uint8_t *) &req, sizeof(req));
}

/*
 * @brief   Pairing state callback.
 *
 * @param   connHandle - connection handle
 * @param   state      - pairing state
 * @param   status     - pairing status
 *
 * @return  none
 */
static void user_gapBondMgr_pairStateCB( uint16_t connHandle, uint8_t state,
                                         uint8_t status )
{
    if (state == GAPBOND_PAIRING_STATE_STARTED)
    {
        Log_info0("Pairing started");
        // LAB_6 - Pairing and Bonding add global state update here
    }
    else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
    {
        if (status == SUCCESS)
        {
            Log_info0("Pairing completed successfully.");
            // LAB_6 - Pairing and Bonding add global state update here
        }
        else
        {
            Log_error1("Pairing failed. Error: %02x", status);
        }
    }
    else if (state == GAPBOND_PAIRING_STATE_BOND_SAVED)
    {
        if (status == SUCCESS)
        {
            Log_info0("Bond info saved to NV");
            // LAB_6 - Pairing and Bonding add global state update here
        }
        else
        {
            Log_error1("Saving bond info to NV failed. Error: %02x", status);
        }
    }
    else if (state == GAPBOND_PAIRING_STATE_BONDED)
    {
        if (status == SUCCESS)
        {
            Log_info0("Re-established pairing from stored bond info.");
            // LAB_6 - Pairing and Bonding add global state update here
        }
        else {
            Log_error1("Bonding re-establishment failed. Error: %02x", status);
        }
    }
}

/**
 * Callback handler for characteristic value changes in services.
 */
static void user_service_ValueChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                        uint8_t paramID, uint8_t *pValue,
                                        uint16_t len )
{
    // See the service header file to compare paramID with characteristic.
    Log_info2("(CB) Characteristic value change: svc(0x%04x) paramID(%d). "
              "Sending msg to app.",
              (IArg )svcUuid, (IArg )paramID);
    user_enqueueCharDataMsg(APP_MSG_SERVICE_WRITE, connHandle, svcUuid, paramID,
                            pValue, len);
}

/**
 * Callback handler for characteristic configuration changes in services.
 */
static void user_service_CfgChangeCB( uint16_t connHandle, uint16_t svcUuid,
                                      uint8_t paramID, uint8_t *pValue,
                                      uint16_t len )
{
    Log_info2("(CB) Char config change: svc(0x%04x) paramID(%d). "
              "Sending msg to app.",
              (IArg )svcUuid, (IArg )paramID);
    user_enqueueCharDataMsg(APP_MSG_SERVICE_CFG, connHandle, svcUuid, paramID,
                            pValue, len);
}

/*
 *  Callbacks from Swi-context
 *****************************************************************************/

/*
 * @brief  Callback from Clock module on timeout
 *
 *         Determines new state after debouncing
 *
 * @param  buttonId    The pin being debounced
 */
static void buttonDebounceSwiFxn( UArg buttonId )
{
    // Used to send message to app
    button_state_t buttonMsg = { .pinId = buttonId };
    uint8_t sendMsg = FALSE;

    // Get current value of the button pin after the clock timeout
    uint8_t buttonPinVal = PIN_getInputValue(buttonId);

    // Set interrupt direction to opposite of debounced state
    // If button is now released (button is active low, so release is high)
    if (buttonPinVal)
    {
        // Enable negative edge interrupts to wait for press
        PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_NEGEDGE);
    }
    else
    {
        // Enable positive edge interrupts to wait for relesae
        PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_POSEDGE);
    }

    switch (buttonId)
    {
        case Board_BUTTON0:
            // If button is now released (buttonPinVal is active low, so release is 1)
            // and button state was pressed (buttonstate is active high so press is 1)
            if (buttonPinVal && button0State)
            {
                // Button was released
                buttonMsg.state = button0State = 0;
                sendMsg = TRUE;
            }
            else if (!buttonPinVal && !button0State)
            {
                // Button was pressed
                buttonMsg.state = button0State = 1;
                sendMsg = TRUE;
            }
            break;

        case Board_BUTTON1:
            // If button is now released (buttonPinVal is active low, so release is 1)
            // and button state was pressed (buttonstate is active high so press is 1)
            if (buttonPinVal && button1State)
            {
                // Button was released
                buttonMsg.state = button1State = 0;
                sendMsg = TRUE;
            }
            else if (!buttonPinVal && !button1State)
            {
                // Button was pressed
                buttonMsg.state = button1State = 1;
                sendMsg = TRUE;
            }
            break;
    }

    if (sendMsg == TRUE)
    {
        user_enqueueRawAppMsg(APP_MSG_BUTTON_DEBOUNCED, (uint8_t *) &buttonMsg,
                              sizeof(buttonMsg));
    }
}

/*
 * @fn      periodicClockTimeoutSwiFxn
 *
 * @brief   Callback from the periodic clock on timeout
 *
 * @param   arg - generic argument
 *
 * @return  none
 */
#ifdef LAB_4      // LAB_4 - Non-Volatile Memory
static void periodicClockTimeoutSwiFxn( UArg arg ) {

    // LAB_4_TODO_2

    // Insert periodic clock handling code here
    
}
#endif /* LAB_4 */

/*
 *  Callbacks from Hwi-context
 *****************************************************************************/

/*
 * @brief  Callback from PIN driver on interrupt
 *
 *         Sets in motion the debouncing.
 *
 * @param  handle    The PIN_Handle instance this is about
 * @param  pinId     The pin that generated the interrupt
 */
static void buttonCallbackFxn( PIN_Handle handle, PIN_Id pinId )
{
    Log_info1("Button interrupt: %s",
              (IArg)((pinId == Board_BUTTON0)?"Button 0":"Button 1"));

    // Disable interrupt on that pin for now. Re-enabled after debounce.
    PIN_setConfig(handle, PIN_BM_IRQ, pinId | PIN_IRQ_DIS);

    // Start debounce timer
    switch (pinId)
    {
        case Board_BUTTON0:
            Clock_start(Clock_handle(&button0DebounceClock));
            break;
        case Board_BUTTON1:
            Clock_start(Clock_handle(&button1DebounceClock));
            break;
    }
}

/******************************************************************************
 *****************************************************************************
 *
 *  Utility functions
 *
 ****************************************************************************
 *****************************************************************************/

#ifdef LAB_4        // LAB_4 - Non-volatile memory
/*
 * @fn      saveSnvState
 *
 * @brief   Writes gSnvState in memory to FLASH if gSnvState has been written to since
 *          the last write
 *
 * @param   appId - the ID for the SNV structure. Must be in the range 0x80 to 0x8F
 * @param   len - the number of bytes to write
 * @param   pData - pointer to the source data
 *
 * @return  none
 */
static void saveSnvState( uint8_t appId, snv_config_t *pState )
{
    // LAB_4_TODO_2

    // Insert handler code here

    //
    // Write out snv structure to SNV if it has changed
    //

}
#endif /* LAB_4 */

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
/*
 * @fn      initSnv
 *
 * @brief   Initialises SNV memory
 *
 * @discussion  SNV state held in memory is initialised from SNV state held in FLASH if
 *          the SNV state in FLASH is valid. SNV state in memory is initialised statically
 *          to default settings. This static state is overwritten when SNV state is successfully
 *          read from FLASH.
 *          Each time that a new firmware image is uploaded, SNV state in FLASH is erased and
 *          a subsequent read will fail. In the event of a read failure, SNV state in FLASH is
 *          initialised with the statically defined SNV state in memory.
 *          In normal operation, after a power cycle or a reset, the SNV state in FLASH is
 *          loaded into memory.
 *
 * @param   appId - the ID for the SNV structure. Must be in the range 0x80 to 0x8F
 *          len - the number of bytes to read from FLASH
 *          pData - pointer to the memory area to load the FLASH data
 *
 * @return  none
 */
static void initSnv(uint8_t appId, snv_config_t *pSnvState)
{
    // LAB_4_TODO_1

    // Insert SNV initialisation code here


        // SNV read OK, set characteristics

    
        // Most likely new firmware uploaded
        // Get default values from characteristics; write to SNV


}
#endif /* LAB_4 */

/*
 * @brief  Generic message constructor for characteristic data.
 *
 *         Sends a message to the application for handling in Task context where
 *         the message payload is a char_data_t struct.
 *
 *         From service callbacks the appMsgType is APP_MSG_SERVICE_WRITE or
 *         APP_MSG_SERVICE_CFG, and functions running in another context than
 *         the Task itself, can set the type to APP_MSG_UPDATE_CHARVAL to
 *         make the user Task loop invoke user_updateCharVal function for them.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @param  connHandle    GAP Connection handle of the relevant connection
 * @param  serviceUUID   16-bit part of the relevant service UUID
 * @param  paramID       Index of the characteristic in the service
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
static void user_enqueueCharDataMsg( app_msg_types_t appMsgType,
                                     uint16_t connHandle, uint16_t serviceUUID,
                                     uint8_t paramID, uint8_t *pValue,
                                     uint16_t len )
{
    // Called in Stack's Task context, so can't do processing here.
    // Send message to application message queue about received data.
    uint16_t readLen = len; // How much data was written to the attribute

    // Allocate memory for the message.
    // Note: The pCharData message doesn't have to contain the data itself, as
    //       that's stored in a variable in the service implementation.
    //
    //       However, to prevent data loss if a new value is received before the
    //       service's container is read out via the GetParameter API is called,
    //       we copy the characteristic's data now.
    app_msg_t *pMsg = ICall_malloc(
            sizeof(app_msg_t) + sizeof(char_data_t) + readLen);

    if (pMsg != NULL)
    {
        pMsg->type = appMsgType;

        char_data_t *pCharData = (char_data_t *) pMsg->pdu;
        pCharData->svcUUID = serviceUUID; // Use 16-bit part of UUID.
        pCharData->paramID = paramID;
        // Copy data from service now.
        memcpy(pCharData->data, pValue, readLen);
        // Update pCharData with how much data we received.
        pCharData->dataLen = readLen;
        // Enqueue the message using pointer to queue node element.
        Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
        // Let application know there's a message.
        Event_post(syncEvent, PRZ_APP_MSG_EVT);
    }
}

/*
 * @brief  Generic message constructor for application messages.
 *
 *         Sends a message to the application for handling in Task context.
 *
 * @param  appMsgType    Enumerated type of message being sent.
 * @oaram  *pValue       Pointer to characteristic value
 * @param  len           Length of characteristic data
 */
static void user_enqueueRawAppMsg( app_msg_types_t appMsgType, uint8_t *pData,
                                   uint16_t len )
{
    // Allocate memory for the message.
    app_msg_t *pMsg = ICall_malloc(sizeof(app_msg_t) + len);

    if (pMsg != NULL)
    {
        pMsg->type = appMsgType;

        // Copy data into message
        memcpy(pMsg->pdu, pData, len);

        // Enqueue the message using pointer to queue node element.
        Queue_enqueue(hApplicationMsgQ, &pMsg->_elem);
//    // Let application know there's a message.
        Event_post(syncEvent, PRZ_APP_MSG_EVT);
    }
}

/*
 * @brief  Convenience function for updating characteristic data via char_data_t
 *         structured message.
 *
 * @note   Must run in Task context in case BLE Stack APIs are invoked.
 *
 * @param  *pCharData  Pointer to struct with value to update.
 */
static void user_updateCharVal( char_data_t *pCharData )
{
    switch (pCharData->svcUUID)
    {
        case LED_SERVICE_SERV_UUID:
            LedService_SetParameter(pCharData->paramID, pCharData->dataLen,
                                    pCharData->data);
            break;

        case BUTTON_SERVICE_SERV_UUID:
            ButtonService_SetParameter(pCharData->paramID, pCharData->dataLen,
                                       pCharData->data);
            break;

    }
}

/*
 * @brief   Convert {0x01, 0x02} to "01:02"
 *
 * @param   src - source byte-array
 * @param   src_len - length of array
 * @param   dst - destination string-array
 * @param   dst_len - length of array
 *
 * @return  array as string
 */
char *Util_convertArrayToHexString( uint8_t const *src, uint8_t src_len,
                                    uint8_t *dst, uint8_t dst_len )
{
    char hex[] = "0123456789ABCDEF";
    uint8_t *pStr = dst;
    uint8_t avail = dst_len - 1;

    memset(dst, 0, avail);

    while (src_len && avail > 3)
    {
        if (avail < dst_len - 1)
        {
            *pStr++ = ':';
            avail -= 1;
        };
        *pStr++ = hex[*src >> 4];
        *pStr++ = hex[*src++ & 0x0F];
        avail -= 2;
        src_len--;
    }

    if (src_len && avail)
        *pStr++ = ':'; // Indicate not all data fit on line.

    return (char *) dst;
}

#if defined(UARTLOG_ENABLE)
/*
 * @brief   Extract the LOCALNAME from Scan/AdvData
 *
 * @param   data - Pointer to the advertisement or scan response data
 *
 * @return  Pointer to null-terminated string with the adv local name.
 */
static char *Util_getLocalNameStr( const uint8_t *data )
{
    uint8_t nuggetLen = 0;
    uint8_t nuggetType = 0;
    uint8_t advIdx = 0;

    static char localNameStr[32] = { 0 };
    memset(localNameStr, 0, sizeof(localNameStr));

    for (advIdx = 0; advIdx < 32;)
    {
        nuggetLen = data[advIdx++];
        nuggetType = data[advIdx];
        if ((nuggetType == GAP_ADTYPE_LOCAL_NAME_COMPLETE
                || nuggetType == GAP_ADTYPE_LOCAL_NAME_SHORT) && nuggetLen < 31)
        {
            memcpy(localNameStr, &data[advIdx + 1], nuggetLen - 1);
            break;
        }
        else
        {
            advIdx += nuggetLen;
        }
    }

    return localNameStr;
}
#endif

/*********************************************************************
 *********************************************************************/
