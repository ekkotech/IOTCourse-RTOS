/******************************************************************************
 * Filename:       als_handler.c
 *
 * Description:    This file contains the configuration
 *              definitions and prototypes for the iOS Workshop
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

/*********************************************************************
 * INCLUDES
 */
#include "labs.h"
#include "common.h"

#include <uartlog/UartLog.h>
#include <driverlib/aux_adc.h>
#include <driverlib/aux_wuc.h>

#include "icall_ble_api.h"
#include <icall.h>

#include "project_zero.h"
#include "als_handler.h"
#include "als_service.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Entity ID globally used to check for source and/or destination of messages
extern ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
extern ICall_SyncHandle syncEvent;

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
#ifdef LAB_2        // LAB_2 - Service Configuration
void user_AlsService_ValueChangeHandler(char_data_t *pCharData);
void user_AlsService_CfgChangeHandler(char_data_t *pCharData);
#endif /* LAB_2 */

#ifdef LAB_5        // LAB_5 - Analogue Input
void als_Hardware_Init();
void als_ProcessPeriodicEvent();
#endif /* LAB_5 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
#ifdef LAB_2        // LAB_2 - Service Configuration
static void processThresholdValueChange(char_data_t *pCharData);
static void processHysteresisValueChange(char_data_t *pCharData);
static void processLMOffOnValueChange(char_data_t *pCharData);
static void processLuminCfgChange(char_data_t *pCharDate);
#endif /* LAB_2 */

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
static void updateSnvState( uint8_t charId, uint16_t len, uint8_t *pData );
#endif /* LAB_4 */

#ifdef LAB_5        // LAB_5 - Analogue Input
static void initADC(void);
static void updateLuminance();
static void notifyIfThreshold(uint16_t lux);
#endif /* LAB_5 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

#ifdef LAB_5        // LAB_5 - Analogue Input
//#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
/*
 * @fn      als_ProcessPeriodicEvent
 *
 * @brief   Perform any required processing in a periodic basis
 *
 * @param   none
 *
 * @return  none
 *
 */
void als_ProcessPeriodicEvent()
{

    Log_info0("In als_ProcessPeriodicEvent");

 #ifdef LAB_5        // LAB_5 - Analogue Input
    updateLuminance();
#endif /* LAB_5 */

}
//#endif /* LAB_4 */

/*
 * @fn      als_Hardware_Init
 *
 * @brief   Initialise any service-specific hardware
 *
 * @param   none
 *
 * @return  none
 *
 */
void als_Hardware_Init()
{
    initADC();
}

/*
 * @fn      als_Resource_Init
 *
 * @brief   Initialise any service-specific resources
 *
 * @param   none
 *
 * @return  none
 *
 */
void als_Resource_Init()
{
    // Insert resource initialisation code here
}
#endif /* LAB_5 */

#ifdef LAB_2        // LAB_2 - Service Configuration
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

void user_AlsService_ValueChangeHandler(char_data_t *pCharData)
{
  static uint8_t pretty_data_holder[16]; // 5 bytes as hex string "AA:BB:CC:DD:EE"
  Util_convertArrayToHexString(pCharData->data, pCharData->dataLen,
                               pretty_data_holder, sizeof(pretty_data_holder));

  switch (pCharData->paramID)
  {
    case ALS_THRESH_ID:
      processThresholdValueChange(pCharData);
      break;

    case ALS_HYST_ID:
      processHysteresisValueChange(pCharData);
      break;

    case ALS_LMOFFON_ID:
      processLMOffOnValueChange(pCharData);
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
void user_AlsService_CfgChangeHandler(char_data_t *pCharData)
{
#if defined(UARTLOG_ENABLE)
  // Cast received data to uint16, as that's the format for CCCD writes.
  uint16_t configValue = *(uint16_t *)pCharData->data;
  char *configValString;

  // Determine what to tell the user
  switch(configValue)
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

  Log_info0(configValString);

#endif
  switch (pCharData->paramID)
  {
    case ALS_LUMIN_ID:
      processLuminCfgChange(pCharData);
      break;
  }
}

/*
 * @fn      processThresholdValueChange
 *
 * @brief   Process a THRESH characteristic value change
 *
 * @param   pCharData  Pointer to the characteristic data
 *
 * @return  None.
 */
static void processThresholdValueChange(char_data_t *pCharData)
{
    Log_info0("In processThresholdValueChange");

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
    if (pCharData->dataLen == sizeof(thresh_char_t))
    {
        updateSnvState(pCharData->paramID, pCharData->dataLen, pCharData->data);
    }
#endif /* LAB_4 */

}

/*
 * @fn      processHysteresisValueChange
 *
 * @brief   Process a HYST characteristic value change
 *
 * @param   pCharData  Pointer to the characteristic data
 *
 * @return  None.
 */
static void processHysteresisValueChange(char_data_t *pCharData)
{
    Log_info0("In processHysteresisValueChange");

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
    if (pCharData->dataLen == sizeof(hyst_char_t))
    {
        updateSnvState(pCharData->paramID, pCharData->dataLen, pCharData->data);
    }
#endif /* LAB_4 */

}

/*
 * @fn      processLMOffOnValueChange
 *
 * @brief   Process a LMOFFON characteristic value change
 *
 * @param   pCharData  Pointer to the characteristic data
 *
 * @return  None.
 */
static void processLMOffOnValueChange(char_data_t *pCharData)
{
    Log_info0("In processLMOffOnValueChange");

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
    if (pCharData->dataLen == sizeof(hyst_char_t))
    {
        updateSnvState(pCharData->paramID, pCharData->dataLen, pCharData->data);
    }
#endif /* LAB_4 */

}

/*
 * @fn      processLuminCfgChange
 *
 * @brief   Process a LUMIN characteristic CCCD change
 *
 * @param   pCharData  Pointer to the characteristic data
 *
 * @return  None.
 */
static void processLuminCfgChange(char_data_t *pCharData)
{
    Log_info0("In processLuminCfgChange");
    
    // LAB_6_TODO - Add code to read Lumin charac and write it right back

}
#endif /* LAB_2 */


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

/*
 *  Callbacks from Swi-context
 *****************************************************************************/

/*
 *  Callbacks from Hwi-context
 *****************************************************************************/


/*
 *  Hardware initialisation, support functions
 *****************************************************************************/

#ifdef LAB_5        // LAB_5 - Analogue Input
/*
 * @fn      initADC
 *
 * @brief   Initialises the CC2640R2F ADC
 *
 * @param   none
 *
 * @return  none
 */
static void initADC(void) {

    //
    // The following API calls are Driverlib-level calls
    // Refer to API sections AUX - Auxiliary Domain (WUC and ADC)
    //
    // Enable to clock to ADI, ANAIF and ADC

    // Disable the ADC before making any changes (clocks have to be enabled beforehand)

    // Set up AUXIO7 as the input - AUXIO7 has a fixed mapping to DIO23



}
#endif /* LAB_5 */

/******************************************************************************
 *****************************************************************************
 *
 *  Utility functions
 *
 ****************************************************************************
 *****************************************************************************/

#ifdef LAB_4        // LAB_4 - Non-volatile memory
/*
 * @fn      updateSnvState
 *
 * @brief   Updates an element in snvState and sets dirty flag true
 *
 * @param   charId - the characteristic ID
 * @param   len - the number of bytes to write
 * @param   pData - pointer to the source data
 *
 * @return  none
 */
static void updateSnvState(uint8_t charId, uint16_t len, uint8_t *pData)
{
    // Insert handler code here
    switch (charId) {
        case ALS_THRESH_ID:
            break;
        case ALS_HYST_ID:
            break;
        case ALS_LMOFFON_ID:
            break;
    }
}
#endif /* LAB_4 */

#ifdef LAB_5        // LAB_5 - Analogue Input
/*********************************************************************
 * @fn      updateLuminance
 *
 * @brief   Updates the ALS luminance characteristic
 *
 * @param   none
 *
 * @return  none
 */
static void updateLuminance()
{

  // LAB_5_TODO
  
    // Insert read ADC & update code here

    //
    // Read ADC
    //

    // Update characteristic if value has changed


}

/*********************************************************************
 * @fn      notifyIfThreshold
 *
 * @brief   Notifies LED String Service if light level has crossed threshold
 *
 * @param   none
 *
 * @return  none
 */
static void notifyIfThreshold(uint16_t lux)
{
    
    // Insert notify code here
}

#endif /* LAB_5 */

/*********************************************************************
*********************************************************************/
