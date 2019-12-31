/******************************************************************************
 * Filename:       lss_service.c
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

#include <string.h>

//#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file
#include <xdc/runtime/Diags.h>
#include <uartlog/UartLog.h>

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and ICall structure definition */
#include "icall_ble_api.h"

#include "lss_service.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * EXTERNALS
 */
extern uint8 g_PairingState;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// LssService Service UUID
CONST uint8_t LssServiceUUID[ATT_UUID_SIZE] =
{
    LSS_BASE128_UUID(LSS_SERVICE_SERV_UUID)
};

// Off/On
CONST uint8_t lss_OFFONUUID[ATT_UUID_SIZE] =
{
    LSS_BASE128_UUID(LSS_OFFON_UUID)
};

// RGB
CONST uint8_t lss_RGBUUID[ATT_UUID_SIZE] =
{
    LSS_BASE128_UUID(LSS_RGB_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static LssServiceCBs_t *pAppCBs = NULL;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t LssServiceDecl = { ATT_UUID_SIZE, LssServiceUUID };


// Characteristic "OFFON" Properties (for declaration)
static uint8_t lss_OFFONProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;

// Characteristic "OFFON" Value variable
static uint8_t lss_OFFONVal[LSS_OFFON_LEN] = { LSS_DEFAULT_OFFON };

// Length of data in characteristic "OFFON" Value variable, initialised to minimum size.
static uint16_t lss_OFFONValLen = LSS_OFFON_LEN_MIN;

// Characteristic "OFFON" User Description
static uint8_t lss_OFFONUserDesc[7] = "Off/On\0";


// Characteristic "RGB" Properties (for declaration)
static uint8_t lss_RGBProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;

// Characteristic "RGB" Value variable
static uint8_t lss_RGBVal[LSS_RGB_LEN] = {LSS_DEFAULT_RED, LSS_DEFAULT_GREEN, LSS_DEFAULT_BLUE};

// Length of data in characteristic "RGB" Value variable, initialized to minimum size.
static uint16_t lss_RGBValLen = LSS_RGB_LEN_MIN;

// Characteristic "RGB" User Description
static uint8_t lss_RGBUserDesc[11] = "RGB Colour\0";

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t LssServiceAttrTbl[] =
{

    // LssService Service Declaration
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID },
        GATT_PERMIT_READ,
        0,
        (uint8_t *)&LssServiceDecl
    },

    // OFFON Characteristic Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &lss_OFFONProps
    },
    // OFFON Characteristic Value
    {
        { ATT_UUID_SIZE, lss_OFFONUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        lss_OFFONVal
    },
    // OFFON Characteristic User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        lss_OFFONUserDesc
    },

    //
    // RGB Characteristic Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &lss_RGBProps
    },
    // RGB Characteristic Value
    {
        { ATT_UUID_SIZE, lss_RGBUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        lss_RGBVal
    },
    // RGB Characteristic User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        lss_RGBUserDesc
    }

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static bStatus_t LssService_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                              uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                              uint16_t maxLen, uint8_t method );
static bStatus_t LssService_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                               uint8_t *pValue, uint16_t len, uint16_t offset,
                                               uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t Lss_ServiceCBs =
{
    LssService_ReadAttrCB,  // Read callback function pointer
    LssService_WriteAttrCB, // Write callback function pointer
    NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * EXTERN FUNCTIONS
 */

/*
 * @brief   sample function
 *
 * @param   param desc
 *
 * @return @ref ret description
 *
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * LedService_AddService- Initializes the LedService service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t LssService_AddService( uint8_t rspTaskId )
{
    uint8_t status;

    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( LssServiceAttrTbl,
                                          GATT_NUM_ATTRS( LssServiceAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &Lss_ServiceCBs );
    Log_info1("Registered service, %d attributes", (IArg)GATT_NUM_ATTRS( LssServiceAttrTbl ));
    return ( status );
}

/*
 * LedService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t LssService_RegisterAppCBs( LssServiceCBs_t *appCallbacks )
{
    if ( appCallbacks )
    {
        pAppCBs = appCallbacks;
        Log_info1("Registered callbacks to application. Struct %p", (IArg)appCallbacks);
        return ( SUCCESS );
    }
    else
    {
        Log_warning0("Null pointer given for app callbacks.");
        return ( FAILURE );
    }
}

/*
 * LssService_SetParameter - Set a LssService parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t LssService_SetParameter( uint8_t param, uint16_t len, void *value )
{
    bStatus_t ret = SUCCESS;
    uint8_t  *pAttrVal;
    uint16_t *pValLen;
    uint16_t valMinLen;
    uint16_t valMaxLen;

    switch ( param )
    {

    case LSS_OFFON_ID:
        pAttrVal  =  lss_OFFONVal;
        pValLen   = &lss_OFFONValLen;
        valMinLen =  LSS_OFFON_LEN_MIN;
        valMaxLen =  LSS_OFFON_LEN;
        Log_info2("SetParameter : %s len: %d", (IArg)"OFFON", (IArg)len);
        break;

    case LSS_RGB_ID:
        pAttrVal  =  lss_RGBVal;
        pValLen   = &lss_RGBValLen;
        valMinLen =  LSS_RGB_LEN_MIN;
        valMaxLen =  LSS_RGB_LEN;
        Log_info2("SetParameter : %s len: %d", (IArg)"RGB", (IArg)len);
        break;

    default:
        Log_error1("SetParameter: Parameter #%d not valid.", (IArg)param);
        return INVALIDPARAMETER;
    }

  // Check bounds, update value and send notification or indication if possible.
    if ( len <= valMaxLen && len >= valMinLen )
    {
        memcpy(pAttrVal, value, len);
        *pValLen = len; // Update length for read and get.
    }
    else
    {
        Log_error3("Length outside bounds: Len: %d MinLen: %d MaxLen: %d.", (IArg)len, (IArg)valMinLen, (IArg)valMaxLen);
        ret = bleInvalidRange;
    }

    return ret;
}

/*
 * LssService_GetParameter - Get a LssService parameter.
 *
 *    param - Profile parameter ID
 *    len   - pointer to a variable that contains the maximum length that can be written to *value.
              After the call, this value will contain the actual returned length.
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t LssService_GetParameter( uint8_t param, uint16_t *len, void *value )
{
    bStatus_t ret = SUCCESS;

    switch ( param )
    {

    case LSS_OFFON_ID:
        *len = MIN(*len, lss_OFFONValLen);
        memcpy(value, lss_OFFONVal, *len);
        Log_info2("GetParameter : %s returning %d bytes", (IArg)"OFFON", (IArg)*len);
        break;

    case LSS_RGB_ID:
        *len = MIN(*len, lss_RGBValLen);
        memcpy(value, lss_RGBVal, *len);
        Log_info2("GetParameter : %s returning %d bytes", (IArg)"RGB", (IArg)*len);
        break;

    default:
        Log_error1("GetParameter: Parameter #%d not valid.", (IArg)param);
        ret = INVALIDPARAMETER;
        break;
    }
    return ret;

}

/*********************************************************************
 * @internal
 * @fn          LssService_findCharParamId
 *
 * @brief       Find the logical param id of an attribute in the service's attr table.
 *
 *              Works only for Characteristic Value attributes and
 *              Client Characteristic Configuration Descriptor attributes.
 *
 * @param       pAttr - pointer to attribute
 *
 * @return      uint8_t paramID (ref led_string_service.h) or 0xFF if not found.
 */
static uint8_t LssService_findCharParamId(gattAttribute_t *pAttr)
{
    // Is this a Client Characteristic Configuration Descriptor?
    if (ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID == *(uint16_t *)pAttr->type.uuid)
        return LssService_findCharParamId(pAttr - 1); // Assume the value attribute precedes CCCD and recurse

    // Is this attribute in "OFFON"?
    else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, lss_OFFONUUID, pAttr->type.len))
        return LSS_OFFON_ID;

    // Is this attribute in "RGB"?
    else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, lss_RGBUUID, pAttr->type.len))
        return LSS_RGB_ID;

    else
        return 0xFF; // Not found. Return invalid.

}

/*********************************************************************
 * @fn          LssService_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t LssService_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{

    bStatus_t status = SUCCESS;
    uint16_t valueLen;
    uint8_t paramID = 0xFF;

    // Find settings for the characteristic to be read.
    paramID = LssService_findCharParamId( pAttr );

    switch ( paramID )
    {

    case LSS_OFFON_ID:

        // We use a read of this characteristic to force pairing
        // status = ATT_ERR_INSUFFICIENT_AUTHEN;
        // LAB_6_TODO - add test for bonded here

        valueLen = lss_OFFONValLen;
        break;

    case LSS_RGB_ID:
        valueLen = lss_RGBValLen;
        break;

    default:
        Log_error0("Attribute was not found.");
        return ATT_ERR_ATTR_NOT_FOUND;
    }

    // Check bounds and return the value
    if ( offset > valueLen )  // Prevent malicious ATT ReadBlob offsets.
    {
        Log_error0("An invalid offset was requested.");
        status = ATT_ERR_INVALID_OFFSET;
    }
    else
    {
        *pLen = MIN(maxLen, valueLen - offset);  // Transmit as much as possible
        memcpy(pValue, pAttr->pValue + offset, *pLen);
    }

    return status;
}

/*********************************************************************
 * @fn      LssService_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t LssService_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method )
{

    bStatus_t status  = SUCCESS;
    uint8_t   paramID = 0xFF;
    uint8_t   changeParamID = 0xFF;
    uint16_t writeLenMin;
    uint16_t writeLenMax;
    uint16_t *pValueLenVar;

    // Find settings for the characteristic to be written.
    paramID = LssService_findCharParamId( pAttr );
    switch ( paramID )
    {

    case LSS_OFFON_ID:
        writeLenMin  = LSS_OFFON_LEN_MIN;
        writeLenMax  = LSS_OFFON_LEN;
        pValueLenVar = &lss_OFFONValLen;
        break;

    case LSS_RGB_ID:
        writeLenMin  = LSS_RGB_LEN_MIN;
        writeLenMax  = LSS_RGB_LEN;
        pValueLenVar = &lss_RGBValLen;
        break;

    default:
        Log_error0("Attribute was not found.");
        return ATT_ERR_ATTR_NOT_FOUND;
    }

    // Check whether the length is within bounds.
    if ( offset >= writeLenMax )
    {
        Log_error0("An invalid offset was requested.");
        status = ATT_ERR_INVALID_OFFSET;
    }
    else if ( offset + len > writeLenMax )
    {
        Log_error0("Invalid value length was received.");
        status = ATT_ERR_INVALID_VALUE_SIZE;
    }
    else if ( offset + len < writeLenMin && ( method == ATT_EXECUTE_WRITE_REQ || method == ATT_WRITE_REQ ) )
    {
        // Refuse writes that are lower than minimum.
        // Note: Cannot determine if a Reliable Write (to several chars) is finished, so those will
        //       only be refused if this attribute is the last in the queue (method is execute).
        //       Otherwise, reliable writes are accepted and parsed piecemeal.
        Log_error0("Invalid value length was received.");
        status = ATT_ERR_INVALID_VALUE_SIZE;
    }
    else
    {
        // Copy pValue into the variable we point to from the attribute table.
        memcpy(pAttr->pValue + offset, pValue, len);

        // Only notify application and update length if enough data is written.
        //
        // Note: If reliable writes are used (meaning several attributes are written to using ATT PrepareWrite),
        //       the application will get a callback for every write with an offset + len larger than _LEN_MIN.
        // Note: For Long Writes (ATT Prepare + Execute towards only one attribute) only one callback will be issued,
        //       because the write fragments are concatenated before being sent here.
        if ( offset + len >= writeLenMin )
        {
            changeParamID = paramID;
            *pValueLenVar = offset + len; // Update data length.
        }
    }

    // Let the application know something changed (if it did) by using the
    // callback it registered earlier (if it did).
    if (changeParamID != 0xFF)
        if ( pAppCBs && pAppCBs->pfnChangeCb )
            pAppCBs->pfnChangeCb( connHandle, LSS_SERVICE_SERV_UUID, paramID, pValue, len+offset ); // Call app function from stack task context.

    return status;

}
