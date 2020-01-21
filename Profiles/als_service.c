/******************************************************************************
 * Filename:       als_service.c
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

#include <string.h>

//#define xdc_runtime_Log_DISABLE_ALL 1  // Add to disable logs from this file
#include <xdc/runtime/Diags.h>
#include <uartlog/UartLog.h>

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and ICall structure definition */
#include "icall_ble_api.h"

#include "labs.h"
#include "common.h"
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

// AlsService Service UUID
CONST uint8_t AlsServiceUUID[ATT_UUID_SIZE] =
{
  ALS_BASE128_UUID(ALS_SERVICE_SERV_UUID)
};

// LUMINANCE
CONST uint8_t als_LUMINUUID[ATT_UUID_SIZE] =
{
  ALS_BASE128_UUID(ALS_LUMIN_UUID)
};

// THRESH
CONST uint8_t als_THRESHUUID[ATT_UUID_SIZE] =
{
    ALS_BASE128_UUID(ALS_THRESH_UUID)
};

// HYST
CONST uint8_t als_HYSTUUID[ATT_UUID_SIZE] =
{
    ALS_BASE128_UUID(ALS_HYST_UUID)
};

// LMOFFON
CONST uint8_t als_LMOFFONUUID[ATT_UUID_SIZE] =
{
     ALS_BASE128_UUID(ALS_LMOFFON_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static AlsServiceCBs_t *pAppCBs = NULL;
static uint8_t als_icall_rsp_task_id = INVALID_TASK_ID;

/*********************************************************************
* Profile Attributes - variables
*/

// Service declaration
static CONST gattAttrType_t AlsServiceDecl = { ATT_UUID_SIZE, AlsServiceUUID };

// Characteristic "LUMINANCE" Properties (for declaration)
static uint8_t als_LUMINProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic "LUMINANCE" Value variable
static uint8_t als_LUMINVal[ALS_LUMIN_LEN] = {0, 0};

// Characteristic "LUMINANCE" Client Characteristic Configuration Descriptor
static gattCharCfg_t *als_LUMINConfig;

// Length of data in characteristic "LUMINANCE" Value Length variable, initialised to minimum size.
static uint16_t als_LUMINValLen = ALS_LUMIN_LEN_MIN;

// Characteristic "LUMINANCE" User Description
static uint8_t als_LUMINUserDesc[10] = "Luminance\0";

// Characteristic "THRESH" Properties (for declaration)
static uint8_t als_THRESHProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;

// Characteristic "THRESH" Value variable
static uint8_t als_THRESHVal[ALS_THRESH_LEN] = { ALS_DEFAULT_THRESH };

// Length of data in characteristic "THRESH" Value variable, initialised to minimum size
static uint16_t als_THRESHValLen = ALS_THRESH_LEN_MIN;

// Characteristic "THRESH" User Description
static uint8_t als_THRESHUserDesc[24] = "Light Monitor Threshold\0";


// Characteristic "HYST" Properties (for declaration)
static uint8_t als_HYSTProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;

// Characteristic "HYST" Value variable
static uint8_t als_HYSTVal[ALS_HYST_LEN] = { ALS_DEFAULT_HYST };

// Length of data in characteristic "HYST" Value variable, initialised to minimum size
static uint16_t als_HYSTValLen = ALS_HYST_LEN_MIN;

// Characteristic "HYST" User Description
static uint8_t als_HYSTUserDesc[25] = "Light Monitor Hysteresis\0";


// Characteristic "LMOFFON" Properties (for declaration)
static uint8_t als_LMOFFONProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;

// Characteristic "LMOFFON" Value variable
static uint8_t als_LMOFFONVal[ALS_LMOFFON_LEN] = { ALS_DEFAULT_LMOFFON };

// Length of data in characteristic "LMOFFON" Value variable, initialised to minimum size
static uint16_t als_LMOFFONValLen = ALS_LMOFFON_LEN_MIN;

// Characteristic "LMOFFON" User Description
static uint8_t als_LMOFFONUserDesc[21] = "Light Monitor Off/On\0";

/*********************************************************************
* Profile Attributes - Table
*/

static gattAttribute_t AlsServiceAttrTbl[] = {

    // AlsService Service Declaration
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID },
        GATT_PERMIT_READ,
        0,
        (uint8_t *)&AlsServiceDecl
        },
    //
    // LUMINANCE Characteristic Declaration
    //
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &als_LUMINProps
    },
    // LUMINANCE Characteristic Value
    {
        { ATT_UUID_SIZE, als_LUMINUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        als_LUMINVal
    },
    // LUMINANCE Client Characteristic Configuration Descriptor (CCCD)
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8_t *)&als_LUMINConfig
    },
    // LUMINANCE Characteristic User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        als_LUMINUserDesc
    },

    // THRESH Characteristic declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &als_THRESHProps
    },
    // THRESH Characteristic Value
    {
        { ATT_UUID_SIZE, als_THRESHUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        als_THRESHVal
    },
    // THRESH Characteristic User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        als_THRESHUserDesc
    },

    //
    // HYST Characteristic declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &als_HYSTProps
    },
    // HYST Characteristic Value
    {
        { ATT_UUID_SIZE, als_HYSTUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        als_HYSTVal
    },
    // HYST Characteristic User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        als_HYSTUserDesc
    },

    //
    // LMOFFON Characteristic declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &als_LMOFFONProps
    },
    // LMOFFON Characteristic Value
    {
        { ATT_UUID_SIZE, als_LMOFFONUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        als_LMOFFONVal
    },
    // LMOFFON Characteristic User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        als_LMOFFONUserDesc
    }

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static bStatus_t AlsService_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                           uint16_t maxLen, uint8_t method );
static bStatus_t AlsService_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                            uint8_t *pValue, uint16_t len, uint16_t offset,
                                            uint8_t method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Als Service Callbacks
CONST gattServiceCBs_t AlsServiceCBs =
{
    AlsService_ReadAttrCB,  // Read callback function pointer
    AlsService_WriteAttrCB, // Write callback function pointer
    NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * EXTERN FUNCTIONS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*
 * AlsService_AddService- Initializes the AlsService service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t AlsService_AddService( uint8_t rspTaskId )
{

    uint8_t status;

    // Allocate Client Characteristic Configuration table
    als_LUMINConfig = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * linkDBNumConns );
    if ( als_LUMINConfig == NULL )
    {
        return ( bleMemAllocError );
    }

    // Initialize Client Characteristic Configuration attributes
    GATTServApp_InitCharCfg( INVALID_CONNHANDLE, als_LUMINConfig );

    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( AlsServiceAttrTbl,
                                          GATT_NUM_ATTRS( AlsServiceAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &AlsServiceCBs );
    Log_info1("Registered service, %d attributes", (IArg)GATT_NUM_ATTRS( AlsServiceAttrTbl ));
    return ( status );
}

/*
 * AlsService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
bStatus_t AlsService_RegisterAppCBs( AlsServiceCBs_t *appCallbacks )
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
 * AlsService_SetParameter - Set an AlsService parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t AlsService_SetParameter( uint8_t param, uint16_t len, void *value )
{

    bStatus_t ret = SUCCESS;
    uint8_t  *pAttrVal;
    uint8_t sendNotiInd = false;
    uint8_t needAuth;
    uint16_t *pValLen;
    uint16_t valMinLen;
    uint16_t valMaxLen;
    gattCharCfg_t *attrConfig;

    switch ( param )
    {

    case ALS_LUMIN_ID:
        pAttrVal  =  als_LUMINVal;
        pValLen   = &als_LUMINValLen;
        valMinLen =  ALS_LUMIN_LEN_MIN;
        valMaxLen =  ALS_LUMIN_LEN;
        sendNotiInd = true;
        attrConfig = als_LUMINConfig;
        needAuth    = FALSE; // Change if authenticated link is required for sending.
        Log_info2("SetParameter : %s len: %d", (IArg)"LUMINANCE", (IArg)len);
        break;

    case ALS_THRESH_ID:
        pAttrVal = als_THRESHVal;
        pValLen = &als_THRESHValLen;
        valMinLen = ALS_THRESH_LEN_MIN;
        valMaxLen = ALS_THRESH_LEN;
        Log_info2("SetParameter : %s len: %d", (IArg)"THRESH", (IArg)len);
        break;

    case ALS_HYST_ID:
        pAttrVal = als_HYSTVal;
        pValLen = &als_HYSTValLen;
        valMinLen = ALS_HYST_LEN_MIN;
        valMaxLen = ALS_HYST_LEN;
        Log_info2("SetParameter : %s len: %d", (IArg)"HYST", (IArg)len);
        break;

    case ALS_LMOFFON_ID:
        pAttrVal = als_LMOFFONVal;
        pValLen = &als_LMOFFONValLen;
        valMinLen = ALS_LMOFFON_LEN_MIN;
        valMaxLen = ALS_LMOFFON_LEN;
        Log_info2("SetParameter : %s len: %d", (IArg)"LMOFFON", (IArg)len);
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

        if (sendNotiInd)
        {
            Log_info2("Trying to send noti/ind: connHandle %x, %s",
                      (IArg)attrConfig[0].connHandle,
                      (IArg)((attrConfig[0].value==0)?"\x1b[33mNoti/ind disabled\x1b[0m" :
                              (attrConfig[0].value==1)?"Notification enabled" :
                                      "Indication enabled"));
            // Try to send notification.
            GATTServApp_ProcessCharCfg( attrConfig, pAttrVal, needAuth,
                                        AlsServiceAttrTbl, GATT_NUM_ATTRS( AlsServiceAttrTbl ),
                                        als_icall_rsp_task_id,  AlsService_ReadAttrCB);
        }
    }
    else
    {
        Log_error3("Length outside bounds: Len: %d MinLen: %d MaxLen: %d.", (IArg)len, (IArg)valMinLen, (IArg)valMaxLen);
        ret = bleInvalidRange;
    }

    return ret;

}


/*
 * AlsService_GetParameter - Get a AlsService parameter.
 *
 *    param - Profile parameter ID
 *    len   - pointer to a variable that contains the maximum length that can be written to *value.
              After the call, this value will contain the actual returned length.
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
bStatus_t AlsService_GetParameter( uint8_t param, uint16_t *len, void *value )
{

    bStatus_t ret = SUCCESS;

    switch ( param )
    {

    case ALS_LUMIN_ID:
        *len = MIN(*len, als_LUMINValLen);
        memcpy(value, als_LUMINVal, *len);
        Log_info2("GetParameter : %s returning %d bytes", (IArg)"LUMINANCE", (IArg)*len);
        break;

    case ALS_THRESH_ID:
        *len = MIN(*len, als_THRESHValLen);
        memcpy(value, als_THRESHVal, *len);
        Log_info2("GetParameter : %s returning %d bytes", (IArg)"THRESH", (IArg)*len);
        break;

    case ALS_HYST_ID:
        *len = MIN(*len, als_HYSTValLen);
        memcpy(value, als_HYSTVal, *len);
        Log_info2("GetParameter : %s returning %d bytes", (IArg)"HYST", (IArg)*len);
        break;

    case ALS_LMOFFON_ID:
        *len = MIN(*len, als_LMOFFONValLen);
        memcpy(value, als_LMOFFONVal, *len);
        Log_info2("GetParameter : %s returning %d bytes", (IArg)"LMOFFON", (IArg)*len);
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
 * @fn          AlsService_findCharParamId
 *
 * @brief       Find the logical param id of an attribute in the service's attr table.
 *
 *              Works only for Characteristic Value attributes and
 *              Client Characteristic Configuration Descriptor attributes.
 *
 * @param       pAttr - pointer to attribute
 *
 * @return      uint8_t paramID (ref als_service.h) or 0xFF if not found.
 */
static uint8_t AlsService_findCharParamId(gattAttribute_t *pAttr)
{

    // Is this a Client Characteristic Configuration Descriptor?
    if (ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID == *(uint16_t *)pAttr->type.uuid)
        return AlsService_findCharParamId(pAttr - 1); // Assume the value attribute precedes CCCD and recurse

    // Is this attribute in "LUMINANCE"?
    else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, als_LUMINUUID, pAttr->type.len))
        return ALS_LUMIN_ID;

    // Is this attribute in "THRESH"?
    else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, als_THRESHUUID, pAttr->type.len))
        return ALS_THRESH_ID;

    // Is this attribute in "HYST"?
    else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, als_HYSTUUID, pAttr->type.len))
        return ALS_HYST_ID;

    // Is this attribute in "LMOFFON"?
    else if ( ATT_UUID_SIZE == pAttr->type.len && !memcmp(pAttr->type.uuid, als_LMOFFONUUID, pAttr->type.len))
        return ALS_LMOFFON_ID;

    else
        return 0xFF; // Not found. Return invalid.
}

/*********************************************************************
 * @fn          AlsService_ReadAttrCB
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
static bStatus_t AlsService_ReadAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                       uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                       uint16_t maxLen, uint8_t method )
{

    bStatus_t status = SUCCESS;
    uint16_t valueLen;
    uint8_t paramID = 0xFF;

    // Find settings for the characteristic to be read.
    paramID = AlsService_findCharParamId( pAttr );
    switch ( paramID )
    {

    case ALS_LUMIN_ID:
        valueLen = als_LUMINValLen;
        /* Other considerations for LUMINANCE can be inserted here */
        break;

    case ALS_THRESH_ID:
        valueLen = als_THRESHValLen;
        break;

    case ALS_HYST_ID:
        valueLen = als_HYSTValLen;
        break;

    case ALS_LMOFFON_ID:
        valueLen = als_LMOFFONValLen;
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
 * @fn      AlsService_WriteAttrCB
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
static bStatus_t AlsService_WriteAttrCB( uint16_t connHandle, gattAttribute_t *pAttr,
                                        uint8_t *pValue, uint16_t len, uint16_t offset,
                                        uint8_t method )
{

    bStatus_t status  = SUCCESS;
    uint8_t   paramID = 0xFF;
    uint8_t   changeParamID = 0xFF;
    uint16_t  writeLenMin;
    uint16_t  writeLenMax;
    uint16_t  *pValueLenVar;

    // See if request is regarding a Client Characterisic Configuration
    if (ATT_BT_UUID_SIZE == pAttr->type.len && GATT_CLIENT_CHAR_CFG_UUID == *(uint16_t *)pAttr->type.uuid)
    {
        Log_info3("WriteAttrCB (CCCD): param: %d connHandle: %d %s",
                  (IArg)AlsService_findCharParamId(pAttr),
                  (IArg)connHandle,
                  (IArg)(method == GATT_LOCAL_WRITE?"- restoring bonded state":"- OTA write"));

        // Allow notification and indication, but do not check if really allowed per CCCD.
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY |
                                                 GATT_CLIENT_CFG_INDICATE );

        if (SUCCESS == status && pAppCBs && pAppCBs->pfnCfgChangeCb)
            pAppCBs->pfnCfgChangeCb( connHandle, ALS_SERVICE_SERV_UUID,
                                     AlsService_findCharParamId(pAttr), pValue, len );

        return status;

    }

    // Find settings for the characteristic to be written.
    paramID = AlsService_findCharParamId( pAttr );
    switch ( paramID )
    {

    case ALS_THRESH_ID:
        writeLenMin  = ALS_THRESH_LEN_MIN;
        writeLenMax  = ALS_THRESH_LEN;
        pValueLenVar = &als_THRESHValLen;
        break;

    case ALS_HYST_ID:
        writeLenMin  = ALS_HYST_LEN_MIN;
        writeLenMax  = ALS_HYST_LEN;
        pValueLenVar = &als_HYSTValLen;
        break;

    case ALS_LMOFFON_ID:
        writeLenMin  = ALS_LMOFFON_LEN_MIN;
        writeLenMax  = ALS_LMOFFON_LEN;
        pValueLenVar = &als_LMOFFONValLen;
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
            pAppCBs->pfnChangeCb( connHandle, ALS_SERVICE_SERV_UUID, paramID, pValue, len+offset ); // Call app function from stack task context.

    return status;

}

/*********************************************************************
*********************************************************************/



