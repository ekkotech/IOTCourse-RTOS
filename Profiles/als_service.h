/******************************************************************************
 * Filename:       als_service.h
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

#ifndef PROFILES_ALS_SERVICE_H_
#define PROFILES_ALS_SERVICE_H_


/*********************************************************************
 * INCLUDES
 */
#include "labs.h"
#include "common.h"

#include <bcomdef.h>

/*********************************************************************
 * CONSTANTS
 */

//
// Default values for characteristics
//
#define ALS_LMTHRESH_DEFAULT        50          // Lux
#define ALS_LMHYST_DEFAULT          10          // Percent
#define ALS_LMOFFON_DEFAULT         1           // On

//
// Ambient Light Service UUIDs
// BASE UUID:
#define ALS_BASE128_UUID(uuid)  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                        LO_UINT16(uuid), HI_UINT16(uuid), 0x00, 0x00

// Service UUID
#define ALS_SERVICE_SERV_UUID       0x0200

// ALS Luminance Characteristic defines
#define ALS_LUMIN_ID                0
#define ALS_LUMIN_UUID              0x0201
#define ALS_LUMIN_LEN               2      // 16-bits representing ADC reading
#define ALS_LUMIN_LEN_MIN           2

// Light Monitor Threshold
#define ALS_LMTHRESH_ID             1
#define ALS_LMTHRESH_UUID           0x0202
#define ALS_LMTHRESH_LEN            2
#define ALS_LMTHRESH_LEN_MIN        2

// Light Monitor Hysteresis
#define ALS_LMHYST_ID               2
#define ALS_LMHYST_UUID             0x0203
#define ALS_LMHYST_LEN              1
#define ALS_LMHYST_LEN_MIN          1

// Light Monitor Off/On
#define ALS_LMOFFON_ID              3
#define ALS_LMOFFON_UUID            0x0204
#define ALS_LMOFFON_LEN             1
#define ALS_LMOFFON_LEN_MIN         1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*AlsServiceChange_t)( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID,
                                                        uint8_t *pValue, uint16_t len );

typedef struct
{
  AlsServiceChange_t        pfnChangeCb;     // Called when characteristic value changes
  AlsServiceChange_t        pfnCfgChangeCb;  // Called when characteristic CCCD changes
} AlsServiceCBs_t;


/*********************************************************************
 * API FUNCTIONS
 */

/*
 * AlsService_AddService- Initializes the AlsService service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t AlsService_AddService( uint8_t rspTaskId );

/*
 * AlsService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t AlsService_RegisterAppCBs( AlsServiceCBs_t *appCallbacks );

/*
 * AlsService_SetParameter - Set a AlsService parameter.
 *
 *    param - Profile parameter ID
 *    len   - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *            the parameter ID and may be cast to the appropriate
 *            data type (example: data type of uint16_t will be cast to
 *            uint16_t pointer).
 */
extern bStatus_t AlsService_SetParameter( uint8_t param, uint16_t len, void *value );

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
extern bStatus_t AlsService_GetParameter( uint8_t param, uint16_t *len, void *value );


/*********************************************************************
*********************************************************************/


#endif /* PROFILES_ALS_SERVICE_H_ */
