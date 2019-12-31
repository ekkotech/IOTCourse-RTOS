/******************************************************************************
 * Filename:       lss_service.h
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

#ifndef PROFILES_LSS_SERVICE_H_
#define PROFILES_LSS_SERVICE_H_


/*********************************************************************
 * INCLUDES
 */
#include <bcomdef.h>

#include "labs.h"
#include "common.h"

/*********************************************************************
 * TYPEDEFS
 */
//
// Memory layout of characteristic data
//
// LED String Service
// PROGRAM characteristic
typedef uint8_t program_char_t;

// OFFON characteristic
typedef uint8_t offon_char_t;

// RGB characteristic
typedef struct rgb_char {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgb_char_t;

/*********************************************************************
 * CONSTANTS
 */

//
// LAB_6 - Pairing and Bonding
// Add define for "pairing not started"

//
// Default values for characteristics
//
#define LSS_DEFAULT_OFFON           0x01        // On
#define LSS_DEFAULT_RED             0x05        // Low-level red
#define LSS_DEFAULT_GREEN           0x00        // Green off
#define LSS_DEFAULT_BLUE            0x00        // Blue off

//
// LED String Service UUIDs
// BASE UUID:
#define LSS_BASE128_UUID(uuid)  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                        LO_UINT16(uuid), HI_UINT16(uuid), 0x00, 0x00

// Service UUID
#define LSS_SERVICE_SERV_UUID       0x0100

// LED String Switch Characteristic defines
#define LSS_OFFON_ID                0
#define LSS_OFFON_UUID              0x0101
#define LSS_OFFON_LEN               sizeof(offon_char_t)
#define LSS_OFFON_LEN_MIN           sizeof(offon_char_t)

// LED String RGB Characteristic defines
#define LSS_RGB_ID                  1
#define LSS_RGB_UUID                0x0102
#define LSS_RGB_LEN                 sizeof(rgb_char_t)
#define LSS_RGB_LEN_MIN             sizeof(rgb_char_t)

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*LssServiceChange_t)( uint16_t connHandle, uint16_t svcUuid, uint8_t paramID, uint8_t *pValue, uint16_t len );

typedef struct
{
  LssServiceChange_t        pfnChangeCb;     // Called when characteristic value changes
  LssServiceChange_t        pfnCfgChangeCb;  // Called when characteristic CCCD changes
} LssServiceCBs_t;


/*********************************************************************
 * API FUNCTIONS
 */

/*
 * LssService_AddService- Initialises the LssService service by registering
 *          GATT attributes with the GATT server.
 *
 *    rspTaskId - The ICall Task Id that should receive responses for Indications.
 */
extern bStatus_t LssService_AddService( uint8_t rspTaskId );

/*
 * LssService_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t LssService_RegisterAppCBs( LssServiceCBs_t *appCallbacks );

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
extern bStatus_t LssService_SetParameter( uint8_t param, uint16_t len, void *value );

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
extern bStatus_t LssService_GetParameter( uint8_t param, uint16_t *len, void *value );


/*********************************************************************
// BASE UUID: 775ed4a2-8aa0-40f6-b037-ea770326e665 : 77 5e d4 a2 8a a0 40 f6 b0 37 ea 77 03 26 e6 65
#define LSS_BASE128_UUID(uuid)  0x65, 0xe6, 0x26, 0x03, 0x77, 0xea, 0x37, 0xb0, 0xf6, 0x40, 0xa0, 0x8a, \
                        LO_UINT16(uuid), HI_UINT16(uuid), 0x5e, 0x77

*********************************************************************/


#endif /* PROFILES_LSS_SERVICE_H_ */
