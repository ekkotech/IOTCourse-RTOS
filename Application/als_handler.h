/******************************************************************************
 * Filename:       als_handler.h
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

#ifndef APPLICATION_ALS_HANDLER_H
#define APPLICATION_ALS_HANDLER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// 100 Lux == well lit room; 1000 Lux = daylight, 10000 Lux = direct bright sunlight
// For a 10K resistor:
//  - 100 Lux = digital code of 476
//  - maximum Lux value = 660
// ADC has an internal voltage reference of 4.3V; ALS has supply voltage of 3.3V
// Maximum ADC reading is 3.3/4.3 * 4095 = 3142
// Resistor value can be adjusted to accommodate different Lux ranges
#define ADC_MULTIPLIER                  100
#define ADC_DIVISOR                     476

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * FUNCTIONS
 */
//
// ALS Handler entry points
void user_AlsService_ValueChangeHandler(char_data_t *pCharData);
void user_AlsService_CfgChangeHandler(char_data_t *pCharData);
void als_Hardware_Init();
void als_Resource_Init();
void als_ProcessPeriodicEvent(uint8_t isFirstRun);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif


#endif /* APPLICATION_ALS_HANDLER_H */
