/******************************************************************************
 * Filename:       labs.h
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

#ifndef APPLICATION_LABS_H_
#define APPLICATION_LABS_H_

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
// Lab source include definitions
//
// Un-comment sections as required. Note that earlier sections must be included prior to
// later sections. E.g. for Lab 3, LAB_1_INCLUDE and LAB_2_INCLUDE must be included
//
// Note that there is no #define for Lab 0 - unnecessary
// Lab 0 - Hardware test, Compile test
//
//#define     LAB_1       // Un-comment to include source for Lab 1 - Apple Interoperability
#ifdef      LAB_1
//#define     LAB_2       // Un-comment to include source for Lab 2 - Service Configuration
#ifdef      LAB_2
//#define     LAB_3       // Un-comment to include source for Lab 3 - Non-Volatile Memory
#ifdef      LAB_3
//#define     LAB_4       // Un-comment to include source for Lab 4 - Analogue Input
#ifdef      LAB_4
//#define     LAB_5       // Un-comment to include source for Lab 5 - Light Monitor implementation
#ifdef      LAB_5
//#define     LAB_6       // Un-comment to include source for Lab 6 - Random Fader Implementation
#ifdef      LAB_6
//#define     LAB_7       // Un-comment to include source for Lab 7 - Pairing and Bonding
#ifdef      LAB_7
#endif /* LAB_7 */
#endif /* LAB_6 */
#endif /* LAB_5 */
#endif /* LAB_4 */
#endif /* LAB_3 */
#endif /* LAB_2 */
#endif /* LAB_1 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * FUNCTIONS
 */


/*
 * api_function_name - purpose
 *
 *    parameters
 */


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif


#endif /* APPLICATION_LABS_H_ */
