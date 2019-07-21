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

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */
#ifndef APPLICATION_COMMON_H_
#define APPLICATION_COMMON_H_

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

/*********************************************************************
 * MACROS
 */
#define member_size(type, member) sizeof(((type *)0)->member)

/*********************************************************************
 * FUNCTIONS
 */


/*
 * api_function_name - purpose
 *
 *    parameters
 */


#endif /* APPLICATION_COMMON_H_ */
