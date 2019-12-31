/******************************************************************************
 * Filename:       lss_handler.h
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

#ifndef APPLICATION_LSS_HANDLER_H
#define APPLICATION_LSS_HANDLER_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "labs.h"

/*********************************************************************
 * CONSTANTS
 */

//
// LED related
//
#define NUM_LED_STRINGS             2
#define LED_STRING_0                0
#define LED_STRING_1                1
// Masks for led update function - OR these together to update either or both LED strings
#define LED_STRING_0_M              0x01
#define LED_STRING_1_M              0x02
// Maximum LEDs per string
#define MAX_LEDS_PER_STRING         60
// TODO: Check this - not clear if the padding is needed
// In-memory array is sized at 64 (see below) for fast updates but SK6812 LEDs are manufactured with cut points
// every 30 LEDs (there is also a 270 ohm terminating resistor at each cut point).
// Hence the actual length of the LED string should be no greater than 60
#define NUM_LEDS_PER_STRING         MAX_LEDS_PER_STRING
#define LED_MAX_VALUE               255
#define LED_MIN_VALUE               0
#define LED_PADDING                 4   // Padding makes the LED array size a power of two (64) which facilitates fast bulk updates
#define BULK_UPDATE_REPEAT_COUNT    6   // Number of iterations in the bulk update routine - 6 == 2^6 == 64 LEDs
#define RGB_COLOURS                 3
#define RGBW_COLOURS                4
#define NUM_COLOURS                 RGB_COLOURS
#define NIBBLES_PER_BYTE            2
#define HWORDS_PER_WORD             2

#define RED_INDEX                   0
#define GREEN_INDEX                 1
#define BLUE_INDEX                  2
#define WHITE_INDEX                 3

#define SWITCH_LEDS_OFF             OFF
#define SWITCH_LEDS_ON              ON
#define LEDS_OFF                    0x00000000
#define LEDS_ON_100                 0x00FFFFFF
#define LEDS_ON_50                  0x007F7F7F
#define RED_MASK                    0xFF000000
#define GREEN_MASK                  0x00FF0000
#define BLUE_MASK                   0x0000FF00
#define WHITE_MASK                  0x000000FF
#define RED_MASK_S                  24
#define GREEN_MASK_S                16
#define BLUE_MASK_S                 8
#define WHITE_MASK_S                0

#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
//
// Bit stream buffer location
#define USE_GPRAM                   // Comment out to use SRAM
#ifdef USE_GPRAM
#define BIT_STREAM_GPRAM_BASE       (uint16_t *)0x11001000      // Top 4k of GPRAM - must be 32-bit aligned
#endif

#endif /* LAB_3 */

// SSI channel configuration
//
#define SSI_NUM_CHANNELS                2
#define SSI_CHANNEL_0                   0
#define SSI_CHANNEL_1                   1
#define SSI_CHANNEL_0_DIO_NUM           IOID_21                 // Defined in ../source/ti/devices/cc26x0r2/driverlib/ioc.h
#define SSI_CHANNEL_1_DIO_NUM           IOID_22                 // Defined in ../source/ti/devices/cc26x0r2/driverlib/ioc.h
#define SSI_CHANNEL_0_PORT_ID           IOC_PORT_MCU_SSI0_TX    // Defined in ../source/ti/devices/cc26x0r2/driverlib/ioc.h
#define SSI_CHANNEL_1_PORT_ID           IOC_PORT_MCU_SSI1_TX    // Defined in ../source/ti/devices/cc26x0r2/driverlib/ioc.h
#define SSI_TX_BUFFER_SIZE              8                       // 8 x 16-bit values
#define SSI_CLOCK_PRESCALE_DIV_2        2                       // Source clock of 48MHz prescaled to 24MHz
#define SSI_CLOCK_DIV_6                 6                       // Pre-scaled clock further divided by (6 + 1) = 3.43MHz
#define SSI_INTERRUPT_NONE              0
//
#define SSI_WAIT_ON_TX_EMPTY_DELAY      40                      // Slightly longer than it takes to clock out a full SSI buffer (37us)
#define SSI_DELAY_100us                 100
#define SSI_DELAY_120us                 120
#define SSI_DELAY_150us                 150
#define SSI_MAX_DELAY                   500

//
// uDMA related
//
#define DMA_ERROR_INTERRUPT_NUMBER      25
#define DMA_ERROR_INTERRUPT_VECTOR      41
#define DMA_CONFIG_BASE_ADDR            0x20000400
#define DMA_NUM_CHANNELS                2
#define DMA_CHANNEL_4                   4
#define DMA_CHANNEL_17                  17
#define DMA_CHANNEL_SSI0                DMA_CHANNEL_4
#define DMA_CHANNEL_SSI1                DMA_CHANNEL_17
#define DMA_CHANNEL_SSI0_M              (1 << DMA_CHANNEL_4)
#define DMA_CHANNEL_SSI1_M              (1 << DMA_CHANNEL_17)
#define DMA_CHANNEL_SSI_BOTH_M          (DMA_CHANNEL_SSI0_M | DMA_CHANNEL_SSI1_M)
#define DMA_CONTROL_ALIGNMENT           1024
//
// Masks, shifts
// UDMA defines in ../source/ti/devices/26x0r2/driverlib/udma.h
#define UDMA_NEXT_USEBURST_M            UDMA_NEXT_USEBURST      // Redefine for consistency
#define DMA_CONTROL_WORD_M              (UDMA_DST_INC_M | UDMA_SRC_INC_M | UDMA_SIZE_M | UDMA_ARB_M | UDMA_NEXT_USEBURST_M | UDMA_MODE_M)

//
// Defaults
// See also characteristic default values in lss_service.h, als_service.h
//
#define LSS_DEFAULT_PEND_TIMEOUT_MS     50      // Milliseconds

//#ifdef LAB_n        // Lab 5 - Random Fader Implementation
// Fader timing (applies to any fader - Random fade or other):
// A fade 'cycle' is the changing of the LEDs from a start colour to an end colour
// Each fade cycle comprises 256 iterations with the LED colours being updated on each iteration
// A fade cycle period of 1 second requires an iteration period of 1/256s = ~4ms
// A cycle period of 1 second is the fastest permitted
// Use a small fade period during testing (e.g. 4 seconds), increase for production
#define FADE_DEFAULT_FADE_PERIOD    LSS_DEFAULT_FADE_PERIOD       // Fade period in seconds
#define FADE_MAX_FADE_PERIOD        60      // seconds
#define FADE_COLOUR_S               8       // Bit shift left for 16-bit unsigned integer calculations
#define FADE_DIRECTION_UP           1
#define FADE_DIRECTION_DOWN         0
#define FADE_NUM_ITERATIONS         256
#define FADE_COLOUR_CODE_MASK       0x0000FF00
// TRNG 64-bit random number; upper 32 bits are discarded; lower 32-bits assigned as below
#define TRNG_RED_MASK               0x000000FF
#define TRNG_GREEN_MASK             0x0000FF00
#define TRNG_BLUE_MASK              0x00FF0000
//#endif /* LAB_n */


/*********************************************************************
 * TYPEDEFS
 */

// SK6812 LED
// The colour layout for the SK6812 LED (GRB) differs from the Bluetooth representation (RGB)
// For efficiency, all colour structures in the application follow the SK6812 model thereby eliminating
// the need for translation when writing to the LEDs
typedef struct led {
    uint8_t green;      // Order of colours is as the SK6812 expects: G, R, B (W)
    uint8_t red;
    uint8_t blue;
} led_t;
// The colour layout in the bit stream buffer is 3 x 32-bit values
typedef struct bitStreamColour
{
    uint32_t green;
    uint32_t red;
    uint32_t blue;

} bitStreamColour_t;

//
// ledBitStream is an overlay of the bitStream array and is used for bulk updating of the bitStream array
//
typedef struct led32 {
    uint32_t green;
    uint32_t red;
    uint32_t blue;
} led32_t;

//
// SSI initialisation config
//
typedef struct {
    uint32_t    chAdr;
    uint32_t    portId;
    uint8_t     ioid;
} ssi_config_t;

//
// uDMA CCD config
//
typedef struct dma_config
{
    uint32_t *pvSrcEndAddr;     //!< The ending source address of the data transfer.
    uint32_t *pvDstEndAddr;     //!< The ending destination address of the data transfer.
    uint32_t ui32Control;       //!< The channel control mode.
    uint32_t ui32Spare;         //!< An unused location.
} dma_config_t;

//
// Fade control structures
//
// For fading, do all calculations as 32-bit integers. When writing to the LEDs,
// the 8-bit colour code is extracted from the second byte
typedef struct {
    int32_t nextColour;
    int32_t increment;
} colour_control_t;

typedef struct {
    Clock_Struct clockDef;
    Clock_Params clockParams;
    uint32_t timeout;           // This is the actual timeout value in clock ticks
                                // used during clock construction
} fade_clock_t;

typedef struct {
    fade_clock_t clock;
    colour_control_t red;
    colour_control_t green;
    colour_control_t blue;
    uint8_t period;             // The period of the fade cycle (in seconds)
    uint8_t iterationCount;
} fade_control_t;

//
// Program enum
//
typedef enum prog_id {
    RGB_SLIDER      = 0,
    RAND_FADE
} prog_id_t;


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */
//
// LSS Handler entry points
#ifdef LAB_2        // LAB_2 - Service Configuration
void user_LssService_ValueChangeHandler(char_data_t *pCharData);
#endif /* LAB_2 */

#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
void lss_Hardware_Init();
void lss_Resource_Init();
#endif

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
void lss_ProcessPeriodicEvent();
#endif

#ifdef LAB_5        // LAB_5 - Analogue Input
void lss_ProcessLightLevelChange();
#endif /* LAB_5 */

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif


#endif /* APPLICATION_LSS_HANDLER_H */
