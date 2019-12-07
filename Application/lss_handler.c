/******************************************************************************
 * Filename:       lss_handler.c
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

#include "icall_ble_api.h"
#include <icall.h>

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/devices/cc26x0r2/driverlib/vims.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <driverlib/ioc.h>
#include <driverlib/prcm.h>
#include <driverlib/ssi.h>
#include <driverlib/trng.h>
#include <driverlib/udma.h>
#include <driverlib/aux_adc.h>
#include <driverlib/aux_wuc.h>

#include <xdc/runtime/Diags.h>
#include <uartlog/UartLog.h>
#include <osal_snv.h>

#include "project_zero.h"
#include "lss_handler.h"
#include "lss_service.h"
#include "als_service.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
// A binary semaphore signalling completion of a DMA transfer (complete transfer of a configured block)
static Semaphore_Params dmaCompleteSemaParams;
static Semaphore_Struct dmaCompleteSema;
static Semaphore_Handle hDmaCompleteSema;

//
// Channel Control Descriptors (CCDs) for DMA channel 4 (SSI0) and DMA channel 17 (SSI1)
// Type dma_config_t is defined in lss_handler.h
#pragma LOCATION( ssi0ControlBlock, DMA_CONFIG_BASE_ADDR + (DMA_CHANNEL_SSI0 * sizeof(dma_config_t)) )
static dma_config_t ssi0ControlBlock;
#pragma LOCATION(ssi1ControlBlock, DMA_CONFIG_BASE_ADDR + (DMA_CHANNEL_SSI1 * sizeof(dma_config_t)) )
static dma_config_t ssi1ControlBlock;

// bitStreamTable contains the bit patterns to transmit via SSI. The first half corresponds to SSI0, the second half SSI1
// DMA writes these bit patterns directly to the SSI peripheral. The table is populated by taking each 4-bit nibble of a GRB
// colour code and looking up the SSI bit pattern in the ledBitStream table. The SK6812 LEDs require data ordered as green, red, blue
// with the MSb first. Hence, the bitStreamTable should be populated with the lookup value in ascending memory locations as:
//          ledBitStream[most significant nibble of green]
//          ledBitStream[least significant nibble of green]
//          ledBitStream[most significant nibble of red]
//          ledBitStream[least significant nibble of red]
//          ledBitStream[most significant nibble of blue]
//          ledBitStream[least significant nibble of blue]
//              repeat as needed...
// Require bitStream aligned on 4 byte boundary (32-bit aligned) for fast bulk updating
//static uint16_t *pBitStream[NUM_LED_STRINGS] = { &bitStream[0], &bitStream[NUM_LEDS_PER_STRING * NUM_COLOURS * HWORDS_PER_WORD] };

#ifdef USE_GPRAM
static uint16_t *pBitStream = BIT_STREAM_GPRAM_BASE;
#else
#pragma DATA_ALIGN ( bitStream, 4 )
static uint16_t bitStream[NUM_LED_STRINGS * NUM_LEDS_PER_STRING * NUM_COLOURS * HWORDS_PER_WORD];
static uint16_t *pBitStream = (uint16_t *)&bitStream;
#endif

static const uint16_t bitPatternTable[16] = {
                                             0x8888,   // 0b1000 1000 1000 1000
                                             0x888c,     // 0b1000 1000 1000 1100
                                             0x88c8,     // 0b1000 1000 1100 1000
                                             0x88cc,     // 0b1000 1000 1100 1100
                                             0x8c88,     // 0b1000 1100 1000 1000
                                             0x8c8c,     // 0b1000 1100 1000 1100
                                             0x8cc8,     // 0b1000 1100 1100 1000
                                             0x8ccc,     // 0b1000 1100 1100 1100
                                             0xc888,     // 0b1100 1000 1000 1000
                                             0xc88c,     // 0b1100 1000 1000 1100
                                             0xc8c8,     // 0b1100 1000 1100 1000
                                             0xc8cc,     // 0b1100 1000 1100 1100
                                             0xcc88,     // 0b1100 1100 1000 1000
                                             0xcc8c,     // 0b1100 1100 1000 1100
                                             0xccc8,     // 0b1100 1100 1100 1000
                                             0xcccc };    // 0b1100 1100 1100 1100

static rgb_char_t ledsOff = { .green = 0, .red = 0, .blue = 0 }; // Utility for turning LEDs off

#endif /* LAB_3 */

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
// Used to trigger setting of LEDs on start-up
// Insert flag here

#endif /* LAB_4  */

//
// Indicates if light level is currently below off/on threshold
//
#ifdef LAB_5        // LAB_5 - Analogue Input
static bool isBelowLMThreshold = false;
#endif /* LAB_5 */

#ifdef LAB_6        // LAB_6 - Random Fader Implementation
// Random Fader clock
// Random Fader control variables
static fade_control_t fadeControl = { .period = FADE_DEFAULT_FADE_PERIOD, .iterationCount = 0 };
#endif /* LAB_6 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
#ifdef LAB_2        // LAB_2 - Service Configuration
void user_LssService_ValueChangeHandler( char_data_t *pCharData );
#endif /* LAB_2 */

#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
void lss_Hardware_Init();
void lss_Resource_Init();
#endif /* LAB_3 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
#ifdef USE_GPRAM
static void disableCache();
#endif
//static void enableCache();

#ifdef LAB_2        // LAB_2 - Service Configuration
static void processOffOnValueChange( char_data_t *pCharData );
static void processRGBValueChange( char_data_t *pCharData );
#endif /* LAB_2 */

#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
static void initSSI();
static void initDMA();
static void initResources();
static void bulkUpdateLeds( rgb_char_t *pColour );
static void writeLeds( Semaphore_Handle handle, uint16_t timeout );
static void dmaCompleteHwiFxn( UArg arg );
static void waitOnSsiSendComplete( void );
#endif /* LAB_3 */

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
static void updateSnvState( uint8_t charId, uint16_t len, uint8_t *pData );
#endif /* LAB_4 */

#ifdef LAB_5        // LAB_5 - Analogue Input
static void startProgram(uint8_t program);
static void stopProgram(uint8_t program);
static void checkLuminanceThreshold();
#endif /* LAB_5 */

#ifdef LAB_6        // LAB_6 - Random Fader Implementation
static void processFadeTimeout();
static void fadeTimeoutSwiFxn(UArg period);
// TRNG related
static void initTRNG();
static uint32_t getTrngRandNumber();
#endif /* LAB_6 */

/*********************************************************************
 * EXTERN FUNCTIONS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

#ifdef LAB_2        // LAB_2 - Service configuration
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
void user_LssService_ValueChangeHandler( char_data_t *pCharData )
{
    static uint8_t pretty_data_holder[16]; // 5 bytes as hex string "AA:BB:CC:DD:EE"
    Util_convertArrayToHexString( pCharData->data, pCharData->dataLen,
                                  pretty_data_holder,
                                  sizeof(pretty_data_holder) );

    switch (pCharData->paramID)
    {
        case LSS_OFFON_ID:
            processOffOnValueChange( pCharData );
            break;

        case LSS_RGB_ID:
            processRGBValueChange( pCharData );
            break;

        default:
            return;
    }
}
#endif /* LAB_2 */

#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
/*
 * @fn      lss_Hardware_Init
 *
 * @brief   Initialise service-specific hardware
 *
 * @param   none
 *
 * @return  none
 */
void lss_Hardware_Init()
{
#ifdef USE_GPRAM
    disableCache();
#endif

    // Initialise SSI and DMA
    initSSI();
    initDMA();

#ifdef LAB_6        // LAB_6 - Random Fader Implementation
    initTRNG();
#endif /* LAB_6 */

}

/*
 * @fn      lss_Resource_Init
 *
 * @brief   Initialise any service-specific resources
 *
 * @param   none
 *
 * @return  none
 */
void lss_Resource_Init()
{
    // Initialise software resources
    initResources();
}
#endif /* LAB_3 */

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
/*
 * @fn      lss_ProcessPeriodicEvent
 *
 * @brief   Perform any required processing in a periodic basis
 *
 * @param   none
 *
 * @return  none
 *
 */
void lss_ProcessPeriodicEvent()
{

    Log_info0("In lss_ProcessPeriodicEvent");

    // Insert handler code here


//#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
//    if (isFirstRun == TRUE)
//    {
//        // Initialise LEDs to some state
//#ifndef LAB_4       // LAB_4 - Non-Volatile Memory
//        led_t *pColour = &ledsOff;
//#else
//        led_t *pColour = snvState.offOn ? &snvState.colour : &ledsOff;
//#endif /* LAB_4 */
//        bulkUpdateLeds( pColour );
//        writeLeds(hDmaCompleteSema, LSS_DEFAULT_PEND_TIMEOUT_MS);
//        isFirstRun = FALSE;
//    }
//#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
//    saveSnvState(SNV_APP_ID, sizeof(snvState), (uint8_t *)&snvState);
//#endif /* LAB_4 */
//
//#ifdef LAB_5
//    checkLuminanceThreshold();
//#endif /* LAB_5 */
//#endif /* LAB_3 */
}
#endif /* LAB_4 */

/*
 * @fn      lss_ProcessFadeTimeoutEvent
 *
 * @brief   Process random fader timeout event
 *
 * @param   none
 *
 * @return  none
 *
 */
#ifdef LAB_6        // LAB_6 - Random Fader Implementation
void lss_ProcessFadeTimeoutEvent()
{
    processFadeTimeout();

}
#endif /* LAB__6 */

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
#ifdef LAB_2        // LAB_2 - Service Configuration

/*
 * @fn      processOffOnValueChange
 *
 * @brief   Process a OFFON characteristic value change
 *
 * @param   pCharData  Pointer to the characteristic data
 *
 * @return  None.
 */
static void processOffOnValueChange( char_data_t *pCharData )
{
    Log_info0( "In processOffOnValueChange" );

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
    if (pCharData->dataLen == sizeof(offon_char_t))
    {
    // Insert handler code here

    }
    else {
        Log_info0("Invalid length for offOn data");
    }
#endif /* LAB_4 */

#ifdef LAB_3x        // LAB_3 - LED String Driver Implementation
#ifndef LAB_4        // LAB_4 - Non-Volatile Memory

    if ((pCharData->dataLen == sizeof(offon_char_t)))
    {
        led_t thisColour = ledsOff;
        uint16_t rgbLen = sizeof(rgb_char_t);
        rgb_char_t thisRgb;

        if (*((offon_char_t *) pCharData->data))
        {
            LssService_GetParameter( LSS_RGB_ID, &rgbLen, &thisRgb );
            thisColour.green = thisRgb.green;
            thisColour.red = thisRgb.red;
            thisColour.blue = thisRgb.blue;
        }
        bulkUpdateLeds( &thisColour );
        writeLeds( hDmaCompleteSema, LSS_DEFAULT_PEND_TIMEOUT_MS );

    }
#else
    if ((pCharData->dataLen == sizeof(offon_char_t)) && (snvState.offOn != *((offon_char_t *)pCharData->data)))
    {
        updateSnvState(pCharData->paramID, pCharData->dataLen, pCharData->data);

        switch (snvState.program)
        {
            case RGB_SLIDER:
            bulkUpdateLeds( snvState.offOn == ON ? &snvState.colour : &ledsOff );
            writeLeds(hDmaCompleteSema, LSS_DEFAULT_PEND_TIMEOUT_MS);
            break;
#ifdef LAB_6        // LAB_6 - Random Fader Implementation
            case RAND_FADE:
            if (((snvState.offOn == ON) && !snvState.lmOffOn) || (snvState.lmOffOn && isBelowLMThreshold))
            {
                startProgram(snvState.program);
            }
            else
            {
                stopProgram(snvState.program);
                bulkUpdateLeds( &ledsOff );
                writeLeds(hDmaCompleteSema, LSS_DEFAULT_PEND_TIMEOUT_MS);
            }
#endif  /* LAB_6 */
        }
    }

#endif /* LAB_4 */
#endif /* LAB_3 */
}

/*
 * @fn      processRGBValueChange
 *
 * @brief   Sets LEDs to new colour
 *
 * @param   pCharData - pointer to characteristic data
 *
 * @return  none
 */
static void processRGBValueChange( char_data_t *pCharData )
{
    Log_info0( "In processRGBValueChange" );

    if (pCharData->dataLen == sizeof(rgb_char_t))
    {
     // Insert handler code here
 
    }
    else {
        Log_info0("Incorrect data size for RGB value change");
    }




#ifdef LAB_3x        // LAB_3 - LED String Driver Implementation

#ifndef LAB_4     // LAB_4 - Non-Volatile Memory
    if (pCharData->dataLen == sizeof(rgb_char_t))
    {
        rgb_char_t *pRgb = (rgb_char_t *) pCharData->data;
        bulkUpdateLeds( pRgb );
        writeLeds( hDmaCompleteSema, LSS_DEFAULT_PEND_TIMEOUT_MS );
    }
    else {
        Log_info0("Incorrect data size for RGB value change");
    }
#else
    if (pCharData->dataLen == sizeof(rgb_char_t))
    {
//        led_t grb = { .green = ((rgb_char_t *)pCharData->data)->green,
//                      .red = ((rgb_char_t *)pCharData->data)->red,
//                      .blue = ((rgb_char_t *)pCharData->data)->blue};

        updateSnvState(LSS_RGB_ID, pCharData->dataLen, &pCharData->data);

        if (snvState.offOn == ON)
        {
            bulkUpdateLeds( &snvState.colour );
            writeLeds(hDmaCompleteSema, LSS_DEFAULT_PEND_TIMEOUT_MS);
        }
    }
    else {
        Log_info0("Incorrect data size for RGB value change");
    }
#endif /* LAB_4 */
#endif /* LAB_3 */
}

#endif /* LAB_2 */

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
 * @fn      dmaCompleteHwiFxn
 *
 * @brief   Hardware interrupt function for uDMA complete
 *
 * @param   arg
 *
 * @return  none
 */
#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
static void dmaCompleteHwiFxn( UArg arg )
{
    // Add HwiFxn handler code here
    // Disable SSI DMA
    
    
    // Clear uDMA REQDONE bits
    
    // Notify task DMA done
    

}
#endif /* LAB_3 */

#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
/*
 *  Hardware initialisation
 *****************************************************************************/
/*
 * @fn      init_SSI
 *
 * @brief   Initialises the SSI channels (SSI0 and SSI1)
 *
 * @param   none
 *
 * @return  none
 */
static void initSSI()
{

    // Turn on power and clock for both SSI channels
    Power_setDependency( PowerCC26XX_PERIPH_SSI0 ); // Enable power & clock to SSI0
    Power_setDependency( PowerCC26XX_PERIPH_SSI1 ); // Enable power & clock to SSI1

    // Enable SSI0 and SSI1 for run-mode operation
    PRCMPeripheralRunEnable( PRCM_PERIPH_SSI0 );
    PRCMPeripheralRunEnable( PRCM_PERIPH_SSI1 );
    // Enable SSI0 and SSI1 to operate while the MCU is in sleep mode
    PRCMPeripheralSleepEnable( PRCM_PERIPH_SSI0 );
    PRCMPeripheralSleepEnable( PRCM_PERIPH_SSI1 );
    // Enable SSI0 and SSI1 to operate while the MCU in deep sleep mode
    PRCMPeripheralDeepSleepEnable( PRCM_PERIPH_SSI0 );
    PRCMPeripheralDeepSleepEnable( PRCM_PERIPH_SSI1 );
    // The above calls only set up shadow registers
    // Load the shadow registers for the changes to take effect
    PRCMLoadSet();

    ssi_config_t ssi[SSI_NUM_CHANNELS] = { { .chAdr = SSI0_BASE, .portId = SSI_CHANNEL_0_PORT_ID, .ioid = SSI_CHANNEL_0_DIO_NUM },
                                           { .chAdr = SSI1_BASE, .portId = SSI_CHANNEL_1_PORT_ID, .ioid = SSI_CHANNEL_1_DIO_NUM } };

    // LAB_3_TODO_1
    
    // Add initialisation code here
    // Iterate over num SSI channels
    
        // Disable SSI channel
        
        // Set clock pre-scaler to 2 (divides the system clock by 2 to 24MHz)
        
        // Configure clock divisor, frame format, data width
        

        // Set up I/O
        // Set up the output pin for each channel
        
        
        
        // Enable output
        

        // Enable SSI channel

}
#endif /* LAB_3 */

#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
/*
 * @fn      initDMA
 *
 * @brief   Initialises the uDMA controller
 *
 * @discussion  Channels - SSI0 TX = channel 4; SSI1 TX = channel 17
 *
 * @param   none
 *
 * @return  none
 */
static void initDMA()
{

    // Turn on power and clock for uDMA peripheral
    Power_setDependency( PowerCC26XX_PERIPH_UDMA );

    // Enable uDMA in run mode, sleep and deep sleep
    PRCMPeripheralRunEnable( PRCM_PERIPH_UDMA );
    PRCMPeripheralSleepEnable( PRCM_PERIPH_UDMA );
    PRCMPeripheralDeepSleepEnable( PRCM_PERIPH_UDMA );
    PRCMLoadSet();      // Load the shadow registers

    // Add initialisation code here
    // Disable all channels before making changes
    
    // Set the base address of the uDMA control table. This is fixed at 0x2000_0400 - mask lower bits as a precaution

    // Enable the uDMA peripheral
    
    

    // The source, destination addresses and transfer modes for both channels do not change so we can set them up here
    
    
    
    

    // The control word for both channels is identical
    
    
    // This bit-wise AND with zero is to force a read of each control block to keep the compiler happy
    // Without this, the complier thinks that the control blocks are being set but not read hence wasting RAM space
    
    

}
#endif /* LAB_3 */

#ifdef LAB_6        // LAB_6 - Random Fader Implementation
/*
 * @fn      initTRNG
 *
 * @brief   Initialises the true random number generator
 *
 * @param   none
 *
 * @return  none
 */
static void initTRNG()
{

    Power_setDependency(PowerCC26XX_PERIPH_TRNG); // Enable power & clock to TRNG

    // Reset the module
    HWREG(TRNG_BASE + TRNG_O_SWRESET) = 1;
    // Wait for reset to complete
    while (HWREG(TRNG_BASE + TRNG_O_SWRESET) > 0)
    {}

    TRNGConfigure(0, 256, 0); // Min samples = max samples, max samples = 2^8, sample every clock
    // Enable the TRNG
    HWREGBITW(TRNG_BASE + TRNG_O_CTL, TRNG_CTL_TRNG_EN_BITN) = 1;

}
#endif  /* LAB_6 */

/*
 *  Resource initialisation
 *****************************************************************************/
/*
 * @fn      initResources
 *
 * @brief   Initialises any required software resources
 *
 * @param   none
 *
 * @return  none
 */
#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
static void initResources()
{
    // Add initialisation code here
    // Create DMA complete semaphore

    // Create Hwi
    // Only need a Hwi for SSI channel 1
 

#ifdef LAB_6        // LAB_6 - Random Fader Implementation
    // Create the clock for the fader
    Clock_Params_init(&fadeControl.clock.clockParams);
    fadeControl.clock.clockParams.arg = PRZ_FADE_TIMEOUT_EVT;
    // Initialise to default timeout
    // Timeout is in clock ticks (256 iterations per period; clock tick is 10us)
    Clock_construct(&fadeControl.clock.clockDef, fadeTimeoutSwiFxn,
            ((FADE_DEFAULT_FADE_PERIOD * USEC_PER_SEC) / FADE_NUM_ITERATIONS) / Clock_tickPeriod,
            &fadeControl.clock.clockParams);
#endif /* LAB_6 */

}
#endif /* LAB_3 */

/******************************************************************************
 *****************************************************************************
 *
 *  Utility functions
 *
 ****************************************************************************
 *****************************************************************************/
#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
/*
 * @fn      bulkUpdateLeds
 *
 * @brief   Updates the entire LED array with the designated colour
 *
 * @discussion  bulkUpdateLeds fills the entire LED array with a single colour
 *
 * @param   pColour - pointer to structure containing SK6812 (GRB) format colour
 *
 * @return  none
 */
static void bulkUpdateLeds( rgb_char_t *pColour )
{

    bitStreamColour_t *colour = (bitStreamColour_t *) pBitStream;
    uint8_t *colourBytes = (uint8_t *) pBitStream;

    colour->green = (bitPatternTable[pColour->green >> 4] | (bitPatternTable[pColour->green & 0x0F] << 16));
    colour->red = (bitPatternTable[pColour->red >> 4] | (bitPatternTable[pColour->red & 0x0F] << 16));
    colour->blue = (bitPatternTable[pColour->blue >> 4] | (bitPatternTable[pColour->blue & 0x0F] << 16));

    for (uint8_t idx = 1; idx < (NUM_LED_STRINGS * NUM_LEDS_PER_STRING); ++idx)
    {
        memcpy( colourBytes + (idx * (NUM_COLOURS * sizeof(uint32_t))),
                colourBytes, (NUM_COLOURS * sizeof(uint32_t)) );
    }
}
#endif /* LAB_3 */

#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
/*
 * @fn      writeLeds
 *
 * @brief   Initiates the transmission of the current contents of the LED buffer
 *          and pends on completion semaphore
 *
 * @discussion  The data transmitted is always for both LED strings and
 *          for the maximum number of LEDs per string regardless of how many
 *          strings are actually attached and how many LEDs per string.
 *          This is to ensure that the worst-case condition is being handled
 *          under all circumstances without any interruption to the radio.
 *          Once both SSI buffers have been primed, and hardware interrupt enabled,
 *          ws_WriteLeds() immediately pends on the SSI completion semaphore waiting
 *          for the remaining LED data to be transmitted by the interrupt service routine
 *
 * @param   handle - handle to a semaphore object on which to pend
 *          timeout - the pend timeout period in milliseconds
 *
 * @return  none
 */
static void writeLeds( Semaphore_Handle handle, uint16_t timeout )
{

    // Add handler code here

}
#endif /* LAB_3 */

#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
/*
 * @fn      waitOnSsiSendComplete
 *
 * @brief   Waits for SSI to transmit all data and adds an incremental delay
 *          to account for SK6812 reset timing requirement
 *
 * @param   arg
 *
 * @return  none
 */
static void waitOnSsiSendComplete()
{

    // Insert handler code here

}
#endif /* LAB_3 */

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

}
#endif /* LAB_4 */

#ifdef LAB_4        // LAB_4 - Non-volatile memory
/*
 * @fn      saveSnvState
 *
 * @brief   Writes snvState in memory to FLASH if snvState has been written to since
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

    // Insert handelr code here
    //
    // Write out snv structure to SNV if it has changed
    //
    if (snvIsDirty == true)
    {
        uint32_t status = osal_snv_write( appId, sizeof(snv_config_t), (uint8_t *)pState );
        if (status == SUCCESS)
        {
            snvIsDirty = false;
        }
        else
        {   Log_info0("SNV write failed");}
    }

}
#endif /* LAB_4 */

#ifdef LAB_5        // LAB_5 - Analogue Input
/*
 * @fn      checkLuminanceThreshold
 *
 * @brief   Updates characteristics values after a read from SNV
 *
 * @param   pState - pointer to the in-memory SNV state
 *
 * @return  none
 */
static void checkLuminanceThreshold()
{

    uint16_t currentValue = 0;
    uint16_t currentValueLen = ALS_LUMIN_LEN;

    AlsService_GetParameter( ALS_LUMIN_ID, &currentValueLen, &currentValue);

    if ( currentValue <= snvState.lmThreshold && !isBelowLMThreshold )
    {
        // Light level has fallen below lower threshold
        if ( snvState.offOn && snvState.lmOffOn)
        {
            // Set LEDs on
            startProgram(snvState.program);
        }
        isBelowLMThreshold = true;
    }
    else if ( currentValue >= (snvState.lmThreshold * (1 + ((float)snvState.lmHysteresis / 100)) ) )
    {
        // Light level has risen above upper threshold
        if ( snvState.offOn && snvState.lmOffOn )
        {
            // Set LEDs off
            stopProgram(snvState.program);
            bulkUpdateLeds( &ledsOff );
            writeLeds(hDmaCompleteSema, LSS_DEFAULT_PEND_TIMEOUT_MS);
        }
        isBelowLMThreshold = false;
    }
    // No change if light level is between low and high thresholds
}
#endif /* LAB_5 */

/*********************************************************************
 * @fn      startProgram
 *
 * @brief   Initialises and starts a program
 *
 * @param   program - identifier for the program
 *
 * @return  none
 */
#ifdef LAB_5        // LAB_5 - Analogue Input
static void startProgram(uint8_t program)
{

    switch (program)
    {
        case RGB_SLIDER:
        {
            bulkUpdateLeds( snvState.offOn == ON ? &snvState.colour : &ledsOff );
            writeLeds(hDmaCompleteSema, LSS_DEFAULT_PEND_TIMEOUT_MS);
            break;
        }
#ifdef LAB_6        // LAB_6 - Random Fader Implementation
        case RAND_FADE:
        {
            // If snvState.offOn == ON, get initial seed from TRNG, start timer
            // If snvState == OFF, switch off LEDs
            if (snvState.offOn == ON)
            {
                fadeControl.iterationCount = 0;
                uint32_t seed = getTrngRandNumber();
                // Set up initial colours
                led_t filler =
                {   .red = (seed & TRNG_RED_MASK), .green = (seed & TRNG_GREEN_MASK) >> 8, .blue = (seed & TRNG_BLUE_MASK) >> 16};
                bulkUpdateLeds( &filler );
                Clock_start(Clock_handle(&fadeControl.clock.clockDef));
            }
            else
            {
                bulkUpdateLeds( &ledsOff );
                writeLeds(hDmaCompleteSema, LSS_DEFAULT_PEND_TIMEOUT_MS);
            }
            break;
        }
#endif /* LAB_6 */
        default:
        Log_info0("WARNING: lssStartProgram: Invalid program id");
        break;
    }
}
#endif /* LAB_5 */

/*
 * @fn      stopProgram
 *
 * @brief   Stops a program, freeing any resources, halting timers etc.
 *
 * @param   program - identifier for the program
 *
 * @return  none
 */
#ifdef LAB_5        // LAB_5 - Analogue Input
static void stopProgram(uint8_t program)
{

    switch (program)
    {
        case RGB_SLIDER:
        // Nothing needed here
        break;
#ifdef LAB_6        // LAB_6 - Random Fader Implementation
        case RAND_FADE:
        // Just stopping the clock will leave the LEDs in a lit state
        Clock_stop(Clock_handle(&fadeControl.clock.clockDef));
        break;
#endif /* LAB_6 */
        default:
        Log_info0("WARNING: lssStopProgram: Invalid program id");
        break;
    }
}
#endif /* LAB_5 */

/*
 * @fn      getTrngRandNumber
 *
 * @brief   Reads the TRNG
 *
 * @discussion  The TRNG takes 2^8 clock cycles (~5us) to generate the first random number
 *          and 2^8 clock cycles to regenerate each additional random number
 *          In the event that TRNG is not ready to supply the first random number, an incrementing number is returned
 *
 * @return  32-bit random number
 */
#ifdef LAB_6        // LAB_6 - Random Fader Implementation
static uint32_t getTrngRandNumber()
{

    static uint32_t filler;
    uint32_t lowRand;

    if ((HWREG(TRNG_BASE + TRNG_O_IRQFLAGSTAT) & TRNG_NUMBER_READY) != 0)
    {
        // Get the low 32-bits of the random number
        lowRand = HWREG(TRNG_BASE + TRNG_O_OUT0);
        // Kick off generation of a new number
        HWREG(TRNG_BASE + TRNG_O_IRQFLAGCLR) = 0x1;
    }
    else
    {
        lowRand = ++filler;
    }

    return lowRand;

}
#endif /* LAB_6 */

#ifdef USE_GPRAM
/*********************************************************************
 * @fn      disableCache
 *
 * @brief   Disables the instruction cache and sets power constraints
 *          This prevents the device from sleeping while using GPRAM
 *
 * @param   None.
 *
 * @return  None.
 */
static void disableCache()
{
    uint_least16_t hwiKey = Hwi_disable();
    Power_setConstraint( PowerCC26XX_SB_VIMS_CACHE_RETAIN );
    Power_setConstraint( PowerCC26XX_NEED_FLASH_IN_IDLE );
    VIMSModeSafeSet( VIMS_BASE, VIMS_MODE_DISABLED, true );
    Hwi_restore(hwiKey);
}
#endif

/*********************************************************************
 * @fn      enableCache
 *
 * @brief   Enables the instruction cache and releases power constraints
 *          Allows device to sleep again
 *
 * @param   None.
 *
 * @return  None.
 */
//static void enableCache()
//{
//    uint_least16_t hwiKey = Hwi_disable();
//    Power_releaseConstraint(PowerCC26XX_SB_VIMS_CACHE_RETAIN);
//    Power_releaseConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
//    VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_ENABLED, true);
//    Hwi_restore(hwiKey);
//}
/*********************************************************************
 *
 *     for (uint8_t idx = 0; idx < SSI_NUM_CHANNELS; ++idx)
    {
        // SSI channels are disabled by default after reset
        // Set clock pre-scaler to 2 (divides the system clock by 2 to 24MHz)
        HWREG(ssi[idx].chAdr + SSI_O_CPSR) = SSI_CLOCK_PRESCALE_DIV_2;
        // Configure clock divisor, frame format, data width
        HWREG(ssi[idx].chAdr + SSI_O_CR0) = ((SSI_CLOCK_DIV_6 << SSI_CR0_SCR_S) | SSI_CR0_FRF_TI_SYNC_SERIAL | SSI_CR0_DSS_16_BIT);

        // Set up I/O
        // Set up the output pin for each channel
        uint32_t temp;
        temp = HWREG( IOC_BASE + (ssi[idx].ioid * sizeof(uint32_t)) ) & ~IOC_IOCFG0_PULL_CTL_M;
        temp |= (IOC_IOCFG0_PULL_CTL_DWN | IOC_IOCFG0_IOCURR_4MA | ssi[idx].portId);
        HWREG(IOC_BASE + (ssi[idx].ioid * sizeof(uint32_t))) = temp;
        // Enable output
        HWREG(GPIO_BASE + GPIO_O_DOE31_0) |= (1 << ssi[idx].ioid);

        // Enable SSI channel
        HWREG(ssi[idx].chAdr + SSI_O_CR1) |= SSI_CR1_SSE;
    }
 *
 *     // Disable all channels before making changes
    HWREG(UDMA0_BASE + UDMA_O_CLEARCHANNELEN) = UDMA_CLEARCHANNELEN_CHNLS_M;
    // Enable the uDMA peripheral
    HWREG(UDMA0_BASE + UDMA_O_CFG) = UDMA_CFG_MASTERENABLE;
    // Set the base address of the uDMA control table. This is fixed at 0x2000_0400 - mask lower bits as a precaution
    HWREG(UDMA0_BASE + UDMA_O_CTRL) = (DMA_CONFIG_BASE_ADDR & UDMA_CTRL_BASEPTR_M);

    // The source, destination addresses and transfer modes for both channels do not change so we can set them up here
    ssi0ControlBlock.pvSrcEndAddr = (uint32_t *) (pBitStream + (NUM_LEDS_PER_STRING * NUM_COLOURS * HWORDS_PER_WORD) - 1);
    ssi0ControlBlock.pvDstEndAddr = (uint32_t *) (SSI0_BASE + SSI_O_DR);
    ssi1ControlBlock.pvSrcEndAddr = (uint32_t *) (pBitStream + (NUM_LED_STRINGS * NUM_LEDS_PER_STRING * NUM_COLOURS * HWORDS_PER_WORD) - 1);
    ssi1ControlBlock.pvDstEndAddr = (uint32_t *) (SSI1_BASE + SSI_O_DR);

    // The control word for both channels is identical
    uint32_t control = 0;
    control = (UDMA_DST_INC_NONE | UDMA_SRC_INC_16 | UDMA_SIZE_16 | UDMA_ARB_4 | UDMA_MODE_BASIC);
    // This bit-wise AND with zero is to force a read of each control block to keep the compiler happy
    // Without this, the complier thinks that the control blocks are being set but not read hence wasting RAM space
    ssi0ControlBlock.ui32Control = (ssi0ControlBlock.ui32Control & 0) | control;
    ssi1ControlBlock.ui32Control = (ssi1ControlBlock.ui32Control & 0) | control;

    // Create Hwi
    // Only need a Hwi for SSI channel 1
    Hwi_create( INT_SSI1_COMB, dmaCompleteHwiFxn, NULL, NULL );
 *
 *
 *     // DMA complete semaphore
    dmaCompleteSemaParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct( &dmaCompleteSema, 0, &dmaCompleteSemaParams );
    hDmaCompleteSema = Semaphore_handle( &dmaCompleteSema );
 *
 *
 *     // Ensure that both channels are disabled before making any changes
    if (!(HWREG(UDMA0_BASE + UDMA_O_SETCHANNELEN) & DMA_CHANNEL_SSI_BOTH_M))
    {
        uint32_t control;
        uint32_t transferCount = (((NUM_LEDS_PER_STRING * NUM_COLOURS * HWORDS_PER_WORD) - 1) << UDMA_XFER_SIZE_S);
        control = ssi0ControlBlock.ui32Control & ~(UDMA_XFER_SIZE_M | UDMA_MODE_M);
        control |= (transferCount | UDMA_MODE_BASIC);
        ssi0ControlBlock.ui32Control = control;
        ssi1ControlBlock.ui32Control = control;

        // Enable uDMA channels
        HWREG(UDMA0_BASE + UDMA_O_SETCHANNELEN) = DMA_CHANNEL_SSI_BOTH_M;
        // Enable SSI DMA operation
        HWREGBITW(SSI0_BASE + SSI_O_DMACR, SSI_DMACR_TXDMAE_BITN) = 1;
        HWREGBITW(SSI1_BASE + SSI_O_DMACR, SSI_DMACR_TXDMAE_BITN) = 1;

        if (handle != NULL)
        {
            if (Semaphore_pend( handle, timeout * (MSEC_PER_SEC / Clock_tickPeriod) ))
            {
                // Ensure that both SSI channels have completed any prior send
                // Add an additional delay to ensure the the 80us reset time for the SK6812 LEDs is met
                waitOnSsiSendComplete();
                Task_sleep( SSI_DELAY_100us / Clock_tickPeriod );
            }
            else
            {
                Log_info0( "Semaphore pend timeout" );
            }
        }
    }
 *
 *
 *     uint8_t loopCount = 0;

    while (!(HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_TFE_M)
            & (HWREG(SSI1_BASE + SSI_O_SR) & SSI_SR_TFE_M))
    {
        Task_sleep( SSI_WAIT_ON_TX_EMPTY_DELAY / Clock_tickPeriod );
        ++loopCount;

        if ((loopCount * SSI_WAIT_ON_TX_EMPTY_DELAY) > SSI_MAX_DELAY)
        {
            break;
        }
    }
 *
 *
 *     // Disable SSI DMA
    HWREGBITW(SSI0_BASE + SSI_O_DMACR, SSI_DMACR_TXDMAE_BITN) = 0;
    HWREGBITW(SSI1_BASE + SSI_O_DMACR, SSI_DMACR_TXDMAE_BITN) = 0;
    // Clear uDMA REQDONE bits
    HWREG(UDMA0_BASE + UDMA_O_REQDONE) = DMA_CHANNEL_SSI_BOTH_M;
    // Notify task DMA done
    Semaphore_post( hDmaCompleteSema );
 *
 *
 *
 *
 *
 *********************************************************************/
