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
#include <ti/drivers/power/PowerCC26XX.h>
#include <driverlib/ioc.h>
#include <driverlib/prcm.h>
#include <driverlib/ssi.h>
#include <driverlib/trng.h>
#include <driverlib/udma.h>
#include <driverlib/aux_adc.h>
#include <driverlib/aux_wuc.h>

//#include <xdc/runtime/Log.h>
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
 * GLOBAL VARIABLES
 */
extern ICall_SyncHandle syncEvent;

/*********************************************************************
 * LOCAL VARIABLES
 */
#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
// A binary semaphore signalling completion of a DMA transfer (complete transfer of a configured block)
static Semaphore_Params dmaCompleteSemaParams;
static Semaphore_Struct dmaCompleteSema;
static Semaphore_Handle hDmaCompleteSema;

#pragma LOCATION(ssi0ControlBlock, DMA_CONFIG_SSI0_ADDR)
static dma_config_t ssi0ControlBlock;
#pragma LOCATION(ssi1ControlBlock, DMA_CONFIG_SSI1_ADDR)
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
#pragma DATA_ALIGN ( bitStream, 4 )
static uint16_t bitStream[NUM_LED_STRINGS * NUM_LEDS_PER_STRING * NUM_COLOURS * NIBBLES_PER_BYTE];
static uint16_t *pBitStream[NUM_LED_STRINGS] = { &bitStream[0], &bitStream[NUM_LEDS_PER_STRING * NUM_COLOURS * NIBBLES_PER_BYTE] };
static const uint16_t bitPatternTable[16] = {
                                    0x8888,     // 0b1000 1000 1000 1000
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
                                    0xcccc};    // 0b1100 1100 1100 1100
static led32_t filler;      // For bulk updating LED bit pattern buffer; populate one 32-bit item and copy (faster than byte-wise memcpy()
static led_t ledsOff = { .green = 0, .red = 0, .blue = 0 };     // Utility for turning LEDs off
//
// Flag to handle any specific processing for the first periodic event
//
static uint8_t isFirstRun = TRUE;
#endif /* LAB_3 */

#ifdef LAB_4      // LAB_4 - Non-Volatile Memory
//
// SNV state
//
static uint8_t snvIsDirty = FALSE;
static snv_config_t snvState = { .offOn = LSS_OFFON_DEFAULT,
                            .colour = { .green = LSS_GREEN_DEFAULT, .red = LSS_RED_DEFAULT, .blue = LSS_BLUE_DEFAULT },
                            .range = { .red = { .lowLimit = LSS_LIMMIN_DEFAULT, .highLimit = LSS_LIMMAX_DEFAULT },
                                       .green = { .lowLimit = LSS_LIMMIN_DEFAULT, .highLimit = LSS_LIMMAX_DEFAULT },
                                       .blue = { .lowLimit = LSS_LIMMIN_DEFAULT, .highLimit = LSS_LIMMAX_DEFAULT }
                            },
                            .program = LSS_PROG_DEFAULT,
                            .fadePeriod = LSS_DEFAULT_FADE_PERIOD,
                            .lmThreshold = LSS_LMTHRESH_DEFAULT,
                            .lmHysteresis = LSS_LMHYST_DEFAULT,
                            .lmOffOn = LSS_LMOFFON_DEFAULT };

#endif /* LAB_4 */

//
// Indicates if light level is currently below off/on threshold
//
#ifdef LAB_5        // LAB_5 - Light Monitor Implementation
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
void user_LssService_ValueChangeHandler(char_data_t *pCharData);
#endif /* LAB_2 */

#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
void lss_Hardware_Init();
void lss_Resource_Init();
#endif /* LAB_3 */

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
void lss_ProcessPeriodicEvent();
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
#ifdef LAB_2        // LAB_2 - Service Configuration
static void processOffOnValueChange(char_data_t *pCharData);
static void processRGBValueChange(char_data_t *pCharData);
#endif /* LAB_2 */

#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
static void initSSI();
static void initDMA();
static void initResources();
static void bulkUpdateLeds(uint8_t ledStringMask, led_t *pColour);
static void writeLeds(Semaphore_Handle handle, uint16_t timeout);
static void dmaCompleteHwiFxn(UArg arg);
static void waitOnSsiSendComplete(void);
#endif /* LAB_3 */

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
static void initSnv(uint8_t appId, uint8_t len, uint8_t *pData);
static void updateSnvState(uint8_t charId, uint16_t len, uint8_t *pData);
static void saveSnvState(uint8_t appId, uint8_t len, uint8_t *pData);
static void setCharacteristicsFromSnv(snv_config_t *pState);
#endif /* LAB_4 */

#ifdef LAB_5        // LAB_5 - Light Monitor Implementation
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
void user_LssService_ValueChangeHandler(char_data_t *pCharData)
{
  static uint8_t pretty_data_holder[16]; // 5 bytes as hex string "AA:BB:CC:DD:EE"
  Util_convertArrayToHexString(pCharData->data, pCharData->dataLen,
                               pretty_data_holder, sizeof(pretty_data_holder));

  switch (pCharData->paramID)
  {
    case LSS_OFFON_ID:
      processOffOnValueChange(pCharData);
      break;

    case LSS_RGB_ID:
      processRGBValueChange(pCharData);
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
    // Initialise SSI and DMA
    initSSI();
    initDMA();

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
    initSnv(SNV_APP_ID, sizeof(snvState), (uint8_t *)&snvState);
#endif /* LAB_4 */

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

#ifdef LAB_4        // LAB_3 - Non-Volatile Memory
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

//#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
//    if (isFirstRun == TRUE)
//    {
//        // Initialise LEDs to some state
//#ifndef LAB_4       // LAB_3 - Non-Volatile Memory
//        led_t *pColour = &ledsOff;
//#else
//        led_t *pColour = snvState.offOn ? &snvState.colour : &ledsOff;
//#endif /* LAB_3 */
//        bulkUpdateLeds( (LED_STRING_0_M | LED_STRING_1_M), pColour );
//        writeLeds(hDmaCompleteSema, LSS_DEFAULT_PEND_TIMEOUT_MS);
//        isFirstRun = FALSE;
//    }
//#ifdef LAB_4        // LAB_3 - Non-Volatile Memory
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
static void processOffOnValueChange(char_data_t *pCharData)
{
    Log_info0("In processOffOnValueChange");

#ifdef LAB_3        // LAB_3 - LED String Driver Implementation
#ifndef LAB_4        // LAB_4 - Non-Volatile Memory


    if ((pCharData->dataLen == sizeof(offon_char_t)))
    {
        led_t thisColour = ledsOff;
        uint16_t rgbLen = sizeof(rgb_char_t);
        rgb_char_t thisRgb;

        if (*((offon_char_t *)pCharData->data))
        {
            LssService_GetParameter( LSS_RGB_ID, &rgbLen, &thisRgb );
            thisColour.green = thisRgb.green;
            thisColour.red = thisRgb.red;
            thisColour.blue = thisRgb.blue;
        }
        bulkUpdateLeds((LED_STRING_0_M | LED_STRING_1_M), &thisColour);
        writeLeds(hDmaCompleteSema, LSS_DEFAULT_PEND_TIMEOUT_MS);

    }
#else
    if ((pCharData->dataLen == sizeof(offon_char_t)) && (snvState.offOn != *((offon_char_t *)pCharData->data)))
    {
        updateSnvState(pCharData->paramID, pCharData->dataLen, pCharData->data);

        switch (snvState.program)
        {
        case RGB_SLIDER:
            bulkUpdateLeds((LED_STRING_0_M | LED_STRING_1_M), snvState.offOn == ON ? &snvState.colour : &ledsOff);
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
                bulkUpdateLeds((LED_STRING_0_M | LED_STRING_1_M), &ledsOff);
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
static void processRGBValueChange(char_data_t *pCharData)
{
    Log_info0("In processRGBValueChange");

#ifdef LAB_3        // LAB_3 - LED String Driver Implementation

#ifndef LAB_4     // LAB_4 - Non-Volatile Memory
    if (pCharData->dataLen == sizeof(rgb_char_t))
    {
        rgb_char_t *pRgb = (rgb_char_t *)pCharData->data;
        led_t grb = { .green = pRgb->green, .red = pRgb->red, .blue = pRgb->blue };
        bulkUpdateLeds((LED_STRING_0_M | LED_STRING_1_M), &grb);
        writeLeds(hDmaCompleteSema, LSS_DEFAULT_PEND_TIMEOUT_MS);
    }
#else
    if (pCharData->dataLen == sizeof(rgb_char_t))
    {
        led_t grb = { .green = ((rgb_char_t *)pCharData->data)->green, .red = ((rgb_char_t *)pCharData->data)->red, .blue = ((rgb_char_t *)pCharData->data)->blue };
        updateSnvState(LSS_RGB_ID, pCharData->dataLen, (uint8_t*)&grb);

        if (snvState.offOn == ON)
        {
            bulkUpdateLeds((LED_STRING_0_M | LED_STRING_1_M), &snvState.colour);
            writeLeds(hDmaCompleteSema, LSS_DEFAULT_PEND_TIMEOUT_MS);
        }
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
static void dmaCompleteHwiFxn(UArg arg)
{
    // Add HwiFxn handler code here
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
static void initSSI() {

    // Turn on power and clock for both SSI channels
    Power_setDependency(PowerCC26XX_PERIPH_SSI0);   // Enable power & clock to SSI0
    Power_setDependency(PowerCC26XX_PERIPH_SSI1);   // Enable power & clock to SSI1

    // Enable SSI0 and SSI1 for run-mode operation
    PRCMPeripheralRunEnable(PRCM_PERIPH_SSI0);
    PRCMPeripheralRunEnable(PRCM_PERIPH_SSI1);
    // Enable SSI0 and SSI1 to operate while the MCU is in sleep mode
    PRCMPeripheralSleepEnable(PRCM_PERIPH_SSI0);
    PRCMPeripheralSleepEnable(PRCM_PERIPH_SSI1);
    // Enable SSI0 and SSI1 to operate while the MCU in deep sleep mode
    PRCMPeripheralDeepSleepEnable(PRCM_PERIPH_SSI0);
    PRCMPeripheralDeepSleepEnable(PRCM_PERIPH_SSI1);
    // The above calls only set up shadow registers
    // Load the shadow registers for the changes to take effect
    PRCMLoadSet();

    uint32_t temp;
    ssi_config_t ssi[SSI_NUM_CHANNELS] = { { .chAdr = SSI0_BASE, .portId = SSI_CHANNEL_0_PORT_ID, .ioid = SSI_CHANNEL_0_OUTPUT_PIN },
                                           { .chAdr = SSI1_BASE, .portId = SSI_CHANNEL_1_PORT_ID, .ioid = SSI_CHANNEL_1_OUTPUT_PIN } };

    // Add initialisation code here
    }

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
    Power_setDependency(PowerCC26XX_PERIPH_UDMA);

    // Enable uDMA in run mode, sleep and deep sleep
    PRCMPeripheralRunEnable(PRCM_PERIPH_UDMA);
    PRCMPeripheralSleepEnable(PRCM_PERIPH_UDMA);
    PRCMPeripheralDeepSleepEnable(PRCM_PERIPH_UDMA);
    PRCMLoadSet();      // Load the shadow registers

    // Add initialisation code here

}
#endif /* LAB_3 */

#ifdef LAB_4        // LAB_4 - Non-Volatile Memory
/*
 * @fn      initSnv
 *
 * @brief   Initialises SNV memory
 *
 * @discussion  SNV state held in memory is initialised from SNV state held in FLASH if
 *          the SNV state in FLASH is valid. SNV state in memory is initialised statically
 *          to default settings. This static state is overwritten when SNV state is successfully
 *          read from FLASH.
 *          Each time that a new firmware image is uploaded, SNV state in FLASH is erased and
 *          a subsequent read will fail. In the event of a read failure, SNV state in FLASH is
 *          initialised with the statically defined SNV state in memory.
 *          In normal operation, after a power cycle or a reset, the SNV state in FLASH is
 *          loaded into memory.
 *
 * @param   appId - the ID for the SNV structure. Must be in the range 0x80 to 0x8F
 *          len - the number of bytes to read from FLASH
 *          pData - pointer to the memory area to load the FLASH data
 *
 * @return  none
 */
static void initSnv(uint8_t appId, uint8_t len, uint8_t *pData)
{

    uint32_t status = SUCCESS;

    status = osal_snv_read(appId, len, pData);
    if (status != SUCCESS)
    {
        // Most likely new firmware uploaded; write default snvState to SNV
        status = osal_snv_write(appId, len, pData);
        if (status != SUCCESS)
        {
            Log_info0("Unable to write snvState to FLASH");
        }
    }

}
#endif /* LAB_4 */

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

     Power_setDependency(PowerCC26XX_PERIPH_TRNG);   // Enable power & clock to TRNG

    // Reset the module
    HWREG(TRNG_BASE + TRNG_O_SWRESET) = 1;
    // Wait for reset to complete
    while (HWREG(TRNG_BASE + TRNG_O_SWRESET) > 0) { }

    TRNGConfigure(0, 256, 0);     // Min samples = max samples, max samples = 2^8, sample every clock
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
    // DMA complete semaphore
    dmaCompleteSemaParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&dmaCompleteSema, 0, &dmaCompleteSemaParams);
    hDmaCompleteSema = Semaphore_handle(&dmaCompleteSema);

#ifdef LAB_4        // LAB_4 - Non-volatile memory
    // Initialise characteristics
    setCharacteristicsFromSnv(&snvState);
#endif /* LAB_4 */

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
 *          For speed, a 32-bit filler pattern is first created and then copied
 *          to the LED array
 *
 *
 * @param   pColour - pointer to structure containing SK6812 (GRB) format colour
 *
 * @return  none
 */
static void bulkUpdateLeds(uint8_t ledStringMask, led_t *pColour) {

    filler.green = (bitPatternTable[pColour->green >> 4] | (bitPatternTable[pColour->green & 0x0F] << 16));
    filler.red = (bitPatternTable[pColour->red >> 4] | (bitPatternTable[pColour->red & 0x0F] << 16));
    filler.blue = (bitPatternTable[pColour->blue >> 4] | (bitPatternTable[pColour->blue & 0x0F] << 16));
    uint32_t *pb;

    // TODO: No need to use two indices here since the LED array is contiguous
    for (uint8_t idx = 0; idx < NUM_LED_STRINGS; ++idx) {
        if (ledStringMask & (1 << idx)) {
            pb = (uint32_t *)pBitStream[idx];
            for (uint8_t jdx = 0; jdx < NUM_LEDS_PER_STRING; ++jdx) {
                *pb++ = filler.green;
                *pb++ = filler.red;
                *pb++ = filler.blue;
            }
        }
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
static void writeLeds(Semaphore_Handle handle, uint16_t timeout) {

    // Ensure that both channels are disabled before making any changes
    if (!(HWREG(UDMA0_BASE + UDMA_O_SETCHANNELEN) & DMA_CHANNEL_SSI_BOTH_M))
    {
        uint32_t control0;
        uint32_t control1;
        control0 = (ssi0ControlBlock.ui32Control & ~(UDMA_XFER_SIZE_M)) | (((NUM_LEDS_PER_STRING * NUM_COLOURS * NIBBLES_PER_BYTE) - 1) << UDMA_XFER_SIZE_S);
        control1 = (ssi1ControlBlock.ui32Control & ~(UDMA_XFER_SIZE_M)) | (((NUM_LEDS_PER_STRING * NUM_COLOURS * NIBBLES_PER_BYTE) - 1) << UDMA_XFER_SIZE_S);
        ssi0ControlBlock.ui32Control = (control0 | UDMA_MODE_BASIC);
        ssi1ControlBlock.ui32Control = (control1 | UDMA_MODE_BASIC);

        // Enable uDMA channels
        HWREG(UDMA0_BASE + UDMA_O_SETCHANNELEN) = DMA_CHANNEL_SSI_BOTH_M;
        // Enable SSI DMA operation
        HWREG(SSI0_BASE + SSI_O_DMACR) |= SSI_DMACR_TXDMAE_M;
        HWREG(SSI1_BASE + SSI_O_DMACR) |= SSI_DMACR_TXDMAE_M;

        if (handle != NULL)
        {
            if (Semaphore_pend(handle, timeout * (MSEC_PER_SEC / Clock_tickPeriod)))
            {
                // Ensure that both SSI channels have completed any prior send
                // Add an additional delay to ensure the the 80us reset time for the SK6812 LEDs is met
                waitOnSsiSendComplete();
                Task_sleep(SSI_DELAY_100us / Clock_tickPeriod);    // TODO: Can this delay be folded into ws_WaitOnSsiSendSomplete ??
            }
            else
            {
                Log_info0("Semaphore pend timeout");
            }
        }
    }
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

    uint8_t loopCount = 0;

    while (!(HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_TFE_M) & (HWREG(SSI1_BASE + SSI_O_SR) & SSI_SR_TFE_M))
    {
        Task_sleep(SSI_WAIT_ON_TX_EMPTY_DELAY/Clock_tickPeriod);
        ++loopCount;

        if ((loopCount * SSI_WAIT_ON_TX_EMPTY_DELAY) > SSI_MAX_DELAY)
        {
            break;
        }
    }

    Task_sleep(SSI_DELAY_100us / Clock_tickPeriod);

}
#endif /* LAB_3 */

#ifdef LAB_4        // LAB_4 - Non-volatile memory
/*
 * @fn      updateSnvState
 *
 * @brief   Updates an element in snvState and sets dirty flag true
 *
 * @param   chard - the characteristic ID
 * @param   len - the number of bytes to write
 * @param   pData - pointer to the source data
 *
 * @return  none
 */
static void updateSnvState(uint8_t charId, uint16_t len, uint8_t *pData)
{
    switch (charId)
    {
     case LSS_OFFON_ID:
        if (len == sizeof(uint8_t)) {
            snvState.offOn = *pData;
        }
        break;
    case LSS_RGB_ID:
        if (len == sizeof(led_t)) {
            snvState.colour.green = ((led_t*)pData)->green;
            snvState.colour.red = ((led_t*)pData)->red;
            snvState.colour.blue = ((led_t*)pData)->blue;
        }
        break;
        // Move these to ALS Service
//    case LSS_LMTHRESH_ID:
//        if (len == sizeof(uint16_t)) {
//            snvState.lmThreshold = *((uint16_t*)pData);
//        }
//        break;
//    case LSS_LMHYST_ID:
//        if (len == sizeof(uint8_t)) {
//            snvState.lmHysteresis = *pData;
//        }
//        break;
//    case LSS_LMOFFON_ID:
//        if (len == sizeof(uint8_t)) {
//            snvState.lmOffOn = *pData;
//        }
//        break;
    default:
        break;
    }

    snvIsDirty = true;

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
static void saveSnvState(uint8_t appId, uint8_t len, uint8_t *pData)
{

    //
    // Write out snv structure to SNV if it has changed
    //
    if (snvIsDirty == true) {
        uint32_t status = osal_snv_write(appId, len, pData);
        if (status == SUCCESS) {
            snvIsDirty = false;
        }
        else { Log_info0("SNV write failed"); }
    }

}
#endif /* LAB_4 */

#ifdef LAB_4        // LAB_4 - Non-volatile memory
/*
 * @fn      setCharacteristicsFromSnv
 *
 * @brief   Updates characteristics values after a read from SNV
 *
 * @param   pState - pointer to the in-memory SNV state
 *
 * @return  none
 */
static void setCharacteristicsFromSnv(snv_config_t *pState) {

    LedStringService_SetParameter(LSS_OFFON_ID, member_size(snv_config_t, offOn), &snvState.offOn);
    // Translate SK6812 GRB colour to Bluetooth RGB colour
    rgb_char_t rgbColour = { .red = snvState.colour.red, .green = snvState.colour.green, .blue = snvState.colour.blue };
    LedStringService_SetParameter(LSS_RGB_ID, member_size(snv_config_t, colour), &rgbColour);
    // How to handle these??
//    LedStringService_SetParameter(LSS_LMTHRESH_ID, member_size(snv_config_t, lmThreshold), &snvState.lmThreshold);
//    LedStringService_SetParameter(LSS_LMHYST_ID, member_size(snv_config_t, lmHysteresis), &snvState.lmHysteresis);
//    LedStringService_SetParameter(LSS_LMOFFON_ID, member_size(snv_config_t, lmOffOn), &snvState.lmOffOn);
    // Set other characteristics as and when implemented

}
#endif /* LAB_4 */

#ifdef LAB_5        // LAB_5 - Light Monitor implementation
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
        if ( snvState.offOn && snvState.lmOffOn) {
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
            bulkUpdateLeds((LED_STRING_0_M | LED_STRING_1_M), &ledsOff);
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
#ifdef LAB_5        // LAB_5 - Light Monitor Implementation
static void startProgram(uint8_t program) {

    switch (program) {
    case RGB_SLIDER: {
       bulkUpdateLeds((LED_STRING_0_M | LED_STRING_1_M), snvState.offOn == ON ? &snvState.colour : &ledsOff);
       writeLeds(hDmaCompleteSema, LSS_DEFAULT_PEND_TIMEOUT_MS);
       break;
    }
#ifdef LAB_6        // LAB_6 - Random Fader Implementation
    case RAND_FADE: {
        // If snvState.offOn == ON, get initial seed from TRNG, start timer
        // If snvState == OFF, switch off LEDs
        if (snvState.offOn == ON)
        {
            fadeControl.iterationCount = 0;
            uint32_t seed = getTrngRandNumber();
            // Set up initial colours
            led_t filler = { .red = (seed & TRNG_RED_MASK), .green = (seed & TRNG_GREEN_MASK) >> 8, .blue = (seed & TRNG_BLUE_MASK) >> 16 };
            bulkUpdateLeds((LED_STRING_0_M | LED_STRING_1_M), &filler);
            Clock_start(Clock_handle(&fadeControl.clock.clockDef));
        }
        else
        {
            bulkUpdateLeds((LED_STRING_0_M | LED_STRING_1_M), &ledsOff);
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
#ifdef LAB_5        // LAB_5 - Light Monitor Implementation
static void stopProgram(uint8_t program) {

    switch (program) {
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

    if ((HWREG(TRNG_BASE + TRNG_O_IRQFLAGSTAT) & TRNG_NUMBER_READY) != 0) {
        // Get the low 32-bits of the random number
        lowRand = HWREG(TRNG_BASE + TRNG_O_OUT0);
        // Kick off generation of a new number
        HWREG(TRNG_BASE + TRNG_O_IRQFLAGCLR) = 0x1;
    }
    else {
        lowRand = ++filler;
    }

    return lowRand;

}
#endif /* LAB_6 */

/*********************************************************************
*********************************************************************/
