/*******************************************************************************
  ADC Peripheral Library Interface Header File

  Company
    Microchip Technology Inc.

  File Name
    plib_adc.h

  Summary
    ADC peripheral library interface.

  Description
    This file defines the interface to the ADC peripheral library.  This
    library provides access to and control of the associated peripheral
    instance.

*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2021 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef PLIB_ADC_H    // Guards against multiple inclusion
#define PLIB_ADC_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

/*  This section lists the other files that are included in this file.
*/

#include "plib_adc_common.h"
#include "pic32cz4010ca80208.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************
/*  The following data type definitions are used by the functions in this
    interface and should be considered part it.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
/* The following functions make up the methods (set of possible operations) of
   this interface.
*/

void ADC_Initialize (void);

/* Enable ADC module */
static inline void ADC_Enable( void )
{
    ADC_REGS->ADC_CTRLA |= ADC_CTRLA_ENABLE_Msk;
    while((ADC_REGS->ADC_SYNCBUSY) != 0U)
    {
        /* Wait for Synchronization */
    }
}


/* Disable ADC module */
static inline void ADC_Disable( void )
{
    ADC_REGS->ADC_CTRLA &= ~ADC_CTRLA_ENABLE_Msk;
    while((ADC_REGS->ADC_SYNCBUSY) != 0U)
    {
        /* Wait for Synchronization */
    }
}

void ADC_CompareEnable(ADC_CORE_NUM core, ADC_CHANNEL_NUM channel);
void ADC_CompareDisable(ADC_CORE_NUM core, ADC_CHANNEL_NUM channel);
void ADC_CompareWinThresholdSet(ADC_CORE_NUM core, uint16_t low_threshold, uint16_t high_threshold);
void ADC_CompareWinModeSet(ADC_CORE_NUM core, ADC_CMPCTRL mode);

void ADC_CoreInterruptsEnable(ADC_CORE_NUM core, ADC_CORE_INT interruptMask);
void ADC_CoreInterruptsDisable(ADC_CORE_NUM core, ADC_CORE_INT interruptMask);
ADC_CORE_INT ADC_CoreInterruptsStatusGet(ADC_CORE_NUM core);
void ADC_CoreInterruptsStatusClear(ADC_CORE_NUM core, ADC_CORE_INT interruptMask);

void ADC_SoftwareControlledConversionEnable(ADC_CORE_NUM core, ADC_CHANNEL_NUM channel);

bool ADC_ChannelResultIsReady(ADC_CORE_NUM core, ADC_CHANNEL_NUM channel);
bool ADC_EOSStatusGet(ADC_CORE_NUM core);
uint32_t ADC_ResultGet( ADC_CORE_NUM core, ADC_CHANNEL_NUM channel);


ADC_GLOBAL_INT ADC_GlobalInterruptsStatusGet(void);

void ADC_GlobalEdgeConversionStart(void);
void ADC_GlobalLevelConversionStart(void);
void ADC_GlobalLevelConversionStop(void);

void ADC_SyncTriggerEnable(void);
void ADC_SyncTriggerDisable(void);
void ADC_SyncTriggerCounterSet(uint16_t counterVal);


void ADC_ChannelSamplingStart(void);
void ADC_ChannelSamplingStop(void);
void ADC_ChannelConversionStart(void);





void ADC_CORE0CallbackRegister(ADC_CORE_CALLBACK callback, uintptr_t context);
void ADC_CORE1CallbackRegister(ADC_CORE_CALLBACK callback, uintptr_t context);
void ADC_CORE2CallbackRegister(ADC_CORE_CALLBACK callback, uintptr_t context);
void ADC_CORE3CallbackRegister(ADC_CORE_CALLBACK callback, uintptr_t context);

// *****************************************************************************

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

}
#endif
// DOM-IGNORE-END

#endif //PLIB_ADC_H

/**
 End of File
*/
