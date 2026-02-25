/*******************************************************************************
  ADC Peripheral Library Interface Source File

  Company
    Microchip Technology Inc.

  File Name
    plib_adc.c

  Summary
    ADC peripheral library source.

  Description
    This file implements the ADC peripheral library.

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
#include "device.h"
#include "plib_adc.h"
#include "interrupts.h"

// *****************************************************************************
// *****************************************************************************
// Section: ADC Implementation
// *****************************************************************************
// *****************************************************************************

/* Load ADC calibration constant */
#define ADC_CALIB_FCCFG65           *((uint32_t*)0xa007184)


static volatile ADC_CORE_CALLBACK_OBJECT ADC_CoreCallbackObj[4];

void ADC_Initialize(void)
{
    /* Copy calibration value for all the enabled ADC cores */
    ADC_REGS->CONFIG[0].ADC_CALCTRL = ADC_CALIB_FCCFG65;
    ADC_REGS->CONFIG[1].ADC_CALCTRL = ADC_CALIB_FCCFG65;
    ADC_REGS->CONFIG[2].ADC_CALCTRL = ADC_CALIB_FCCFG65;
    ADC_REGS->CONFIG[3].ADC_CALCTRL = ADC_CALIB_FCCFG65;

    /*
    ONDEMAND = false
    RUNSTDBY = false
    ANAEN = 1
    */
    ADC_REGS->ADC_CTRLA = 0x4;

    /*
    VREFSEL = AVDD_AVSS
    WKUPEXP = 4
    CTLCKDIV = 0
    */
    ADC_REGS->ADC_CTRLD = 0x4000000;



    /* Enable ADC Core 0 interrupts
       EOSRDY = true
       CHNERRC = true
       FLTRDY = false
       CMPHIT = false
    */
    ADC_REGS->INT[0].ADC_INTENSET = 0xB70C00;

    /* Configure ADC Core 0 Control Register
       ADCDIV = 2
       STRGLVL = Edge
       STRGSRC = EVENT_USER0
       EIRQOVR = false
       EIS = 0
       SELRES = 12_BITS
       SAMC = 4
    */
    ADC_REGS->CONFIG[0].ADC_CORCTRL = 0x2050C04;




    /* Configure ADC Core 0 Channel Configuration Register 4 */
    ADC_REGS->CONFIG[0].ADC_CHNCFG4 = 0x10110111;


    /* Configure ADC Core 0 Event Control Register
       CMPEO = false
       RESRDYEO = false
       STARTEI = true
    */
    ADC_REGS->CONFIG[0].ADC_EVCTRL = 0x1;



    /* Enable ADC Core 1 interrupts
       EOSRDY = true
       CHNERRC = true
       FLTRDY = false
       CMPHIT = false
    */
    ADC_REGS->INT[1].ADC_INTENSET = 0x350C00;

    /* Configure ADC Core 1 Control Register
       ADCDIV = 2
       STRGLVL = Edge
       STRGSRC = EVENT_USER0
       EIRQOVR = false
       EIS = 0
       SELRES = 12_BITS
       SAMC = 4
    */
    ADC_REGS->CONFIG[1].ADC_CORCTRL = 0x2050C04;




    /* Configure ADC Core 1 Channel Configuration Register 4 */
    ADC_REGS->CONFIG[1].ADC_CHNCFG4 = 0x110101;


    /* Configure ADC Core 1 Event Control Register
       CMPEO = false
       RESRDYEO = false
       STARTEI = true
    */
    ADC_REGS->CONFIG[1].ADC_EVCTRL = 0x1;



    /* Enable ADC Core 2 interrupts
       EOSRDY = true
       CHNERRC = true
       FLTRDY = false
       CMPHIT = false
    */
    ADC_REGS->INT[2].ADC_INTENSET = 0x3C0C00;

    /* Configure ADC Core 2 Control Register
       ADCDIV = 2
       STRGLVL = Edge
       STRGSRC = EVENT_USER0
       EIRQOVR = false
       EIS = 0
       SELRES = 12_BITS
       SAMC = 4
    */
    ADC_REGS->CONFIG[2].ADC_CORCTRL = 0x2050C04;




    /* Configure ADC Core 2 Channel Configuration Register 4 */
    ADC_REGS->CONFIG[2].ADC_CHNCFG4 = 0x111100;


    /* Configure ADC Core 2 Event Control Register
       CMPEO = false
       RESRDYEO = false
       STARTEI = true
    */
    ADC_REGS->CONFIG[2].ADC_EVCTRL = 0x1;



    /* Enable ADC Core 3 interrupts
       EOSRDY = true
       CHNERRC = true
       FLTRDY = false
       CMPHIT = false
    */
    ADC_REGS->INT[3].ADC_INTENSET = 0x3F0C00;

    /* Configure ADC Core 3 Control Register
       ADCDIV = 2
       STRGLVL = Edge
       STRGSRC = EVENT_USER0
       EIRQOVR = false
       EIS = 0
       SELRES = 12_BITS
       SAMC = 4
    */
    ADC_REGS->CONFIG[3].ADC_CORCTRL = 0x2050C04;




    /* Configure ADC Core 3 Channel Configuration Register 4 */
    ADC_REGS->CONFIG[3].ADC_CHNCFG4 = 0x111111;


    /* Configure ADC Core 3 Event Control Register
       CMPEO = false
       RESRDYEO = false
       STARTEI = true
    */
    ADC_REGS->CONFIG[3].ADC_EVCTRL = 0x1;





    /* Analog and bias circuitry enable for the ADC SAR Core n (ANLEN) */
    ADC_REGS->ADC_CTRLD |= 0xF00000U;

    /* Enable the ADC Core n modules digital interface (CHNEN) */
    ADC_REGS->ADC_CTRLD |= 0xf0000U;

    /*Enable ADC module */
    ADC_REGS->ADC_CTRLA |= ADC_CTRLA_ENABLE_Msk;

    while((ADC_REGS->ADC_SYNCBUSY) != 0U)
    {
        /* Wait for Synchronization */
    }

    /* Wait for WKUPEXP delay to expire after which ADC SAR Core n is Ready (CRDY) */
    while ((ADC_REGS->ADC_CTLINTFLAG & 0xFU) == 0U)
    {

    }

    /* Wait for voltage reference to be stable */
    while ((ADC_REGS->ADC_CTLINTFLAG & ADC_CTLINTFLAG_VREFRDY_Msk) == 0U)
    {

    }

}

/* Enable channel compare mode */
void ADC_CompareEnable(ADC_CORE_NUM core, ADC_CHANNEL_NUM channel)
{
    ADC_Disable();

    ADC_REGS->CONFIG[core].ADC_CHNCFG1 |= (1UL << (uint32_t)channel);

    ADC_REGS->ADC_CMPCTRL[core] |= ADC_CMPCTRL_CMPEN_Msk;

    ADC_Enable();
}


/* Disable channel compare mode */
void ADC_CompareDisable(ADC_CORE_NUM core, ADC_CHANNEL_NUM channel)
{
    ADC_Disable();

    ADC_REGS->CONFIG[core].ADC_CHNCFG1 &= ~(1UL << (uint32_t)channel);

    ADC_Enable();
}

/* Configure window comparison threshold values */
void ADC_CompareWinThresholdSet(ADC_CORE_NUM core, uint16_t low_threshold, uint16_t high_threshold)
{
    ADC_Disable();

    ADC_REGS->ADC_CMPCTRL[core] = (ADC_REGS->ADC_CMPCTRL[core] & ~(ADC_CMPCTRL_ADCMPHI_Msk | ADC_CMPCTRL_ADCMPLO_Msk)) | (((uint32_t)low_threshold << ADC_CMPCTRL_ADCMPLO_Pos) | ((uint32_t)high_threshold << ADC_CMPCTRL_ADCMPHI_Pos));

    ADC_Enable();
}

/* Configure window comparison event mode */
void ADC_CompareWinModeSet(ADC_CORE_NUM core, ADC_CMPCTRL mode)
{
    ADC_Disable();

    ADC_REGS->ADC_CMPCTRL[core] = (ADC_REGS->ADC_CMPCTRL[core] & ~(ADC_CMPCTRL_IELOLO_Msk | ADC_CMPCTRL_IELOHI_Msk | ADC_CMPCTRL_IEBTWN_Msk | ADC_CMPCTRL_IEHILO_Msk | ADC_CMPCTRL_IEHIHI_Msk)) | mode;

    ADC_Enable();
}

void ADC_CoreInterruptsEnable(ADC_CORE_NUM core, ADC_CORE_INT interruptMask)
{
    ADC_REGS->INT[core].ADC_INTENSET = interruptMask;
}

void ADC_CoreInterruptsDisable(ADC_CORE_NUM core, ADC_CORE_INT interruptMask)
{
    ADC_REGS->INT[core].ADC_INTENCLR = interruptMask;
}

ADC_CORE_INT ADC_CoreInterruptsStatusGet(ADC_CORE_NUM core)
{
    return ADC_REGS->INT[core].ADC_INTFLAG;
}

void ADC_CoreInterruptsStatusClear(ADC_CORE_NUM core, ADC_CORE_INT interruptMask)
{
    ADC_REGS->INT[core].ADC_INTFLAG = interruptMask;
}

void ADC_SoftwareControlledConversionEnable(ADC_CORE_NUM core, ADC_CHANNEL_NUM channel)
{
    ADC_REGS->ADC_CTRLB = (ADC_REGS->ADC_CTRLB & ~(ADC_CTRLB_ADCORSEL_Msk | ADC_CTRLB_ADCHSEL_Msk)) | (((uint32_t)core << ADC_CTRLB_ADCORSEL_Pos) | ((uint32_t)channel << ADC_CTRLB_ADCHSEL_Pos));

    while((ADC_REGS->ADC_SYNCBUSY) != 0U)
    {
        /* Wait for Synchronization */
    }

    ADC_REGS->ADC_CTRLB |= ADC_CTRLB_SWCNVEN_Msk;

    while((ADC_REGS->ADC_SYNCBUSY) != 0U)
    {
        /* Wait for Synchronization */
    }
}

bool ADC_ChannelResultIsReady(ADC_CORE_NUM core, ADC_CHANNEL_NUM channel)
{
    return (bool)((ADC_REGS->INT[core].ADC_INTFLAG & (1UL << (ADC_INTFLAG_CHRDY_Pos + (uint32_t)channel))) != 0U);
}

bool ADC_EOSStatusGet(ADC_CORE_NUM core)
{
    return (bool)((ADC_REGS->INT[core].ADC_INTFLAG & ADC_INTFLAG_EOSRDY_Msk) == ADC_INTFLAG_EOSRDY_Msk);
}

/* Read the conversion result for the given core, channel */
uint32_t ADC_ResultGet( ADC_CORE_NUM core, ADC_CHANNEL_NUM channel)
{
    ADC_REGS->ADC_CORCHDATAID = (ADC_REGS->ADC_CORCHDATAID & ~(ADC_CORCHDATAID_CORDYID_Msk | ADC_CORCHDATAID_CHRDYID_Msk)) | (((uint32_t)core << ADC_CORCHDATAID_CORDYID_Pos) | (uint32_t)channel);

    return ADC_REGS->ADC_CHRDYDAT;
}

ADC_GLOBAL_INT ADC_GlobalInterruptsStatusGet(void)
{
    return ADC_REGS->ADC_CTLINTFLAG;
}

void ADC_GlobalEdgeConversionStart(void)
{
    ADC_REGS->ADC_CTRLB |= ADC_CTRLB_GSWTRG_Msk;

    while((ADC_REGS->ADC_SYNCBUSY) != 0U)
    {
        /* Wait for Synchronization */
    }
}

void ADC_GlobalLevelConversionStart(void)
{
    ADC_REGS->ADC_CTRLB |= ADC_CTRLB_LSWTRG_Msk;

    while((ADC_REGS->ADC_SYNCBUSY) != 0U)
    {
        /* Wait for Synchronization */
    }
}

void ADC_GlobalLevelConversionStop(void)
{
    ADC_REGS->ADC_CTRLB &= ~ADC_CTRLB_LSWTRG_Msk;

    while((ADC_REGS->ADC_SYNCBUSY) != 0U)
    {
        /* Wait for Synchronization */
    }
}

void ADC_SyncTriggerEnable(void)
{
    ADC_REGS->ADC_CTRLB |= (1UL<<11U);

    while((ADC_REGS->ADC_SYNCBUSY) != 0U)
    {
        /* Wait for Synchronization */
    }
}

void ADC_SyncTriggerDisable(void)
{
    ADC_REGS->ADC_CTRLB &= ~(1UL<<11U);

    while((ADC_REGS->ADC_SYNCBUSY) != 0U)
    {
        /* Wait for Synchronization */
    }
}


void ADC_SyncTriggerCounterSet(uint16_t counterVal)
{
    ADC_Disable();

    ADC_REGS->ADC_CTRLC = (ADC_REGS->ADC_CTRLC & ~ADC_CTRLC_CNT_Msk) | (counterVal);

    while((ADC_REGS->ADC_SYNCBUSY) != 0U)
    {
        /* Wait for Synchronization */
    }

    ADC_Enable();
}

void ADC_ChannelSamplingStart(void)
{
    ADC_REGS->ADC_CTRLB |= ADC_CTRLB_SAMP_Msk;

    while((ADC_REGS->ADC_SYNCBUSY) != 0U)
    {
        /* Wait for Synchronization */
    }
}

void ADC_ChannelSamplingStop(void)
{
    ADC_REGS->ADC_CTRLB &= ~ADC_CTRLB_SAMP_Msk;

    while((ADC_REGS->ADC_SYNCBUSY) != 0U)
    {
        /* Wait for Synchronization */
    }
}

void ADC_ChannelConversionStart(void)
{
    ADC_REGS->ADC_CTRLB |= ADC_CTRLB_RQCNVRT_Msk;

    while((ADC_REGS->ADC_SYNCBUSY) != 0U)
    {
        /* Wait for Synchronization */
    }
}




void ADC_CORE0CallbackRegister(ADC_CORE_CALLBACK callback, uintptr_t context)
{
   ADC_CoreCallbackObj[0].callback = callback;
   ADC_CoreCallbackObj[0].context = context;
}
void ADC_CORE1CallbackRegister(ADC_CORE_CALLBACK callback, uintptr_t context)
{
   ADC_CoreCallbackObj[1].callback = callback;
   ADC_CoreCallbackObj[1].context = context;
}
void ADC_CORE2CallbackRegister(ADC_CORE_CALLBACK callback, uintptr_t context)
{
   ADC_CoreCallbackObj[2].callback = callback;
   ADC_CoreCallbackObj[2].context = context;
}
void ADC_CORE3CallbackRegister(ADC_CORE_CALLBACK callback, uintptr_t context)
{
   ADC_CoreCallbackObj[3].callback = callback;
   ADC_CoreCallbackObj[3].context = context;
}


void __attribute__((used)) ADC_CORE1_InterruptHandler( void )
{
   uint32_t status;

   status = ADC_REGS->INT[0].ADC_INTFLAG;

   /* Clear interrupt flag */
   ADC_REGS->INT[0].ADC_INTFLAG = status;

   if (ADC_CoreCallbackObj[0].callback != NULL)
   {
       uintptr_t context = ADC_CoreCallbackObj[0].context;
       ADC_CoreCallbackObj[0].callback(status, context);
   }
}
void __attribute__((used)) ADC_CORE2_InterruptHandler( void )
{
   uint32_t status;

   status = ADC_REGS->INT[1].ADC_INTFLAG;

   /* Clear interrupt flag */
   ADC_REGS->INT[1].ADC_INTFLAG = status;

   if (ADC_CoreCallbackObj[1].callback != NULL)
   {
       uintptr_t context = ADC_CoreCallbackObj[1].context;
       ADC_CoreCallbackObj[1].callback(status, context);
   }
}
void __attribute__((used)) ADC_CORE3_InterruptHandler( void )
{
   uint32_t status;

   status = ADC_REGS->INT[2].ADC_INTFLAG;

   /* Clear interrupt flag */
   ADC_REGS->INT[2].ADC_INTFLAG = status;

   if (ADC_CoreCallbackObj[2].callback != NULL)
   {
       uintptr_t context = ADC_CoreCallbackObj[2].context;
       ADC_CoreCallbackObj[2].callback(status, context);
   }
}
void __attribute__((used)) ADC_CORE4_InterruptHandler( void )
{
   uint32_t status;

   status = ADC_REGS->INT[3].ADC_INTFLAG;

   /* Clear interrupt flag */
   ADC_REGS->INT[3].ADC_INTFLAG = status;

   if (ADC_CoreCallbackObj[3].callback != NULL)
   {
       uintptr_t context = ADC_CoreCallbackObj[3].context;
       ADC_CoreCallbackObj[3].callback(status, context);
   }
}


