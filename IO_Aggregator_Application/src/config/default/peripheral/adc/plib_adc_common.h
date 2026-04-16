/*******************************************************************************
  ADCHS Peripheral Library Interface Header File

  Company
    Microchip Technology Inc.

  File Name
    plib_adchs_common.h

  Summary
    Commonly needed stuff for the ADCHS peripheral libraries interfaces.

  Description
    This file defines several items commonly needed by the interfaces to
    the ADCHS peripheral libraries.

*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
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

#ifndef PLIB_ADCHS_COMMON_H    // Guards against multiple inclusion
#define PLIB_ADCHS_COMMON_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

/*  This section lists the other files that are included in this file.
*/

#include <stddef.h>
#include <stdbool.h>


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

// *****************************************************************************

typedef uint32_t ADC_CMPCTRL;
typedef uint32_t ADC_CORE_INT;
typedef uint32_t ADC_GLOBAL_INT;

#define ADC_CORE_INT_CHRDY_0                (1 << (ADC_INTFLAG_CHRDY_Pos + 0))
#define ADC_CORE_INT_CHRDY_1                (1 << (ADC_INTFLAG_CHRDY_Pos + 1))
#define ADC_CORE_INT_CHRDY_2                (1 << (ADC_INTFLAG_CHRDY_Pos + 2))
#define ADC_CORE_INT_CHRDY_3                (1 << (ADC_INTFLAG_CHRDY_Pos + 3))
#define ADC_CORE_INT_CHRDY_4                (1 << (ADC_INTFLAG_CHRDY_Pos + 4))
#define ADC_CORE_INT_CHRDY_5                (1 << (ADC_INTFLAG_CHRDY_Pos + 5))
#define ADC_CORE_INT_CHRDY_6                (1 << (ADC_INTFLAG_CHRDY_Pos + 6))
#define ADC_CORE_INT_CHRDY_7                (1 << (ADC_INTFLAG_CHRDY_Pos + 7))
#define ADC_CORE_INT_CHRDY_8                (1 << (ADC_INTFLAG_CHRDY_Pos + 8))
#define ADC_CORE_INT_CHRDY_9                (1 << (ADC_INTFLAG_CHRDY_Pos + 9))
#define ADC_CORE_INT_CHRDY_10                (1 << (ADC_INTFLAG_CHRDY_Pos + 10))
#define ADC_CORE_INT_CHRDY_11                (1 << (ADC_INTFLAG_CHRDY_Pos + 11))
#define ADC_CORE_INT_CHRDY_12                (1 << (ADC_INTFLAG_CHRDY_Pos + 12))
#define ADC_CORE_INT_CHRDY_13                (1 << (ADC_INTFLAG_CHRDY_Pos + 13))
#define ADC_CORE_INT_CHRDY_14                (1 << (ADC_INTFLAG_CHRDY_Pos + 14))
#define ADC_CORE_INT_CHRDY_15                (1 << (ADC_INTFLAG_CHRDY_Pos + 15))

#define ADC_CORE_INT_EOSRDY                 (ADC_INTFLAG_EOSRDY_Msk)
#define ADC_CORE_INT_CHNERRC                (ADC_INTFLAG_CHNERRC_Msk)
#define ADC_CORE_INT_FLTRDY                 (ADC_INTFLAG_FLTRDY_Msk)
#define ADC_CORE_INT_CHRDYC                 (ADC_INTFLAG_CHRDYC_Msk)
#define ADC_CORE_INT_SOVFL                  (ADC_INTFLAG_SOVFL_Msk)
#define ADC_CORE_INT_CMPHIT                 (ADC_INTFLAG_CMPHIT_Msk)

#define ADC_GLOBAL_INT_PFFHFUL              (ADC_CTLINTENSET_PFFHFUL_Msk)
#define ADC_GLOBAL_INT_PFFRDY               (ADC_CTLINTENSET_PFFRDY_Msk)
#define ADC_GLOBAL_INT_PFFOVF               (ADC_CTLINTENSET_PFFOVF_Msk)
#define ADC_GLOBAL_INT_PFFUNF               (ADC_CTLINTENSET_PFFUNF_Msk)
#define ADC_GLOBAL_INT_VREFRDY              (ADC_CTLINTENSET_VREFRDY_Msk)
#define ADC_GLOBAL_INT_VREFUPD              (ADC_CTLINTENSET_VREFUPD_Msk)
#define ADC_GLOBAL_INT_CRRDY0               (1 << (ADC_CTLINTENSET_CRRDY_Pos+0))
#define ADC_GLOBAL_INT_CRRDY1               (1 << (ADC_CTLINTENSET_CRRDY_Pos+1))
#define ADC_GLOBAL_INT_CRRDY2               (1 << (ADC_CTLINTENSET_CRRDY_Pos+2))
#define ADC_GLOBAL_INT_CRRDY3               (1 << (ADC_CTLINTENSET_CRRDY_Pos+3))

#define ADC_CMP_MODE_IEHIHI                 (ADC_CMPCTRL_IEHIHI_Msk)
#define ADC_CMP_MODE_IEHILO                 (ADC_CMPCTRL_IEHILO_Msk)
#define ADC_CMP_MODE_IELOHI                 (ADC_CMPCTRL_IELOHI_Msk)
#define ADC_CMP_MODE_IELOLO                 (ADC_CMPCTRL_IELOLO_Msk)
#define ADC_CMP_MODE_IEBTWN                 (ADC_CMPCTRL_IEBTWN_Msk)

/* Helper macros to extract channel id and core id from the ADC result read from the FIFO */
#define ADC_FIFO_CHNID_GET(fifo_data)           ((fifo_data & ADC_PFFDATA_PFFCHNID_Msk) >> ADC_PFFDATA_PFFCHNID_Pos)
#define ADC_FIFO_CORID_GET(fifo_data)           ((fifo_data & ADC_PFFDATA_PFFCORID_Msk) >> ADC_PFFDATA_PFFCORID_Pos)
#define ADC_FIFO_CNT_GET(fifo_data)             ((fifo_data & ADC_PFFDATA_PFFCNT_Msk) >> ADC_PFFDATA_PFFCNT_Pos)
#define ADC_FIFO_DATA_GET(fifo_data)            ((fifo_data & ADC_PFFDATA_PFFDATA_Msk) >> ADC_PFFDATA_PFFDATA_Pos)



typedef enum
{
    ADC_CORE_NUM0 = 0U,
    ADC_CORE_NUM1 = 1U,
    ADC_CORE_NUM2 = 2U,
    ADC_CORE_NUM3 = 3U,
}ADC_CORE_NUM;


typedef enum
{
    ADC_CH0 = 0U,
    ADC_CH1 = 1U,
    ADC_CH2 = 2U,
    ADC_CH3 = 3U,
    ADC_CH4 = 4U,
    ADC_CH5 = 5U,
    ADC_CH6 = 6U,
    ADC_CH7 = 7U,
    ADC_CH8 = 8U,
    ADC_CH9 = 9U,
    ADC_CH10 = 10U,
    ADC_CH11 = 11U,
    ADC_CH12 = 12U,
    ADC_CH13 = 13U,
    ADC_CH14 = 14U,
    ADC_CH15 = 15U,
}ADC_CHANNEL_NUM;



// *****************************************************************************

typedef void (*ADC_GLOBAL_CALLBACK)(ADC_GLOBAL_INT status, uintptr_t context);
typedef void (*ADC_CORE_CALLBACK)(ADC_CORE_INT status, uintptr_t context);

// *****************************************************************************

typedef struct
{
    ADC_GLOBAL_CALLBACK callback;
    uintptr_t context;
}ADC_GLOBAL_CALLBACK_OBJECT;

typedef struct
{
    ADC_CORE_CALLBACK callback;
    uintptr_t context;
}ADC_CORE_CALLBACK_OBJECT;

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif
// DOM-IGNORE-END

#endif //PLIB_ADC_COMMMON_H

/**
 End of File
*/
