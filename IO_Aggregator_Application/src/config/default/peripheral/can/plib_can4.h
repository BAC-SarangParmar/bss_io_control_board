/*******************************************************************************
  CAN Peripheral Library Interface Header File

  Company:
    Microchip Technology Inc.

  File Name:
    plib_can4.h

  Summary:
    CAN PLIB interface declarations.

  Description:
    The CAN plib provides a simple interface to manage the CAN modules on
    Microchip microcontrollers. This file defines the interface declarations
    for the CAN plib.

  Remarks:
    None.

*******************************************************************************/
//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef PLIB_CAN4_H
#define PLIB_CAN4_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

/*
 * This section lists the other files that are included in this file.
 */
#include <stdbool.h>
#include <string.h>

#include "device.h"
#include "plib_can_common.h"

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
#define CAN4_CLOCK_FREQUENCY    150000000U

/* CAN4 Message RAM Configuration Size */
#define CAN4_RX_FIFO0_ELEMENT_SIZE       16U
#define CAN4_RX_FIFO0_SIZE               16U
#define CAN4_RX_FIFO1_ELEMENT_SIZE       16U
#define CAN4_RX_FIFO1_SIZE               16U
#define CAN4_TX_FIFO_BUFFER_ELEMENT_SIZE 16U
#define CAN4_TX_FIFO_BUFFER_SIZE         16U
#define CAN4_TX_EVENT_FIFO_SIZE          8U
#define CAN4_EXT_MSG_ID_FILTER_SIZE      8U

/* CAN4_MESSAGE_RAM_CONFIG_SIZE to be used by application or driver
   for allocating buffer from non-cached contiguous memory */
#define CAN4_MESSAGE_RAM_CONFIG_SIZE     64U

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void CAN4_Initialize(void);
bool CAN4_MessageTransmitFifo(uint8_t numberOfMessage, CAN_TX_BUFFER *txBuffer);
uint8_t CAN4_TxFifoFreeLevelGet(void);
bool CAN4_TxBufferIsBusy(uint8_t bufferNumber);
bool CAN4_TxEventFifoRead(uint8_t numberOfTxEvent, CAN_TX_EVENT_FIFO *txEventFifo);
uint8_t CAN4_TxEventFifoFillLevelGet(void);
bool CAN4_MessageReceiveFifo(CAN_RX_FIFO_NUM rxFifoNum, uint8_t numberOfMessage, CAN_RX_BUFFER *rxBuffer);
uint8_t CAN4_RxFifoFillLevelGet(CAN_RX_FIFO_NUM rxFifoNum);
CAN_ERROR CAN4_ErrorGet(void);
void CAN4_ErrorCountGet(uint8_t *txErrorCount, uint8_t *rxErrorCount);
bool CAN4_InterruptGet(CAN_INTERRUPT_MASK interruptMask);
void CAN4_InterruptClear(CAN_INTERRUPT_MASK interruptMask);
void CAN4_MessageRAMConfigSet(uint8_t *msgRAMConfigBaseAddress);
bool CAN4_ExtendedFilterElementSet(uint8_t filterNumber, can_xidfe_registers_t *extMsgIDFilterElement);
bool CAN4_ExtendedFilterElementGet(uint8_t filterNumber, can_xidfe_registers_t *extMsgIDFilterElement);
void CAN4_SleepModeEnter(void);
void CAN4_SleepModeExit(void);
bool CAN4_BitTimingCalculationGet(CAN_BIT_TIMING_SETUP *setup, CAN_BIT_TIMING *bitTiming);
bool CAN4_BitTimingSet(CAN_BIT_TIMING *bitTiming);

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
// DOM-IGNORE-END

#endif // PLIB_CAN4_H

/*******************************************************************************
 End of File
*/
