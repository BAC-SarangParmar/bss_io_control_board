/*******************************************************************************
  PORT PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_port.h

  Summary:
    PORT PLIB Header File

  Description:
    This file provides an interface to control and interact with PORT-I/O
    Pin controller module.

*******************************************************************************/

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

#ifndef PLIB_PORT_H
#define PLIB_PORT_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "device.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility
    extern "C" {
#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data types and constants
// *****************************************************************************
// *****************************************************************************

/*** Macros for Eth_RxD1 pin ***/
#define Eth_RxD1_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 10U)) & 0x01U)
#define Eth_RxD1_PIN                  PORT_PIN_PD10

/*** Macros for Digital_Input_41 pin ***/
#define Digital_Input_41_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 8U))
#define Digital_Input_41_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 8U))
#define Digital_Input_41_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 8U))
#define Digital_Input_41_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 8U))
#define Digital_Input_41_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 8U))
#define Digital_Input_41_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 8U)) & 0x01U)
#define Digital_Input_41_PIN                  PORT_PIN_PD08

/*** Macros for Digital_Input_42 pin ***/
#define Digital_Input_42_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 6U))
#define Digital_Input_42_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 6U))
#define Digital_Input_42_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 6U))
#define Digital_Input_42_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 6U))
#define Digital_Input_42_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 6U))
#define Digital_Input_42_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 6U)) & 0x01U)
#define Digital_Input_42_PIN                  PORT_PIN_PD06

/*** Macros for Digital_Input_16 pin ***/
#define Digital_Input_16_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 19U))
#define Digital_Input_16_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 19U))
#define Digital_Input_16_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 19U))
#define Digital_Input_16_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 19U))
#define Digital_Input_16_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 19U))
#define Digital_Input_16_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 19U)) & 0x01U)
#define Digital_Input_16_PIN                  PORT_PIN_PD19

/*** Macros for Digital_Input_14 pin ***/
#define Digital_Input_14_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 4U))
#define Digital_Input_14_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 4U))
#define Digital_Input_14_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 4U))
#define Digital_Input_14_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 4U))
#define Digital_Input_14_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 4U))
#define Digital_Input_14_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 4U)) & 0x01U)
#define Digital_Input_14_PIN                  PORT_PIN_PD04

/*** Macros for Digital_Input_12 pin ***/
#define Digital_Input_12_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 16U))
#define Digital_Input_12_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 16U))
#define Digital_Input_12_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 16U))
#define Digital_Input_12_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 16U))
#define Digital_Input_12_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 16U))
#define Digital_Input_12_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 16U)) & 0x01U)
#define Digital_Input_12_PIN                  PORT_PIN_PD16

/*** Macros for Digital_Input_11 pin ***/
#define Digital_Input_11_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 27U))
#define Digital_Input_11_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 27U))
#define Digital_Input_11_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 27U))
#define Digital_Input_11_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 27U))
#define Digital_Input_11_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 27U))
#define Digital_Input_11_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 27U)) & 0x01U)
#define Digital_Input_11_PIN                  PORT_PIN_PD27

/*** Macros for Digital_Input_10 pin ***/
#define Digital_Input_10_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 3U))
#define Digital_Input_10_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 3U))
#define Digital_Input_10_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 3U))
#define Digital_Input_10_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 3U))
#define Digital_Input_10_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 3U))
#define Digital_Input_10_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 3U)) & 0x01U)
#define Digital_Input_10_PIN                  PORT_PIN_PD03

/*** Macros for I2C_SDA pin ***/
#define I2C_SDA_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 14U)) & 0x01U)
#define I2C_SDA_PIN                  PORT_PIN_PD14

/*** Macros for Eth_REF_CLK pin ***/
#define Eth_REF_CLK_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 12U)) & 0x01U)
#define Eth_REF_CLK_PIN                  PORT_PIN_PD12

/*** Macros for Eth_RXD0 pin ***/
#define Eth_RXD0_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 11U)) & 0x01U)
#define Eth_RXD0_PIN                  PORT_PIN_PD11

/*** Macros for Digital_Input_17 pin ***/
#define Digital_Input_17_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 9U))
#define Digital_Input_17_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 9U))
#define Digital_Input_17_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 9U))
#define Digital_Input_17_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 9U))
#define Digital_Input_17_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 9U))
#define Digital_Input_17_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 9U)) & 0x01U)
#define Digital_Input_17_PIN                  PORT_PIN_PD09

/*** Macros for Digital_Input_43 pin ***/
#define Digital_Input_43_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 20U))
#define Digital_Input_43_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 20U))
#define Digital_Input_43_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 20U))
#define Digital_Input_43_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 20U))
#define Digital_Input_43_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 20U))
#define Digital_Input_43_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 20U)) & 0x01U)
#define Digital_Input_43_PIN                  PORT_PIN_PD20

/*** Macros for Digital_Input_44 pin ***/
#define Digital_Input_44_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 5U))
#define Digital_Input_44_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 5U))
#define Digital_Input_44_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 5U))
#define Digital_Input_44_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 5U))
#define Digital_Input_44_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 5U))
#define Digital_Input_44_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 5U)) & 0x01U)
#define Digital_Input_44_PIN                  PORT_PIN_PD05

/*** Macros for Digital_Input_15 pin ***/
#define Digital_Input_15_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 18U))
#define Digital_Input_15_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 18U))
#define Digital_Input_15_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 18U))
#define Digital_Input_15_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 18U))
#define Digital_Input_15_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 18U))
#define Digital_Input_15_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 18U)) & 0x01U)
#define Digital_Input_15_PIN                  PORT_PIN_PD18

/*** Macros for Digital_Input_13 pin ***/
#define Digital_Input_13_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 17U))
#define Digital_Input_13_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 17U))
#define Digital_Input_13_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 17U))
#define Digital_Input_13_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 17U))
#define Digital_Input_13_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 17U))
#define Digital_Input_13_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 17U)) & 0x01U)
#define Digital_Input_13_PIN                  PORT_PIN_PD17

/*** Macros for RS485_TX2 pin ***/
#define RS485_TX2_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 28U)) & 0x01U)
#define RS485_TX2_PIN                  PORT_PIN_PD28

/*** Macros for RS485_EN1 pin ***/
#define RS485_EN1_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 26U))
#define RS485_EN1_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 26U))
#define RS485_EN1_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 26U))
#define RS485_EN1_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 26U))
#define RS485_EN1_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 26U))
#define RS485_EN1_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 26U)) & 0x01U)
#define RS485_EN1_PIN                  PORT_PIN_PD26

/*** Macros for Digital_Input_9 pin ***/
#define Digital_Input_9_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 2U))
#define Digital_Input_9_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 2U))
#define Digital_Input_9_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 2U))
#define Digital_Input_9_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 2U))
#define Digital_Input_9_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 2U))
#define Digital_Input_9_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 2U)) & 0x01U)
#define Digital_Input_9_PIN                  PORT_PIN_PD02

/*** Macros for Can3_TX pin ***/
#define Can3_TX_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 13U)) & 0x01U)
#define Can3_TX_PIN                  PORT_PIN_PD13

/*** Macros for Eth_TXEN pin ***/
#define Eth_TXEN_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 2U)) & 0x01U)
#define Eth_TXEN_PIN                  PORT_PIN_PA02

/*** Macros for Eth_TXD0 pin ***/
#define Eth_TXD0_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 1U)) & 0x01U)
#define Eth_TXD0_PIN                  PORT_PIN_PA01

/*** Macros for Eth_MDIO pin ***/
#define Eth_MDIO_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 4U)) & 0x01U)
#define Eth_MDIO_PIN                  PORT_PIN_PA04

/*** Macros for Eth_MDC pin ***/
#define Eth_MDC_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 3U)) & 0x01U)
#define Eth_MDC_PIN                  PORT_PIN_PA03

/*** Macros for RS485_RX1 pin ***/
#define RS485_RX1_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 25U)) & 0x01U)
#define RS485_RX1_PIN                  PORT_PIN_PD25

/*** Macros for Digital_Input_18 pin ***/
#define Digital_Input_18_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 21U))
#define Digital_Input_18_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 21U))
#define Digital_Input_18_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 21U))
#define Digital_Input_18_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 21U))
#define Digital_Input_18_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 21U))
#define Digital_Input_18_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 21U)) & 0x01U)
#define Digital_Input_18_PIN                  PORT_PIN_PD21

/*** Macros for I2C_SCL pin ***/
#define I2C_SCL_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 15U)) & 0x01U)
#define I2C_SCL_PIN                  PORT_PIN_PD15

/*** Macros for RS485_RX2 pin ***/
#define RS485_RX2_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 29U)) & 0x01U)
#define RS485_RX2_PIN                  PORT_PIN_PD29

/*** Macros for Can5_RX pin ***/
#define Can5_RX_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 23U)) & 0x01U)
#define Can5_RX_PIN                  PORT_PIN_PD23

/*** Macros for RS485_TX1 pin ***/
#define RS485_TX1_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 24U)) & 0x01U)
#define RS485_TX1_PIN                  PORT_PIN_PD24

/*** Macros for Digital_Input_49 pin ***/
#define Digital_Input_49_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 17U))
#define Digital_Input_49_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 17U))
#define Digital_Input_49_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 17U))
#define Digital_Input_49_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 17U))
#define Digital_Input_49_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 17U))
#define Digital_Input_49_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 17U)) & 0x01U)
#define Digital_Input_49_PIN                  PORT_PIN_PC17

/*** Macros for Eth_RXER pin ***/
#define Eth_RXER_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 6U)) & 0x01U)
#define Eth_RXER_PIN                  PORT_PIN_PA06

/*** Macros for Eth_RXDV pin ***/
#define Eth_RXDV_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 5U)) & 0x01U)
#define Eth_RXDV_PIN                  PORT_PIN_PA05

/*** Macros for Eth_TXD1 pin ***/
#define Eth_TXD1_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 0U)) & 0x01U)
#define Eth_TXD1_PIN                  PORT_PIN_PA00

/*** Macros for LED_Status pin ***/
#define LED_Status_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 14U))
#define LED_Status_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 14U))
#define LED_Status_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 14U))
#define LED_Status_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 14U))
#define LED_Status_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 14U))
#define LED_Status_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 14U)) & 0x01U)
#define LED_Status_PIN                  PORT_PIN_PC14

/*** Macros for Can3_RX pin ***/
#define Can3_RX_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 29U)) & 0x01U)
#define Can3_RX_PIN                  PORT_PIN_PC29

/*** Macros for Can5_TX pin ***/
#define Can5_TX_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 21U)) & 0x01U)
#define Can5_TX_PIN                  PORT_PIN_PA21

/*** Macros for Can4_RX pin ***/
#define Can4_RX_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 30U)) & 0x01U)
#define Can4_RX_PIN                  PORT_PIN_PA30

/*** Macros for RS485_EN2 pin ***/
#define RS485_EN2_Set()               (PORT_REGS->GROUP[4].PORT_OUTSET = ((uint32_t)1U << 4U))
#define RS485_EN2_Clear()             (PORT_REGS->GROUP[4].PORT_OUTCLR = ((uint32_t)1U << 4U))
#define RS485_EN2_Toggle()            (PORT_REGS->GROUP[4].PORT_OUTTGL = ((uint32_t)1U << 4U))
#define RS485_EN2_OutputEnable()      (PORT_REGS->GROUP[4].PORT_DIRSET = ((uint32_t)1U << 4U))
#define RS485_EN2_InputEnable()       (PORT_REGS->GROUP[4].PORT_DIRCLR = ((uint32_t)1U << 4U))
#define RS485_EN2_Get()               (((PORT_REGS->GROUP[4].PORT_IN >> 4U)) & 0x01U)
#define RS485_EN2_PIN                  PORT_PIN_PE04

/*** Macros for Digital_Input_19 pin ***/
#define Digital_Input_19_Set()               (PORT_REGS->GROUP[3].PORT_OUTSET = ((uint32_t)1U << 7U))
#define Digital_Input_19_Clear()             (PORT_REGS->GROUP[3].PORT_OUTCLR = ((uint32_t)1U << 7U))
#define Digital_Input_19_Toggle()            (PORT_REGS->GROUP[3].PORT_OUTTGL = ((uint32_t)1U << 7U))
#define Digital_Input_19_OutputEnable()      (PORT_REGS->GROUP[3].PORT_DIRSET = ((uint32_t)1U << 7U))
#define Digital_Input_19_InputEnable()       (PORT_REGS->GROUP[3].PORT_DIRCLR = ((uint32_t)1U << 7U))
#define Digital_Input_19_Get()               (((PORT_REGS->GROUP[3].PORT_IN >> 7U)) & 0x01U)
#define Digital_Input_19_PIN                  PORT_PIN_PD07

/*** Macros for Relay_Output_2 pin ***/
#define Relay_Output_2_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 13U))
#define Relay_Output_2_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 13U))
#define Relay_Output_2_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 13U))
#define Relay_Output_2_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 13U))
#define Relay_Output_2_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 13U))
#define Relay_Output_2_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 13U)) & 0x01U)
#define Relay_Output_2_PIN                  PORT_PIN_PC13

/*** Macros for LED_Error pin ***/
#define LED_Error_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 12U))
#define LED_Error_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 12U))
#define LED_Error_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 12U))
#define LED_Error_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 12U))
#define LED_Error_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 12U))
#define LED_Error_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 12U)) & 0x01U)
#define LED_Error_PIN                  PORT_PIN_PC12

/*** Macros for Digital_Input_21 pin ***/
#define Digital_Input_21_Set()               (PORT_REGS->GROUP[0].PORT_OUTSET = ((uint32_t)1U << 23U))
#define Digital_Input_21_Clear()             (PORT_REGS->GROUP[0].PORT_OUTCLR = ((uint32_t)1U << 23U))
#define Digital_Input_21_Toggle()            (PORT_REGS->GROUP[0].PORT_OUTTGL = ((uint32_t)1U << 23U))
#define Digital_Input_21_OutputEnable()      (PORT_REGS->GROUP[0].PORT_DIRSET = ((uint32_t)1U << 23U))
#define Digital_Input_21_InputEnable()       (PORT_REGS->GROUP[0].PORT_DIRCLR = ((uint32_t)1U << 23U))
#define Digital_Input_21_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 23U)) & 0x01U)
#define Digital_Input_21_PIN                  PORT_PIN_PA23

/*** Macros for Can4_TX pin ***/
#define Can4_TX_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 31U)) & 0x01U)
#define Can4_TX_PIN                  PORT_PIN_PA31

/*** Macros for Digital_Input_22 pin ***/
#define Digital_Input_22_Set()               (PORT_REGS->GROUP[0].PORT_OUTSET = ((uint32_t)1U << 22U))
#define Digital_Input_22_Clear()             (PORT_REGS->GROUP[0].PORT_OUTCLR = ((uint32_t)1U << 22U))
#define Digital_Input_22_Toggle()            (PORT_REGS->GROUP[0].PORT_OUTTGL = ((uint32_t)1U << 22U))
#define Digital_Input_22_OutputEnable()      (PORT_REGS->GROUP[0].PORT_DIRSET = ((uint32_t)1U << 22U))
#define Digital_Input_22_InputEnable()       (PORT_REGS->GROUP[0].PORT_DIRCLR = ((uint32_t)1U << 22U))
#define Digital_Input_22_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 22U)) & 0x01U)
#define Digital_Input_22_PIN                  PORT_PIN_PA22

/*** Macros for Digital_Input_20 pin ***/
#define Digital_Input_20_Set()               (PORT_REGS->GROUP[4].PORT_OUTSET = ((uint32_t)1U << 5U))
#define Digital_Input_20_Clear()             (PORT_REGS->GROUP[4].PORT_OUTCLR = ((uint32_t)1U << 5U))
#define Digital_Input_20_Toggle()            (PORT_REGS->GROUP[4].PORT_OUTTGL = ((uint32_t)1U << 5U))
#define Digital_Input_20_OutputEnable()      (PORT_REGS->GROUP[4].PORT_DIRSET = ((uint32_t)1U << 5U))
#define Digital_Input_20_InputEnable()       (PORT_REGS->GROUP[4].PORT_DIRCLR = ((uint32_t)1U << 5U))
#define Digital_Input_20_Get()               (((PORT_REGS->GROUP[4].PORT_IN >> 5U)) & 0x01U)
#define Digital_Input_20_PIN                  PORT_PIN_PE05

/*** Macros for Ethernet_Reset pin ***/
#define Ethernet_Reset_Set()               (PORT_REGS->GROUP[6].PORT_OUTSET = ((uint32_t)1U << 11U))
#define Ethernet_Reset_Clear()             (PORT_REGS->GROUP[6].PORT_OUTCLR = ((uint32_t)1U << 11U))
#define Ethernet_Reset_Toggle()            (PORT_REGS->GROUP[6].PORT_OUTTGL = ((uint32_t)1U << 11U))
#define Ethernet_Reset_OutputEnable()      (PORT_REGS->GROUP[6].PORT_DIRSET = ((uint32_t)1U << 11U))
#define Ethernet_Reset_InputEnable()       (PORT_REGS->GROUP[6].PORT_DIRCLR = ((uint32_t)1U << 11U))
#define Ethernet_Reset_Get()               (((PORT_REGS->GROUP[6].PORT_IN >> 11U)) & 0x01U)
#define Ethernet_Reset_PIN                  PORT_PIN_PG11

/*** Macros for Digital_Input_48 pin ***/
#define Digital_Input_48_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 15U))
#define Digital_Input_48_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 15U))
#define Digital_Input_48_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 15U))
#define Digital_Input_48_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 15U))
#define Digital_Input_48_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 15U))
#define Digital_Input_48_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 15U)) & 0x01U)
#define Digital_Input_48_PIN                  PORT_PIN_PC15

/*** Macros for efuse2_den pin ***/
#define efuse2_den_Set()               (PORT_REGS->GROUP[6].PORT_OUTSET = ((uint32_t)1U << 10U))
#define efuse2_den_Clear()             (PORT_REGS->GROUP[6].PORT_OUTCLR = ((uint32_t)1U << 10U))
#define efuse2_den_Toggle()            (PORT_REGS->GROUP[6].PORT_OUTTGL = ((uint32_t)1U << 10U))
#define efuse2_den_OutputEnable()      (PORT_REGS->GROUP[6].PORT_DIRSET = ((uint32_t)1U << 10U))
#define efuse2_den_InputEnable()       (PORT_REGS->GROUP[6].PORT_DIRCLR = ((uint32_t)1U << 10U))
#define efuse2_den_Get()               (((PORT_REGS->GROUP[6].PORT_IN >> 10U)) & 0x01U)
#define efuse2_den_PIN                  PORT_PIN_PG10

/*** Macros for efuse2_in pin ***/
#define efuse2_in_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 27U))
#define efuse2_in_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 27U))
#define efuse2_in_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 27U))
#define efuse2_in_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 27U))
#define efuse2_in_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 27U))
#define efuse2_in_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 27U)) & 0x01U)
#define efuse2_in_PIN                  PORT_PIN_PC27

/*** Macros for SD_CMD pin ***/
#define SD_CMD_Get()               (((PORT_REGS->GROUP[6].PORT_IN >> 3U)) & 0x01U)
#define SD_CMD_PIN                  PORT_PIN_PG03

/*** Macros for SD_CD pin ***/
#define SD_CD_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 28U)) & 0x01U)
#define SD_CD_PIN                  PORT_PIN_PC28

/*** Macros for Digital_Input_47 pin ***/
#define Digital_Input_47_Set()               (PORT_REGS->GROUP[4].PORT_OUTSET = ((uint32_t)1U << 0U))
#define Digital_Input_47_Clear()             (PORT_REGS->GROUP[4].PORT_OUTCLR = ((uint32_t)1U << 0U))
#define Digital_Input_47_Toggle()            (PORT_REGS->GROUP[4].PORT_OUTTGL = ((uint32_t)1U << 0U))
#define Digital_Input_47_OutputEnable()      (PORT_REGS->GROUP[4].PORT_DIRSET = ((uint32_t)1U << 0U))
#define Digital_Input_47_InputEnable()       (PORT_REGS->GROUP[4].PORT_DIRCLR = ((uint32_t)1U << 0U))
#define Digital_Input_47_Get()               (((PORT_REGS->GROUP[4].PORT_IN >> 0U)) & 0x01U)
#define Digital_Input_47_PIN                  PORT_PIN_PE00

/*** Macros for Digital_Input_23 pin ***/
#define Digital_Input_23_Set()               (PORT_REGS->GROUP[4].PORT_OUTSET = ((uint32_t)1U << 1U))
#define Digital_Input_23_Clear()             (PORT_REGS->GROUP[4].PORT_OUTCLR = ((uint32_t)1U << 1U))
#define Digital_Input_23_Toggle()            (PORT_REGS->GROUP[4].PORT_OUTTGL = ((uint32_t)1U << 1U))
#define Digital_Input_23_OutputEnable()      (PORT_REGS->GROUP[4].PORT_DIRSET = ((uint32_t)1U << 1U))
#define Digital_Input_23_InputEnable()       (PORT_REGS->GROUP[4].PORT_DIRCLR = ((uint32_t)1U << 1U))
#define Digital_Input_23_Get()               (((PORT_REGS->GROUP[4].PORT_IN >> 1U)) & 0x01U)
#define Digital_Input_23_PIN                  PORT_PIN_PE01

/*** Macros for Digital_Input_24 pin ***/
#define Digital_Input_24_Set()               (PORT_REGS->GROUP[4].PORT_OUTSET = ((uint32_t)1U << 6U))
#define Digital_Input_24_Clear()             (PORT_REGS->GROUP[4].PORT_OUTCLR = ((uint32_t)1U << 6U))
#define Digital_Input_24_Toggle()            (PORT_REGS->GROUP[4].PORT_OUTTGL = ((uint32_t)1U << 6U))
#define Digital_Input_24_OutputEnable()      (PORT_REGS->GROUP[4].PORT_DIRSET = ((uint32_t)1U << 6U))
#define Digital_Input_24_InputEnable()       (PORT_REGS->GROUP[4].PORT_DIRCLR = ((uint32_t)1U << 6U))
#define Digital_Input_24_Get()               (((PORT_REGS->GROUP[4].PORT_IN >> 6U)) & 0x01U)
#define Digital_Input_24_PIN                  PORT_PIN_PE06

/*** Macros for efuse3_in pin ***/
#define efuse3_in_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 11U))
#define efuse3_in_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 11U))
#define efuse3_in_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 11U))
#define efuse3_in_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 11U))
#define efuse3_in_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 11U))
#define efuse3_in_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 11U)) & 0x01U)
#define efuse3_in_PIN                  PORT_PIN_PC11

/*** Macros for efuse3_den pin ***/
#define efuse3_den_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 10U))
#define efuse3_den_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 10U))
#define efuse3_den_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 10U))
#define efuse3_den_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 10U))
#define efuse3_den_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 10U))
#define efuse3_den_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 10U)) & 0x01U)
#define efuse3_den_PIN                  PORT_PIN_PC10

/*** Macros for Digital_Input_46 pin ***/
#define Digital_Input_46_Set()               (PORT_REGS->GROUP[6].PORT_OUTSET = ((uint32_t)1U << 2U))
#define Digital_Input_46_Clear()             (PORT_REGS->GROUP[6].PORT_OUTCLR = ((uint32_t)1U << 2U))
#define Digital_Input_46_Toggle()            (PORT_REGS->GROUP[6].PORT_OUTTGL = ((uint32_t)1U << 2U))
#define Digital_Input_46_OutputEnable()      (PORT_REGS->GROUP[6].PORT_DIRSET = ((uint32_t)1U << 2U))
#define Digital_Input_46_InputEnable()       (PORT_REGS->GROUP[6].PORT_DIRCLR = ((uint32_t)1U << 2U))
#define Digital_Input_46_Get()               (((PORT_REGS->GROUP[6].PORT_IN >> 2U)) & 0x01U)
#define Digital_Input_46_PIN                  PORT_PIN_PG02

/*** Macros for Digital_Input_1 pin ***/
#define Digital_Input_1_Set()               (PORT_REGS->GROUP[4].PORT_OUTSET = ((uint32_t)1U << 2U))
#define Digital_Input_1_Clear()             (PORT_REGS->GROUP[4].PORT_OUTCLR = ((uint32_t)1U << 2U))
#define Digital_Input_1_Toggle()            (PORT_REGS->GROUP[4].PORT_OUTTGL = ((uint32_t)1U << 2U))
#define Digital_Input_1_OutputEnable()      (PORT_REGS->GROUP[4].PORT_DIRSET = ((uint32_t)1U << 2U))
#define Digital_Input_1_InputEnable()       (PORT_REGS->GROUP[4].PORT_DIRCLR = ((uint32_t)1U << 2U))
#define Digital_Input_1_Get()               (((PORT_REGS->GROUP[4].PORT_IN >> 2U)) & 0x01U)
#define Digital_Input_1_PIN                  PORT_PIN_PE02

/*** Macros for efuse3_is pin ***/
#define efuse3_is_Get()               (((PORT_REGS->GROUP[4].PORT_IN >> 7U)) & 0x01U)
#define efuse3_is_PIN                  PORT_PIN_PE07

/*** Macros for efuse4_is pin ***/
#define efuse4_is_Get()               (((PORT_REGS->GROUP[4].PORT_IN >> 8U)) & 0x01U)
#define efuse4_is_PIN                  PORT_PIN_PE08

/*** Macros for SD_DATA2 pin ***/
#define SD_DATA2_Get()               (((PORT_REGS->GROUP[6].PORT_IN >> 1U)) & 0x01U)
#define SD_DATA2_PIN                  PORT_PIN_PG01

/*** Macros for Digital_Input_45 pin ***/
#define Digital_Input_45_Set()               (PORT_REGS->GROUP[6].PORT_OUTSET = ((uint32_t)1U << 5U))
#define Digital_Input_45_Clear()             (PORT_REGS->GROUP[6].PORT_OUTCLR = ((uint32_t)1U << 5U))
#define Digital_Input_45_Toggle()            (PORT_REGS->GROUP[6].PORT_OUTTGL = ((uint32_t)1U << 5U))
#define Digital_Input_45_OutputEnable()      (PORT_REGS->GROUP[6].PORT_DIRSET = ((uint32_t)1U << 5U))
#define Digital_Input_45_InputEnable()       (PORT_REGS->GROUP[6].PORT_DIRCLR = ((uint32_t)1U << 5U))
#define Digital_Input_45_Get()               (((PORT_REGS->GROUP[6].PORT_IN >> 5U)) & 0x01U)
#define Digital_Input_45_PIN                  PORT_PIN_PG05

/*** Macros for SD_DATA1 pin ***/
#define SD_DATA1_Get()               (((PORT_REGS->GROUP[6].PORT_IN >> 0U)) & 0x01U)
#define SD_DATA1_PIN                  PORT_PIN_PG00

/*** Macros for Analog_20 pin ***/
#define Analog_20_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 7U)) & 0x01U)
#define Analog_20_PIN                  PORT_PIN_PA07

/*** Macros for Analog_19 pin ***/
#define Analog_19_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 8U)) & 0x01U)
#define Analog_19_PIN                  PORT_PIN_PA08

/*** Macros for Digital_Input_2 pin ***/
#define Digital_Input_2_Set()               (PORT_REGS->GROUP[4].PORT_OUTSET = ((uint32_t)1U << 9U))
#define Digital_Input_2_Clear()             (PORT_REGS->GROUP[4].PORT_OUTCLR = ((uint32_t)1U << 9U))
#define Digital_Input_2_Toggle()            (PORT_REGS->GROUP[4].PORT_OUTTGL = ((uint32_t)1U << 9U))
#define Digital_Input_2_OutputEnable()      (PORT_REGS->GROUP[4].PORT_DIRSET = ((uint32_t)1U << 9U))
#define Digital_Input_2_InputEnable()       (PORT_REGS->GROUP[4].PORT_DIRCLR = ((uint32_t)1U << 9U))
#define Digital_Input_2_Get()               (((PORT_REGS->GROUP[4].PORT_IN >> 9U)) & 0x01U)
#define Digital_Input_2_PIN                  PORT_PIN_PE09

/*** Macros for efuse4_in pin ***/
#define efuse4_in_Set()               (PORT_REGS->GROUP[6].PORT_OUTSET = ((uint32_t)1U << 9U))
#define efuse4_in_Clear()             (PORT_REGS->GROUP[6].PORT_OUTCLR = ((uint32_t)1U << 9U))
#define efuse4_in_Toggle()            (PORT_REGS->GROUP[6].PORT_OUTTGL = ((uint32_t)1U << 9U))
#define efuse4_in_OutputEnable()      (PORT_REGS->GROUP[6].PORT_DIRSET = ((uint32_t)1U << 9U))
#define efuse4_in_InputEnable()       (PORT_REGS->GROUP[6].PORT_DIRCLR = ((uint32_t)1U << 9U))
#define efuse4_in_Get()               (((PORT_REGS->GROUP[6].PORT_IN >> 9U)) & 0x01U)
#define efuse4_in_PIN                  PORT_PIN_PG09

/*** Macros for SD_SCK pin ***/
#define SD_SCK_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 30U)) & 0x01U)
#define SD_SCK_PIN                  PORT_PIN_PC30

/*** Macros for Can2_RX pin ***/
#define Can2_RX_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 25U)) & 0x01U)
#define Can2_RX_PIN                  PORT_PIN_PC25

/*** Macros for Can2_TX pin ***/
#define Can2_TX_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 26U)) & 0x01U)
#define Can2_TX_PIN                  PORT_PIN_PC26

/*** Macros for Analog_14 pin ***/
#define Analog_14_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 9U)) & 0x01U)
#define Analog_14_PIN                  PORT_PIN_PA09

/*** Macros for Analog_13 pin ***/
#define Analog_13_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 10U)) & 0x01U)
#define Analog_13_PIN                  PORT_PIN_PA10

/*** Macros for Digital_Input_3 pin ***/
#define Digital_Input_3_Set()               (PORT_REGS->GROUP[4].PORT_OUTSET = ((uint32_t)1U << 10U))
#define Digital_Input_3_Clear()             (PORT_REGS->GROUP[4].PORT_OUTCLR = ((uint32_t)1U << 10U))
#define Digital_Input_3_Toggle()            (PORT_REGS->GROUP[4].PORT_OUTTGL = ((uint32_t)1U << 10U))
#define Digital_Input_3_OutputEnable()      (PORT_REGS->GROUP[4].PORT_DIRSET = ((uint32_t)1U << 10U))
#define Digital_Input_3_InputEnable()       (PORT_REGS->GROUP[4].PORT_DIRCLR = ((uint32_t)1U << 10U))
#define Digital_Input_3_Get()               (((PORT_REGS->GROUP[4].PORT_IN >> 10U)) & 0x01U)
#define Digital_Input_3_PIN                  PORT_PIN_PE10

/*** Macros for efuse1_in pin ***/
#define efuse1_in_Set()               (PORT_REGS->GROUP[6].PORT_OUTSET = ((uint32_t)1U << 7U))
#define efuse1_in_Clear()             (PORT_REGS->GROUP[6].PORT_OUTCLR = ((uint32_t)1U << 7U))
#define efuse1_in_Toggle()            (PORT_REGS->GROUP[6].PORT_OUTTGL = ((uint32_t)1U << 7U))
#define efuse1_in_OutputEnable()      (PORT_REGS->GROUP[6].PORT_DIRSET = ((uint32_t)1U << 7U))
#define efuse1_in_InputEnable()       (PORT_REGS->GROUP[6].PORT_DIRCLR = ((uint32_t)1U << 7U))
#define efuse1_in_Get()               (((PORT_REGS->GROUP[6].PORT_IN >> 7U)) & 0x01U)
#define efuse1_in_PIN                  PORT_PIN_PG07

/*** Macros for efuse1_den pin ***/
#define efuse1_den_Set()               (PORT_REGS->GROUP[6].PORT_OUTSET = ((uint32_t)1U << 6U))
#define efuse1_den_Clear()             (PORT_REGS->GROUP[6].PORT_OUTCLR = ((uint32_t)1U << 6U))
#define efuse1_den_Toggle()            (PORT_REGS->GROUP[6].PORT_OUTTGL = ((uint32_t)1U << 6U))
#define efuse1_den_OutputEnable()      (PORT_REGS->GROUP[6].PORT_DIRSET = ((uint32_t)1U << 6U))
#define efuse1_den_InputEnable()       (PORT_REGS->GROUP[6].PORT_DIRCLR = ((uint32_t)1U << 6U))
#define efuse1_den_Get()               (((PORT_REGS->GROUP[6].PORT_IN >> 6U)) & 0x01U)
#define efuse1_den_PIN                  PORT_PIN_PG06

/*** Macros for efuse4_den pin ***/
#define efuse4_den_Set()               (PORT_REGS->GROUP[6].PORT_OUTSET = ((uint32_t)1U << 8U))
#define efuse4_den_Clear()             (PORT_REGS->GROUP[6].PORT_OUTCLR = ((uint32_t)1U << 8U))
#define efuse4_den_Toggle()            (PORT_REGS->GROUP[6].PORT_OUTTGL = ((uint32_t)1U << 8U))
#define efuse4_den_OutputEnable()      (PORT_REGS->GROUP[6].PORT_DIRSET = ((uint32_t)1U << 8U))
#define efuse4_den_InputEnable()       (PORT_REGS->GROUP[6].PORT_DIRCLR = ((uint32_t)1U << 8U))
#define efuse4_den_Get()               (((PORT_REGS->GROUP[6].PORT_IN >> 8U)) & 0x01U)
#define efuse4_den_PIN                  PORT_PIN_PG08

/*** Macros for Analog_18 pin ***/
#define Analog_18_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 24U)) & 0x01U)
#define Analog_18_PIN                  PORT_PIN_PA24

/*** Macros for Analog_17 pin ***/
#define Analog_17_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 25U)) & 0x01U)
#define Analog_17_PIN                  PORT_PIN_PA25

/*** Macros for Digital_Input_5 pin ***/
#define Digital_Input_5_Set()               (PORT_REGS->GROUP[4].PORT_OUTSET = ((uint32_t)1U << 3U))
#define Digital_Input_5_Clear()             (PORT_REGS->GROUP[4].PORT_OUTCLR = ((uint32_t)1U << 3U))
#define Digital_Input_5_Toggle()            (PORT_REGS->GROUP[4].PORT_OUTTGL = ((uint32_t)1U << 3U))
#define Digital_Input_5_OutputEnable()      (PORT_REGS->GROUP[4].PORT_DIRSET = ((uint32_t)1U << 3U))
#define Digital_Input_5_InputEnable()       (PORT_REGS->GROUP[4].PORT_DIRCLR = ((uint32_t)1U << 3U))
#define Digital_Input_5_Get()               (((PORT_REGS->GROUP[4].PORT_IN >> 3U)) & 0x01U)
#define Digital_Input_5_PIN                  PORT_PIN_PE03

/*** Macros for Digital_Input_4 pin ***/
#define Digital_Input_4_Set()               (PORT_REGS->GROUP[4].PORT_OUTSET = ((uint32_t)1U << 11U))
#define Digital_Input_4_Clear()             (PORT_REGS->GROUP[4].PORT_OUTCLR = ((uint32_t)1U << 11U))
#define Digital_Input_4_Toggle()            (PORT_REGS->GROUP[4].PORT_OUTTGL = ((uint32_t)1U << 11U))
#define Digital_Input_4_OutputEnable()      (PORT_REGS->GROUP[4].PORT_DIRSET = ((uint32_t)1U << 11U))
#define Digital_Input_4_InputEnable()       (PORT_REGS->GROUP[4].PORT_DIRCLR = ((uint32_t)1U << 11U))
#define Digital_Input_4_Get()               (((PORT_REGS->GROUP[4].PORT_IN >> 11U)) & 0x01U)
#define Digital_Input_4_PIN                  PORT_PIN_PE11

/*** Macros for Digital_Input_6 pin ***/
#define Digital_Input_6_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 0U))
#define Digital_Input_6_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 0U))
#define Digital_Input_6_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 0U))
#define Digital_Input_6_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 0U))
#define Digital_Input_6_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 0U))
#define Digital_Input_6_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 0U)) & 0x01U)
#define Digital_Input_6_PIN                  PORT_PIN_PB00

/*** Macros for Digital_Output_13 pin ***/
#define Digital_Output_13_Set()               (PORT_REGS->GROUP[5].PORT_OUTSET = ((uint32_t)1U << 7U))
#define Digital_Output_13_Clear()             (PORT_REGS->GROUP[5].PORT_OUTCLR = ((uint32_t)1U << 7U))
#define Digital_Output_13_Toggle()            (PORT_REGS->GROUP[5].PORT_OUTTGL = ((uint32_t)1U << 7U))
#define Digital_Output_13_OutputEnable()      (PORT_REGS->GROUP[5].PORT_DIRSET = ((uint32_t)1U << 7U))
#define Digital_Output_13_InputEnable()       (PORT_REGS->GROUP[5].PORT_DIRCLR = ((uint32_t)1U << 7U))
#define Digital_Output_13_Get()               (((PORT_REGS->GROUP[5].PORT_IN >> 7U)) & 0x01U)
#define Digital_Output_13_PIN                  PORT_PIN_PF07

/*** Macros for Digital_Input_8 pin ***/
#define Digital_Input_8_Set()               (PORT_REGS->GROUP[5].PORT_OUTSET = ((uint32_t)1U << 8U))
#define Digital_Input_8_Clear()             (PORT_REGS->GROUP[5].PORT_OUTCLR = ((uint32_t)1U << 8U))
#define Digital_Input_8_Toggle()            (PORT_REGS->GROUP[5].PORT_OUTTGL = ((uint32_t)1U << 8U))
#define Digital_Input_8_OutputEnable()      (PORT_REGS->GROUP[5].PORT_DIRSET = ((uint32_t)1U << 8U))
#define Digital_Input_8_InputEnable()       (PORT_REGS->GROUP[5].PORT_DIRCLR = ((uint32_t)1U << 8U))
#define Digital_Input_8_Get()               (((PORT_REGS->GROUP[5].PORT_IN >> 8U)) & 0x01U)
#define Digital_Input_8_PIN                  PORT_PIN_PF08

/*** Macros for Digital_Input_7 pin ***/
#define Digital_Input_7_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 8U))
#define Digital_Input_7_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 8U))
#define Digital_Input_7_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 8U))
#define Digital_Input_7_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 8U))
#define Digital_Input_7_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 8U))
#define Digital_Input_7_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 8U)) & 0x01U)
#define Digital_Input_7_PIN                  PORT_PIN_PC08

/*** Macros for Digital_Input_30 pin ***/
#define Digital_Input_30_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 23U))
#define Digital_Input_30_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 23U))
#define Digital_Input_30_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 23U))
#define Digital_Input_30_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 23U))
#define Digital_Input_30_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 23U))
#define Digital_Input_30_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 23U)) & 0x01U)
#define Digital_Input_30_PIN                  PORT_PIN_PC23

/*** Macros for Relay_Output_1 pin ***/
#define Relay_Output_1_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 24U))
#define Relay_Output_1_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 24U))
#define Relay_Output_1_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 24U))
#define Relay_Output_1_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 24U))
#define Relay_Output_1_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 24U))
#define Relay_Output_1_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 24U)) & 0x01U)
#define Relay_Output_1_PIN                  PORT_PIN_PC24

/*** Macros for Digital_Output_12 pin ***/
#define Digital_Output_12_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 9U))
#define Digital_Output_12_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 9U))
#define Digital_Output_12_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 9U))
#define Digital_Output_12_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 9U))
#define Digital_Output_12_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 9U))
#define Digital_Output_12_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 9U)) & 0x01U)
#define Digital_Output_12_PIN                  PORT_PIN_PC09

/*** Macros for Analog_1 pin ***/
#define Analog_1_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 26U)) & 0x01U)
#define Analog_1_PIN                  PORT_PIN_PA26

/*** Macros for Analog_2 pin ***/
#define Analog_2_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 27U)) & 0x01U)
#define Analog_2_PIN                  PORT_PIN_PA27

/*** Macros for Analog_3 pin ***/
#define Analog_3_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 19U)) & 0x01U)
#define Analog_3_PIN                  PORT_PIN_PA19

/*** Macros for Analog_4 pin ***/
#define Analog_4_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 1U)) & 0x01U)
#define Analog_4_PIN                  PORT_PIN_PB01

/*** Macros for Digital_Output_14 pin ***/
#define Digital_Output_14_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 17U))
#define Digital_Output_14_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 17U))
#define Digital_Output_14_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 17U))
#define Digital_Output_14_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 17U))
#define Digital_Output_14_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 17U))
#define Digital_Output_14_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 17U)) & 0x01U)
#define Digital_Output_14_PIN                  PORT_PIN_PB17

/*** Macros for Can0_TX pin ***/
#define Can0_TX_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 5U)) & 0x01U)
#define Can0_TX_PIN                  PORT_PIN_PC05

/*** Macros for Can1_RX pin ***/
#define Can1_RX_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 6U)) & 0x01U)
#define Can1_RX_PIN                  PORT_PIN_PC06

/*** Macros for Can1_TX pin ***/
#define Can1_TX_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 7U)) & 0x01U)
#define Can1_TX_PIN                  PORT_PIN_PC07

/*** Macros for Analog_15 pin ***/
#define Analog_15_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 11U)) & 0x01U)
#define Analog_15_PIN                  PORT_PIN_PA11

/*** Macros for Analog_16 pin ***/
#define Analog_16_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 12U)) & 0x01U)
#define Analog_16_PIN                  PORT_PIN_PA12

/*** Macros for Analog_10 pin ***/
#define Analog_10_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 20U)) & 0x01U)
#define Analog_10_PIN                  PORT_PIN_PA20

/*** Macros for Debug_Transmit pin ***/
#define Debug_Transmit_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 0U)) & 0x01U)
#define Debug_Transmit_PIN                  PORT_PIN_PC00

/*** Macros for Digital_Output_2 pin ***/
#define Digital_Output_2_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 21U))
#define Digital_Output_2_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 21U))
#define Digital_Output_2_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 21U))
#define Digital_Output_2_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 21U))
#define Digital_Output_2_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 21U))
#define Digital_Output_2_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 21U)) & 0x01U)
#define Digital_Output_2_PIN                  PORT_PIN_PC21

/*** Macros for Digital_Output_3 pin ***/
#define Digital_Output_3_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 22U))
#define Digital_Output_3_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 22U))
#define Digital_Output_3_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 22U))
#define Digital_Output_3_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 22U))
#define Digital_Output_3_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 22U))
#define Digital_Output_3_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 22U)) & 0x01U)
#define Digital_Output_3_PIN                  PORT_PIN_PC22

/*** Macros for efuse1_is pin ***/
#define efuse1_is_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 13U)) & 0x01U)
#define efuse1_is_PIN                  PORT_PIN_PA13

/*** Macros for efuse2_is pin ***/
#define efuse2_is_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 14U)) & 0x01U)
#define efuse2_is_PIN                  PORT_PIN_PA14

/*** Macros for Analog_12 pin ***/
#define Analog_12_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 2U)) & 0x01U)
#define Analog_12_PIN                  PORT_PIN_PB02

/*** Macros for Digital_Output_19 pin ***/
#define Digital_Output_19_Set()               (PORT_REGS->GROUP[5].PORT_OUTSET = ((uint32_t)1U << 1U))
#define Digital_Output_19_Clear()             (PORT_REGS->GROUP[5].PORT_OUTCLR = ((uint32_t)1U << 1U))
#define Digital_Output_19_Toggle()            (PORT_REGS->GROUP[5].PORT_OUTTGL = ((uint32_t)1U << 1U))
#define Digital_Output_19_OutputEnable()      (PORT_REGS->GROUP[5].PORT_DIRSET = ((uint32_t)1U << 1U))
#define Digital_Output_19_InputEnable()       (PORT_REGS->GROUP[5].PORT_DIRCLR = ((uint32_t)1U << 1U))
#define Digital_Output_19_Get()               (((PORT_REGS->GROUP[5].PORT_IN >> 1U)) & 0x01U)
#define Digital_Output_19_PIN                  PORT_PIN_PF01

/*** Macros for Digital_Output_20 pin ***/
#define Digital_Output_20_Set()               (PORT_REGS->GROUP[5].PORT_OUTSET = ((uint32_t)1U << 2U))
#define Digital_Output_20_Clear()             (PORT_REGS->GROUP[5].PORT_OUTCLR = ((uint32_t)1U << 2U))
#define Digital_Output_20_Toggle()            (PORT_REGS->GROUP[5].PORT_OUTTGL = ((uint32_t)1U << 2U))
#define Digital_Output_20_OutputEnable()      (PORT_REGS->GROUP[5].PORT_DIRSET = ((uint32_t)1U << 2U))
#define Digital_Output_20_InputEnable()       (PORT_REGS->GROUP[5].PORT_DIRCLR = ((uint32_t)1U << 2U))
#define Digital_Output_20_Get()               (((PORT_REGS->GROUP[5].PORT_IN >> 2U)) & 0x01U)
#define Digital_Output_20_PIN                  PORT_PIN_PF02

/*** Macros for Digital_Output_21 pin ***/
#define Digital_Output_21_Set()               (PORT_REGS->GROUP[5].PORT_OUTSET = ((uint32_t)1U << 6U))
#define Digital_Output_21_Clear()             (PORT_REGS->GROUP[5].PORT_OUTCLR = ((uint32_t)1U << 6U))
#define Digital_Output_21_Toggle()            (PORT_REGS->GROUP[5].PORT_OUTTGL = ((uint32_t)1U << 6U))
#define Digital_Output_21_OutputEnable()      (PORT_REGS->GROUP[5].PORT_DIRSET = ((uint32_t)1U << 6U))
#define Digital_Output_21_InputEnable()       (PORT_REGS->GROUP[5].PORT_DIRCLR = ((uint32_t)1U << 6U))
#define Digital_Output_21_Get()               (((PORT_REGS->GROUP[5].PORT_IN >> 6U)) & 0x01U)
#define Digital_Output_21_PIN                  PORT_PIN_PF06

/*** Macros for Digital_Output_22 pin ***/
#define Digital_Output_22_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 26U))
#define Digital_Output_22_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 26U))
#define Digital_Output_22_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 26U))
#define Digital_Output_22_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 26U))
#define Digital_Output_22_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 26U))
#define Digital_Output_22_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 26U)) & 0x01U)
#define Digital_Output_22_PIN                  PORT_PIN_PB26

/*** Macros for Digital_Output_23 pin ***/
#define Digital_Output_23_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 12U))
#define Digital_Output_23_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 12U))
#define Digital_Output_23_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 12U))
#define Digital_Output_23_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 12U))
#define Digital_Output_23_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 12U))
#define Digital_Output_23_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 12U)) & 0x01U)
#define Digital_Output_23_PIN                  PORT_PIN_PB12

/*** Macros for Digital_Output_24 pin ***/
#define Digital_Output_24_Set()               (PORT_REGS->GROUP[6].PORT_OUTSET = ((uint32_t)1U << 4U))
#define Digital_Output_24_Clear()             (PORT_REGS->GROUP[6].PORT_OUTCLR = ((uint32_t)1U << 4U))
#define Digital_Output_24_Toggle()            (PORT_REGS->GROUP[6].PORT_OUTTGL = ((uint32_t)1U << 4U))
#define Digital_Output_24_OutputEnable()      (PORT_REGS->GROUP[6].PORT_DIRSET = ((uint32_t)1U << 4U))
#define Digital_Output_24_InputEnable()       (PORT_REGS->GROUP[6].PORT_DIRCLR = ((uint32_t)1U << 4U))
#define Digital_Output_24_Get()               (((PORT_REGS->GROUP[6].PORT_IN >> 4U)) & 0x01U)
#define Digital_Output_24_PIN                  PORT_PIN_PG04

/*** Macros for Digital_Output_1 pin ***/
#define Digital_Output_1_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 3U))
#define Digital_Output_1_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 3U))
#define Digital_Output_1_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 3U))
#define Digital_Output_1_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 3U))
#define Digital_Output_1_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 3U))
#define Digital_Output_1_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 3U)) & 0x01U)
#define Digital_Output_1_PIN                  PORT_PIN_PC03

/*** Macros for Can0_RX pin ***/
#define Can0_RX_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 4U)) & 0x01U)
#define Can0_RX_PIN                  PORT_PIN_PC04

/*** Macros for Analog_9 pin ***/
#define Analog_9_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 28U)) & 0x01U)
#define Analog_9_PIN                  PORT_PIN_PA28

/*** Macros for Analog_6 pin ***/
#define Analog_6_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 29U)) & 0x01U)
#define Analog_6_PIN                  PORT_PIN_PA29

/*** Macros for Debug_RXD pin ***/
#define Debug_RXD_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 1U)) & 0x01U)
#define Debug_RXD_PIN                  PORT_PIN_PC01

/*** Macros for Digital_Output_11 pin ***/
#define Digital_Output_11_Set()               (PORT_REGS->GROUP[2].PORT_OUTSET = ((uint32_t)1U << 2U))
#define Digital_Output_11_Clear()             (PORT_REGS->GROUP[2].PORT_OUTCLR = ((uint32_t)1U << 2U))
#define Digital_Output_11_Toggle()            (PORT_REGS->GROUP[2].PORT_OUTTGL = ((uint32_t)1U << 2U))
#define Digital_Output_11_OutputEnable()      (PORT_REGS->GROUP[2].PORT_DIRSET = ((uint32_t)1U << 2U))
#define Digital_Output_11_InputEnable()       (PORT_REGS->GROUP[2].PORT_DIRCLR = ((uint32_t)1U << 2U))
#define Digital_Output_11_Get()               (((PORT_REGS->GROUP[2].PORT_IN >> 2U)) & 0x01U)
#define Digital_Output_11_PIN                  PORT_PIN_PC02

/*** Macros for IO_Exp_Reset pin ***/
#define IO_Exp_Reset_Set()               (PORT_REGS->GROUP[0].PORT_OUTSET = ((uint32_t)1U << 15U))
#define IO_Exp_Reset_Clear()             (PORT_REGS->GROUP[0].PORT_OUTCLR = ((uint32_t)1U << 15U))
#define IO_Exp_Reset_Toggle()            (PORT_REGS->GROUP[0].PORT_OUTTGL = ((uint32_t)1U << 15U))
#define IO_Exp_Reset_OutputEnable()      (PORT_REGS->GROUP[0].PORT_DIRSET = ((uint32_t)1U << 15U))
#define IO_Exp_Reset_InputEnable()       (PORT_REGS->GROUP[0].PORT_DIRCLR = ((uint32_t)1U << 15U))
#define IO_Exp_Reset_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 15U)) & 0x01U)
#define IO_Exp_Reset_PIN                  PORT_PIN_PA15

/*** Macros for Analog_11 pin ***/
#define Analog_11_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 16U)) & 0x01U)
#define Analog_11_PIN                  PORT_PIN_PA16

/*** Macros for Analog_8 pin ***/
#define Analog_8_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 18U)) & 0x01U)
#define Analog_8_PIN                  PORT_PIN_PA18

/*** Macros for Analog_5 pin ***/
#define Analog_5_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 3U)) & 0x01U)
#define Analog_5_PIN                  PORT_PIN_PB03

/*** Macros for Analog_7 pin ***/
#define Analog_7_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 19U)) & 0x01U)
#define Analog_7_PIN                  PORT_PIN_PB19

/*** Macros for IO_Expander_Int pin ***/
#define IO_Expander_Int_Set()               (PORT_REGS->GROUP[5].PORT_OUTSET = ((uint32_t)1U << 0U))
#define IO_Expander_Int_Clear()             (PORT_REGS->GROUP[5].PORT_OUTCLR = ((uint32_t)1U << 0U))
#define IO_Expander_Int_Toggle()            (PORT_REGS->GROUP[5].PORT_OUTTGL = ((uint32_t)1U << 0U))
#define IO_Expander_Int_OutputEnable()      (PORT_REGS->GROUP[5].PORT_DIRSET = ((uint32_t)1U << 0U))
#define IO_Expander_Int_InputEnable()       (PORT_REGS->GROUP[5].PORT_DIRCLR = ((uint32_t)1U << 0U))
#define IO_Expander_Int_Get()               (((PORT_REGS->GROUP[5].PORT_IN >> 0U)) & 0x01U)
#define IO_Expander_Int_PIN                  PORT_PIN_PF00

/*** Macros for Digital_Input_32 pin ***/
#define Digital_Input_32_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 23U))
#define Digital_Input_32_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 23U))
#define Digital_Input_32_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 23U))
#define Digital_Input_32_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 23U))
#define Digital_Input_32_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 23U))
#define Digital_Input_32_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 23U)) & 0x01U)
#define Digital_Input_32_PIN                  PORT_PIN_PB23

/*** Macros for Digital_Input_34 pin ***/
#define Digital_Input_34_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 8U))
#define Digital_Input_34_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 8U))
#define Digital_Input_34_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 8U))
#define Digital_Input_34_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 8U))
#define Digital_Input_34_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 8U))
#define Digital_Input_34_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 8U)) & 0x01U)
#define Digital_Input_34_PIN                  PORT_PIN_PB08

/*** Macros for Digital_Input_36 pin ***/
#define Digital_Input_36_Set()               (PORT_REGS->GROUP[5].PORT_OUTSET = ((uint32_t)1U << 4U))
#define Digital_Input_36_Clear()             (PORT_REGS->GROUP[5].PORT_OUTCLR = ((uint32_t)1U << 4U))
#define Digital_Input_36_Toggle()            (PORT_REGS->GROUP[5].PORT_OUTTGL = ((uint32_t)1U << 4U))
#define Digital_Input_36_OutputEnable()      (PORT_REGS->GROUP[5].PORT_DIRSET = ((uint32_t)1U << 4U))
#define Digital_Input_36_InputEnable()       (PORT_REGS->GROUP[5].PORT_DIRCLR = ((uint32_t)1U << 4U))
#define Digital_Input_36_Get()               (((PORT_REGS->GROUP[5].PORT_IN >> 4U)) & 0x01U)
#define Digital_Input_36_PIN                  PORT_PIN_PF04

/*** Macros for Digital_Input_38 pin ***/
#define Digital_Input_38_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 28U))
#define Digital_Input_38_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 28U))
#define Digital_Input_38_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 28U))
#define Digital_Input_38_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 28U))
#define Digital_Input_38_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 28U))
#define Digital_Input_38_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 28U)) & 0x01U)
#define Digital_Input_38_PIN                  PORT_PIN_PB28

/*** Macros for Digital_Input_40 pin ***/
#define Digital_Input_40_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 9U))
#define Digital_Input_40_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 9U))
#define Digital_Input_40_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 9U))
#define Digital_Input_40_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 9U))
#define Digital_Input_40_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 9U))
#define Digital_Input_40_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 9U)) & 0x01U)
#define Digital_Input_40_PIN                  PORT_PIN_PB09

/*** Macros for Digital_Output_5 pin ***/
#define Digital_Output_5_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 29U))
#define Digital_Output_5_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 29U))
#define Digital_Output_5_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 29U))
#define Digital_Output_5_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 29U))
#define Digital_Output_5_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 29U))
#define Digital_Output_5_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 29U)) & 0x01U)
#define Digital_Output_5_PIN                  PORT_PIN_PB29

/*** Macros for Digital_Output_8 pin ***/
#define Digital_Output_8_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 30U))
#define Digital_Output_8_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 30U))
#define Digital_Output_8_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 30U))
#define Digital_Output_8_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 30U))
#define Digital_Output_8_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 30U))
#define Digital_Output_8_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 30U)) & 0x01U)
#define Digital_Output_8_PIN                  PORT_PIN_PB30

/*** Macros for Digital_Output_18 pin ***/
#define Digital_Output_18_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 24U))
#define Digital_Output_18_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 24U))
#define Digital_Output_18_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 24U))
#define Digital_Output_18_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 24U))
#define Digital_Output_18_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 24U))
#define Digital_Output_18_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 24U)) & 0x01U)
#define Digital_Output_18_PIN                  PORT_PIN_PB24

/*** Macros for Digital_Output_16 pin ***/
#define Digital_Output_16_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 13U))
#define Digital_Output_16_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 13U))
#define Digital_Output_16_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 13U))
#define Digital_Output_16_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 13U))
#define Digital_Output_16_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 13U))
#define Digital_Output_16_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 13U)) & 0x01U)
#define Digital_Output_16_PIN                  PORT_PIN_PB13

/*** Macros for Digital_Output_9 pin ***/
#define Digital_Output_9_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 15U))
#define Digital_Output_9_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 15U))
#define Digital_Output_9_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 15U))
#define Digital_Output_9_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 15U))
#define Digital_Output_9_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 15U))
#define Digital_Output_9_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 15U)) & 0x01U)
#define Digital_Output_9_PIN                  PORT_PIN_PB15

/*** Macros for Digital_Output_10 pin ***/
#define Digital_Output_10_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 16U))
#define Digital_Output_10_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 16U))
#define Digital_Output_10_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 16U))
#define Digital_Output_10_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 16U))
#define Digital_Output_10_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 16U))
#define Digital_Output_10_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 16U)) & 0x01U)
#define Digital_Output_10_PIN                  PORT_PIN_PB16

/*** Macros for Digital_Input_25 pin ***/
#define Digital_Input_25_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 18U))
#define Digital_Input_25_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 18U))
#define Digital_Input_25_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 18U))
#define Digital_Input_25_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 18U))
#define Digital_Input_25_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 18U))
#define Digital_Input_25_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 18U)) & 0x01U)
#define Digital_Input_25_PIN                  PORT_PIN_PB18

/*** Macros for Digital_Input_27 pin ***/
#define Digital_Input_27_Set()               (PORT_REGS->GROUP[0].PORT_OUTSET = ((uint32_t)1U << 17U))
#define Digital_Input_27_Clear()             (PORT_REGS->GROUP[0].PORT_OUTCLR = ((uint32_t)1U << 17U))
#define Digital_Input_27_Toggle()            (PORT_REGS->GROUP[0].PORT_OUTTGL = ((uint32_t)1U << 17U))
#define Digital_Input_27_OutputEnable()      (PORT_REGS->GROUP[0].PORT_DIRSET = ((uint32_t)1U << 17U))
#define Digital_Input_27_InputEnable()       (PORT_REGS->GROUP[0].PORT_DIRCLR = ((uint32_t)1U << 17U))
#define Digital_Input_27_Get()               (((PORT_REGS->GROUP[0].PORT_IN >> 17U)) & 0x01U)
#define Digital_Input_27_PIN                  PORT_PIN_PA17

/*** Macros for Digital_Input_28 pin ***/
#define Digital_Input_28_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 20U))
#define Digital_Input_28_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 20U))
#define Digital_Input_28_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 20U))
#define Digital_Input_28_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 20U))
#define Digital_Input_28_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 20U))
#define Digital_Input_28_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 20U)) & 0x01U)
#define Digital_Input_28_PIN                  PORT_PIN_PB20

/*** Macros for Digital_Input_26 pin ***/
#define Digital_Input_26_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 21U))
#define Digital_Input_26_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 21U))
#define Digital_Input_26_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 21U))
#define Digital_Input_26_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 21U))
#define Digital_Input_26_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 21U))
#define Digital_Input_26_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 21U)) & 0x01U)
#define Digital_Input_26_PIN                  PORT_PIN_PB21

/*** Macros for Digital_Input_29 pin ***/
#define Digital_Input_29_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 6U))
#define Digital_Input_29_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 6U))
#define Digital_Input_29_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 6U))
#define Digital_Input_29_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 6U))
#define Digital_Input_29_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 6U))
#define Digital_Input_29_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 6U)) & 0x01U)
#define Digital_Input_29_PIN                  PORT_PIN_PB06

/*** Macros for Digital_Input_31 pin ***/
#define Digital_Input_31_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 22U))
#define Digital_Input_31_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 22U))
#define Digital_Input_31_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 22U))
#define Digital_Input_31_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 22U))
#define Digital_Input_31_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 22U))
#define Digital_Input_31_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 22U)) & 0x01U)
#define Digital_Input_31_PIN                  PORT_PIN_PB22

/*** Macros for Digital_Input_33 pin ***/
#define Digital_Input_33_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 7U))
#define Digital_Input_33_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 7U))
#define Digital_Input_33_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 7U))
#define Digital_Input_33_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 7U))
#define Digital_Input_33_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 7U))
#define Digital_Input_33_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 7U)) & 0x01U)
#define Digital_Input_33_PIN                  PORT_PIN_PB07

/*** Macros for Digital_Input_35 pin ***/
#define Digital_Input_35_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 27U))
#define Digital_Input_35_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 27U))
#define Digital_Input_35_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 27U))
#define Digital_Input_35_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 27U))
#define Digital_Input_35_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 27U))
#define Digital_Input_35_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 27U)) & 0x01U)
#define Digital_Input_35_PIN                  PORT_PIN_PB27

/*** Macros for Digital_Input_37 pin ***/
#define Digital_Input_37_Set()               (PORT_REGS->GROUP[5].PORT_OUTSET = ((uint32_t)1U << 3U))
#define Digital_Input_37_Clear()             (PORT_REGS->GROUP[5].PORT_OUTCLR = ((uint32_t)1U << 3U))
#define Digital_Input_37_Toggle()            (PORT_REGS->GROUP[5].PORT_OUTTGL = ((uint32_t)1U << 3U))
#define Digital_Input_37_OutputEnable()      (PORT_REGS->GROUP[5].PORT_DIRSET = ((uint32_t)1U << 3U))
#define Digital_Input_37_InputEnable()       (PORT_REGS->GROUP[5].PORT_DIRCLR = ((uint32_t)1U << 3U))
#define Digital_Input_37_Get()               (((PORT_REGS->GROUP[5].PORT_IN >> 3U)) & 0x01U)
#define Digital_Input_37_PIN                  PORT_PIN_PF03

/*** Macros for Digital_Input_39 pin ***/
#define Digital_Input_39_Set()               (PORT_REGS->GROUP[5].PORT_OUTSET = ((uint32_t)1U << 5U))
#define Digital_Input_39_Clear()             (PORT_REGS->GROUP[5].PORT_OUTCLR = ((uint32_t)1U << 5U))
#define Digital_Input_39_Toggle()            (PORT_REGS->GROUP[5].PORT_OUTTGL = ((uint32_t)1U << 5U))
#define Digital_Input_39_OutputEnable()      (PORT_REGS->GROUP[5].PORT_DIRSET = ((uint32_t)1U << 5U))
#define Digital_Input_39_InputEnable()       (PORT_REGS->GROUP[5].PORT_DIRCLR = ((uint32_t)1U << 5U))
#define Digital_Input_39_Get()               (((PORT_REGS->GROUP[5].PORT_IN >> 5U)) & 0x01U)
#define Digital_Input_39_PIN                  PORT_PIN_PF05

/*** Macros for Digital_Output_4 pin ***/
#define Digital_Output_4_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 10U))
#define Digital_Output_4_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 10U))
#define Digital_Output_4_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 10U))
#define Digital_Output_4_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 10U))
#define Digital_Output_4_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 10U))
#define Digital_Output_4_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 10U)) & 0x01U)
#define Digital_Output_4_PIN                  PORT_PIN_PB10

/*** Macros for Digital_Output_6 pin ***/
#define Digital_Output_6_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 11U))
#define Digital_Output_6_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 11U))
#define Digital_Output_6_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 11U))
#define Digital_Output_6_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 11U))
#define Digital_Output_6_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 11U))
#define Digital_Output_6_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 11U)) & 0x01U)
#define Digital_Output_6_PIN                  PORT_PIN_PB11

/*** Macros for Digital_Output_17 pin ***/
#define Digital_Output_17_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 31U))
#define Digital_Output_17_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 31U))
#define Digital_Output_17_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 31U))
#define Digital_Output_17_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 31U))
#define Digital_Output_17_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 31U))
#define Digital_Output_17_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 31U)) & 0x01U)
#define Digital_Output_17_PIN                  PORT_PIN_PB31

/*** Macros for Digital_Output_15 pin ***/
#define Digital_Output_15_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 25U))
#define Digital_Output_15_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 25U))
#define Digital_Output_15_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 25U))
#define Digital_Output_15_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 25U))
#define Digital_Output_15_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 25U))
#define Digital_Output_15_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 25U)) & 0x01U)
#define Digital_Output_15_PIN                  PORT_PIN_PB25

/*** Macros for Digital_Output_7 pin ***/
#define Digital_Output_7_Set()               (PORT_REGS->GROUP[1].PORT_OUTSET = ((uint32_t)1U << 14U))
#define Digital_Output_7_Clear()             (PORT_REGS->GROUP[1].PORT_OUTCLR = ((uint32_t)1U << 14U))
#define Digital_Output_7_Toggle()            (PORT_REGS->GROUP[1].PORT_OUTTGL = ((uint32_t)1U << 14U))
#define Digital_Output_7_OutputEnable()      (PORT_REGS->GROUP[1].PORT_DIRSET = ((uint32_t)1U << 14U))
#define Digital_Output_7_InputEnable()       (PORT_REGS->GROUP[1].PORT_DIRCLR = ((uint32_t)1U << 14U))
#define Digital_Output_7_Get()               (((PORT_REGS->GROUP[1].PORT_IN >> 14U)) & 0x01U)
#define Digital_Output_7_PIN                  PORT_PIN_PB14

// *****************************************************************************
/* PORT Group

  Summary:
    Identifies the port groups available on the device.

  Description:
    These macros identifies all the ports groups that are available on this
    device.

  Remarks:
    The caller should not use the constant expressions assigned to any of
    the preprocessor macros as these may vary between devices.

    Port groups shown here are the ones available on the selected device. Not
    all ports groups are implemented. Refer to the device specific datasheet
    for more details. The MHC will generate these macros with the port
    groups that are available on the device.
*/

/* Group 0 */
#define PORT_GROUP_0 (PORT_BASE_ADDRESS + (0U * 0x80U))

/* Group 1 */
#define PORT_GROUP_1 (PORT_BASE_ADDRESS + (1U * 0x80U))

/* Group 2 */
#define PORT_GROUP_2 (PORT_BASE_ADDRESS + (2U * 0x80U))

/* Group 3 */
#define PORT_GROUP_3 (PORT_BASE_ADDRESS + (3U * 0x80U))

/* Group 4 */
#define PORT_GROUP_4 (PORT_BASE_ADDRESS + (4U * 0x80U))

/* Group 5 */
#define PORT_GROUP_5 (PORT_BASE_ADDRESS + (5U * 0x80U))

/* Group 6 */
#define PORT_GROUP_6 (PORT_BASE_ADDRESS + (6U * 0x80U))


/* Helper macros to get port information from the pin */
#define GET_PORT_GROUP(pin)  ((PORT_GROUP)(PORT_BASE_ADDRESS + (0x80U * (((uint32_t)pin) >> 5U))))
#define GET_PIN_MASK(pin)   (((uint32_t)(0x1U)) << (((uint32_t)pin) & 0x1FU))

/* Named type for port group */
typedef uint32_t PORT_GROUP;


typedef enum
{
PERIPHERAL_FUNCTION_A = 0x0,
PERIPHERAL_FUNCTION_B = 0x1,
PERIPHERAL_FUNCTION_C = 0x2,
PERIPHERAL_FUNCTION_D = 0x3,
PERIPHERAL_FUNCTION_E = 0x4,
PERIPHERAL_FUNCTION_F = 0x5,
PERIPHERAL_FUNCTION_G = 0x6,
PERIPHERAL_FUNCTION_H = 0x7,
PERIPHERAL_FUNCTION_I = 0x8,
PERIPHERAL_FUNCTION_J = 0x9,
PERIPHERAL_FUNCTION_K = 0xA,
PERIPHERAL_FUNCTION_L = 0xB,
PERIPHERAL_FUNCTION_M = 0xC,
PERIPHERAL_FUNCTION_N = 0xD,

}PERIPHERAL_FUNCTION;

// *****************************************************************************
/* PORT Pins

  Summary:
    Identifies the available Ports pins.

  Description:
    This enumeration identifies all the ports pins that are available on this
    device.

  Remarks:
    The caller should not use the constant expressions assigned to any of
    the enumeration constants as these may vary between devices.

    Port pins shown here are the ones available on the selected device. Not
    all ports pins within a port group are implemented. Refer to the device
    specific datasheet for more details.
*/

typedef enum
{
    /* PA00 pin */
    PORT_PIN_PA00 = 0U,

    /* PA01 pin */
    PORT_PIN_PA01 = 1U,

    /* PA02 pin */
    PORT_PIN_PA02 = 2U,

    /* PA03 pin */
    PORT_PIN_PA03 = 3U,

    /* PA04 pin */
    PORT_PIN_PA04 = 4U,

    /* PA05 pin */
    PORT_PIN_PA05 = 5U,

    /* PA06 pin */
    PORT_PIN_PA06 = 6U,

    /* PA07 pin */
    PORT_PIN_PA07 = 7U,

    /* PA08 pin */
    PORT_PIN_PA08 = 8U,

    /* PA09 pin */
    PORT_PIN_PA09 = 9U,

    /* PA10 pin */
    PORT_PIN_PA10 = 10U,

    /* PA11 pin */
    PORT_PIN_PA11 = 11U,

    /* PA12 pin */
    PORT_PIN_PA12 = 12U,

    /* PA13 pin */
    PORT_PIN_PA13 = 13U,

    /* PA14 pin */
    PORT_PIN_PA14 = 14U,

    /* PA15 pin */
    PORT_PIN_PA15 = 15U,

    /* PA16 pin */
    PORT_PIN_PA16 = 16U,

    /* PA17 pin */
    PORT_PIN_PA17 = 17U,

    /* PA18 pin */
    PORT_PIN_PA18 = 18U,

    /* PA19 pin */
    PORT_PIN_PA19 = 19U,

    /* PA20 pin */
    PORT_PIN_PA20 = 20U,

    /* PA21 pin */
    PORT_PIN_PA21 = 21U,

    /* PA22 pin */
    PORT_PIN_PA22 = 22U,

    /* PA23 pin */
    PORT_PIN_PA23 = 23U,

    /* PA24 pin */
    PORT_PIN_PA24 = 24U,

    /* PA25 pin */
    PORT_PIN_PA25 = 25U,

    /* PA26 pin */
    PORT_PIN_PA26 = 26U,

    /* PA27 pin */
    PORT_PIN_PA27 = 27U,

    /* PA28 pin */
    PORT_PIN_PA28 = 28U,

    /* PA29 pin */
    PORT_PIN_PA29 = 29U,

    /* PA30 pin */
    PORT_PIN_PA30 = 30U,

    /* PA31 pin */
    PORT_PIN_PA31 = 31U,

    /* PB00 pin */
    PORT_PIN_PB00 = 32U,

    /* PB01 pin */
    PORT_PIN_PB01 = 33U,

    /* PB02 pin */
    PORT_PIN_PB02 = 34U,

    /* PB03 pin */
    PORT_PIN_PB03 = 35U,

    /* PB04 pin */
    PORT_PIN_PB04 = 36U,

    /* PB05 pin */
    PORT_PIN_PB05 = 37U,

    /* PB06 pin */
    PORT_PIN_PB06 = 38U,

    /* PB07 pin */
    PORT_PIN_PB07 = 39U,

    /* PB08 pin */
    PORT_PIN_PB08 = 40U,

    /* PB09 pin */
    PORT_PIN_PB09 = 41U,

    /* PB10 pin */
    PORT_PIN_PB10 = 42U,

    /* PB11 pin */
    PORT_PIN_PB11 = 43U,

    /* PB12 pin */
    PORT_PIN_PB12 = 44U,

    /* PB13 pin */
    PORT_PIN_PB13 = 45U,

    /* PB14 pin */
    PORT_PIN_PB14 = 46U,

    /* PB15 pin */
    PORT_PIN_PB15 = 47U,

    /* PB16 pin */
    PORT_PIN_PB16 = 48U,

    /* PB17 pin */
    PORT_PIN_PB17 = 49U,

    /* PB18 pin */
    PORT_PIN_PB18 = 50U,

    /* PB19 pin */
    PORT_PIN_PB19 = 51U,

    /* PB20 pin */
    PORT_PIN_PB20 = 52U,

    /* PB21 pin */
    PORT_PIN_PB21 = 53U,

    /* PB22 pin */
    PORT_PIN_PB22 = 54U,

    /* PB23 pin */
    PORT_PIN_PB23 = 55U,

    /* PB24 pin */
    PORT_PIN_PB24 = 56U,

    /* PB25 pin */
    PORT_PIN_PB25 = 57U,

    /* PB26 pin */
    PORT_PIN_PB26 = 58U,

    /* PB27 pin */
    PORT_PIN_PB27 = 59U,

    /* PB28 pin */
    PORT_PIN_PB28 = 60U,

    /* PB29 pin */
    PORT_PIN_PB29 = 61U,

    /* PB30 pin */
    PORT_PIN_PB30 = 62U,

    /* PB31 pin */
    PORT_PIN_PB31 = 63U,

    /* PC00 pin */
    PORT_PIN_PC00 = 64U,

    /* PC01 pin */
    PORT_PIN_PC01 = 65U,

    /* PC02 pin */
    PORT_PIN_PC02 = 66U,

    /* PC03 pin */
    PORT_PIN_PC03 = 67U,

    /* PC04 pin */
    PORT_PIN_PC04 = 68U,

    /* PC05 pin */
    PORT_PIN_PC05 = 69U,

    /* PC06 pin */
    PORT_PIN_PC06 = 70U,

    /* PC07 pin */
    PORT_PIN_PC07 = 71U,

    /* PC08 pin */
    PORT_PIN_PC08 = 72U,

    /* PC09 pin */
    PORT_PIN_PC09 = 73U,

    /* PC10 pin */
    PORT_PIN_PC10 = 74U,

    /* PC11 pin */
    PORT_PIN_PC11 = 75U,

    /* PC12 pin */
    PORT_PIN_PC12 = 76U,

    /* PC13 pin */
    PORT_PIN_PC13 = 77U,

    /* PC14 pin */
    PORT_PIN_PC14 = 78U,

    /* PC15 pin */
    PORT_PIN_PC15 = 79U,

    /* PC17 pin */
    PORT_PIN_PC17 = 81U,

    /* PC18 pin */
    PORT_PIN_PC18 = 82U,

    /* PC19 pin */
    PORT_PIN_PC19 = 83U,

    /* PC20 pin */
    PORT_PIN_PC20 = 84U,

    /* PC21 pin */
    PORT_PIN_PC21 = 85U,

    /* PC22 pin */
    PORT_PIN_PC22 = 86U,

    /* PC23 pin */
    PORT_PIN_PC23 = 87U,

    /* PC24 pin */
    PORT_PIN_PC24 = 88U,

    /* PC25 pin */
    PORT_PIN_PC25 = 89U,

    /* PC26 pin */
    PORT_PIN_PC26 = 90U,

    /* PC27 pin */
    PORT_PIN_PC27 = 91U,

    /* PC28 pin */
    PORT_PIN_PC28 = 92U,

    /* PC29 pin */
    PORT_PIN_PC29 = 93U,

    /* PC30 pin */
    PORT_PIN_PC30 = 94U,

    /* PC31 pin */
    PORT_PIN_PC31 = 95U,

    /* PD00 pin */
    PORT_PIN_PD00 = 96U,

    /* PD01 pin */
    PORT_PIN_PD01 = 97U,

    /* PD02 pin */
    PORT_PIN_PD02 = 98U,

    /* PD03 pin */
    PORT_PIN_PD03 = 99U,

    /* PD04 pin */
    PORT_PIN_PD04 = 100U,

    /* PD05 pin */
    PORT_PIN_PD05 = 101U,

    /* PD06 pin */
    PORT_PIN_PD06 = 102U,

    /* PD07 pin */
    PORT_PIN_PD07 = 103U,

    /* PD08 pin */
    PORT_PIN_PD08 = 104U,

    /* PD09 pin */
    PORT_PIN_PD09 = 105U,

    /* PD10 pin */
    PORT_PIN_PD10 = 106U,

    /* PD11 pin */
    PORT_PIN_PD11 = 107U,

    /* PD12 pin */
    PORT_PIN_PD12 = 108U,

    /* PD13 pin */
    PORT_PIN_PD13 = 109U,

    /* PD14 pin */
    PORT_PIN_PD14 = 110U,

    /* PD15 pin */
    PORT_PIN_PD15 = 111U,

    /* PD16 pin */
    PORT_PIN_PD16 = 112U,

    /* PD17 pin */
    PORT_PIN_PD17 = 113U,

    /* PD18 pin */
    PORT_PIN_PD18 = 114U,

    /* PD19 pin */
    PORT_PIN_PD19 = 115U,

    /* PD20 pin */
    PORT_PIN_PD20 = 116U,

    /* PD21 pin */
    PORT_PIN_PD21 = 117U,

    /* PD23 pin */
    PORT_PIN_PD23 = 119U,

    /* PD24 pin */
    PORT_PIN_PD24 = 120U,

    /* PD25 pin */
    PORT_PIN_PD25 = 121U,

    /* PD26 pin */
    PORT_PIN_PD26 = 122U,

    /* PD27 pin */
    PORT_PIN_PD27 = 123U,

    /* PD28 pin */
    PORT_PIN_PD28 = 124U,

    /* PD29 pin */
    PORT_PIN_PD29 = 125U,

    /* PE00 pin */
    PORT_PIN_PE00 = 128U,

    /* PE01 pin */
    PORT_PIN_PE01 = 129U,

    /* PE02 pin */
    PORT_PIN_PE02 = 130U,

    /* PE03 pin */
    PORT_PIN_PE03 = 131U,

    /* PE04 pin */
    PORT_PIN_PE04 = 132U,

    /* PE05 pin */
    PORT_PIN_PE05 = 133U,

    /* PE06 pin */
    PORT_PIN_PE06 = 134U,

    /* PE07 pin */
    PORT_PIN_PE07 = 135U,

    /* PE08 pin */
    PORT_PIN_PE08 = 136U,

    /* PE09 pin */
    PORT_PIN_PE09 = 137U,

    /* PE10 pin */
    PORT_PIN_PE10 = 138U,

    /* PE11 pin */
    PORT_PIN_PE11 = 139U,

    /* PF00 pin */
    PORT_PIN_PF00 = 160U,

    /* PF01 pin */
    PORT_PIN_PF01 = 161U,

    /* PF02 pin */
    PORT_PIN_PF02 = 162U,

    /* PF03 pin */
    PORT_PIN_PF03 = 163U,

    /* PF04 pin */
    PORT_PIN_PF04 = 164U,

    /* PF05 pin */
    PORT_PIN_PF05 = 165U,

    /* PF06 pin */
    PORT_PIN_PF06 = 166U,

    /* PF07 pin */
    PORT_PIN_PF07 = 167U,

    /* PF08 pin */
    PORT_PIN_PF08 = 168U,

    /* PG00 pin */
    PORT_PIN_PG00 = 192U,

    /* PG01 pin */
    PORT_PIN_PG01 = 193U,

    /* PG02 pin */
    PORT_PIN_PG02 = 194U,

    /* PG03 pin */
    PORT_PIN_PG03 = 195U,

    /* PG04 pin */
    PORT_PIN_PG04 = 196U,

    /* PG05 pin */
    PORT_PIN_PG05 = 197U,

    /* PG06 pin */
    PORT_PIN_PG06 = 198U,

    /* PG07 pin */
    PORT_PIN_PG07 = 199U,

    /* PG08 pin */
    PORT_PIN_PG08 = 200U,

    /* PG09 pin */
    PORT_PIN_PG09 = 201U,

    /* PG10 pin */
    PORT_PIN_PG10 = 202U,

    /* PG11 pin */
    PORT_PIN_PG11 = 203U,

    /* This element should not be used in any of the PORT APIs.
     * It will be used by other modules or application to denote that none of
     * the PORT Pin is used */
    PORT_PIN_NONE = 65535U,

} PORT_PIN;

// *****************************************************************************
// *****************************************************************************
// Section: Generated API based on pin configurations done in Pin Manager
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Function:
    void PORT_Initialize(void)

  Summary:
    Initializes the PORT Library.

  Description:
    This function initializes all ports and pins as configured in the
    MHC Pin Manager.

  Precondition:
    None.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>

    PORT_Initialize();

    </code>

  Remarks:
    The function should be called once before calling any other PORTS PLIB
    functions.
*/

void PORT_Initialize(void);

// *****************************************************************************
// *****************************************************************************
// Section: PORT APIs which operates on multiple pins of a group
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    uint32_t PORT_GroupRead(PORT_GROUP group)

  Summary:
    Read all the I/O pins in the specified port group.

  Description:
    The function reads the hardware pin state of all pins in the specified group
    and returns this as a 32 bit value. Each bit in the 32 bit value represent a
    pin. For example, bit 0 in group 0 will represent pin PA0. Bit 1 will
    represent PA1 and so on. The application should only consider the value of
    the port group pins which are implemented on the device.

  Precondition:
    The PORT_Initialize() function should have been called. Input buffer
    (INEN bit in the Pin Configuration register) should be enabled in MHC.

  Parameters:
    group - One of the IO groups from the enum PORT_GROUP.

  Returns:
    A 32-bit value representing the hardware state of of all the I/O pins in the
    selected port group.

  Example:
    <code>

    uint32_t value;
    value = PORT_Read(PORT_GROUP_C);

    </code>

  Remarks:
    None.
*/

uint32_t PORT_GroupRead(PORT_GROUP group);

// *****************************************************************************
/* Function:
    uint32_t PORT_GroupLatchRead(PORT_GROUP group)

  Summary:
    Read the data driven on all the I/O pins of the selected port group.

  Description:
    The function will return a 32-bit value representing the logic levels being
    driven on the output pins within the group. The function will not sample the
    actual hardware state of the output pin. Each bit in the 32-bit return value
    will represent one of the 32 port pins within the group. The application
    should only consider the value of the pins which are available on the
    device.

  Precondition:
    The PORT_Initialize() function should have been called.

  Parameters:
    group - One of the IO groups from the enum PORT_GROUP.

  Returns:
    A 32-bit value representing the output state of of all the I/O pins in the
    selected port group.

  Example:
    <code>

    uint32_t value;
    value = PORT_GroupLatchRead(PORT_GROUP_C);

    </code>

  Remarks:
    None.
*/

uint32_t PORT_GroupLatchRead(PORT_GROUP group);

// *****************************************************************************
/* Function:
    void PORT_GroupWrite(PORT_GROUP group, uint32_t mask, uint32_t value);

  Summary:
    Write value on the masked pins of the selected port group.

  Description:
    This function writes the value contained in the value parameter to the
    port group. Port group pins which are configured for output will be updated.
    The mask parameter provides additional control on the bits in the group to
    be affected. Setting a bit to 1 in the mask will cause the corresponding
    bit in the port group to be updated. Clearing a bit in the mask will cause
    that corresponding bit in the group to stay unaffected. For example,
    setting a mask value 0xFFFFFFFF will cause all bits in the port group
    to be updated. Setting a value 0x3 will only cause port group bit 0 and
    bit 1 to be updated.

    For port pins which are not configured for output and have the pull feature
    enabled, this function will affect pull value (pull up or pull down). A bit
    value of 1 will enable the pull up. A bit value of 0 will enable the pull
    down.

  Precondition:
    The PORT_Initialize() function should have been called.

  Parameters:
    group - One of the IO groups from the enum PORT_GROUP.

    mask  - A 32 bit value in which positions of 0s and 1s decide
             which IO pins of the selected port group will be written.
             1's - Will write to corresponding IO pins.
             0's - Will remain unchanged.

    value - Value which has to be written/driven on the I/O
             lines of the selected port for which mask bits are '1'.
             Values for the corresponding mask bit '0' will be ignored.
             Refer to the function description for effect on pins
             which are not configured for output.

  Returns:
    None.

  Example:
    <code>

    PORT_GroupWrite(PORT_GROUP_C, 0x0F, 0xF563D453);

    </code>

  Remarks:
    None.
*/

void PORT_GroupWrite(PORT_GROUP group, uint32_t mask, uint32_t value);

// *****************************************************************************
/* Function:
    void PORT_GroupSet(PORT_GROUP group, uint32_t mask)

  Summary:
    Set the selected IO pins of a group.

  Description:
    This function sets (drives a logic high) on the selected output pins of a
    group. The mask parameter control the pins to be updated. A mask bit
    position with a value 1 will cause that corresponding port pin to be set. A
    mask bit position with a value 0 will cause the corresponding port pin to
    stay un-affected.

  Precondition:
    The PORT_Initialize() function should have been called.

  Parameters:
    group - One of the IO ports from the enum PORT_GROUP.
    mask - A 32 bit value in which a bit represent a pin in the group. If the
    value of the bit is 1, the corresponding port pin will driven to logic 1. If
    the value of the bit is 0. the corresponding port pin will stay un-affected.

  Returns:
    None.

  Example:
    <code>

    PORT_GroupSet(PORT_GROUP_C, 0x00A0);

    </code>

  Remarks:
    If the port pin within the the group is not configured for output and has
    the pull feature enabled, driving a logic 1 on this pin will cause the pull
    up to be enabled.
*/

void PORT_GroupSet(PORT_GROUP group, uint32_t mask);

// *****************************************************************************
/* Function:
    void PORT_GroupClear(PORT_GROUP group, uint32_t mask)

  Summary:
    Clears the selected IO pins of a group.

  Description:
    This function clears (drives a logic 0) on the selected output pins of a
    group. The mask parameter control the pins to be updated. A mask bit
    position with a value 1 will cause that corresponding port pin to be clear.
    A mask bit position with a value 0 will cause the corresponding port pin to
    stay un-affected.

  Precondition:
    The PORT_Initialize() function should have been called.

  Parameters:
    group - One of the IO ports from the enum PORT_GROUP.
    mask - A 32 bit value in which a bit represent a pin in the group. If the
    value of the bit is 1, the corresponding port pin will driven to logic 0. If
    the value of the bit is 0. the corresponding port pin will stay un-affected.

  Returns:
    None.

  Example:
    <code>

    PORT_GroupClear(PORT_GROUP_C, 0x00A0);

    </code>

  Remarks:
    If the port pin within the the group is not configured for output and has
    the pull feature enabled, driving a logic 0 on this pin will cause the pull
    down to be enabled.
*/

void PORT_GroupClear(PORT_GROUP group, uint32_t mask);

// *****************************************************************************
/* Function:
    void PORT_GroupToggle(PORT_GROUP group, uint32_t mask)

  Summary:
    Toggles the selected IO pins of a group.

  Description:
    This function toggles the selected output pins of a group. The mask
    parameter control the pins to be updated. A mask bit position with a value 1
    will cause that corresponding port pin to be toggled.  A mask bit position
    with a value 0 will cause the corresponding port pin to stay un-affected.

  Precondition:
    The PORT_Initialize() function should have been called.

  Parameters:
    group - One of the IO ports from the enum PORT_GROUP.
    mask - A 32 bit value in which a bit represent a pin in the group. If the
    value of the bit is 1, the corresponding port pin will be toggled. If the
    value of the bit is 0. the corresponding port pin will stay un-affected.

  Returns:
    None.

  Example:
    <code>

    PORT_GroupToggle(PORT_GROUP_C, 0x00A0);

    </code>

  Remarks:
    If the port pin within the the group is not configured for output and has
    the pull feature enabled, driving a logic 0 on this pin will cause the pull
    down to be enabled. Driving a logic 1 on this pin will cause the pull up to
    be enabled.
*/

void PORT_GroupToggle(PORT_GROUP group, uint32_t mask);

// *****************************************************************************
/* Function:
    void PORT_GroupInputEnable(PORT_GROUP group, uint32_t mask)

  Summary:
    Configures the selected IO pins of a group as input.

  Description:
    This function configures the selected IO pins of a group as input. The pins
    to be configured as input are selected by setting the corresponding bits in
    the mask parameter to 1.

  Precondition:
    The PORT_Initialize() function should have been called.

  Parameters:
    group - One or more of the of the IO ports from the enum PORT_GROUP.
    mask - A 32 bit value in which a bit represents a pin in the group. If the
    value of the bit is 1, the corresponding port pin will be configured as
    input. If the value of the bit is 0. the corresponding port pin will stay
    un-affected.

  Returns:
    None.

  Example:
    <code>

    PORT_GroupInputEnable(PORT_GROUP_C, 0x00A0);

    </code>

  Remarks:
   None.
*/

void PORT_GroupInputEnable(PORT_GROUP group, uint32_t mask);

// *****************************************************************************
/* Function:
    void PORT_GroupOutputEnable(PORT_GROUP group, uint32_t mask)

  Summary:
    Configures the selected IO pins of a group as output.

  Description:
    This function configures the selected IO pins of a group as output. The pins
    to be configured as output are selected by setting the corresponding bits in
    the mask parameter to 1.

  Precondition:
    The PORT_Initialize() function should have been called.

  Parameters:
    group - One or more of the of the IO ports from the enum PORT_GROUP.
    mask - A 32 bit value in which a bit represents a pin in the group. If the
    value of the bit is 1, the corresponding port pin will be configured as
    output. If the value of the bit is 0. the corresponding port pin will stay
    un-affected.

  Returns:
    None.

  Example:
    <code>

    PORT_GroupOutputEnable(PORT_GROUP_C, 0x00A0);

    </code>

  Remarks:
    None.
*/

void PORT_GroupOutputEnable(PORT_GROUP group, uint32_t mask);

// *****************************************************************************
/* Function:
    void PORT_PinPeripheralFunctionConfig(PORT_PIN pin, PERIPHERAL_FUNCTION function)

  Summary:
    Configures the peripheral function on the selected port pin

  Description:
    This function configures the selected peripheral function on the given port pin.

  Remarks:
    None
*/
void PORT_PinPeripheralFunctionConfig(PORT_PIN pin, PERIPHERAL_FUNCTION function);

// *****************************************************************************
/* Function:
    void PORT_PinGPIOConfig(PORT_PIN pin)

  Summary:
    Configures the selected pin as GPIO

  Description:
    This function configures the given pin as GPIO.

  Remarks:
    None
*/
void PORT_PinGPIOConfig(PORT_PIN pin);

// *****************************************************************************
// *****************************************************************************
// Section: PORT APIs which operates on one pin at a time
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Function:
    void PORT_PinWrite(PORT_PIN pin, bool value)

  Summary:
    Writes the specified value to the selected pin.

  Description:
    This function writes/drives the "value" on the selected I/O line/pin.

  Precondition:
    The PORT_Initialize() function should have been called once.

  Parameters:
    pin - One of the IO pins from the enum PORT_PIN.
    value - value to be written on the selected pin.
            true  = set pin to high (1).
            false = clear pin to low (0).

  Returns:
    None.

  Example:
    <code>

    bool value = true;
    PORT_PinWrite(PORT_PIN_PB3, value);

    </code>

  Remarks:
    Calling this function with an input pin with the pull-up/pull-down feature
    enabled will affect the pull-up/pull-down configuration. If the value is
    false, the pull-down will be enabled. If the value is true, the pull-up will
    be enabled.
*/

static inline void PORT_PinWrite(PORT_PIN pin, bool value)
{
    PORT_GroupWrite(GET_PORT_GROUP(pin),
                    GET_PIN_MASK(pin),
                    (value ? GET_PIN_MASK(pin) : 0U));
}


// *****************************************************************************
/* Function:
    bool PORT_PinRead(PORT_PIN pin)

  Summary:
    Read the selected pin value.

  Description:
    This function reads the present state at the selected input pin.  The
    function can also be called to read the value of an output pin if input
    sampling on the output pin is enabled in MHC. If input synchronization on
    the pin is disabled in MHC, the function will cause a 2 PORT Clock cycles
    delay. Enabling the synchronization eliminates the delay but will increase
    power consumption.

  Precondition:
    The PORT_Initialize() function should have been called. Input buffer
    (INEN bit in the Pin Configuration register) should be enabled in MHC.

  Parameters:
    pin - the port pin whose state needs to be read.

  Returns:
    true - the state at the pin is a logic high.
    false - the state at the pin is a logic low.

  Example:
    <code>

    bool value;
    value = PORT_PinRead(PORT_PIN_PB3);

    </code>

  Remarks:
    None.
*/

static inline bool PORT_PinRead(PORT_PIN pin)
{
    return ((PORT_GroupRead(GET_PORT_GROUP(pin)) & GET_PIN_MASK(pin)) != 0U);
}


// *****************************************************************************
/* Function:
    bool PORT_PinLatchRead(PORT_PIN pin)

  Summary:
    Read the value driven on the selected pin.

  Description:
    This function reads the data driven on the selected I/O line/pin. The
    function does not sample the state of the hardware pin. It only returns the
    value that is written to output register. Refer to the PORT_PinRead()
    function if the state of the output pin needs to be read.

  Precondition:
    The PORT_Initialize() function should have been called.

  Parameters:
    pin - One of the IO pins from the enum PORT_PIN.

  Returns:
    true - the present value in the output latch is a logic high.
    false - the present value in the output latch is a logic low.

  Example:
    <code>

    bool value;
    value = PORT_PinLatchRead(PORT_PIN_PB3);

    </code>

  Remarks:
    To read actual pin value, PIN_Read API should be used.
*/

static inline bool PORT_PinLatchRead(PORT_PIN pin)
{
    return ((PORT_GroupLatchRead(GET_PORT_GROUP(pin)) & GET_PIN_MASK(pin)) != 0U);
}


// *****************************************************************************
/* Function:
    void PORT_PinToggle(PORT_PIN pin)

  Summary:
    Toggles the selected pin.

  Description:
    This function toggles/inverts the present value on the selected I/O line/pin.

  Precondition:
    The PORT_Initialize() function should have been called.

  Parameters:
    pin - One of the IO pins from the enum PORT_PIN.

  Returns:
    None.

  Example:
    <code>

    PORT_PinToggle(PORT_PIN_PB3);

    </code>

  Remarks:
    None.
*/

static inline void PORT_PinToggle(PORT_PIN pin)
{
    PORT_GroupToggle(GET_PORT_GROUP(pin), GET_PIN_MASK(pin));
}


// *****************************************************************************
/* Function:
    void PORT_PinSet(PORT_PIN pin)

  Summary:
    Sets the selected pin.

  Description:
    This function drives a logic 1 on the selected I/O line/pin.

  Precondition:
    The PORT_Initialize() function should have been called.

  Parameters:
    pin - One of the IO pins from the enum PORT_PIN.

  Returns:
    None.

  Example:
    <code>

    PORT_PinSet(PORT_PIN_PB3);

    </code>

  Remarks:
    None.
*/

static inline void PORT_PinSet(PORT_PIN pin)
{
    PORT_GroupSet(GET_PORT_GROUP(pin), GET_PIN_MASK(pin));
}


// *****************************************************************************
/* Function:
    void PORT_PinClear(PORT_PIN pin)

  Summary:
    Clears the selected pin.

  Description:
    This function drives a logic 0 on the selected I/O line/pin.

  Precondition:
    The PORT_Initialize() function should have been called.

  Parameters:
    pin - One of the IO pins from the enum PORT_PIN.

  Returns:
    None.

  Example:
    <code>

    PORT_PinClear(PORT_PIN_PB3);

    </code>

  Remarks:
    None.
*/

static inline void PORT_PinClear(PORT_PIN pin)
{
    PORT_GroupClear(GET_PORT_GROUP(pin), GET_PIN_MASK(pin));
}


// *****************************************************************************
/* Function:
    void PORT_PinInputEnable(PORT_PIN pin)

  Summary:
    Configures the selected IO pin as input.

  Description:
    This function configures the selected IO pin as input. This function
    override the MHC input output pin settings.

  Precondition:
    The PORT_Initialize() function should have been called.

  Parameters:
    pin - One of the IO pins from the enum PORT_PIN.

  Returns:
    None.

  Example:
    <code>

    PORT_PinInputEnable(PORT_PIN_PB3);

    </code>

  Remarks:
    None.
*/

static inline void PORT_PinInputEnable(PORT_PIN pin)
{
    PORT_GroupInputEnable(GET_PORT_GROUP(pin), GET_PIN_MASK(pin));
}


// *****************************************************************************
/* Function:
    void PORT_PinOutputEnable(PORT_PIN pin)

  Summary:
    Enables selected IO pin as output.

  Description:
    This function enables selected IO pin as output. Calling this function will
    override the MHC input output pin configuration.

  Precondition:
    The PORT_Initialize() function should have been called.

  Parameters:
    pin - One of the IO pins from the enum PORT_PIN.

  Returns:
    None.

  Example:
    <code>

    PORT_PinOutputEnable(PORT_PIN_PB3);

    </code>

  Remarks:
    None.
*/

static inline void PORT_PinOutputEnable(PORT_PIN pin)
{
    PORT_GroupOutputEnable(GET_PORT_GROUP(pin), GET_PIN_MASK(pin));
}

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

}

#endif
// DOM-IGNORE-END
#endif // PLIB_PORT_H
