/*
 *      Copyright (c) 2024 Bacancy System LLP.
 *      All rights reserved.
 *      Use of copyright notice does not imply publication.
 *      @author: Bacancy System LLP
 *      Date & Time: 29/03/2024 11:20
 *
 *
 *                      CONFIDENTIAL INFORMATION
 *                      ------------------------
 *      This Document contains Confidential Information or Trade Secrets,
 *      or both, which are the property of Bacancy System LLP.
 *      This document may not be copied, reproduced, reduced to any
 *      electronic medium or machine readable form or otherwise
 *      duplicated and the information herein may not be used,
 *      disseminated or otherwise disclosed, except with the prior
 *      written consent of Bacancy System LLP.
 *
 */
 /*!
 * \file : io_expander.c
 * \brief : includes functions to perform io_expander functionalities
 */

/***********************************************************************
 * Include Header Files
 **********************************************************************/

#include "app.h"
#include "definitions.h"
#include "io_expander.h"

/**********************************************************************
* PRIVATE FUNCTION
**********************************************************************/
/* I2C callback */
void APP_I2CCallback(uintptr_t context)
{
    i2cTransferDone = true;
}
/* Write a TCA9539 register */
void TCA9539_WriteRegister(uint8_t reg, uint8_t value)
{
    i2cTransferDone = false;
    i2cTxBuf[0] = reg;
    i2cTxBuf[1] = value;
    if(SERCOM7_I2C_Write(TCA9539_I2C_ADDRESS, i2cTxBuf, 2))
    {
        SYS_CONSOLE_PRINT("write sucess\r\n");
    }

    uint32_t timeout = 100000; // Prevent infinite loop
    while (!i2cTransferDone && timeout--)
    {
//        SYS_CONSOLE_PRINT("busy\r\n");
        __NOP(); // optional small delay
    }

    if (timeout == 0)
    {
        SYS_CONSOLE_PRINT("I2C write timeout for reg 0x%02X\r\n", reg);
    }
}
/* Read a TCA9539 register */
uint8_t TCA9539_ReadRegister(uint8_t reg)
{
    i2cTransferDone = false;
    SERCOM7_I2C_WriteRead(TCA9539_I2C_ADDRESS, &reg, 1, i2cRxBuf, 1);

    uint32_t timeout = 100000; // Prevent infinite loop
    while (!i2cTransferDone && timeout--)
    {
        __NOP(); // optional small delay
    }

    if (timeout == 0)
    {
        SYS_CONSOLE_PRINT("I2C read timeout for reg 0x%02X\r\n", reg);
        return 0;
    }

    return i2cRxBuf[0];
}
/***********************************************************************
 * PUBLIC FUNCTIONS
 **********************************************************************/

bool IOExpander1_Configure_Ports(void)
{
    uint8_t wrData[3];
    bool success = true;

    wrData[0] = 0x04;//polarity inversion port0
    wrData[1] = 0x00;//configure as input port0 
    if (!SERCOM7_I2C_Write(TCA9539_I2C_ADDRESS, wrData, 2))
    {
        SYS_CONSOLE_MESSAGE("Error: I2C write failed at step 1 (0x04, 0x00)\r\n");
        success = false;
    }
    vTaskDelay(100);

    wrData[0] = 0x05;//polarity inversion port0
    wrData[1] = 0x01;//configure as input port1  
    if (!SERCOM7_I2C_Write(TCA9539_I2C_ADDRESS, wrData, 2))
    {
        SYS_CONSOLE_MESSAGE("Error: I2C write failed at step 2 (0x05, 0x01)\r\n");
        success = false;
    }
    vTaskDelay(100);

    if (success)
    {
        SYS_CONSOLE_MESSAGE("I2C port configuration done for - 0x77 - Success\r\n");
    }
    else
    {
        SYS_CONSOLE_MESSAGE("I2C port configuration failed for - 0x77\r\n");
    }

    return success;
}
/**
 * @brief:
 * @param[in]
 * @param[out]
 */
void vConfigureIOexpanders(void)
{
    /* power on set */
    IO_Exp_Reset_Set();
    if (!IOExpander1_Configure_Ports())
    {
        SYS_CONSOLE_MESSAGE("Error: IOExpander1 configuration failed!\r\n");
    }     
    SERCOM7_I2C_CallbackRegister(APP_I2CCallback, 0);

    /* Configure all pins as inputs */
    TCA9539_WriteRegister(TCA9539_CONFIG_PORT0, 0xFF);
    TCA9539_WriteRegister(TCA9539_CONFIG_PORT1, 0xFF);
}
/***********************************************************************
 * END OF FILE
 **********************************************************************/
