/*******************************************************************************
  Application to Can Handler

  Summary:
    Support for CAN Handler in IO Aggregator Application

  Description:
    -Implements the application
 *******************************************************************************/

/*
Copyright (C) 2023-2024, Bacancy Systems LLP, and its subsidiaries. All rights reserved.

The software and documentation is provided by Bacancy Systems LLP and its contributors
"as is" and any express, implied or statutory warranties, including, but not
limited to, the implied warranties of merchantability, fitness for a particular
purpose and non-infringement of third party intellectual property rights are
disclaimed to the fullest extent permitted by law. In no event shall microchip
or its contributors be liable for any direct, indirect, incidental, special,
exemplary, or consequential damages (including, but not limited to, procurement
of substitute goods or services; loss of use, data, or profits; or business
interruption) however caused and on any theory of liability, whether in contract,
strict liability, or tort (including negligence or otherwise) arising in any way
out of the use of the software and documentation, even if advised of the
possibility of such damage.

Except as expressly permitted hereunder and subject to the applicable license terms
for any third-party software incorporated in the software and any applicable open
source software license terms, no license or other rights, whether express or
implied, are granted under any patent or other intellectual property rights of
Bacancy Systems LLP or any third party.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include "AppCanHandler.h"

// ********** CAN Message RAM Allocation **********
uint8_t CACHE_ALIGN __attribute__((space(data), section (".can0_message_ram"))) Can0MessageRAM[CAN0_MESSAGE_RAM_CONFIG_SIZE];
uint8_t CACHE_ALIGN __attribute__((space(data), section (".can1_message_ram"))) Can1MessageRAM[CAN1_MESSAGE_RAM_CONFIG_SIZE];
uint8_t CACHE_ALIGN __attribute__((space(data), section (".can2_message_ram"))) Can2MessageRAM[CAN2_MESSAGE_RAM_CONFIG_SIZE];
uint8_t CACHE_ALIGN __attribute__((space(data), section (".can3_message_ram"))) Can3MessageRAM[CAN3_MESSAGE_RAM_CONFIG_SIZE];
uint8_t CACHE_ALIGN __attribute__((space(data), section (".can4_message_ram"))) Can4MessageRAM[CAN4_MESSAGE_RAM_CONFIG_SIZE];
uint8_t CACHE_ALIGN __attribute__((space(data), section (".can5_message_ram"))) Can5MessageRAM[CAN5_MESSAGE_RAM_CONFIG_SIZE];

uint8_t CanMessageRAM[CAN3_MESSAGE_RAM_CONFIG_SIZE];

#define CAN_TX_PAYLOAD_SIZE 8
#define CAN_TASK_DELAY 10U
static uint32_t status = 0;// ********** CAN Data Buffers **********
// ********** CAN Data Buffers **********
static uint8_t u8CAN0buffer[256];
static uint8_t u8CAN1buffer[256];
static uint8_t u8CAN2buffer[256];

// Declare the mutex handle globally
SemaphoreHandle_t xCan0RXQueueMutex;
SemaphoreHandle_t xCan0TXQueueMutex;
SemaphoreHandle_t xCan1RXQueueMutex;
SemaphoreHandle_t xCan1TXQueueMutex;
SemaphoreHandle_t xCan2RXQueueMutex;
SemaphoreHandle_t xCan2TXQueueMutex;

// ********** FreeRTOS Queue Handlers **********
QueueHandle_t xCAN0RXQueueHandler;
QueueHandle_t xCAN1RXQueueHandler;
QueueHandle_t xCAN2RXQueueHandler;
QueueHandle_t xCAN0TXQueueHandler;
QueueHandle_t xCAN1TXQueueHandler;
QueueHandle_t xCAN2TXQueueHandler;

/* ========================================================
    FUNCTION PROTOTYPES
     ============================================================ */
extern void vProcessBMSCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus);
extern void vProcessPMCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus);

/**
 * @brief Swaps the endianness of a 32-bit unsigned integer.
 *
 * This function reverses the byte order of a 32-bit value, converting 
 * between little-endian and big-endian formats.
 *
 * @param value The 32-bit unsigned integer to swap.
 * @return The 32-bit unsigned integer with reversed byte order.
 *
 * Example:
 *  Input:  0xAABBCCDD  (stored as DD CC BB AA in memory)
 *  Output: 0xDDCCBBAA  (stored as AA BB CC DD in memory)
 */
uint32_t swap_endian_32(uint32_t value) {
    return ((value >> 24) & 0x000000FF) |
           ((value >>  8) & 0x0000FF00) |
           ((value <<  8) & 0x00FF0000) |
           ((value << 24) & 0xFF000000);
}

/**
 * @brief Transmits a CAN message on the specified CAN interface.
 *
 * This function sends a CAN frame using one of the available CAN channels.
 * It extracts the 29-bit CAN ID from the first 4 bytes of the input data,
 * sets up the transmission buffer, and sends the payload.
 *
 * @param u8data   Pointer to the data buffer containing the CAN ID (4 bytes) 
 *                 followed by the payload.
 * @param i8len    Length of the data (not directly used, as payload size is fixed).
 * @return True if the message was successfully transmitted, false otherwise.
 *
 * Example Usage:
 *  uint8_t message[12] = { 0x1A, 0x2B, 0x3C, 0x4D,  // CAN ID: 0x1A2B3C4D
 *                          0x11, 0x22, 0x33, 0x44,  // Data Payload
 *                          0x55, 0x66, 0x77, 0x88 };
 *  bool status = CAN0_Write( message, 12);
 */

// Generalized CAN write function
bool CAN_Write(uint8_t canIndex, uint8_t *u8data, char i8len)
{
    static CAN_TX_BUFFER txBuffer; // Use static buffer to avoid excessive stack usage
    memset(&txBuffer, 0, sizeof(CAN_TX_BUFFER));

    // Extract 29-bit CAN ID from the first 4 bytes
    uint32_t canId = ((uint32_t)u8data[0] << 24) |
                     ((uint32_t)u8data[1] << 16) |
                     ((uint32_t)u8data[2] << 8)  |
                     ((uint32_t)u8data[3]);

    txBuffer.id  = canId & 0x1FFFFFFF;  // 29-bit Extended ID
    txBuffer.xtd = 1;                  // Mark as extended frame
    txBuffer.dlc = CAN_TX_PAYLOAD_SIZE;

    SYS_CONSOLE_PRINT("Writing Data on CAN%d, ID: %08X, Payload: ", canIndex, canId);
    for (uint8_t i = 0; i < CAN_TX_PAYLOAD_SIZE; i++)
    {
        txBuffer.data[i] = u8data[i + 4];
        SYS_CONSOLE_PRINT("%02X ", txBuffer.data[i]);
    }
    SYS_CONSOLE_PRINT("\r\n");

    switch (canIndex)
    {
        case 0: return CAN0_MessageTransmitFifo(1, &txBuffer);
        case 1: return CAN1_MessageTransmitFifo(1, &txBuffer);
        case 2: return CAN2_MessageTransmitFifo(1, &txBuffer);
        // case 3: return CAN3_MessageTransmitFifo(1, &txBuffer);
        // case 4: return CAN4_MessageTransmitFifo(1, &txBuffer);
        // case 5: return CAN5_MessageTransmitFifo(1, &txBuffer);
        default:
            SYS_CONSOLE_PRINT("Invalid CAN index.\r\n");
            return false;
    }
}
bool CAN0_Write(const CAN_TX_BUFFER *const tx_buffer)
{
    if (tx_buffer == NULL)
    {
        SYS_CONSOLE_PRINT("CAN0: NULL TX buffer\r\n");
        return false;
    }

    // Validate DLC
    if (tx_buffer->dlc > 8)
    {
        SYS_CONSOLE_PRINT("CAN0: Invalid DLC\r\n");
        return false;
    }

    // Try transmit
    if (!CAN0_MessageTransmitFifo(1U, tx_buffer))
    {
        SYS_CONSOLE_PRINT("CAN0: TX failed\r\n");
        return false;
    }

    return true;
}
bool CAN1_Write(const CAN_TX_BUFFER *const tx_buffer)
{
    if (tx_buffer == NULL)
    {
        SYS_CONSOLE_PRINT("CAN1: NULL TX buffer\r\n");
        return false;
    }

    // Validate DLC
    if (tx_buffer->dlc > 8)
    {
        SYS_CONSOLE_PRINT("CAN1: Invalid DLC\r\n");
        return false;
    }

    // Attempt transmit using FIFO 0
    if (!CAN1_MessageTransmitFifo(0, tx_buffer))
    {
        SYS_CONSOLE_PRINT("CAN1: TX failed\r\n");
        return false;
    }

    return true;
}

bool CAN2_Write(const CAN_TX_BUFFER *const tx_buffer)
{
    if (tx_buffer == NULL)
    {
        SYS_CONSOLE_PRINT("CAN2: NULL TX buffer\r\n");
        return false;
    }

    // Validate DLC
    if (tx_buffer->dlc > 8)
    {
        SYS_CONSOLE_PRINT("CAN2: Invalid DLC\r\n");
        return false;
    }

    // Attempt transmit using FIFO 0
    if (!CAN2_MessageTransmitFifo(0, tx_buffer))
    {
        SYS_CONSOLE_PRINT("CAN2: TX failed\r\n");
        return false;
    }

    return true;
}
void vProcessCanRxMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus)
{
    if (rxBuf == NULL)
    {
        SYS_CONSOLE_PRINT("Received NULL CAN RX buffer\r\n");
        return;
    }
    if (rxBuf->xtd)
    {
        vProcessPMCanMessage(rxBuf, canBus);
    }
    else
    {
        vProcessBMSCanMessage(rxBuf, canBus);
    }
}

/**
 * @brief CAN0 Server Task
 * 
 * This FreeRTOS task handles TCP/UDP communication for CAN0.
 * It manages incoming connections, reads data from the socket,
 * transmits it to the CAN bus, and processes CAN queue messages
 * to send over the network. The task also ensures reconnection
 * if the socket is closed or disconnected.
 * 
 * @param[in] pvParameters Pointer to task parameters (unused).
 */

void vCan0HandlerServerTask(void *pvParameters)
{
    (void)pvParameters; // Unused parameter
    SYS_CONSOLE_PRINT("In Function: %s\r\n", __FUNCTION__);
    while (true)
    {
        CAN_RX_BUFFER canRxBuffer = {0};
        CAN_TX_BUFFER canTxBuffer = {0};

        // -----------------------------
        // Handle CAN TX Queue
        // -----------------------------
        if (xSemaphoreTake(xCan0TXQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            if (xQueueReceive(xCAN0TXQueueHandler, &canTxBuffer, 0) == pdPASS)
            {
                if (!CAN0_Write(&canTxBuffer))
                {
                    SYS_CONSOLE_PRINT("CAN0: Write failed\r\n");
                }
            }

            xSemaphoreGive(xCan0TXQueueMutex);
        }

        // -----------------------------
        // 2. Handle CAN RX Queue
        // -----------------------------
        if (xSemaphoreTake(xCan0RXQueueMutex, pdMS_TO_TICKS(10)) == pdPASS)
        {
            if (xQueueReceive(xCAN0RXQueueHandler, &canRxBuffer, 0) == pdPASS)
            {
                vProcessCanRxMessage(&canRxBuffer, CANBUS_0); // Process received CAN message for CAN0
            }
            xSemaphoreGive(xCan0RXQueueMutex);
        }

        // -----------------------------
        // 3. Small delay to yield CPU
        // -----------------------------
        vTaskDelay(pdMS_TO_TICKS(CAN_TASK_DELAY));
    }
}

void vCan1HandlerServerTask(void *pvParameters)
{
    (void)pvParameters; // Unused parameter
    SYS_CONSOLE_PRINT("In Function: %s\r\n", __FUNCTION__);

    while (true)
    {
        CAN_RX_BUFFER canRxBuffer = {0};
        CAN_TX_BUFFER canTxBuffer = {0};

        // -----------------------------
        // Handle CAN TX Queue
        // -----------------------------
        if (xSemaphoreTake(xCan1TXQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            if (xQueueReceive(xCAN1TXQueueHandler, &canTxBuffer, 0) == pdPASS)
            {
                if (!CAN1_Write(&canTxBuffer))
                {
                    SYS_CONSOLE_PRINT("CAN1: Write failed\r\n");
                }
            }

            xSemaphoreGive(xCan1TXQueueMutex);
        }

        // -----------------------------
        // Handle CAN RX Queue
        // -----------------------------
        if (xSemaphoreTake(xCan1RXQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            if (xQueueReceive(xCAN1RXQueueHandler, &canRxBuffer, 0) == pdPASS)
            {
                vProcessCanRxMessage(&canRxBuffer, CANBUS_1); // Process received CAN message for CAN1
            }

            xSemaphoreGive(xCan1RXQueueMutex);
        }

        // -----------------------------
        // Small delay to yield CPU
        // -----------------------------
        vTaskDelay(pdMS_TO_TICKS(CAN_TASK_DELAY));
    }
}

void vCan2HandlerServerTask(void *pvParameters)
{
    (void)pvParameters; // Unused parameter
    SYS_CONSOLE_PRINT("In Function: %s\r\n", __FUNCTION__);

    while (true)
    {
        CAN_RX_BUFFER canRxBuffer = {0};
        CAN_TX_BUFFER canTxBuffer = {0};

        // -----------------------------
        // Handle CAN TX Queue
        // -----------------------------
        if (xSemaphoreTake(xCan2TXQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            if (xQueueReceive(xCAN2TXQueueHandler, &canTxBuffer, 0) == pdPASS)
            {
                if (!CAN2_Write(&canTxBuffer))
                {
                    SYS_CONSOLE_PRINT("CAN2: Write failed\r\n");
                }
            }

            xSemaphoreGive(xCan2TXQueueMutex);
        }

        // -----------------------------
        // Handle CAN RX Queue
        // -----------------------------
        if (xSemaphoreTake(xCan2RXQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            if (xQueueReceive(xCAN2RXQueueHandler, &canRxBuffer, 0) == pdPASS)
            {
                vProcessCanRxMessage(&canRxBuffer, CANBUS_2); // Process received CAN message for CAN2
            }

            xSemaphoreGive(xCan2RXQueueMutex);
        }

        // -----------------------------
        // Small delay to yield CPU
        // -----------------------------
        vTaskDelay(pdMS_TO_TICKS(CAN_TASK_DELAY));
    }
}
static void vDisplayCanRxMessage(CAN_RX_BUFFER *rxBuf, uint8_t rxBufLen)
{
    uint8_t length = 0;
    uint8_t msgLength = 0;
    uint32_t id = 0;
    id = rxBuf->xtd ? rxBuf->id : READ_ID(rxBuf->id);
    msgLength = rxBuf->dlc;
    length = msgLength;
    SYS_CONSOLE_PRINT(" Message - ID : 0x%x Length : 0x%x ", (unsigned int)id, (unsigned int)msgLength);
    SYS_CONSOLE_PRINT("Message : ");
    while (length)
    {
        SYS_CONSOLE_PRINT("0x%x ", rxBuf->data[msgLength - length--]);
    }
    SYS_CONSOLE_PRINT("\r\n");
}
static void vDisplayCanErrorStatus(uint32_t status, const char *canName)
{
    SYS_CONSOLE_PRINT("%s status: 0x%08lX\r\n", canName, (unsigned long)status);

    switch (status & CAN_PSR_LEC_Msk)
    {
        case 1: SYS_CONSOLE_PRINT("%s: Stuff error\n", canName); break;
        case 2: SYS_CONSOLE_PRINT("%s: Form error\n", canName); break;
        case 3: SYS_CONSOLE_PRINT("%s: ACK error\n", canName); break;
        case 4: SYS_CONSOLE_PRINT("%s: Bit-1 error\n", canName); break;
        case 5: SYS_CONSOLE_PRINT("%s: Bit-0 error\n", canName); break;
        case 6: SYS_CONSOLE_PRINT("%s: CRC error\n", canName); break;
        case 7: SYS_CONSOLE_PRINT("%s: LEC unchanged\n", canName); break;
        default: break;
    }

    if (status & CAN_PSR_BO_Msk)
    {
        SYS_CONSOLE_PRINT("%s: Bus-Off state detected\r\n", canName);
    }
}

static inline bool CAN_IsRxOK(uint32_t status)
{
    uint32_t lec = status & CAN_PSR_LEC_Msk;
    return (lec == CAN_ERROR_NONE) || (lec == CAN_ERROR_LEC_NC);
}

void vCanRxHandlerTask(void *pvParameters)
{
    while (true)
    {
        if (CAN0_InterruptGet(CAN_INTERRUPT_RF0N_MASK))
        {
            CAN0_InterruptClear(CAN_INTERRUPT_RF0N_MASK);

            uint32_t status = CAN0_ErrorGet();
            vDisplayCanErrorStatus(status, "CAN0");

            if (CAN_IsRxOK(status))
            {
                uint8_t u8NumberOfMessage = CAN0_RxFifoFillLevelGet(CAN_RX_FIFO_0);

                if (u8NumberOfMessage > 0)
                {
                    if (u8NumberOfMessage > CAN0_RX_FIFO0_SIZE)
                    {
                        SYS_CONSOLE_PRINT("CAN0 RX FIFO overflow!\r\n");
                        u8NumberOfMessage = CAN0_RX_FIFO0_SIZE;
                    }

                    CAN_RX_BUFFER canRxBuffer[CAN0_RX_FIFO0_SIZE] = {0};

                    if (CAN0_MessageReceiveFifo(CAN_RX_FIFO_0, u8NumberOfMessage, canRxBuffer))
                    {
                        for (uint8_t u8Count = 0; u8Count < u8NumberOfMessage; u8Count++)
                        {
                            vDisplayCanRxMessage(&canRxBuffer[u8Count], CAN0_RX_FIFO0_ELEMENT_SIZE);

                            if (xSemaphoreTake(xCan0RXQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                            {
                                if (xQueueSend(xCAN0RXQueueHandler, &canRxBuffer[u8Count], 0) != pdPASS)
                                {
                                    SYS_CONSOLE_PRINT("CAN0 queue full\r\n");
                                }
                                xSemaphoreGive(xCan0RXQueueMutex);
                            }
                            else
                            {
                                SYS_CONSOLE_PRINT("CAN0 mutex timeout\r\n");
                            }
                        }
                    }
                    else
                    {
                        SYS_CONSOLE_PRINT("Error receiving CAN0 message\r\n");
                    }
                }
            }
        }

        if (CAN1_InterruptGet(CAN_INTERRUPT_RF0N_MASK))
        {
            CAN1_InterruptClear(CAN_INTERRUPT_RF0N_MASK);

            uint32_t status = CAN1_ErrorGet();
            vDisplayCanErrorStatus(status, "CAN1");

            if (CAN_IsRxOK(status))
            {
              uint8_t u8NumberOfMessage = CAN1_RxFifoFillLevelGet(CAN_RX_FIFO_0);

                if (u8NumberOfMessage > 0)
                {
                    if (u8NumberOfMessage > CAN1_RX_FIFO0_SIZE)
                    {
                        SYS_CONSOLE_PRINT("CAN1 RX FIFO overflow!\r\n");
                        u8NumberOfMessage = CAN1_RX_FIFO0_SIZE;
                    }

                    CAN_RX_BUFFER canRxBuffer[CAN1_RX_FIFO0_SIZE] = {0};

                    if (CAN1_MessageReceiveFifo(CAN_RX_FIFO_0, u8NumberOfMessage, canRxBuffer))
                    {
                        for (uint8_t i = 0; i < u8NumberOfMessage; i++)
                        {
                            vDisplayCanRxMessage(&canRxBuffer[i], CAN1_RX_FIFO0_ELEMENT_SIZE);

                            if (xSemaphoreTake(xCan1RXQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                            {
                                if (xQueueSend(xCAN1RXQueueHandler, &canRxBuffer[i], 0) != pdPASS)
                                {
                                    SYS_CONSOLE_PRINT("CAN1 queue full\r\n");
                                }
                                xSemaphoreGive(xCan1RXQueueMutex);
                            }
                            else
                            {
                                SYS_CONSOLE_PRINT("CAN1 mutex timeout\r\n");
                            }
                        }
                    }
                    else
                    {
                        SYS_CONSOLE_PRINT("Error receiving CAN1 message\r\n");
                    }
                }
            }
        }

        if (CAN2_InterruptGet(CAN_INTERRUPT_RF0N_MASK))
        {
            CAN2_InterruptClear(CAN_INTERRUPT_RF0N_MASK);

            uint32_t status = CAN2_ErrorGet();
            vDisplayCanErrorStatus(status, "CAN2");

            if (CAN_IsRxOK(status))
            {
               uint8_t u8NumberOfMessage = CAN2_RxFifoFillLevelGet(CAN_RX_FIFO_0);

                if (u8NumberOfMessage > 0)
                {
                    if (u8NumberOfMessage > CAN2_RX_FIFO0_SIZE)
                    {
                        SYS_CONSOLE_PRINT("CAN2 RX FIFO overflow!\r\n");
                        u8NumberOfMessage = CAN2_RX_FIFO0_SIZE;
                    }

                    CAN_RX_BUFFER canRxBuffer[CAN2_RX_FIFO0_SIZE] = {0};

                    if (CAN2_MessageReceiveFifo(CAN_RX_FIFO_0, u8NumberOfMessage, canRxBuffer))
                    {
                        for (uint8_t i = 0; i < u8NumberOfMessage; i++)
                        {
                            vDisplayCanRxMessage(&canRxBuffer[i], CAN2_RX_FIFO0_ELEMENT_SIZE);

                            if (xSemaphoreTake(xCan2RXQueueMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                            {
                                if (xQueueSend(xCAN2RXQueueHandler, &canRxBuffer[i], 0) != pdPASS)
                                {
                                    SYS_CONSOLE_PRINT("CAN2 queue full\r\n");
                                }
                                xSemaphoreGive(xCan2RXQueueMutex);
                            }
                            else
                            {
                                SYS_CONSOLE_PRINT("CAN2 mutex timeout\r\n");
                            }
                        }
                    }
                    else
                    {
                        SYS_CONSOLE_PRINT("Error receiving CAN2 message\r\n");
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
/**
 * @brief Initializes CAN handlers and message RAM configuration.
 * 
 * This function configures message RAM for all CAN instances and initializes
 * FreeRTOS queues for CAN message handling. It also creates the required 
 * FreeRTOS tasks for CAN reception and TCP/UDP server handling.
 */
void vCanHandlerInit(void)
{
    /* Set Message RAM Configuration */   
    CAN0_MessageRAMConfigSet(Can0MessageRAM);
    CAN1_MessageRAMConfigSet(Can1MessageRAM);
    CAN2_MessageRAMConfigSet(Can2MessageRAM);

    /* Create CAN Queues */
    xCAN0RXQueueHandler = xQueueCreate(CAN_QUEUE_SIZE, CAN_QUEUE_ITEM_SIZE);
    xCAN0TXQueueHandler = xQueueCreate(CAN_QUEUE_SIZE, CAN_QUEUE_ITEM_SIZE);
    xCAN1RXQueueHandler = xQueueCreate(CAN_QUEUE_SIZE, CAN_QUEUE_ITEM_SIZE);
    xCAN1TXQueueHandler = xQueueCreate(CAN_QUEUE_SIZE, CAN_QUEUE_ITEM_SIZE);
    xCAN2RXQueueHandler = xQueueCreate(CAN_QUEUE_SIZE, CAN_QUEUE_ITEM_SIZE);
    xCAN2TXQueueHandler = xQueueCreate(CAN_QUEUE_SIZE, CAN_QUEUE_ITEM_SIZE);

    /* Check if all required queues were created successfully */
    if (xCAN0RXQueueHandler == NULL || xCAN0TXQueueHandler == NULL || 
        xCAN1RXQueueHandler == NULL || xCAN1TXQueueHandler == NULL || 
        xCAN2RXQueueHandler == NULL || xCAN2TXQueueHandler == NULL) 
    {
        SYS_CONSOLE_PRINT("Error: CANRX Queue creation failed\r\n");
        return;
    }
    SYS_CONSOLE_PRINT("CANRX Queue creation successful\r\n"); 
    xCan0RXQueueMutex = xSemaphoreCreateMutex();
    xCan0TXQueueMutex = xSemaphoreCreateMutex();
    xCan1RXQueueMutex = xSemaphoreCreateMutex();
    xCan1TXQueueMutex = xSemaphoreCreateMutex();
    xCan2RXQueueMutex = xSemaphoreCreateMutex();
    xCan2TXQueueMutex = xSemaphoreCreateMutex();

    if (xCan0RXQueueMutex == NULL || xCan0TXQueueMutex == NULL ||
        xCan1RXQueueMutex == NULL || xCan1TXQueueMutex == NULL ||
        xCan2RXQueueMutex == NULL || xCan2TXQueueMutex == NULL)
    {
        // Handle error: At least one mutex creation failed
         SYS_CONSOLE_PRINT("Error: Mutex creation failed\r\n");
    }

    /* Create Tasks */
    xTaskCreate(vCanRxHandlerTask, "vCanRxHandlerTask", CAN_RX_HANDLER_HEAP_DEPTH, NULL, CAN_RX_HANDLER_TASK_PRIORITY, NULL);
    xTaskCreate(vCan0HandlerServerTask, "vCan0HandlerServerTask", CAN_SERVER_HANDLER_HEAP_DEPTH, NULL, CAN_SERVER_HANDLER_TASK_PRIORITY, NULL);
    xTaskCreate(vCan1HandlerServerTask, "vCan1HandlerServerTask", CAN_SERVER_HANDLER_HEAP_DEPTH, NULL, CAN_SERVER_HANDLER_TASK_PRIORITY, NULL);
    xTaskCreate(vCan2HandlerServerTask, "vCan2HandlerServerTask", CAN_SERVER_HANDLER_HEAP_DEPTH, NULL, CAN_SERVER_HANDLER_TASK_PRIORITY, NULL);
}
void vSendCanTxMsgToQueue(const CAN_TX_BUFFER *const pCanTxBuffer, uint8_t u8DockNo)
{
    if (pCanTxBuffer == NULL)
    {
        SYS_CONSOLE_PRINT("vSendCanTxMsgToQueue: NULL CAN TX buffer\r\n");
        return;
    }
    QueueHandle_t tx_queue = NULL;
    SemaphoreHandle_t tx_mutex = NULL;

    switch (u8DockNo)
    {
    case DOCK_1:
        tx_queue = xCAN0TXQueueHandler;
        tx_mutex = xCan0TXQueueMutex;
        break;
    case DOCK_2:
        tx_queue = xCAN1TXQueueHandler;
        tx_mutex = xCan1TXQueueMutex;
        break;
    case DOCK_3:
        tx_queue = xCAN2TXQueueHandler;
        tx_mutex = xCan2TXQueueMutex;
        break;
    default:
        SYS_CONSOLE_PRINT("vSendCanTxMsgToQueue: Invalid dock number\r\n");
        return;
    }

    if ((tx_queue != NULL) && (tx_mutex != NULL))
    {
        if (xSemaphoreTake(tx_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            if (xQueueSend(tx_queue, pCanTxBuffer, 0) != pdPASS)
            {
                SYS_CONSOLE_PRINT("Dock %u TX queue full\r\n", u8DockNo);
            }
            xSemaphoreGive(tx_mutex);
        }
        else
        {
            SYS_CONSOLE_PRINT("Dock %u TX mutex timeout\r\n", u8DockNo);
        }
    }
}