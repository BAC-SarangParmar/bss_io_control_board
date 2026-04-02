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
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include "definitions.h"
#include "AppCanHandler.h"

/* ================= DEBUG CONTROL ================= */

/* Set to 1 → Enable debug prints */
/* Set to 0 → Disable debug prints */
#define CAN_DEBUG_ENABLE    1

#if CAN_DEBUG_ENABLE
    #define CAN_DEBUG_EXEC(x)           do { x; } while(0)
#else
    #define CAN_DEBUG_EXEC(x)
#endif
/* ================= CONFIGURATION ================= */
#define CAN_TX_PAYLOAD_SIZE 8
#define CAN_TASK_DELAY 10U

/* ================= MESSAGE RAM ================= */
uint8_t CACHE_ALIGN __attribute__((space(data), section(".can0_message_ram"))) Can0MessageRAM[CAN0_MESSAGE_RAM_CONFIG_SIZE];
uint8_t CACHE_ALIGN __attribute__((space(data), section(".can1_message_ram"))) Can1MessageRAM[CAN1_MESSAGE_RAM_CONFIG_SIZE];
uint8_t CACHE_ALIGN __attribute__((space(data), section(".can2_message_ram"))) Can2MessageRAM[CAN2_MESSAGE_RAM_CONFIG_SIZE];

/* ================= QUEUES ================= */
QueueHandle_t xCAN0RXQueueHandler;
QueueHandle_t xCAN1RXQueueHandler;
QueueHandle_t xCAN2RXQueueHandler;

QueueHandle_t xCAN0TXQueueHandler;
QueueHandle_t xCAN1TXQueueHandler;
QueueHandle_t xCAN2TXQueueHandler;

/* ================= STATIC BUFFERS ================= */
static CAN_RX_BUFFER can0RxBuffer[CAN0_RX_FIFO0_SIZE];
static CAN_RX_BUFFER can1RxBuffer[CAN1_RX_FIFO0_SIZE];
static CAN_RX_BUFFER can2RxBuffer[CAN2_RX_FIFO0_SIZE];

/* ================= EXTERNAL ================= */
extern void vProcessBMSCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus);
extern void vProcessPMCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus);

/**
 * @brief Swap byte order of a 32-bit unsigned integer.
 *
 * This function converts a 32-bit value between little-endian and big-endian
 * formats by reversing the byte order.
 *
 * @param[in] value 32-bit input value
 *
 * @return uint32_t Byte-swapped value
 *
 * @note Useful when handling CAN data from different endianness systems.
 */
uint32_t swap_endian_32(uint32_t value)
{
    return ((value >> 24) & 0x000000FF) |
           ((value >>  8) & 0x0000FF00) |
           ((value <<  8) & 0x00FF0000) |
           ((value << 24) & 0xFF000000);
}

/* ================= CAN WRITE ================= */
/**
 * @brief Transmit CAN frame using raw byte buffer.
 *
 * This function extracts a 29-bit extended CAN ID from the first 4 bytes
 * of the input buffer and sends the remaining 8 bytes as payload.
 *
 * Expected data format:
 * ----------------------------------------
 * | Byte 0-3 | CAN ID (Big Endian)        |
 * | Byte 4-11| Payload (8 bytes max)      |
 * ----------------------------------------
 *
 * @param[in] canIndex  CAN controller index (0, 1, 2)
 * @param[in] u8data    Pointer to input buffer (min 12 bytes required)
 * @param[in] i8len     Length of input buffer
 *
 * @return true  Transmission successful
 * @return false Transmission failed / invalid input
 */
bool CAN_Write(uint8_t canIndex, uint8_t *u8data, char i8len)
{
    if (u8data == NULL || i8len < (CAN_TX_PAYLOAD_SIZE + 4))
    {
        SYS_CONSOLE_PRINT("Invalid CAN write input\r\n");
        return false;
    }

    static CAN_TX_BUFFER txBuffer;
    memset(&txBuffer, 0, sizeof(CAN_TX_BUFFER));

    /* Extract 29-bit CAN ID */
    uint32_t canId = ((uint32_t)u8data[0] << 24) |
                     ((uint32_t)u8data[1] << 16) |
                     ((uint32_t)u8data[2] << 8)  |
                     ((uint32_t)u8data[3]);

    txBuffer.id  = canId & 0x1FFFFFFF;
    txBuffer.xtd = 1;
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
        default:
            SYS_CONSOLE_PRINT("Invalid CAN index\r\n");
            return false;
    }
}

/**
 * @brief Transmit CAN message on CAN0 interface.
 *
 * This function sends a CAN frame using CAN0 transmit FIFO.
 * It validates the input buffer and DLC before transmission.
 *
 * @param[in] tx_buffer Pointer to CAN TX buffer structure
 *
 * @return true  Transmission successful
 * @return false Transmission failed / invalid input
 */
bool CAN0_Write(const CAN_TX_BUFFER *const tx_buffer)
{
    if (tx_buffer == NULL)
    {
        SYS_CONSOLE_PRINT("CAN0: NULL TX buffer\r\n");
        return false;
    }

    if (tx_buffer->dlc > CAN_TX_PAYLOAD_SIZE)
    {
        SYS_CONSOLE_PRINT("CAN0: Invalid DLC\r\n");
        return false;
    }

    if (!CAN0_MessageTransmitFifo(1U, tx_buffer))
    {
        SYS_CONSOLE_PRINT("CAN0: TX failed\r\n");
        return false;
    }

    return true;
}

/**
 * @brief Transmit CAN message on CAN1 interface.
 *
 * This function sends a CAN frame using CAN1 transmit FIFO.
 * It validates the input buffer and DLC before transmission.
 *
 * @param[in] tx_buffer Pointer to CAN TX buffer structure
 *
 * @return true  Transmission successful
 * @return false Transmission failed / invalid input
 */
bool CAN1_Write(const CAN_TX_BUFFER *const tx_buffer)
{
    if (tx_buffer == NULL)
    {
        SYS_CONSOLE_PRINT("CAN1: NULL TX buffer\r\n");
        return false;
    }

    if (tx_buffer->dlc > CAN_TX_PAYLOAD_SIZE)
    {
        SYS_CONSOLE_PRINT("CAN1: Invalid DLC\r\n");
        return false;
    }

    if (!CAN1_MessageTransmitFifo(1U, tx_buffer))
    {
        SYS_CONSOLE_PRINT("CAN1: TX failed\r\n");
        return false;
    }

    return true;
}

/**
 * @brief Transmit CAN message on CAN2 interface.
 *
 * This function sends a CAN frame using CAN2 transmit FIFO.
 * It validates the input buffer and DLC before transmission.
 *
 * @param[in] tx_buffer Pointer to CAN TX buffer structure
 *
 * @return true  Transmission successful
 * @return false Transmission failed / invalid input
 */
bool CAN2_Write(const CAN_TX_BUFFER *const tx_buffer)
{
    if (tx_buffer == NULL)
    {
        SYS_CONSOLE_PRINT("CAN2: NULL TX buffer\r\n");
        return false;
    }

    if (tx_buffer->dlc > CAN_TX_PAYLOAD_SIZE)
    {
        SYS_CONSOLE_PRINT("CAN2: Invalid DLC\r\n");
        return false;
    }

    if (!CAN2_MessageTransmitFifo(1U, tx_buffer))
    {
        SYS_CONSOLE_PRINT("CAN2: TX failed\r\n");
        return false;
    }

    return true;
}
/**
 * @brief Process received CAN message based on frame type.
 *
 * This function routes the received CAN message to the appropriate
 * processing handler based on whether the frame is extended or standard.
 *
 * @param[in] rxBuf Pointer to received CAN buffer
 * @param[in] canBus CAN bus identifier
 *
 * @note
 * - Extended ID → Power Module processing
 * - Standard ID → BMS processing
 */
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
 * @brief Initialize CAN message RAM, queues, and tasks.
 *
 * This function performs:
 * - CAN message RAM configuration
 * - FreeRTOS queue creation for RX/TX
 * - Task creation for CAN handling
 *
 * @note Must be called during system initialization.
 */
void vCanHandlerInit(void)
{
    CAN0_MessageRAMConfigSet(Can0MessageRAM);
    CAN1_MessageRAMConfigSet(Can1MessageRAM);
    CAN2_MessageRAMConfigSet(Can2MessageRAM);


    xCAN0RXQueueHandler = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CAN_RX_BUFFER));
    xCAN0TXQueueHandler = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CAN_TX_BUFFER));
    xCAN1RXQueueHandler = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CAN_RX_BUFFER));
    xCAN1TXQueueHandler = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CAN_TX_BUFFER));
    xCAN2RXQueueHandler = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CAN_RX_BUFFER));
    xCAN2TXQueueHandler = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CAN_TX_BUFFER));

    if (!xCAN0RXQueueHandler || !xCAN0TXQueueHandler ||
        !xCAN1RXQueueHandler || !xCAN1TXQueueHandler ||
        !xCAN2RXQueueHandler || !xCAN2TXQueueHandler)
    {
        SYS_CONSOLE_PRINT("Error: Queue creation failed\r\n");
        return;
    }

    SYS_CONSOLE_PRINT("Queue creation successful\r\n");

    if (xTaskCreate(vCanRxHandlerTask, "CAN_RX", CAN_RX_HANDLER_HEAP_DEPTH, NULL,
                    CAN_RX_HANDLER_TASK_PRIORITY, NULL) != pdPASS)
    {
        SYS_CONSOLE_PRINT("Error: RX Task create failed\r\n");
    }

    if (xTaskCreate(vCan0HandlerServerTask, "CAN0_SRV", CAN_SERVER_HANDLER_HEAP_DEPTH, NULL,
                    CAN_SERVER_HANDLER_TASK_PRIORITY, NULL) != pdPASS)
    {
        SYS_CONSOLE_PRINT("Error: CAN0 Task failed\r\n");
    }

    if (xTaskCreate(vCan1HandlerServerTask, "CAN1_SRV", CAN_SERVER_HANDLER_HEAP_DEPTH, NULL,
                    CAN_SERVER_HANDLER_TASK_PRIORITY, NULL) != pdPASS)
    {
        SYS_CONSOLE_PRINT("Error: CAN1 Task failed\r\n");
    }

    if (xTaskCreate(vCan2HandlerServerTask, "CAN2_SRV", CAN_SERVER_HANDLER_HEAP_DEPTH, NULL,
                    CAN_SERVER_HANDLER_TASK_PRIORITY, NULL) != pdPASS)
    {
        SYS_CONSOLE_PRINT("Error: CAN2 Task failed\r\n");
    }
}

/**
 * @brief Send CAN TX message to respective queue.
 *
 * This function routes the given CAN TX buffer to the correct
 * CAN transmit queue based on dock number.
 *
 * @param[in] pCanTxBuffer Pointer to CAN TX buffer
 * @param[in] u8DockNo     Dock identifier (DOCK_1, DOCK_2, DOCK_3)
 *
 * @note Non-blocking queue send with short timeout.
 */
void vSendCanTxMsgToQueue(const CAN_TX_BUFFER *const pCanTxBuffer, uint8_t u8DockNo)
{
    if (pCanTxBuffer == NULL)
    {
        SYS_CONSOLE_PRINT("NULL TX buffer\r\n");
        return;
    }

    QueueHandle_t tx_queue = NULL;

    switch (u8DockNo)
    {
        case DOCK_1: tx_queue = xCAN0TXQueueHandler; break;
        case DOCK_2: tx_queue = xCAN1TXQueueHandler; break;
        case DOCK_3: tx_queue = xCAN2TXQueueHandler; break;
        default:
            SYS_CONSOLE_PRINT("Invalid dock number\r\n");
            return;
    }

    if (tx_queue != NULL)
    {
        if (xQueueSend(tx_queue, pCanTxBuffer, pdMS_TO_TICKS(5)) != pdPASS)
        {
            SYS_CONSOLE_PRINT("Dock %u TX queue full\r\n", u8DockNo);
        }
    }
}
/**
 * @brief Check if CAN RX status is valid.
 *
 * This function evaluates the Last Error Code (LEC) field
 * from the CAN protocol status register to determine whether
 * the received frame is valid.
 *
 * @param[in] status CAN error/status register value
 *
 * @return true  No critical RX error
 * @return false RX error detected
 */
static inline bool CAN_IsRxOK(uint32_t status)
{
    uint32_t lec = status & CAN_PSR_LEC_Msk;

    return (lec == CAN_ERROR_NONE) || (lec == CAN_ERROR_LEC_NC);
}

/**
 * @brief Display CAN error status.
 *
 * This function decodes and prints the CAN error status
 * based on the Protocol Status Register (PSR).
 *
 * @param[in] status  CAN error/status register value
 * @param[in] canName Name of CAN module (e.g., "CAN0")
 */
static void vDisplayCanErrorStatus(uint32_t status, const char *canName)
{
    // SYS_CONSOLE_PRINT("%s status: 0x%08lX\r\n", canName, (unsigned long)status);

    switch (status & CAN_PSR_LEC_Msk)
    {
        case 1: SYS_CONSOLE_PRINT("%s: Stuff error\r\n", canName); break;
        case 2: SYS_CONSOLE_PRINT("%s: Form error\r\n", canName); break;
        case 3: SYS_CONSOLE_PRINT("%s: ACK error\r\n", canName); break;
        case 4: SYS_CONSOLE_PRINT("%s: Bit-1 error\r\n", canName); break;
        case 5: SYS_CONSOLE_PRINT("%s: Bit-0 error\r\n", canName); break;
        case 6: SYS_CONSOLE_PRINT("%s: CRC error\r\n", canName); break;
        case 7: SYS_CONSOLE_PRINT("%s: LEC unchanged\r\n", canName); break;
        default: break;
    }

    if (status & CAN_PSR_BO_Msk)
    {
        SYS_CONSOLE_PRINT("%s: Bus-Off state detected\r\n", canName);
    }
}
/**
 * @brief Display received CAN message.
 *
 * This function prints the CAN ID, DLC, and payload data
 * for debugging purposes.
 *
 * @param[in] rxBuf   Pointer to received CAN buffer
 * @param[in] rxBufLen Length of buffer (not used internally)
 */
static void vDisplayCanRxMessage(CAN_RX_BUFFER *rxBuf, uint8_t rxBufLen)
{
    if (rxBuf == NULL)
    {
        SYS_CONSOLE_PRINT("RX buffer is NULL\r\n");
        return;
    }

    uint8_t length = rxBuf->dlc;
    uint32_t id = rxBuf->xtd ? rxBuf->id : READ_ID(rxBuf->id);

    SYS_CONSOLE_PRINT("Message - ID: 0x%lx Length: %u Data: ",
                      (unsigned long)id, length);

    for (uint8_t i = 0; i < length; i++)
    {
        SYS_CONSOLE_PRINT("0x%02X ", rxBuf->data[i]);
    }

    SYS_CONSOLE_PRINT("\r\n");
}
/**
 * @brief CAN RX handler FreeRTOS task.
 *
 * This task continuously monitors CAN interrupt flags,
 * reads messages from RX FIFO, and pushes them into
 * corresponding FreeRTOS queues.
 *
 * @param[in] pvParameters Unused task parameter
 *
 * @note
 * - Handles CAN0, CAN1, CAN2
 * - Runs in polling mode with small delay
 */
void vCanRxHandlerTask(void *pvParameters)
{
    (void)pvParameters;
    SYS_CONSOLE_PRINT("CAN RX Handler Task started\r\n");
    while (true)
    {
        /* ================= CAN0 ================= */
        if (CAN0_InterruptGet(CAN_INTERRUPT_RF0N_MASK))
        {
            // SYS_CONSOLE_PRINT("CAN0 RX interrupt\r\n");
            CAN0_InterruptClear(CAN_INTERRUPT_RF0N_MASK);

            uint32_t status = CAN0_ErrorGet();

            CAN_DEBUG_EXEC(
                vDisplayCanErrorStatus(status, "CAN0");
            );

            if (CAN_IsRxOK(status))
            {
                uint8_t count = CAN0_RxFifoFillLevelGet(CAN_RX_FIFO_0);

                if (count > CAN0_RX_FIFO0_SIZE)
                {
                    SYS_CONSOLE_PRINT("CAN0 FIFO overflow\r\n");
                    count = CAN0_RX_FIFO0_SIZE;
                }

                if (count > 0 && CAN0_MessageReceiveFifo(CAN_RX_FIFO_0, count, can0RxBuffer))
                {
                    for (uint8_t i = 0; i < count; i++)
                    {
                        CAN_DEBUG_EXEC(
                            vDisplayCanRxMessage(&can0RxBuffer[i], CAN0_RX_FIFO0_ELEMENT_SIZE);
                        );

                        if (xQueueSend(xCAN0RXQueueHandler, &can0RxBuffer[i], pdMS_TO_TICKS(5)) != pdPASS)
                        {
                            SYS_CONSOLE_PRINT("CAN0 queue full\r\n");
                        }
                    }
                }
                else
                {
                    SYS_CONSOLE_PRINT("CAN0 receive failed\r\n");
                }
            }
        }

        /* ================= CAN1 ================= */
        if (CAN1_InterruptGet(CAN_INTERRUPT_RF0N_MASK))
        {
            CAN1_InterruptClear(CAN_INTERRUPT_RF0N_MASK);

            uint32_t status = CAN1_ErrorGet();

            CAN_DEBUG_EXEC(
                vDisplayCanErrorStatus(status, "CAN1");
            );

            if (CAN_IsRxOK(status))
            {
                uint8_t count = CAN1_RxFifoFillLevelGet(CAN_RX_FIFO_0);

                if (count > CAN1_RX_FIFO0_SIZE)
                {
                    SYS_CONSOLE_PRINT("CAN1 FIFO overflow\r\n");
                    count = CAN1_RX_FIFO0_SIZE;
                }

                if (count > 0 && CAN1_MessageReceiveFifo(CAN_RX_FIFO_0, count, can1RxBuffer))
                {
                    for (uint8_t i = 0; i < count; i++)
                    {
                        CAN_DEBUG_EXEC(
                            vDisplayCanRxMessage(&can1RxBuffer[i], CAN1_RX_FIFO0_ELEMENT_SIZE);
                        );

                        if (xQueueSend(xCAN1RXQueueHandler, &can1RxBuffer[i], pdMS_TO_TICKS(5)) != pdPASS)
                        {
                            SYS_CONSOLE_PRINT("CAN1 queue full\r\n");
                        }
                    }
                }
                else
                {
                    SYS_CONSOLE_PRINT("CAN1 receive failed\r\n");
                }
            }
        }

        /* ================= CAN2 ================= */
        if (CAN2_InterruptGet(CAN_INTERRUPT_RF0N_MASK))
        {
            CAN2_InterruptClear(CAN_INTERRUPT_RF0N_MASK);

            uint32_t status = CAN2_ErrorGet();

            CAN_DEBUG_EXEC(
                vDisplayCanErrorStatus(status, "CAN2");
            );

            if (CAN_IsRxOK(status))
            {
                uint8_t count = CAN2_RxFifoFillLevelGet(CAN_RX_FIFO_0);

                if (count > CAN2_RX_FIFO0_SIZE)
                {
                    SYS_CONSOLE_PRINT("CAN2 FIFO overflow\r\n");
                    count = CAN2_RX_FIFO0_SIZE;
                }

                if (count > 0 && CAN2_MessageReceiveFifo(CAN_RX_FIFO_0, count, can2RxBuffer))
                {
                    for (uint8_t i = 0; i < count; i++)
                    {
                        CAN_DEBUG_EXEC(
                            vDisplayCanRxMessage(&can2RxBuffer[i], CAN2_RX_FIFO0_ELEMENT_SIZE);
                        );

                        if (xQueueSend(xCAN2RXQueueHandler, &can2RxBuffer[i], pdMS_TO_TICKS(5)) != pdPASS)
                        {
                            SYS_CONSOLE_PRINT("CAN2 queue full\r\n");
                        }
                    }
                }
                else
                {
                    SYS_CONSOLE_PRINT("CAN2 receive failed\r\n");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
/**
 * @brief CAN0 server task for TX/RX processing.
 *
 * This task:
 * - Sends messages from CAN0 TX queue
 * - Processes messages from CAN0 RX queue
 *
 * @param[in] pvParameters Unused task parameter
 */
void vCan0HandlerServerTask(void *pvParameters)
{
    (void)pvParameters;
    CAN_RX_BUFFER rx;
    CAN_TX_BUFFER tx;

    while (true)
    {
        if (xQueueReceive(xCAN0TXQueueHandler, &tx, 0) == pdPASS)
            CAN0_Write(&tx);

        if (xQueueReceive(xCAN0RXQueueHandler, &rx, 0) == pdPASS)
            vProcessCanRxMessage(&rx, CANBUS_0);

        vTaskDelay(pdMS_TO_TICKS(CAN_TASK_DELAY));
    }
}

/**
 * @brief CAN1 server task for TX/RX processing.
 *
 * This task:
 * - Sends messages from CAN1 TX queue
 * - Processes messages from CAN1 RX queue
 *
 * @param[in] pvParameters Unused task parameter
 */
void vCan1HandlerServerTask(void *pvParameters)
{
    (void)pvParameters;
    CAN_RX_BUFFER rx;
    CAN_TX_BUFFER tx;

    while (true)
    {
        if (xQueueReceive(xCAN1TXQueueHandler, &tx, 0) == pdPASS)
            CAN1_Write(&tx);

        if (xQueueReceive(xCAN1RXQueueHandler, &rx, 0) == pdPASS)
            vProcessCanRxMessage(&rx, CANBUS_1);

        vTaskDelay(pdMS_TO_TICKS(CAN_TASK_DELAY));
    }
}
/**
 * @brief CAN2 server task for TX/RX processing.
 *
 * This task:
 * - Sends messages from CAN2 TX queue
 * - Processes messages from CAN2 RX queue
 *
 * @param[in] pvParameters Unused task parameter
 */
void vCan2HandlerServerTask(void *pvParameters)
{
    (void)pvParameters;
    CAN_RX_BUFFER rx;
    CAN_TX_BUFFER tx;

    while (true)
    {
        if (xQueueReceive(xCAN2TXQueueHandler, &tx, 0) == pdPASS)
            CAN2_Write(&tx);

        if (xQueueReceive(xCAN2RXQueueHandler, &rx, 0) == pdPASS)
            vProcessCanRxMessage(&rx, CANBUS_2);

        vTaskDelay(pdMS_TO_TICKS(CAN_TASK_DELAY));
    }
}