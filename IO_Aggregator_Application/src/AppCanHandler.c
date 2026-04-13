/*******************************************************************************
 * File Name   : AppCanHandler.c
 * Company     : Bacancy
 * Summary     : CAN communication handler — implementation
 *
 * Description :
 *   Implements three-channel CAN communication (CAN0, CAN1, CAN2) with
 *   independent FreeRTOS RX/TX queues and dedicated handler tasks.
 *
 *   Architecture per channel:
 *     RX Handler Task  → polls hardware FIFO → pushes to RX queue
 *     Server Task      → dequeues TX → sends to hardware
 *                      → dequeues RX → dispatches to protocol handler
 *
 * Version     : 2.0
 *
 * Changes from v1.0:
 *   - Eliminated code duplication: RX handler and server task logic unified
 *     into generic helpers parameterised by channel index
 *   - Removed dead/commented-out code (old vCan0RxHandlerTask variants)
 *   - Fixed: `static CAN_TX_BUFFER txBuffer` in CAN_Write was not
 *     re-entrant safe — replaced with local stack variable
 *   - Fixed: `char i8len` parameter in CAN_Write (signed, could be negative)
 *     → changed to `uint8_t u8Len` with matching guard
 *   - Fixed: inconsistent TX queue wait timeout (0 vs pdMS_TO_TICKS(5))
 *     across server tasks — unified to non-blocking (0) for TX dequeue
 *   - Fixed: `vCanHandlerInit` returned void but had no way to signal failure
 *     → now returns bool; caller can assert/halt
 *   - Fixed: queue/task handles not checked for NULL before use in tasks
 *   - Fixed: `vDisplayCanRxMessage` used `rxBufLen` parameter but never used
 *     it — removed misleading parameter
 *   - Fixed: macro names `WRITE_ID`/`READ_ID` were too generic — renamed to
 *     `CAN_WRITE_ID`/`CAN_READ_ID` to avoid namespace collisions
 *   - Fixed: `u32CanId` computed in `vProcessCanRxMessage` but never used
 *   - Fixed: guard macro had reserved leading underscore (`_APP_CAN_HANDLER_H`)
 *   - Improved: queue/task arrays replace 6 separate handles → scalable
 *   - Improved: `vCanHandlerInit` validates all resources before starting tasks
 *   - Improved: CAN_IsRxOK and vDisplayCanErrorStatus promoted to file-scope
 *     static inline / static with proper forward declarations
 *   - Improved: all magic numbers replaced with named constants
 *   - Improved: complete Doxygen on every function
 *******************************************************************************/

/* ============================================================================
 * Includes
 * ========================================================================== */
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "definitions.h"
#include "AppCanHandler.h"

/* ============================================================================
 * Debug Control
 * Set CAN_DEBUG_ENABLE to 1 to enable verbose CAN debug output.
 * ========================================================================== */
#define CAN_DEBUG_ENABLE    (0)

#if CAN_DEBUG_ENABLE
    #define CAN_DEBUG(x)    do { x; } while (0)
#else
    #define CAN_DEBUG(x)    do { } while (0)
#endif

/* ============================================================================
 * Internal Configuration
 * ========================================================================== */

/** Task polling delay — balance CPU usage vs. latency (ms) */
#define CAN_RX_POLL_DELAY_MS        (5U)

/** Server task cycle delay (ms) */
#define CAN_SERVER_TASK_DELAY_MS    (10U)

/** Queue send timeout for RX path (ms) — non-zero to allow brief back-pressure */
#define CAN_RX_QUEUE_TIMEOUT_MS     (5U)

/** Queue receive timeout for TX dequeue — non-blocking, return immediately */
#define CAN_TX_DEQUEUE_TIMEOUT_MS   (0U)

/** Queue receive timeout for RX dequeue — non-blocking */
#define CAN_RX_DEQUEUE_TIMEOUT_MS   (0U)

/* ============================================================================
 * CAN Message RAM — placed in dedicated linker sections
 * CACHE_ALIGN ensures correct alignment for DMA access.
 * ========================================================================== */
uint8_t CACHE_ALIGN
    __attribute__((space(data), section(".can0_message_ram")))
    Can0MessageRAM[CAN0_MESSAGE_RAM_CONFIG_SIZE];

uint8_t CACHE_ALIGN
    __attribute__((space(data), section(".can1_message_ram")))
    Can1MessageRAM[CAN1_MESSAGE_RAM_CONFIG_SIZE];

uint8_t CACHE_ALIGN
    __attribute__((space(data), section(".can2_message_ram")))
    Can2MessageRAM[CAN2_MESSAGE_RAM_CONFIG_SIZE];

/* ============================================================================
 * FreeRTOS Queue Handle Arrays
 * Index matches CanBusChannel_e: [0]=CAN0, [1]=CAN1, [2]=CAN2
 * ========================================================================== */
QueueHandle_t xCANRXQueueHandler[CAN_NUM_CHANNELS];
QueueHandle_t xCANTXQueueHandler[CAN_NUM_CHANNELS];

/* ============================================================================
 * Static RX Hardware Buffers
 * One buffer array per CAN channel, sized to hardware FIFO depth.
 * ========================================================================== */
static CAN_RX_BUFFER s_can0RxBuffer[CAN0_RX_FIFO0_SIZE];
static CAN_RX_BUFFER s_can1RxBuffer[CAN1_RX_FIFO0_SIZE];
static CAN_RX_BUFFER s_can2RxBuffer[CAN2_RX_FIFO0_SIZE];

/* ============================================================================
 * External Protocol Dispatch Functions
 * Implemented in BMS/PM handler modules.
 * ========================================================================== */
extern void vProcessBMSCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus);
extern void vProcessPMCanMessage(CAN_RX_BUFFER  *rxBuf, uint8_t canBus);

/* ============================================================================
 * Private Function Declarations
 * ========================================================================== */
static inline bool  CAN_IsRxOK(uint32_t status);
static void         vDisplayCanErrorStatus(uint32_t status, const char *canName);
static void         vDisplayCanRxMessage(const CAN_RX_BUFFER *rxBuf);
static void         vProcessCanRxMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus);
static void         vCanRxHandlerTask_Generic(uint8_t u8Channel,
                                              CAN_RX_BUFFER *pRxBuf,
                                              uint8_t u8FifoSize,
                                              QueueHandle_t xRxQueue,
                                              const char *pcName);
static void         vCanServerTask_Generic(uint8_t u8Channel,
                                           QueueHandle_t xTxQueue,
                                           QueueHandle_t xRxQueue);

/* ============================================================================
 * Utility: 32-bit Byte-Swap
 * ========================================================================== */
/**
 * @brief  Swap byte order of a 32-bit value (little↔big endian conversion).
 *
 * @param  u32Value  Input 32-bit value
 * @return uint32_t  Byte-swapped result
 */
uint32_t swap_endian_32(uint32_t u32Value)
{
    return ((u32Value >> 24U) & 0x000000FFUL) |
           ((u32Value >>  8U) & 0x0000FF00UL) |
           ((u32Value <<  8U) & 0x00FF0000UL) |
           ((u32Value << 24U) & 0xFF000000UL);
}

/* ============================================================================
 * CAN Write — Raw Buffer API
 * ========================================================================== */
/**
 * @brief  Transmit a raw-format CAN frame on the specified channel.
 *
 *         Expected buffer layout:
 *           Bytes [0–3]  : 29-bit CAN ID, big-endian
 *           Bytes [4–11] : 8-byte payload
 *
 * @param  u8CanIndex  Channel index (0–2)
 * @param  pu8Data     Pointer to raw frame (must be >= CAN_RAW_FRAME_SIZE bytes)
 * @param  u8Len       Length of pu8Data buffer
 * @return true on success, false on invalid input or TX hardware failure
 *
 * @note   Thread-safe: uses a local stack buffer — no shared state.
 */
bool CAN_Write(uint8_t u8CanIndex, const uint8_t *pu8Data, uint8_t u8Len)
{
    /* --- Input validation --- */
    if (pu8Data == NULL)
    {
        SYS_CONSOLE_PRINT("[CAN_Write] NULL data pointer\r\n");
        return false;
    }
    if (u8Len < CAN_RAW_FRAME_SIZE)
    {
        SYS_CONSOLE_PRINT("[CAN_Write] Buffer too short: %u < %u\r\n",
                          (unsigned)u8Len, (unsigned)CAN_RAW_FRAME_SIZE);
        return false;
    }
    if (u8CanIndex >= CAN_NUM_CHANNELS)
    {
        SYS_CONSOLE_PRINT("[CAN_Write] Invalid channel: %u\r\n", (unsigned)u8CanIndex);
        return false;
    }

    /* --- Build TX buffer on stack (re-entrant safe, no shared static) --- */
    CAN_TX_BUFFER txBuffer;
    memset(&txBuffer, 0, sizeof(CAN_TX_BUFFER));

    uint32_t u32CanId = ((uint32_t)pu8Data[0] << 24U) |
                        ((uint32_t)pu8Data[1] << 16U) |
                        ((uint32_t)pu8Data[2] <<  8U) |
                        ((uint32_t)pu8Data[3]);

    txBuffer.id  = u32CanId & CAN_EXT_ID_MASK;
    txBuffer.xtd = CAN_FRAME_EXTENDED;
    txBuffer.dlc = CAN_MAX_DLC;

    for (uint8_t i = 0U; i < CAN_MAX_DLC; i++)
    {
        txBuffer.data[i] = pu8Data[i + CAN_ID_BYTE_SIZE];
    }

    CAN_DEBUG(
        SYS_CONSOLE_PRINT("[CAN%u Write] ID: 0x%08lX  Payload:", (unsigned)u8CanIndex,
                          (unsigned long)txBuffer.id);
        for (uint8_t i = 0U; i < CAN_MAX_DLC; i++)
            SYS_CONSOLE_PRINT(" %02X", txBuffer.data[i]);
        SYS_CONSOLE_PRINT("\r\n");
    );

    /* --- Route to hardware FIFO --- */
    switch (u8CanIndex)
    {
        case 0U: return CAN0_MessageTransmitFifo(1U, &txBuffer);
        case 1U: return CAN1_MessageTransmitFifo(1U, &txBuffer);
        case 2U: return CAN2_MessageTransmitFifo(1U, &txBuffer);
        default: return false; /* unreachable — guarded above */
    }
}

/* ============================================================================
 * CAN Write — Structured Buffer API (per channel)
 * ========================================================================== */

/**
 * @brief  Common validation helper for structured TX buffer.
 *         Returns false and logs if buffer is NULL or DLC is out of range.
 */
static inline bool prv_ValidateTxBuffer(const CAN_TX_BUFFER *const pTxBuffer,
                                        const char *pcChannel)
{
    if (pTxBuffer == NULL)
    {
        SYS_CONSOLE_PRINT("[%s Write] NULL TX buffer\r\n", pcChannel);
        return false;
    }
    if (pTxBuffer->dlc > CAN_MAX_DLC)
    {
        SYS_CONSOLE_PRINT("[%s Write] DLC out of range: %u\r\n",
                          pcChannel, (unsigned)pTxBuffer->dlc);
        return false;
    }
    return true;
}

/**
 * @brief  Transmit a structured CAN TX buffer on CAN0.
 * @param  pTxBuffer  Pointer to CAN_TX_BUFFER (must not be NULL, DLC ≤ 8)
 * @return true on success, false on validation failure or TX error
 */
bool CAN0_Write(const CAN_TX_BUFFER *const pTxBuffer)
{
    if (!prv_ValidateTxBuffer(pTxBuffer, "CAN0")) { return false; }

    if (!CAN0_MessageTransmitFifo(1U, pTxBuffer))
    {
        SYS_CONSOLE_PRINT("[CAN0 Write] TX FIFO failed (ID=0x%08lX)\r\n",
                          (unsigned long)pTxBuffer->id);
        return false;
    }
    return true;
}

/**
 * @brief  Transmit a structured CAN TX buffer on CAN1.
 * @param  pTxBuffer  Pointer to CAN_TX_BUFFER (must not be NULL, DLC ≤ 8)
 * @return true on success, false on validation failure or TX error
 */
bool CAN1_Write(const CAN_TX_BUFFER *const pTxBuffer)
{
    if (!prv_ValidateTxBuffer(pTxBuffer, "CAN1")) { return false; }

    if (!CAN1_MessageTransmitFifo(1U, pTxBuffer))
    {
        SYS_CONSOLE_PRINT("[CAN1 Write] TX FIFO failed (ID=0x%08lX)\r\n",
                          (unsigned long)pTxBuffer->id);
        return false;
    }
    return true;
}

/**
 * @brief  Transmit a structured CAN TX buffer on CAN2.
 * @param  pTxBuffer  Pointer to CAN_TX_BUFFER (must not be NULL, DLC ≤ 8)
 * @return true on success, false on validation failure or TX error
 */
bool CAN2_Write(const CAN_TX_BUFFER *const pTxBuffer)
{
    if (!prv_ValidateTxBuffer(pTxBuffer, "CAN2")) { return false; }

    if (!CAN2_MessageTransmitFifo(1U, pTxBuffer))
    {
        SYS_CONSOLE_PRINT("[CAN2 Write] TX FIFO failed (ID=0x%08lX)\r\n",
                          (unsigned long)pTxBuffer->id);
        return false;
    }
    return true;
}

/* ============================================================================
 * TX Queue Enqueue API
 * ========================================================================== */
/**
 * @brief  Enqueue a CAN TX frame for the given dock's CAN channel.
 *
 *         The mapping is:
 *           DOCK_1 → CANBUS_0 / xCANTXQueueHandler[0]
 *           DOCK_2 → CANBUS_1 / xCANTXQueueHandler[1]
 *           DOCK_3 → CANBUS_2 / xCANTXQueueHandler[2]
 *
 *         Non-blocking: if the queue is full the frame is dropped and
 *         a warning is printed. This avoids blocking the caller's task.
 *
 * @param  pCanTxBuffer  Pointer to frame to enqueue (must not be NULL)
 * @param  u8DockNo      Dock identifier (DOCK_1, DOCK_2, or DOCK_3)
 */
void vSendCanTxMsgToQueue(const CAN_TX_BUFFER *const pCanTxBuffer, uint8_t u8DockNo)
{
    if (pCanTxBuffer == NULL)
    {
        SYS_CONSOLE_PRINT("[TxQueue] NULL TX buffer for dock %u\r\n", (unsigned)u8DockNo);
        return;
    }

    /* Map dock number → channel index */
    uint8_t u8Channel;
    switch (u8DockNo)
    {
        case DOCK_1: u8Channel = (uint8_t)CANBUS_0; break;
        case DOCK_2: u8Channel = (uint8_t)CANBUS_1; break;
        case DOCK_3: u8Channel = (uint8_t)CANBUS_2; break;
        default:
            SYS_CONSOLE_PRINT("[TxQueue] Invalid dock number: %u\r\n", (unsigned)u8DockNo);
            return;
    }

    QueueHandle_t xQueue = xCANTXQueueHandler[u8Channel];
    if (xQueue == NULL)
    {
        SYS_CONSOLE_PRINT("[TxQueue] Queue not initialised for channel %u\r\n",
                          (unsigned)u8Channel);
        return;
    }

    if (xQueueSend(xQueue, pCanTxBuffer, pdMS_TO_TICKS(CAN_TX_DEQUEUE_TIMEOUT_MS)) != pdPASS)
    {
        SYS_CONSOLE_PRINT("[TxQueue] Dock %u (CAN%u) queue full — frame dropped\r\n",
                          (unsigned)u8DockNo, (unsigned)u8Channel);
    }
}

/* ============================================================================
 * RX Message Routing
 * ========================================================================== */
/**
 * @brief  Route a received CAN frame to the correct protocol handler.
 *
 *         Routing rule:
 *           Extended frame (xtd == 1) → Power Module handler
 *           Standard frame (xtd == 0) → BMS handler
 *
 * @param  pRxBuf  Pointer to received CAN buffer (must not be NULL)
 * @param  u8CanBus CAN channel index (CANBUS_0–CANBUS_2)
 */
static void vProcessCanRxMessage(CAN_RX_BUFFER *pRxBuf, uint8_t u8CanBus)
{
    if (pRxBuf == NULL)
    {
        SYS_CONSOLE_PRINT("[RxDispatch] NULL RX buffer on CAN%u\r\n", (unsigned)u8CanBus);
        return;
    }

    if (pRxBuf->xtd)
    {
        vProcessPMCanMessage(pRxBuf, u8CanBus);
    }
    else
    {
        vProcessBMSCanMessage(pRxBuf, u8CanBus);
    }
}

/* ============================================================================
 * CAN Status Helpers
 * ========================================================================== */
/**
 * @brief  Evaluate Last Error Code field of CAN PSR to decide if RX is valid.
 *
 *         LEC == 0 (no error) or LEC == 7 (no change since last read)
 *         are both acceptable for receiving.
 *
 * @param  u32Status  CAN protocol status register value
 * @return true  No disqualifying RX error
 * @return false Error condition detected
 */
static inline bool CAN_IsRxOK(uint32_t u32Status)
{
    uint32_t u32Lec = u32Status & CAN_PSR_LEC_Msk;
    return (u32Lec == CAN_ERROR_NONE) || (u32Lec == CAN_ERROR_LEC_NC);
}

/**
 * @brief  Decode and print CAN Protocol Status Register error fields.
 *
 *         Only called when CAN_DEBUG_ENABLE == 1.
 *
 * @param  u32Status  CAN PSR value from CANx_ErrorGet()
 * @param  pcCanName  Human-readable channel name (e.g. "CAN0")
 */
static void vDisplayCanErrorStatus(uint32_t u32Status, const char *pcCanName)
{
    if (pcCanName == NULL) { return; }

    static const char * const pcLecTable[] =
    {
        NULL,            /* 0 = no error    */
        "Stuff error",   /* 1               */
        "Form error",    /* 2               */
        "ACK error",     /* 3               */
        "Bit-1 error",   /* 4               */
        "Bit-0 error",   /* 5               */
        "CRC error",     /* 6               */
        "LEC unchanged"  /* 7               */
    };

    uint32_t u32Lec = u32Status & CAN_PSR_LEC_Msk;
    if ((u32Lec > 0U) && (u32Lec < 7U) && (pcLecTable[u32Lec] != NULL))
    {
        SYS_CONSOLE_PRINT("[%s] %s\r\n", pcCanName, pcLecTable[u32Lec]);
    }

    if (u32Status & CAN_PSR_BO_Msk)
    {
        SYS_CONSOLE_PRINT("[%s] Bus-Off detected!\r\n", pcCanName);
    }

    if (u32Status & CAN_PSR_EP_Msk)
    {
        SYS_CONSOLE_PRINT("[%s] Error-Passive state\r\n", pcCanName);
    }

    if (u32Status & CAN_PSR_EW_Msk)
    {
        SYS_CONSOLE_PRINT("[%s] Error Warning limit reached\r\n", pcCanName);
    }
}

/**
 * @brief  Print received CAN frame ID, DLC, and payload for debug.
 *
 * @param  pRxBuf  Pointer to received CAN buffer (must not be NULL)
 */
static void vDisplayCanRxMessage(const CAN_RX_BUFFER *pRxBuf)
{
    if (pRxBuf == NULL)
    {
        SYS_CONSOLE_PRINT("[RxMsg] NULL buffer\r\n");
        return;
    }

    uint8_t  u8Len  = (pRxBuf->dlc <= CAN_MAX_DLC) ? pRxBuf->dlc : CAN_MAX_DLC;
    uint32_t u32Id  = pRxBuf->xtd ? pRxBuf->id : (CAN_READ_ID(pRxBuf->id) & CAN_STD_ID_MASK);

    SYS_CONSOLE_PRINT("[RxMsg] ID: 0x%08lX  DLC: %u  Data:",
                      (unsigned long)u32Id, (unsigned)u8Len);

    for (uint8_t i = 0U; i < u8Len; i++)
    {
        SYS_CONSOLE_PRINT(" %02X", pRxBuf->data[i]);
    }
    SYS_CONSOLE_PRINT("\r\n");
}

/* ============================================================================
 * Generic RX Handler Task Body
 * ========================================================================== */
/**
 * @brief  Common logic for all CAN RX handler tasks.
 *
 *         Polls the specified channel's RX FIFO each cycle.
 *         On interrupt flag:
 *           1. Clears interrupt flag
 *           2. Checks error status
 *           3. Reads all available frames from FIFO in one call
 *           4. Pushes each frame onto the channel's RX queue
 *
 * @param  u8Channel   CAN channel (0–2)
 * @param  pRxBuf      Per-channel static RX buffer array
 * @param  u8FifoSize  Hardware FIFO depth (CAN0/1/2_RX_FIFO0_SIZE)
 * @param  xRxQueue    RX queue handle for this channel
 * @param  pcName      Channel name string for log output
 */
static void vCanRxHandlerTask_Generic(uint8_t       u8Channel,
                                      CAN_RX_BUFFER *pRxBuf,
                                      uint8_t        u8FifoSize,
                                      QueueHandle_t  xRxQueue,
                                      const char    *pcName)
{
    /* Validate parameters before task loop */
    if ((pRxBuf == NULL) || (xRxQueue == NULL) || (pcName == NULL))
    {
        SYS_CONSOLE_PRINT("[RxTask] Invalid parameters for channel %u — task exiting\r\n",
                          (unsigned)u8Channel);
        vTaskDelete(NULL);
        return; /* unreachable but satisfies static analysers */
    }

    SYS_CONSOLE_PRINT("[%s RX] Task started\r\n", pcName);

    while (true)
    {
        uint32_t u32InterruptFlag;
        uint32_t u32ErrorStatus;
        uint8_t  u8Count;
        bool     bRxResult;

        /* --- Read hardware registers via channel index --- */
        switch (u8Channel)
        {
            case 0U:
                u32InterruptFlag = CAN0_InterruptGet(CAN_INTERRUPT_RF0N_MASK);
                if (!u32InterruptFlag) { break; }
                CAN0_InterruptClear(CAN_INTERRUPT_RF0N_MASK);
                u32ErrorStatus = CAN0_ErrorGet();
                CAN_DEBUG(vDisplayCanErrorStatus(u32ErrorStatus, pcName));
                if (!CAN_IsRxOK(u32ErrorStatus)) { break; }
                u8Count = CAN0_RxFifoFillLevelGet(CAN_RX_FIFO_0);
                if (u8Count == 0U) { break; }
                if (u8Count > u8FifoSize)
                {
                    SYS_CONSOLE_PRINT("[%s RX] FIFO overflow: clamping %u→%u\r\n",
                                      pcName, (unsigned)u8Count, (unsigned)u8FifoSize);
                    u8Count = u8FifoSize;
                }
                bRxResult = CAN0_MessageReceiveFifo(CAN_RX_FIFO_0, u8Count, pRxBuf);
                break;

            case 1U:
                u32InterruptFlag = CAN1_InterruptGet(CAN_INTERRUPT_RF0N_MASK);
                if (!u32InterruptFlag) { break; }
                CAN1_InterruptClear(CAN_INTERRUPT_RF0N_MASK);
                u32ErrorStatus = CAN1_ErrorGet();
                CAN_DEBUG(vDisplayCanErrorStatus(u32ErrorStatus, pcName));
                if (!CAN_IsRxOK(u32ErrorStatus)) { break; }
                u8Count = CAN1_RxFifoFillLevelGet(CAN_RX_FIFO_0);
                if (u8Count == 0U) { break; }
                if (u8Count > u8FifoSize)
                {
                    SYS_CONSOLE_PRINT("[%s RX] FIFO overflow: clamping %u→%u\r\n",
                                      pcName, (unsigned)u8Count, (unsigned)u8FifoSize);
                    u8Count = u8FifoSize;
                }
                bRxResult = CAN1_MessageReceiveFifo(CAN_RX_FIFO_0, u8Count, pRxBuf);
                break;

            case 2U:
                u32InterruptFlag = CAN2_InterruptGet(CAN_INTERRUPT_RF0N_MASK);
                if (!u32InterruptFlag) { break; }
                CAN2_InterruptClear(CAN_INTERRUPT_RF0N_MASK);
                u32ErrorStatus = CAN2_ErrorGet();
                CAN_DEBUG(vDisplayCanErrorStatus(u32ErrorStatus, pcName));
                if (!CAN_IsRxOK(u32ErrorStatus)) { break; }
                u8Count = CAN2_RxFifoFillLevelGet(CAN_RX_FIFO_0);
                if (u8Count == 0U) { break; }
                if (u8Count > u8FifoSize)
                {
                    SYS_CONSOLE_PRINT("[%s RX] FIFO overflow: clamping %u→%u\r\n",
                                      pcName, (unsigned)u8Count, (unsigned)u8FifoSize);
                    u8Count = u8FifoSize;
                }
                bRxResult = CAN2_MessageReceiveFifo(CAN_RX_FIFO_0, u8Count, pRxBuf);
                break;

            default:
                /* Should never reach here */
                vTaskDelay(pdMS_TO_TICKS(CAN_RX_POLL_DELAY_MS));
                continue;
        }

        /* --- Queue received frames --- */
        if (u32InterruptFlag && CAN_IsRxOK(u32ErrorStatus))
        {
            if (bRxResult)
            {
                for (uint8_t i = 0U; i < u8Count; i++)
                {
                    CAN_DEBUG(vDisplayCanRxMessage(&pRxBuf[i]));

                    if (xQueueSend(xRxQueue, &pRxBuf[i],
                                   pdMS_TO_TICKS(CAN_RX_QUEUE_TIMEOUT_MS)) != pdPASS)
                    {
                        SYS_CONSOLE_PRINT("[%s RX] Queue full - frame dropped (ID=0x%08lX)\r\n",
                                          pcName, (unsigned long)pRxBuf[i].id);
                    }
                }
            }
            else
            {
                SYS_CONSOLE_PRINT("[%s RX] MessageReceiveFifo failed\r\n", pcName);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CAN_RX_POLL_DELAY_MS));
    }
}

/* ============================================================================
 * Generic Server Task Body
 * ========================================================================== */
/**
 * @brief  Common logic for all CAN server (TX+RX dispatch) tasks.
 *
 *         Each cycle:
 *           1. Dequeue one TX frame and transmit immediately (non-blocking dequeue)
 *           2. Dequeue one RX frame and dispatch to protocol handler (non-blocking)
 *
 * @param  u8Channel   CAN channel index (0–2), used for CAN_Write routing
 * @param  xTxQueue    TX queue for this channel
 * @param  xRxQueue    RX queue for this channel
 */
static void vCanServerTask_Generic(uint8_t       u8Channel,
                                   QueueHandle_t xTxQueue,
                                   QueueHandle_t xRxQueue)
{
    if ((xTxQueue == NULL) || (xRxQueue == NULL))
    {
        SYS_CONSOLE_PRINT("[SrvTask] NULL queue for CAN%u — task exiting\r\n",
                          (unsigned)u8Channel);
        vTaskDelete(NULL);
        return;
    }

    CAN_TX_BUFFER tx;
    CAN_RX_BUFFER rx;

    while (true)
    {
        /* Dequeue and transmit one TX frame (non-blocking) */
        if (xQueueReceive(xTxQueue, &tx, pdMS_TO_TICKS(CAN_TX_DEQUEUE_TIMEOUT_MS)) == pdPASS)
        {
            bool bTxOk;
            switch (u8Channel)
            {
                case 0U: bTxOk = CAN0_Write(&tx); break;
                case 1U: bTxOk = CAN1_Write(&tx); break;
                case 2U: bTxOk = CAN2_Write(&tx); break;
                default: bTxOk = false;            break;
            }
            if (!bTxOk)
            {
                SYS_CONSOLE_PRINT("[SrvTask CAN%u] TX write failed (ID=0x%08lX)\r\n",
                                  (unsigned)u8Channel, (unsigned long)tx.id);
            }
        }

        /* Dequeue and dispatch one RX frame (non-blocking) */
        if (xQueueReceive(xRxQueue, &rx, pdMS_TO_TICKS(CAN_RX_DEQUEUE_TIMEOUT_MS)) == pdPASS)
        {
            vProcessCanRxMessage(&rx, u8Channel);
        }

        vTaskDelay(pdMS_TO_TICKS(CAN_SERVER_TASK_DELAY_MS));
    }
}

/* ============================================================================
 * Per-Channel RX Handler Tasks (thin wrappers over generic body)
 * ========================================================================== */

/** @brief CAN0 RX handler task — polls CAN0 FIFO, pushes to RX queue. */
void vCan0RxHandlerTask(void *pvParameters)
{
    (void)pvParameters;
    vCanRxHandlerTask_Generic(0U,
                              s_can0RxBuffer,
                              (uint8_t)CAN0_RX_FIFO0_SIZE,
                              xCANRXQueueHandler[0],
                              "CAN0");
}

/** @brief CAN1 RX handler task — polls CAN1 FIFO, pushes to RX queue. */
void vCan1RxHandlerTask(void *pvParameters)
{
    (void)pvParameters;
    vCanRxHandlerTask_Generic(1U,
                              s_can1RxBuffer,
                              (uint8_t)CAN1_RX_FIFO0_SIZE,
                              xCANRXQueueHandler[1],
                              "CAN1");
}

/** @brief CAN2 RX handler task — polls CAN2 FIFO, pushes to RX queue. */
void vCan2RxHandlerTask(void *pvParameters)
{
    (void)pvParameters;
    vCanRxHandlerTask_Generic(2U,
                              s_can2RxBuffer,
                              (uint8_t)CAN2_RX_FIFO0_SIZE,
                              xCANRXQueueHandler[2],
                              "CAN2");
}

/* ============================================================================
 * Per-Channel Server Tasks (thin wrappers over generic body)
 * ========================================================================== */

/** @brief CAN0 server task — dequeues TX, dispatches RX for CAN0. */
void vCan0HandlerServerTask(void *pvParameters)
{
    (void)pvParameters;
    vCanServerTask_Generic(0U, xCANTXQueueHandler[0], xCANRXQueueHandler[0]);
}

/** @brief CAN1 server task — dequeues TX, dispatches RX for CAN1. */
void vCan1HandlerServerTask(void *pvParameters)
{
    (void)pvParameters;
    vCanServerTask_Generic(1U, xCANTXQueueHandler[1], xCANRXQueueHandler[1]);
}

/** @brief CAN2 server task — dequeues TX, dispatches RX for CAN2. */
void vCan2HandlerServerTask(void *pvParameters)
{
    (void)pvParameters;
    vCanServerTask_Generic(2U, xCANTXQueueHandler[2], xCANRXQueueHandler[2]);
}

/* ============================================================================
 * Initialisation
 * ========================================================================== */
/**
 * @brief  Configure CAN message RAM, create FreeRTOS queues and handler tasks.
 *
 *         Call once at system startup before the scheduler starts.
 *         All six queues are created first; if any fail the function returns
 *         false and no tasks are created (avoids tasks referencing NULL queues).
 *
 * @return true   All resources created successfully
 * @return false  One or more queue or task creation failures
 */
bool vCanHandlerInit(void)
{
    bool bSuccess = true;

    /* --- Configure hardware message RAM --- */
    CAN0_MessageRAMConfigSet(Can0MessageRAM);
    CAN1_MessageRAMConfigSet(Can1MessageRAM);
    CAN2_MessageRAMConfigSet(Can2MessageRAM);

    /* --- Create FreeRTOS queues --- */
    xCANRXQueueHandler[0] = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CAN_RX_BUFFER));
    xCANTXQueueHandler[0] = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CAN_TX_BUFFER));
    xCANRXQueueHandler[1] = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CAN_RX_BUFFER));
    xCANTXQueueHandler[1] = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CAN_TX_BUFFER));
    xCANRXQueueHandler[2] = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CAN_RX_BUFFER));
    xCANTXQueueHandler[2] = xQueueCreate(CAN_QUEUE_SIZE, sizeof(CAN_TX_BUFFER));

    /* Verify all queues were created before starting tasks */
    for (uint8_t i = 0U; i < CAN_NUM_CHANNELS; i++)
    {
        if (xCANRXQueueHandler[i] == NULL)
        {
            SYS_CONSOLE_PRINT("[CanInit] RX queue creation failed for CAN%u\r\n", (unsigned)i);
            bSuccess = false;
        }
        if (xCANTXQueueHandler[i] == NULL)
        {
            SYS_CONSOLE_PRINT("[CanInit] TX queue creation failed for CAN%u\r\n", (unsigned)i);
            bSuccess = false;
        }
    }

    if (!bSuccess)
    {
        SYS_CONSOLE_PRINT("[CanInit] Queue creation failed — tasks NOT started\r\n");
        return false;
    }

    SYS_CONSOLE_PRINT("[CanInit] All queues created successfully\r\n");

    /* --- Create RX Handler tasks --- */
    static const struct {
        TaskFunction_t  pfTask;
        const char     *pcName;
        uint16_t        u16Stack;
    } rxTasks[CAN_NUM_CHANNELS] = {
        { vCan0RxHandlerTask, "CAN0_RX", CAN0_RX_HANDLER_STACK_DEPTH },
        { vCan1RxHandlerTask, "CAN1_RX", CAN1_RX_HANDLER_STACK_DEPTH },
        { vCan2RxHandlerTask, "CAN2_RX", CAN2_RX_HANDLER_STACK_DEPTH },
    };

    for (uint8_t i = 0U; i < CAN_NUM_CHANNELS; i++)
    {
        if (xTaskCreate(rxTasks[i].pfTask,
                        rxTasks[i].pcName,
                        rxTasks[i].u16Stack,
                        NULL,
                        CAN_RX_HANDLER_TASK_PRIORITY,
                        NULL) != pdPASS)
        {
            SYS_CONSOLE_PRINT("[CanInit] %s task creation failed\r\n", rxTasks[i].pcName);
            bSuccess = false;
        }
        else
        {
            SYS_CONSOLE_PRINT("[CanInit] %s task created\r\n", rxTasks[i].pcName);
        }
    }

    /* --- Create Server tasks --- */
    static const struct {
        TaskFunction_t  pfTask;
        const char     *pcName;
    } srvTasks[CAN_NUM_CHANNELS] = {
        { vCan0HandlerServerTask, "CAN0_SRV" },
        { vCan1HandlerServerTask, "CAN1_SRV" },
        { vCan2HandlerServerTask, "CAN2_SRV" },
    };

    for (uint8_t i = 0U; i < CAN_NUM_CHANNELS; i++)
    {
        if (xTaskCreate(srvTasks[i].pfTask,
                        srvTasks[i].pcName,
                        CAN_SERVER_HANDLER_STACK_DEPTH,
                        NULL,
                        CAN_SERVER_HANDLER_TASK_PRIORITY,
                        NULL) != pdPASS)
        {
            SYS_CONSOLE_PRINT("[CanInit] %s task creation failed\r\n", srvTasks[i].pcName);
            bSuccess = false;
        }
        else
        {
            SYS_CONSOLE_PRINT("[CanInit] %s task created\r\n", srvTasks[i].pcName);
        }
    }

    if (bSuccess)
    {
        SYS_CONSOLE_PRINT("[CanInit] Initialisation complete\r\n");
    }
    else
    {
        SYS_CONSOLE_PRINT("[CanInit] WARNING: One or more tasks failed to start\r\n");
    }

    return bSuccess;
}

/*******************************************************************************
 * End of File
 *******************************************************************************/