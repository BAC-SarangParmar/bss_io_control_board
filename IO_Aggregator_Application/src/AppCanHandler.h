/*******************************************************************************
 * File Name   : AppCanHandler.h
 * Company     : Bacancy 
 * Summary     : CAN communication handler — header file
 *
 * Description :
 *   Provides function prototypes, macros, and type definitions for managing
 *   three-channel CAN communication (CAN0, CAN1, CAN2) over FreeRTOS queues.
 *   Each channel has independent RX/TX queues and dedicated handler tasks.
 *
 * Version     : 2.0
 *******************************************************************************/

#ifndef APP_CAN_HANDLER_H   /* Guard renamed: no leading underscore (reserved by C standard) */
#define APP_CAN_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Includes
 * ========================================================================== */
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "IOHandler.h"

/* ============================================================================
 * CAN Channel Count
 * ========================================================================== */
#define CAN_NUM_CHANNELS        (3U)    /**< Total CAN channels supported (CAN0–CAN2) */

/* ============================================================================
 * CAN Frame Field Definitions
 * ========================================================================== */
#define CAN_EXT_ID_MASK         (0x1FFFFFFFUL) /**< 29-bit extended CAN ID mask        */
#define CAN_STD_ID_MASK         (0x7FFUL)      /**< 11-bit standard CAN ID mask        */
#define CAN_MAX_DLC             (8U)           /**< Maximum CAN data length code       */

#define CAN_ID_BYTE_SIZE        (4U)           /**< Bytes used to carry CAN ID in buffer */
#define CAN_PAYLOAD_BYTE_SIZE   (8U)           /**< CAN payload size in bytes          */
#define CAN_RAW_FRAME_SIZE      (CAN_ID_BYTE_SIZE + CAN_PAYLOAD_BYTE_SIZE) /**< Total raw frame size */

/* ============================================================================
 * Frame Type Identifiers
 * ========================================================================== */
#define CAN_FRAME_EXTENDED      (1U)    /**< Extended (29-bit) CAN frame        */
#define CAN_FRAME_STANDARD      (0U)    /**< Standard (11-bit) CAN frame        */

/* ============================================================================
 * CAN ID Shift Macros
 * ========================================================================== */
/** @brief Convert 29-bit ID for hardware FIFO transmission format */
#define CAN_WRITE_ID(id)        ((uint32_t)(id) << 18U)

/** @brief Extract 29-bit ID from hardware FIFO receive format */
#define CAN_READ_ID(id)         ((uint32_t)(id) >> 18U)

/* ============================================================================
 * FreeRTOS Queue Configuration
 * ========================================================================== */
#define CAN_QUEUE_SIZE          (50U)   /**< Max messages per queue             */
#define CAN_QUEUE_ITEM_SIZE     (128U)  /**< Reserved — actual item = sizeof(CAN_RX/TX_BUFFER) */

/* ============================================================================
 * FreeRTOS Task Configuration
 * ========================================================================== */
/** Stack depth (in words) for each RX handler task */
#define CAN0_RX_HANDLER_STACK_DEPTH     (512U)
#define CAN1_RX_HANDLER_STACK_DEPTH     (512U)
#define CAN2_RX_HANDLER_STACK_DEPTH     (512U)

/** Stack depth (in words) for each server (TX+RX dispatch) task */
#define CAN_SERVER_HANDLER_STACK_DEPTH  (512U)

/** Task priorities */
#define CAN_RX_HANDLER_TASK_PRIORITY    (tskIDLE_PRIORITY + 4U)
#define CAN_SERVER_HANDLER_TASK_PRIORITY (tskIDLE_PRIORITY + 3U)

/* ============================================================================
 * CAN Channel Index Enumeration
 * ========================================================================== */
/**
 * @brief CAN bus channel identifiers.
 *        Used as index into per-channel queue/task arrays.
 */
// typedef enum
// {
//     CANBUS_0 = 0U,  /**< CAN channel 0 — mapped to DOCK_1 */
//     CANBUS_1 = 1U,  /**< CAN channel 1 — mapped to DOCK_2 */
//     CANBUS_2 = 2U,  /**< CAN channel 2 — mapped to DOCK_3 */
//     CANBUS_MAX      /**< Sentinel — total channel count    */
// } CanBusChannel_e;

/* ============================================================================
 * Externally visible Queue Handles
 * (access restricted — use API functions where possible)
 * ========================================================================== */
extern QueueHandle_t xCANRXQueueHandler[CAN_NUM_CHANNELS];
extern QueueHandle_t xCANTXQueueHandler[CAN_NUM_CHANNELS];

/* ============================================================================
 * Function Prototypes — Initialization
 * ========================================================================== */

/**
 * @brief  Initialize CAN message RAM, FreeRTOS queues, and handler tasks.
 *
 *         Must be called once during system startup before any CAN activity.
 *         Halts (asserts) in debug builds if queue or task creation fails.
 *
 * @return true  All resources created successfully
 * @return false One or more queue/task creation failures (see console output)
 */
bool vCanHandlerInit(void);

/* ============================================================================
 * Function Prototypes — CAN Write API
 * ========================================================================== */

/**
 * @brief  Transmit a raw byte-array CAN frame on the specified channel.
 *
 *         Buffer layout:
 *           Bytes [0–3] : 29-bit CAN ID (big-endian)
 *           Bytes [4–11]: 8-byte payload
 *
 * @param  u8CanIndex  CAN channel index (0, 1, or 2)
 * @param  pu8Data     Pointer to raw frame buffer (min CAN_RAW_FRAME_SIZE bytes)
 * @param  i8Len       Total buffer length
 * @return true on success, false on invalid input or TX failure
 */
bool CAN_Write(uint8_t u8CanIndex, const uint8_t *pu8Data, uint8_t u8Len);

/**
 * @brief  Transmit a structured CAN TX buffer on CAN0.
 * @param  pTxBuffer  Pointer to populated CAN_TX_BUFFER (must not be NULL)
 * @return true on success, false on failure
 */
bool CAN0_Write(const CAN_TX_BUFFER *const pTxBuffer);

/**
 * @brief  Transmit a structured CAN TX buffer on CAN1.
 * @param  pTxBuffer  Pointer to populated CAN_TX_BUFFER (must not be NULL)
 * @return true on success, false on failure
 */
bool CAN1_Write(const CAN_TX_BUFFER *const pTxBuffer);

/**
 * @brief  Transmit a structured CAN TX buffer on CAN2.
 * @param  pTxBuffer  Pointer to populated CAN_TX_BUFFER (must not be NULL)
 * @return true on success, false on failure
 */
bool CAN2_Write(const CAN_TX_BUFFER *const pTxBuffer);

/**
 * @brief  Enqueue a CAN TX buffer for transmission on the dock's CAN channel.
 *
 *         Non-blocking — drops message and logs warning if queue is full.
 *
 * @param  pCanTxBuffer  Pointer to CAN TX buffer (must not be NULL)
 * @param  u8DockNo      Dock identifier (DOCK_1, DOCK_2, DOCK_3)
 */
void vSendCanTxMsgToQueue(const CAN_TX_BUFFER *const pCanTxBuffer, uint8_t u8DockNo);

/* ============================================================================
 * Function Prototypes — RX Handler Tasks
 * ========================================================================== */

/**
 * @brief  FreeRTOS task: poll CAN0 RX FIFO and push messages to RX queue.
 * @param  pvParameters  Unused (pass NULL)
 */
void vCan0RxHandlerTask(void *pvParameters);

/**
 * @brief  FreeRTOS task: poll CAN1 RX FIFO and push messages to RX queue.
 * @param  pvParameters  Unused (pass NULL)
 */
void vCan1RxHandlerTask(void *pvParameters);

/**
 * @brief  FreeRTOS task: poll CAN2 RX FIFO and push messages to RX queue.
 * @param  pvParameters  Unused (pass NULL)
 */
void vCan2RxHandlerTask(void *pvParameters);

/* ============================================================================
 * Function Prototypes — Server Tasks (TX dequeue + RX dispatch)
 * ========================================================================== */

/**
 * @brief  FreeRTOS task: dequeue TX frames for CAN0 and dispatch RX messages.
 * @param  pvParameters  Unused (pass NULL)
 */
void vCan0HandlerServerTask(void *pvParameters);

/**
 * @brief  FreeRTOS task: dequeue TX frames for CAN1 and dispatch RX messages.
 * @param  pvParameters  Unused (pass NULL)
 */
void vCan1HandlerServerTask(void *pvParameters);

/**
 * @brief  FreeRTOS task: dequeue TX frames for CAN2 and dispatch RX messages.
 * @param  pvParameters  Unused (pass NULL)
 */
void vCan2HandlerServerTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* APP_CAN_HANDLER_H */

/*******************************************************************************
 * End of File
 *******************************************************************************/