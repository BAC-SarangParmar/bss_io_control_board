/*******************************************************************************
 * AppCANHandler Application Header File
 *
 * Company:
 *   Bacancy - SunMobility
 *
 * File Name:
 *   AppCanHandler.h
 *
 * Summary:
 *   This header file provides function prototypes and definitions for the CAN
 *   communication handling in the IO Aggregator application.
 *
 * Description:
 *   This file defines function prototypes, macros, and data types used for
 *   managing CAN communication. It includes definitions for CAN server ports,
 *   queue sizes, and task priorities.
 *******************************************************************************/

#ifndef _APP_CAN_HANDLER_H
#define _APP_CAN_HANDLER_H

// *****************************************************************************
// Section: Included Files
// *****************************************************************************
#include <stddef.h>   // Defines NULL
#include <stdbool.h>  // Defines true
#include <stdlib.h>   // Defines EXIT_FAILURE
#include "IOHandler.h" // Include IO Handler dependencies

// *****************************************************************************
// CAN Configuration Macros
// *****************************************************************************

#define CAN_MSG_EID_MASK  0x1FFFFFFF  // 29-bit Extended Identifier Mask

// Task configuration for CAN message reception and server handling
#define CAN_RX_HANDLER_HEAP_DEPTH     1024
#define CAN_RX_HANDLER_TASK_PRIORITY  4//6//(tskIDLE_PRIORITY + 4)  // Highest priority

#define CAN_SERVER_HANDLER_HEAP_DEPTH 512
#define CAN_SERVER_HANDLER_TASK_PRIORITY 4//5//(tskIDLE_PRIORITY + 3)

// Queue configuration for CAN message handling
#define CAN_QUEUE_SIZE      (50U)   // Maximum number of messages in the queue
#define CAN_QUEUE_ITEM_SIZE (128U)  // Size of each queue item in bytes

// Macros for shifting CAN message IDs
#define WRITE_ID(id) ((id) << 18)  // Convert 29-bit ID for transmission
#define READ_ID(id)  ((id) >> 18)  // Convert received ID to readable format

// *****************************************************************************
// CAN Server Port Configuration
// *****************************************************************************
#if HEV_IO_Aggregator
    extern flash_data_t writeData;    
    extern TCP_SOCKET sCan0ServerSocket ;
    extern TCP_SOCKET sCan1ServerSocket ;
    extern TCP_SOCKET sCan2ServerSocket ;
    extern TCP_SOCKET sCan3ServerSocket ;
    extern TCP_SOCKET sCan4ServerSocket ;
    extern TCP_SOCKET sCan5ServerSocket ;    
#endif

#if Two_Wheeler_IO_Aggregator
    #define CAN0_SERVER_PORT  32000
    #define CAN1_SERVER_PORT  32001
    #define CAN2_SERVER_PORT  32002
    #define CAN3_SERVER_PORT  32003
    #define CAN4_SERVER_PORT  32004
    #define CAN5_SERVER_PORT  32005
#endif

// *****************************************************************************
// Data Size Definitions
// *****************************************************************************
#define CAN_ID_SIZE   (4U)   // First 4 bytes contain the CAN ID
#define CAN_DATA_SIZE (8U)   // Next 8 bytes contain the CAN data payload
#define TCP_DATA_SIZE (CAN_ID_SIZE + CAN_DATA_SIZE) // Total data size for TCP transmission

// *****************************************************************************
// Global Variables
// *****************************************************************************
extern uint8_t CanMessageRAM[CAN3_MESSAGE_RAM_CONFIG_SIZE];

// *****************************************************************************
// Function Prototypes
// *****************************************************************************

/**
 * @brief CAN message reception task.
 *
 * This task handles incoming CAN messages and processes them accordingly.
 *
 * @param pvParameters Task parameters (unused).
 */
void vCanRxHandlerTask(void *pvParameters);

/**
 * @brief Server tasks for handling CAN communication over TCP.
 *
 * These functions handle server sockets for different CAN channels.
 *
 * @param pvParameters Task parameters (unused).
 */
void vCan0HandlerServerTask(void *pvParameters);
void vCan1HandlerServerTask(void *pvParameters);
void vCan2HandlerServerTask(void *pvParameters);
void vCan3HandlerServerTask(void *pvParameters);
void vCan4HandlerServerTask(void *pvParameters);
void vCan5HandlerServerTask(void *pvParameters);

/**
 * @brief Initializes the CAN handler module.
 *
 * This function sets up CAN communication, including task creation and queue initialization.
 */
void vCanHandlerInit(void);

/**
 * @brief Transmits a CAN message on the specified CAN interface.
 *
 * @param u8data Pointer to the data buffer containing the CAN ID (4 bytes)
 *               followed by the payload.
 * @param i8len Length of the data (not directly used, as payload size is fixed).
 * @return True if the message was successfully transmitted, false otherwise.
 */
bool CAN_Write(uint8_t canIndex, uint8_t *u8data, char i8len);
bool CAN0_Write(uint8_t *u8data, char i8len);
bool CAN1_Write(uint8_t *u8data, char i8len);
bool CAN2_Write(uint8_t *u8data, char i8len);
bool CAN3_Write(uint8_t *u8data, char i8len);
bool CAN4_Write(uint8_t *u8data, char i8len);
bool CAN5_Write(uint8_t *u8data, char i8len);

#endif /* _APP_CAN_HANDLER_H */

/*******************************************************************************
 * End of File
 *******************************************************************************/
