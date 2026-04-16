/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    common.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
#include "definitions.h"                /* SYS function prototypes */
#include "configuration.h"
#include "device.h"
#include "tcpip_manager_control.h"
#include "library/tcpip/tcpip_helpers.h"
#include <math.h>
#include <stddef.h>                     /* For NULL definition */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "common.h"                     /* Own header for prototypes and types */

/* ************************************************************************** */
/* Section: File Scope or Global Data                                         */
/* ************************************************************************** */

/* MISRA C:2004 Rule 8.7 - Static storage duration for file scope variables. */
static TCP_SOCKET sDebugServerSocket = INVALID_SOCKET;

/* ************************************************************************** */
/* Section: Local Prototypes                                                  */
/* ************************************************************************** */

/**
 * @brief  Parse and handle incoming debug packets.
 * @param  u8Buffer Pointer to the input buffer containing the packet.
 * @param  inbPacketSz Size of the incoming packet.
 * @return None.
 */
static void ParseDebugPacket(const uint8_t u8Buffer[], uint32_t inbPacketSz);

/**
 * @brief  FreeRTOS task that handles the debug server socket.
 *         Accepts new connections and processes incoming debug commands.
 * @param  pvParameters Pointer to task parameters (unused).
 * @return None.
 */
static void vCommonServerTask(void *pvParameters);

/* ************************************************************************** */
/* Section: Function Definitions                                              */
/* ************************************************************************** */

/**
 * @brief  Report an error code via TCP/IP.
 * @param  code The error code to send.
 * @return None.
 */
void ReportError(ERROR_CODE_t code)
{
    uint8_t txBuf[2U];
    txBuf[0] = (uint8_t)code;
    
    /* Store code in Big Endian (network order) */
    txBuf[0] = (uint8_t)(code >> 8);   // High byte
    txBuf[1] = (uint8_t)(code & 0xFF); // Low byte

    /* Send error code to Ethernet (TCP/IP) */
    (void)TCPIP_TCP_ArrayPut(sDebugServerSocket, txBuf, (uint16_t)sizeof(txBuf));
    (void)TCPIP_TCP_Flush(sDebugServerSocket);
}

/**
 * @brief  Parse and handle incoming debug packets.
 * @param  u8Buffer Pointer to the input buffer containing the packet.
 * @param  inbPacketSz Size of the incoming packet.
 * @return None.
 */
static void ParseDebugPacket(const uint8_t u8Buffer[], uint32_t inbPacketSz)
{
    (void)SYS_CONSOLE_PRINT("Parsing incoming packet...\n");

    /* Check if input buffer is NULL */
    if (u8Buffer == NULL)
    {
        (void)SYS_CONSOLE_PRINT("Error: Input buffer is NULL\n");
        return;
    }

    switch (u8Buffer[0])
    {
        case 0x02U: /* Bootloader trigger */
            (void)SYS_CONSOLE_MESSAGE("\n\r####### Bootloader Triggered #######\n\r");
            (void)SYS_CONSOLE_MESSAGE("\n\r####### Program new firmware from Bootloader #######\n\r");

            ramStart[0] = BTL_TRIGGER_PATTERN;
            ramStart[1] = BTL_TRIGGER_PATTERN;
            ramStart[2] = BTL_TRIGGER_PATTERN;
            ramStart[3] = BTL_TRIGGER_PATTERN;

            DCACHE_CLEAN_BY_ADDR(ramStart, 16U);

            SYS_RESET_SoftwareReset();
            break;

        case 0x03U: /* Soft Reset */
            (void)SYS_CONSOLE_PRINT("Soft Reset\n");
            NVIC_SystemReset();
            break;

        case 0x04U: /* configure Serial number */
            if (inbPacketSz >= 3U)
            {
                serialnum = ((uint16_t)u8Buffer[1] << 8U) | u8Buffer[2];
                (void)SYS_CONSOLE_PRINT("Received Serial Number: 0x%04X (%u)\n", serialnum, serialnum);
                saveOutputsToFlash(doStatus, relayStatus, serialnum);
            }
            else
            {
                (void)SYS_CONSOLE_PRINT("Error: Incomplete serial number data\n");
            }
            break;

        case 0x10U:    /* Give me SN & FW */
        {
            uint8_t txBuf[5U];
            txBuf[0] = (uint8_t)(serialnum >> 8U);         /* MSB first */
            txBuf[1] = (uint8_t)(serialnum & 0xFFU);
            txBuf[2] = (uint8_t)((SYS_FW_VERSION >> 16U) & 0xFFU); /* Major */
            txBuf[3] = (uint8_t)((SYS_FW_VERSION >> 8U) & 0xFFU);  /* Minor */
            txBuf[4] = (uint8_t)(SYS_FW_VERSION & 0xFFU);          /* Patch */

            if (TCPIP_TCP_IsConnected(sDebugServerSocket) != false)
            {
                (void)TCPIP_TCP_ArrayPut(sDebugServerSocket, txBuf, (uint16_t)sizeof(txBuf));
                (void)TCPIP_TCP_Flush(sDebugServerSocket);
                (void)SYS_CONSOLE_PRINT("Sent SN+FW via TCP: SN=0x%04X, FW=%u.%u.%u\n",
                                   serialnum,
                                   txBuf[2], txBuf[3], txBuf[4]);
            }
            else
            {
                (void)SYS_CONSOLE_PRINT("TCP Socket not ready\n");
            }
            
            #if Two_Wheeler_IO_Aggregator      
                /* Ensure the socket is ready to send data */
                if (TCPIP_UDP_PutIsReady(sDebugServerSocket) >= (int32_t)sizeof(txBuf)) 
                {
                    (void)TCPIP_UDP_ArrayPut(sDebugServerSocket, txBuf, (uint16_t)sizeof(txBuf));
                    (void)TCPIP_UDP_Flush(sDebugServerSocket);  /* Ensure data is sent */
                    (void)TCPIP_UDP_Discard(sDebugServerSocket);

                    (void)SYS_CONSOLE_PRINT("Sent SN+FW via UDP: SN=0x%04X, FW=%u.%u.%u\n",
                                   serialnum,
                                   txBuf[2], txBuf[3], txBuf[4]);
                } 
                else 
                {
                    (void)SYS_CONSOLE_PRINT("UDP Socket not ready\n");
                }
            #endif             
            break;
        }

        default:
            /* No action for unrecognized command */
            break;
    }
}

/**
 * @brief  FreeRTOS task that handles the debug server socket.
 *         Accepts new connections and processes incoming debug commands.
 * @param  pvParameters Pointer to task parameters (unused).
 * @return None.
 */
static void vCommonServerTask(void *pvParameters)
{
    (void)pvParameters; /* To avoid unused parameter warning */

    /* Print function entry for debugging */
    (void)SYS_CONSOLE_PRINT("In Function: %s\r\n", __func__);

    /* Open server socket based on the aggregator type */
    sDebugServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, DEBUG_SERVER_PORT_HEV, 0U);

    if (sDebugServerSocket == INVALID_SOCKET)
    {
        (void)SYS_CONSOLE_PRINT("Error opening IO server socket\r\n");
        vTaskDelete(NULL);
        return;
    }

    for (;;)
    {
        uint8_t u8Buffer[256U] = {0};

        if (TCPIP_TCP_IsConnected(sDebugServerSocket) != false)
        {
            int16_t bytesRead = TCPIP_TCP_ArrayGet(sDebugServerSocket, u8Buffer, (uint16_t)sizeof(u8Buffer));

            if (bytesRead > 0)
            {
                (void)SYS_CONSOLE_PRINT("Got Data on Debug Server Handler (TCP)\r\n");
                /* Read and process incoming commands if any */
                ParseDebugPacket(u8Buffer, (uint32_t)bytesRead);
            }

            if ((TCPIP_TCP_IsConnected(sDebugServerSocket) == false) ||
                (TCPIP_TCP_WasDisconnected(sDebugServerSocket) != false))
            {
                (void)SYS_CONSOLE_PRINT("\r\nTCP Connection Closed\r\n");
                TCPIP_TCP_Close(sDebugServerSocket);
                sDebugServerSocket = INVALID_SOCKET;
            }
        }

        /* Ensure reconnection logic is handled correctly */
        if (sDebugServerSocket == INVALID_SOCKET)
        {
            (void)SYS_CONSOLE_PRINT("TCP server Debug socket disconnected, attempting to reconnect...\r\n");

            /* Attempt to open the socket again */
            sDebugServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, DEBUG_SERVER_PORT_HEV, 0U);

            if (sDebugServerSocket == INVALID_SOCKET)
            {
                (void)SYS_CONSOLE_PRINT("Failed to reopen Debug socket, retrying...\r\n");
            }
            else
            {
                (void)SYS_CONSOLE_PRINT("Reconnected successfully IO Socket!\r\n");
            }

            vTaskDelay(RECONNECT_DELAY_MS);
        }
        vTaskDelay(10U); /* Add a small delay to avoid consuming too much CPU time */
    }
}

/* MISRA C:2012 Compliant Definitions */
#define HEARTBEAT_INTERVAL_MS   (1000U)   /* 1 second (Rule 8.9: Constant defined instead of magic number) */
#define HEARTBEAT_MSG           "HEARTBEAT\n"  /* Rule 7.4: String literal as constant */

/* Heartbeat FreeRTOS task */
static void vHeartbeatTask(void *pvParameters)
{
    /* Rule 2.7: Parameter intentionally unused */
    (void)pvParameters;

    /* Intentional infinite loop - Rule 15.5 deviation (task design requires it) */
    for (;;)
    {
        if ((sDebugServerSocket != INVALID_SOCKET) &&
            (TCPIP_TCP_IsConnected(sDebugServerSocket) != false))
        {
            /* Send heartbeat via TCP */
            /* Cast from const char* to uint8_t* is required by API - Rule 11.3 deviation */
            (void)TCPIP_TCP_ArrayPut(sDebugServerSocket,
                                     (uint8_t *)HEARTBEAT_MSG,
                                     (uint16_t)strlen(HEARTBEAT_MSG));

            /* Ignoring return values as per system design - Rule 17.7 justification */
            (void)TCPIP_TCP_Flush(sDebugServerSocket);

            (void)SYS_CONSOLE_PRINT("Heartbeat sent via TCP\n");
        }

        #if Two_Wheeler_IO_Aggregator
        if ((sDebugServerSocket != INVALID_SOCKET) &&
            ((uint32_t)TCPIP_UDP_PutIsReady(sDebugServerSocket) >= (uint32_t)strlen(HEARTBEAT_MSG)))
        {
            /* Send heartbeat via UDP */
            (void)TCPIP_UDP_ArrayPut(sDebugServerSocket,
                                     (uint8_t *)HEARTBEAT_MSG,   /* Rule 11.3 deviation */
                                     (uint16_t)strlen(HEARTBEAT_MSG));

            (void)TCPIP_UDP_Flush(sDebugServerSocket);
            (void)TCPIP_UDP_Discard(sDebugServerSocket);

            (void)SYS_CONSOLE_PRINT("Heartbeat sent via UDP\n");
        }
        #endif /* Two_Wheeler_IO_Aggregator */

        /* Status indicators */
        LED_Status_Toggle();   /* Rule 8.7: Function declared and used */
        WDT_Clear();           /* Refresh watchdog */

        /* Delay between heartbeats */
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS));
    }
}

/**
 * @brief  Initializes the common task handler for the server.
 * @return None.
 */
void vCommonTaskHandler(void)
{
    (void)xTaskCreate(vCommonServerTask,
                      "vCommonHandlerServerTask",
                      DEBUG_SERVER_HANDLER_HEAP_DEPTH,
                      NULL,
                      DEBUG_SERVER_HANDLER_TASK_PRIORITY,
                      NULL);

    (void)xTaskCreate(vHeartbeatTask,
                      "vHeartbeatTask",
                      configMINIMAL_STACK_SIZE, /* adjust stack size if needed */
                      NULL,
                      tskIDLE_PRIORITY + 1,
                      NULL);
}


/* *****************************************************************************
 End of File
 */