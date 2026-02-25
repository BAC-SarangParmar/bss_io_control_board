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
// ********** CAN Server Sockets **********
TCP_SOCKET sCan0ServerSocket = INVALID_SOCKET;
TCP_SOCKET sCan1ServerSocket = INVALID_SOCKET;
TCP_SOCKET sCan2ServerSocket = INVALID_SOCKET;
TCP_SOCKET sCan3ServerSocket = INVALID_SOCKET;
TCP_SOCKET sCan4ServerSocket = INVALID_SOCKET;
TCP_SOCKET sCan5ServerSocket = INVALID_SOCKET;


#if Two_Wheeler_IO_Aggregator
 #define CCU_IP_ADDRESS "192.168.1.251"
// ********** Port Definitions **********    
#define CAN0_CCU_PORT     "33001"

#define CAN1_MIRROR1_PORT "33002"
#define CAN1_MIRROR2_PORT "33102"
#define CAN1_MIRROR3_PORT "33105"

#define CAN2_MIRROR1_PORT "33003"
#define CAN2_MIRROR2_PORT "33103"
#define CAN2_MIRROR3_PORT "33105"

#define CAN3_MIRROR1_PORT "33004"
#define CAN3_MIRROR2_PORT "33104"
#define CAN3_MIRROR3_PORT "33105"
#endif
#define CAN_TX_PAYLOAD_SIZE 8
// ********** CAN RX Buffers **********
static uint8_t can0rxFiFo0[CAN0_RX_FIFO0_SIZE];
static uint8_t can1rxFiFo0[CAN1_RX_FIFO0_SIZE];
static uint8_t can2rxFiFo0[CAN2_RX_FIFO0_SIZE];
static uint8_t can3rxFiFo0[CAN3_RX_FIFO0_SIZE];
static uint8_t can4rxFiFo0[CAN4_RX_FIFO0_SIZE];
static uint8_t can5rxFiFo0[CAN5_RX_FIFO0_SIZE];

static uint32_t status = 0;// ********** CAN Data Buffers **********
// ********** CAN Data Buffers **********
static uint8_t u8CAN0buffer[256];
static uint8_t u8CAN1buffer[256];
static uint8_t u8CAN2buffer[256];
static uint8_t u8CAN3buffer[256];
static uint8_t u8CAN4buffer[256];
static uint8_t u8CAN5buffer[256];

// Declare the mutex handle globally
SemaphoreHandle_t xCan0QueueMutex;
SemaphoreHandle_t xCan1QueueMutex;
SemaphoreHandle_t xCan2QueueMutex;
SemaphoreHandle_t xCan3QueueMutex;
SemaphoreHandle_t xCan4QueueMutex;
SemaphoreHandle_t xCan5QueueMutex;

// ********** FreeRTOS Queue Handlers **********
QueueHandle_t xCAN0QueueHandler;
QueueHandle_t xCAN1QueueHandler;
QueueHandle_t xCAN2QueueHandler;
QueueHandle_t xCAN3QueueHandler;
QueueHandle_t xCAN4QueueHandler;
QueueHandle_t xCAN5QueueHandler;
// ********** FreeRTOS Queue Buffers **********
static uint8_t u8CAN0QueueBuffer[128];
static uint8_t u8CAN1QueueBuffer[128];
static uint8_t u8CAN2QueueBuffer[128];
static uint8_t u8CAN3QueueBuffer[128];
static uint8_t u8CAN4QueueBuffer[128];
static uint8_t u8CAN5QueueBuffer[128];


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
        case 3: return CAN3_MessageTransmitFifo(1, &txBuffer);
        case 4: return CAN4_MessageTransmitFifo(1, &txBuffer);
        case 5: return CAN5_MessageTransmitFifo(1, &txBuffer);
        default:
            SYS_CONSOLE_PRINT("Invalid CAN index.\r\n");
            return false;
    }
}

bool CAN0_Write(uint8_t *u8data, char i8len)
{
    CAN_TX_BUFFER txBuffer;
    memset(&txBuffer, 0, sizeof(CAN_TX_BUFFER));

    uint32_t canId = ((uint32_t)u8data[0] << 24) |
                     ((uint32_t)u8data[1] << 16) |
                     ((uint32_t)u8data[2] << 8)  |
                     ((uint32_t)u8data[3]);

    txBuffer.id = WRITE_ID(canId);
    txBuffer.dlc = CAN_TX_PAYLOAD_SIZE;

    for (uint8_t i = 0; i < CAN_TX_PAYLOAD_SIZE; i++)
    {
        txBuffer.data[i] = u8data[i + 4];
    }

    bool result = CAN0_MessageTransmitFifo(1, &txBuffer);
    if (!result)
    {
        SYS_CONSOLE_PRINT("Error: CAN0 message transmission failed!\r\n");
    }
    return result;
}

bool CAN1_Write(uint8_t *u8data, char i8len)
{
    CAN_TX_BUFFER txBuffer;
    memset(&txBuffer, 0, sizeof(CAN_TX_BUFFER));

    uint32_t canId = ((uint32_t)u8data[0] << 24) |
                     ((uint32_t)u8data[1] << 16) |
                     ((uint32_t)u8data[2] << 8)  |
                     ((uint32_t)u8data[3]);

    txBuffer.id = WRITE_ID(canId);
    txBuffer.dlc = CAN_TX_PAYLOAD_SIZE;

    for (uint8_t i = 0; i < CAN_TX_PAYLOAD_SIZE; i++)
    {
        txBuffer.data[i] = u8data[i + 4];
    }

    bool result = CAN1_MessageTransmitFifo(1, &txBuffer);
    if (!result)
    {
        SYS_CONSOLE_PRINT("Error: CAN1 message transmission failed!\r\n");
    }
    return result;
}

bool CAN2_Write(uint8_t *u8data, char i8len)
{
    CAN_TX_BUFFER txBuffer;
    memset(&txBuffer, 0, sizeof(CAN_TX_BUFFER));

    uint32_t canId = ((uint32_t)u8data[0] << 24) |
                     ((uint32_t)u8data[1] << 16) |
                     ((uint32_t)u8data[2] << 8)  |
                     ((uint32_t)u8data[3]);

    txBuffer.id = WRITE_ID(canId);
    txBuffer.dlc = CAN_TX_PAYLOAD_SIZE;

    for (uint8_t i = 0; i < CAN_TX_PAYLOAD_SIZE; i++)
    {
        txBuffer.data[i] = u8data[i + 4];
    }

    bool result = CAN2_MessageTransmitFifo(1, &txBuffer);
    if (!result)
    {
        SYS_CONSOLE_PRINT("Error: CAN2 message transmission failed!\r\n");
    }
    return result;
}

bool CAN3_Write(uint8_t *u8data, char i8len)
{
    CAN_TX_BUFFER txBuffer;
    memset(&txBuffer, 0, sizeof(CAN_TX_BUFFER));

    uint32_t canId = ((uint32_t)u8data[0] << 24) |
                     ((uint32_t)u8data[1] << 16) |
                     ((uint32_t)u8data[2] << 8)  |
                     ((uint32_t)u8data[3]);

    txBuffer.id = WRITE_ID(canId);
    txBuffer.dlc = CAN_TX_PAYLOAD_SIZE;

    for (uint8_t i = 0; i < CAN_TX_PAYLOAD_SIZE; i++)
    {
        txBuffer.data[i] = u8data[i + 4];
    }

    bool result = CAN3_MessageTransmitFifo(1, &txBuffer);
    if (!result)
    {
        SYS_CONSOLE_PRINT("Error: CAN3 message transmission failed!\r\n");
    }
    return result;
}

bool CAN4_Write(uint8_t *u8data, char i8len)
{
    CAN_TX_BUFFER txBuffer;
    memset(&txBuffer, 0, sizeof(CAN_TX_BUFFER));

    uint32_t canId = ((uint32_t)u8data[0] << 24) |
                     ((uint32_t)u8data[1] << 16) |
                     ((uint32_t)u8data[2] << 8)  |
                     ((uint32_t)u8data[3]);

    txBuffer.id = WRITE_ID(canId);
    txBuffer.dlc = CAN_TX_PAYLOAD_SIZE;

    for (uint8_t i = 0; i < CAN_TX_PAYLOAD_SIZE; i++)
    {
        txBuffer.data[i] = u8data[i + 4];
    }

    bool result = CAN4_MessageTransmitFifo(1, &txBuffer);
    if (!result)
    {
        SYS_CONSOLE_PRINT("Error: CAN4 message transmission failed!\r\n");
    }
    return result;
}

bool CAN5_Write(uint8_t *u8data, char i8len)
{
    CAN_TX_BUFFER txBuffer;
    memset(&txBuffer, 0, sizeof(CAN_TX_BUFFER));

    uint32_t canId = ((uint32_t)u8data[0] << 24) |
                     ((uint32_t)u8data[1] << 16) |
                     ((uint32_t)u8data[2] << 8)  |
                     ((uint32_t)u8data[3]);

    txBuffer.id = WRITE_ID(canId);
    txBuffer.dlc = CAN_TX_PAYLOAD_SIZE;

    for (uint8_t i = 0; i < CAN_TX_PAYLOAD_SIZE; i++)
    {
        txBuffer.data[i] = u8data[i + 4];
    }

    bool result = CAN5_MessageTransmitFifo(1, &txBuffer);
    if (!result)
    {
        SYS_CONSOLE_PRINT("Error: CAN5 message transmission failed!\r\n");
    }
    return result;
}
/*
 * @brief Transmit received CAN messages over TCP
 * 
 * This function processes received CAN messages and transmits them over a TCP socket.
 * It ensures compliance with MISRA standards, including type safety, pointer arithmetic,
 * and avoiding potential run-time issues.
 * 
 * @param numberOfMessage Number of CAN messages to process
 * @param rxBuf Pointer to the received CAN buffer
 * @param rxBufLen Length of each received CAN message
 * @param sCanServerSocket TCP socket for transmission
 */
void vCanRxMessageTxTCP(uint8_t numberOfMessage, CAN_RX_BUFFER *rxBuf, uint8_t rxBufLen, TCP_SOCKET sCanServerSocket)
{
    uint32_t id = 0;
    uint8_t tcpBuffer[13U]; /* 1 byte header + 4 bytes CAN ID + 8 bytes CAN Data */

    if ((rxBuf == NULL) || (numberOfMessage == 0U))
    {
        return; /* Prevent null pointer dereference and unnecessary execution */
    }
    for (uint8_t count = 0; count < numberOfMessage; count++)
    {
        id = rxBuf->xtd ? rxBuf->id : READ_ID(rxBuf->id);
        id = swap_endian_32(id);/* Convert CAN ID to big-endian format */

        /* Construct the extra byte (4 bits CAN type, 4 bits payload length) */
        uint8_t headerByte = (rxBuf->xtd != 0U) ? 0x8U : 0x0U;
        headerByte |= 0x8U; /* Payload length fixed to 8 bytes */

        /* Copy header byte, CAN ID, and Data to buffer */
        tcpBuffer[0] = headerByte;
        (void)memcpy(&tcpBuffer[1], &id, sizeof(id));
        (void)memcpy(&tcpBuffer[5], rxBuf->data, 8U);       
        // Transmit over TCP if connected
        if (TCPIP_TCP_IsConnected(sCanServerSocket))
        {
            TCPIP_TCP_ArrayPut(sCanServerSocket, tcpBuffer, sizeof(tcpBuffer));
            TCPIP_TCP_Flush(sCanServerSocket);

            // Debug Output
//            SYS_CONSOLE_PRINT("Transmitting over Ethernet -> Header: 0x%x, CAN ID: 0x%x, Data: ", headerByte, (unsigned int)id);
//            for (uint8_t i = 0; i < 8; i++)
//            {
//                SYS_CONSOLE_PRINT("0x%x ", rxBuf->data[i]);
//            }
//            SYS_CONSOLE_PRINT("\r\n");
        }
        rxBuf += rxBufLen;  // Move to next message after processing                
    }
}

/**********************************************************************
 * @brief   Sends a UDP packet containing CAN message data
 * 
 * @param   ipAddress         Pointer to a string containing the destination IP address
 * @param   port              Pointer to a string containing the destination port number
 * @param   rxBuf             Pointer to the CAN receive buffer containing messages
 * @param   numberOfMessage   Number of CAN messages to send
 * 
 * @details This function converts a given IP address string into an IPV4 address,
 *          opens a UDP client socket, and transmits CAN messages over UDP.
 *          Each message includes a 1-byte header, a 4-byte CAN ID, and an 8-byte payload.
 **********************************************************************/
void Send_UDP_Packet(const char* ipAddress, const char* port, CAN_RX_BUFFER *rxBuf, uint8_t numberOfMessage)
{
    IPV4_ADDR addr;          /* Structure to store the destination IP address */
    uint16_t udpPort;        /* UDP port number */
    UDP_SOCKET udpSocket;    /* UDP socket handle */
    uint8_t tcpBuffer[13U];  /* Buffer for UDP payload: header + CAN ID + CAN data */
    uint32_t id;             /* Variable to store the CAN ID */
    uint8_t headerByte;      /* Header byte for UDP packet */
    uint8_t count;           /* Loop counter */

    /* Convert port string to integer */
    udpPort = (uint16_t)atoi(port);

    /* Convert string IP address to IPV4 address structure */
    if (TCPIP_Helper_StringToIPAddress(ipAddress, &addr) == false)
    {
        SYS_CONSOLE_PRINT("Invalid IP address: %s\r\n", ipAddress);
        return;  /* Exit function if IP address conversion fails */
    }

    /* Open a UDP client socket */
    udpSocket = TCPIP_UDP_ClientOpen(IP_ADDRESS_TYPE_IPV4, udpPort, (IP_MULTI_ADDRESS*)&addr);
    if (udpSocket == INVALID_SOCKET)
    {
        SYS_CONSOLE_PRINT("Failed to open UDP socket\r\n");
        return;  /* Exit function if socket opening fails */
    }

    /* Loop through each CAN message */
    for (count = 0U; count < numberOfMessage; count++)
    {
        /* Extract and convert CAN ID to big-endian format */
        id = (rxBuf->xtd != 0U) ? rxBuf->id : READ_ID(rxBuf->id);
        id = swap_endian_32(id);

        /* Construct the extra byte (4 bits CAN type, 4 bits payload length) */
        headerByte = (rxBuf->xtd != 0U) ? 0x8U : 0x0U;
        headerByte |= 0x8U; /* Payload length fixed to 8 bytes */

        /* Copy header byte, CAN ID, and data to the transmission buffer */
        tcpBuffer[0] = headerByte;
        (void)memcpy(&tcpBuffer[1], &id, sizeof(id));
        (void)memcpy(&tcpBuffer[5], rxBuf->data, 8U);

        /* Check if socket is ready before sending data */
        if (TCPIP_UDP_PutIsReady(udpSocket) >= sizeof(tcpBuffer))
        {
            (void)TCPIP_UDP_ArrayPut(udpSocket, tcpBuffer, sizeof(tcpBuffer));
            (void)TCPIP_UDP_Flush(udpSocket);
            
            SYS_CONSOLE_PRINT("UDP Packet sent to %s:%s -> CAN ID: 0x%x, Data: ", ipAddress, port, (unsigned int)id);

            /* Print CAN message data */
            for (uint8_t i = 0U; i < 8U; i++)
            {
                SYS_CONSOLE_PRINT("0x%x ", rxBuf->data[i]);
            }
            SYS_CONSOLE_PRINT("\r\n");
        }
        else
        {
            SYS_CONSOLE_PRINT("UDP socket not ready for sending\r\n");
        }

        rxBuf++; /* Move to the next CAN message */
    }

    /* Close the UDP socket after transmitting all messages */
    TCPIP_UDP_Close(udpSocket);
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
    SYS_CONSOLE_PRINT("In Function: %s\r\n", __FUNCTION__);

    while (true) {
        #if HEV_IO_Aggregator
            /* Handle TCP connection for CAN0 Server */
            if (TCPIP_TCP_IsConnected(sCan0ServerSocket)) {
                int16_t bytesRead = TCPIP_TCP_ArrayGet(sCan0ServerSocket, u8CAN0buffer, sizeof(u8CAN0buffer));
                /* Check if data was received */
                if (bytesRead > 0) {
                    SYS_CONSOLE_PRINT("Got Data on CAN0 Server Handler - Data Send to CAN0\r\n");
                    CAN_Write(0,(uint8_t*)u8CAN0buffer, (uint8_t)bytesRead);
//                    CAN0_Write((uint8_t*)u8CAN0buffer, (uint8_t)bytesRead);
                }
                /* Handle disconnection scenarios */
                if (!TCPIP_TCP_IsConnected(sCan0ServerSocket) || TCPIP_TCP_WasDisconnected(sCan0ServerSocket)) {
                    SYS_CONSOLE_PRINT("\r\nTCP Connection Closed\r\n");
                    TCPIP_TCP_Close(sCan0ServerSocket);
                    sCan0ServerSocket = INVALID_SOCKET;
                }
            }

            /* Reconnection logic if socket is invalid */
            if (sCan0ServerSocket == INVALID_SOCKET) {
                sCan0ServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, writeData.canPorts[0], 0);

                if (sCan0ServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen CAN0 socket, retrying...\r\n");
                } 
                vTaskDelay(RECONNECT_DELAY_MS);
            }
            /* Process CAN0 queue messages and transmit via TCP */
            xSemaphoreTake(xCan0QueueMutex, portMAX_DELAY);
            if (xQueueReceive(xCAN0QueueHandler, &u8CAN0QueueBuffer, (TickType_t) 10) == pdPASS) {
                vCanRxMessageTxTCP(1, (CAN_RX_BUFFER*)&u8CAN0QueueBuffer, sizeof(CAN_RX_BUFFER), sCan0ServerSocket);                
            }
            xSemaphoreGive(xCan0QueueMutex);            
        #elif Two_Wheeler_IO_Aggregator
            /* Handle UDP connection for CAN0 Server */
            if (TCPIP_UDP_IsConnected(sCan0ServerSocket)) {
                int16_t bytesRead = TCPIP_UDP_ArrayGet(sCan0ServerSocket, u8CAN0buffer, sizeof(u8CAN0buffer));
                /* Check if data was received */
                if (bytesRead > 0) {
                    SYS_CONSOLE_PRINT("Got Data on CAN0 Server Handler - Data Send to CAN0\r\n");
//                    CAN0_Write((uint8_t*)u8CAN0buffer, (uint8_t)bytesRead);
                    CAN_Write(0,(uint8_t*)u8CAN0buffer, (uint8_t)bytesRead);
                }
            }
            /* Reconnection logic if socket is invalid */
            if (sCan0ServerSocket == INVALID_SOCKET) {
                sCan0ServerSocket = TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE_IPV4, CAN0_SERVER_PORT, 0);

                if (sCan0ServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen CAN0 socket, retrying...\r\n");
                } 
                vTaskDelay(RECONNECT_DELAY_MS);
            }
            /* Process CAN0 queue messages and transmit via UDP */
            if( xQueueReceive(xCAN0QueueHandler, &u8CAN0QueueBuffer, (TickType_t) 10) == pdPASS )
            {
//                vCanRxMessageTxTCP_UDP(1, (CAN_RX_BUFFER*)&u8CAN0QueueBuffer, sizeof(CAN_RX_BUFFER), sCan0ServerSocket); 
//                vCanRxMessageTx_UDP(1, (CAN_RX_BUFFER*)&u8CAN0QueueBuffer, sizeof(CAN_RX_BUFFER), sCan0ServerSocket);
                Send_UDP_Packet(CCU_IP_ADDRESS, CAN0_CCU_PORT, (CAN_RX_BUFFER*)u8CAN0QueueBuffer, 1);
            } 
            
        #endif
        vTaskDelay(10); // Add a small delay to avoid consuming too much CPU time
    }
}

/**
 * @brief CAN1 Server Task
 * 
 * This FreeRTOS task handles TCP/UDP communication for CAN1.
 * It manages incoming connections, reads data from the socket,
 * transmits it to the CAN bus, and processes CAN queue messages
 * to send over the network. The task also ensures reconnection
 * if the socket is closed or disconnected.
 * 
 * @param[in] pvParameters Pointer to task parameters (unused).
 */
void vCan1HandlerServerTask(void *pvParameters) {
    SYS_CONSOLE_PRINT("In Function: %s\r\n", __FUNCTION__);

    while (true) {
        #if HEV_IO_Aggregator
            // CAN1 Server Handling
            if (TCPIP_TCP_IsConnected(sCan1ServerSocket)) {
                int16_t bytesRead = TCPIP_TCP_ArrayGet(sCan1ServerSocket, u8CAN1buffer, sizeof(u8CAN1buffer));

                if (bytesRead > 0) {
                    SYS_CONSOLE_PRINT("Got Data on CAN1 Server Handler - Data Send to CAN1\r\n");
//                    CAN1_Write((uint8_t*)u8CAN1buffer, (uint8_t)bytesRead);
                    CAN_Write(1,(uint8_t*)u8CAN1buffer, (uint8_t)bytesRead);
                }

                if (!TCPIP_TCP_IsConnected(sCan1ServerSocket) || TCPIP_TCP_WasDisconnected(sCan1ServerSocket)) {
                    SYS_CONSOLE_PRINT("\r\nTCP Connection Closed\r\n");
                    TCPIP_TCP_Close(sCan1ServerSocket);
                    sCan1ServerSocket = INVALID_SOCKET;
                }
            }

            // Ensure reconnection logic is handled correctly
            if (sCan1ServerSocket == INVALID_SOCKET) {
                sCan1ServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, writeData.canPorts[1], 0);

                if (sCan1ServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen CAN1 socket, retrying...\r\n");
                } 
                vTaskDelay(RECONNECT_DELAY_MS);
            }
            xSemaphoreTake(xCan1QueueMutex, portMAX_DELAY);
            if (xQueueReceive(xCAN1QueueHandler, &u8CAN1QueueBuffer, (TickType_t) 10) == pdPASS) {
                vCanRxMessageTxTCP(1, (CAN_RX_BUFFER*)&u8CAN1QueueBuffer, sizeof(CAN_RX_BUFFER), sCan1ServerSocket);                
            }
            xSemaphoreGive(xCan1QueueMutex);   
        #elif Two_Wheeler_IO_Aggregator
            if (TCPIP_UDP_IsConnected(sCan1ServerSocket)) {
                int16_t bytesRead = TCPIP_UDP_ArrayGet(sCan1ServerSocket, u8CAN1buffer, sizeof(u8CAN1buffer));
                if (bytesRead > 0) {
                    SYS_CONSOLE_PRINT("Got Data on CAN1 Server Handler - Data Send to CAN1\r\n");
//                    CAN1_Write((uint8_t*)u8CAN1buffer, (uint8_t)bytesRead);
                    CAN_Write(1,(uint8_t*)u8CAN1buffer, (uint8_t)bytesRead);
                }
            }
            // Ensure reconnection logic is handled correctly
            if (sCan1ServerSocket == INVALID_SOCKET) {
                sCan1ServerSocket = TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE_IPV4, CAN1_SERVER_PORT, 0);

                if (sCan1ServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen CAN1 socket, retrying...\r\n");
                } 
                vTaskDelay(RECONNECT_DELAY_MS);
            }
            if( xQueueReceive(xCAN1QueueHandler, &u8CAN1QueueBuffer, (TickType_t) 10) == pdPASS )
            {
                Send_UDP_Packet(CCU_IP_ADDRESS, CAN1_MIRROR1_PORT, (CAN_RX_BUFFER*)u8CAN1QueueBuffer, 1);
                Send_UDP_Packet(CCU_IP_ADDRESS, CAN1_MIRROR2_PORT, (CAN_RX_BUFFER*)u8CAN1QueueBuffer, 1);
                Send_UDP_Packet(CCU_IP_ADDRESS, CAN1_MIRROR3_PORT, (CAN_RX_BUFFER*)u8CAN1QueueBuffer, 1);
            }                         
        #endif

        vTaskDelay(10); // Add a small delay to avoid consuming too much CPU time
    }
}

/**
 * @brief CAN2 Server Task
 * 
 * This FreeRTOS task handles TCP/UDP communication for CAN2.
 * It manages incoming connections, reads data from the socket,
 * transmits it to the CAN bus, and processes CAN queue messages
 * to send over the network. The task also ensures reconnection
 * if the socket is closed or disconnected.
 * 
 * @param[in] pvParameters Pointer to task parameters (unused).
 */
void vCan2HandlerServerTask(void *pvParameters) {
    SYS_CONSOLE_PRINT("In Function: %s\r\n", __FUNCTION__);
    
    while (true) {
        #if HEV_IO_Aggregator
           if (TCPIP_TCP_IsConnected(sCan2ServerSocket)) {
                int16_t bytesRead = TCPIP_TCP_ArrayGet(sCan2ServerSocket, u8CAN2buffer, sizeof(u8CAN2buffer));

                if (bytesRead >= 12) {  // Expecting at least 4 bytes CAN ID + 8 bytes CAN Data
                    SYS_CONSOLE_PRINT("Got Data on CAN2 Server Handler - Data Send to CAN2\r\n");
//                    CAN2_Write((uint8_t*)u8CAN2buffer, (uint8_t)bytesRead);
                    CAN_Write(2,(uint8_t*)u8CAN2buffer, (uint8_t)bytesRead);
                }

                // Handle TCP connection loss
                if (!TCPIP_TCP_IsConnected(sCan2ServerSocket) || TCPIP_TCP_WasDisconnected(sCan2ServerSocket)) {
                    SYS_CONSOLE_PRINT("\r\nCAN2 TCP Connection Closed\r\n");
                    TCPIP_TCP_Close(sCan2ServerSocket);
                    sCan2ServerSocket = INVALID_SOCKET;
                }
            }                       
            // Ensure reconnection logic is handled correctly
            if (sCan2ServerSocket == INVALID_SOCKET) {
                // Attempt to open the socket again
                sCan2ServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, writeData.canPorts[2], 0);

                if (sCan2ServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen CAN2 socket, retrying...\r\n");
                }
                vTaskDelay(RECONNECT_DELAY_MS);
            }
            xSemaphoreTake(xCan2QueueMutex, portMAX_DELAY);
            if (xQueueReceive(xCAN2QueueHandler, &u8CAN2QueueBuffer, (TickType_t) 10) == pdPASS) {
                vCanRxMessageTxTCP(1, (CAN_RX_BUFFER*)&u8CAN2QueueBuffer, sizeof(CAN_RX_BUFFER), sCan2ServerSocket);                
            }
            xSemaphoreGive(xCan2QueueMutex);           
        #elif Two_Wheeler_IO_Aggregator
            if (TCPIP_UDP_IsConnected(sCan2ServerSocket)) {
                int16_t bytesRead = TCPIP_UDP_ArrayGet(sCan2ServerSocket, u8CAN2buffer, sizeof(u8CAN2buffer));
                if (bytesRead > 0) {
                    SYS_CONSOLE_PRINT("Got Data on CAN2 Server Handler - Data Send to CAN2\r\n");
//                    CAN2_Write((uint8_t*)u8CAN2buffer, (uint8_t)bytesRead);
                    CAN_Write(2,(uint8_t*)u8CAN2buffer, (uint8_t)bytesRead);
                }
            }
            // Ensure reconnection logic is handled correctly
            if (sCan2ServerSocket == INVALID_SOCKET) {
                sCan2ServerSocket = TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE_IPV4, CAN2_SERVER_PORT, 0);

                if (sCan2ServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen CAN2 socket, retrying...\r\n");
                } 
                vTaskDelay(RECONNECT_DELAY_MS);
            }
            if( xQueueReceive(xCAN2QueueHandler, &u8CAN2QueueBuffer, (TickType_t) 10) == pdPASS )
            {
                Send_UDP_Packet(CCU_IP_ADDRESS, CAN2_MIRROR1_PORT, (CAN_RX_BUFFER*)u8CAN2QueueBuffer, 1);
                Send_UDP_Packet(CCU_IP_ADDRESS, CAN2_MIRROR2_PORT, (CAN_RX_BUFFER*)u8CAN2QueueBuffer, 1);
                Send_UDP_Packet(CCU_IP_ADDRESS, CAN2_MIRROR3_PORT, (CAN_RX_BUFFER*)u8CAN2QueueBuffer, 1);              
            }                       
        #endif
        
        vTaskDelay(10); // Add a small delay to avoid consuming too much CPU time
    }
}

/**
 * @brief CAN3 Server Task
 * 
 * This FreeRTOS task handles TCP/UDP communication for CAN3.
 * It manages incoming connections, reads data from the socket,
 * transmits it to the CAN bus, and processes CAN queue messages
 * to send over the network. The task also ensures reconnection
 * if the socket is closed or disconnected.
 * 
 * @param[in] pvParameters Pointer to task parameters (unused).
 */
void vCan3HandlerServerTask(void *pvParameters) {
    SYS_CONSOLE_PRINT("In Function: %s\r\n", __FUNCTION__);

    while (true) {
        #if HEV_IO_Aggregator
            // CAN3 Server Handling
            if (TCPIP_TCP_IsConnected(sCan3ServerSocket)) {
                int16_t bytesRead = TCPIP_TCP_ArrayGet(sCan3ServerSocket, u8CAN3buffer, sizeof(u8CAN3buffer));

                if (bytesRead > 0) {
                    SYS_CONSOLE_PRINT("Got Data on CAN3 Server Handler - Data Send to CAN3\r\n");
//                    CAN3_Write((uint8_t*)u8CAN3buffer, (uint8_t)bytesRead);
                    CAN_Write(3,(uint8_t*)u8CAN3buffer, (uint8_t)bytesRead);
                }

                if (!TCPIP_TCP_IsConnected(sCan3ServerSocket) || TCPIP_TCP_WasDisconnected(sCan3ServerSocket)) {
                    SYS_CONSOLE_PRINT("\r\nTCP Connection Closed\r\n");
                    TCPIP_TCP_Close(sCan3ServerSocket);
                    sCan3ServerSocket = INVALID_SOCKET;
                }
            }

            // Ensure reconnection logic is handled correctly
            if (sCan3ServerSocket == INVALID_SOCKET) {
                sCan3ServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, writeData.canPorts[3], 0);

                if (sCan3ServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen CAN3 socket, retrying...\r\n");
                } 
                vTaskDelay(RECONNECT_DELAY_MS);
            }
            xSemaphoreTake(xCan3QueueMutex, portMAX_DELAY);
            if (xQueueReceive(xCAN3QueueHandler, &u8CAN3QueueBuffer, (TickType_t) 10) == pdPASS) {
                vCanRxMessageTxTCP(1, (CAN_RX_BUFFER*)&u8CAN3QueueBuffer, sizeof(CAN_RX_BUFFER), sCan3ServerSocket);                
            }
            xSemaphoreGive(xCan3QueueMutex);             
        #elif Two_Wheeler_IO_Aggregator
            if (TCPIP_UDP_IsConnected(sCan3ServerSocket)) {
                int16_t bytesRead = TCPIP_UDP_ArrayGet(sCan3ServerSocket, u8CAN3buffer, sizeof(u8CAN3buffer));
                if (bytesRead > 0) {
                    SYS_CONSOLE_PRINT("Got Data on CAN3 Server Handler - Data Send to CAN3\r\n");
//                    CAN3_Write((uint8_t*)u8CAN3buffer, (uint8_t)bytesRead);
                    CAN_Write(3,(uint8_t*)u8CAN3buffer, (uint8_t)bytesRead);
                }
            }
            // Ensure reconnection logic is handled correctly
            if (sCan3ServerSocket == INVALID_SOCKET) {
                sCan3ServerSocket = TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE_IPV4, CAN3_SERVER_PORT, 0);

                if (sCan3ServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen CAN3 socket, retrying...\r\n");
                } 
                vTaskDelay(RECONNECT_DELAY_MS);
            }
            if( xQueueReceive(xCAN3QueueHandler, &u8CAN3QueueBuffer, (TickType_t) 10) == pdPASS )
            {
                Send_UDP_Packet(CCU_IP_ADDRESS, CAN3_MIRROR1_PORT, (CAN_RX_BUFFER*)u8CAN3QueueBuffer, 1);
                Send_UDP_Packet(CCU_IP_ADDRESS, CAN3_MIRROR2_PORT, (CAN_RX_BUFFER*)u8CAN3QueueBuffer, 1);
                Send_UDP_Packet(CCU_IP_ADDRESS, CAN3_MIRROR3_PORT, (CAN_RX_BUFFER*)u8CAN3QueueBuffer, 1);                
            }                         
        #endif

        vTaskDelay(10); // Add a small delay to avoid consuming too much CPU time
    }
}

/**
 * @brief CAN4 Server Task
 * 
 * This FreeRTOS task handles TCP/UDP communication for CAN4.
 * It manages incoming connections, reads data from the socket,
 * transmits it to the CAN bus, and processes CAN queue messages
 * to send over the network. The task also ensures reconnection
 * if the socket is closed or disconnected.
 * 
 * @param[in] pvParameters Pointer to task parameters (unused).
 */
void vCan4HandlerServerTask(void *pvParameters) {
    SYS_CONSOLE_PRINT("In Function: %s\r\n", __FUNCTION__);

    while (true) {
        #if HEV_IO_Aggregator
            // CAN4 Server Handling
            if (TCPIP_TCP_IsConnected(sCan4ServerSocket)) {
                int16_t bytesRead = TCPIP_TCP_ArrayGet(sCan4ServerSocket, u8CAN4buffer, sizeof(u8CAN4buffer));

                if (bytesRead > 0) {
                    SYS_CONSOLE_PRINT("Got Data on CAN4 Server Handler - Data Send to CAN4\r\n");
//                    CAN4_Write((uint8_t*)u8CAN4buffer, (uint8_t)bytesRead);
                    CAN_Write(4,(uint8_t*)u8CAN4buffer, (uint8_t)bytesRead);
                }

                if (!TCPIP_TCP_IsConnected(sCan4ServerSocket) || TCPIP_TCP_WasDisconnected(sCan4ServerSocket)) {
                    SYS_CONSOLE_PRINT("\r\nTCP Connection Closed\r\n");
                    TCPIP_TCP_Close(sCan4ServerSocket);
                    sCan4ServerSocket = INVALID_SOCKET;
                }
            }

            // Ensure reconnection logic is handled correctly
            if (sCan4ServerSocket == INVALID_SOCKET) {
                sCan4ServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, writeData.canPorts[4], 0);

                if (sCan4ServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen CAN4 socket, retrying...\r\n");
                } 
                vTaskDelay(RECONNECT_DELAY_MS);
            }
            xSemaphoreTake(xCan4QueueMutex, portMAX_DELAY);
            if (xQueueReceive(xCAN4QueueHandler, &u8CAN4QueueBuffer, (TickType_t) 10) == pdPASS) {
                vCanRxMessageTxTCP(1, (CAN_RX_BUFFER*)&u8CAN4QueueBuffer, sizeof(CAN_RX_BUFFER), sCan4ServerSocket);                
            }
            xSemaphoreGive(xCan4QueueMutex);             
        #elif Two_Wheeler_IO_Aggregator
            if (TCPIP_UDP_IsConnected(sCan4ServerSocket)) {
                int16_t bytesRead = TCPIP_UDP_ArrayGet(sCan4ServerSocket, u8CAN4buffer, sizeof(u8CAN4buffer));
                if (bytesRead > 0) {
                    SYS_CONSOLE_PRINT("Got Data on CAN4 Server Handler - Data Send to CAN4\r\n");
//                    CAN4_Write((uint8_t*)u8CAN4buffer, (uint8_t)bytesRead);
                    CAN_Write(4,(uint8_t*)u8CAN4buffer, (uint8_t)bytesRead);
                }
            }
            // Ensure reconnection logic is handled correctly
            if (sCan4ServerSocket == INVALID_SOCKET) {
                sCan4ServerSocket = TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE_IPV4, CAN4_SERVER_PORT, 0);

                if (sCan4ServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen CAN0 socket, retrying...\r\n");
                } 
                vTaskDelay(RECONNECT_DELAY_MS);
            }
            if( xQueueReceive(xCAN4QueueHandler, &u8CAN4QueueBuffer, (TickType_t) 10) == pdPASS )
            {
//                vCanRxMessageTxTCP_UDP(1, (CAN_RX_BUFFER*)&u8CAN4QueueBuffer, sizeof(CAN_RX_BUFFER), sCan4ServerSocket);               
            }             
        #endif

        vTaskDelay(10); // Add a small delay to avoid consuming too much CPU time
    }
}

/**
 * @brief CAN5 Server Task
 * 
 * This FreeRTOS task handles TCP/UDP communication for CAN5.
 * It manages incoming connections, reads data from the socket,
 * transmits it to the CAN bus, and processes CAN queue messages
 * to send over the network. The task also ensures reconnection
 * if the socket is closed or disconnected.
 * 
 * @param[in] pvParameters Pointer to task parameters (unused).
 */
void vCan5HandlerServerTask(void *pvParameters) {
    SYS_CONSOLE_PRINT("In Function: %s\r\n", __FUNCTION__);
    
    while (true) {
        #if HEV_IO_Aggregator
            // CAN5 Server Handling
            if (TCPIP_TCP_IsConnected(sCan5ServerSocket)) {
                int16_t bytesRead = TCPIP_TCP_ArrayGet(sCan5ServerSocket, u8CAN5buffer, sizeof(u8CAN5buffer));

                if (bytesRead > 0) {
                    SYS_CONSOLE_PRINT("Got Data on CAN5 Server Handler - Data Send to CAN5\r\n");
//                    CAN5_Write((uint8_t*)u8CAN5buffer, (uint8_t)bytesRead);
                    CAN_Write(5,(uint8_t*)u8CAN5buffer, (uint8_t)bytesRead);
                }

                if (!TCPIP_TCP_IsConnected(sCan5ServerSocket) || TCPIP_TCP_WasDisconnected(sCan5ServerSocket)) {
                    SYS_CONSOLE_PRINT("\r\nTCP Connection Closed\r\n");
                    TCPIP_TCP_Close(sCan5ServerSocket);
                    sCan5ServerSocket = INVALID_SOCKET;
                }
            }

            // Ensure reconnection logic is handled correctly
            if (sCan5ServerSocket == INVALID_SOCKET) {
                sCan5ServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, writeData.canPorts[5], 0);

                if (sCan5ServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen CAN5 socket, retrying...\r\n");
                }
                vTaskDelay(RECONNECT_DELAY_MS);
            } 
            xSemaphoreTake(xCan5QueueMutex, portMAX_DELAY);
            if( xQueueReceive(xCAN5QueueHandler, &u8CAN5QueueBuffer, (TickType_t) 10) == pdPASS ) {
                vCanRxMessageTxTCP(1, (CAN_RX_BUFFER*)&u8CAN5QueueBuffer, sizeof(CAN_RX_BUFFER), sCan5ServerSocket);               
            }
            xSemaphoreGive(xCan5QueueMutex);             
        #elif Two_Wheeler_IO_Aggregator
            if (TCPIP_UDP_IsConnected(sCan5ServerSocket)) {
                int16_t bytesRead = TCPIP_UDP_ArrayGet(sCan5ServerSocket, u8CAN5buffer, sizeof(u8CAN5buffer));
                if (bytesRead > 0) {
                    SYS_CONSOLE_PRINT("Got Data on CAN5 Server Handler - Data Send to CAN5\r\n");
//                    CAN5_Write((uint8_t*)u8CAN5buffer, (uint8_t)bytesRead);
                    CAN_Write(5,(uint8_t*)u8CAN5buffer, (uint8_t)bytesRead);
                }
            }
            // Ensure reconnection logic is handled correctly
            if (sCan5ServerSocket == INVALID_SOCKET) {
                sCan5ServerSocket = TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE_IPV4, CAN5_SERVER_PORT, 0);

                if (sCan5ServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen CAN5 socket, retrying...\r\n");
                } 
                vTaskDelay(RECONNECT_DELAY_MS);
            }
            if( xQueueReceive(xCAN5QueueHandler, &u8CAN5QueueBuffer, (TickType_t) 10) == pdPASS )
            {
//                vCanRxMessageTxTCP_UDP(1, (CAN_RX_BUFFER*)&u8CAN5QueueBuffer, sizeof(CAN_RX_BUFFER), sCan5ServerSocket);                
            }             
        #endif
        
        vTaskDelay(10); // Add a small delay to avoid consuming too much CPU time
    }
}

static void vDisplayCanRxMessage(uint8_t numberOfMessage, CAN_RX_BUFFER *rxBuf, uint8_t rxBufLen)
{
    uint8_t length = 0;
    uint8_t msgLength = 0;
    uint32_t id = 0;

    for (uint8_t count = 0; count < numberOfMessage; count++)
    {
        id = rxBuf->xtd ? rxBuf->id : READ_ID(rxBuf->id);
        msgLength = rxBuf->dlc;
        length = msgLength;
        SYS_CONSOLE_PRINT(" Message - ID : 0x%x Length : 0x%x ", (unsigned int)id, (unsigned int)msgLength);
        SYS_CONSOLE_PRINT("Message : ");
        while(length)
        {
            SYS_CONSOLE_PRINT("0x%x ", rxBuf->data[msgLength - length--]);
        }
        SYS_CONSOLE_PRINT("\r\n");
        rxBuf += rxBufLen;
    }
}

void vCanRxHandlerTask(void *pvParameters) {
    uint8_t numberOfMessage = 0; 
    
    while(true) {       
        if (CAN0_InterruptGet(CAN_INTERRUPT_RF0N_MASK))
        {    
            CAN0_InterruptClear(CAN_INTERRUPT_RF0N_MASK);

            /* Check CAN Status */
            status = CAN0_ErrorGet();
            /* Decode and print it */
            SYS_CONSOLE_PRINT("CAN0 status: 0x%08lX\r\n", (unsigned long)status);
            /* -------- Last-Error-Code (LEC) field -------- */
            switch (status & 0x7u)
            {
//                case 0: SYS_CONSOLE_PRINT("  ? No error\n"); break;
                case 1: SYS_CONSOLE_PRINT("  ? Stuff error\n"); ReportError(ERR_CAN1_STUFF); break;
                case 2: SYS_CONSOLE_PRINT("  ? Form error\n"); break;
                case 3: SYS_CONSOLE_PRINT("  ? ACK error\n"); ReportError(ERR_CAN1_ACK); break;
                case 4: SYS_CONSOLE_PRINT("  ? Bit-1 error (recessive expected)\n"); break;
                case 5: SYS_CONSOLE_PRINT("  ? Bit-0 error (dominant expected)\n"); break;
                case 6: SYS_CONSOLE_PRINT("  ? CRC error\n"); ReportError(ERR_CAN1_CRC); break;
                case 7: SYS_CONSOLE_PRINT("  ? LEC unchanged\n"); break;
                default:  break;
            }

            /* -------- Status flags -------- */
            if (status & CAN_PSR_BO_Msk)  { SYS_CONSOLE_PRINT("  ? **Bus-Off** state\n"); ReportError(ERR_CAN1_BUS_OFF); }
//            if (status & CAN_PSR_EP_Msk)  SYS_CONSOLE_PRINT("  ? Error-Passive state\n");
//            if (status & CAN_PSR_EW_Msk)  SYS_CONSOLE_PRINT("  ? Error-Warning state\n");
//            if (status & CAN_PSR_PXE_Msk) SYS_CONSOLE_PRINT("  ? Protocol-Exception event\n");
//            if (status & CAN_PSR_DLEC_Msk)SYS_CONSOLE_PRINT("  ? Data-phase error (CAN FD)\n");
            
            if (((status & CAN_PSR_LEC_Msk) == CAN_ERROR_NONE) || ((status & CAN_PSR_LEC_Msk) == CAN_ERROR_LEC_NC))
            {
                numberOfMessage = CAN0_RxFifoFillLevelGet(CAN_RX_FIFO_0);
                if (numberOfMessage != 0)
                {
                    memset(can0rxFiFo0, 0x00, (numberOfMessage * CAN0_RX_FIFO0_ELEMENT_SIZE));
                    if (CAN0_MessageReceiveFifo(CAN_RX_FIFO_0, numberOfMessage, (CAN_RX_BUFFER *)can0rxFiFo0) == true)
                    {
//                        SYS_CONSOLE_PRINT(" CAN0 Rx FIFO0 : New Message Received\r\n");
                        vDisplayCanRxMessage(numberOfMessage, (CAN_RX_BUFFER *)can0rxFiFo0, CAN0_RX_FIFO0_ELEMENT_SIZE);
                        xSemaphoreTake(xCan0QueueMutex, portMAX_DELAY);
                        if (xQueueSend(xCAN0QueueHandler, &can0rxFiFo0, (TickType_t) 10) != pdPASS) {
                            SYS_CONSOLE_PRINT("Failed to send message in CAN0 queue\r\n");
                        }
                        xSemaphoreGive(xCan0QueueMutex); 
                    }
                    else
                    {
                        SYS_CONSOLE_PRINT(" Error in received CAN0 message\r\n");
                    }
                }
            }
        }       

        if (CAN1_InterruptGet(CAN_INTERRUPT_RF0N_MASK))
        {    
            CAN1_InterruptClear(CAN_INTERRUPT_RF0N_MASK);

            /* Check CAN Status */
            status = CAN1_ErrorGet();
            /* Decode and print it */
            SYS_CONSOLE_PRINT("CAN1 status: 0x%08lX\r\n", (unsigned long)status);
            /* -------- Last-Error-Code (LEC) field -------- */
            switch (status & 0x7u)
            {
//                case 0: SYS_CONSOLE_PRINT("  ? No error\n"); break;
                case 1: SYS_CONSOLE_PRINT("  ? Stuff error\n"); ReportError(ERR_CAN2_STUFF); break;
                case 2: SYS_CONSOLE_PRINT("  ? Form error\n"); break;
                case 3: SYS_CONSOLE_PRINT("  ? ACK error\n"); ReportError(ERR_CAN2_ACK); break;
                case 4: SYS_CONSOLE_PRINT("  ? Bit-1 error (recessive expected)\n"); break;
                case 5: SYS_CONSOLE_PRINT("  ? Bit-0 error (dominant expected)\n"); break;
                case 6: SYS_CONSOLE_PRINT("  ? CRC error\n"); ReportError(ERR_CAN2_CRC); break;
                case 7: SYS_CONSOLE_PRINT("  ? LEC unchanged\n"); break;
                default:  break;
            }

            /* -------- Status flags -------- */
            if (status & CAN_PSR_BO_Msk)  { SYS_CONSOLE_PRINT("  ? **Bus-Off** state\n"); ReportError(ERR_CAN2_BUS_OFF); }
//            if (status & CAN_PSR_EP_Msk)  SYS_CONSOLE_PRINT("  ? Error-Passive state\n");
//            if (status & CAN_PSR_EW_Msk)  SYS_CONSOLE_PRINT("  ? Error-Warning state\n");
//            if (status & CAN_PSR_PXE_Msk) SYS_CONSOLE_PRINT("  ? Protocol-Exception event\n");
//            if (status & CAN_PSR_DLEC_Msk)SYS_CONSOLE_PRINT("  ? Data-phase error (CAN FD)\n");
            
            if (((status & CAN_PSR_LEC_Msk) == CAN_ERROR_NONE) || ((status & CAN_PSR_LEC_Msk) == CAN_ERROR_LEC_NC))
            {
                numberOfMessage = CAN1_RxFifoFillLevelGet(CAN_RX_FIFO_0);
                if (numberOfMessage != 0)
                {
                    memset(can1rxFiFo0, 0x00, (numberOfMessage * CAN1_RX_FIFO0_ELEMENT_SIZE));
                    if (CAN1_MessageReceiveFifo(CAN_RX_FIFO_0, numberOfMessage, (CAN_RX_BUFFER *)can1rxFiFo0) == true)
                    {
//                        SYS_CONSOLE_PRINT(" CAN1 Rx FIFO0 : New Message Received\r\n");
                        vDisplayCanRxMessage(numberOfMessage, (CAN_RX_BUFFER *)can1rxFiFo0, CAN1_RX_FIFO0_ELEMENT_SIZE);
                        xSemaphoreTake(xCan1QueueMutex, portMAX_DELAY);
                        if (xQueueSend(xCAN1QueueHandler, &can1rxFiFo0, (TickType_t) 10) != pdPASS) {
                            SYS_CONSOLE_PRINT("Failed to send message in CAN1 queue\r\n");
                        }
                        xSemaphoreGive(xCan1QueueMutex);                        
                    }
                    else
                    {
                        SYS_CONSOLE_PRINT(" Error in received CAN1 message\r\n");
                    }
                }
            }           
        }
        if (CAN2_InterruptGet(CAN_INTERRUPT_RF0N_MASK))
        {    
            CAN2_InterruptClear(CAN_INTERRUPT_RF0N_MASK);

            /* Check CAN Status */
            status = CAN2_ErrorGet();
            /* Decode and print it */
            SYS_CONSOLE_PRINT("CAN2 status: 0x%08lX\r\n", (unsigned long)status);
            /* -------- Last-Error-Code (LEC) field -------- */
            switch (status & 0x7u)
            {
//                case 0: SYS_CONSOLE_PRINT("  ? No error\n"); break;
                case 1: SYS_CONSOLE_PRINT("  ? Stuff error\n"); ReportError(ERR_CAN3_STUFF); break;
                case 2: SYS_CONSOLE_PRINT("  ? Form error\n"); break;
                case 3: SYS_CONSOLE_PRINT("  ? ACK error\n"); ReportError(ERR_CAN3_ACK); break;
                case 4: SYS_CONSOLE_PRINT("  ? Bit-1 error (recessive expected)\n"); break;
                case 5: SYS_CONSOLE_PRINT("  ? Bit-0 error (dominant expected)\n"); break;
                case 6: SYS_CONSOLE_PRINT("  ? CRC error\n"); ReportError(ERR_CAN3_CRC); break;
                case 7: SYS_CONSOLE_PRINT("  ? LEC unchanged\n"); break;
                default:  break;
            }

            /* -------- Status flags -------- */
            if (status & CAN_PSR_BO_Msk)  { SYS_CONSOLE_PRINT("  ? **Bus-Off** state\n"); ReportError(ERR_CAN3_BUS_OFF); }
//            if (status & CAN_PSR_EP_Msk)  SYS_CONSOLE_PRINT("  ? Error-Passive state\n");
//            if (status & CAN_PSR_EW_Msk)  SYS_CONSOLE_PRINT("  ? Error-Warning state\n");
//            if (status & CAN_PSR_PXE_Msk) SYS_CONSOLE_PRINT("  ? Protocol-Exception event\n");
//            if (status & CAN_PSR_DLEC_Msk)SYS_CONSOLE_PRINT("  ? Data-phase error (CAN FD)\n");
            
            if (((status & CAN_PSR_LEC_Msk) == CAN_ERROR_NONE) || ((status & CAN_PSR_LEC_Msk) == CAN_ERROR_LEC_NC))
            {
                numberOfMessage = CAN2_RxFifoFillLevelGet(CAN_RX_FIFO_0);
                if (numberOfMessage != 0)
                {
                    memset(can2rxFiFo0, 0x00, (numberOfMessage * CAN2_RX_FIFO0_ELEMENT_SIZE));
                    if (CAN2_MessageReceiveFifo(CAN_RX_FIFO_0, numberOfMessage, (CAN_RX_BUFFER *)can2rxFiFo0) == true)
                    {
//                        SYS_CONSOLE_PRINT(" CAN2 Rx FIFO0 : New Message Received\r\n");
                        vDisplayCanRxMessage(numberOfMessage, (CAN_RX_BUFFER *)can2rxFiFo0, CAN2_RX_FIFO0_ELEMENT_SIZE);
                        xSemaphoreTake(xCan2QueueMutex, portMAX_DELAY);
                        if (xQueueSend(xCAN2QueueHandler, &can2rxFiFo0, (TickType_t) 10) != pdPASS) {
                            SYS_CONSOLE_PRINT("Failed to send message in CAN2 queue\r\n");
                        }
                        xSemaphoreGive(xCan2QueueMutex); 
                    }
                    else
                    {
                        SYS_CONSOLE_PRINT(" Error in received CAN2 message\r\n");
                    }
                }
            }           
        }       
       
        if (CAN3_InterruptGet(CAN_INTERRUPT_RF0N_MASK))
        {    
            CAN3_InterruptClear(CAN_INTERRUPT_RF0N_MASK);

            /* Check CAN Status */
            status = CAN3_ErrorGet();
            /* Decode and print it */
            SYS_CONSOLE_PRINT("CAN3 status: 0x%08lX\r\n", (unsigned long)status);
            /* -------- Last-Error-Code (LEC) field -------- */
            switch (status & 0x7u)
            {
//                case 0: SYS_CONSOLE_PRINT("  ? No error\n"); break;
                case 1: SYS_CONSOLE_PRINT("  ? Stuff error\n"); ReportError(ERR_CAN4_STUFF); break;
                case 2: SYS_CONSOLE_PRINT("  ? Form error\n"); break;
                case 3: SYS_CONSOLE_PRINT("  ? ACK error\n"); ReportError(ERR_CAN4_ACK); break;
                case 4: SYS_CONSOLE_PRINT("  ? Bit-1 error (recessive expected)\n"); break;
                case 5: SYS_CONSOLE_PRINT("  ? Bit-0 error (dominant expected)\n"); break;
                case 6: SYS_CONSOLE_PRINT("  ? CRC error\n"); ReportError(ERR_CAN4_CRC); break;
                case 7: SYS_CONSOLE_PRINT("  ? LEC unchanged\n"); break;
                default:  break;
            }

            /* -------- Status flags -------- */
            if (status & CAN_PSR_BO_Msk)  { SYS_CONSOLE_PRINT("  ? **Bus-Off** state\n"); ReportError(ERR_CAN4_BUS_OFF); }
//            if (status & CAN_PSR_EP_Msk)  SYS_CONSOLE_PRINT("  ? Error-Passive state\n");
//            if (status & CAN_PSR_EW_Msk)  SYS_CONSOLE_PRINT("  ? Error-Warning state\n");
//            if (status & CAN_PSR_PXE_Msk) SYS_CONSOLE_PRINT("  ? Protocol-Exception event\n");
//            if (status & CAN_PSR_DLEC_Msk)SYS_CONSOLE_PRINT("  ? Data-phase error (CAN FD)\n");
            
            if (((status & CAN_PSR_LEC_Msk) == CAN_ERROR_NONE) || ((status & CAN_PSR_LEC_Msk) == CAN_ERROR_LEC_NC))
            {
                numberOfMessage = CAN3_RxFifoFillLevelGet(CAN_RX_FIFO_0);
                if (numberOfMessage != 0)
                {
                    memset(can3rxFiFo0, 0x00, (numberOfMessage * CAN3_RX_FIFO0_ELEMENT_SIZE));
                    if (CAN3_MessageReceiveFifo(CAN_RX_FIFO_0, numberOfMessage, (CAN_RX_BUFFER *)can3rxFiFo0) == true)
                    {
//                        SYS_CONSOLE_PRINT(" CAN3 Rx FIFO0 : New Message Received\r\n");
                        vDisplayCanRxMessage(numberOfMessage, (CAN_RX_BUFFER *)can3rxFiFo0, CAN3_RX_FIFO0_ELEMENT_SIZE);
                        xSemaphoreTake(xCan3QueueMutex, portMAX_DELAY);
                        if (xQueueSend(xCAN3QueueHandler, &can3rxFiFo0, (TickType_t) 10) != pdPASS) {
                            SYS_CONSOLE_PRINT("Failed to send message in CAN3 queue\r\n");
                        }
                        xSemaphoreGive(xCan3QueueMutex); 
                    }
                    else
                    {
                        SYS_CONSOLE_PRINT(" Error in received CAN3 message\r\n");
                    }
                }
            }           
        }
        
        if (CAN4_InterruptGet(CAN_INTERRUPT_RF0N_MASK))
        {    
            CAN4_InterruptClear(CAN_INTERRUPT_RF0N_MASK);

            /* Check CAN Status */
            status = CAN4_ErrorGet();
            /* Decode and print it */
            SYS_CONSOLE_PRINT("CAN4 status: 0x%08lX\r\n", (unsigned long)status);
            /* -------- Last-Error-Code (LEC) field -------- */
            switch (status & 0x7u)
            {
//                case 0: SYS_CONSOLE_PRINT("  ? No error\n"); break;
                case 1: SYS_CONSOLE_PRINT("  ? Stuff error\n"); ReportError(ERR_CAN5_STUFF); break;
                case 2: SYS_CONSOLE_PRINT("  ? Form error\n"); break;
                case 3: SYS_CONSOLE_PRINT("  ? ACK error\n"); ReportError(ERR_CAN5_ACK); break;
                case 4: SYS_CONSOLE_PRINT("  ? Bit-1 error (recessive expected)\n"); break;
                case 5: SYS_CONSOLE_PRINT("  ? Bit-0 error (dominant expected)\n"); break;
                case 6: SYS_CONSOLE_PRINT("  ? CRC error\n"); ReportError(ERR_CAN5_CRC); break;
                case 7: SYS_CONSOLE_PRINT("  ? LEC unchanged\n"); break;
                default:  break;
            }

            /* -------- Status flags -------- */
            if (status & CAN_PSR_BO_Msk)  { SYS_CONSOLE_PRINT("  ? **Bus-Off** state\n"); ReportError(ERR_CAN5_BUS_OFF); }
//            if (status & CAN_PSR_EP_Msk)  SYS_CONSOLE_PRINT("  ? Error-Passive state\n");
//            if (status & CAN_PSR_EW_Msk)  SYS_CONSOLE_PRINT("  ? Error-Warning state\n");
//            if (status & CAN_PSR_PXE_Msk) SYS_CONSOLE_PRINT("  ? Protocol-Exception event\n");
//            if (status & CAN_PSR_DLEC_Msk)SYS_CONSOLE_PRINT("  ? Data-phase error (CAN FD)\n");
            
            if (((status & CAN_PSR_LEC_Msk) == CAN_ERROR_NONE) || ((status & CAN_PSR_LEC_Msk) == CAN_ERROR_LEC_NC))
            {
                numberOfMessage = CAN4_RxFifoFillLevelGet(CAN_RX_FIFO_0);
                if (numberOfMessage != 0)
                {
                    memset(can4rxFiFo0, 0x00, (numberOfMessage * CAN4_RX_FIFO0_ELEMENT_SIZE));
                    if (CAN4_MessageReceiveFifo(CAN_RX_FIFO_0, numberOfMessage, (CAN_RX_BUFFER *)can4rxFiFo0) == true)
                    {
//                        SYS_CONSOLE_PRINT(" CAN4 Rx FIFO0 : New Message Received\r\n");
                        vDisplayCanRxMessage(numberOfMessage, (CAN_RX_BUFFER *)can4rxFiFo0, CAN4_RX_FIFO0_ELEMENT_SIZE);
                        xSemaphoreTake(xCan4QueueMutex, portMAX_DELAY);
                        if (xQueueSend(xCAN4QueueHandler, &can4rxFiFo0, (TickType_t) 10) != pdPASS) {
                            SYS_CONSOLE_PRINT("Failed to send message in CAN4 queue\r\n");
                        }
                        xSemaphoreGive(xCan4QueueMutex);                        
                    }
                    else
                    {
                        SYS_CONSOLE_PRINT(" Error in received CAN4 message\r\n");
                    }
                }
            }           
        }
        
        if (CAN5_InterruptGet(CAN_INTERRUPT_RF0N_MASK))
        {    
            CAN5_InterruptClear(CAN_INTERRUPT_RF0N_MASK);

            /* Check CAN Status */
            status = CAN5_ErrorGet();
            /* Decode and print it */
            SYS_CONSOLE_PRINT("CAN5 status: 0x%08lX\r\n", (unsigned long)status);
            /* -------- Last-Error-Code (LEC) field -------- */
            switch (status & 0x7u)
            {
//                case 0: SYS_CONSOLE_PRINT("  ? No error\n"); break;
                case 1: SYS_CONSOLE_PRINT("  ? Stuff error\n"); ReportError(ERR_CAN6_STUFF); break;
                case 2: SYS_CONSOLE_PRINT("  ? Form error\n"); break;
                case 3: SYS_CONSOLE_PRINT("  ? ACK error\n"); ReportError(ERR_CAN6_ACK); break;
                case 4: SYS_CONSOLE_PRINT("  ? Bit-1 error (recessive expected)\n"); break;
                case 5: SYS_CONSOLE_PRINT("  ? Bit-0 error (dominant expected)\n"); break;
                case 6: SYS_CONSOLE_PRINT("  ? CRC error\n"); ReportError(ERR_CAN6_CRC); break;
                case 7: SYS_CONSOLE_PRINT("  ? LEC unchanged\n"); break;
                default:  break;
            }

            /* -------- Status flags -------- */
            if (status & CAN_PSR_BO_Msk)  { SYS_CONSOLE_PRINT("  ? **Bus-Off** state\n"); ReportError(ERR_CAN6_BUS_OFF); }
//            if (status & CAN_PSR_EP_Msk)  SYS_CONSOLE_PRINT("  ? Error-Passive state\n");
//            if (status & CAN_PSR_EW_Msk)  SYS_CONSOLE_PRINT("  ? Error-Warning state\n");
//            if (status & CAN_PSR_PXE_Msk) SYS_CONSOLE_PRINT("  ? Protocol-Exception event\n");
//            if (status & CAN_PSR_DLEC_Msk)SYS_CONSOLE_PRINT("  ? Data-phase error (CAN FD)\n");
            
            if (((status & CAN_PSR_LEC_Msk) == CAN_ERROR_NONE) || ((status & CAN_PSR_LEC_Msk) == CAN_ERROR_LEC_NC))
            {
                numberOfMessage = CAN5_RxFifoFillLevelGet(CAN_RX_FIFO_0);
                if (numberOfMessage != 0)
                {
                    memset(can5rxFiFo0, 0x00, (numberOfMessage * CAN5_RX_FIFO0_ELEMENT_SIZE));
                    if (CAN5_MessageReceiveFifo(CAN_RX_FIFO_0, numberOfMessage, (CAN_RX_BUFFER *)can5rxFiFo0) == true)
                    {
//                        SYS_CONSOLE_PRINT(" CAN5 Rx FIFO0 : New Message Received\r\n");
                        vDisplayCanRxMessage(numberOfMessage, (CAN_RX_BUFFER *)can5rxFiFo0, CAN5_RX_FIFO0_ELEMENT_SIZE);
                        xSemaphoreTake(xCan5QueueMutex, portMAX_DELAY);
                        if (xQueueSend(xCAN5QueueHandler, &can5rxFiFo0, (TickType_t) 10) != pdPASS) {
                            SYS_CONSOLE_PRINT("Failed to send message in CAN5 queue\r\n");
                        }
                        xSemaphoreGive(xCan5QueueMutex);                         
                    }
                    else
                    {
                        SYS_CONSOLE_PRINT(" Error in received CAN5 message\r\n");
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
    CAN3_MessageRAMConfigSet(Can3MessageRAM);
    
#if HEV_IO_Aggregator   
    CAN4_MessageRAMConfigSet(Can4MessageRAM);
    CAN5_MessageRAMConfigSet(Can5MessageRAM);
#endif    

    /* Create CAN Queues */
    xCAN0QueueHandler = xQueueCreate(CAN_QUEUE_SIZE, CAN_QUEUE_ITEM_SIZE);
    xCAN1QueueHandler = xQueueCreate(CAN_QUEUE_SIZE, CAN_QUEUE_ITEM_SIZE);
    xCAN2QueueHandler = xQueueCreate(CAN_QUEUE_SIZE, CAN_QUEUE_ITEM_SIZE);
    xCAN3QueueHandler = xQueueCreate(CAN_QUEUE_SIZE, CAN_QUEUE_ITEM_SIZE);
    
#if HEV_IO_Aggregator
    xCAN4QueueHandler = xQueueCreate(CAN_QUEUE_SIZE, CAN_QUEUE_ITEM_SIZE);
    xCAN5QueueHandler = xQueueCreate(CAN_QUEUE_SIZE, CAN_QUEUE_ITEM_SIZE);

    /* Check if all required queues were created successfully */
    if (xCAN0QueueHandler == NULL || xCAN1QueueHandler == NULL || 
        xCAN2QueueHandler == NULL || xCAN3QueueHandler == NULL || 
        xCAN4QueueHandler == NULL || xCAN5QueueHandler == NULL) 
    {
        SYS_CONSOLE_PRINT("Error: CANRX Queue creation failed\r\n");
        return;
    }
    SYS_CONSOLE_PRINT("CANRX Queue creation successful\r\n");
#elif Two_Wheeler_IO_Aggregator
    /* Check if all required queues were created successfully */
    if (xCAN0QueueHandler == NULL || xCAN1QueueHandler == NULL || 
        xCAN2QueueHandler == NULL || xCAN3QueueHandler == NULL) 
    {
        SYS_CONSOLE_PRINT("Error: CANRX Queue creation failed\r\n");
        return;
    }
    SYS_CONSOLE_PRINT("CANRX Queue creation successful\r\n");
#endif    
    xCan0QueueMutex = xSemaphoreCreateMutex();
    xCan1QueueMutex = xSemaphoreCreateMutex();
    xCan2QueueMutex = xSemaphoreCreateMutex();
    xCan3QueueMutex = xSemaphoreCreateMutex();
    xCan4QueueMutex = xSemaphoreCreateMutex();
    xCan5QueueMutex = xSemaphoreCreateMutex();

    if (xCan0QueueMutex == NULL || xCan1QueueMutex == NULL || 
        xCan2QueueMutex == NULL || xCan3QueueMutex == NULL || 
        xCan4QueueMutex == NULL || xCan5QueueMutex == NULL)
    {
        // Handle error: At least one mutex creation failed
         SYS_CONSOLE_PRINT("Error: Mutex creation failed\r\n");
    }

    /* Create Tasks */
    xTaskCreate(vCanRxHandlerTask, "vCanRxHandlerTask", CAN_RX_HANDLER_HEAP_DEPTH, NULL, CAN_RX_HANDLER_TASK_PRIORITY, NULL);
    xTaskCreate(vCan0HandlerServerTask, "vCan0HandlerServerTask", CAN_SERVER_HANDLER_HEAP_DEPTH, NULL, CAN_SERVER_HANDLER_TASK_PRIORITY, NULL);
    xTaskCreate(vCan1HandlerServerTask, "vCan1HandlerServerTask", CAN_SERVER_HANDLER_HEAP_DEPTH, NULL, CAN_SERVER_HANDLER_TASK_PRIORITY, NULL);
    xTaskCreate(vCan2HandlerServerTask, "vCan2HandlerServerTask", CAN_SERVER_HANDLER_HEAP_DEPTH, NULL, CAN_SERVER_HANDLER_TASK_PRIORITY, NULL);
    xTaskCreate(vCan3HandlerServerTask, "vCan3HandlerServerTask", CAN_SERVER_HANDLER_HEAP_DEPTH, NULL, CAN_SERVER_HANDLER_TASK_PRIORITY, NULL);

#if HEV_IO_Aggregator      
    xTaskCreate(vCan4HandlerServerTask, "vCan4HandlerServerTask", CAN_SERVER_HANDLER_HEAP_DEPTH, NULL, CAN_SERVER_HANDLER_TASK_PRIORITY, NULL);
    xTaskCreate(vCan5HandlerServerTask, "vCan5HandlerServerTask", CAN_SERVER_HANDLER_HEAP_DEPTH, NULL, CAN_SERVER_HANDLER_TASK_PRIORITY, NULL);
#endif    
}
