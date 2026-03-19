/* ************************************************************************** */
/** @file TelemetryHandler.c
 *  @brief Telemetry Handler for PM, BMS, and Temperature data
 *
 *  Sends telemetry data to client via TCP every 1 second
 */
/* ************************************************************************** */

#include "TelemetryHandler.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "definitions.h"                // SYS function prototypes
#include "configuration.h"
#include "device.h"
#include "tcpip_manager_control.h"
#include "library/tcpip/tcpip_helpers.h"
#include "sessionDBHandler.h" /* For accessing session data */
/* ****************************************************************************/
/* Macro Definitions                                                          */
/* ****************************************************************************/
#define TELEMETRY_TASK_STACK_SIZE    1024U
#define TELEMETRY_TASK_PRIORITY      2U
#define TELEMETRY_TASK_DELAY_MS      10U
/* TCP/IP Definitions */
#define TELEMETRY_PORT_HEV           8889
#define FACTOR_1000                 1000U
#define TELEMETRY_COMPARTMENT_ID  0x01U
/* ************************************************************************** */
/* Global Variables                                                           */
/* ************************************************************************** */
TELEMETRY_Data_t telemetryData[4];       /**< Global telemetry data */

/* TCP Socket */
static TCP_SOCKET sTelemetryServerSocket = INVALID_SOCKET;

/* Task handle */
TaskHandle_t xTelemetryTaskHandle = NULL;

/* Unique ID counter for broadcast messages */
static uint32_t telemetryUniqueId = 1U;

/* Output buffer for TCP frame */
static uint8_t telemetryFrameBuf[128];

/* ************************************************************************** */
/* Local Function Prototypes                                                  */
/* ************************************************************************** */
static uint16_t Telemetry_CalculateCRC(const uint8_t *data, uint16_t length);
static void Telemetry_SendFrame(uint8_t cmdId,
                                uint8_t compartmentId,
                                uint8_t dockId,
                                const uint8_t *payload,
                                uint8_t payloadLen);
static void Telemetry_SendPM(uint8_t compartmentId, uint8_t dockId);
static void Telemetry_SendBMS(uint8_t compartmentId, uint8_t dockId);
static void Telemetry_SendTemperature(uint8_t compartmentId, uint8_t dockId);

/* ************************************************************************** */
/* Internal Functions                                                         */
/* ************************************************************************** */

/* ************************************************************************** */
/* Fill Telemetry Data from SESSION DB using setter/getter macros              */
/* ************************************************************************** */
static void Telemetry_UpdateFromSession(uint8_t dockId)
{
    /* ---------------- PM Data ---------------- */
    telemetryData[dockId].pmData.u32PmOutputVoltage_mV = SESSION_GetPmOutputVoltage(dockId) * FACTOR_1000; /* Convert V to mV */
    telemetryData[dockId].pmData.u32PmOutputCurrent_mA = SESSION_GetPmOutputCurrent(dockId) * FACTOR_1000; /* Convert A to mA */
    telemetryData[dockId].pmData.u32OutputPower_W    = SESSION_GetOutputPower(dockId);
    telemetryData[dockId].pmData.u32TotalEnergy_Wh   = SESSION_GetEnergyDelivered(dockId);
    telemetryData[dockId].pmData.u32PMFaultCode      = SESSION_GetPMFaultCode(dockId);
    telemetryData[dockId].pmData.u8PMTemperature     = SESSION_GetPMTemperature(dockId);
    telemetryData[dockId].pmData.u8PMStatus          = SESSION_GetPMRxStatus(dockId);

    /* ---------------- BMS Data ---------------- */
    telemetryData[dockId].bmsData.u32BMSDemandVoltage = SESSION_GetBMSDemandVoltage(dockId) * FACTOR_1000; /* Convert V to mV */
    telemetryData[dockId].bmsData.u32BMSDemandCurrent = SESSION_GetBMSDemandCurrent(dockId) * FACTOR_1000; /* Convert A to mA */
    telemetryData[dockId].bmsData.u32EstimatedTime_s  = SESSION_GetEnergyDelivered(dockId); /* or another 32-bit field */
    telemetryData[dockId].bmsData.u32BMSFaultCode     = SESSION_GetBMSFaultCode(dockId);
    telemetryData[dockId].bmsData.u8CurrentSoc        = SESSION_GetCurrentSoc(dockId);
    telemetryData[dockId].bmsData.u8InitialSoc        = SESSION_GetInitialSoc(dockId);
    telemetryData[dockId].bmsData.u8BMSTemperature    = SESSION_GetBMSTemperature(dockId);
    telemetryData[dockId].bmsData.u8BMSStatus         = SESSION_GetBMSRxStatus(dockId);

    /* ---------------- Temperature Data ---------------- */
    telemetryData[dockId].tempData.u8CompartmentTemperature = SESSION_GetDockTemperature(dockId);
    telemetryData[dockId].tempData.u8DockTemperature        = SESSION_GetPMTemperature(dockId);
}
/**
 * @brief Send a single telemetry frame over TCP
 */
static void Telemetry_SendFrame(uint8_t cmdId,
                                uint8_t compartmentId,
                                uint8_t dockId,
                                const uint8_t *payload,
                                uint8_t payloadLen)
{
    uint16_t index = 0U;
    uint16_t crc;

    /* Unique ID */
    telemetryFrameBuf[index++] = (uint8_t)((telemetryUniqueId >> 24U) & 0xFFU);
    telemetryFrameBuf[index++] = (uint8_t)((telemetryUniqueId >> 16U) & 0xFFU);
    telemetryFrameBuf[index++] = (uint8_t)((telemetryUniqueId >> 8U) & 0xFFU);
    telemetryFrameBuf[index++] = (uint8_t)(telemetryUniqueId & 0xFFU);

    /* Message Type: Broadcast */
    telemetryFrameBuf[index++] = (uint8_t)((TELEMETRY_MSG_TYPE_BROADCAST >> 8U) & 0xFFU);
    telemetryFrameBuf[index++] = (uint8_t)(TELEMETRY_MSG_TYPE_BROADCAST & 0xFFU);

    /* Compartment and Dock IDs */
    telemetryFrameBuf[index++] = compartmentId;
    telemetryFrameBuf[index++] = dockId;

    /* Command ID */
    telemetryFrameBuf[index++] = cmdId;

    /* Payload Length */
    telemetryFrameBuf[index++] = payloadLen;

    /* Copy payload */
    memcpy(&telemetryFrameBuf[index], payload, payloadLen);
    index += payloadLen;

    /* Calculate CRC16 */
    crc = Telemetry_CalculateCRC(telemetryFrameBuf, index);
    telemetryFrameBuf[index++] = (uint8_t)((crc >> 8U) & 0xFFU);
    telemetryFrameBuf[index++] = (uint8_t)(crc & 0xFFU);

    /* Send via TCP */
    TCPIP_TCP_ArrayPut(sTelemetryServerSocket, telemetryFrameBuf, index);
    TCPIP_TCP_Flush(sTelemetryServerSocket);

    /* Increment Unique ID for next frame */
    telemetryUniqueId++;
}

/**
 * @brief Send Power Module telemetry
 */
static void Telemetry_SendPM(uint8_t compartmentId, uint8_t dockId)
{
    uint8_t payload[22U];
    uint16_t i = 0U;

    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32PmOutputVoltage_mV >> 24U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32PmOutputVoltage_mV >> 16U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32PmOutputVoltage_mV >> 8U) & 0xFFU);
    payload[i++] = (uint8_t)(telemetryData[dockId].pmData.u32PmOutputVoltage_mV & 0xFFU);

    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32PmOutputCurrent_mA >> 24U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32PmOutputCurrent_mA >> 16U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32PmOutputCurrent_mA >> 8U) & 0xFFU);
    payload[i++] = (uint8_t)(telemetryData[dockId].pmData.u32PmOutputCurrent_mA & 0xFFU);

    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32OutputPower_W >> 24U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32OutputPower_W >> 16U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32OutputPower_W >> 8U) & 0xFFU);
    payload[i++] = (uint8_t)(telemetryData[dockId].pmData.u32OutputPower_W & 0xFFU);

    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32TotalEnergy_Wh >> 24U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32TotalEnergy_Wh >> 16U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32TotalEnergy_Wh >> 8U) & 0xFFU);
    payload[i++] = (uint8_t)(telemetryData[dockId].pmData.u32TotalEnergy_Wh & 0xFFU);

    payload[i++] = telemetryData[dockId].pmData.u8PMTemperature;

    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32PMFaultCode >> 24U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32PMFaultCode >> 16U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].pmData.u32PMFaultCode >> 8U) & 0xFFU);
    payload[i++] = (uint8_t)(telemetryData[dockId].pmData.u32PMFaultCode & 0xFFU);

    payload[i++] = telemetryData[dockId].pmData.u8PMStatus;

    Telemetry_SendFrame(TELEMETRY_CMD_PM_DATA, compartmentId, dockId, payload, i);
}

/**
 * @brief Send BMS telemetry
 */
static void Telemetry_SendBMS(uint8_t compartmentId, uint8_t dockId)
{
    uint8_t payload[15U];
    uint16_t i = 0U;

    payload[i++] = (uint8_t)((telemetryData[dockId].bmsData.u32BMSDemandVoltage >> 24U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].bmsData.u32BMSDemandVoltage >> 16U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].bmsData.u32BMSDemandVoltage >> 8U) & 0xFFU);
    payload[i++] = (uint8_t)(telemetryData[dockId].bmsData.u32BMSDemandVoltage & 0xFFU);

    payload[i++] = (uint8_t)((telemetryData[dockId].bmsData.u32BMSDemandCurrent >> 24U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].bmsData.u32BMSDemandCurrent >> 16U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].bmsData.u32BMSDemandCurrent >> 8U) & 0xFFU);
    payload[i++] = (uint8_t)(telemetryData[dockId].bmsData.u32BMSDemandCurrent & 0xFFU);

    payload[i++] = telemetryData[dockId].bmsData.u8CurrentSoc;
    payload[i++] = telemetryData[dockId].bmsData.u8InitialSoc;
    payload[i++] = telemetryData[dockId].bmsData.u8BMSTemperature;

    payload[i++] = (uint8_t)((telemetryData[dockId].bmsData.u32EstimatedTime_s >> 24U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].bmsData.u32EstimatedTime_s >> 16U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].bmsData.u32EstimatedTime_s >> 8U) & 0xFFU);
    payload[i++] = (uint8_t)(telemetryData[dockId].bmsData.u32EstimatedTime_s & 0xFFU);

    payload[i++] = (uint8_t)((telemetryData[dockId].bmsData.u32BMSFaultCode >> 24U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].bmsData.u32BMSFaultCode >> 16U) & 0xFFU);
    payload[i++] = (uint8_t)((telemetryData[dockId].bmsData.u32BMSFaultCode >> 8U) & 0xFFU);
    payload[i++] = (uint8_t)(telemetryData[dockId].bmsData.u32BMSFaultCode & 0xFFU);

    payload[i++] = telemetryData[dockId].bmsData.u8BMSStatus;

    Telemetry_SendFrame(TELEMETRY_CMD_BMS_DATA, compartmentId, dockId, payload, i);
}

/**
 * @brief Send Temperature telemetry
 */
static void Telemetry_SendTemperature(uint8_t compartmentId, uint8_t dockId)
{
    uint8_t payload[4U];

    payload[0] = telemetryData[dockId].tempData.u8CompartmentTemperature;
    payload[1] = telemetryData[dockId].tempData.u8DockTemperature;
    payload[2] = telemetryData[dockId].tempData.u8DockTemperature;
    payload[3] = telemetryData[dockId].tempData.u8DockTemperature;

    Telemetry_SendFrame(TELEMETRY_CMD_TEMP_DATA, compartmentId, dockId, payload, 4U);
}

/* ************************************************************************** */
/* Public Functions                                                           */
/* ************************************************************************** */

/**
 * @brief Initialize telemetry TCP server and task
 */
/* Telemetry Init */
void Telemetry_Init(void)
{
    sTelemetryServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, TELEMETRY_PORT_HEV, 0U);
    if (sTelemetryServerSocket == INVALID_SOCKET)
    {
        SYS_CONSOLE_PRINT("Error opening TCP server socket\r\n");
        return;
    }

    (void)xTaskCreate(Telemetry_Task,
                      "Telemetry_Task",
                      TELEMETRY_TASK_STACK_SIZE,
                      NULL,
                      TELEMETRY_TASK_PRIORITY,
                      &xTelemetryTaskHandle);
}

/**
 * @brief FreeRTOS task to send telemetry data periodically
 */
void Telemetry_Task(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        /* Check if TCP socket is valid */
        if (sTelemetryServerSocket == INVALID_SOCKET)
        {
            /* Attempt to reopen TCP server socket */
            sTelemetryServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, TELEMETRY_PORT_HEV, 0U);
            if (sTelemetryServerSocket == INVALID_SOCKET)
            {
                /* Failed to open, wait and retry next loop */
                SYS_CONSOLE_PRINT("Telemetry: TCP server socket invalid, retrying...\r\n");
                vTaskDelay(pdMS_TO_TICKS(1000U));
                continue;
            }
            else
            {
                SYS_CONSOLE_PRINT("Telemetry: TCP server socket re-established.\r\n");
            }
        }
        if (TCPIP_TCP_IsConnected(sTelemetryServerSocket))
        {
            for (uint8_t dock = 1U; dock < 4U; dock++)
            {
                Telemetry_UpdateFromSession(dock);
                Telemetry_SendBMS(TELEMETRY_COMPARTMENT_ID, dock);
                vTaskDelay(pdMS_TO_TICKS(1U)); /* Small delay between sends */
                Telemetry_SendPM(TELEMETRY_COMPARTMENT_ID, dock);
                vTaskDelay(pdMS_TO_TICKS(1U)); /* Small delay between sends */
            }
            Telemetry_SendTemperature(TELEMETRY_COMPARTMENT_ID, 0xFFU); /* Broadcast dock ID for temperature */
        }
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_TASK_DELAY_MS)); /* 1-second interval */
    }
}

/* ************************************************************************** */
/* CRC16-CCITT Calculation                                                    */
/* ************************************************************************** */
static uint16_t Telemetry_CalculateCRC(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < length; i++)
    {
        crc ^= ((uint16_t)data[i] << 8U);
        for (uint8_t j = 0U; j < 8U; j++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

/* ************************************************************************** */
/* End of File                                                                */
/* ************************************************************************** */