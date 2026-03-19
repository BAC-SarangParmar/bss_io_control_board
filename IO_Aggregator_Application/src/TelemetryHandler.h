/* ************************************************************************** */
/** @file TelemetryHandler.h
 *  @brief Telemetry Handler for PM, BMS, and Temperature data
 *
 *  Sends telemetry data to client via TCP every 1 second
 */
/* ************************************************************************** */

#ifndef _TELEMETRY_HANDLER_H
#define _TELEMETRY_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

/* Command IDs */
#define TELEMETRY_CMD_PM_DATA        0x10
#define TELEMETRY_CMD_BMS_DATA       0x11
#define TELEMETRY_CMD_TEMP_DATA      0x12

/* Broadcast message type */
#define TELEMETRY_MSG_TYPE_BROADCAST 0x0004

/* Telemetry Data Structures */

typedef struct
{
    uint32_t u32PmOutputVoltage_mV;   /**< Charging Voltage (mV) */
    uint32_t u32PmOutputCurrent_mA;   /**< Charging Current (mA) */
    uint32_t u32OutputPower_W;        /**< Output Power (W) */
    uint32_t u32TotalEnergy_Wh;       /**< Total Energy (Wh) */
    uint32_t u32PMFaultCode;          /**< Fault Code */
    uint8_t  u8PMTemperature;         /**< Temperature (°C) */
    uint8_t  u8PMStatus;              /**< Status */
} TELEMETRY_PMData_t;

typedef struct
{
    uint32_t u32BMSDemandVoltage;     /**< Demand Voltage (mV) */
    uint32_t u32BMSDemandCurrent;     /**< Demand Current (mA) */
    uint32_t u32EstimatedTime_s;      /**< Estimated Charging Time (s) */
    uint32_t u32BMSFaultCode;         /**< Fault Code */
    uint8_t  u8CurrentSoc;            /**< SOC (%) */
    uint8_t  u8InitialSoc;            /**< Initial SOC (internal use) */
    uint8_t  u8BMSTemperature;        /**< Temperature (°C) */
    uint8_t  u8BMSStatus;             /**< Status */
} TELEMETRY_BMSData_t;

typedef struct
{
    uint8_t u8CompartmentTemperature; /**< Compartment Temperature (°C) */
    uint8_t u8DockTemperature;       /**< Dock Temperature (°C) */
} TELEMETRY_TempData_t;

/* Global Telemetry Data */
typedef struct
{
    TELEMETRY_PMData_t pmData;
    TELEMETRY_BMSData_t bmsData;
    TELEMETRY_TempData_t tempData;
} TELEMETRY_Data_t;

extern TELEMETRY_Data_t telemetryData[4];

/* Function Prototypes */
void Telemetry_Init(void);
void Telemetry_Task(void *pvParameters);

#endif /* _TELEMETRY_HANDLER_H */
/* *****************************************************************************
 End of File
 */ 