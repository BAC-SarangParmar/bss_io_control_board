/* ************************************************************************** */
/** @file    SessionDBHandler.h
  @brief   Session Database Handler Interface

  @Company
    Your Company Name

  @Summary
    Provides APIs to manage charging session data.

  @Description
    This module handles storage, retrieval, and management of charging
    session information such as session start, stop, status, and logs.
 */
/* ************************************************************************** */

#ifndef _SESSION_DB_HANDLER_H
#define _SESSION_DB_HANDLER_H

/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

/* ************************************************************************** */
/* Section: Constants                                                         */
/* ************************************************************************** */
/**
 * @brief Invalid Session ID
 */
#define SESSION_DB_INVALID_ID         (0xFFU)

/* Set bit */
#define SET_BIT(code, bit)     ((code) |= (1U << (bit)))

/* Clear bit */
#define CLEAR_BIT(code, bit)   ((code) &= ~(1U << (bit)))

/* Get bit (returns 0 or 1) */
#define GET_BIT(code, bit)     (((code) >> (bit)) & 0x1U)

#define FACTOR_10 10U
#define FACTOR_100 100U
#define FACTOR_1000 1000U
#define FACTOR_10_F 10.0f
#define FACTOR_100_F 100.0f
#define FACTOR_1000_F 1000.0f
#define FACTOR_0_1 0.1f
#define FACTOR_0_01 0.01f
#define FACTOR_0_001 0.001f
/* ************************************************************************** */
/* Section: Data Types                                                        */
/* ************************************************************************** */

  /* Charging State Enum */
typedef enum
{
  CH_STATE_INIT = 0,
  CH_STATE_AUTH_SUCCESS,
  CH_STATE_PARAM_VALIDATE,
  CH_STATE_CONNECTION_CONFIRMED,
  CH_STATE_INITIALIZE,
  CH_STATE_PRECHARGE,
  CH_STATE_CHARGING,
  CH_STATE_SHUTDOWN,
  CH_STATE_SESSION_COMPLETE,
  CH_STATE_ERROR
} CH_State_e;

typedef enum
{
    COMPARTMENT = 0,
    DOCK_1 = 1,
    DOCK_2,
    DOCK_3,
    MAX_DOCKS
} Dock_e;

typedef enum {
  CANBUS_0,
  CANBUS_1,
  CANBUS_2,
  CANBUS_3,
  CANBUS_4,
  CANBUS_5,
} CANBus_e;
typedef enum
{
  RECTIFIER_OFF = 0,
  RECTIFIER_ON
} RectifierState_e;
typedef enum
{
    GET_PARA = 0x01U,
    SET_PARA = 0x02U
}GetSet_e;

typedef enum
{
    CHARGING_SESSION_STARTED = 0x01U,
    CHARGING_SESSION_STOPPED = 0x00U,
} SessionEvent_e;

typedef enum
{
    SYSTEM_FAULT_NONE = 0,

    /* -------- Critical Faults (0–15) -------- */
    SYSTEM_FAULT_ESTOP_TRIGGERED = 0,
    SYSTEM_FAULT_BMS_ERROR,
    SYSTEM_FAULT_PM_ERROR,
    SYSTEM_FAULT_BMS_COMMUNICATION_FAILURE,
    SYSTEM_FAULT_PM_COMMUNICATION_FAILURE,
    SYSTEM_FAULT_PM_ZERO_CURRENT,
    SYSTEM_FAULT_PRECHARGE_FAILURE,

    /* -------- Optional Extended Critical -------- */
    SYSTEM_FAULT_BMS_OVER_TEMPERATURE,
    SYSTEM_FAULT_PM_OVER_TEMPERATURE,

    /* -------- Non-Critical (16–31) -------- */
    // Add warnings here if needed

} system_fault_t;

typedef enum
{
    STOP_REASON_NONE = 0,
    STOP_REASON_MCU_REQUEST,
    STOP_REASON_FAULT,
    STOP_REASON_TIMEOUT
} charging_stop_reason_t;

typedef enum
{
    /* Critical Faults (0–15) */
    BMS_FAULT_ENERGY_TRANSFER_ERROR = 0,
    BMS_FAULT_EV_CON_STATUS,
    BMS_FAULT_CHARGING_STOP_CONTROL,
    BMS_FAULT_BATTERY_OVERVOLT,

    /* Add more critical faults here */

    /* Warnings (16–31) */
    BMS_WARN_HIGH_TEMP = 16,
    BMS_WARN_VOLTAGE_DEVIATION,

    /* Add more warnings here */

} eBMSFaultBits_t;

typedef enum
{
    /* -------- Critical Faults (0–15) -------- */
    PM_FAULT_INPUT_UNDERVOLT = 0,
    PM_FAULT_PHASE_LOSS,
    PM_FAULT_INPUT_OVERVOLT,
    PM_FAULT_OUTPUT_OVERVOLT,
    PM_FAULT_OUTPUT_OVERCURRENT,
    PM_FAULT_OVER_TEMP,
    PM_FAULT_FAN_FAULT,
    PM_FAULT_HW_FAULT,
    PM_FAULT_BUS_EXCEPTION,
    PM_FAULT_SCI_EXCEPTION,
    PM_FAULT_DISCHARGE_FAULT,
    PM_FAULT_PFC_SHUTDOWN,

    /* -------- Non-Critical / Warning (16–31) -------- */
    PM_WARN_OUTPUT_UNDERVOLT = 16,
    PM_WARN_OUTPUT_OVERVOLT,
    PM_WARN_POWER_LIMIT

} PM_FaultEnum_t;
/**
 * @brief Session data structure
 */
typedef struct
{
    CH_State_e eChargingState;
    RectifierState_e bPMOnOffStatus;
    bool bStartChargingComm;
    uint8_t u8CurrentSoc;
    uint8_t u8InitialSoc;
    uint8_t u8SessionEndReason;
    uint8_t u8BMSRxStatue;
    uint8_t u8PMRxStatus;
    uint8_t u8AuthenticationCommand;
    uint8_t u8DockTemperature;
    uint8_t u8BMSTemperature;
    uint8_t u8PMTemperature;
    uint32_t u32BMSFaultBitmap;
    uint32_t u32PMFaultBitmap;
    uint16_t u16PMFaultCode;
    uint16_t u16OutputPower;
    uint16_t u16SessionFaultCode;
    uint16_t u16EstimatedChargingTime;
    float fPmOutputVoltage;
    float fPmOutputCurrent;
    float fPmSetVoltage;
    float fPmSetCurrent;
    float fBMSDemandVoltage;
    float fBMSDemandCurrent;
    uint32_t u32EnergyDelivered;
    volatile uint32_t u32PMLastRxTime;
    volatile uint32_t u32BMSLastRxTime;
} SESSION_Data_t;
extern SESSION_Data_t sessionDB[MAX_DOCKS]; /* Global session database */

/*
* Session Database Handler Macros
*/

/* State */
#define SESSION_SetChargingState(idx, val)          (sessionDB[(idx)].eChargingState = (val))
#define SESSION_GetChargingState(idx)               (sessionDB[(idx)].eChargingState)

#define SESSION_SetPMState(idx, val)        (sessionDB[(idx)].bPMOnOffStatus = (val))
#define SESSION_GetPMState(idx)             (sessionDB[(idx)].bPMOnOffStatus)

#define SESSION_SetStartChargingComm(idx, val)          (sessionDB[(idx)].bStartChargingComm = (val))
#define SESSION_GetStartChargingComm(idx)               (sessionDB[(idx)].bStartChargingComm)

/* Vehicle and PM status variables */
#define SESSION_SetCurrentSoc(idx, val)     (sessionDB[(idx)].u8CurrentSoc = (val))
#define SESSION_GetCurrentSoc(idx)          (sessionDB[(idx)].u8CurrentSoc)

#define SESSION_SetInitialSoc(idx, val)     (sessionDB[(idx)].u8InitialSoc = (val))
#define SESSION_GetInitialSoc(idx)          (sessionDB[(idx)].u8InitialSoc)

#define SESSION_SetSessionEndReason(idx,val) (sessionDB[(idx)].u8SessionEndReason = (val))
#define SESSION_GetSessionEndReason(idx)    (sessionDB[(idx)].u8SessionEndReason)

#define SESSION_SetBMSRxStatus(idx,val)     (sessionDB[(idx)].u8BMSRxStatue = (val))
#define SESSION_GetBMSRxStatus(idx)         (sessionDB[(idx)].u8BMSRxStatue)

#define SESSION_SetPMRxStatus(idx,val)      (sessionDB[(idx)].u8PMRxStatus = (val))
#define SESSION_GetPMRxStatus(idx)          (sessionDB[(idx)].u8PMRxStatus)

#define SESSION_SetAuthenticationCommand(idx,val) (sessionDB[(idx)].u8AuthenticationCommand = (val))
#define SESSION_GetAuthenticationCommand(idx)     (sessionDB[(idx)].u8AuthenticationCommand)

#define SESSION_SetDockTemperature(idx,val) (sessionDB[(idx)].u8DockTemperature = (val))
#define SESSION_GetDockTemperature(idx)     (sessionDB[(idx)].u8DockTemperature)

#define SESSION_SetBMSTemperature(idx,val)  (sessionDB[(idx)].u8BMSTemperature = (val))
#define SESSION_GetBMSTemperature(idx)      (sessionDB[(idx)].u8BMSTemperature)

#define SESSION_SetPMTemperature(idx,val)   (sessionDB[(idx)].u8PMTemperature = (val))
#define SESSION_GetPMTemperature(idx)       (sessionDB[(idx)].u8PMTemperature)

#define SESSION_SetBMSFaultBitmap(idx,val)    (sessionDB[(idx)].u32BMSFaultBitmap = (val))
#define SESSION_GetBMSFaultBitmap(idx)        (sessionDB[(idx)].u32BMSFaultBitmap)

#define SESSION_SetPMFaultBitmap(idx,val)     (sessionDB[(idx)].u32PMFaultBitmap = (val))
#define SESSION_GetPMFaultBitmap(idx)         (sessionDB[(idx)].u32PMFaultBitmap)

/* 16-bit variables */
#define SESSION_SetPMFaultCode(idx,val)     (sessionDB[(idx)].u16PMFaultCode = (val))
#define SESSION_GetPMFaultCode(idx)         (sessionDB[(idx)].u16PMFaultCode)

#define SESSION_SetPmOutputVoltage(idx,val) (sessionDB[(idx)].fPmOutputVoltage = (val))
#define SESSION_GetPmOutputVoltage(idx)     (sessionDB[(idx)].fPmOutputVoltage)

#define SESSION_SetPmOutputCurrent(idx,val) (sessionDB[(idx)].fPmOutputCurrent = (val))
#define SESSION_GetPmOutputCurrent(idx)     (sessionDB[(idx)].fPmOutputCurrent)

#define SESSION_SetBMSDemandVoltage(idx,val) (sessionDB[(idx)].fBMSDemandVoltage = (val))
#define SESSION_GetBMSDemandVoltage(idx)     (sessionDB[(idx)].fBMSDemandVoltage)

#define SESSION_SetBMSDemandCurrent(idx,val) (sessionDB[(idx)].fBMSDemandCurrent = (val))
#define SESSION_GetBMSDemandCurrent(idx)     (sessionDB[(idx)].fBMSDemandCurrent)

#define SESSION_SetPmSetVoltage(idx,val) (sessionDB[(idx)].fPmSetVoltage = (val))
#define SESSION_GetPmSetVoltage(idx)     (sessionDB[(idx)].fPmSetVoltage)

#define SESSION_SetPmSetCurrent(idx,val) (sessionDB[(idx)].fPmSetCurrent = (val))
#define SESSION_GetPmSetCurrent(idx)     (sessionDB[(idx)].fPmSetCurrent)

#define SESSION_SetOutputPower(idx,val)  (sessionDB[(idx)].u16OutputPower = (val))
#define SESSION_GetOutputPower(idx)      (sessionDB[(idx)].u16OutputPower)

#define SESSION_SetSystemFaultBitmap(idx,val) (sessionDB[(idx)].u16SessionFaultCode = (val))
#define SESSION_GetSystemFaultBitmap(idx)     (sessionDB[(idx)].u16SessionFaultCode)

#define SESSION_SetEstimatedChargingTime(idx,val)      (sessionDB[(idx)].u16EstimatedChargingTime = (val))
#define SESSION_GetEstimatedChargingTime(idx)          (sessionDB[(idx)].u16EstimatedChargingTime)
/* 32-bit variable */
#define SESSION_SetEnergyDelivered(idx,val)  (sessionDB[(idx)].u32EnergyDelivered = (val))
#define SESSION_GetEnergyDelivered(idx)      (sessionDB[(idx)].u32EnergyDelivered)

#define SESSION_SetPMLastRxTime(idx,val)    (sessionDB[(idx)].u32PMLastRxTime = (val))
#define SESSION_GetPMLastRxTime(idx)        (sessionDB[(idx)].u32PMLastRxTime)

#define SESSION_SetBMSLastRxTime(idx,val)   (sessionDB[(idx)].u32BMSLastRxTime = (val))
#define SESSION_GetBMSLastRxTime(idx)       (sessionDB[(idx)].u32BMSLastRxTime)
/* ************************************************************************** */
/* Section: Interface Functions                                               */
/* ************************************************************************** */
void SESSION_ResetAll(void);
void SESSION_ResetSession(uint8_t idx);
void SESSION_ResetBMSData(uint8_t idx);
void SESSION_ResetPMData(uint8_t idx);
void SESSION_ResetTempData(uint8_t idx);
/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _SESSION_DB_HANDLER_H */

/* *****************************************************************************
 End of File
 */