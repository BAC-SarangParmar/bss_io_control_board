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

// /* ************************************************************************** */
// /* Section: Constants                                                         */
// /* ************************************************************************** */
// /**
//  * @brief Invalid Session ID
//  */
// #define SESSION_DB_INVALID_ID         (0xFFU)

// /* ************************************************************************** */
// /* Section: Data Types                                                        */
// /* ************************************************************************** */

  /* Charging State Enum */
typedef enum
{
  CH_STATE_INIT = 0,
  CH_STATE_AUTH_SUCCESS,
  CH_STATE_PARAM_VALIDATE,
  CH_STATE_INITIALIZE,
  CH_STATE_PRECHARGE,
  CH_STATE_CHARGING,
  CH_STATE_SHUTDOWN,
  CH_STATE_SESSION_COMPLETE,
  CH_STATE_ERROR
} CH_State_e;

// typedef enum
// {
//     DOCK_1 = 0,
//     DOCK_2,
//     DOCK_3,
//     MAX_DOCKS
// } Dock_e;

// typedef enum
// {
//     GET_PARA = 0x01U,
//     SET_PARA = 0x02U
// }

// typedef enum
// {
//     CHARGING_SESSION_STARTED = 0x01U,
//     CHARGING_SESSION_STOPPED = 0x00U,
// } SessionEvent_e;
// /**
//  * @brief Session data structure
//  */
typedef struct
{
    CH_State_e state;
    bool bPMOnOffStatus;
    uint8_t u8CurrentSoc;
    uint8_t u8InitialSoc;
    uint8_t u8SessionEndReason;
    uint8_t u8BMSRxStatue;
    uint8_t u8PMRxStatus;
    uint8_t u8AuthenticationCommand;
    uint8_t u8DockTemperature;
    uint8_t u8BMSTemperature;
    uint8_t u8PMTemperature;
    uint8_t u8BMSFaultCode;
    uint8_t u8PMFaultCode;
    uint16_t u16PmOutputVoltage;
    uint16_t u16PmOutputCurrent;
    uint16_t u16BMSDemandVoltage;
    uint16_t u16BMSDemandCurrent;
    uint16_t u16OutputPower;
    uint32_t u32EnergyDelivered;
} SESSION_Data_t;
extern SESSION_Data_t sessionDB[4];

/*
* Session Database Handler Macros
*/

/* State */
#define SESSION_SetState(idx, val)          (sessionDB[(idx)].state = (val))
#define SESSION_GetState(idx)               (sessionDB[(idx)].state)

#define SESSION_SetPMState(idx, val)          (sessionDB[(idx)].bPMOnOffStatus = (val))
#define SESSION_GetPMState(idx)               (sessionDB[(idx)].bPMOnOffStatus)

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

#define SESSION_SetBMSFaultCode(idx,val)    (sessionDB[(idx)].u8BMSFaultCode = (val))
#define SESSION_GetBMSFaultCode(idx)        (sessionDB[(idx)].u8BMSFaultCode)

#define SESSION_SetPMFaultCode(idx,val)     (sessionDB[(idx)].u8PMFaultCode = (val))
#define SESSION_GetPMFaultCode(idx)         (sessionDB[(idx)].u8PMFaultCode)

/* 16-bit variables */
#define SESSION_SetPmOutputVoltage(idx,val) (sessionDB[(idx)].u16PmOutputVoltage = (val))
#define SESSION_GetPmOutputVoltage(idx)     (sessionDB[(idx)].u16PmOutputVoltage)

#define SESSION_SetPmOutputCurrent(idx,val) (sessionDB[(idx)].u16PmOutputCurrent = (val))
#define SESSION_GetPmOutputCurrent(idx)     (sessionDB[(idx)].u16PmOutputCurrent)

#define SESSION_SetBMSDemandVoltage(idx,val) (sessionDB[(idx)].u16BMSDemandVoltage = (val))
#define SESSION_GetBMSDemandVoltage(idx)     (sessionDB[(idx)].u16BMSDemandVoltage)

#define SESSION_SetBMSDemandCurrent(idx,val) (sessionDB[(idx)].u16BMSDemandCurrent = (val))
#define SESSION_GetBMSDemandCurrent(idx)     (sessionDB[(idx)].u16BMSDemandCurrent)

#define SESSION_SetOutputPower(idx,val)      (sessionDB[(idx)].u16OutputPower = (val))
#define SESSION_GetOutputPower(idx)          (sessionDB[(idx)].u16OutputPower)

/* 32-bit variable */
#define SESSION_SetEnergyDelivered(idx,val)  (sessionDB[(idx)].u32EnergyDelivered = (val))
#define SESSION_GetEnergyDelivered(idx)      (sessionDB[(idx)].u32EnergyDelivered)

/* ************************************************************************** */
/* Section: Interface Functions                                               */
/* ************************************************************************** */

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _SESSION_DB_HANDLER_H */

/* *****************************************************************************
 End of File
 */