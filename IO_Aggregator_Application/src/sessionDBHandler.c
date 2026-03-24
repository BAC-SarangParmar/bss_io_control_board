/* ************************************************************************** */
/** @file    SessionDBHandler.c
  @brief   Session Database Handler Implementation

  @Company
    Your Company Name

  @Summary
    Implements session database APIs.

  @Description
    This module manages charging session storage, update, and retrieval.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */

#include "SessionDBHandler.h"

/* ************************************************************************** */
/* Section: Private Macros                                                    */
/* ************************************************************************** */

/* ************************************************************************** */
/* Section: Private Variables                                                 */
/* ************************************************************************** */

SESSION_Data_t sessionDB[MAX_DOCKS]; /* Global session database */

/* ************************************************************************** */
/* Section: Private Function Prototypes                                       */
/* ************************************************************************** */

/* ************************************************************************** */
/* Section: Global Functions                                                  */
/* ************************************************************************** */

/* ************************************************************************** */
/* Reset Functions                                                            */
/* ************************************************************************** */

/**
 * @brief Reset PM-related data for a given session index
 */
void SESSION_ResetPMData(uint8_t idx)
{
    SESSION_SetPMRxStatus(idx, 0U);
    SESSION_SetPMTemperature(idx, 0U);
    SESSION_SetPMFaultCode(idx, 0U);
    SESSION_SetPmOutputVoltage(idx, 0U);
    SESSION_SetPmOutputCurrent(idx, 0U);
    SESSION_SetOutputPower(idx, 0U);
}

/**
 * @brief Reset BMS-related data for a given session index
 */
void SESSION_ResetBMSData(uint8_t idx)
{
    SESSION_SetBMSRxStatus(idx, 0U);
    SESSION_SetBMSTemperature(idx, 0U);
    SESSION_SetBMSFaultCode(idx, 0U);
    SESSION_SetBMSDemandVoltage(idx, 0U);
    SESSION_SetBMSDemandCurrent(idx, 0U);
    SESSION_SetCurrentSoc(idx, 0U);
    SESSION_SetInitialSoc(idx, 0U);
}

/**
 * @brief Reset temperature-related data for a given session index
 */
void SESSION_ResetTempData(uint8_t idx)
{
    SESSION_SetDockTemperature(idx, 0U);
}

/**
 * @brief Reset the entire session data for a given session index
 */
void SESSION_ResetSession(uint8_t idx)
{
    SESSION_SetState(idx, 0U);
    SESSION_SetSessionEndReason(idx, 0U);
    SESSION_SetAuthenticationCommand(idx, 0U);
    SESSION_SetEnergyDelivered(idx, 0UL);

    /* Reset all sub-components */
    SESSION_ResetPMData(idx);
    SESSION_ResetBMSData(idx);
    SESSION_ResetTempData(idx);
}

/**
 * @brief Reset all session entries in sessionDB
 */
void SESSION_ResetAll(void)
{
    for (uint8_t idx = 0U; idx < MAX_DOCKS; idx++)
    {
        SESSION_ResetSession(idx);
    }
}
/* *****************************************************************************
 End of File
 */