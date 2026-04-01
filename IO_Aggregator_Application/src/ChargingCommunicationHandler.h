#ifndef CHARGING_COMMUNICATION_HANDLER_H
#define CHARGING_COMMUNICATION_HANDLER_H

/**
 * @file    ChargingCommunicationHandler.h
 * @brief   Charging CAN communication handler interface
 *
 * @details
 * Provides APIs for handling communication between BMS and Power Module (PM)
 * over CAN during charging operation.
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *                              MACROS
 * ========================================================================== */

#define TONHE_MODULE_START      (0xAAU)
#define TONHE_MODULE_STOP       (0x55U)
#define TONHE_MOD_BASE_ID       (0x080601A0U)

#define TONHE_MAX_VOLTAGE 58U
#define TONHE_MAX_CURRENT 55U
#define TONHE_MIN_CURRENT 1U
/* ============================================================================
 *                              TYPEDEFS
 * ========================================================================== */

/**
 * @brief TONHE Power Module TX frame
 */
typedef struct
{
    uint8_t  u8ModuleStartStop;
    uint8_t  u8ChargingMode;
    uint16_t u16chargingVoltage;
    uint16_t u16chargingCurrent;
    uint16_t u16StandBy;
} tonhe_pm_Tx_t;

typedef struct
{
    uint8_t u8Chargingmodulestatus;
    uint16_t u16Outputvoltage;
    uint16_t u16Outputcurrent;
    uint16_t u16FaultInfo;
    uint8_t u8PFCFaultInfo;
} tonhe_pm_Rx_t;
/* ============================================================================
 *                              FUNCTION PROTOTYPES
 * ========================================================================== */

/**
 * @brief Initialize charging communication module
 */
void vChargingCommunicationInit(void);

/**
 * @brief Periodic CAN TX timer callback
 *
 * @param xTimer Timer handle
 */
void vChargingCanCommunicationTxTimerCallback(void *xTimer);

/**
 * @brief Process BMS messages and send responses
 *
 * @param u8DockNo Dock number
 */
void vProcessBMSMessage(uint8_t u8DockNo);

/**
 * @brief Process PM command transmission
 *
 * @param u8DockNo Dock number
 */
void vProcessPMMessage(uint8_t u8DockNo);

#ifdef __cplusplus
}
#endif

#endif /* CHARGING_COMMUNICATION_HANDLER_H */