/******************************************************************************
 * @file    ChargingCommunicationHandler.h
 * @brief   Charging CAN Communication Handler — Public Interface
 *
 * @details
 * This module manages all CAN bus communication during a charging session.
 * It handles two independent communication channels per dock:
 *
 *   1. BMS Channel  — Exchanges charging frames with the Battery Management
 *                     System (EV side) using the protocol selected at compile
 *                     time via CHARGING_PROTOCOL in ChargingHandler.h:
 *                       - PROTOCOL_17017_25 : LEVDC ISO 17017-25 CAN frames
 *                       - PROTOCOL_TVS_PROP : TVS proprietary CAN frames
 *
 *   2. PM Channel   — Sends control commands to and receives telemetry from
 *                     the TONHE Power Module (protocol-independent).
 *
 * A periodic FreeRTOS software timer (100 ms) drives all TX frames.
 * Incoming CAN frames are dispatched by the CAN ISR/task layer into
 * vProcessBMSCanMessage() and vProcessPMCanMessage().
 *
 * @note
 *   Include this header AFTER ChargingHandler.h so that the protocol
 *   selection macro (CHARGING_PROTOCOL) and all type definitions are visible.
 *
 * @author  Sarang Parmar
 * @date    2026-03-16
 * @version 2.0
 ******************************************************************************/

#ifndef CHARGING_COMMUNICATION_HANDLER_H
#define CHARGING_COMMUNICATION_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "ChargingHandler.h"   /* Must precede this header — supplies CHARGING_PROTOCOL */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * SECTION 1: TONHE POWER MODULE — CONSTANTS & FRAME TYPES
 * (Protocol-independent; used regardless of BMS protocol selection)
 * ========================================================================== */

/** @defgroup TONHE_CAN TONHE Power Module CAN IDs and Limits
 *  @{
 */
#define TONHE_MODULE_TX_ID       (0x080601A0U) /**< Extended CAN ID for TX TONHE PM       */
#define TONHE_MODULE_RX_ID       (0x1801A001U) /**< Extended CAN ID for RX TONHE PM       */
#define TONHE_MODULE_START      (0xAAU)       /**< Command: start power output         */
#define TONHE_MODULE_STOP       (0x55U)       /**< Command: stop power output          */

#define TONHE_MAX_VOLTAGE       (58U)         /**< Max allowable set voltage (V)       */
#define TONHE_MAX_CURRENT       (55U)         /**< Max allowable set current (A)       */
#define TONHE_MIN_CURRENT       (1U)          /**< Min allowable set current (A)       */
/** @} */

/**
 * @brief TONHE PM TX frame — sent by EVSE to command the power module.
 *
 * Byte layout (8 bytes, Extended CAN, ID = TONHE_MODULE_TX_ID):
 *   Byte 0      : ModuleStartStop  (0xAA = start, 0x55 = stop)
 *   Byte 1      : ChargingMode     (1 = constant-current/voltage mode)
 *   Bytes 2-3   : ChargingVoltage  (raw = physical_V x 10)
 *   Bytes 4-5   : ChargingCurrent  (raw = physical_A x 100)
 *   Bytes 6-7   : StandBy          (reserved, set to 0)
 */
typedef struct __attribute__((packed))
{
    uint8_t  u8ModuleStartStop;   /**< Start / Stop command byte               */
    uint8_t  u8ChargingMode;      /**< Charging mode selection                 */
    uint16_t u16chargingVoltage;  /**< Set voltage (raw = physical x 10)       */
    uint16_t u16chargingCurrent;  /**< Set current (raw = physical x 100)      */
    uint16_t u16StandBy;          /**< Reserved standby word                   */
} tonhe_pm_Tx_t;

/**
 * @brief TONHE PM RX frame — received from power module as telemetry.
 *
 * Byte layout (8 bytes, Extended CAN, ID = TONHE_MODULE_RX_ID):
 *   Byte 0      : ChargingModuleStatus (module operational state)
 *   Bytes 1-2   : OutputVoltage        (raw; physical_V = raw / 10)
 *   Bytes 3-4   : OutputCurrent        (raw; physical_A = raw / 100)
 *   Bytes 5-6   : FaultInfo            (bit-field fault register)
 *   Byte 7      : PFCFaultInfo         (PFC-specific fault register)
 */
typedef struct __attribute__((packed))
{
    uint8_t  u8Chargingmodulestatus; /**< Module status byte                   */
    uint16_t u16Outputvoltage;       /**< Output voltage (raw; phys = raw/10)  */
    uint16_t u16Outputcurrent;       /**< Output current (raw; phys = raw/100) */
    uint16_t u16FaultInfo;           /**< Fault register bit-field             */
    uint8_t  u8PFCFaultInfo;         /**< PFC fault register                   */
} tonhe_pm_Rx_t;


/* ============================================================================
 * SECTION 2: CAN ID VALIDATION HELPERS
 * ========================================================================== */

/**
 * @brief  Check whether a received CAN ID belongs to a known PM frame.
 * @param  canId  Raw CAN identifier from the RX buffer.
 * @return true   if canId matches any expected PM message ID.
 * @return false  otherwise.
 */
bool bIsValidPMCanID(uint32_t canId);

/**
 * @brief  Check whether a received CAN ID belongs to a known BMS frame.
 *
 * The set of valid IDs depends on the active CHARGING_PROTOCOL:
 *   PROTOCOL_17017_25 : LEVDC_CAN_ID_EV_REQUEST, _EV_CHARGING_INFO, _EV_CONTROL_OPTION
 *   PROTOCOL_TVS_PROP : TVS_CAN_ID_STATUS, TVS_CAN_ID_PROFILE
 *
 * @param  canId  Raw CAN identifier from the RX buffer.
 * @return true   if canId matches any expected BMS message ID.
 * @return false  otherwise.
 */
bool bIsValidBMSCanID(uint32_t canId);


/* ============================================================================
 * SECTION 3: CAN RX DISPATCH — called from CAN ISR / RX task
 * ========================================================================== */

/**
 * @brief  Dispatch an incoming CAN frame to the PM processing path.
 *
 * Resolves dock number from canBus index, updates the PM last-RX
 * timestamp, validates the CAN ID, decodes voltage/current/fault
 * telemetry, and stores results via SESSION_SetPm* accessors.
 *
 * @param  rxBuf  Pointer to the populated CAN RX buffer.  Must not be NULL.
 * @param  canBus CAN bus index (CANBUS_0 = DOCK_1, CANBUS_1 = DOCK_2, ...).
 */
void vProcessPMCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus);

/**
 * @brief  Dispatch an incoming CAN frame to the BMS processing path.
 *
 * Resolves dock number from canBus index, updates the BMS last-RX
 * timestamp, validates the CAN ID, and stores the decoded frame into
 * the shared protocol data store (LEVDC or TVS, as selected).
 *
 * @param  rxBuf  Pointer to the populated CAN RX buffer.  Must not be NULL.
 * @param  canBus CAN bus index (CANBUS_0 = DOCK_1, CANBUS_1 = DOCK_2, ...).
 */
void vProcessBMSCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus);


/* ============================================================================
 * SECTION 4: PERIODIC TX PROCESSING — driven by the 100 ms timer
 * ========================================================================== */

/**
 * @brief  Build and enqueue all BMS TX frames for one dock.
 *
 * Behaviour is protocol-dependent:
 *   PROTOCOL_17017_25 : Enqueues 0x508 (EVSE Status), 0x509 (Output Info),
 *                       and 0x510 (Capability) every 100 ms.
 *   PROTOCOL_TVS_PROP : Enqueues 0x90 (Charger Info) and 0x91 (Charge
 *                       Profile) every 100 ms; 0x92 (FW Version Info) is
 *                       sent every 1000 ms via an internal sub-counter.
 *
 * @note   Silently returns if SESSION_GetStartChargingComm() is false.
 *
 * @param  u8DockNo  Dock number (DOCK_1 through MAX_DOCKS-1).
 */
void vProcessBMSMessage(uint8_t u8DockNo);

/**
 * @brief  Build and enqueue the TONHE PM command frame for one dock.
 *
 * Reads set-points from the session DB, clamps them to safe TONHE
 * limits, scales to raw CAN units, and enqueues an Extended CAN TX frame.
 *
 * @param  u8DockNo  Dock number (DOCK_1 through MAX_DOCKS-1).
 */
void vProcessPMMessage(uint8_t u8DockNo);


/* ============================================================================
 * SECTION 5: TIMER CALLBACK & INITIALISATION
 * ========================================================================== */

/**
 * @brief  FreeRTOS software timer callback — fires every 100 ms.
 *
 * Iterates over all docks and calls vProcessBMSMessage() followed by
 * vProcessPMMessage() for each.  For the TVS protocol, an internal
 * tick counter gates the 0x92 FW Version frame to 1000 ms intervals.
 *
 * @param  xTimer  FreeRTOS timer handle (unused; cast to void).
 */
void vChargingCanCommunicationTxTimerCallback(void *xTimer);

/**
 * @brief  One-time initialisation of the Charging Communication module.
 *
 * Creates and starts the 100 ms CAN TX FreeRTOS software timer.
 * Must be called once during system start-up before tasks begin
 * processing charging frames.
 */
void vChargingCommunicationInit(void);

#ifdef __cplusplus
}
#endif

#endif /* CHARGING_COMMUNICATION_HANDLER_H */