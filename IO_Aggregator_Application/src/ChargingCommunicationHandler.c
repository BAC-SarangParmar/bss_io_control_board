/******************************************************************************
 * @file    ChargingCommunicationHandler.c
 * @brief   Charging CAN Communication Handler — Implementation
 *
 * @details
 * Implements periodic CAN TX (BMS + PM) and CAN RX dispatch for every dock.
 *
 * BMS protocol is selected at compile time in ChargingHandler.h:
 *   PROTOCOL_17017_25 — LEVDC ISO 17017-25 standard CAN frames
 *   PROTOCOL_TVS_PROP — TVS proprietary CAN frames
 *
 * Power Module communication (TONHE) is protocol-independent and always
 * active.
 *
 * Timer layout:
 *   One 100 ms FreeRTOS software timer drives all TX.
 *   TVS 0x92 (FW Version) is further divided to fire every 1000 ms
 *   via an internal tick counter (TVS_FW_VERSION_TX_DIVIDER).
 *
 * Dock ↔ CAN bus mapping:
 *   CANBUS_0 → DOCK_1
 *   CANBUS_1 → DOCK_2
 *   CANBUS_2 → DOCK_3
 *  CANBUS_3 → DOCK_4 (reserved)
 *  CANBUS_4 → DOCK_5 (reserved)
 *  CANBUS_5 → DOCK_6 (reserved)
 *
 * @author  Sarang Parmar
 * @date    2026-03-16
 * @version 2.0
 ******************************************************************************/

/* ============================================================================
 * INCLUDES
 * ========================================================================== */
#include "definitions.h"
#include "configuration.h"
#include "device.h"
#include "timers.h"
#include "ChargingCommunicationHandler.h"
#include "sessionDBHandler.h"
#include "ChargingHandler.h"

/* ============================================================================
 * MACROS
 * ========================================================================== */

/** Periodic CAN TX interval in milliseconds */
#define CAN_COMMUNICATION_TX_INTERVAL_MS    (101U)

#if (CHARGING_PROTOCOL == PROTOCOL_TVS_PROP)
/**
 * Number of 100 ms ticks before a TVS 0x92 (FW Version Info) frame is sent.
 * 10 ticks × 100 ms = 1000 ms.
 */
#define TVS_FW_VERSION_TX_DIVIDER           (10U)
#endif /* PROTOCOL_TVS_PROP */

/* ============================================================================
 * PRIVATE VARIABLES
 * ========================================================================== */

/** Handle for the 100 ms CAN TX software timer */
static TimerHandle_t xTimerCanCommunicationTx = NULL;

typedef enum
{
    DELTA3KW_PENDING_NONE = 0,
    DELTA3KW_PENDING_VOLTAGE,
    DELTA3KW_PENDING_CURRENT
} delta3kw_pending_e;

static delta3kw_pending_e s_eDelta3kwPending[NUM_RECTIFIERS + 1U] = {DELTA3KW_PENDING_NONE};

static inline bool bDelta3kwIndexValid(uint8_t u8RectifierIndex)
{
    return (u8RectifierIndex < (NUM_RECTIFIERS + 1U));
}

static inline uint32_t u32Delta3kwExtractResultValue(const uint8_t *pu8Data)
{
    return ((uint32_t)pu8Data[0] << 24) |
           ((uint32_t)pu8Data[1] << 16) |
           ((uint32_t)pu8Data[2] << 8)  |
            (uint32_t)pu8Data[3];
}

static bool s_bDelta3kwModuleOn[MAX_DOCKS] = {false};
static power_module_msg_st delta3kwMsgDb[DB_DELTA3KW_MSG_MAX] = {
    POWER_MODULE_DB_MSG_ENTRY(DB_DELTA3KW_SET_VOLTAGE, DELTA3KW_MOD_WRITE_ID, DELTA3KW_SET_VOLTAGE, NOT_BUSY),
    POWER_MODULE_DB_MSG_ENTRY(DB_DELTA3KW_SET_CURRENT, DELTA3KW_MOD_WRITE_ID, DELTA3KW_SET_CURR_LIMIT, NOT_BUSY),
    POWER_MODULE_DB_MSG_ENTRY(DB_DELTA3KW_TURN_ON, DELTA3KW_MOD_ON_OFF_ID, DELTA3KW_POWER_MOD_ON, NOT_BUSY),
    POWER_MODULE_DB_MSG_ENTRY(DB_DELTA3KW_TURN_OFF, DELTA3KW_MOD_ON_OFF_ID, DELTA3KW_POWER_MOD_OFF, NOT_BUSY),
    POWER_MODULE_DB_MSG_ENTRY(DB_DELTA3KW_GET_VOLTAGE, DELTA3KW_MOD_READ_ID, DELTA3KW_GET_VOLTAGE, NOT_BUSY),
    POWER_MODULE_DB_MSG_ENTRY(DB_DELTA3KW_GET_CURRENT, DELTA3KW_MOD_READ_ID, DELTA3KW_GET_CURRENT, NOT_BUSY)};

#if (CHARGING_PROTOCOL == PROTOCOL_TVS_PROP)
/**
 * Rolling tick counter used to sub-divide the 100 ms timer to produce the
 * 1000 ms interval required by the TVS 0x92 FW Version Info frame.
 * Range: 0 … (TVS_FW_VERSION_TX_DIVIDER - 1).
 */
static uint8_t u8TVSFwVersionTxTick = 0U;
#endif /* PROTOCOL_TVS_PROP */

/* ============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ========================================================================== */

/**
 * @brief  Resolve a CAN bus index to its corresponding Dock_e value.
 *
 * @param  canBus      CAN bus index (CANBUS_0, CANBUS_1, CANBUS_2).
 * @param  pDockNo     Output pointer; set to the resolved dock on success.
 * @return true        if canBus is a valid, mapped index.
 * @return false       if canBus is out of range (caller should discard frame).
 */
static bool bResolveDockFromCanBus(uint8_t canBus, uint8_t *pDockNo);

/**
 * @brief  Build and enqueue TONHE PM command CAN frame for one dock.
 * @param  u8DockNo    Dock number (DOCK_1 … MAX_DOCKS-1).
 */
static void TonhePmExecuteCommand(uint8_t u8DockNo);
static void Delta3KwSendCurrentRead(uint8_t u8DockNo);
static void Delta3KwPmExecuteCommand(uint8_t u8DockNo);
static void Delta3KwPmPollStatus(uint8_t u8DockNo);

/* Protocol-specific private TX helpers */
#if (CHARGING_PROTOCOL == PROTOCOL_17017_25)
static void vSendLevdcBMSFrames(uint8_t u8DockNo);
#endif

#if (CHARGING_PROTOCOL == PROTOCOL_TVS_PROP)
static void vSendTVSBMSFrames(uint8_t u8DockNo);
#endif

/* ============================================================================
 * PRIVATE FUNCTIONS
 * ========================================================================== */

/**
 * @brief  Resolve CAN bus index to dock number.
 *
 * Centralises the bus→dock mapping so that every function that performs it
 * uses a single, consistent implementation.
 */
static bool bResolveDockFromCanBus(uint8_t canBus, uint8_t *pDockNo)
{
    if (pDockNo == NULL)
    {
        return false;
    }

    switch (canBus)
    {
        case CANBUS_0: *pDockNo = DOCK_1; return true;
        case CANBUS_1: *pDockNo = DOCK_2; return true;
        case CANBUS_2: *pDockNo = DOCK_3; return true;
        case CANBUS_3: *pDockNo = DOCK_4; return true;
        case CANBUS_4: *pDockNo = DOCK_5; return true;
        case CANBUS_5: *pDockNo = DOCK_6; return true;
        default:                           return false;
    }
}

/* --------------------------------------------------------------------------
 * TONHE Power Module Command
 * -------------------------------------------------------------------------- */

/**
 * @brief  Build and enqueue TONHE PM command frame for one dock.
 *
 * Reads the voltage and current set-points from the session database,
 * clamps both values to the TONHE hardware limits, converts to raw
 * CAN units (voltage × 10, current × 100), and enqueues an Extended
 * CAN TX frame on the bus associated with the given dock.
 *
 * If SESSION_GetPMState() is true the module is commanded to start;
 * otherwise it is commanded to stop.
 *
 * @param  u8DockNo  Dock number (DOCK_1 … MAX_DOCKS-1).
 *                   Returns immediately if u8DockNo is 0 (invalid).
 */
static void TonhePmExecuteCommand(uint8_t u8DockNo)
{
    if (u8DockNo == 0U)
    {
        return;
    }

    tonhe_pm_Tx_t tx = {0};

    /* Read configured PM set-points from session DB */
    uint16_t u16Voltage = (uint16_t)SESSION_GetPmVoltageSetpoint(u8DockNo);
    uint16_t u16Current = (uint16_t)SESSION_GetPmCurrentSetpoint(u8DockNo);

    /* Clamp voltage and current to TONHE hardware limits */
    u16Voltage = (u16Voltage > TONHE_MAX_VOLTAGE) ? TONHE_MAX_VOLTAGE : u16Voltage;
    u16Current = (u16Current > TONHE_MAX_CURRENT) ? TONHE_MAX_CURRENT : u16Current;
    u16Current = (u16Current < TONHE_MIN_CURRENT) ? TONHE_MIN_CURRENT : u16Current;

    /* Scale to raw CAN units */
    tx.u16chargingVoltage = (uint16_t)(u16Voltage * (uint16_t)FACTOR_10);
    tx.u16chargingCurrent = (uint16_t)(u16Current * (uint16_t)FACTOR_100);

    /* Set module state based on PM enable flag */
    if (SESSION_GetPMState(u8DockNo))
    {
        tx.u8ChargingMode    = 1U;
        tx.u8ModuleStartStop = TONHE_MODULE_START;
    }
    else
    {
        tx.u8ModuleStartStop = TONHE_MODULE_STOP;
    }

    /* Build and enqueue the Extended CAN TX frame */
    CAN_TX_BUFFER canTx = {0};
    canTx.id  = TONHE_MODULE_TX_ID;
    canTx.dlc = CAN_PAYLOAD_BYTE_SIZE;
    canTx.xtd = CAN_FRAME_EXTENDED;

    memcpy(canTx.data, &tx, sizeof(tonhe_pm_Tx_t));
    vSendCanTxMsgToQueue(&canTx, u8DockNo);
}

static void Delta3KwPmExecuteCommand(uint8_t u8DockNo)
{
    if (u8DockNo == 0U)
    {
        return;
    }

    /* Read configured PM set-points from session DB */
    float fVoltage = SESSION_GetPmVoltageSetpoint(u8DockNo);
    float fCurrent = SESSION_GetPmCurrentSetpoint(u8DockNo);

    /* Clamp voltage and current to DELTA hardware limits */
    fVoltage = (fVoltage > DELTA_MAX_VOLTAGE) ? DELTA_MAX_VOLTAGE : fVoltage;
    fCurrent = (fCurrent > DELTA_MAX_CURRENT) ? DELTA_MAX_CURRENT : fCurrent;
    fCurrent = (fCurrent < DELTA_MIN_CURRENT) ? DELTA_MIN_CURRENT : fCurrent;

    /* Scale to raw CAN units */
    uint32_t u32VoltageRaw = (uint32_t)(fVoltage * FACTOR_10);
    uint32_t u32CurrentRaw = (uint32_t)(fCurrent * FACTOR_10);

    /* Determine module start/stop from PM enable flag */
    bool bModuleStart = SESSION_GetPMState(u8DockNo);

    /* ---- SET_VOLTAGE frame ---- */
    CAN_TX_BUFFER txV = {0};
    power_module_msg_st *pCmdV = &delta3kwMsgDb[DB_DELTA3KW_SET_VOLTAGE];
    txV.id  = pCmdV->u32MsgId;
    txV.dlc = CAN_PAYLOAD_BYTE_SIZE;
    txV.xtd = CAN_FRAME_EXTENDED;
    memcpy(txV.data, pCmdV->u8MsgData, sizeof(pCmdV->u8MsgData));
    txV.data[5] = (u32VoltageRaw >> 8) & 0xFF;
    txV.data[6] = u32VoltageRaw & 0xFF;
    txV.data[7] = VOLTAGE_CHECKSUM_CAL(u32VoltageRaw);
    vSendCanTxMsgToQueue(&txV, u8DockNo);
    vTaskDelay(5);

    /* ---- SET_CURRENT frame ---- */
    CAN_TX_BUFFER txC = {0};
    power_module_msg_st *pCmdC = &delta3kwMsgDb[DB_DELTA3KW_SET_CURRENT];
    txC.id  = pCmdC->u32MsgId;
    txC.dlc = CAN_PAYLOAD_BYTE_SIZE;
    txC.xtd = CAN_FRAME_EXTENDED;
    memcpy(txC.data, pCmdC->u8MsgData, sizeof(pCmdC->u8MsgData));
    txC.data[5] = (u32CurrentRaw >> 8) & 0xFF;
    txC.data[6] = u32CurrentRaw & 0xFF;
    txC.data[7] = CURRENT_CHECKSUM_CAL(u32CurrentRaw);
    vSendCanTxMsgToQueue(&txC, u8DockNo);
    vTaskDelay(5);

    /* ---- START/STOP frame ---- */
    CAN_TX_BUFFER txS = {0};
    power_module_msg_st *pCmdS = &delta3kwMsgDb[bModuleStart ? DB_DELTA3KW_TURN_ON : DB_DELTA3KW_TURN_OFF];
    txS.id  = pCmdS->u32MsgId;
    txS.dlc = CAN_PAYLOAD_BYTE_SIZE;
    txS.xtd = CAN_FRAME_EXTENDED;
    memcpy(txS.data, pCmdS->u8MsgData, sizeof(pCmdS->u8MsgData));
    vSendCanTxMsgToQueue(&txS, u8DockNo);
    vTaskDelay(5);

    if (bModuleStart)
    {
        s_bDelta3kwModuleOn[u8DockNo] = true;
        s_eDelta3kwPending[u8DockNo]  = DELTA3KW_PENDING_NONE; /* clean start */
    }
    else
    {
        s_bDelta3kwModuleOn[u8DockNo] = false;
        s_eDelta3kwPending[u8DockNo]  = DELTA3KW_PENDING_NONE; /* clear any stale wait */
    }
}

static void Delta3KwPmPollStatus(uint8_t u8DockNo)
{
    if (u8DockNo == 0U)
    {
        return;
    }

    if (!s_bDelta3kwModuleOn[u8DockNo])
    {
        return;
    }

    if (s_eDelta3kwPending[u8DockNo] != DELTA3KW_PENDING_NONE)
    {
        return;
    }

    CAN_TX_BUFFER txGV = {0};
    power_module_msg_st *pCmdGV = &delta3kwMsgDb[DB_DELTA3KW_GET_VOLTAGE];

    txGV.id  = pCmdGV->u32MsgId;
    txGV.dlc = CAN_PAYLOAD_BYTE_SIZE;
    txGV.xtd = CAN_FRAME_EXTENDED;
    memcpy(txGV.data, pCmdGV->u8MsgData, sizeof(pCmdGV->u8MsgData));

    s_eDelta3kwPending[u8DockNo] = DELTA3KW_PENDING_VOLTAGE;

    vSendCanTxMsgToQueue(&txGV, u8DockNo);
}

static void Delta3KwSendCurrentRead(uint8_t u8DockNo)
{
    CAN_TX_BUFFER txGC = {0};

    power_module_msg_st *pCmdGC =
        &delta3kwMsgDb[DB_DELTA3KW_GET_CURRENT];

    txGC.id  = pCmdGC->u32MsgId;
    txGC.dlc = CAN_PAYLOAD_BYTE_SIZE;
    txGC.xtd = CAN_FRAME_EXTENDED;

    memcpy(txGC.data,
           pCmdGC->u8MsgData,
           sizeof(pCmdGC->u8MsgData));

    s_eDelta3kwPending[u8DockNo] = DELTA3KW_PENDING_CURRENT;

    vSendCanTxMsgToQueue(&txGC, u8DockNo);
}

/* --------------------------------------------------------------------------
 * LEVDC (ISO 17017-25) Protocol — TX helpers
 * -------------------------------------------------------------------------- */
#if (CHARGING_PROTOCOL == PROTOCOL_17017_25)

/**
 * @brief  Enqueue all three LEVDC EVSE→EV TX frames for one dock.
 *
 * Reads the current TX frame data from the shared LEVDC data store
 * (bGetSetLevdcBMSData / GET_PARA) and enqueues:
 *   0x508 — EVSE Status
 *   0x509 — EVSE Output Info
 *   0x510 — EVSE Capability
 *
 * All frames are Standard CAN (CAN_FRAME_STANDARD), 8-byte DLC.
 * The CAN ID macro CAN_WRITE_ID() applies any platform-specific ID encoding.
 *
 * @param  u8DockNo  Dock number (DOCK_1 … MAX_DOCKS-1).
 */
static void vSendLevdcBMSFrames(uint8_t u8DockNo)
{
    CAN_TX_BUFFER tx        = {0};
    ChargingMsgFrameInfo_t info = {0};

    (void)bGetSetLevdcBMSData(u8DockNo, &info, GET_PARA);

    tx.dlc = CAN_PAYLOAD_BYTE_SIZE;
    tx.xtd = CAN_FRAME_STANDARD;

    /* 0x508 — EVSE Status */
    tx.id = CAN_WRITE_ID(LEVDC_CAN_ID_EVSE_STATUS);
    memcpy(tx.data, &info.LevdcTX_508ID_Info, sizeof(LEVDC_Tx508_t));
    vSendCanTxMsgToQueue(&tx, u8DockNo);

    /* 0x509 — EVSE Output Info */
    tx.id = CAN_WRITE_ID(LEVDC_CAN_ID_EVSE_OUTPUT_INFO);
    memcpy(tx.data, &info.LevdcTX_509ID_Info, sizeof(LEVDC_Tx509_t));
    vSendCanTxMsgToQueue(&tx, u8DockNo);

    /* 0x510 — EVSE Capability */
    tx.id = CAN_WRITE_ID(LEVDC_CAN_ID_EVSE_CAPABILITY);
    memcpy(tx.data, &info.LevdcTX_510ID_Info, sizeof(LEVDC_Tx510_t));
    vSendCanTxMsgToQueue(&tx, u8DockNo);
}

#endif /* PROTOCOL_17017_25 */

/* --------------------------------------------------------------------------
 * TVS Proprietary Protocol — TX helpers
 * -------------------------------------------------------------------------- */
#if (CHARGING_PROTOCOL == PROTOCOL_TVS_PROP)

/**
 * @brief  Enqueue all TVS Charger→BMS TX frames for one dock.
 *
 * Called every 100 ms from vProcessBMSMessage().
 * Always enqueues:
 *   0x90 — Charger Info
 *   0x91 — Charge Profile
 * Enqueues every 1000 ms (when u8TVSFwVersionTxTick == 0):
 *   0x92 — FW Version Info
 *
 * All frames are Standard CAN (CAN_FRAME_STANDARD), 8-byte DLC.
 *
 * The function reads TVS live frame data from the shared data store
 * (bGetSetTVSBMSData / GET_PARA) for frames 0x90 and 0x91.
 * Frame 0x92 is populated from compile-time firmware version constants.
 *
 * @param  u8DockNo  Dock number (DOCK_1 … MAX_DOCKS-1).
 */
static void vSendTVSBMSFrames(uint8_t u8DockNo)
{
    CAN_TX_BUFFER  tx   = {0};
    TVS_MsgFrameInfo_t info = {0};

    (void)bGetSetTVSBMSData(u8DockNo, &info, GET_PARA);

    tx.dlc = CAN_PAYLOAD_BYTE_SIZE;
    tx.xtd = CAN_FRAME_STANDARD;

    /* 0x90 — Charger Info (100 ms) */
    tx.id = CAN_WRITE_ID(TVS_CAN_ID_INFO);
    memcpy(tx.data, &info.TVS_Tx90_ChargerInfo, sizeof(TVS_Tx90_Info_t));
    vSendCanTxMsgToQueue(&tx, u8DockNo);

    /* 0x91 — Charge Profile (100 ms) */
    tx.id = CAN_WRITE_ID(TVS_CAN_ID_CHARGE_PROFILE);
    memcpy(tx.data, &info.TVS_Tx91_ChargeProfile, sizeof(TVS_Tx91_ChargeProfile_t));
    vSendCanTxMsgToQueue(&tx, u8DockNo);

    /* 0x92 — FW Version Info (1000 ms sub-divided from 100 ms timer) */
    if (u8TVSFwVersionTxTick == 0U)
    {
        TVS_Tx92_FMVersionInfo_t fwInfo = {0};
        fwInfo.u8FWVersionMajor     = FW_VERSION_MAJOR;
        fwInfo.u8FWVersionMinor     = FW_VERSION_MINOR;
        fwInfo.u8FWVersionIteration = FW_VERSION_ITERATION;
        fwInfo.u8FWChargerType      = (uint8_t)FW_CHARGER_TYPE;
        fwInfo.u8FWReleaseDateDD    = FW_RELEASE_DATE_DD;
        fwInfo.u8FWReleaseDateMM    = FW_RELEASE_DATE_MM;
        fwInfo.u8FWReleaseDateY1Y2  = FW_RELEASE_DATE_Y1Y2;
        fwInfo.u8FWReleaseDateY3Y4  = FW_RELEASE_DATE_Y3Y4;

        tx.id = CAN_WRITE_ID(TVS_CAN_ID_FM_VERSION_INFO);
        memcpy(tx.data, &fwInfo, sizeof(TVS_Tx92_FMVersionInfo_t));
        vSendCanTxMsgToQueue(&tx, u8DockNo);
    }
}

#endif /* PROTOCOL_TVS_PROP */

/* ============================================================================
 * PUBLIC FUNCTIONS
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * CAN ID Validation
 * -------------------------------------------------------------------------- */

bool bIsValidPMCanID(uint32_t canId)
{
    return (canId == TONHE_MODULE_RX_ID);
}

bool bIsValidBMSCanID(uint32_t canId)
{
#if (CHARGING_PROTOCOL == PROTOCOL_17017_25)
    return (canId == LEVDC_CAN_ID_EV_REQUEST       ||
            canId == LEVDC_CAN_ID_EV_CHARGING_INFO ||
            canId == LEVDC_CAN_ID_EV_CONTROL_OPTION);

#elif (CHARGING_PROTOCOL == PROTOCOL_TVS_PROP)
    return (canId == TVS_CAN_ID_STATUS  ||
            canId == TVS_CAN_ID_PROFILE);

#else
    #error "bIsValidBMSCanID: unsupported CHARGING_PROTOCOL"
    return false;
#endif
}

/* --------------------------------------------------------------------------
 * PM Periodic TX
 * -------------------------------------------------------------------------- */

void vProcessPMMessage(uint8_t u8DockNo)
{
    #if (RECTIFIER_SELECTION == DELTA_RECTIFIER)
        Delta3KwPmExecuteCommand(u8DockNo);
        Delta3KwPmPollStatus(u8DockNo);
    #else               
    TonhePmExecuteCommand(u8DockNo);
    #endif
}

/* --------------------------------------------------------------------------
 * BMS Periodic TX
 * -------------------------------------------------------------------------- */

void vProcessBMSMessage(uint8_t u8DockNo)
{
    /* Do not send BMS frames until the charging session has started */
    if (!SESSION_GetStartChargingComm(u8DockNo))
    {
        return;
    }

#if (CHARGING_PROTOCOL == PROTOCOL_17017_25)
    vSendLevdcBMSFrames(u8DockNo);

#elif (CHARGING_PROTOCOL == PROTOCOL_TVS_PROP)
    vSendTVSBMSFrames(u8DockNo);

#else
    #error "vProcessBMSMessage: unsupported CHARGING_PROTOCOL"
#endif
}

/* --------------------------------------------------------------------------
 * 100 ms Timer Callback
 * -------------------------------------------------------------------------- */

void vChargingCanCommunicationTxTimerCallback(void *xTimer)
{
    (void)xTimer;

    /* Transmit BMS and PM frames for every dock */
    for (uint8_t u8DockNo = DOCK_1; u8DockNo <= SESSION_GetMaxDocks(); u8DockNo++)
    {
        vProcessBMSMessage(u8DockNo);
        vProcessPMMessage(u8DockNo);
    }

#if (CHARGING_PROTOCOL == PROTOCOL_TVS_PROP)
    /* Advance the 0x92 sub-divider counter (wraps 0 … TVS_FW_VERSION_TX_DIVIDER-1) */
    u8TVSFwVersionTxTick++;
    if (u8TVSFwVersionTxTick >= TVS_FW_VERSION_TX_DIVIDER)
    {
        u8TVSFwVersionTxTick = 0U;
    }
#endif /* PROTOCOL_TVS_PROP */
}

/* --------------------------------------------------------------------------
 * Module Initialisation
 * -------------------------------------------------------------------------- */

void vChargingCommunicationInit(void)
{
    static int32_t timerId = 0;

    xTimerCanCommunicationTx = xTimerCreate(
        "CanCommTxTimer",
        pdMS_TO_TICKS(CAN_COMMUNICATION_TX_INTERVAL_MS),
        pdTRUE,                             /* Auto-reload */
        (void *)&timerId,
        vChargingCanCommunicationTxTimerCallback
    );

    if (xTimerCanCommunicationTx != NULL)
    {
        xTimerStart(xTimerCanCommunicationTx, 0);
    }
    else
    {
        SYS_CONSOLE_PRINT("ChargingCommHandler: CAN Tx Timer creation failed\r\n");
    }
}

/* --------------------------------------------------------------------------
 * PM CAN RX Dispatch
 * -------------------------------------------------------------------------- */

void vProcessPMCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus)
{
    if (rxBuf == NULL)
    {
        return;
    }

    uint8_t u8DockNo = 0U;
    if (!bResolveDockFromCanBus(canBus, &u8DockNo))
    {
        return;
    }

    /* Record the time of the last valid PM frame (for timeout detection) */
    SESSION_SetPMLastRxTime(u8DockNo, xTaskGetTickCount());
    // SYS_CONSOLE_PRINT("Dock %u : Last Rx Tick %d\r\n",u8DockNo, SESSION_GetPMLastRxTime(u8DockNo));
    /* Validate CAN ID before decoding payload */
    if (!bIsValidPMCanID(rxBuf->id))
    {
        return;
    }

    /* Decode TONHE PM RX telemetry */
    tonhe_pm_Rx_t pmRx = {0};
    memcpy(&pmRx, rxBuf->data, sizeof(tonhe_pm_Rx_t));

    float    fVoltage    = (float)pmRx.u16Outputvoltage / FACTOR_10;
    float    fCurrent    = (float)pmRx.u16Outputcurrent / FACTOR_100;
    uint16_t u16FaultInfo = pmRx.u16FaultInfo;

    // SYS_CONSOLE_PRINT("G%u: PM CAN RX: Volt=%u (%.1fV), Curr=%u (%.2fA), Fault=0x%04X\r\n",
    //                   (unsigned)u8DockNo, (unsigned)pmRx.u16Outputvoltage, fVoltage,
    //                   (unsigned)pmRx.u16Outputcurrent, fCurrent, u16FaultInfo);
    SESSION_SetPmOutputVoltage(u8DockNo, fVoltage);
    SESSION_SetPmOutputCurrent(u8DockNo, fCurrent);
    SESSION_SetPMFaultCode(u8DockNo, u16FaultInfo);
}

void vProcessDelta3KwPMCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus)
{
    if (rxBuf == NULL)
    {
        return;
    }
    
    uint8_t u8DockNo = 0U;
    if (!bResolveDockFromCanBus(canBus, &u8DockNo))
    {
        return;
    }

    if (!bDelta3kwIndexValid(u8DockNo))
    {
        return;
    }

    /* Record the time of the last valid PM frame (for timeout detection) */
    SESSION_SetPMLastRxTime(u8DockNo, xTaskGetTickCount());

    uint32_t canId     = rxBuf->id;
    uint8_t  u8DataLen = rxBuf->dlc;

    /* Validate CAN ID before decoding payload */
    if (!(canId == DELTA3KW_READ_ID_1))
    {
        return;
    }

    if (u8DataLen < 4U)
    {
        // SYS_CONSOLE_PRINT("Dock %u: result frame too short (dlc=%u)\r\n", u8DockNo, u8DataLen);
        return;
    }

    uint32_t u32RawValue = u32Delta3kwExtractResultValue(rxBuf->data);
    delta3kw_pending_e ePending = s_eDelta3kwPending[u8DockNo];

    if (ePending == DELTA3KW_PENDING_VOLTAGE)
    {
        float fVoltage = (float)u32RawValue / FACTOR_100;
        SESSION_SetPmOutputVoltage(u8DockNo, fVoltage);
        Delta3KwSendCurrentRead(u8DockNo);
    }
    else if (ePending == DELTA3KW_PENDING_CURRENT)
    {
        float fCurrent = (float)u32RawValue / FACTOR_100;
        SESSION_SetPmOutputCurrent(u8DockNo, fCurrent);
        s_eDelta3kwPending[u8DockNo] = DELTA3KW_PENDING_NONE;
    }
    else
    {
        // SYS_CONSOLE_PRINT("Dock %u: unsolicited/unmatched result, dropped\r\n", u8DockNo);
    }

    SYS_CONSOLE_PRINT("Dock %u: Voltage=%.2f, Current=%.2f\r\n", u8DockNo, SESSION_GetPmOutputVoltage(u8DockNo), SESSION_GetPmOutputCurrent(u8DockNo));
}

/* --------------------------------------------------------------------------
 * BMS CAN RX Dispatch
 * -------------------------------------------------------------------------- */

void vProcessBMSCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus)
{
    if (rxBuf == NULL)
    {
        return;
    }

    uint8_t u8DockNo = 0U;
    if (!bResolveDockFromCanBus(canBus, &u8DockNo))
    {
        return;
    }

    /* Record the time of the last valid BMS frame (for timeout detection) */
    SESSION_SetBMSLastRxTime(u8DockNo, xTaskGetTickCount());

    uint32_t canId = CAN_READ_ID(rxBuf->id);

    if (!bIsValidBMSCanID(canId))
    {
        return;
    }

    /* ------------------------------------------------------------------ */
#if (CHARGING_PROTOCOL == PROTOCOL_17017_25)
    /* ------------------------------------------------------------------ */

    ChargingMsgFrameInfo_t info = {0};
    (void)bGetSetLevdcBMSData(u8DockNo, &info, GET_PARA);

    switch (canId)
    {
        case LEVDC_CAN_ID_EV_REQUEST:
            if (rxBuf->dlc >= sizeof(LEVDC_Rx500_t))
            {
                memcpy(&info.LevdcRX_500ID_Info, rxBuf->data, sizeof(LEVDC_Rx500_t));
            }
            break;

        case LEVDC_CAN_ID_EV_CHARGING_INFO:
            if (rxBuf->dlc >= sizeof(LEVDC_Rx501_t))
            {
                memcpy(&info.LevdcRX_501ID_Info, rxBuf->data, sizeof(LEVDC_Rx501_t));
            }
            break;

        case LEVDC_CAN_ID_EV_CONTROL_OPTION:
            if (rxBuf->dlc >= sizeof(LEVDC_Rx502_t))
            {
                memcpy(&info.LevdcRX_502ID_Info, rxBuf->data, sizeof(LEVDC_Rx502_t));
            }
            break;

        default:
            /* ID passed bIsValidBMSCanID but is unhandled here — safe no-op */
            break;
    }

    (void)bGetSetLevdcBMSData(u8DockNo, &info, SET_PARA);

    /* ------------------------------------------------------------------ */
#elif (CHARGING_PROTOCOL == PROTOCOL_TVS_PROP)
    /* ------------------------------------------------------------------ */

    TVS_MsgFrameInfo_t info = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &info, GET_PARA);

    switch (canId)
    {
        case TVS_CAN_ID_STATUS:
            if (rxBuf->dlc >= sizeof(TVS_Rx100_Status_t))
            {
                memcpy(&info.TVS_Rx100_BMSStatus, rxBuf->data, sizeof(TVS_Rx100_Status_t));
            }
            break;

        case TVS_CAN_ID_PROFILE:
            if (rxBuf->dlc >= sizeof(TVS_Rx101_Profile_t))
            {
                memcpy(&info.TVS_Rx101_BMSProfile, rxBuf->data, sizeof(TVS_Rx101_Profile_t));
            }
            break;

        default:
            /* ID passed bIsValidBMSCanID but is unhandled here — safe no-op */
            break;
    }

    (void)bGetSetTVSBMSData(u8DockNo, &info, SET_PARA);

    /* ------------------------------------------------------------------ */
#else
    #error "vProcessBMSCanMessage: unsupported CHARGING_PROTOCOL"
#endif
}

/* ============================================================================
 * END OF FILE
 * ========================================================================== */