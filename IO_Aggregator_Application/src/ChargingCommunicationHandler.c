/**
 * @file    ChargingCommunicationHandler.c
 * @brief   Source file for Charging Communication Handler module
 *
 * @details
 * This file contains the implementation of the Charging Communication Handler.
 */

/* ============================================================================
 *                              INCLUDES
 * ========================================================================== */
#include "definitions.h"                // SYS function prototypes
#include "configuration.h"
#include "device.h"
#include "timers.h"
#include "ChargingCommunicationHandler.h"
#include "sessionDBHandler.h"
#include "ChargingHandler.h"
/* ============================================================================
 *                              MACROS
 * ========================================================================== */

#define CAN_COMMUNICATION_TX_INTERVAL_MS 100U


/* ============================================================================
 *                              PRIVATE TYPEDEFS
 * ========================================================================== */

 TimerHandle_t xTimerCanCommunicationTx = NULL;

/* ============================================================================
 *                              PRIVATE VARIABLES
 * ========================================================================== */

/* Define static/global variables here */


/* ============================================================================
 *                              PRIVATE FUNCTION PROTOTYPES
 * ========================================================================== */

/* Declare static functions here */
/**
 * @brief Process received PM CAN message
 *
 * @param rxBuf Pointer to received buffer
 * @param canBus CAN bus index
 */
void vProcessPMCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus);

/**
 * @brief Process received BMS CAN message
 *
 * @param rxBuf Pointer to received buffer
 * @param canBus CAN bus index
 */
void vProcessBMSCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus);

/**
 * @brief Validate received CAN ID
 *
 * @param canId CAN identifier
 * @return true if valid
 */
bool bIsValidBMSCanID(uint32_t canId);
/* ============================================================================
 *                              PRIVATE FUNCTIONS
 * ========================================================================== */

/**
 * @brief Generate TONHE CAN ID based on dock number
 */
static inline uint32_t u32GetTonheCANID(uint8_t u8DockNo)
{
    return TONHE_MOD_BASE_ID + ((u8DockNo - 1U) * 0x100U);
}


/**
 * @brief Prepare and send TONHE PM command
 *
 * This function reads the set voltage and current for a dock,
 * clamps them to safe limits, prepares the PM TX frame, and
 * sends it over the CAN bus.
 *
 * @param u8DockNo Dock number (1-based)
 */
static void TonhePmExecuteCommand(uint8_t u8DockNo)
{
    if (u8DockNo == 0U)
    {
        return;
    }

    tonhe_pm_Tx_t tx = {0};

    /* Read configured PM setpoints */
    uint16_t u16Voltage = (uint16_t)SESSION_GetPmSetVoltage(u8DockNo);
    uint16_t u16Current = (uint16_t)SESSION_GetPmSetCurrent(u8DockNo);

    u16Voltage = (u16Voltage > TONHE_MAX_VOLTAGE) ? TONHE_MAX_VOLTAGE : u16Voltage;
    u16Current = (u16Current > TONHE_MAX_CURRENT) ? TONHE_MAX_CURRENT : u16Current;
    u16Current = (u16Current < TONHE_MIN_CURRENT) ? TONHE_MIN_CURRENT : u16Current;
    u16Voltage = u16Voltage * FACTOR_10;
    u16Current = u16Current * FACTOR_100;

    tx.u16chargingVoltage = u16Voltage;
    tx.u16chargingCurrent = u16Current;

    /* Set charging mode and module start/stop */
    if (SESSION_GetPMState(u8DockNo))
    {
        tx.u8ChargingMode = 1U;
        tx.u8ModuleStartStop = TONHE_MODULE_START;
    }
    else
    {
        tx.u8ModuleStartStop = TONHE_MODULE_STOP;
    }

    /* Prepare CAN TX buffer */
    CAN_TX_BUFFER canTx = {0};
    canTx.id  = TONHE_MOD_BASE_ID;//u32GetTonheCANID(u8DockNo);  // Use function to calculate CAN ID
    canTx.dlc = CAN_DATA_SIZE;
    canTx.xtd = EXT_CAN_MSG;

    memcpy(canTx.data, &tx, sizeof(tx));

    /* Send message to CAN queue */
    vSendCanTxMsgToQueue(&canTx, u8DockNo);
}


/* ============================================================================
 *                              PUBLIC FUNCTIONS
 * ========================================================================== */

void vProcessPMMessage(uint8_t u8DockNo)
{
    TonhePmExecuteCommand(u8DockNo);
}


void vProcessBMSMessage(uint8_t u8DockNo)
{
    if (!SESSION_GetStartChargingComm(u8DockNo))
    {
        return;
    }

    CAN_TX_BUFFER tx = {0};
    ChargingMsgFrameInfo_t info = {0};

    (void)u8GetSetBMSData(u8DockNo, &info, GET_PARA);

    tx.dlc = CAN_DATA_SIZE;
    tx.xtd = STD_CAN_MSG;

    tx.id = WRITE_ID(LEVDC_CAN_ID_EVSE_STATUS);
    memcpy(tx.data, &info.LevdcTX_508ID_Info, sizeof(LEVDC_Tx508_t));
    vSendCanTxMsgToQueue(&tx, u8DockNo);

    tx.id = WRITE_ID(LEVDC_CAN_ID_EVSE_OUTPUT_INFO);
    memcpy(tx.data, &info.LevdcTX_509ID_Info, sizeof(LEVDC_Tx509_t));
    vSendCanTxMsgToQueue(&tx, u8DockNo);

    tx.id = WRITE_ID(LEVDC_CAN_ID_EVSE_CAPABILITY);
    memcpy(tx.data, &info.LevdcTX_510ID_Info, sizeof(LEVDC_Tx510_t));
    vSendCanTxMsgToQueue(&tx, u8DockNo);
}


void vChargingCanCommunicationTxTimerCallback(void *xTimer)
{
    (void)xTimer;

    for (uint8_t dock = DOCK_1; dock < MAX_DOCKS; dock++)
    {
        vProcessBMSMessage(dock);
        vProcessPMMessage(dock);
    }
}


void vChargingCommunicationInit(void)
{
    static int32_t timerId = 0;

    xTimerCanCommunicationTx = xTimerCreate(
        "CanCommTxTimer",
        pdMS_TO_TICKS(CAN_COMMUNICATION_TX_INTERVAL_MS),
        pdTRUE,
        (void *)&timerId,
        vChargingCanCommunicationTxTimerCallback
    );

    if (xTimerCanCommunicationTx != NULL)
    {
        xTimerStart(xTimerCanCommunicationTx, 0);
    }
    else
    {
        SYS_CONSOLE_PRINT("CAN Tx Timer creation failed\r\n");
    }
}


bool bIsValidBMSCanID(uint32_t canId)
{
    return (canId == LEVDC_CAN_ID_EV_REQUEST ||
            canId == LEVDC_CAN_ID_EV_CHARGING_INFO ||
            canId == LEVDC_CAN_ID_EV_CONTROL_OPTION);
}

bool bIsValidPMCanID(uint32_t canId)
{
    return (canId == TONHE_MOD_BASE_ID);
}

void vProcessPMCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus)
{
    if (rxBuf == NULL)
    {
        return;
    }
        Dock_e u8DockNo = DOCK_1;
    switch (canBus)
    {
        case CANBUS_0: u8DockNo = DOCK_1; break;
        case CANBUS_1: u8DockNo = DOCK_2; break;
        case CANBUS_2: u8DockNo = DOCK_3; break;
        default: return;
    }
    SESSION_SetPMLastRxTime(u8DockNo, xTaskGetTickCount());
    
    uint32_t canId = rxBuf->id;
    if (!bIsValidPMCanID(canId))
    {
        return;
    }

    tonhe_pm_Rx_t pmRx = {0};
    memcpy(&pmRx, rxBuf->data, sizeof(tonhe_pm_Rx_t));
    float fVoltage = pmRx.u16Outputvoltage / (float)FACTOR_10;
    float fCurrent = pmRx.u16Outputcurrent / (float)FACTOR_100;
    uint16_t u16FaultInfo = pmRx.u16FaultInfo;

    SESSION_SetPmOutputCurrent(u8DockNo, fCurrent);
    SESSION_SetPmOutputVoltage(u8DockNo, fVoltage);
    SESSION_SetPMFaultCode(u8DockNo, u16FaultInfo);
}

void vProcessBMSCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus)
{
    if (rxBuf == NULL)
    {
        return;
    }
    uint8_t u8DockNo = DOCK_1;

    switch (canBus)
    {
    case CANBUS_0:
        u8DockNo = DOCK_1;
        break;
    case CANBUS_1:
        u8DockNo = DOCK_2;
        break;
    case CANBUS_2:
        u8DockNo = DOCK_3;
        break;
    default:
        return;
    }

    SESSION_SetBMSLastRxTime(u8DockNo, xTaskGetTickCount());
    uint32_t canId = READ_ID(rxBuf->id);

    if (!bIsValidBMSCanID(canId))
    {
        return;
    }

    ChargingMsgFrameInfo_t info = {0};

    (void)u8GetSetBMSData(u8DockNo, &info, GET_PARA);

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
        break;
    }

    (void)u8GetSetBMSData(u8DockNo, &info, SET_PARA);
}

/* ============================================================================
 *                              END OF FILE
 * ========================================================================== */