/******************************************************************************
 * @file    ChargingHandler.c
 * @brief   Charging Handler module implementation
 *
 * @details
 * Implements charging control logic including initialization, monitoring,
 * and state transitions.
 *
 * @author  Sarang Parmar
 * @date    16-Mar-2026
 * @version 1.0
 *
 ******************************************************************************/

/******************************************************************************
 * Includes
 ******************************************************************************/

#include "ChargingHandler.h"
#include "definitions.h"                // SYS function prototypes
#include "configuration.h"
#include "device.h"
#include "timers.h"
/******************************************************************************
 * Private Macros
 ******************************************************************************/
/******************************************************************************
 * CHARGING Task Configuration Macros
 ******************************************************************************/

/* Task Name */
#define CHARGING_TASK_NAME                "CHARGING_TASK"
/* Task Stack Size (in words) */
#define CHARGING_TASK_STACK_SIZE_WORDS    (1024U)
/* Task Priority */
#define CHARGING_TASK_PRIORITY            (tskIDLE_PRIORITY + 2U)
/* Task Queue Length (if needed later) */
#define CHARGING_TASK_QUEUE_LENGTH        (5U)
/* Task Delay */
#define CHARGING_TASK_DELAY_MS            (100U)
/* LEVDC Shutoff Delay */
#define LEVDC_SHUTOFF_DELAY_MS 2000U
#define CAN_COMMUNICATION_TX_INTERVAL_MS 100U
/* Number of PMs per group for TONHE modules */
#define LEVDC_POWER_RESOLUTION    50U   /* Power resolution in watts */
#define LEVDC_MAX_VOLTAGE         120U  /* As per Standard: 120 volt requirement */
#define LEVDC_MAX_CURRENT         100U  /* As per Standard: 100 Amp requirement */
#define LEVDC_RATED_DC_OP_POWER   ((LEVDC_MAX_VOLTAGE * LEVDC_MAX_CURRENT) / LEVDC_POWER_RESOLUTION)
/******************************************************************************
 * Private Type Definitions
 ******************************************************************************/

/******************************************************************************
 * Private Variables
 ******************************************************************************/
static TaskHandle_t xCHARGING_TASK = NULL;
ChargingMsgFrameInfo_t Charging_LiveInfo[MAX_DOCKS];
TimerHandle_t xTimerCanCommunicationTx = NULL;
/******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
static void CHARGING_TASK(void *pvParameters);

static void Charging_StateMachine(uint8_t u8DockNo);
void vChargingCanCommunicationTxTimerCallback(TimerHandle_t xTimer);
/******************************************************************************
 * Global Function Definitions
 ******************************************************************************/
void vProcessBMSCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus);
void vProcessPMCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus);
/******************************************************************************
 * @brief  Initialize CHARGING Task
 *
 * @return true  - Task created successfully
 * @return false - Task creation failed
//  ******************************************************************************/
bool CHARGING_Task_Initialize(void)
{
    bool bStatus = false;

    BaseType_t xTaskStatus;

    xTaskStatus = xTaskCreate(CHARGING_TASK,
                              CHARGING_TASK_NAME,
                              CHARGING_TASK_STACK_SIZE_WORDS,
                              NULL,
                              CHARGING_TASK_PRIORITY,
                              &xCHARGING_TASK);

    if (xTaskStatus == pdPASS)
    {
        SYS_CONSOLE_PRINT("CHARGING_TASK Created\r\n");
        vChargingCommunicationInit();
        bStatus = true;
    }
    else
    {
        LOG_E("CHARGING_TASK Creation Failed\r\n");
    }

    return bStatus;
}
uint8_t u8GetSetBMSData(uint8_t u8DockNo,
                                 ChargingMsgFrameInfo_t *psData,
                                 uint8_t u8Operation)
{
    uint8_t u8Ret = false;
    /* Input validation */
    if ((psData == NULL) || (u8DockNo >= MAX_DOCKS)) {
        return false;
    }
    taskENTER_CRITICAL();
    switch (u8Operation) {
        case SET_PARA: {
            memcpy((void *)&Charging_LiveInfo[u8DockNo],
                   (const void *)psData,
                   sizeof(ChargingMsgFrameInfo_t));
            u8Ret = true;
        }
        break;
        case GET_PARA: {
            memcpy((void *)psData,
                   (const void *)&Charging_LiveInfo[u8DockNo],
                   sizeof(ChargingMsgFrameInfo_t));
            u8Ret = true;
        }
        break;
        default: {
            /* Invalid operation */
            u8Ret = false;
        }
        break;
    }
    taskEXIT_CRITICAL();
    return u8Ret;
}

/* EV readiness check — preserves original logic but safer copy-by-value */
static bool bIsEvReadyForCharging(uint8_t u8DockNo)
{
    bool bRet = true;
    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    if (sChargingLiveInfo.LevdcRX_500ID_Info.u8EvChargingEnable)
    {
        SYS_CONSOLE_PRINT("MAIN: EV Charging Enabled for G%d\r\n", (int)u8DockNo);
        bRet = true;
    }
    else if (sChargingLiveInfo.LevdcRX_500ID_Info.u8EvConStatus ||
             sChargingLiveInfo.LevdcRX_500ID_Info.u8EvChargingPosition ||
             sChargingLiveInfo.LevdcRX_500ID_Info.u8WaitReqToEngTransfer ||
             sChargingLiveInfo.LevdcRX_500ID_Info.u8EnergyTransferError)
    {
        LOG_E("MAIN: Energy Transfer Error/EV Connection/Position/Wait Request Issue for G%d\r\n", (int)u8DockNo);
        bRet = false;
    }
    SYS_CONSOLE_PRINT("MAIN: bIsEvReadyForCharging(G%d) = %d\r\n", (int)u8DockNo, (int)bRet);
    return bRet;
}
/******************************************************************************
 * Private Function Definitions
 ******************************************************************************/
static void CHARGING_TASK(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        for (uint8_t u8DockNo = DOCK_1; u8DockNo < MAX_DOCKS; u8DockNo++)
        {
            Charging_StateMachine(u8DockNo);
        }

        vTaskDelay(pdMS_TO_TICKS(CHARGING_TASK_DELAY_MS));
    }
}

static void v17017_SendInitReq(uint8_t u8DockNo)
{
    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};

    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStopCtrl = EVSE_NOERROR;
    sChargingLiveInfo.LevdcTX_509ID_Info.u8ControlProtocolNum = 1;
    sChargingLiveInfo.LevdcTX_510ID_Info.u8EVSEVolatageControlOpt = VOLTAGE_CONTROL_ENABLED;

    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
    if (SESSION_GetAuthenticationCommand(u8DockNo) == 1U)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);
    }
}

static void v17017_AuthSuccess(uint8_t u8DockNo)
{
    SESSION_SetChargingState(u8DockNo, CH_STATE_PARAM_VALIDATE);
}

static void v17017_ValidateParameters(uint8_t u8DockNo)
{
    if (SESSION_GetBMSRxStatus(u8DockNo) == false)
    {
        SESSION_SetStartChargingComm(u8DockNo, false);
        return;
    }

    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStopCtrl = EVSE_NOERROR;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStatus = EVSE_CHARGING;

    sChargingLiveInfo.LevdcTX_509ID_Info.u8AvailDCOutputPower = 0xFF;
    sChargingLiveInfo.LevdcTX_509ID_Info.u16RemainChargeTime = 0xFFFF;
    sChargingLiveInfo.LevdcTX_508ID_Info.u16RatedOutputVol = (uint16_t)(LEVDC_MAX_VOLTAGE * FACTOR_10);
    sChargingLiveInfo.LevdcTX_508ID_Info.u16ConfDCvolLimit = (uint16_t)(LEVDC_MAX_VOLTAGE * FACTOR_10);
    sChargingLiveInfo.LevdcTX_508ID_Info.u16AvailOutputCur = (uint16_t)(LEVDC_MAX_CURRENT * FACTOR_10);

    sChargingLiveInfo.LevdcTX_508ID_Info.u8EVCompatible = EV_COMPATIBLE;
    SESSION_SetStartChargingComm(u8DockNo, true);
    SESSION_SetChargingState(u8DockNo, CH_STATE_CONNECTION_CONFIRMED);
    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
}

static void v17017_ConnectionConfirmed(uint8_t u8DockNo)
{
    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    sChargingLiveInfo.LevdcTX_508ID_Info.u8ConLatchStatus = GUN_LATCHED;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8ChargingSysError = EVSE_NOERROR;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseMalFunctionError = EVSE_NOERROR;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStatus = EVSE_STANDBY;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStopCtrl = EVSE_NOERROR;

    uint16_t u16OutputVoltage = (uint16_t)(sChargingLiveInfo.LevdcRX_500ID_Info.u16DcOutputVolTarget / FACTOR_10);
    uint16_t u16OutputCurrent = (uint16_t)(sChargingLiveInfo.LevdcRX_500ID_Info.u16ReqDcCurrent / FACTOR_10);
    uint16_t u16BatVoltMaxLimit = (uint16_t)(sChargingLiveInfo.LevdcRX_500ID_Info.u16DcOutputVoltLimit / FACTOR_10);
    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
    SYS_CONSOLE_PRINT("G%d ->Demand V: %d I: %d CAN: %d | Bat V Limit: %d\r\n",
          (int)u8DockNo,
          (int)u16OutputVoltage,
          (int)u16OutputCurrent,
          (int)SESSION_GetBMSRxStatus(u8DockNo),
          (int)u16BatVoltMaxLimit);

    if ((u16BatVoltMaxLimit <= LEVDC_MAX_VOLTAGE) && (u16OutputCurrent <= LEVDC_MAX_CURRENT))
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_INITIALIZE);
    }
    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
}

static void v17017_InitializeState(uint8_t u8DockNo)
{
    if (bIsEvReadyForCharging(u8DockNo))
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_PRECHARGE);
    }
    else
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_CONNECTION_CONFIRMED);
    }
}

static void v17017_PreChargingState(uint8_t u8DockNo)
{
    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    sChargingLiveInfo.LevdcTX_508ID_Info.u8EVSEReadyForCharge = EVSE_READY_FOR_CHARGE;

    sChargingLiveInfo.LevdcTX_509ID_Info.u8AvailDCOutputPower = (uint8_t)LEVDC_RATED_DC_OP_POWER;
    sChargingLiveInfo.LevdcTX_509ID_Info.u16RemainChargeTime = sChargingLiveInfo.LevdcRX_501ID_Info.u16EstimatedChargingTime;

    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
    SESSION_SetChargingState(u8DockNo, CH_STATE_CHARGING);

    SESSION_SetPMState(u8DockNo, RECTIFIER_ON);
}

static void v17017_StartCharging(uint8_t u8DockNo)
{

    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStatus = EVSE_CHARGING;

    sChargingLiveInfo.LevdcTX_509ID_Info.u16EVSEoutputVoltage = (uint16_t)(SESSION_GetPmOutputVoltage(u8DockNo) * FACTOR_10);
    sChargingLiveInfo.LevdcTX_509ID_Info.u16EVSEoutputCurrent = (uint16_t)(SESSION_GetPmOutputCurrent(u8DockNo) * FACTOR_10);
    vDO_Operation(GPIO_AC_RELAY_ON, u8DockNo);
    vDO_Operation(GPIO_DC_RELAY_ON, u8DockNo);

    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);

    SESSION_SetChargingState(u8DockNo, CH_STATE_CHARGING);
    if (SESSION_GetAuthenticationCommand(u8DockNo) == 0U)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);
    }
}

static void v17017_Shutdown(uint8_t u8DockNo)
{
    vDO_Operation(GPIO_AC_RELAY_OFF, u8DockNo);
    vDO_Operation(GPIO_DC_RELAY_OFF, u8DockNo);
    SESSION_SetPMState(u8DockNo, RECTIFIER_OFF);

    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStatus = EVSE_STANDBY;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8ConLatchStatus = GUN_UNLATCHED;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStopCtrl = EVSE_ERROR;
    sChargingLiveInfo.LevdcTX_508ID_Info.u16AvailOutputCur = 0U;
    sChargingLiveInfo.LevdcTX_508ID_Info.u16RatedOutputVol = 0U;

    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);

    vTaskDelay(LEVDC_SHUTOFF_DELAY_MS);
    SESSION_SetStartChargingComm(u8DockNo, false);
    (void)memset(&sChargingLiveInfo, 0, sizeof(sChargingLiveInfo));
    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);

    SESSION_SetChargingState(u8DockNo, CH_STATE_SESSION_COMPLETE);
}

static void v17017_SessionComplete(uint8_t u8DockNo)
{
    SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);
}

void Charging_StateMachine(uint8_t u8DockNo)
{
    switch (SESSION_GetChargingState(u8DockNo))
    {
    case CH_STATE_INIT:
    {
        v17017_SendInitReq(u8DockNo);
    }
    break;

    case CH_STATE_AUTH_SUCCESS:
    {
        v17017_AuthSuccess(u8DockNo);
    }
    break;

    case CH_STATE_PARAM_VALIDATE:
    {
        v17017_ValidateParameters(u8DockNo);
    }
    break;

    case CH_STATE_CONNECTION_CONFIRMED:
    {
        v17017_ConnectionConfirmed(u8DockNo);
    }
    break;

    case CH_STATE_INITIALIZE:
    {
        v17017_InitializeState(u8DockNo);
    }
    break;

    case CH_STATE_PRECHARGE:
    {
        v17017_PreChargingState(u8DockNo);
    }
    break;

    case CH_STATE_CHARGING:
    {
        v17017_StartCharging(u8DockNo);
    }
    break;

    case CH_STATE_SHUTDOWN:
    {
        v17017_Shutdown(u8DockNo);
    }
    break;

    case CH_STATE_SESSION_COMPLETE:
    {
        v17017_SessionComplete(u8DockNo);
    }
    break;

    default:
    {
        /* Safety fallback */
        SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);
    }
    break;
    }
}

/**
 * @brief Get TONHE CAN ID based on dock number and index of rectifier
 */
static inline uint32_t u32GetTonheCANID(uint8_t u8DockNo)
{
    return TONHE_MOD_BASE_ID + ((u8DockNo - 1) * 0x100U); // IDs are offset by 0x100 each
}

static void TonhePmExecuteCommand(uint8_t u8DockNo)
{
    tonhe_pm_Tx_t tonhe_liveinfo_tx;
    uint16_t u16PmVoltage = (uint16_t)(SESSION_GetPmSetVoltage(u8DockNo) * FACTOR_10);
    uint16_t u16PmCurrent = (uint16_t)(SESSION_GetPmSetCurrent(u8DockNo) * FACTOR_100);

    tonhe_liveinfo_tx.u16chargingVoltage = u16PmVoltage;
    tonhe_liveinfo_tx.u16chargingCurrent = u16PmCurrent;

    if (SESSION_GetPMState(u8DockNo))
    {
        tonhe_liveinfo_tx.u8ChargingMode = 1;
        tonhe_liveinfo_tx.u8ModuleStartStop = TONHE_MODULE_START;
    }
    else
    {
        memset(&tonhe_liveinfo_tx, 0, sizeof(tonhe_pm_Tx_t));
        tonhe_liveinfo_tx.u8ModuleStartStop = TONHE_MODULE_STOP;
    }

    CAN_TX_BUFFER canTxBuffer = {0};
    canTxBuffer.id = u32GetTonheCANID(u8DockNo);
    canTxBuffer.dlc = CAN_DATA_SIZE;
    canTxBuffer.xtd = EXT_CAN_MSG;
    memcpy(canTxBuffer.data, &tonhe_liveinfo_tx, CAN_DATA_SIZE);
    vSendCanTxMsgToQueue(&canTxBuffer, u8DockNo);
}

void vProcessBMSMessage(uint8_t u8DockNo)
{
    CAN_TX_BUFFER canTxBuffer = {0};
    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);
    /* Process BMS message and update Charging_LiveInfo as needed */
    /* For example, update SOC, voltage, current, etc. based on received BMS data */
    /* This is a placeholder for actual BMS message processing logic */
    if (SESSION_GetStartChargingComm(u8DockNo))
    {
        canTxBuffer.dlc = CAN_DATA_SIZE;
        canTxBuffer.xtd = STD_CAN_MSG;

        canTxBuffer.id = WRITE_ID(LEVDC_CAN_ID_EVSE_STATUS);
        memcpy(canTxBuffer.data, &sChargingLiveInfo.LevdcTX_508ID_Info, sizeof(LEVDC_Tx508_t));
        vSendCanTxMsgToQueue(&canTxBuffer, u8DockNo);

        canTxBuffer.id = WRITE_ID(LEVDC_CAN_ID_EVSE_OUTPUT_INFO);
        memcpy(canTxBuffer.data, &sChargingLiveInfo.LevdcTX_509ID_Info, sizeof(LEVDC_Tx509_t));
        vSendCanTxMsgToQueue(&canTxBuffer, u8DockNo);

        canTxBuffer.id = WRITE_ID(LEVDC_CAN_ID_EVSE_CAPABILITY);
        memcpy(canTxBuffer.data, &sChargingLiveInfo.LevdcTX_510ID_Info, sizeof(LEVDC_Tx510_t));
        vSendCanTxMsgToQueue(&canTxBuffer, u8DockNo);
    }
}
void vProcessPMMessage(uint8_t u8DockNo)
{
    TonhePmExecuteCommand(u8DockNo);
}
void vChargingCanCommunicationTxTimerCallback(TimerHandle_t xTimer)
{
    /* This callback can be used to trigger periodic CAN communication tasks related to charging */
    /* For example, it could set a flag or send a message to the CHARGING_TASK to perform certain actions */
    /* Implementation depends on specific requirements for CAN communication during charging */
    /* Placeholder for future CAN communication logic */
    (void)xTimer; // Unused parameter

    for (uint8_t u8DockNo = DOCK_1; u8DockNo < MAX_DOCKS; u8DockNo++)
    {
       vProcessBMSMessage(u8DockNo); // Process BMS messages for each dock, which may involve CAN communication
       vProcessPMMessage(u8DockNo);  // Process PM messages for each dock, which may involve CAN communication
    }
}

void vChargingCommunicationInit(void)
{
    if (xTimerCanCommunicationTx == NULL)
    {
        int32_t tickIdComm = 0;
        xTimerCanCommunicationTx = xTimerCreate("CanCommTxTimer",
                                               pdMS_TO_TICKS(CAN_COMMUNICATION_TX_INTERVAL_MS),
                                               pdTRUE,
                                               (void *)&tickIdComm,
                                               vChargingCanCommunicationTxTimerCallback);
        if (xTimerCanCommunicationTx != NULL)
        {
            xTimerStart(xTimerCanCommunicationTx, 0);
        }
        else
        {
            LOG_E("Failed to create CAN Communication Tx Timer\r\n");
        }
    }
}

void vProcessPMCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus)
{
    // Process PM message based on CAN bus and message content
    SYS_CONSOLE_PRINT("Processing PM CAN message from CAN%d\r\n", canBus);
    // Add specific processing logic here based on message ID and data
}
bool bIsValidCanID(uint32_t canId)
{
    bool bRet = false;
    if(canId == LEVDC_CAN_ID_EV_REQUEST ||
       canId == LEVDC_CAN_ID_EV_CHARGING_INFO ||
       canId == LEVDC_CAN_ID_EV_CONTROL_OPTION)
    {
        bRet = true;
    }
    return bRet;
}
void vProcessBMSCanMessage(CAN_RX_BUFFER *rxBuf, uint8_t canBus)
{
    if (rxBuf == NULL)
    {
        SYS_CONSOLE_PRINT("BMS: NULL RX buffer\r\n");
        return;
    }

    SYS_CONSOLE_PRINT("Processing BMS CAN message from CAN%d\r\n", canBus);

    uint32_t u32CanId = READ_ID(rxBuf->id);

    if (!bIsValidCanID(u32CanId))
    {
        return;
    }

    // Map CAN bus → Dock
    uint8_t u8DockNo = DOCK_1; // Default to DOCK_1, will be overridden by switch-case
    switch (canBus)
    {
        case CANBUS_0: u8DockNo = DOCK_1; break;
        case CANBUS_1: u8DockNo = DOCK_2; break;
        case CANBUS_2: u8DockNo = DOCK_3; break;
        default:
            SYS_CONSOLE_PRINT("Invalid CAN bus\r\n");
            return;
    }

    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};

    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    switch (u32CanId)
    {
        case LEVDC_CAN_ID_EV_REQUEST:
            if (rxBuf->dlc >= sizeof(LEVDC_Rx500_t))
            {
                memcpy(&sChargingLiveInfo.LevdcRX_500ID_Info,
                       rxBuf->data,
                       sizeof(LEVDC_Rx500_t));
            }
            break;

        case LEVDC_CAN_ID_EV_CHARGING_INFO:
            if (rxBuf->dlc >= sizeof(LEVDC_Rx501_t))
            {
                memcpy(&sChargingLiveInfo.LevdcRX_501ID_Info,
                       rxBuf->data,
                       sizeof(LEVDC_Rx501_t));
            }
            break;

        case LEVDC_CAN_ID_EV_CONTROL_OPTION:
            if (rxBuf->dlc >= sizeof(LEVDC_Rx502_t))
            {
                memcpy(&sChargingLiveInfo.LevdcRX_502ID_Info,
                       rxBuf->data,
                       sizeof(LEVDC_Rx502_t));
            }
            break;

        default:
            SYS_CONSOLE_PRINT("Unhandled CAN ID: 0x%X on CAN%d\r\n",
                              (unsigned int)u32CanId, canBus);
            break;
    }

    (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
}