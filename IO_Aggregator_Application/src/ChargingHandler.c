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
#include "ChargingCommunicationHandler.h"
// #include "timers.h"
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
#define CHARGING_TASK_DELAY_MS            (1000U)
/* LEVDC Shutoff Delay */
#define LEVDC_SHUTOFF_DELAY_MS 2000U
/* Fault Check Cooldown Period */
#define FAULT_CHECK_COOLDOWN_MS 10000U  /* Skip fault checking for 10000 ms after a fault occurs */
/* Number of PMs per group for TONHE modules */
#define LEVDC_POWER_RESOLUTION    50U   /* Power resolution in watts */
#define LEVDC_MAX_VOLTAGE         120U  /* As per Standard: 120 volt requirement */
#define LEVDC_MAX_CURRENT         100U  /* As per Standard: 100 Amp requirement */
#define LEVDC_RATED_DC_OP_POWER   ((LEVDC_MAX_VOLTAGE * LEVDC_MAX_CURRENT) / LEVDC_POWER_RESOLUTION)

/* Define these as per your hardware specs */
#define CHARGER_TYPE_AC                 0x01
#define OUTPUT_ENABLE                   0x01
#define OUTPUT_DISABLE                  0x00
#define FAN_ENABLE                      0x01
#define FAN_DISABLE                     0x00
#define AC_INPUT_VALID                  0x01
#define ERROR_NONE                      0x00
#define DERATING_NONE                   0x00
#define DERATING_ACTIVE                 0x01
#define PRE_CHARGE_VOLTAGE_THRESHOLD    60u
#define TEMP_DERATING_THRESHOLD         45u
#define SOC_CHARGE_COMPLETE             100u
/******************************************************************************
 * Private Type Definitions
 ******************************************************************************/

/******************************************************************************
 * Private Variables
 ******************************************************************************/
static TaskHandle_t xCHARGING_TASK = NULL;
ChargingMsgFrameInfo_t Charging_LiveInfo[MAX_DOCKS];
/******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
static void CHARGING_TASK(void *pvParameters);

static void Charging_StateMachine(uint8_t u8DockNo);

/* Charging Process Handler */
static void vChargingProcessHandler(uint8_t u8DockNo);
/* Vehicle & Power Module */
static void vUpdateVehiclePMInfo(uint8_t u8DockNo);
static void vEnergyTimeCalculation(uint8_t u8DockNo);
static void updateSystemState(uint8_t u8DockNo);

/* LED */
void vSetLedState(uint8_t u8DockNo, uint8_t ledColor, uint8_t ledState);

/* Stop & Fault Handling */
static bool bCheckStopCondition(uint8_t u8DockNo);
static bool bCheckFaultCondition(uint8_t u8DockNo);

/* Individual Fault Checks */
static bool bCheckEStopFault(uint8_t u8DockNo);
static bool bCheckBMSFault(uint8_t u8DockNo);
static bool bCheckPMFault(uint8_t u8DockNo);
static bool bCheckBMSStatus(uint8_t u8DockNo);
static bool bCheckPMStatus(uint8_t u8DockNo);
static bool bCheckZeroCurrentFault(uint8_t u8DockNo);
static bool bCheckPrechargeFailure(uint8_t u8DockNo);

/* Fault Status Helpers */
static bool bGetBMSFaultStatus(uint8_t u8DockNo);
static bool bGetPMFaultStatus(uint8_t u8DockNo);

/*==============================================================================
 * TVS Charging State Function Prototypes
 *============================================================================*/
static void vTVS_SendInitReq          (uint8_t u8DockNo);
static void vTVS_AuthSuccess          (uint8_t u8DockNo);
static void vTVS_ValidateParameters   (uint8_t u8DockNo);
static void vTVS_ConnectionConfirmed  (uint8_t u8DockNo);
static void vTVS_InitializeState      (uint8_t u8DockNo);
static void vTVS_PreChargingState     (uint8_t u8DockNo);
static void vTVS_StartCharging        (uint8_t u8DockNo);
static void vTVS_Shutdown             (uint8_t u8DockNo);
static void vTVS_SessionComplete      (uint8_t u8DockNo);
static void vTVS_SessionError         (uint8_t u8DockNo);


/* Utility Functions */
static const char* CH_GetStateString(CH_State_e state);
static void vPrintSystemData(uint8_t u8DockNo);
static void vPrintStateAndDeviceInfo(uint8_t dockNo);
static void vPrintChargingInfo(uint8_t u8DockNo);
/******************************************************************************
 * Global Function Definitions
 ******************************************************************************/

/******************************************************************************
 * @brief  Initialize CHARGING Task
 *
 * @return true  - Task created successfully
 * @return false - Task creation failed
//  ******************************************************************************/
bool ChargingTask_Init(void)
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
        SYS_CONSOLE_PRINT("CHARGING_TASK Creation Failed\r\n");
    }
    return bStatus;
}
bool bGetSetLevdcBMSData(uint8_t u8DockNo,
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

/*==============================================================================
 * TVS Unified GET/SET - Full Frame
 * @param u8DockNo   : Dock index (0 to MAX_DOCKS-1)
 * @param psData     : Pointer to TVS_MsgFrameInfo_t
 * @param u8Operation: SET_PARA or GET_PARA
 * @return           : true on success, false on failure
 *============================================================================*/
bool bGetSetTVSBMSData(uint8_t u8DockNo,
                    TVS_MsgFrameInfo_t *psData,
                    uint8_t u8Operation)
{
    bool bRet = false;

    /* Input validation */
    if ((psData == NULL) || (u8DockNo >= MAX_DOCKS)) {
        return false;
    }

    taskENTER_CRITICAL();
    switch (u8Operation)
    {
        case SET_PARA: {
            memcpy((void *)&TVS_LiveInfo[u8DockNo],
                   (const void *)psData,
                   sizeof(TVS_MsgFrameInfo_t));
            bRet = true;
        }
        break;

        case GET_PARA: {
            memcpy((void *)psData,
                   (const void *)&TVS_LiveInfo[u8DockNo],
                   sizeof(TVS_MsgFrameInfo_t));
            bRet = true;
        }
        break;

        default: {
            /* Invalid operation */
            bRet = false;
        }
        break;
    }
    taskEXIT_CRITICAL();
    return bRet;
}

/* EV readiness check — preserves original logic but safer copy-by-value */
static bool bIsEvReadyForCharging(uint8_t u8DockNo)
{
    bool bRet = true;
    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

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
        SYS_CONSOLE_PRINT("MAIN: Energy Transfer Error/EV Connection/Position/Wait Request Issue for G%d\r\n", (int)u8DockNo);
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
    SYS_CONSOLE_PRINT("In Function: %s\r\n", __FUNCTION__);
    for (;;)
    {
        for (uint8_t u8DockNo = DOCK_1; u8DockNo < MAX_DOCKS; u8DockNo++)
        {
            vChargingProcessHandler(u8DockNo);
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

    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
    if (SESSION_GetAuthenticationCommand(u8DockNo) == 1U)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_AUTH_SUCCESS);
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
    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

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
    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
}

static void v17017_ConnectionConfirmed(uint8_t u8DockNo)
{
    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    sChargingLiveInfo.LevdcTX_508ID_Info.u8ConLatchStatus = GUN_LATCHED;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8ChargingSysError = EVSE_NOERROR;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseMalFunctionError = EVSE_NOERROR;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStatus = EVSE_STANDBY;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStopCtrl = EVSE_NOERROR;

    uint16_t u16OutputVoltage = (uint16_t)(sChargingLiveInfo.LevdcRX_500ID_Info.u16DcOutputVolTarget / FACTOR_10);
    uint16_t u16OutputCurrent = (uint16_t)(sChargingLiveInfo.LevdcRX_500ID_Info.u16ReqDcCurrent / FACTOR_10);
    uint16_t u16BatVoltMaxLimit = (uint16_t)(sChargingLiveInfo.LevdcRX_500ID_Info.u16DcOutputVoltLimit / FACTOR_10);
    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
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
    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
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
    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    sChargingLiveInfo.LevdcTX_508ID_Info.u8EVSEReadyForCharge = EVSE_READY_FOR_CHARGE;

    sChargingLiveInfo.LevdcTX_509ID_Info.u8AvailDCOutputPower = (uint8_t)LEVDC_RATED_DC_OP_POWER;
    sChargingLiveInfo.LevdcTX_509ID_Info.u16RemainChargeTime = sChargingLiveInfo.LevdcRX_501ID_Info.u16EstimatedChargingTime;

    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
    SESSION_SetChargingState(u8DockNo, CH_STATE_CHARGING);

    SESSION_SetPMState(u8DockNo, RECTIFIER_ON);
}

static void v17017_StartCharging(uint8_t u8DockNo)
{

    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStatus = EVSE_CHARGING;

    sChargingLiveInfo.LevdcTX_509ID_Info.u16EVSEoutputVoltage = (uint16_t)(SESSION_GetPmOutputVoltage(u8DockNo) * FACTOR_10);
    sChargingLiveInfo.LevdcTX_509ID_Info.u16EVSEoutputCurrent = (uint16_t)(SESSION_GetPmOutputCurrent(u8DockNo) * FACTOR_10);
    bGPIO_Operation(DO_AC_RELAY_ON, u8DockNo);
    bGPIO_Operation(DO_DC_RELAY_ON, u8DockNo);

    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);

    SESSION_SetChargingState(u8DockNo, CH_STATE_CHARGING);
}

static void v17017_Shutdown(uint8_t u8DockNo)
{
    bGPIO_Operation(DO_AC_RELAY_OFF, u8DockNo);
    bGPIO_Operation(DO_DC_RELAY_OFF, u8DockNo);
    SESSION_SetPMState(u8DockNo, RECTIFIER_OFF);

    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStatus = EVSE_STANDBY;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8ConLatchStatus = GUN_UNLATCHED;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStopCtrl = EVSE_ERROR;
    sChargingLiveInfo.LevdcTX_508ID_Info.u16AvailOutputCur = 0U;
    sChargingLiveInfo.LevdcTX_508ID_Info.u16RatedOutputVol = 0U;

    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);

    vTaskDelay(LEVDC_SHUTOFF_DELAY_MS);
    SESSION_SetStartChargingComm(u8DockNo, false);
    (void)memset(&sChargingLiveInfo, 0, sizeof(sChargingLiveInfo));
    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);

    SESSION_SetChargingState(u8DockNo, CH_STATE_SESSION_COMPLETE);
}

static void v17017_SessionComplete(uint8_t u8DockNo)
{
    if (SESSION_GetSystemFaultBitmap(u8DockNo) != SYSTEM_FAULT_NONE)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_ERROR);
    }
    else
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);
    }
}
static void v17017_SessionError(uint8_t u8DockNo)
{
    if (SESSION_GetSystemFaultBitmap(u8DockNo) == SYSTEM_FAULT_NONE)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);
    }
}

/*==============================================================================
 * CH_STATE_INIT
 * - First state on session start
 * - Send initial handshake request to BMS/EVSE
 *============================================================================*/
static void vTVS_SendInitReq(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};

    /* Read current frame */
    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    /* TODO: Populate init request fields */
    sTVSFrame.TVS_Tx90_ChargerInfo.u8ChargerType = CHARGER_TYPE_AC;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Output      = OUTPUT_DISABLE;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8ErrorState  = ERROR_NONE;

    /* Write back */
    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);

    if (SESSION_GetAuthenticationCommand(u8DockNo) == 1U)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_AUTH_SUCCESS);
    }
}

/*==============================================================================
 * CH_STATE_AUTH_SUCCESS
 * - Authentication with BMS confirmed
 * - Validate charger compatibility
 *============================================================================*/
static void vTVS_AuthSuccess(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    /* TODO: Set auth success flags / compatibility check */
    sTVSFrame.TVS_Tx90_ChargerInfo.u8ACInput = AC_INPUT_VALID;

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);

    SESSION_SetChargingState(u8DockNo, CH_STATE_PARAM_VALIDATE);
}

/*==============================================================================
 * CH_STATE_PARAM_VALIDATE
 * - Validate BMS charge profile parameters
 * - Check voltage, current limits before proceeding
 *============================================================================*/
static void vTVS_ValidateParameters(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    /* TODO: Validate BMS profile parameters */
    uint8_t u8MaxVol = sTVSFrame.TVS_Rx101_BMSProfile.u8MaxChargeVoltage;
    uint8_t u8MaxCur = sTVSFrame.TVS_Rx101_BMSProfile.u8MaxChargeCurrent;

    if ((u8MaxVol == 0u) || (u8MaxCur == 0u))
    {
        /* Invalid parameters - go to error */
        SESSION_SetChargingState(u8DockNo, CH_STATE_ERROR);
        return;
    }

    /* Parameters valid - update charge profile */
    sTVSFrame.TVS_Tx91_ChargeProfile.u8ChargingVoltage = u8MaxVol;
    sTVSFrame.TVS_Tx91_ChargeProfile.u8ChargingCurrent = u8MaxCur;

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);

    SESSION_SetChargingState(u8DockNo, CH_STATE_CONNECTION_CONFIRMED);
}

/*==============================================================================
 * CH_STATE_CONNECTION_CONFIRMED
 * - Physical connector latch confirmed
 * - Ready to proceed to initialization
 *============================================================================*/
static void vTVS_ConnectionConfirmed(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    /* TODO: Confirm connector latch status from BMS */
    uint8_t u8BMSError = sTVSFrame.TVS_Rx100_BMSStatus.u8ErrorState;

    if (u8BMSError != ERROR_NONE)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_ERROR);
        return;
    }

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);

    SESSION_SetChargingState(u8DockNo, CH_STATE_INITIALIZE);
}

/*==============================================================================
 * CH_STATE_INITIALIZE
 * - Initialize charger hardware and output stage
 * - Apply charge profile from BMS
 *============================================================================*/
static void vTVS_InitializeState(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    /* TODO: Initialize charger output parameters */
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Output    = OUTPUT_ENABLE;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Fan       = FAN_ENABLE;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Derating  = DERATING_NONE;

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);

    SESSION_SetChargingState(u8DockNo, CH_STATE_PRECHARGE);
}

/*==============================================================================
 * CH_STATE_PRECHARGE
 * - Apply pre-charge current before main charging
 * - Monitor battery voltage rise
 *============================================================================*/
static void vTVS_PreChargingState(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    /* TODO: Apply pre-charge current */
    uint8_t u8PreChargeCur = sTVSFrame.TVS_Rx101_BMSProfile.u8PreChargeCurrent;
    uint8_t u8BattVoltage  = sTVSFrame.TVS_Rx100_BMSStatus.u8Voltage;

    sTVSFrame.TVS_Tx91_ChargeProfile.u8ChargingCurrent = u8PreChargeCur;

    /* TODO: Check pre-charge completion threshold */
    if (u8BattVoltage >= PRE_CHARGE_VOLTAGE_THRESHOLD)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_CHARGING);
    }

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);
}

/*==============================================================================
 * CH_STATE_CHARGING
 * - Main charging loop
 * - Monitor SOC, temperature, errors continuously
 *============================================================================*/
static void vTVS_StartCharging(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    uint8_t u8SOC         = sTVSFrame.TVS_Rx100_BMSStatus.u8SOC;
    uint8_t u8Temperature = sTVSFrame.TVS_Rx100_BMSStatus.u8Temperature;
    uint8_t u8ErrorState  = sTVSFrame.TVS_Rx100_BMSStatus.u8ErrorState;

    /* TODO: Error check during charging */
    if (u8ErrorState != ERROR_NONE)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_ERROR);
        return;
    }

    /* TODO: Apply derating on high temperature */
    if (u8Temperature >= TEMP_DERATING_THRESHOLD)
    {
        sTVSFrame.TVS_Tx90_ChargerInfo.u8Derating = DERATING_ACTIVE;
    }

    /* TODO: Check charge complete condition */
    if (u8SOC >= SOC_CHARGE_COMPLETE)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_SHUTDOWN);
    }

    /* Apply max charge current and voltage */
    sTVSFrame.TVS_Tx91_ChargeProfile.u8ChargingCurrent = sTVSFrame.TVS_Rx101_BMSProfile.u8MaxChargeCurrent;
    sTVSFrame.TVS_Tx91_ChargeProfile.u8ChargingVoltage = sTVSFrame.TVS_Rx101_BMSProfile.u8MaxChargeVoltage;

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);
}

/*==============================================================================
 * CH_STATE_SHUTDOWN
 * - Graceful shutdown of charger output
 * - Ramp down current before disconnecting
 *============================================================================*/
static void vTVS_Shutdown(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    /* TODO: Ramp down output and disable charger */
    sTVSFrame.TVS_Tx91_ChargeProfile.u8ChargingCurrent = 0u;
    sTVSFrame.TVS_Tx91_ChargeProfile.u8ChargingVoltage = 0u;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Output            = OUTPUT_DISABLE;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Fan               = FAN_DISABLE;

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);

    SESSION_SetChargingState(u8DockNo, CH_STATE_SESSION_COMPLETE);
}

/*==============================================================================
 * CH_STATE_SESSION_COMPLETE
 * - Session ended successfully
 * - Reset live data and return to INIT
 *============================================================================*/
static void vTVS_SessionComplete(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};

    /* Clear all live data for this dock */
    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);

    /* TODO: Log session complete event */

    SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);
}

/*==============================================================================
 * CH_STATE_ERROR
 * - Handle fault conditions
 * - Disable output, log error, reset state
 *============================================================================*/
static void vTVS_SessionError(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    /* TODO: Log error state and fault source */
    uint8_t u8ErrorState = sTVSFrame.TVS_Rx100_BMSStatus.u8ErrorState;
    (void)u8ErrorState; /* TODO: Pass to fault logger */

    /* Safe state - disable all outputs */
    sTVSFrame.TVS_Tx91_ChargeProfile.u8ChargingCurrent = 0u;
    sTVSFrame.TVS_Tx91_ChargeProfile.u8ChargingVoltage = 0u;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Output            = OUTPUT_DISABLE;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Fan               = FAN_DISABLE;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8ErrorState        = u8ErrorState;

    bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);

    /* TODO: Add error recovery delay or retry logic before reset */
    SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);
}

static void Charging_StateMachine(uint8_t u8DockNo)
{
#if 0
    switch (SESSION_GetChargingState(u8DockNo))
    {
    case CH_STATE_INIT: v17017_SendInitReq(u8DockNo); break;
    case CH_STATE_AUTH_SUCCESS: v17017_AuthSuccess(u8DockNo); break;
    case CH_STATE_PARAM_VALIDATE: v17017_ValidateParameters(u8DockNo); break;
    case CH_STATE_CONNECTION_CONFIRMED: v17017_ConnectionConfirmed(u8DockNo); break;
    case CH_STATE_INITIALIZE: v17017_InitializeState(u8DockNo); break;
    case CH_STATE_PRECHARGE: v17017_PreChargingState(u8DockNo); break;
    case CH_STATE_CHARGING: v17017_StartCharging(u8DockNo); break;
    case CH_STATE_SHUTDOWN: v17017_Shutdown(u8DockNo); break;
    case CH_STATE_SESSION_COMPLETE: v17017_SessionComplete(u8DockNo); break;
    case CH_STATE_ERROR: v17017_SessionError(u8DockNo); break;
    default: SESSION_SetChargingState(u8DockNo, CH_STATE_INIT); break;
    }
#endif
    switch (SESSION_GetChargingState(u8DockNo))
        {
        case CH_STATE_INIT: vTVS_SendInitReq(u8DockNo); break;
        case CH_STATE_AUTH_SUCCESS: vTVS_AuthSuccess(u8DockNo); break;
        case CH_STATE_PARAM_VALIDATE: vTVS_ValidateParameters(u8DockNo); break;
        case CH_STATE_CONNECTION_CONFIRMED: vTVS_ConnectionConfirmed(u8DockNo); break;
        case CH_STATE_INITIALIZE: vTVS_InitializeState(u8DockNo); break;
        case CH_STATE_PRECHARGE: vTVS_PreChargingState(u8DockNo); break;
        case CH_STATE_CHARGING: vTVS_StartCharging(u8DockNo); break;
        case CH_STATE_SHUTDOWN: vTVS_Shutdown(u8DockNo); break;
        case CH_STATE_SESSION_COMPLETE: vTVS_SessionComplete(u8DockNo); break;
        case CH_STATE_ERROR: vTVS_SessionError(u8DockNo); break;
        default: SESSION_SetChargingState(u8DockNo, CH_STATE_INIT); break;
        }

}

/*===============================================================
 * 
 ================================================================*/
static void vChargingProcessHandler(uint8_t u8DockNo)
{
    vPrintSystemData(u8DockNo);
    updateSystemState(u8DockNo);
    vUpdateVehiclePMInfo(u8DockNo);
    vEnergyTimeCalculation(u8DockNo);
    CH_State_e eState = SESSION_GetChargingState(u8DockNo);
    bool bStop = bCheckStopCondition(u8DockNo);
    if (bStop == true)
    {
        if ((eState == CH_STATE_CHARGING) || (eState == CH_STATE_PRECHARGE))
        {
            SYS_CONSOLE_PRINT("[GUN %d] Fault Stop, fault Code: %d .\r\n", u8DockNo, SESSION_GetSystemFaultBitmap(u8DockNo));
            SESSION_SetChargingState(u8DockNo, CH_STATE_SHUTDOWN);
            SESSION_SetAuthenticationCommand(u8DockNo, 0U);
        }
        else
        {
            SESSION_SetChargingState(u8DockNo, CH_STATE_ERROR);
        }
    }
}
/* -----------------------
 * Vehicle & Power Module Update
 * ----------------------- */
static void vUpdateVehiclePMInfo(uint8_t u8DockNo)
{
    float fDemandVoltage = 0U;
    float fDemandCurrent = 0U;
    uint16_t u16EstChrgTime = 0U;
    uint16_t u16CurrentSoc = 0U;
    /* Check if charging is active */
    CH_State_e eLiveStage = SESSION_GetChargingState(u8DockNo);
    if (eLiveStage != CH_STATE_CHARGING)
        return;

    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    fDemandVoltage = (sChargingLiveInfo.LevdcRX_500ID_Info.u16DcOutputVolTarget * FACTOR_0_1);
    fDemandCurrent = (sChargingLiveInfo.LevdcRX_500ID_Info.u16ReqDcCurrent * FACTOR_0_1);
    u16EstChrgTime = sChargingLiveInfo.LevdcRX_501ID_Info.u16EstimatedChargingTime;
    u16CurrentSoc = sChargingLiveInfo.LevdcRX_501ID_Info.u8ChargingRate;

    SESSION_SetBMSDemandVoltage(u8DockNo, fDemandVoltage);
    SESSION_SetBMSDemandCurrent(u8DockNo, fDemandCurrent);

    /* Common updates */
    SESSION_SetEstimatedChargingTime(u8DockNo, u16EstChrgTime);
    SESSION_SetCurrentSoc(u8DockNo, u16CurrentSoc);

    /* Initialize initial SOC if not set */
    if (SESSION_GetInitialSoc(u8DockNo) == 0U && u16CurrentSoc != 0)
    {
        SESSION_SetInitialSoc(u8DockNo, u16CurrentSoc);
        SYS_CONSOLE_PRINT("GunNo: %d Initial SOC: %d\r\n", u8DockNo, u16CurrentSoc);
    }
}

/**
 * @brief Perform manual energy and time calculation for charging session
 *
 * @details
 * This function calculates energy using V * I over time (1-second interval).
 * It updates session energy, total energy, and charging time.
 *
 * @param u8DockNo Gun index
 */
static void vEnergyTimeCalculation(uint8_t u8DockNo)
{
    static uint32_t u32PreviousTick[MAX_DOCKS] = {0};
    const float fMinValidVoltage = 5.0f;
    const float fMinValidCurrent = 1.0f;
    
    CH_State_e eLiveStage = SESSION_GetChargingState(u8DockNo);

    /* ======================= CHARGING ACTIVE ======================= */
    if (eLiveStage == CH_STATE_CHARGING)
    {
        uint32_t u32CurrentTick = xTaskGetTickCount();

        /* Execute every 1 second */
        if ((u32PreviousTick[u8DockNo] + pdMS_TO_TICKS(1000U)) <= u32CurrentTick)
        {
            u32PreviousTick[u8DockNo] = u32CurrentTick;

            float fVoltage = SESSION_GetPmOutputVoltage(u8DockNo);
            float fCurrent = SESSION_GetPmOutputCurrent(u8DockNo);
            /* Power (W) = V * I */
            float fPowerWatt = fVoltage * fCurrent;
            SESSION_SetOutputPower(u8DockNo, fPowerWatt);
            if ((fVoltage > fMinValidVoltage) && (fCurrent > fMinValidCurrent))
            {
                /* Energy for 1 second: Wh = W / 3600 */
                float fDeltaWh = fPowerWatt / 3600.0f;
                /* Convert Wh → kWh */
                float fDeltaKwh = fDeltaWh / FACTOR_1000_F;
                float fTotalEnergy = SESSION_GetEnergyDelivered(u8DockNo);
                float fNewEnergy = fTotalEnergy + fDeltaKwh;

                SESSION_SetEnergyDelivered(u8DockNo, fNewEnergy);
            }
            else
            {
                SYS_CONSOLE_PRINT("[Gun %d] Invalid V/I (V=%d, I=%d)\r\n", u8DockNo, fVoltage, fCurrent);
            }
        }
    }

    /* ======================= SESSION START ======================= */
    else if (eLiveStage == CH_STATE_PRECHARGE)
    {
        float fStartEnergy = 0.0f;
        SESSION_SetEnergyDelivered(u8DockNo, fStartEnergy);
        u32PreviousTick[u8DockNo] = 0U;
        SYS_CONSOLE_PRINT("[Gun %d] Session started", u8DockNo);
    }

    /* ======================= SESSION END ======================= */
    else if (eLiveStage == CH_STATE_SESSION_COMPLETE)
    {
        float fFinalEnergy = SESSION_GetEnergyDelivered(u8DockNo);
        SYS_CONSOLE_PRINT("[Gun %d] Session ended. Energy = %.3f kWh\r\n", u8DockNo, fFinalEnergy);
        u32PreviousTick[u8DockNo] = 0U;
    }
    else
    {
        // Do nothing 
    }
}
/**
 * @brief Update system state based on charging state machine
 *
 * @param u8DockNo Dock/Gun index
 */
static void updateSystemState(uint8_t u8DockNo)
{
    static uint8_t bSessionActive[MAX_DOCKS] = {0};
    static uint8_t u8PrvLiveState[MAX_DOCKS] = {CH_STATE_INIT};
    CH_State_e eState = SESSION_GetChargingState(u8DockNo);

    if (eState != u8PrvLiveState[u8DockNo])
    {
        switch (eState)
        {
        /* ================= INIT ================= */
        case CH_STATE_INIT:
            vSetLedState(u8DockNo, LED_BLUE, LED_STATE_BLINK);
            SESSION_ResetBMSData(u8DockNo);
            break;

        /* ================= PRE-CHARGING FLOW ================= */
        case CH_STATE_AUTH_SUCCESS:
        case CH_STATE_PARAM_VALIDATE:
        case CH_STATE_CONNECTION_CONFIRMED:
        case CH_STATE_INITIALIZE:
            vSetLedState(u8DockNo, LED_BLUE, LED_STATE_STEADY);
            bSessionActive[u8DockNo] = 1U;
            break;

        case CH_STATE_PRECHARGE:
            vSetLedState(u8DockNo, LED_BLUE, LED_STATE_BLINK);
            bSessionActive[u8DockNo] = 1U;
            break;

        /* ================= ACTIVE CHARGING ================= */
        case CH_STATE_CHARGING:
            vSetLedState(u8DockNo, LED_GREEN, LED_STATE_STEADY);
            bSessionActive[u8DockNo] = 1U;
            break;

        /* ================= SHUTDOWN / COMPLETE ================= */
        case CH_STATE_SHUTDOWN:
        case CH_STATE_SESSION_COMPLETE:
            vSetLedState(u8DockNo, LED_GREEN, LED_STATE_BLINK);
            break;

        /* ================= ERROR ================= */
        case CH_STATE_ERROR:
            vSetLedState(u8DockNo, LED_RED, LED_STATE_STEADY);
            break;

        /* ================= DEFAULT ================= */
        default:
            SYS_CONSOLE_PRINT("Unknown state for Gun %d: %d\r\n", u8DockNo, eState);
            vSetLedState(u8DockNo, LED_RED, LED_STATE_BLINK);
            break;
        }

        /* ================= SESSION CLEANUP ================= */
        if ((eState != CH_STATE_AUTH_SUCCESS) &&
            (eState != CH_STATE_PARAM_VALIDATE) &&
            (eState != CH_STATE_CONNECTION_CONFIRMED) &&
            (eState != CH_STATE_INITIALIZE) &&
            (eState != CH_STATE_PRECHARGE) &&
            (eState != CH_STATE_CHARGING) &&
            bSessionActive[u8DockNo])
        {
            SYS_CONSOLE_PRINT("Session cleanup for Gun %d\r\n", u8DockNo);

            bSessionActive[u8DockNo] = 0U;
            SESSION_ResetSession(u8DockNo);
        }
    }
}


void vSetLedState(uint8_t u8DockNo, uint8_t ledColor, uint8_t ledState)
{
    switch (ledColor)
    {
    case LED_RED:
        break;
    case LED_GREEN:
        break;
    case LED_BLUE:
        break;
    default:
        // Invalid LED color
        break;
    }
}

static bool bCheckStopCondition(uint8_t u8DockNo)
{
    static uint32_t u32FaultCheckCooldownTick[MAX_DOCKS] = {0};
    CH_State_e eState = SESSION_GetChargingState(u8DockNo);
    // 1. User stop
    if ((eState == CH_STATE_CHARGING) && (SESSION_GetAuthenticationCommand(u8DockNo) == 0U))
    {
        SYS_CONSOLE_PRINT("[GUN %d] User Stop Command Received\r\n", u8DockNo);
        SESSION_SetSessionEndReason(u8DockNo, STOP_REASON_MCU_REQUEST);
        return true;
    }

    // 2. Fault Condition
    if (xTaskGetTickCount() >= u32FaultCheckCooldownTick[u8DockNo])
    {
        if (bCheckFaultCondition(u8DockNo) == true)
        {
            u32FaultCheckCooldownTick[u8DockNo] = xTaskGetTickCount() + pdMS_TO_TICKS(FAULT_CHECK_COOLDOWN_MS);
            SESSION_SetSessionEndReason(u8DockNo, STOP_REASON_FAULT);
            return true;
        }
    }
    return false;
}
static bool bCheckFaultCondition(uint8_t u8DockNo)
{
    uint32_t u32SystemFault = 0;

    /* -------- Critical Faults -------- */

    if (bCheckEStopFault(u8DockNo))
    {
        SET_BIT(u32SystemFault, SYSTEM_FAULT_ESTOP_TRIGGERED);
    }

    if (bCheckBMSFault(u8DockNo))
    {
        SET_BIT(u32SystemFault, SYSTEM_FAULT_BMS_ERROR);
    }

    if (bCheckPMFault(u8DockNo))
    {
        SET_BIT(u32SystemFault, SYSTEM_FAULT_PM_ERROR);
    }

    if (bCheckBMSStatus(u8DockNo))
    {
        SET_BIT(u32SystemFault, SYSTEM_FAULT_BMS_COMMUNICATION_FAILURE);
    }

    if (bCheckPMStatus(u8DockNo))
    {
        SET_BIT(u32SystemFault, SYSTEM_FAULT_PM_COMMUNICATION_FAILURE);
    }

    if (bCheckZeroCurrentFault(u8DockNo))
    {
        SET_BIT(u32SystemFault, SYSTEM_FAULT_PM_ZERO_CURRENT);
    }

    if (bCheckPrechargeFailure(u8DockNo))
    {
        SET_BIT(u32SystemFault, SYSTEM_FAULT_PRECHARGE_FAILURE);
    }

    /* Store complete bitmap */
    SESSION_SetSystemFaultBitmap(u8DockNo, u32SystemFault);

    /* Return TRUE if any critical fault (0–15 bits set) */
    if (u32SystemFault & 0x0000FFFF)
    {
        return true;
    }

    return false;
}

static bool bCheckEStopFault(uint8_t u8DockNo)
{
    if (bGPIO_Operation(DI_E_STOP_STATUS, u8DockNo) == true)
    {
        return true;
    }
    return false;
}



static bool bCheckBMSFault(uint8_t u8DockNo)
{
    static uint32_t u32BMSFaultTicks[MAX_DOCKS] = {0};
    const uint32_t u32FaultThresholdMs = 2000U; // 2 second threshold
    uint32_t u32CurrentTick = xTaskGetTickCount();
    CH_State_e eState = SESSION_GetChargingState(u8DockNo);
    bool bBMSFault = bGetBMSFaultStatus(u8DockNo);
    if ((bBMSFault == true) && (eState == CH_STATE_CHARGING))
    {
        if (u32BMSFaultTicks[u8DockNo] == 0U)
        {
            u32BMSFaultTicks[u8DockNo] = u32CurrentTick;
        }

        uint32_t u32ElapsedMs = (u32CurrentTick - u32BMSFaultTicks[u8DockNo]) * portTICK_PERIOD_MS;
        if (u32ElapsedMs >= u32FaultThresholdMs)
        {
            SYS_CONSOLE_PRINT("BMS fault triggered for Gun %d after %lu ms\r\n", u8DockNo, u32ElapsedMs);
            u32BMSFaultTicks[u8DockNo] = 0U;
            return true;
        }
    }
    else
    {
        u32BMSFaultTicks[u8DockNo] = 0U;
    }
    return false;
}

/* Dependent function for BMS fault status */
static bool bGetBMSFaultStatus(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    uint8_t u8BMSError    = sTVSFrame.TVS_Rx100_BMSStatus.u8ErrorState;
     /* --- Error / stop check --- */
    if (u8BMSError == TVS_BMS_ERR_STOP_CHARGING)
    {
        SYS_CONSOLE_PRINT("G%d -> BMS Stop Charging request\r\n", (int)u8DockNo);
        return true;
    }
#if 0
    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    uint32_t u32FaultCode = 0;

    if (bGetSetLevdcBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA) == false)
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] BMS data read failed\r\n", u8DockNo);
        return true;
    }

    /* -------- Critical Faults -------- */

    if (sChargingLiveInfo.LevdcRX_500ID_Info.u8EnergyTransferError)
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] EnergyTransferError\r\n", u8DockNo);
        SET_BIT(u32FaultCode, BMS_FAULT_ENERGY_TRANSFER_ERROR);
    }

    // if (sChargingLiveInfo.LevdcRX_500ID_Info.u8EvConStatus)
    // {
    //     SYS_CONSOLE_PRINT("MAIN: [Dock %d] EvConStatus\r\n", u8DockNo);
    //     SET_BIT(u32FaultCode, BMS_FAULT_EV_CON_STATUS);
    // }

    if (sChargingLiveInfo.LevdcRX_500ID_Info.u8EvChargingStopControl)
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] ChargingStopControl\r\n", u8DockNo);
        SET_BIT(u32FaultCode, BMS_FAULT_CHARGING_STOP_CONTROL);
    }

    if (sChargingLiveInfo.LevdcRX_500ID_Info.u8BatteryOverVol)
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] BatteryOverVoltage\r\n", u8DockNo);
        SET_BIT(u32FaultCode, BMS_FAULT_BATTERY_OVERVOLT);
    }

    /* -------- Warnings -------- */

    if (sChargingLiveInfo.LevdcRX_500ID_Info.u8HighBatteryTemp)
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] HighBatteryTemp\r\n", u8DockNo);
        SET_BIT(u32FaultCode, BMS_WARN_HIGH_TEMP);
    }

    if (sChargingLiveInfo.LevdcRX_500ID_Info.u8BatterVoltageDeviError)
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] VoltageDeviation\r\n", u8DockNo);
        SET_BIT(u32FaultCode, BMS_WARN_VOLTAGE_DEVIATION);
    }

    /* Store final fault bitmap */
    SESSION_SetBMSFaultBitmap(u8DockNo, u32FaultCode);

    /* Return TRUE if any critical fault */
    if (u32FaultCode & 0x0000FFFF)
    {
        return true;
    }
#endif
    return false;
}

static bool bCheckPMFault(uint8_t u8DockNo)
{
    static uint32_t u32PMFaultTicks[MAX_DOCKS] = {0};
    const uint32_t u32FaultThresholdMs = 2000U; // 2 second threshold
    uint32_t u32CurrentTick = xTaskGetTickCount();
    CH_State_e eState = SESSION_GetChargingState(u8DockNo);
    bool bPMFault = bGetPMFaultStatus(u8DockNo);
    if ((bPMFault == true) && (eState == CH_STATE_CHARGING))
    {
        if (u32PMFaultTicks[u8DockNo] == 0U)
        {
            u32PMFaultTicks[u8DockNo] = u32CurrentTick;
        }
        
        uint32_t u32ElapsedMs = (u32CurrentTick - u32PMFaultTicks[u8DockNo]) * portTICK_PERIOD_MS;
        if (u32ElapsedMs >= u32FaultThresholdMs)
        {
            SYS_CONSOLE_PRINT("PM fault triggered for Gun %d after %lu ms\r\n", u8DockNo, u32ElapsedMs);
            u32PMFaultTicks[u8DockNo] = 0U;
            return true;
        }
    }
    else
    {
        u32PMFaultTicks[u8DockNo] = 0U;
    }
    return false;
}
static bool bGetPMFaultStatus(uint8_t u8DockNo)
{
    uint32_t u32FaultCode = SESSION_GetPMFaultCode(u8DockNo);
    uint32_t u32PMFaultBitmap = 0;

    /* -------- Critical Faults (0–15) -------- */

    if (u32FaultCode & (1U << 0))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Critical Fault: Module input undervoltage\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_FAULT_INPUT_UNDERVOLT);
    }

    if (u32FaultCode & (1U << 1))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Critical Fault: Module input phase loss\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_FAULT_PHASE_LOSS);
    }

    if (u32FaultCode & (1U << 2))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Critical Fault: Module input overvoltage\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_FAULT_INPUT_OVERVOLT);
    }

    if (u32FaultCode & (1U << 3))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Critical Fault: Module output overvoltage\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_FAULT_OUTPUT_OVERVOLT);
    }

    if (u32FaultCode & (1U << 4))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Critical Fault: Module output overcurrent\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_FAULT_OUTPUT_OVERCURRENT);
    }

    if (u32FaultCode & (1U << 5))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Critical Fault: Module temperature high\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_FAULT_OVER_TEMP);
    }

    if (u32FaultCode & (1U << 6))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Critical Fault: Module fan fault\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_FAULT_FAN_FAULT);
    }

    if (u32FaultCode & (1U << 7))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Critical Fault: Module hardware fault\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_FAULT_HW_FAULT);
    }

    if (u32FaultCode & (1U << 8))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Critical Fault: Bus exception\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_FAULT_BUS_EXCEPTION);
    }

    if (u32FaultCode & (1U << 9))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Critical Fault: SCI communication exception\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_FAULT_SCI_EXCEPTION);
    }

    if (u32FaultCode & (1U << 10))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Critical Fault: Discharge fault\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_FAULT_DISCHARGE_FAULT);
    }

    if (u32FaultCode & (1U << 11))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Critical Fault: PFC shutdown due to exception\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_FAULT_PFC_SHUTDOWN);
    }

    /* -------- Non-Critical Faults / Warnings (16–31) -------- */

    if (u32FaultCode & (1U << 12))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Warning: Output undervoltage\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_WARN_OUTPUT_UNDERVOLT);
    }

    if (u32FaultCode & (1U << 13))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Warning: Output overvoltage\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_WARN_OUTPUT_OVERVOLT);
    }

    if (u32FaultCode & (1U << 14))
    {
        SYS_CONSOLE_PRINT("MAIN: [Dock %d] PM Warning: Power limit due to high\r\n", u8DockNo);
        SET_BIT(u32PMFaultBitmap, PM_WARN_POWER_LIMIT);
    }

    /* Store final bitmap */
    SESSION_SetPMFaultBitmap(u8DockNo, u32PMFaultBitmap);

    /* Return TRUE if any critical fault (0–15 bits) */
    if (u32PMFaultBitmap & 0x0000FFFF)
    {
        return true;
    }

    return false;
}

static bool bCheckBMSStatus(uint8_t u8DockNo)
{
    bool bRet = false;
    const uint32_t u32FaultThresholdMs = 10000U; // 10 second threshold
    uint32_t u32CurrentTick = xTaskGetTickCount();
    uint32_t u32LastBMSRxTick = SESSION_GetBMSLastRxTime(u8DockNo);
    CH_State_e eState = SESSION_GetChargingState(u8DockNo);
    uint32_t u32ElapsedMs = (u32CurrentTick - u32LastBMSRxTick);

    if (u32ElapsedMs >= u32FaultThresholdMs)
    {
        if (SESSION_GetBMSRxStatus(u8DockNo) == true)
        {
            SESSION_SetBMSRxStatus(u8DockNo, false);
            SYS_CONSOLE_PRINT("BMS communication timeout for Gun %d: Last Rx %lu ms ago\r\n", u8DockNo, u32ElapsedMs);
        }
        if (eState == CH_STATE_CHARGING)
        {
            bRet = true;
        }
    }
    else 
    {
        if (u32LastBMSRxTick == 0)
        {
            SESSION_SetBMSRxStatus(u8DockNo, false);
        }
        else if (SESSION_GetBMSRxStatus(u8DockNo) == false)
        {
            SESSION_SetBMSRxStatus(u8DockNo, true);
            SYS_CONSOLE_PRINT("BMS communication restored for Gun %d: Last Rx %lu ms ago\r\n", u8DockNo, u32ElapsedMs);
        }
    }
    return bRet;
}

static bool bCheckPMStatus(uint8_t u8DockNo)
{
    bool bRet = false;
    const uint32_t u32FaultThresholdMs = 10000U; // 10 second threshold
    uint32_t u32CurrentTick = xTaskGetTickCount();
    uint32_t u32LastPMRxTick = SESSION_GetPMLastRxTime(u8DockNo);
    CH_State_e eState = SESSION_GetChargingState(u8DockNo);
    uint32_t u32ElapsedMs = (u32CurrentTick - u32LastPMRxTick);

    if (u32ElapsedMs >= u32FaultThresholdMs)
    {
        if (SESSION_GetPMRxStatus(u8DockNo) == true)
        {
            SESSION_SetPMRxStatus(u8DockNo, false);
            SYS_CONSOLE_PRINT("PM communication timeout for Gun %d: Last Rx %lu ms ago\r\n", u8DockNo, u32ElapsedMs);
        }
        if (eState == CH_STATE_CHARGING)
        {
            bRet = true;
        }
    }
    else 
    {
        if (u32LastPMRxTick == 0)
        {
            SESSION_SetPMRxStatus(u8DockNo, false);
        }
        else if (SESSION_GetPMRxStatus(u8DockNo) == false)
        {
            SESSION_SetPMRxStatus(u8DockNo, true);
            SYS_CONSOLE_PRINT("PM communication restored for Gun %d: Last Rx %lu ms ago\r\n", u8DockNo, u32ElapsedMs);
        }
    }
    return bRet;
}

static bool bCheckZeroCurrentFault(uint8_t u8DockNo)
{
    static uint32_t u32ZeroCurrentTicks[MAX_DOCKS] = {0};
    const uint32_t u32FaultThresholdMs = 30000U; // 30 second threshold
    uint32_t u32CurrentTick = xTaskGetTickCount();
    const float fZeroCurrentThreshold = 0.5f; // Threshold to consider as zero current
    float fCurrent = SESSION_GetPmOutputCurrent(u8DockNo);
    CH_State_e eState = SESSION_GetChargingState(u8DockNo);
    
    if ((eState == CH_STATE_CHARGING) && (fCurrent < fZeroCurrentThreshold)) // Threshold for zero current fault
    {
        if (u32ZeroCurrentTicks[u8DockNo] == 0U)
        {
            u32ZeroCurrentTicks[u8DockNo] = u32CurrentTick;
        }
        
        uint32_t u32ElapsedMs = (xu32CurrentTick - u32ZeroCurrentTicks[u8DockNo]);
        if (u32ElapsedMs >= u32FaultThresholdMs)
        {
            SYS_CONSOLE_PRINT("Zero current fault detected for Gun %d after %lu ms: Current = %.2f A\r\n", u8DockNo, u32ElapsedMs, fCurrent);
            u32ZeroCurrentTicks[u8DockNo] = 0U;
            return true;
        }
    }
    else
    {
        u32ZeroCurrentTicks[u8DockNo] = 0U;
    }
    return false;
}

static bool bCheckPrechargeFailure(uint8_t u8DockNo)
{
    static uint32_t u32PrechargeFailureTicks[MAX_DOCKS] = {0};
    const uint32_t u32FaultThresholdMs = 30000U; // 30 seconds
    uint32_t u32CurrentTick = xTaskGetTickCount();
    if ((SESSION_GetAuthenticationCommand(u8DockNo) == 0U) ||
        (SESSION_GetChargingState(u8DockNo) == CH_STATE_CHARGING)){ // Only check during precharge phase
        u32PrechargeFailureTicks[u8DockNo] = 0U;
        return false;
    }

    if (u32PrechargeFailureTicks[u8DockNo] == 0U) {
        u32PrechargeFailureTicks[u8DockNo] = u32CurrentTick + pdMS_TO_TICKS(u32FaultThresholdMs);
        return false;
    }

    if (xTaskGetTickCount() >= u32PrechargeFailureTicks[u8DockNo]) {
        SYS_CONSOLE_PRINT("Precharge failure detected for Dock %d\r\n", u8DockNo);
        u32PrechargeFailureTicks[u8DockNo] = 0U;
        return true;
    }
    return false;
}

void vPrintSystemData(uint8_t dockNo)
{
    uint32_t currentTick = xTaskGetTickCount();
    static uint32_t u32NextPrintTick[MAX_DOCKS] = {0};
    const uint32_t printIntervalTicks = pdMS_TO_TICKS(10000U); // 10 seconds
    CH_State_e chargingState = SESSION_GetChargingState(dockNo);
    static CH_State_e prevChargingState[MAX_DOCKS] = {CH_STATE_INIT};
    /* Check if interval elapsed (overflow safe) */
    if (currentTick > u32NextPrintTick[dockNo] || chargingState != prevChargingState[dockNo])
    {
        prevChargingState[dockNo] = chargingState;
        u32NextPrintTick[dockNo] = currentTick + printIntervalTicks;
        /* Print all system info */
        vPrintStateAndDeviceInfo(dockNo);
        if (chargingState == CH_STATE_CHARGING)
        {
            vPrintChargingInfo(dockNo);
        }
    }
}
static void vPrintStateAndDeviceInfo(uint8_t dockNo)
{
    uint8_t bmsStatus = SESSION_GetBMSRxStatus(dockNo);
    uint8_t pmStatus  = SESSION_GetPMRxStatus(dockNo);

    CH_State_e chargingState = SESSION_GetChargingState(dockNo);
    uint32_t authCmd       = SESSION_GetAuthenticationCommand(dockNo);
    uint32_t sysFault      = SESSION_GetSystemFaultBitmap(dockNo);
    uint32_t bmsFault      = SESSION_GetBMSFaultBitmap(dockNo);
    uint32_t pmFault       = SESSION_GetPMFaultBitmap(dockNo);

    SYS_CONSOLE_PRINT(
        "\r\n================ CHARGING INFO - Dock %d ================\r\n"
        "Charging State        : %s\r\n"
        "Authentication Cmd    : %lu\r\n"
        "BMS Status            : %s\r\n"
        "PM Status             : %s\r\n"
        "System Fault Bitmap   : 0x%08lX\r\n"
        "BMS Fault Bitmap      : 0x%08lX\r\n"
        "PM Fault Bitmap       : 0x%08lX\r\n"
        "=======================================================\r\n",
        dockNo,
        CH_GetStateString(chargingState),
        authCmd,
        bmsStatus ? "Connected" : "Disconnected",
        pmStatus  ? "Connected" : "Disconnected",
        sysFault,
        bmsFault,
        pmFault
    );
}

static void vPrintChargingInfo(uint8_t u8DockNo)
{
    uint16_t u16CurrentSOC = SESSION_GetCurrentSoc(u8DockNo);
    uint16_t u16InitialSOC = SESSION_GetInitialSoc(u8DockNo);
    float fDemandVoltage = (float)SESSION_GetBMSDemandVoltage(u8DockNo);
    float fDemandCurrent = SESSION_GetBMSDemandCurrent(u8DockNo);
    float fOutputVoltage = (float)SESSION_GetPmOutputVoltage(u8DockNo);
    float fOutputCurrent = (float)SESSION_GetPmOutputCurrent(u8DockNo);
    uint16_t u16SessionTime = 0;//SESSION_GetSessionTime(u8DockNo);
    float fSessionEnergy = (float)SESSION_GetEnergyDelivered(u8DockNo);

    SYS_CONSOLE_PRINT("====================== CHARGING INFO - Dock %d ======================\r\n", u8DockNo);
    SYS_CONSOLE_PRINT("Demand : [Voltage: %4.0f V] [Current: %4.0f A]\r\n", fDemandVoltage, fDemandCurrent);
    SYS_CONSOLE_PRINT("Output : [Voltage: %4.0f V] [Current: %4.0f A]\r\n", fOutputVoltage, fOutputCurrent);
    SYS_CONSOLE_PRINT("SOC : %3d%% (Initial: %3d%%)\r\n", u16CurrentSOC, u16InitialSOC);
    SYS_CONSOLE_PRINT("Energy : %6.2f Wh\r\n", fSessionEnergy);
    SYS_CONSOLE_PRINT("===================================================================\r\n");
}

static const char* CH_GetStateString(CH_State_e state)
{
    switch(state)
    {
        case CH_STATE_INIT: return "INIT";
        case CH_STATE_AUTH_SUCCESS: return "AUTH_SUCCESS";
        case CH_STATE_PARAM_VALIDATE: return "PARAM_VALIDATE";
        case CH_STATE_CONNECTION_CONFIRMED: return "CONNECTION_CONFIRMED";
        case CH_STATE_INITIALIZE: return "INITIALIZE";
        case CH_STATE_PRECHARGE: return "PRECHARGE";
        case CH_STATE_CHARGING: return "CHARGING";
        case CH_STATE_SHUTDOWN: return "SHUTDOWN";
        case CH_STATE_SESSION_COMPLETE: return "SESSION_COMPLETE";
        case CH_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}
//////////////////////////////////////////////////////////////////////////////////


/*==============================================================================
 * TVS CAN Signal Resolution & Offset Factors
 *============================================================================*/
#define TVS_FACTOR_CURRENT_0x91        0.015625f   /* 0x91 CHARGER_Current resolution      */
#define TVS_FACTOR_VOLTAGE_0x91        0.015625f   /* 0x91 CHARGER_TerminalVoltage res      */
#define TVS_FACTOR_BMS_PROFILE         0.001f      /* 0x101 BMS profile signals resolution  */

#define TVS_OFFSET_AC_INPUT            50          /* CHARGER_ACInput offset = 50V          */
#define TVS_OFFSET_TEMPERATURE         50          /* CHARGER_Temperature offset = -50°C    */
#define TVS_OFFSET_BMS_TEMPERATURE     30          /* BMS_Temperature offset = -30°C        */

/*==============================================================================
 * TVS Charger Limits
 *============================================================================*/
#define TVS_MAX_VOLTAGE                 128.0f      /* V  - BMS_Voltage max         */
#define TVS_MAX_CURRENT                 65.535f     /* A  - BMS profile max         */
#define TVS_MIN_AC_INPUT                50          /* V  - AC input min (offset)   */
#define TVS_MAX_AC_INPUT                305         /* V  - AC input max            */
#define TVS_SHUTOFF_DELAY_MS            pdMS_TO_TICKS(500)

/*==============================================================================
 * TVS Byte 4 Bitfield Mask helpers (ErrorState=bits0-3, Fan=bit4, Output=bit5, Derating=bit6)
 *============================================================================*/
#define TVS_BYTE4_ERR_MASK              0x0Fu
#define TVS_BYTE4_FAN_MASK              0x10u
#define TVS_BYTE4_OUTPUT_MASK           0x20u
#define TVS_BYTE4_DERATING_MASK         0x40u



/*==============================================================================
 * Static Live Data Store
 *============================================================================*/
 TVS_MsgFrameInfo_t TVS_LiveInfo[MAX_DOCKS];

/*==============================================================================
 * Compile-time size assertions
 *============================================================================*/
static_assert(sizeof(TVS_Tx90_Info_t)          == 8U, "TVS_Tx90_Info_t must be 8 bytes");
static_assert(sizeof(TVS_Tx91_ChargeProfile_t) == 8U, "TVS_Tx91_ChargeProfile_t must be 8 bytes");
static_assert(sizeof(TVS_Tx92_FMVersionInfo_t) == 8U, "TVS_Tx92_FMVersionInfo_t must be 8 bytes");
static_assert(sizeof(TVS_Rx100_Status_t)       == 8U, "TVS_Rx100_Status_t must be 8 bytes");
static_assert(sizeof(TVS_Rx101_Profile_t)      == 8U, "TVS_Rx101_Profile_t must be 8 bytes");

/*==============================================================================
 * Signal Encode / Decode Helpers
 *============================================================================*/

/* Encode physical -> raw */
#define TVS_ENCODE_CHARGER_CURRENT(phys)     ((uint16_t)((phys) / TVS_FACTOR_CURRENT_0x91))
#define TVS_ENCODE_CHARGER_VOLTAGE(phys)     ((uint16_t)((phys) / TVS_FACTOR_VOLTAGE_0x91))
#define TVS_ENCODE_AC_INPUT(phys)            ((uint8_t)((phys)  - TVS_OFFSET_AC_INPUT))
#define TVS_ENCODE_TEMPERATURE(phys)         ((uint8_t)((phys)  + TVS_OFFSET_TEMPERATURE))
#define TVS_ENCODE_BMS_TEMPERATURE(phys)     ((uint8_t)((phys)  + TVS_OFFSET_BMS_TEMPERATURE))

/* Decode raw -> physical */
#define TVS_DECODE_CHARGER_CURRENT(raw)      ((float)(raw) * TVS_FACTOR_CURRENT_0x91)
#define TVS_DECODE_CHARGER_VOLTAGE(raw)      ((float)(raw) * TVS_FACTOR_VOLTAGE_0x91)
#define TVS_DECODE_BMS_PROFILE_CURRENT(raw)  ((float)(raw) * TVS_FACTOR_BMS_PROFILE)
#define TVS_DECODE_BMS_PROFILE_VOLTAGE(raw)  ((float)(raw) * TVS_FACTOR_BMS_PROFILE)
#define TVS_DECODE_AC_INPUT(raw)             ((uint16_t)(raw) + TVS_OFFSET_AC_INPUT)
#define TVS_DECODE_TEMPERATURE(raw)          ((int16_t)(raw)  - TVS_OFFSET_TEMPERATURE)
#define TVS_DECODE_BMS_TEMPERATURE(raw)      ((int16_t)(raw)  - TVS_OFFSET_BMS_TEMPERATURE)

/*==============================================================================
 * GET/SET Function
 *============================================================================*/
bool bGetSetTVSBMSData(uint8_t u8DockNo,
                    TVS_MsgFrameInfo_t *psData,
                    uint8_t u8Operation)
{
    bool bRet = false;

    if ((psData == NULL) || (u8DockNo >= MAX_DOCKS)) {
        return false;
    }

    taskENTER_CRITICAL();
    switch (u8Operation)
    {
        case SET_PARA: {
            memcpy((void *)&TVS_LiveInfo[u8DockNo],
                   (const void *)psData,
                   sizeof(TVS_MsgFrameInfo_t));
            bRet = true;
        }
        break;
        case GET_PARA: {
            memcpy((void *)psData,
                   (const void *)&TVS_LiveInfo[u8DockNo],
                   sizeof(TVS_MsgFrameInfo_t));
            bRet = true;
        }
        break;
        default: {
            bRet = false;
        }
        break;
    }
    taskEXIT_CRITICAL();
    return bRet;
}

/*==============================================================================
 * State Function Prototypes
 *============================================================================*/
static void vTVS_SendInitReq         (uint8_t u8DockNo);
static void vTVS_AuthSuccess         (uint8_t u8DockNo);
static void vTVS_ValidateParameters  (uint8_t u8DockNo);
static void vTVS_ConnectionConfirmed (uint8_t u8DockNo);
static void vTVS_InitializeState     (uint8_t u8DockNo);
static void vTVS_PreChargingState    (uint8_t u8DockNo);
static void vTVS_StartCharging       (uint8_t u8DockNo);
static void vTVS_Shutdown            (uint8_t u8DockNo);
static void vTVS_SessionComplete     (uint8_t u8DockNo);
static void vTVS_SessionError        (uint8_t u8DockNo);

/*==============================================================================
 * CH_STATE_INIT
 * - Populate charger identity and FW version
 * - Send initial frame to BMS
 * - Wait for authentication command from BMS
 *============================================================================*/
static void vTVS_SendInitReq(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);
    /* Charger identity */
    sTVSFrame.TVS_Tx90_ChargerInfo.u8ChargerType  = TVS_CHARGER_TYPE_3KW_DELTA_OFFBOARD;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8ErrorState   = TVS_CHARGER_ERR_NONE;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Fan          = TVS_FAN_OFF;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Output       = TVS_OUTPUT_OFF;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Derating     = TVS_DERATING_NONE;

    /* Firmware version */
    sTVSFrame.TVS_Tx92_FMVersionInfo.u8FWVersionMajor     = FW_VERSION_MAJOR;
    sTVSFrame.TVS_Tx92_FMVersionInfo.u8FWVersionMinor     = FW_VERSION_MINOR;
    sTVSFrame.TVS_Tx92_FMVersionInfo.u8FWVersionIteration = FW_VERSION_ITERATION;
    sTVSFrame.TVS_Tx92_FMVersionInfo.u8FWChargerType      = FW_CHARGER_TYPE;
    sTVSFrame.TVS_Tx92_FMVersionInfo.u8FWReleaseDateDD    = FW_RELEASE_DATE_DD;
    sTVSFrame.TVS_Tx92_FMVersionInfo.u8FWReleaseDateMM    = FW_RELEASE_DATE_MM;
    sTVSFrame.TVS_Tx92_FMVersionInfo.u8FWReleaseDateY1Y2  = FW_RELEASE_DATE_Y1Y2;
    sTVSFrame.TVS_Tx92_FMVersionInfo.u8FWReleaseDateY3Y4  = FW_RELEASE_DATE_Y3Y4;

    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);

    /* Advance only when BMS sends auth command */
    if (SESSION_GetAuthenticationCommand(u8DockNo) == 1U)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_AUTH_SUCCESS);
    }
}

/*==============================================================================
 * CH_STATE_AUTH_SUCCESS
 * - Authentication acknowledged
 * - Immediately advance to parameter validation
 *============================================================================*/
static void vTVS_AuthSuccess(uint8_t u8DockNo)
{
    SESSION_SetChargingState(u8DockNo, CH_STATE_PARAM_VALIDATE);
}

/*==============================================================================
 * CH_STATE_PARAM_VALIDATE
 * - Wait for first valid BMS Rx frame (0x100 + 0x101)
 * - Populate charger capability frame (0x90 + 0x91)
 * - Validate BMS demand against charger limits
 *============================================================================*/
static void vTVS_ValidateParameters(uint8_t u8DockNo)
{
    if (SESSION_GetBMSRxStatus(u8DockNo) == false)
    {
        SESSION_SetStartChargingComm(u8DockNo, false);
        return;
    }

    TVS_MsgFrameInfo_t sTVSFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    /* Decode BMS profile — physical values */
    float fDemandVoltage = TVS_DECODE_BMS_PROFILE_VOLTAGE(sTVSFrame.TVS_Rx101_BMSProfile.u16MaxChargeVoltage);
    float fDemandCurrent = TVS_DECODE_BMS_PROFILE_CURRENT(sTVSFrame.TVS_Rx101_BMSProfile.u16MaxChargeCurrent);

    SYS_CONSOLE_PRINT("G%d -> BMS Profile V: %.3f  I: %.3f\r\n",
                      (int)u8DockNo, (double)fDemandVoltage, (double)fDemandCurrent);

    sTVSFrame.TVS_Tx90_ChargerInfo.u8ErrorState = TVS_CHARGER_ERR_NONE;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Output     = TVS_OUTPUT_OFF;

    SESSION_SetStartChargingComm(u8DockNo, true);
    SESSION_SetChargingState(u8DockNo, CH_STATE_CONNECTION_CONFIRMED);
    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);
}

/*==============================================================================
 * CH_STATE_CONNECTION_CONFIRMED
 * - Read BMS demanded voltage / current (0x101)
 * - Validate against charger hardware limits
 * - Advance to INITIALIZE when within limits
 *============================================================================*/
static void vTVS_ConnectionConfirmed(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    /* Decode physical values from BMS profile */
    float fDemandVoltage   = TVS_DECODE_BMS_PROFILE_VOLTAGE(sTVSFrame.TVS_Rx101_BMSProfile.u16MaxChargeVoltage);
    float fDemandCurrent   = TVS_DECODE_BMS_PROFILE_CURRENT(sTVSFrame.TVS_Rx101_BMSProfile.u16MaxChargeCurrent);

    /* Store in session */
    SESSION_SetBMSDemandVoltage(u8DockNo, fDemandVoltage);
    SESSION_SetBMSDemandCurrent(u8DockNo, fDemandCurrent);

    SYS_CONSOLE_PRINT("G%d -> Demand V: %.3f  I: %.3f\r\n",
                      (int)u8DockNo,
                      (double)fDemandVoltage,
                      (double)fDemandCurrent);

    /* Charger error clear */
    sTVSFrame.TVS_Tx90_ChargerInfo.u8ErrorState = TVS_CHARGER_ERR_NONE;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Output     = TVS_OUTPUT_OFF;

    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);

    /* Validate limits */
    if ((fDemandVoltage <= TVS_MAX_VOLTAGE) && (fDemandCurrent <= TVS_MAX_CURRENT) &&
        (fDemandVoltage != 0.0f) && (fDemandCurrent != 0.0f))
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_INITIALIZE);
    }
}

/*==============================================================================
 * CH_STATE_INITIALIZE
 * - Wait for BMS ready signal (BMS_ErrorState == NO_ERROR)
 * - Gate to PRECHARGE once BMS is ready
 *============================================================================*/
static void vTVS_InitializeState(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    uint8_t u8BMSError = sTVSFrame.TVS_Rx100_BMSStatus.u8ErrorState;

    if (u8BMSError == TVS_BMS_ERR_NONE)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_PRECHARGE);
    }
    else
    {
        /* BMS not ready — stay in CONNECTION_CONFIRMED */
        SESSION_SetChargingState(u8DockNo, CH_STATE_CONNECTION_CONFIRMED);
    }
}

/*==============================================================================
 * CH_STATE_PRECHARGE
 * - Apply pre-charge current from BMS profile (0x101 BMS_PreChargeCurrent)
 * - Turn on charger output
 * - Advance to CHARGING
 *============================================================================*/
static void vTVS_PreChargingState(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    /* Enable charger output and fan */
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Output   = TVS_OUTPUT_ON;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Fan      = TVS_FAN_ON;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Derating = TVS_DERATING_NONE;

    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);

    SESSION_SetPMState(u8DockNo, RECTIFIER_ON);
    SESSION_SetChargingState(u8DockNo, CH_STATE_CHARGING);
}

/*==============================================================================
 * CH_STATE_CHARGING
 * - Apply full charge current / voltage from BMS profile
 * - Report live output voltage / current back to BMS (0x91)
 * - Monitor BMS_ErrorState, SOC, temperature
 * - Apply derating on charger over-temperature
 * - Advance to SHUTDOWN on SOC complete or BMS stop request
 *============================================================================*/
static void vTVS_StartCharging(uint8_t u8DockNo)
{
    TVS_MsgFrameInfo_t sTVSFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    /* --- GPIO relay control --- */
    bGPIO_Operation(DO_AC_RELAY_ON, u8DockNo);
    bGPIO_Operation(DO_DC_RELAY_ON, u8DockNo);

    float fOutputVoltage = SESSION_GetPmOutputVoltage(u8DockNo);
    float fOutputCurrent = SESSION_GetPmOutputCurrent(u8DockNo);
    /* --- Apply demand to charger profile --- */
    sTVSFrame.TVS_Tx91_ChargeProfile.u16ChargingCurrent = TVS_ENCODE_CHARGER_CURRENT(fOutputCurrent);
    sTVSFrame.TVS_Tx91_ChargeProfile.u16ChargingVoltage = TVS_ENCODE_CHARGER_VOLTAGE(fOutputVoltage);

    /* --- Rolling counter update --- */
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Counter++;

    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);

    /* Stay in CHARGING */
    SESSION_SetChargingState(u8DockNo, CH_STATE_CHARGING);
}

/*==============================================================================
 * CH_STATE_SHUTDOWN
 * - Ramp down charger output to zero
 * - Turn off relays and rectifier
 * - Clear live frame after delay
 *============================================================================*/
static void vTVS_Shutdown(uint8_t u8DockNo)
{
    /* Relays off first */
    bGPIO_Operation(DO_AC_RELAY_OFF, u8DockNo);
    bGPIO_Operation(DO_DC_RELAY_OFF, u8DockNo);
    SESSION_SetPMState(u8DockNo, RECTIFIER_OFF);

    TVS_MsgFrameInfo_t sTVSFrame = {0};
    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, GET_PARA);

    /* Zero out charge profile */
    sTVSFrame.TVS_Tx91_ChargeProfile.u16ChargingCurrent = 0U;
    sTVSFrame.TVS_Tx91_ChargeProfile.u16ChargingVoltage = 0U;

    /* Safe charger info state */
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Output   = TVS_OUTPUT_OFF;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Fan      = TVS_FAN_OFF;
    sTVSFrame.TVS_Tx90_ChargerInfo.u8Derating = TVS_DERATING_NONE;

    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);

    /* Shutoff delay then clear all live data */
    vTaskDelay(TVS_SHUTOFF_DELAY_MS);
    SESSION_SetStartChargingComm(u8DockNo, false);

    (void)memset(&sTVSFrame, 0, sizeof(sTVSFrame));
    (void)bGetSetTVSBMSData(u8DockNo, &sTVSFrame, SET_PARA);

    SESSION_SetChargingState(u8DockNo, CH_STATE_SESSION_COMPLETE);
}

/*==============================================================================
 * CH_STATE_SESSION_COMPLETE
 * - Check for lingering faults
 * - Go to ERROR if fault present, else back to INIT
 *============================================================================*/
static void vTVS_SessionComplete(uint8_t u8DockNo)
{
    if (SESSION_GetSystemFaultBitmap(u8DockNo) != SYSTEM_FAULT_NONE)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_ERROR);
    }
    else
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);
    }
}

/*==============================================================================
 * CH_STATE_ERROR
 * - Clear fault and return to INIT once system fault is resolved
 *============================================================================*/
static void vTVS_SessionError(uint8_t u8DockNo)
{
    if (SESSION_GetSystemFaultBitmap(u8DockNo) == SYSTEM_FAULT_NONE)
    {
        SESSION_SetChargingState(u8DockNo, CH_STATE_INIT);
    }
}