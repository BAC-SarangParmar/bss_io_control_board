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
bool bGetSetBMSData(uint8_t u8DockNo,
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
    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

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

    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
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
    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

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
    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
}

static void v17017_ConnectionConfirmed(uint8_t u8DockNo)
{
    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    sChargingLiveInfo.LevdcTX_508ID_Info.u8ConLatchStatus = GUN_LATCHED;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8ChargingSysError = EVSE_NOERROR;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseMalFunctionError = EVSE_NOERROR;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStatus = EVSE_STANDBY;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStopCtrl = EVSE_NOERROR;

    uint16_t u16OutputVoltage = (uint16_t)(sChargingLiveInfo.LevdcRX_500ID_Info.u16DcOutputVolTarget / FACTOR_10);
    uint16_t u16OutputCurrent = (uint16_t)(sChargingLiveInfo.LevdcRX_500ID_Info.u16ReqDcCurrent / FACTOR_10);
    uint16_t u16BatVoltMaxLimit = (uint16_t)(sChargingLiveInfo.LevdcRX_500ID_Info.u16DcOutputVoltLimit / FACTOR_10);
    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
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
    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
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
    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    sChargingLiveInfo.LevdcTX_508ID_Info.u8EVSEReadyForCharge = EVSE_READY_FOR_CHARGE;

    sChargingLiveInfo.LevdcTX_509ID_Info.u8AvailDCOutputPower = (uint8_t)LEVDC_RATED_DC_OP_POWER;
    sChargingLiveInfo.LevdcTX_509ID_Info.u16RemainChargeTime = sChargingLiveInfo.LevdcRX_501ID_Info.u16EstimatedChargingTime;

    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
    SESSION_SetChargingState(u8DockNo, CH_STATE_CHARGING);

    SESSION_SetPMState(u8DockNo, RECTIFIER_ON);
}

static void v17017_StartCharging(uint8_t u8DockNo)
{

    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStatus = EVSE_CHARGING;

    sChargingLiveInfo.LevdcTX_509ID_Info.u16EVSEoutputVoltage = (uint16_t)(SESSION_GetPmOutputVoltage(u8DockNo) * FACTOR_10);
    sChargingLiveInfo.LevdcTX_509ID_Info.u16EVSEoutputCurrent = (uint16_t)(SESSION_GetPmOutputCurrent(u8DockNo) * FACTOR_10);
    bGPIO_Operation(DO_AC_RELAY_ON, u8DockNo);
    bGPIO_Operation(DO_DC_RELAY_ON, u8DockNo);

    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);

    SESSION_SetChargingState(u8DockNo, CH_STATE_CHARGING);
}

static void v17017_Shutdown(uint8_t u8DockNo)
{
    bGPIO_Operation(DO_AC_RELAY_OFF, u8DockNo);
    bGPIO_Operation(DO_DC_RELAY_OFF, u8DockNo);
    SESSION_SetPMState(u8DockNo, RECTIFIER_OFF);

    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStatus = EVSE_STANDBY;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8ConLatchStatus = GUN_UNLATCHED;
    sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStopCtrl = EVSE_ERROR;
    sChargingLiveInfo.LevdcTX_508ID_Info.u16AvailOutputCur = 0U;
    sChargingLiveInfo.LevdcTX_508ID_Info.u16RatedOutputVol = 0U;

    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);

    vTaskDelay(LEVDC_SHUTOFF_DELAY_MS);
    SESSION_SetStartChargingComm(u8DockNo, false);
    (void)memset(&sChargingLiveInfo, 0, sizeof(sChargingLiveInfo));
    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);

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
static void Charging_StateMachine(uint8_t u8DockNo)
{
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
    (void)bGetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

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
    ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
    uint32_t u32FaultCode = 0;

    if (bGetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA) == false)
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
