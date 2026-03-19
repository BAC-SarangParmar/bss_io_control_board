// /******************************************************************************
//  * @file    ChargingHandler.c
//  * @brief   Charging Handler module implementation
//  *
//  * @details
//  * Implements charging control logic including initialization, monitoring,
//  * and state transitions.
//  *
//  * @author  Sarang Parmar
//  * @date    16-Mar-2026
//  * @version 1.0
//  *
//  ******************************************************************************/

// /******************************************************************************
//  * Includes
//  ******************************************************************************/

#include "ChargingHandler.h"
#include "definitions.h"                // SYS function prototypes
#include "configuration.h"
#include "device.h"

// /******************************************************************************
//  * Private Macros
//  ******************************************************************************/
// /******************************************************************************
//  * CHARGING Task Configuration Macros
//  ******************************************************************************/

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


// #define LEVDC_MAX_VOLTAGE 120 // As per Stander 120 volt and 100 Amp req
// #define LEVDC_MAX_CURRENT 100
// /******************************************************************************
//  * Private Type Definitions
//  ******************************************************************************/

// /******************************************************************************
//  * Private Variables
//  ******************************************************************************/
static TaskHandle_t xCHARGING_TASK = NULL;
// CH_State_e CH_LiveStage[MAX_DOCKS] = {CH_STATE_INIT};
// ChargingMsgFrameInfo_t Charging_LiveInfo[MAX_DOCKS];
// bool bStartChargingComm[MAX_DOCKS] = {false};
// /******************************************************************************
//  * Private Function Prototypes
//  ******************************************************************************/
static void CHARGING_TASK(void *pvParameters);

// static void Charging_StateMachine(uint8_t u8DockNo);
// static void Charging_StateInit(uint8_t u8DockNo);
// static void Charging_StateAuthSuccess(uint8_t u8DockNo);
// static void Charging_StateParamValidate(uint8_t u8DockNo);
// static void Charging_StateInitialize(uint8_t u8DockNo);
// static void Charging_StatePrecharge(uint8_t u8DockNo);
// static void Charging_StateCharging(uint8_t u8DockNo);
// static void Charging_StateShutdown(uint8_t u8DockNo);
// static void Charging_StateSessionComplete(uint8_t u8DockNo);
// static void Charging_StateError(uint8_t u8DockNo);
// /******************************************************************************
//  * Global Function Definitions
//  ******************************************************************************/
// /******************************************************************************
//  * @brief  Initialize CHARGING Task
//  *
//  * @return true  - Task created successfully
//  * @return false - Task creation failed
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
        LOG_I("CHARGING_TASK Created\r\n");
        bStatus = true;
    }
    else
    {
        LOG_E("CHARGING_TASK Creation Failed\r\n");
    }

    return bStatus;
}
// uint8_t u8GetSetBMSData(uint8_t u8DockNo,
//                                  ChargingMsgFrameInfo_t *psData,
//                                  uint8_t u8Operation)
// {
//     uint8_t u8Ret = false;
//     /* Input validation */
//     if ((psData == NULL) || (u8DockNo >= MAX_DOCKS))
//     {
//         return false;
//     }
//     taskENTER_CRITICAL();

//     switch (u8Operation)
//     {
//         case SET_PARA:
//         {
//             memcpy((void *)&Charging_LiveInfo[u8DockNo],
//                    (const void *)psData,
//                    sizeof(ChargingMsgFrameInfo_t));

//             u8Ret = true;
//         }
//         break;

//         case GET_PARA:
//         {
//             memcpy((void *)psData,
//                    (const void *)&Charging_LiveInfo[u8DockNo],
//                    sizeof(ChargingMsgFrameInfo_t));

//             u8Ret = true;
//         }
//         break;

//         default:
//         {
//             /* Invalid operation */
//             u8Ret = false;
//         }
//         break;
//     }

//     taskEXIT_CRITICAL();

//     return u8Ret;
// }

// /* EV readiness check — preserves original logic but safer copy-by-value */
// static bool bIsEvReadyForCharging(uint8_t u8DockNo)
// {
//     bool bRet = true;
//     ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
//     (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

//     if (sChargingLiveInfo.LevdcRX_500ID_Info.u8EvChargingEnable)
//     {
//         LOG_I("MAIN: EV Charging Enabled for G%d\r\n", (int)u8DockNo);
//         bRet = true;
//     }
//     else if (sChargingLiveInfo.LevdcRX_500ID_Info.u8EvConStatus ||
//              sChargingLiveInfo.LevdcRX_500ID_Info.u8EvChargingPosition ||
//              sChargingLiveInfo.LevdcRX_500ID_Info.u8WaitReqToEngTransfer ||
//              sChargingLiveInfo.LevdcRX_500ID_Info.u8EnergyTransferError)
//     {
//         LOG_E("MAIN: Energy Transfer Error/EV Connection/Position/Wait Request Issue for G%d\r\n", (int)u8DockNo);
//         bRet = false;
//     }
//     LOG_I("MAIN: bIsEvReadyForCharging(G%d) = %d\r\n", (int)u8DockNo, (int)bRet);
//     return bRet;
// }
// /******************************************************************************
//  * Private Function Definitions
//  ******************************************************************************/
static void CHARGING_TASK(void *pvParameters)
{
    (void)pvParameters;

    for (;;)
    {
        // for (uint8_t u8DockNo = DOCK_1; u8DockNo < MAX_DOCKS; u8DockNo++)
        // {
        //     // Charging_StateMachine(u8DockNo);
        // }

        vTaskDelay(pdMS_TO_TICKS(CHARGING_TASK_DELAY_MS));
    }
}

// static void v17017_SendInitReq(uint8_t u8DockNo)
// {
//     ChargingMsgFrameInfo_t sChargingLiveInfo = {0};

//     sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStopCtrl = EVSE_NOERROR;
//     sChargingLiveInfo.LevdcTX_509ID_Info.u8ControlProtocolNum = 1;
//     sChargingLiveInfo.LevdcTX_510ID_Info.u8EVSEVolatageControlOpt = VOLTAGE_CONTROL_ENABLED;

//     (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);

//     SET_CHARGING_LIVE_STAGE(u8DockNo, INIT_PROTOCOL_REQUEST);
// }

// static void v17017_AuthSuccess(uint8_t u8DockNo)
// {
//     SET_CHARGING_LIVE_STAGE(u8DockNo, PARAMETER_VALIDATION);
// }

// static void v17017_ValidateParameters(uint8_t u8DockNo)
// {
//     if (SESSION_GetBMSRxStatus(u8DockNo) == false)
//     {
//         bStartChargingComm[u8DockNo] = false;
//         return;
//     }

//     ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
//     (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

//     sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStopCtrl = EVSE_NOERROR;
//     sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStatus = EVSE_CHARGING;

//     sChargingLiveInfo.LevdcTX_509ID_Info.u8AvailDCOutputPower = 0xFF;
//     sChargingLiveInfo.LevdcTX_509ID_Info.u16RemainChargeTime = 0xFFFF;
//     sChargingLiveInfo.LevdcTX_508ID_Info.u16RatedOutputVol = (uint16_t)(LEVDC_MAX_VOLTAGE * FACTOR_10);
//     sChargingLiveInfo.LevdcTX_508ID_Info.u16ConfDCvolLimit = (uint16_t)(LEVDC_MAX_VOLTAGE * FACTOR_10);
//     sChargingLiveInfo.LevdcTX_508ID_Info.u16AvailOutputCur = (uint16_t)(LEVDC_MAX_VOLTAGE * FACTOR_10);

//     sChargingLiveInfo.LevdcTX_508ID_Info.u8EVCompatible = EV_COMPATIBLE;
//     bStartChargingComm[u8DockNo] = true;
//     SET_CHARGING_LIVE_STAGE(u8DockNo, CONNECTION_CONFIRMED);
//     (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
// }

// static void v17017_ConnectionConfirmed(uint8_t u8DockNo)
// {
//     ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
//     (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

//     sChargingLiveInfo.LevdcTX_508ID_Info.u8ConLatchStatus = GUN_LATCHED;
//     sChargingLiveInfo.LevdcTX_508ID_Info.u8ChargingSysError = EVSE_NOERROR;
//     sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseMalFunctionError = EVSE_NOERROR;
//     sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStatus = EVSE_STANDBY;
//     sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStopCtrl = EVSE_NOERROR;

//     uint16_t u16OutputVoltage = (uint16_t)(sChargingLiveInfo.LevdcRX_500ID_Info.u16DcOutputVolTarget / FACTOR_10);
//     uint16_t u16OutputCurrent = (uint16_t)(sChargingLiveInfo.LevdcRX_500ID_Info.u16ReqDcCurrent / FACTOR_10);
//     uint16_t u16BatVoltMaxLimit = (uint16_t)(sChargingLiveInfo.LevdcRX_500ID_Info.u16DcOutputVoltLimit / FACTOR_10);
//     (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
//     LOG_I("G%d ->Demand V: %d I: %d CAN: %d | Bat V Limit: %d\r\n",
//           (int)u8DockNo,
//           (int)u16OutputVoltage,
//           (int)u16OutputCurrent,
//           (int)SESSION_GetBMSRxStatus(u8DockNo),
//           (int)u16BatVoltMaxLimit);

//     if ((u16BatVoltMaxLimit <= LEVDC_MAX_VOLTAGE) && (u16OutputCurrent <= LEVDC_MAX_CURRENT))
//     {
//         SET_CHARGING_LIVE_STAGE(u8DockNo, INITIALIZE_STATE);
//     }
//     (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
// }

// static void v17017_InitializeState(uint8_t u8DockNo)
// {
//     if (bIsEvReadyForCharging(u8DockNo))
//     {
//         SET_CHARGING_LIVE_STAGE(u8DockNo, PRECHARGING_STATE);
//     }
//     else
//     {
//         SET_CHARGING_LIVE_STAGE(u8DockNo, CONNECTION_CONFIRMED);
//     }
// }

// static void v17017_PreChargingState(uint8_t u8DockNo)
// {
//     ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
//     (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

//     sChargingLiveInfo.LevdcTX_508ID_Info.u8EVSEReadyForCharge = EVSE_READY_FOR_CHARGE;

//     sChargingLiveInfo.LevdcTX_509ID_Info.u8AvailDCOutputPower = (uint8_t)RATED_DC_OUTPUT_POWER;
//     sChargingLiveInfo.LevdcTX_509ID_Info.u16RemainChargeTime = sChargingLiveInfo.LevdcRX_501ID_Info.u16EstimatedChargingTime;

//     (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);
//     SET_CHARGING_LIVE_STAGE(u8DockNo, CHARGING_ACTIVE);

//     SESSION_SetPMState(u8DockNo, RECTIFIER_ON);
// }

// static void v17017_StartCharging(uint8_t u8DockNo)
// {

//     ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
//     (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

//     sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStatus = EVSE_CHARGING;

//     sChargingLiveInfo.LevdcTX_509ID_Info.u16EVSEoutputVoltage = (uint16_t)(SESSION_GetOutputVoltage(u8DockNo) * FACTOR_10);
//     sChargingLiveInfo.LevdcTX_509ID_Info.u16EVSEoutputCurrent = (uint16_t)(SESSION_GetOutputCurrent(u8DockNo) * FACTOR_10);
//     vDO_Operation(OP_AC_CON_ON, u8DockNo);
//     vDO_Operation(OP_DC_CON_ON, u8DockNo);

//     (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);

//     SET_CHARGING_LIVE_STAGE(u8DockNo, CHARGING_ACTIVE);
// }

// static void v17017_Shutdown(uint8_t u8DockNo)
// {
//     vDO_Operation(OP_AC_CON_OFF, u8DockNo);
//     vDO_Operation(OP_DC_CON_OFF, u8DockNo);
//     SESSION_SetPMState(u8DockNo, RECTIFIER_OFF);

//     ChargingMsgFrameInfo_t sChargingLiveInfo = {0};
//     (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, GET_PARA);

//     sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStatus = EVSE_STANDBY;
//     sChargingLiveInfo.LevdcTX_508ID_Info.u8ConLatchStatus = GUN_UNLATCHED;
//     sChargingLiveInfo.LevdcTX_508ID_Info.u8EvseStopCtrl = EVSE_ERROR;
//     sChargingLiveInfo.LevdcTX_508ID_Info.u16AvailOutputCur = 0U;
//     sChargingLiveInfo.LevdcTX_508ID_Info.u16RatedOutputVol = 0U;

//     (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);

//     vTaskDelay(LEVDC_SHUTOFF_DELAY);

//     LEVDC_TxMsg[u8DockNo] = LEVDC_NO_NEED_TO_SEND_MSG;

//     if (GET_CONNECTOR_GUN_TYPE(u8DockNo) == LEVDC_TYPE6)
//     {
//         vDO_Operation(OP_AUX_RELAY_OFF, u8DockNo);
//     }

//     (void)memset(&sChargingLiveInfo, 0, sizeof(sChargingLiveInfo));
//     (void)u8GetSetBMSData(u8DockNo, &sChargingLiveInfo, SET_PARA);

//     SET_CHARGING_LIVE_STAGE(u8DockNo, SESSION_COMPLETE);
// }

// static void v17017_SessionComplete(uint8_t u8DockNo)
// {
//     SET_CHARGING_LIVE_STAGE(u8DockNo, INIT_PROTOCOL_REQUEST);
// }

// void Charging_StateMachine(uint8_t u8DockNo)
// {
//     switch (GET_CHARGING_LIVE_STAGE(u8DockNo))
//     {
//     case INIT_PROTOCOL_REQUEST:
//     {
//         v17017_SendInitReq(u8DockNo);
//     }
//     break;

//     case AUTH_SUCCESS:
//     {
//         v17017_AuthSuccess(u8DockNo);
//     }
//     break;

//     case PARAMETER_VALIDATION:
//     {
//         v17017_ValidateParameters(u8DockNo);
//     }
//     break;

//     case CONNECTION_CONFIRMED:
//     {
//         v17017_ConnectionConfirmed(u8DockNo);
//     }
//     break;

//     case INITIALIZE_STATE:
//     {
//         v17017_InitializeState(u8DockNo);
//     }
//     break;

//     case PRECHARGING_STATE:
//     {
//         v17017_PreChargingState(u8DockNo);
//     }
//     break;

//     case CHARGING_ACTIVE:
//     {
//         v17017_StartCharging(u8DockNo);
//     }
//     break;

//     case SHUTDOWN_STATE:
//     {
//         v17017_Shutdown(u8DockNo);
//     }
//     break;

//     case SESSION_COMPLETE:
//     {
//         v17017_SessionComplete(u8DockNo);
//     }
//     break;

//     default:
//     {
//         /* Safety fallback */
//         SET_CHARGING_LIVE_STAGE(u8DockNo, INIT_PROTOCOL_REQUEST);
//     }
//     break;
//     }
// }
