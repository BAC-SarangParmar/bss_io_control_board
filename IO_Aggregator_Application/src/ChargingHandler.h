/******************************************************************************
 * File Name   : ChargingHandler.h
 * Description : Portable EV DC Charging State Machine Handler
 *
 * This module implements the complete LEVDC charging process state machine.
 * It is designed to be portable and independent from platform specific
 * drivers. All hardware and protocol interactions are performed through
 * callback interfaces provided by the application layer.
 *
 * Author      : Refactored Architecture
 ******************************************************************************/

#ifndef CHARGING_HANDLER_H
#define CHARGING_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "sessionDBHandler.h"
/*=================================================================
   MACRO DEFINITIONS
   ============================================================ */
/*LEVDC 17017-25 CAN IDs*/
#define LEVDC_CAN_ID_EV_REQUEST            0x500
#define LEVDC_CAN_ID_EV_CHARGING_INFO      0x501
#define LEVDC_CAN_ID_EV_CONTROL_OPTION     0x502
#define LEVDC_CAN_ID_EV_RESERVED_580       0x580
#define LEVDC_CAN_ID_EV_RESERVED_581       0x581
#define LEVDC_CAN_ID_EV_RESERVED_582       0x582
#define LEVDC_CAN_ID_EV_RESERVED_583       0x583

#define LEVDC_CAN_ID_EVSE_STATUS           0x508
#define LEVDC_CAN_ID_EVSE_OUTPUT_INFO      0x509
#define LEVDC_CAN_ID_EVSE_CAPABILITY       0x510
#define LEVDC_CAN_ID_EVSE_CHARGER_ID       0x584
//////////////////////////////////////////////////

/*TVS Propritry CAN IDs*/
/*CHARGER CAN IDs*/
#define TVS_CAN_ID_INFO                0x90
#define TVS_CAN_ID_CHARGE_PROFILE      0x91
#define TVS_CAN_ID_FM_VERSION_INFO     0x92
/*BMS CAN IDs*/
#define TVS_CAN_ID_STATUS              0x100
#define TVS_CAN_ID_PROFILE             0x101
//////////////////////////////////////////////////
/* ============================================================
   LEVDC 17017-25 CAN Message Structures
    - These structures represent the data format for CAN messages
      defined in the LEVDC 17017-25 standard.
   ============================================================ */

/* TX CAN Message Structures */
typedef struct
{
    uint8_t u8ChargingSysError : 1;
    uint8_t u8EvseMalFunctionError : 1;
    uint8_t u8EVCompatible : 1;
    uint8_t u8Res : 5;

    uint8_t u8EvseStopCtrl : 1;
    uint8_t u8EvseStatus : 1;
    uint8_t u8ConLatchStatus : 1;
    uint8_t u8EVSEReadyForCharge : 1;
    uint8_t u8WaitStatebfrCharg : 1;
    uint8_t u8Res1 : 3;

    uint16_t u16RatedOutputVol;
    uint16_t u16AvailOutputCur;
    uint16_t u16ConfDCvolLimit;
} LEVDC_Tx508_t;

typedef struct
{
    uint8_t u8ControlProtocolNum;
    uint8_t u8AvailDCOutputPower;

    uint16_t u16EVSEoutputVoltage;
    uint16_t u16EVSEoutputCurrent;
    uint16_t u16RemainChargeTime;

} LEVDC_Tx509_t;

typedef struct
{
    uint8_t u8Res : 1;
    uint8_t u8EVSEVolatageControlOpt : 1;
} LEVDC_Tx510_t;

typedef struct
{
    uint8_t u8ChargerID1;
    uint8_t u8ChargerID2;
    uint8_t u8ChargerID3;
    uint8_t u8ChargerID4;
    uint8_t u8ChargerID5;
    uint8_t u8ChargerID6;
    uint8_t u8ChargerID7;
    uint8_t u8ChargerID8;
} LEVDC_Tx584_t;

/* RX CAN Message Structures */
typedef struct
{
    uint8_t u8EnergyTransferError : 1;
    uint8_t u8BatteryOverVol : 1;
    uint8_t u8BatteryUnderVol : 1;
    uint8_t u8BatterCurrentDeviError : 1;
    uint8_t u8HighBatteryTemp : 1;
    uint8_t u8BatterVoltageDeviError : 1;
    uint8_t u8Res : 2;
    uint8_t u8EvChargingEnable : 1;
    uint8_t u8EvConStatus : 1;
    uint8_t u8EvChargingPosition : 1;
    uint8_t u8EvChargingStopControl : 1;
    uint8_t u8WaitReqToEngTransfer : 1;
    uint8_t u8DigitalCommToggle : 1;
    uint8_t u8Res1 : 2;
    uint16_t u16ReqDcCurrent;
    uint16_t u16DcOutputVolTarget;
    uint16_t u16DcOutputVoltLimit;
} LEVDC_Rx500_t;

typedef struct
{
    uint8_t u8ControlProtocolNo;
    uint8_t u8ChargingRate;
    uint16_t u16MaxChargingTime;
    uint16_t u16EstimatedChargingTime;
    uint16_t u16Res;
} LEVDC_Rx501_t;

typedef struct
{
    uint8_t u8VoltageControlOption : 1;
} LEVDC_Rx502_t;

typedef struct
{
    uint8_t u8FutureDev;
} LEVDC_Rx580_t, LEVDC_Rx581_t, LEVDC_Rx582_t, LEVDC_Rx583_t;

/* Aggregated Message Frame Info */
typedef struct
{
    LEVDC_Tx508_t LevdcTX_508ID_Info;
    LEVDC_Tx509_t LevdcTX_509ID_Info;
    LEVDC_Tx510_t LevdcTX_510ID_Info;
    LEVDC_Tx584_t LevdcTX_584ID_Info;
    LEVDC_Rx500_t LevdcRX_500ID_Info;
    LEVDC_Rx501_t LevdcRX_501ID_Info;
    LEVDC_Rx502_t LevdcRX_502ID_Info;
    LEVDC_Rx580_t LevdcRX_580ID_Info;
    LEVDC_Rx581_t LevdcRX_581ID_Info;
    LEVDC_Rx582_t LevdcRX_582ID_Info;
    LEVDC_Rx583_t LevdcRX_583ID_Info;
} ChargingMsgFrameInfo_t;
extern ChargingMsgFrameInfo_t Charging_LiveInfo[MAX_DOCKS];
///////////////////////////////////////////////////////////////////
/*==============================================================================
 * TVS CAN Structures  — exact byte/bit layout per DBC spec
 *============================================================================*/

/* 0x90 - Charger Info TX (Charger -> BMS) 100ms */
typedef struct
{
    uint8_t  u8ChargerType;           /* Byte 0     - CHARGER_Type          (res=1, off=0)   */
    uint8_t  u8ACInput;               /* Byte 1     - CHARGER_ACInput       (res=1, off=50)  */
    uint8_t  u8Temperature;           /* Byte 2     - CHARGER_Temperature   (res=1, off=-50) */
    uint8_t  u8Counter;               /* Byte 3     - CHARGER_Counter       (res=1, off=0)   */
    uint8_t  u8ErrorState   : 4;      /* Byte 4[0:3]- CHARGER_ErrorState    (4-bit)          */
    uint8_t  u8Fan          : 1;      /* Byte 4[4]  - CHARGER_Fan           (1-bit)          */
    uint8_t  u8Output       : 1;      /* Byte 4[5]  - CHARGER_Output        (1-bit)          */
    uint8_t  u8Derating     : 1;      /* Byte 4[6]  - Charger_deratting     (1-bit)          */
    uint8_t  u8Res          : 1;      /* Byte 4[7]  - Reserved                               */
    uint8_t  u8Res1[3];               /* Byte 5-7   - Reserved                               */
} TVS_Tx90_Info_t;

/* 0x91 - Charger Charge Profile TX (Charger -> BMS) 100ms */
typedef struct
{
    uint16_t u16ChargingCurrent;      /* Byte 0-1   - CHARGER_Current       (res=0.015625)   */
    uint16_t u16ChargingVoltage;      /* Byte 2-3   - CHARGER_TerminalVoltage(res=0.015625)  */
    uint8_t  u8Res[4];                /* Byte 4-7   - Reserved                               */
} TVS_Tx91_ChargeProfile_t;

/* 0x92 - Charger FM Version Info TX (Charger -> BMS) 1000ms */
typedef struct
{
    uint8_t  u8FWVersionMajor;        /* Byte 0     - FW_version_major                       */
    uint8_t  u8FWVersionMinor;        /* Byte 1     - FW_version_minor                       */
    uint8_t  u8FWVersionIteration;    /* Byte 2     - FW_version_iteration                   */
    uint8_t  u8FWChargerType;         /* Byte 3     - FW_CHARGER_type                        */
    uint8_t  u8FWReleaseDateDD;       /* Byte 4     - FW_Realese_Date_DD                     */
    uint8_t  u8FWReleaseDateMM;       /* Byte 5     - FW_Realese_Date_MM                     */
    uint8_t  u8FWReleaseDateY1Y2;     /* Byte 6     - FW_Realese_Date_Y1Y2                   */
    uint8_t  u8FWReleaseDateY3Y4;     /* Byte 7     - FW_Realese_Date_Y3Y4                   */
} TVS_Tx92_FMVersionInfo_t;

/* 0x100 - BMS Status RX (BMS -> Charger) 100ms */
typedef struct
{
    uint16_t u16BMSCurrent;           /* Byte 0-1   - BMS_Current           (res=0.03125)    */
    uint16_t u16BMSVoltage;           /* Byte 2-3   - BMS_Voltage [13:0]    (res=0.015625)   */
    uint8_t  u8Counter;               /* Byte 4     - BMS_Counter                            */
    uint8_t  u8SOC;                   /* Byte 5     - BMS_SOC                                */
    uint8_t  u8ErrorState;            /* Byte 6     - BMS_ErrorState                         */
    uint8_t  u8Temperature;           /* Byte 7     - BMS_Temperature       (off=-30)        */
} TVS_Rx100_Status_t;

/* 0x101 - BMS Profile RX (BMS -> Charger) 100ms */
typedef struct
{
    uint16_t u16MaxChargeCurrent;     /* Byte 0-1   - BMS_MaxChargeCurrent  (res=0.001)      */
    uint16_t u16MaxChargeVoltage;     /* Byte 2-3   - BMS_MaxChargeVoltage  (res=0.001)      */
    uint16_t u16CutOffChargeCurrent;  /* Byte 4-5   - BMS_CutOffChargeCurrent(res=0.001)     */
    uint16_t u16PreChargeCurrent;     /* Byte 6-7   - BMS_PreChargeCurrent  (res=0.001)      */
} TVS_Rx101_Profile_t;

/*==============================================================================
 * TVS Aggregated Message Frame
 *============================================================================*/
typedef struct
{
    TVS_Tx90_Info_t          TVS_Tx90_ChargerInfo;
    TVS_Tx91_ChargeProfile_t TVS_Tx91_ChargeProfile;
    TVS_Tx92_FMVersionInfo_t TVS_Tx92_FMVersionInfo;
    TVS_Rx100_Status_t       TVS_Rx100_BMSStatus;
    TVS_Rx101_Profile_t      TVS_Rx101_BMSProfile;
} TVS_MsgFrameInfo_t;
///////////////////////////////////////////////////////////////////


    /*Evse Stop Control Bit Description*/
    typedef enum
    {
        EVSE_NOERROR = 0U,
        EVSE_ERROR = 1U
    } ErrorState_e;

    /* EVSE Operational Status Bit Description*/
    typedef enum VoltageControlOption
    {
        NO_VOLTAGE_CONTROL = 0,
        VOLTAGE_CONTROL_ENABLED = 1
    } VoltageControlOption_e;

    /* EVSE Charging State Description*/
    typedef enum ChargingState
    {
        EVSE_STANDBY = 0,
        EVSE_CHARGING = 1
    } ChargingState_e;

    /* EV incompatibility Bit Description*/
    typedef enum EvIncompatibility
    {
        EV_COMPATIBLE = 0U,
        EV_INCOMPATIBLE = 1U
    } EvIncompatibility;

    typedef enum EvSupplyEquipmentState
    {
        EVSE_NOT_READY = 0U,
        EVSE_READY = 1U
    } EvSupplyEquipmentState_e;
    /* EV incompatibility Bit Description*/
    typedef enum
    {
        GUN_UNLATCHED = 0U,
        GUN_LATCHED = 1U
    } EvseGunLatch;

    typedef enum
    {
        EVSE_NOT_READY_FOR_CHARGE = 0U,
        EVSE_READY_FOR_CHARGE = 1U
    } EvseReadyForCharge_e;

    typedef enum
    {
        LED_RED = 1U,
        LED_GREEN = 2U,
        LED_BLUE = 3U,
        LED_NAME_MAX,
    } ledname_e;

    typedef enum
    {
        LED_STATE_STEADY = 1U,
        LED_STATE_BLINK
    } ledStatus_e;

/*==============================================================================
 * TVS Charger Type Definitions (0x90 - CHARGER_Type)
 *============================================================================*/
typedef enum
{
    TVS_CHARGER_TYPE_TVS650W_FRIWO      = 1,
    TVS_CHARGER_TYPE_TVS950W_FRIWO      = 2,
    TVS_CHARGER_TYPE_TVS500W_FRIWO      = 3,
    TVS_CHARGER_TYPE_TVS650W_NAPINO     = 4,
    TVS_CHARGER_TYPE_TVS650W_ANEVOLVE   = 5,
    TVS_CHARGER_TYPE_TVS550W_FRIWO      = 8,
    TVS_CHARGER_TYPE_TVS580W_FRIWO      = 9,
    TVS_CHARGER_TYPE_TVS1350W           = 10,
    TVS_CHARGER_TYPE_TVS1500W_DELTA     = 11,
    TVS_CHARGER_TYPE_3KW_DELTA_OFFBOARD = 22

} TVS_ChargerType_t;


/*==============================================================================
 * TVS Charger Error State (0x90 - CHARGER_ErrorState, Byte 4 bits 0-3)
 *============================================================================*/
typedef enum
{
    TVS_CHARGER_ERR_NONE              = 0,
    TVS_CHARGER_ERR_BATT_SHORT        = 1,
    TVS_CHARGER_ERR_BATT_REVERSE      = 2,
    TVS_CHARGER_ERR_BATT_VOL_RANGE    = 3,
    TVS_CHARGER_ERR_LOST_CAN          = 4,
    TVS_CHARGER_ERR_BATT_REPORT       = 5,
    TVS_CHARGER_ERR_BATT_DISCONNECTED = 6,
    TVS_CHARGER_ERR_OVER_TEMP         = 7,
    TVS_CHARGER_ERR_OVER_VOLTAGE      = 8,
    TVS_CHARGER_ERR_OVER_CURRENT      = 9,
    TVS_CHARGER_ERR_AC_INPUT_RANGE    = 10,
    TVS_CHARGER_ERR_FAN               = 11,
    TVS_CHARGER_ERR_INTERNAL          = 12,
    TVS_CHARGER_ERR_TERMINAL_SHORT    = 13,
    TVS_CHARGER_ERR_OVER_CHARGE_TIME  = 14

} TVS_ChargerError_t;


/*==============================================================================
 * TVS Charger Fan State
 *============================================================================*/
typedef enum
{
    TVS_FAN_OFF = 0,
    TVS_FAN_ON  = 1

} TVS_FanState_t;


/*==============================================================================
 * TVS Charger Output State
 *============================================================================*/
typedef enum
{
    TVS_OUTPUT_OFF = 0,
    TVS_OUTPUT_ON  = 1

} TVS_OutputState_t;


/*==============================================================================
 * TVS Charger Derating State
 *============================================================================*/
typedef enum
{
    TVS_DERATING_NONE   = 0,
    TVS_DERATING_ACTIVE = 1

} TVS_DeratingState_t;

/*==============================================================================
 * TVS BMS Error State (0x100 - BMS_ErrorState)
 *============================================================================*/
typedef enum
{
    TVS_BMS_ERR_NONE          = 0,
    TVS_BMS_ERR_STOP_CHARGING = 1

} TVS_BmsError_t;


/*=================================================================
   FUNCTION PROTOTYPES
    ============================================================ */
/******************************************************************************
 * @brief  Initialize CHARGING Task
 *
 * @return true  - Task created successfully
 * @return false - Task creation failed
//  ******************************************************************************/
bool ChargingTask_Init(void);
#ifdef __cplusplus
}
#endif

#endif