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

#define TONHE_MODULE_START (0xAA)
#define TONHE_MODULE_STOP (0x55)
#define TONHE_MOD_BASE_ID (0x080601A0U)
#pragma pack(push,1)

/* ============================================================
   EV → EVSE MESSAGE (CAN ID 0x500)
   EV Charging Request & Battery Status
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


/**
 *  PM_TONHE Msg TX
 */
typedef struct Tonhe_PM_Tx
{
    uint8_t u8ModuleStartStop;
    uint8_t u8ChargingMode;
    uint16_t u16chargingVoltage;
    uint16_t u16chargingCurrent;
    uint16_t u16StandBy;
} tonhe_pm_Tx_t;
#pragma pack(pop)

extern ChargingMsgFrameInfo_t Charging_LiveInfo[MAX_DOCKS];
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

/*=================================================================
   FUNCTION PROTOTYPES
    ============================================================ */

#ifdef __cplusplus
}
#endif

#endif