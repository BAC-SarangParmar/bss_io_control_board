#ifndef COMMON_H
#define COMMON_H

/**
 * @file    common.h
 * @brief   Common definitions, error codes, and function prototypes.
 *
 * @details This header provides common macros, error code enumerations, and
 *          function prototypes used throughout the project. It is designed to
 *          comply with MISRA C standards.
 */

#include <stdint.h>
#include <stddef.h>

/* ************************************************************************** */
/* Section: Macro Definitions                                                 */
/* ************************************************************************** */

/**
 * @brief Task heap depth for the debug server handler.
 */
#define DEBUG_SERVER_HANDLER_HEAP_DEPTH     (512U)

/**
 * @brief Task priority for the debug server handler.
 */
#define DEBUG_SERVER_HANDLER_TASK_PRIORITY  (tskIDLE_PRIORITY + 3U)

/**
 * @brief TCP/IP server port for the debug server.
 */
#define DEBUG_SERVER_PORT_HEV               (9999U)

/* ************************************************************************** */
/* Section: Error Code Definitions                                            */
/* ************************************************************************** */

/**
 * @enum  ERROR_CODE_t
 * @brief Enumerates all possible error codes for CAN (6x), RS485 (2x),
 *        Digital Outputs (24x), Relay Outputs (2x), Digital Inputs (60x),
 *        Analog Inputs (20x), and general errors.
 *
 * Encoding scheme (for easy extension and decoding):
 *   0x1XYY  - CAN Interface errors (X = channel 1-6, YY = error type)
 *   0x2XYY  - RS485 errors (X = channel 1-2, YY = error type)
 *   0x3XYY  - Digital Output errors (X = output 1-24, YY = error type)
 *   0x4XYY  - Relay Output errors (X = relay 1-2, YY = error type)
 *   0x5XYY  - Digital Input errors (X = input 1-60, YY = error type)
 *   0x6XYY  - Analog Input errors (X = input 1-20, YY = error type)
 *   0xF000  - General/Unknown
 */
typedef enum
{
    /* ---------------- CAN (6x) ---------------- */
    /* YY: 01=Bus-Off, 02=Timeout, 03=CRC, 04=ACK, 05=Stuff, 06=Overload */
    ERR_CAN1_BUS_OFF           = 0x1101U,
    ERR_CAN1_TIMEOUT           = 0x1102U,
    ERR_CAN1_CRC               = 0x1103U,
    ERR_CAN1_ACK               = 0x1104U,
    ERR_CAN1_STUFF             = 0x1105U,
    ERR_CAN1_OVERLOAD          = 0x1106U,
    ERR_CAN2_BUS_OFF           = 0x1201U,
    ERR_CAN2_TIMEOUT           = 0x1202U,
    ERR_CAN2_CRC               = 0x1203U,
    ERR_CAN2_ACK               = 0x1204U,
    ERR_CAN2_STUFF             = 0x1205U,
    ERR_CAN2_OVERLOAD          = 0x1206U,
    ERR_CAN3_BUS_OFF           = 0x1301U,
    ERR_CAN3_TIMEOUT           = 0x1302U,
    ERR_CAN3_CRC               = 0x1303U,
    ERR_CAN3_ACK               = 0x1304U,
    ERR_CAN3_STUFF             = 0x1305U,
    ERR_CAN3_OVERLOAD          = 0x1306U,
    ERR_CAN4_BUS_OFF           = 0x1401U,
    ERR_CAN4_TIMEOUT           = 0x1402U,
    ERR_CAN4_CRC               = 0x1403U,
    ERR_CAN4_ACK               = 0x1404U,
    ERR_CAN4_STUFF             = 0x1405U,
    ERR_CAN4_OVERLOAD          = 0x1406U,
    ERR_CAN5_BUS_OFF           = 0x1501U,
    ERR_CAN5_TIMEOUT           = 0x1502U,
    ERR_CAN5_CRC               = 0x1503U,
    ERR_CAN5_ACK               = 0x1504U,
    ERR_CAN5_STUFF             = 0x1505U,
    ERR_CAN5_OVERLOAD          = 0x1506U,
    ERR_CAN6_BUS_OFF           = 0x1601U,
    ERR_CAN6_TIMEOUT           = 0x1602U,
    ERR_CAN6_CRC               = 0x1603U,
    ERR_CAN6_ACK               = 0x1604U,
    ERR_CAN6_STUFF             = 0x1605U,
    ERR_CAN6_OVERLOAD          = 0x1606U,

    /* ---------------- RS485 (2x) with only 3 errors supported ---------------- */
    /* YY: 01=Overrun, 02=No Response, 03=Invalid CRC */
    ERR_RS485_1_OVERRUN        = 0x2101U,
    ERR_RS485_1_NO_RESPONSE    = 0x2102U,
    ERR_RS485_1_INVALID_CRC    = 0x2103U,
    ERR_RS485_2_OVERRUN        = 0x2201U,
    ERR_RS485_2_NO_RESPONSE    = 0x2202U,
    ERR_RS485_2_INVALID_CRC    = 0x2203U,
} ERROR_CODE_t;

#ifdef __cplusplus
extern "C" {
#endif

/* ************************************************************************** */
/* Section: Function Prototypes                                               */
/* ************************************************************************** */

/**
 * @brief Initializes and creates the debug server task for handling communication.
 * @details This function is typically called at system startup to spawn the FreeRTOS
 *          server task responsible for debug communication and packet handling.
 * @return None.
 */
void vCommonTaskHandler(void);

/**
 * @brief Sends the provided error code through the debug server interface.
 * @param code The error code to report.
 * @return None.
 */
void ReportError(ERROR_CODE_t code);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_H */

/* *****************************************************************************
 End of File
 */