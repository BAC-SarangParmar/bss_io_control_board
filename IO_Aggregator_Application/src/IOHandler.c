/* ************************************************************************** */
/** @file IOHandler.c
 *  @brief I/O status management and flash config storage for IO Aggregator board.
 *  @company BACANCY SYSTEMS PVT.LTD.
 *  @details
 *    Implements digital/analog pin management, flash I/O state storage,
 *    and port configurations for the IO Aggregator board.
 *    MISRA C:2023 compliant coding style and documentation.
 */
/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "definitions.h"                // SYS function prototypes
#include "configuration.h"
#include "device.h"
#include "tcpip_manager_control.h"
#include "library/tcpip/tcpip_helpers.h"
#include <math.h>
#include "sessionDBHandler.h"
/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Macro Definitions                                                 */
/* ************************************************************************** */
#define NUM_DIGITAL_PINS            60U         /**< Total number of digital input pins */
#define RELAY_COUNT                 6U          /**< Total number of relays */
#define DIGITAL_OUTPUT              24U         /**< Total number of digital output pins */

#define TCA9539_REG_OUTPUT_PORT0    0x06U       /**< IO Expander Port 0 output register address */
#define TCA9539_REG_OUTPUT_PORT1    0x07U       /**< IO Expander Port 1 output register address */

#define FLASH_MAGIC                 0xDEADBEEFU /**< Magic number to verify flash data validity */
#define FLASH_ADDRESS               0x0C0F0000U /**< Start address in flash for storing config */

#define ADC_TIMEOUT                 (10000U)    /**< Timeout for ADC operations in microseconds */

#define TCP_MIN_FRAME_SIZE       12U          /**< Minimum valid TCP frame size (header only) */
//---------------- ADC and Circuit Parameters ----------------
#if PT100_Sensor
static const float VREF_L          = 4.9500F;
static const float ADC_MAX_VAL     = 4095.0F;
static const float VREF            = 3.290F;
static const float GAIN            = 10.9F;
static const float RREF            = 3300.0F;
static const float TEMP_COEFF_PT100= 0.00385F;
#endif

#if NTC_Sensor
static const float COEFF_A         = 0.0011279F;
static const float COEFF_B         = 0.00023429F;
static const float COEFF_C         = 0.000000087298F;
static const uint32_t REF_RESISTOR = 4700U;
#endif

#define MAX_BATCH_SIZE              16
#define TUPLE_SIZE                  6
#define HEADER_SIZE                 4
#define CRC_SIZE                    2
#define MESSAGE_MAX_SIZE            (HEADER_SIZE + (MAX_BATCH_SIZE * TUPLE_SIZE) + CRC_SIZE)

#define MSG_TYPE_REQUEST            0x01
#define MSG_TYPE_RESPONSE           0x02

volatile bool Two_Wheeler_IO_Aggregator1 = false;             /**< Flag for board version selection */
volatile bool Board_Selection = false;                        /**< General board configuration selector */

// Command types
const char msgSubTypeDigitalRead  = 0x00;
const char msgSubTypeDigitalWrite = 0x01;
const char msgSubTypeAnalogRead   = 0x03;

// Error types
const char errorNoError           = 0x00;
const char errorInvalidMsg        = 0x01;
const char errorInvalidCrc        = 0x02;
const char errorInvalidTuples     = 0x04;
const char errorInvalidPort       = 0x08;
const char errorInvalidValue      = 0x10;
const char errorSafety            = 0x20;
const char errorInternalFailure   = 0x40;
const char errorSecurityIntrusion = 0x80;

// Incoming Command for HEV
char inbMsgSubType = 0x00;
int inbNumTuples = 0; // Allow only  max of 1 for now
short inbPortNo = -1;
int inbPortVal = -1;
short inbCrc = -1;
char inbPacketBuf[256] = { 0x00 };
int inbPacketSz = 0;

// Outgoing Command for HEV
char outbErrorCode = errorNoError;
char outbMsgSubType = 0x00;
int outbNumTuples = 0; // Allow only  max of 1 for now
short outbPortNo = -1;
int outbPortVal = -1;
short outbCrc = -1;
char outbPacketBuf[256] = { 0x00 };
int outbPacketSz = 0;
short outbRemotePort = 0;

uint32_t doStatus = 0U;               /**< Current digital output status */
uint16_t relayStatus = 0U;            /**< Current relay output status */
uint16_t serialnum = 0U;              /**< Device serial number stored in flash */

// Default output states
uint32_t digitalOutputs;     // All outputs off
uint16_t relayOutputs;           // All relays off
uint16_t serialnum;
// Function prototypes
void saveOutputsToFlash(uint32_t digital, uint16_t relay, uint16_t serialnum);
void readOutputsFromFlash(void);
static TCP_SOCKET sIOServerSocket = INVALID_SOCKET;
char IOreceiveBuffer[128] = {0};

//I2C flag and buffer
uint8_t i2cTxBuf[2];
uint8_t i2cRxBuf[2];
bool i2cTransferDone = false;

void ReadAllAnalogInputPins();
bool IOExpander1_Configure_Ports(void);
bool IOExpander2_Configure_Ports(void);
void ChargingControl(uint8_t u8DockNo, uint8_t action);
uint32_t Analog_Input_Get(uint8_t channel);
void HandleGPIOCommand(void);
void HandleAnalogRead(void);
void HandleChargingCommand(uint8_t u8DockNo);
void HandleBootloaderCommand(void);
void SendResponsePayload(uint8_t *payload, uint8_t len);
volatile  uint16_t currentTempAIQueueBuffer[NUM_TEMPERATURE_ANALOG_PINS];
volatile  uint16_t currentAIQueueBuffer[NUM_ANALOG_PINS];
char messageAnalog[MESSAGE_BUFFER_SIZE];
uint32_t total_length = 0U;
volatile  uint8_t currentDIQueueBuffer[NUM_DIGITAL_PINS];
uint16_t CalculateCRC(uint8_t *data, uint16_t length);
static void SetStaticIPAddress(const char* ipStr);
volatile bool bIOOperation = false;
uint32_t *ramStart = (uint32_t *)BTL_TRIGGER_RAM_START;
TCP_RequestFrame_t Reqframe_St;
const Board_gpio_st Gpio_conf = {
    .AC_Relay_Pin = {61,62,63},
    .DC_Relay_Pin = {64,65,66},
    .Solenoid_PinHi = {67,68,69},
    .Solenoid_PinLo = {70,71,72},
    .R_LED_Pin = {73,74,75},
    .G_LED_Pin = {76,77,78},
    .B_LED_Pin = {79,80,81},
    .Dock_Fan_Pin = {86,87,88},
    .Compartment_Fan_Pin = {85},
    .DoorLock_Pin = {1,2,3},
    .SolenoidLock_Pin = {4,5,6},
    .EStop_Pin = {7}};
/*  A brief description of a section can be given directly below the section
    banner.
 */
void printSavedFlashData(void)
{
    flash_data_t savedData;
    FCW_Read((uint32_t *)&savedData, sizeof(savedData), FLASH_ADDRESS);

    if (savedData.magic != FLASH_MAGIC) {
        SYS_CONSOLE_PRINT("Flash is empty or invalid. Magic: 0x%08X\n", savedData.magic);
        return;
    }

    SYS_CONSOLE_PRINT("---- SAVED FLASH DATA ----\n");
    SYS_CONSOLE_PRINT("Digital Outputs: 0x%08X\n", savedData.digitalOutputs);
    SYS_CONSOLE_PRINT("Relay Outputs  : 0x%04X\n", savedData.relayOutputs);

    for (int i = 0; i < 6; i++) {
        SYS_CONSOLE_PRINT("CAN%d: Port=%u, Baud=%lu\n", 
            i + 1, savedData.canPorts[i], savedData.canBaudRates[i]);
    }

    for (int i = 0; i < 2; i++) {
        SYS_CONSOLE_PRINT("RS485%d: Port=%u, Baud=%lu, DataBits=%u, Parity=%c, StopBits=%u\n",
            i + 1,
            savedData.rs485Ports[i],
            savedData.rs485Config[i].baudRate,
            savedData.rs485Config[i].dataBits,
            savedData.rs485Config[i].parity,
            savedData.rs485Config[i].stopBits);
    }

    SYS_CONSOLE_PRINT("--------------------------\n");
}

/*******************************************************************************
 *  \brief     Saves digital and relay output states to flash memory.
 *  \param[in] digital - 32-bit digital output data.
 *  \param[in] relay   - 16-bit relay output data.
 *  \details   This function writes the provided output states into non-volatile  
 *             flash memory after ensuring data cache coherence.
 *  \return    None (void function).
 *******************************************************************************/
void saveOutputsToFlash(uint32_t digital, uint16_t relay, uint16_t serialnum)
{
    writeData.magic = FLASH_MAGIC;
    writeData.digitalOutputs = digital;
    writeData.relayOutputs = relay;
    writeData.serialNumber = serialnum;
    
    uint8_t writeBuffer[sizeof(flash_data_t)];
    memcpy(writeBuffer, &writeData, sizeof(flash_data_t));

    DCACHE_CLEAN_BY_ADDR((uint32_t *)writeBuffer, sizeof(writeBuffer));
    FCW_PageErase(FLASH_ADDRESS);
    while (FCW_IsBusy());

    FCW_RowWrite((uint32_t *)writeBuffer, FLASH_ADDRESS);
    while (FCW_IsBusy());
    SYS_CONSOLE_MESSAGE("Configuration saved to flash.\r\n");
}
void PrintLoadedConfig(void)
{
    SYS_CONSOLE_PRINT("=== Loaded Configuration ===\r\n");

    for (int i = 0; i < 6; i++)
    {
        SYS_CONSOLE_PRINT("CAN Port %d -> TCP Port: %d\r\n", i + 1, writeData.canPorts[i]);
    }

    for (int i = 0; i < 2; i++)
    {
        SYS_CONSOLE_PRINT("RS485 Port %d -> TCP Port: %d\r\n", i + 1, writeData.rs485Ports[i]);
    }

    for (int i = 0; i < 6; i++)
    {
        SYS_CONSOLE_PRINT("CAN Speed_%d: %d\r\n", i + 1, writeData.canBaudRates[i]);
    }
    for (int i = 0; i < 2; i++)
    {
        SYS_CONSOLE_PRINT("RS485_%d Config -> Baud: %lu, DataBits: %u, Parity: %c, StopBits: %u\r\n",
            i + 1,
            writeData.rs485Config[i].baudRate,
            writeData.rs485Config[i].dataBits,
            writeData.rs485Config[i].parity,
            writeData.rs485Config[i].stopBits);
    }
}

/*******************************************************************************
 *  \brief     Restore saved output states from flash memory.
 *
 *  \details   This function reads a configuration structure from non-volatile 
 *             flash memory, validates the integrity of the data using a magic 
 *             number and default validity checks, and restores the relay and 
 *             digital output states accordingly. If the flash data is invalid 
 *             or uninitialized, it initializes default configuration values, 
 *             applies them, and writes them to flash memory.
 *
 *             Additionally, it ensures all CAN server sockets are closed 
 *             before reinitializing with the restored or default configuration.
 *
 *  \note      This function should be called during system startup to restore 
 *             prior output states or initialize defaults.
 *
 *  \return    void
 *******************************************************************************/
void readOutputsFromFlash(void)
{
    uint8_t readBuffer[sizeof(flash_data_t)];
    FCW_Read((uint32_t *)readBuffer, sizeof(flash_data_t), FLASH_ADDRESS);
    flash_data_t *pDataVerify = (flash_data_t *)readBuffer;

    bool flashInvalid = false;

    // Check if flash completely empty (0x00 or 0xFF)
    bool flashCompletelyEmpty = true;
    for (size_t i = 0; i < sizeof(flash_data_t); i++)
    {
        if (readBuffer[i] != 0x00 && readBuffer[i] != 0xFF)
        {
            flashCompletelyEmpty = false;
            break;
        }
    }

    // Basic flash checks
    if (pDataVerify->magic != FLASH_MAGIC || flashCompletelyEmpty)
    {
        flashInvalid = true;
    }
    else
    {
        for (int i = 0; i < 6; i++)
        {
            if (pDataVerify->canPorts[i] == 0 || pDataVerify->canBaudRates[i] == 0)
            {
                flashInvalid = true;
                break;
            }
        }

        for (int i = 0; i < 2 && !flashInvalid; i++)
        {
            if (pDataVerify->rs485Ports[i] == 0 ||
                pDataVerify->rs485Config[i].baudRate == 0 ||
                pDataVerify->rs485Config[i].dataBits == 0 ||
                pDataVerify->rs485Config[i].stopBits == 0)
            {
                flashInvalid = true;
                break;
            }
        }
    }

    if (!flashInvalid)
    {
        memcpy(&writeData, pDataVerify, sizeof(flash_data_t));
        SYS_CONSOLE_PRINT("Restored Outputs - Digital: %08X, Relay: %04X\r\n", 
                          writeData.digitalOutputs, writeData.relayOutputs);
        SYS_CONSOLE_MESSAGE("Configuration loaded from flash.\r\n");
#if HEV_IO_Aggregator
        // Restore RS485 UART configuration
        for (int uartIndex = 0; uartIndex < 2; uartIndex++) {
            int baud = writeData.rs485Config[uartIndex].baudRate;
            int dataBits = writeData.rs485Config[uartIndex].dataBits;
            int stopBits = writeData.rs485Config[uartIndex].stopBits;
            int parityEncoded = writeData.rs485Config[uartIndex].parity;

            USART_DATA dataWidth;
            switch (dataBits) {
                case 5: dataWidth = USART_DATA_5_BIT; break;
                case 6: dataWidth = USART_DATA_6_BIT; break;
                case 7: dataWidth = USART_DATA_7_BIT; break;
                case 8: dataWidth = USART_DATA_8_BIT; break;
                default:
                    SYS_CONSOLE_PRINT("? Invalid Data Bits (%d) for RS485_%d\r\n", dataBits, uartIndex + 1);
                    continue;
            }

            USART_PARITY parity;
            switch (parityEncoded) {
                case 0: parity = USART_PARITY_NONE; break;
                case 1: parity = USART_PARITY_ODD; break;
                case 2: parity = USART_PARITY_EVEN; break;
                default:
                    SYS_CONSOLE_PRINT("? Invalid Parity (%d) for RS485_%d\r\n", parityEncoded, uartIndex + 1);
                    continue;
            }

            USART_STOP stop = (stopBits == 1) ? USART_STOP_0_BIT : USART_STOP_1_BIT;

            USART_SERIAL_SETUP setup = {
                .baudRate = baud,
                .dataWidth = dataWidth,
                .parity = parity,
                .stopBits = stop
            };

            if (uartIndex == 0) {
                SERCOM8_USART_ReadAbort();
                if (!SERCOM8_USART_SerialSetup(&setup, 0)) {
                    SYS_CONSOLE_PRINT("? RS485_1 setup from flash failed: Baud=%d Data=%d Parity=%d Stop=%d\r\n",
                                      baud, dataBits, parityEncoded, stopBits);
                } else {
                    SYS_CONSOLE_PRINT("RS485_1 setup from flash done: Baud=%d Data=%d Parity=%d Stop=%d\r\n",
                                      baud, dataBits, parityEncoded, stopBits);
                    uint8_t dummy;
                    bool result = SERCOM8_USART_Read(&dummy, 1);
                    SYS_CONSOLE_PRINT("RS485_1 RX resume: %s\r\n", result ? "OK" : "FAILED");
                }
            } else if (uartIndex == 1) {
                SERCOM9_USART_ReadAbort();
                if (!SERCOM9_USART_SerialSetup(&setup, 0)) {
                    SYS_CONSOLE_PRINT("? RS485_2 setup from flash failed: Baud=%d Data=%d Parity=%d Stop=%d\r\n",
                                      baud, dataBits, parityEncoded, stopBits);
                } else {
                    SYS_CONSOLE_PRINT("RS485_2 setup from flash done: Baud=%d Data=%d Parity=%d Stop=%d\r\n",
                                      baud, dataBits, parityEncoded, stopBits);
                    uint8_t dummy;
                    bool result = SERCOM9_USART_Read(&dummy, 1);
                    SYS_CONSOLE_PRINT("RS485_2 RX resume: %s\r\n", result ? "OK" : "FAILED");
                }
            }
        }
#endif    
        serialnum = writeData.serialNumber;
        // Restore relay outputs
        uint16_t relayStatus = writeData.relayOutputs;
        for (int i = 0; i < RELAY_COUNT; i++)
        {
            if (relayStatus & (1 << i))
            {
                switch (i)
                {
                    case 0: efuse1_in_Set(); break;
                    case 1: efuse2_in_Set(); break;
                    case 2: efuse3_in_Set(); break;
                    case 3: efuse4_in_Set(); break;
                    case 4: Relay_Output_1_Set(); break;
                    case 5: Relay_Output_2_Set(); break;
                }
            }
            else
            {
                switch (i)
                {
                    case 0: efuse1_in_Clear(); break;
                    case 1: efuse2_in_Clear(); break;
                    case 2: efuse3_in_Clear(); break;
                    case 3: efuse4_in_Clear(); break;                    
                    case 4: Relay_Output_1_Clear(); break;
                    case 5: Relay_Output_2_Clear(); break;
                }
            }
        }

        // Restore digital outputs
        uint32_t doStatus = writeData.digitalOutputs;
        for (int i = 0; i < DIGITAL_OUTPUT; i++)
        {
            if (doStatus & (1 << i))
            {
                switch (i)
                {
                    case 0: Digital_Output_1_Set(); break;
                    case 1: Digital_Output_2_Set(); break;
                    case 2: Digital_Output_3_Set(); break;
                    case 3: Digital_Output_4_Set(); break;
                    case 4: Digital_Output_5_Set(); break;
                    case 5: Digital_Output_6_Set(); break;
                    case 6: Digital_Output_7_Set(); break;
                    case 7: Digital_Output_8_Set(); break;
                    case 8: Digital_Output_9_Set(); break;
                    case 9: Digital_Output_10_Set(); break;
                    case 10: Digital_Output_11_Set(); break;
                    case 11: Digital_Output_12_Set(); break;
                    case 12: Digital_Output_13_Set(); break;
                    case 13: Digital_Output_14_Set(); break;
                    case 14: Digital_Output_15_Set(); break;
                    case 15: Digital_Output_16_Set(); break;
                    case 16: Digital_Output_17_Set(); break;
                    case 17: Digital_Output_18_Set(); break;
                    case 18: Digital_Output_19_Set(); break;
                    case 19: Digital_Output_20_Set(); break;
                    case 20: Digital_Output_21_Set(); break;
                    case 21: Digital_Output_22_Set(); break;
                    case 22: Digital_Output_23_Set(); break;
                    case 23: Digital_Output_24_Set(); break;
                    
                }
            }
            else
            {
                switch (i)
                {
                    case 0: Digital_Output_1_Clear(); break;
                    case 1: Digital_Output_2_Clear(); break;
                    case 2: Digital_Output_3_Clear(); break;
                    case 3: Digital_Output_4_Clear(); break;
                    case 4: Digital_Output_5_Clear(); break;
                    case 5: Digital_Output_6_Clear(); break;
                    case 6: Digital_Output_7_Clear(); break;
                    case 7: Digital_Output_8_Clear(); break;
                    case 8: Digital_Output_9_Clear(); break;
                    case 9: Digital_Output_10_Clear(); break;
                    case 10: Digital_Output_11_Clear(); break;
                    case 11: Digital_Output_12_Clear(); break;
                    case 12: Digital_Output_13_Clear(); break;
                    case 13: Digital_Output_14_Clear(); break;
                    case 14: Digital_Output_15_Clear(); break;
                    case 15: Digital_Output_16_Clear(); break;
                    case 16: Digital_Output_17_Clear(); break;
                    case 17: Digital_Output_18_Clear(); break;
                    case 18: Digital_Output_19_Clear(); break;
                    case 19: Digital_Output_20_Clear(); break;
                    case 20: Digital_Output_21_Clear(); break;
                    case 21: Digital_Output_22_Clear(); break;
                    case 22: Digital_Output_23_Clear(); break;
                    case 23: Digital_Output_24_Clear(); break;                    
                }
            }
        }

        SYS_CONSOLE_PRINT("Restored Relay and Digital Outputs from Flash\n");
    }
    else
    {
        SYS_CONSOLE_MESSAGE("Flash data invalid or empty. Loading defaults...\r\n");

        memset(&writeData, 0, sizeof(flash_data_t));
        writeData.magic = FLASH_MAGIC;
#if HEV_IO_Aggregator
        // Default values
        uint16_t canPorts[6] = {8120, 8121, 8122, 8123, 8124, 8125};
        uint16_t rs485Ports[2] = {9114, 9115};
        uint32_t canBaudRates[6] = {500000, 500000, 500000, 500000, 500000, 500000};
        uint32_t uartBaud[2] = {9600, 9600};
        uint8_t dataBits[2] = {8, 8};
        char parity[2] = {'N', 'N'};
        uint8_t stopBits[2] = {1, 1};

        memcpy(writeData.canPorts, canPorts, sizeof(canPorts));
        memcpy(writeData.rs485Ports, rs485Ports, sizeof(rs485Ports));
        memcpy(writeData.canBaudRates, canBaudRates, sizeof(canBaudRates));

        for (int i = 0; i < 2; i++)
        {
            writeData.rs485Config[i].baudRate = uartBaud[i];
            writeData.rs485Config[i].dataBits = dataBits[i];
            writeData.rs485Config[i].parity = parity[i];
            writeData.rs485Config[i].stopBits = stopBits[i];
        }
#endif
        // Save defaults to flash
        uint8_t writeBuffer[sizeof(flash_data_t)];
        memcpy(writeBuffer, &writeData, sizeof(flash_data_t));
        DCACHE_CLEAN_BY_ADDR((uint32_t *)writeBuffer, sizeof(writeBuffer));

        FCW_PageErase(FLASH_ADDRESS);
        while (FCW_IsBusy());

        FCW_RowWrite((uint32_t *)writeBuffer, FLASH_ADDRESS);
        while (FCW_IsBusy());

        SYS_CONSOLE_MESSAGE("Default configuration saved to flash.\r\n");
    }  
}


/*******************************************************************************
 *  \brief     Stores the current relay and digital output states.
 *  \details   Reads the output states, encodes them into bit fields, and saves 
 *             them into flash memory.
 *  \return    None (void function).
 *******************************************************************************/
void StoreRelay_DigitalOutputsFrame() {
    relayStatus |= (efuse1_in_Get() << 0); 
    relayStatus |= (efuse2_in_Get() << 1); 
    relayStatus |= (efuse3_in_Get() << 2); 
    relayStatus |= (efuse4_in_Get() << 3);
    relayStatus |= (Relay_Output_1_Get() << 4);
    relayStatus |= (Relay_Output_2_Get() << 5);
     
    doStatus |= (Digital_Output_1_Get() << 0);
    doStatus |= (Digital_Output_2_Get() << 1);
    doStatus |= (Digital_Output_3_Get() << 2);
    doStatus |= (Digital_Output_4_Get() << 3);
    doStatus |= (Digital_Output_5_Get() << 4);
    doStatus |= (Digital_Output_6_Get() << 5);
    doStatus |= (Digital_Output_7_Get() << 6);
    doStatus |= (Digital_Output_8_Get() << 7);
    doStatus |= (Digital_Output_9_Get() << 8);
    doStatus |= (Digital_Output_10_Get() << 9);
    doStatus |= (Digital_Output_11_Get() << 10);
    doStatus |= (Digital_Output_12_Get() << 11);
    doStatus |= (Digital_Output_13_Get() << 12);
    doStatus |= (Digital_Output_14_Get() << 13);
    doStatus |= (Digital_Output_15_Get() << 14);
    doStatus |= (Digital_Output_16_Get() << 15);
    doStatus |= (Digital_Output_17_Get() << 16);
    doStatus |= (Digital_Output_18_Get() << 17);
    doStatus |= (Digital_Output_19_Get() << 18);
    doStatus |= (Digital_Output_20_Get() << 19);
    doStatus |= (Digital_Output_21_Get() << 20);
    doStatus |= (Digital_Output_22_Get() << 21);
    doStatus |= (Digital_Output_23_Get() << 22);
    doStatus |= (Digital_Output_24_Get() << 23);
    
    /* Save the extracted output states to flash */
    saveOutputsToFlash(doStatus, relayStatus,serialnum);
//    printSavedFlashData();
    SYS_CONSOLE_PRINT("Saved Relay and Digital Outputs to Flash\n");
}

// Serial number mapping is now sequential as per new ranges
// 1-60: Digital inputs
// 61-84: Digital outputs
// 85-90: Relay inputs/EFuse
// 91-110: Analog inputs

uint16_t GetPortFromSerialNumber(uint8_t serialNumber) {
    // Digital Inputs: 1-60
    if (serialNumber >= 1U && serialNumber <= 60U) {
        return serialNumber;
    }
    // Digital Outputs: 61-84
    if (serialNumber >= 61U && serialNumber <= 84U) {
        return serialNumber;
    }
    // Relay Inputs: 85-90
    if (serialNumber >= 85U && serialNumber <= 90U) {
        return serialNumber;
    }
    // Digital Inputs: 91-110
    if (serialNumber >= 91U && serialNumber <= 110U) {
        return serialNumber;
    }

    // If serial number is outside valid range, return invalid port
    return 0xFFFFU;
}

/**
 * @brief Reads analog input values from ADC channels and stores them in a queue buffer.
 *
 * This function iterates through all available analog input channels, waits for the ADC conversion
 * to complete with a timeout mechanism, and then converts the raw ADC value into a scaled voltage.
 * The resulting value is stored in the currentAIQueueBuffer.
 *
 * MISRA C Compliance:
 * - Ensures explicit type handling to avoid implicit promotions.
 * - Uses explicit comparison instead of boolean negation.
 * - Implements timeout protection to prevent infinite loops.
 * - Ensures safe type conversions with explicit casting.
 *
 * @note ADC_VREF should be defined as a valid floating-point constant.
 */

#define SAMPLE_LEN 50   // Number of samples for moving average

typedef struct {
    uint16_t buffer[SAMPLE_LEN];
    uint32_t sum;
    uint8_t index;
} MovingAverage_t;

MovingAverage_t ma[20];  // One MA per channel

void MovingAverage_Init_All(void)
{
    for (int i = 0; i < 20; i++) {
        ma[i].sum = 0;
        ma[i].index = 0;
        for (int j = 0; j < SAMPLE_LEN; j++) {
            ma[i].buffer[j] = 0;
        }
    }
}

uint16_t MovingAverage_Update(uint8_t channel, uint16_t newValue)
{
    ma[channel].sum -= ma[channel].buffer[ma[channel].index];  
    ma[channel].buffer[ma[channel].index] = newValue;          
    ma[channel].sum += newValue;                               
    ma[channel].index = (ma[channel].index + 1) % SAMPLE_LEN;  

    return (uint16_t)(ma[channel].sum / SAMPLE_LEN);
}

static inline float TempCalcPT100(float in_volt)
{
    float resistance = (RREF * in_volt) / ((GAIN * VREF_L) - in_volt);
    float temperature = ((resistance / 100.0F) - 1.0F) / TEMP_COEFF_PT100;
    return temperature;
}

static inline float TempCalcNTC(float in_volt)
{
    float resistance = REF_RESISTOR * (in_volt / (VREF - in_volt));
    float lnRes = logf(resistance);
    float invTemp = COEFF_A + (COEFF_B * lnRes) + (COEFF_C * (lnRes * lnRes * lnRes));
    float temperature = (1.0F / invTemp) - 273.15F;
    return temperature;
}

/**
 * @brief Checks for changes in analog input values and sends updates over TCP/IP.
 *
 * This function compares the current analog input buffer with the previous values.
 * If changes are detected, it batches up to 16 changes in a message and transmits
 * them to an IO socket with CRC validation.
 *
 * MISRA C Compliance:
 * - Ensures explicit type handling to avoid implicit promotions.
 * - Uses explicit comparisons instead of boolean negation.
 * - Avoids magic numbers by using named constants.
 * - Ensures buffer boundaries are respected.
 * - Adds explicit type casting to prevent implicit conversions.
 *
 * @note Requires valid `sIOServerSocket`, `CalculateCRC()`, and `TCPIP_TCP_ArrayPut()`.
 */
void ReadAllAnalogInputPins(void)
{
    uint16_t adc_avg; /* Local variable for averaged ADC values */
    char *pWrite = messageAnalog; /* Safe buffer pointer */

    volatile uint32_t all_adc_data[ALL_ANALOG_PINS]; /* Store raw ADC values */
    static const ADC_CORE_NUM adc_cores[ALL_ANALOG_PINS] = {
        ADC_CORE_NUM2, ADC_CORE_NUM2, ADC_CORE_NUM0, ADC_CORE_NUM0,
        ADC_CORE_NUM0, ADC_CORE_NUM1, ADC_CORE_NUM0, ADC_CORE_NUM1,
        ADC_CORE_NUM1, ADC_CORE_NUM0, ADC_CORE_NUM1, ADC_CORE_NUM0,
        ADC_CORE_NUM3, ADC_CORE_NUM3, ADC_CORE_NUM2, ADC_CORE_NUM2,
        ADC_CORE_NUM3, ADC_CORE_NUM3, ADC_CORE_NUM3, ADC_CORE_NUM3
    };

    static const ADC_CHANNEL_NUM adc_channels[ALL_ANALOG_PINS] = {
        ADC_CH5, ADC_CH4, ADC_CH5, ADC_CH2,
        ADC_CH0, ADC_CH4, ADC_CH7, ADC_CH0,
        ADC_CH5, ADC_CH4, ADC_CH2, ADC_CH1,
        ADC_CH0, ADC_CH1, ADC_CH3, ADC_CH2,
        ADC_CH4, ADC_CH5, ADC_CH2, ADC_CH3
    };

    /* ---- Start ADC Conversion ---- */
    RTC_Timer32Start();             /* MISRA: Explicit call to start RTC timer */
    ADC_GlobalEdgeConversionStart(); /* Start global ADC conversion */

    while (!ADC_CORE_INT_EOSRDY)
    {
        /* Busy wait until conversion complete - acceptable in this context */
    }

    const float inv_adc_max = 1.0F / ADC_MAX_VAL; /* Precompute inverse for efficiency */

    for (uint8_t i = 0U; i < ALL_ANALOG_PINS; i++)
    {
        /* Get raw ADC data */
        all_adc_data[i] = ADC_ResultGet(adc_cores[i], adc_channels[i]);

        /* Apply moving average to smooth signal */
        adc_avg = MovingAverage_Update(i, (uint16_t)all_adc_data[i]);

        /* Convert ADC to voltage */
        float adc_voltage = (adc_avg * VREF) * inv_adc_max;
        int32_t len = 0;

        if (i < 16U)   /* Temperature channels */
        {
            float temperature;

        #if defined(PT100_Sensor)
            temperature = TempCalcPT100(adc_voltage);
        #elif defined(NTC_Sensor)
            temperature = TempCalcNTC(adc_voltage);
        #else
            temperature = (float)adc_avg; /* Default raw fallback */
        #endif

            /* Convert temperature to 0.01 �C fixed-point format */
            uint16_t tempValue = (uint16_t)(temperature * 100.0F);
            if (tempValue > 65535U) { tempValue = 65535U; } /* Clamp to max */
            currentTempAIQueueBuffer[i] = tempValue;
        }
        else /* Non-temperature analog inputs */
        {
            /* Scale to 0.01 V units */
            uint16_t value16 = (uint16_t)(adc_voltage * 100.0F);
            if (value16 > 65535U) { value16 = 65535U; } /* Clamp */
            currentAIQueueBuffer[i - 16U] = value16;
        }
    }
}

/**
 * @brief Sets a static IP address for the GMAC network interface.
 *
 * This function disables DHCP and assigns a static IP address and subnet mask
 * to the GMAC interface. It validates the provided IP address format before
 * attempting to configure the network.
 *
 * MISRA C Compliance:
 * - Ensures proper type handling to avoid implicit promotions.
 * - Uses explicit comparisons instead of boolean negation.
 * - Avoids magic numbers by using named constants.
 * - Ensures proper null-checking and input validation.
 *
 * @param[in] ipStr Pointer to the string containing the IP address.
 *
 * @note This function assumes `TCPIP_STACK_NetHandleGet()`, `TCPIP_DHCP_Disable()`,
 * and `TCPIP_STACK_NetAddressSet()` are available in the system.
 */
static void SetStaticIPAddress(const char* ipStr)
{
    TCPIP_NET_HANDLE netH;
    IPV4_ADDR ipAddress;
    IPV4_ADDR subnetMask;
    bool status = false;

    /* Validate input pointer */
    if (ipStr == NULL) 
    {
        SYS_CONSOLE_PRINT("Error: NULL IP address string\r\n");
        return;
    }

    /* Get network interface handle for GMAC */
    netH = TCPIP_STACK_NetHandleGet("GMAC");

    if (netH == 0U) /* MISRA C: Avoid implicit comparison */
    {
        SYS_CONSOLE_PRINT("Error: Failed to get network handle\r\n");
        return;
    }

    /* Disable DHCP before setting static IP */
    (void) TCPIP_DHCP_Disable(netH); /* MISRA C: Explicitly ignore return value */

    /* Convert selected IP string to IPV4_ADDR structure */
    if (TCPIP_Helper_StringToIPAddress(ipStr, &ipAddress) == false)
    {
        SYS_CONSOLE_PRINT("Error: Invalid IP address format\r\n");
        return;
    }

    /* Define a constant subnet mask */
    static const char* SUBNET_MASK_STR = "255.255.0.0";

    /* Convert subnet mask string to IPV4_ADDR structure */
    if (TCPIP_Helper_StringToIPAddress(SUBNET_MASK_STR, &subnetMask) == false)
    {
        SYS_CONSOLE_PRINT("Error: Invalid subnet mask format\r\n");
        return;
    }

    /* Set Static IP Address */
    status = TCPIP_STACK_NetAddressSet(netH, &ipAddress, &subnetMask, true);

    if (status == true)
    {
        SYS_CONSOLE_PRINT("Static IP set successfully: %s\r\n", ipStr);
    }
    else
    {
        SYS_CONSOLE_PRINT("Error: Failed to set static IP\r\n");
    }
}


/**
 * @brief Reads digital input values and updates the input buffer.
 *
 * This function reads the status of 40 direct digital inputs and 8 expander inputs.
 * It also sets IP addresses based on board selection conditions.
 *
 * MISRA C Compliance:
 * - Ensures proper type handling to avoid implicit promotions.
 * - Uses explicit comparisons instead of boolean negation.
 * - Reduces redundant function calls with a loop-based approach.
 * - Avoids magic numbers by defining named constants.
 * - Ensures proper null-checking and input validation.
 */
void readDigitalInputs() {
    uint8_t expanderData0 = TCA9539_ReadRegister(INPUT_PORT_REG);
    uint8_t expanderData1 = TCA9539_ReadRegister(INPUT_PORT_REG_2);
    
    // Set the digital input data (first 40 from direct digital pins, last 8 from expander)
    currentDIQueueBuffer[0] = (Digital_Input_1_Get()) ? 1 : 0;
    currentDIQueueBuffer[1] = (Digital_Input_2_Get()) ? 1 : 0;
    currentDIQueueBuffer[2] = (Digital_Input_3_Get()) ? 1 : 0;
    currentDIQueueBuffer[3] = (Digital_Input_4_Get()) ? 1 : 0;
    currentDIQueueBuffer[4] = (Digital_Input_5_Get()) ? 1 : 0;
    currentDIQueueBuffer[5] = (Digital_Input_6_Get()) ? 1 : 0;
    currentDIQueueBuffer[6] = (Digital_Input_7_Get()) ? 1 : 0;
    currentDIQueueBuffer[7] = (Digital_Input_8_Get()) ? 1 : 0;
    currentDIQueueBuffer[8] = (Digital_Input_9_Get()) ? 1 : 0;
    currentDIQueueBuffer[9] = (Digital_Input_10_Get()) ? 1 : 0;
    currentDIQueueBuffer[10] = (Digital_Input_11_Get()) ? 1 : 0;
    currentDIQueueBuffer[11] = (Digital_Input_12_Get()) ? 1 : 0;
    currentDIQueueBuffer[12] = (Digital_Input_13_Get()) ? 1 : 0;
    currentDIQueueBuffer[13] = (Digital_Input_14_Get()) ? 1 : 0;
    currentDIQueueBuffer[14] = (Digital_Input_15_Get()) ? 1 : 0;
    currentDIQueueBuffer[15] = (Digital_Input_16_Get()) ? 1 : 0;
    currentDIQueueBuffer[16] = (Digital_Input_17_Get()) ? 1 : 0;
    currentDIQueueBuffer[17] = (Digital_Input_18_Get()) ? 1 : 0;
    currentDIQueueBuffer[18] = (Digital_Input_19_Get()) ? 1 : 0;
    currentDIQueueBuffer[19] = (Digital_Input_20_Get()) ? 1 : 0;
    currentDIQueueBuffer[20] = (Digital_Input_21_Get()) ? 1 : 0;
    currentDIQueueBuffer[21] = (Digital_Input_22_Get()) ? 1 : 0;
    currentDIQueueBuffer[22] = (Digital_Input_23_Get()) ? 1 : 0;
    currentDIQueueBuffer[23] = (Digital_Input_24_Get()) ? 1 : 0;
    currentDIQueueBuffer[24] = (Digital_Input_25_Get()) ? 1 : 0;
    currentDIQueueBuffer[25] = (Digital_Input_26_Get()) ? 1 : 0;
    currentDIQueueBuffer[26] = (Digital_Input_27_Get()) ? 1 : 0;
    currentDIQueueBuffer[27] = (Digital_Input_28_Get()) ? 1 : 0;
    currentDIQueueBuffer[28] = (Digital_Input_29_Get()) ? 1 : 0;
    currentDIQueueBuffer[29] = (Digital_Input_30_Get()) ? 1 : 0;
    currentDIQueueBuffer[30] = (Digital_Input_31_Get()) ? 1 : 0;
    currentDIQueueBuffer[31] = (Digital_Input_32_Get()) ? 1 : 0;
    currentDIQueueBuffer[32] = (Digital_Input_33_Get()) ? 1 : 0;
    currentDIQueueBuffer[33] = (Digital_Input_34_Get()) ? 1 : 0;
    currentDIQueueBuffer[34] = (Digital_Input_35_Get()) ? 1 : 0;
    currentDIQueueBuffer[35] = (Digital_Input_36_Get()) ? 1 : 0;
    currentDIQueueBuffer[36] = (Digital_Input_37_Get()) ? 1 : 0;
    currentDIQueueBuffer[37] = (Digital_Input_38_Get()) ? 1 : 0;
    currentDIQueueBuffer[38] = (Digital_Input_39_Get()) ? 1 : 0;
    currentDIQueueBuffer[39] = (Digital_Input_40_Get()) ? 1 : 0;
    currentDIQueueBuffer[40] = (Digital_Input_41_Get()) ? 1 : 0;
    currentDIQueueBuffer[41] = (Digital_Input_42_Get()) ? 1 : 0;
    currentDIQueueBuffer[42] = (Digital_Input_43_Get()) ? 1 : 0;
    currentDIQueueBuffer[43] = (Digital_Input_44_Get()) ? 1 : 0;
    currentDIQueueBuffer[44] = (Digital_Input_45_Get()) ? 1 : 0;
    currentDIQueueBuffer[45] = (Digital_Input_46_Get()) ? 1 : 0;
    currentDIQueueBuffer[46] = (Digital_Input_47_Get()) ? 1 : 0;
    currentDIQueueBuffer[47] = (Digital_Input_48_Get()) ? 1 : 0;
    currentDIQueueBuffer[48] = (Digital_Input_49_Get()) ? 1 : 0;   

    // Set the digital input data from the expander (inputs 50 to 60)
    /* Read DI 50 to 60 - from IO Expander */
    /* Port 0 => DI 50 to DI 57 => Buffer index 49 to 56 */
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        currentDIQueueBuffer[49U + i] = ((expanderData0 & (1U << i)) != 0U) ? 1U : 0U;
    }

    /* Port 1 => DI 58 to DI 60 => Buffer index 57 to 59 */
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        currentDIQueueBuffer[57U + i] = ((expanderData1 & (1U << i)) != 0U) ? 1U : 0U;
    }

    if (Board_Selection == false)
    {
        Board_Selection = true;
        /* Set Static IP */
        SetStaticIPAddress("192.168.1.231");
    } 
}


/**
 * @brief Calculates the CRC-16 checksum using the polynomial 0x8005.
 *
 * This function computes a 16-bit **CRC (Cyclic Redundancy Check)** for the given data buffer.
 * It implements a **bitwise** algorithm with a standard **0x8005 polynomial** (CRC-16).
 *
 * ## CRC Calculation Details:
 * - **Initial CRC Value**: `0xFFFF`
 * - **Polynomial Used**: `0x8005`
 * - **Bitwise Processing**: Each byte is XORed into the CRC register and then shifted bit-by-bit.
 *
 * @param data Pointer to the data buffer.
 * @param length Number of bytes in the data buffer.
 * @return Computed 16-bit CRC value.
 */
uint16_t CalculateCRC(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    uint16_t polynomial = 0x1021;

    for(uint16_t i = 0; i < length; i++)
    {
        crc ^= ((uint16_t)data[i] << 8);

        for(uint8_t j = 0; j < 8; j++)
        {
            if(crc & 0x8000)
                crc = ((crc << 1) ^ polynomial) & 0xFFFF;
            else
                crc = (crc << 1) & 0xFFFF;
        }
    }

    return crc;
}
/**
 * @brief Parses an incoming packet and validates its structure.
 *
 * This function extracts information from an incoming **IO packet**, including:
 * - **Message type**
 * - **Message subtype**
 * - **Number of tuples**
 * - **Port number and value**
 * - **CRC validation**
 *
 * It performs **error handling** for:
 * - Invalid buffer
 * - Invalid message type/subtype
 * - Incorrect tuple count
 * - Invalid port numbers or values
 * - Packet size mismatches
 *
 * @param u8IOBuffer Pointer to the received packet buffer.
 * @param inbPacketSz Size of the incoming packet.
 */
void ParseAndProcessInbPacket(uint8_t *buf, uint16_t len)
{
    if(buf == NULL)
    {
        SYS_CONSOLE_PRINT("[TCP] ERROR: NULL buffer\r\n");
        return;
    }

    SYS_CONSOLE_PRINT("\r\n[TCP] RX Frame (%u bytes): ", len);
    for(uint16_t i=0;i<len;i++)
        SYS_CONSOLE_PRINT("%02X ",buf[i]);
    SYS_CONSOLE_PRINT("\r\n");

    /* Minimum frame validation */
    if(len < TCP_MIN_FRAME_SIZE)
    {
        SYS_CONSOLE_PRINT("[TCP] ERROR: Frame too short\r\n");
        return;
    }

    /* Parse header */
    Reqframe_St.uniqueId =
            ((uint32_t)buf[0] << 24) |
            ((uint32_t)buf[1] << 16) |
            ((uint32_t)buf[2] << 8) |
            buf[3];

    Reqframe_St.msgType = ((uint16_t)buf[4] << 8) | buf[5];
    Reqframe_St.compartmentId = buf[6];
    Reqframe_St.dockId = buf[7];
    Reqframe_St.commandId = buf[8];
    Reqframe_St.payloadLen = buf[9];

    Reqframe_St.payload = &buf[10];

    uint16_t expectedLen = 10 + Reqframe_St.payloadLen + 2;

    if(len != expectedLen)
    {
        SYS_CONSOLE_PRINT("[TCP] ERROR: Length mismatch\r\n");
        return;
    }

    /* CRC validation */
    uint16_t rxCrc = ((uint16_t)buf[len-2] << 8) | buf[len-1];
    uint16_t calcCrc = CalculateCRC(buf, len-2);

    if(rxCrc != calcCrc)
    {
        SYS_CONSOLE_PRINT("[TCP] ERROR: CRC mismatch\r\n");
        return;
    }

    SYS_CONSOLE_PRINT("[TCP] Frame Valid\r\n");

    /* Message type validation */

    if(Reqframe_St.msgType != MSG_TYPE_REQUEST)
    {
        SYS_CONSOLE_PRINT("[TCP] ERROR: Not a request frame\r\n");
        return;
    }

    /* Command dispatcher */

    switch(Reqframe_St.commandId)
    {
        case CMD_GPIO_OPERATION:
            HandleGPIOCommand();
            break;

        case CMD_ANALOG_READ:
            HandleAnalogRead();
            break;

        case CMD_CHARGING_COMMAND:
            uint8_t u8DockNo = Reqframe_St.dockId;
            HandleChargingCommand(u8DockNo);
            break;

        case CMD_BOOT_MODE_COMMAND:
            HandleBootloaderCommand();
            return;

        case CMD_SOFT_RESET_COMMAND:
            // HandleSoftResetCommand();
            return;

        default:
            SYS_CONSOLE_PRINT("[TCP] ERROR: Unknown Command\r\n");
            outbErrorCode = errorInvalidMsg;
            PrepareDispatchOutbPacket();
            break;
    }

    ResetInbOutbData();
}

void HandleGPIOCommand(void)
{
    uint8_t payload[5];

    uint8_t msgType;
    uint16_t portNo;
    uint8_t portVal = 0;
    uint8_t outVal = 0;
    uint8_t errorCode = errorNoError;

    msgType = Reqframe_St.payload[0];

    if ((msgType == msgSubTypeDigitalWrite && Reqframe_St.payloadLen != 4) ||
        (msgType == msgSubTypeDigitalRead && Reqframe_St.payloadLen != 3))
    {
        SYS_CONSOLE_PRINT("[GPIO] ERROR: Payload size mismatch\r\n");
        errorCode = errorInvalidMsg;
    }

    portNo = ((uint16_t)Reqframe_St.payload[1] << 8) | Reqframe_St.payload[2];

    if (msgType == msgSubTypeDigitalWrite)
    {
        portVal = Reqframe_St.payload[3];
    }

    if (portNo < 1 || portNo > 110)
    {
        SYS_CONSOLE_PRINT("[GPIO] ERROR: Invalid Port\r\n");
        errorCode = errorInvalidPort;
    }

    if ((portVal != 0) && (portVal != 1))
    {
        SYS_CONSOLE_PRINT("[GPIO] ERROR: Invalid Value\r\n");
        errorCode = errorInvalidValue;
    }

    if (errorCode == errorNoError)
    {
        ProcessInbPacket(msgType, portNo, portVal, &outVal, &errorCode);
        StoreRelay_DigitalOutputsFrame();
    }

    // Build response
    payload[0] = msgType;
    payload[1] = (portNo >> 8) & 0xFF;
    payload[2] = portNo & 0xFF;
    payload[3] = errorCode;
    payload[4] = (msgType == msgSubTypeDigitalWrite) ? portVal : outVal;

    SendResponsePayload(payload, 5);
}

void HandleAnalogRead(void)
{
    if(Reqframe_St.payloadLen < 1)
    {
        outbErrorCode = errorInvalidMsg;
        PrepareDispatchOutbPacket();
        return;
    }

    uint8_t channel = Reqframe_St.payload[0];

    SYS_CONSOLE_PRINT("[ANALOG] Read Channel %d\r\n",channel);

    uint32_t adcValue = Analog_Input_Get(channel);

    uint8_t payload[5] = {0};

    payload[0] = channel;

    payload[1] = (adcValue >> 24) & 0xFF;
    payload[2] = (adcValue >> 16) & 0xFF;
    payload[3] = (adcValue >> 8) & 0xFF;
    payload[4] = adcValue & 0xFF;

    SendResponsePayload(payload,5);
}

void HandleChargingCommand(uint8_t u8DockNo)
{
    if(Reqframe_St.payloadLen < 1)
    {
        outbErrorCode = errorInvalidMsg;
        PrepareDispatchOutbPacket();
        return;
    }

    uint8_t action = Reqframe_St.payload[0];

    SYS_CONSOLE_PRINT("[CHARGING] Action: %s\r\n",
                      action ? "START" : "STOP");
    ChargingControl(u8DockNo,action);
    uint8_t status = 01; // Assume success for now

    uint8_t payload[1];
    payload[0] = status;

    SendResponsePayload(payload,1);
}

void HandleBootloaderCommand(void)
{
    SYS_CONSOLE_PRINT("[SYSTEM] Bootloader Trigger\r\n");

    ramStart[0] = BTL_TRIGGER_PATTERN;
    ramStart[1] = BTL_TRIGGER_PATTERN;
    ramStart[2] = BTL_TRIGGER_PATTERN;
    ramStart[3] = BTL_TRIGGER_PATTERN;

    DCACHE_CLEAN_BY_ADDR(ramStart, 16);

    SYS_RESET_SoftwareReset();
}

void SendResponsePayload(uint8_t *payload, uint8_t len)
{
    uint16_t index = 0;

    outbPacketBuf[index++] = (Reqframe_St.uniqueId >> 24);
    outbPacketBuf[index++] = (Reqframe_St.uniqueId >> 16);
    outbPacketBuf[index++] = (Reqframe_St.uniqueId >> 8);
    outbPacketBuf[index++] = Reqframe_St.uniqueId;

    outbPacketBuf[index++] = 0x00;
    outbPacketBuf[index++] = 0x02;

    outbPacketBuf[index++] = Reqframe_St.compartmentId;
    outbPacketBuf[index++] = Reqframe_St.dockId;
    outbPacketBuf[index++] = Reqframe_St.commandId;

    outbPacketBuf[index++] = len;

    memcpy(&outbPacketBuf[index], payload, len);
    index += len;

    uint16_t crc = CalculateCRC(outbPacketBuf,index);

    outbPacketBuf[index++] = (crc >> 8);
    outbPacketBuf[index++] = crc;

    TCPIP_TCP_ArrayPut(sIOServerSocket,outbPacketBuf,index);
}

/**
 * @brief Prepares and dispatches an outbound packet over TCP.
 *
 * This function constructs an outbound packet (`outbPacketBuf`) with 
 * appropriate fields such as error code, message subtype, port number, 
 * and port value. The packet is then transmitted over a TCP socket.
 *
 * Steps:
 * 1. Populate `outbPacketBuf` with relevant data.
 * 2. Compute `outbPacketSz` based on the number of tuples.
 * 3. Print the packet contents for debugging.
 * 4. Send the packet using `TCPIP_TCP_ArrayPut`.
 *
 * @note Ensure `outbPacketBuf` has enough allocated memory to avoid overflow.
 */

void PrepareDispatchOutbPacket()
{
    uint16_t index = 0;

    /* Unique ID (same as request) */
    outbPacketBuf[index++] = (Reqframe_St.uniqueId >> 24) & 0xFF;
    outbPacketBuf[index++] = (Reqframe_St.uniqueId >> 16) & 0xFF;
    outbPacketBuf[index++] = (Reqframe_St.uniqueId >> 8) & 0xFF;
    outbPacketBuf[index++] = Reqframe_St.uniqueId & 0xFF;

    /* Message Type = Response (0x0002) */
    outbPacketBuf[index++] = 0x00;
    outbPacketBuf[index++] = 0x02;

    /* Compartment ID */
    outbPacketBuf[index++] = Reqframe_St.compartmentId;

    /* Dock ID */
    outbPacketBuf[index++] = Reqframe_St.dockId;

    /* Command ID */
    outbPacketBuf[index++] = Reqframe_St.commandId;

    if (outbErrorCode != errorNoError) {
        /* Payload Length */
        outbPacketBuf[index++] = Reqframe_St.payloadLen;
        outbPacketBuf[index++] = Reqframe_St.payload[0];
        outbPacketBuf[index++] = Reqframe_St.payload[1];
        outbPacketBuf[index++] = Reqframe_St.payload[2];
        outbPacketBuf[index++] = outbErrorCode; // Add error code in payload for error response
    } else {
        /* Payload Length */
        outbPacketBuf[index++] = Reqframe_St.payloadLen + 1; // If success then add 1 byte for port value;
        outbPacketBuf[index++] = Reqframe_St.payload[0];
        outbPacketBuf[index++] = Reqframe_St.payload[1];
        outbPacketBuf[index++] = Reqframe_St.payload[2];
        outbPacketBuf[index++] = errorNoError; // Add error code in payload for error response
        if (inbMsgSubType == msgSubTypeDigitalWrite) {
            outbPacketBuf[index++] = (inbPortVal != 0) ? 0x01 : 0x00;
        } else {
            outbPacketBuf[index++] = (outbPortVal != 0) ? 0x01 : 0x00;
        }
    }
    /* Calculate CRC */
    uint16_t crc = CalculateCRC(outbPacketBuf, index);

    outbPacketBuf[index++] = (crc >> 8) & 0xFF;
    outbPacketBuf[index++] = crc & 0xFF;

    outbPacketSz = index;

    /* Debug print */
    SYS_CONSOLE_PRINT("[TCP] Response Frame: ");
    for(uint16_t i = 0; i < outbPacketSz; i++)
    {
        SYS_CONSOLE_PRINT("%02X ", outbPacketBuf[i]);
    }
    SYS_CONSOLE_PRINT("\r\n");

    /* Send TCP */
    TCPIP_TCP_ArrayPut(sIOServerSocket, (const uint8_t*)outbPacketBuf, outbPacketSz);
}
/**
 * @brief   Resets inbound and outbound data structures.
 * 
 * This function clears and resets various inbound (`inb*`) and outbound (`outb*`) 
 * data variables to their default states. It ensures that:
 * - Message subtype, port numbers, and values are set to zero or -1 as needed.
 * - The packet buffers (`inbPacketBuf` and `outbPacketBuf`) are fully cleared (set to 0x00).
 * - Packet sizes are reset to zero.
 * - The error code for outbound messages is reset to `errorNoError`.
 * 
 * This function is typically used before processing new messages to ensure
 * no residual data from previous transmissions affects the new operation.
 */
void ResetInbOutbData()
{
    inbMsgSubType = 0x00;
    inbNumTuples = 0; // Allow only max of 1 for now
    inbPortNo = -1;
    inbPortVal = -1;
    inbCrc = -1;
    
    for (int i = 0; i < 256; i++)
    {
        inbPacketBuf[i] = 0x00;
    }
    inbPacketSz = 0;

    // Outgoing Command
    outbRemotePort = 0;
    outbErrorCode = errorNoError;
    outbMsgSubType = 0x00;
    outbNumTuples = 0; // Allow only max of 1 for now
    outbPortNo = -1;
    outbPortVal = -1;
    outbCrc = -1;

    Reqframe_St.uniqueId = 0;
    Reqframe_St.msgType = 0;
    Reqframe_St.compartmentId = 0;
    Reqframe_St.dockId = 0;
    Reqframe_St.commandId = 0;
    Reqframe_St.payloadLen = 0;
    Reqframe_St.payload = NULL;

    for (int i = 0; i < 256; i++)
    {
        outbPacketBuf[i] = 0x00;
    }
    
    outbPacketSz = 0;
}
/**
 * @brief   IO Handler Server Task.
 * 
 * This FreeRTOS task is responsible for:
 * - Configuring IO expanders.
 * - Initializing and managing server communication (TCP for HEV, UDP for Two-Wheeler).
 * - Reading stored output states from Flash.
 * - Handling digital and analog input readings.
 * - Processing incoming commands and responding accordingly.
 * - Ensuring reconnection if the TCP/UDP server socket is disconnected.
 * 
 * The task continuously runs in a loop, performing these operations periodically.
 * 
 * @param pvParameters Unused task parameter.
 */

void vIOHandlerServerTask(void *pvParameters)
{
    // Configure IO expanders
    vConfigureIOexpanders();

    // Read stored output states from Flash memory
    SYS_CONSOLE_PRINT("Reading stored output states from Flash\r\n");
    readOutputsFromFlash();

    // Print function entry for debugging
    SYS_CONSOLE_PRINT("In Function: %s\r\n", __FUNCTION__);

    // Open server socket based on the aggregator type
    sIOServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, IO_SERVER_PORT_HEV, 0);

    if (sIOServerSocket == INVALID_SOCKET)
    {
        SYS_CONSOLE_PRINT("Error opening IO server socket\r\n");
        vTaskDelete(NULL);
        return;
    }
    while (true)
    {
        // Read digital and analog inputs
        readDigitalInputs();
        ReadAllAnalogInputPins();
        uint8_t u8IOBuffer[256] = {0};
        if (TCPIP_TCP_IsConnected(sIOServerSocket))
        {
            int16_t bytesRead = TCPIP_TCP_ArrayGet(sIOServerSocket, u8IOBuffer, sizeof(u8IOBuffer));

            if (bytesRead > 0)
            {
                SYS_CONSOLE_PRINT("Got Data on IO Server Handler (TCP)\r\n");
                // Read and process incoming commands if any
                ParseAndProcessInbPacket(u8IOBuffer, bytesRead);
            }

            if (!TCPIP_TCP_IsConnected(sIOServerSocket) || TCPIP_TCP_WasDisconnected(sIOServerSocket))
            {
                SYS_CONSOLE_PRINT("\r\nTCP Connection Closed\r\n");
                TCPIP_TCP_Close(sIOServerSocket);
                sIOServerSocket = INVALID_SOCKET;
            }
        }
        // Ensure reconnection logic is handled correctly
        if (sIOServerSocket == INVALID_SOCKET)
        {
            SYS_CONSOLE_PRINT("TCP server socket disconnected, attempting to reconnect...\r\n");

            // Attempt to open the socket again
            sIOServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, IO_SERVER_PORT_HEV, 0);

            if (sIOServerSocket == INVALID_SOCKET)
            {
                SYS_CONSOLE_PRINT("Failed to reopen IO socket, retrying...\r\n");
            }
            else
            {
                SYS_CONSOLE_PRINT("Reconnected successfully IO Socket!\r\n");
            }

            vTaskDelay(RECONNECT_DELAY_MS);
        }
        vTaskDelay(10); // Add a small delay to avoid consuming too much CPU time
    }
}
/**
 * @brief   Initializes and starts IO handler tasks.
 * 
 * This function creates two FreeRTOS tasks:
 * - vIOHandlerServerTask: Manages IO server communication and processing.
 * - vToggleLEDTask: Handles periodic LED toggling.
 * 
 * The tasks are created with predefined stack sizes and priorities.
 */
void vIOHandler()
{
    // Create IO Handler Server Task for managing IO communication
    xTaskCreate(vIOHandlerServerTask, 
                "vIOHandlerServerTask", 
                IO_SERVER_HANDLER_HEAP_DEPTH, 
                NULL, 
                IO_SERVER_HANDLER_TASK_PRIORITY, 
                NULL);    
}


/**
 * @brief  Get the digital input value from IO Expander for input 50.
 * @return 1 if the input is high, 0 if low.
 */
uint8_t Digital_Input_50_Get(void)
{
    /* Accessing digital input 50 from index 49 */
    return currentDIQueueBuffer[49];
}

/**
 * @brief  Get the digital input value from IO Expander for input 51.
 * @return 1 if the input is high, 0 if low.
 */
uint8_t Digital_Input_51_Get(void)
{
    /* Accessing digital input 51 from index 50 */
    return currentDIQueueBuffer[50];
}

/**
 * @brief  Get the digital input value from IO Expander for input 52.
 * @return 1 if the input is high, 0 if low.
 */
uint8_t Digital_Input_52_Get(void)
{
    /* Accessing digital input 52 from index 51 */
    return currentDIQueueBuffer[51];
}

/**
 * @brief  Get the digital input value from IO Expander for input 53.
 * @return 1 if the input is high, 0 if low.
 */
uint8_t Digital_Input_53_Get(void)
{
    /* Accessing digital input 53 from index 52 */
    return currentDIQueueBuffer[52];
}

/**
 * @brief  Get the digital input value from IO Expander for input 54.
 * @return 1 if the input is high, 0 if low.
 */
uint8_t Digital_Input_54_Get(void)
{
    /* Accessing digital input 54 from index 53 */
    return currentDIQueueBuffer[53];
}

/**
 * @brief  Get the digital input value from IO Expander for input 55.
 * @return 1 if the input is high, 0 if low.
 */
uint8_t Digital_Input_55_Get(void)
{
    /* Accessing digital input 55 from index 54 */
    return currentDIQueueBuffer[54];
}

/**
 * @brief  Get the digital input value from IO Expander for input 56.
 * @return 1 if the input is high, 0 if low.
 */
uint8_t Digital_Input_56_Get(void)
{
    /* Accessing digital input 56 from index 55 */
    return currentDIQueueBuffer[55];
}

/**
 * @brief  Get the digital input value from IO Expander for input 57.
 * @return 1 if the input is high, 0 if low.
 */
uint8_t Digital_Input_57_Get(void)
{
    /* Accessing digital input 57 from index 56 */
    return currentDIQueueBuffer[56];
}

/**
 * @brief  Get the digital input value from IO Expander for input 58.
 * @return 1 if the input is high, 0 if low.
 */
uint8_t Digital_Input_58_Get(void)
{
    /* Accessing digital input 58 from index 57 */
    return currentDIQueueBuffer[57];
}

/**
 * @brief  Get the digital input value from IO Expander for input 59.
 * @return 1 if the input is high, 0 if low.
 */
uint8_t Digital_Input_59_Get(void)
{
    /* Accessing digital input 59 from index 58 */
    return currentDIQueueBuffer[58];
}

/**
 * @brief  Get the digital input value from IO Expander for input 60.
 * @return 1 if the input is high, 0 if low.
 */
uint8_t Digital_Input_60_Get(void)
{
    /* Accessing digital input 60 from index 59 */
    return currentDIQueueBuffer[59];
}

/**
 * @brief  Get the Analog value for Chennel 1.
 * @return Analog Value.
 */
uint32_t Analog_Input_Get(uint8_t channel)
{
    return currentTempAIQueueBuffer[channel - 1];
}

uint8_t GPIO_Read(uint16_t portNo, uint8_t *pErrorCode)
{
    switch (portNo)
    {
        // Digital Inputs (1–60)
        case 1:  return Digital_Input_1_Get();
        case 2:  return Digital_Input_2_Get();
        case 3:  return Digital_Input_3_Get();
        case 4:  return Digital_Input_4_Get();
        case 5:  return Digital_Input_5_Get();
        case 6:  return Digital_Input_6_Get();
        case 7:  return Digital_Input_7_Get();
        case 8:  return Digital_Input_8_Get();
        case 9:  return Digital_Input_9_Get();
        case 10: return Digital_Input_10_Get();
        case 11: return Digital_Input_11_Get();
        case 12: return Digital_Input_12_Get();
        case 13: return Digital_Input_13_Get();
        case 14: return Digital_Input_14_Get();
        case 15: return Digital_Input_15_Get();
        case 16: return Digital_Input_16_Get();
        case 17: return Digital_Input_17_Get();
        case 18: return Digital_Input_18_Get();
        case 19: return Digital_Input_19_Get();
        case 20: return Digital_Input_20_Get();
        case 21: return Digital_Input_21_Get();
        case 22: return Digital_Input_22_Get();
        case 23: return Digital_Input_23_Get();
        case 24: return Digital_Input_24_Get();
        case 25: return Digital_Input_25_Get();
        case 26: return Digital_Input_26_Get();
        case 27: return Digital_Input_27_Get();
        case 28: return Digital_Input_28_Get();
        case 29: return Digital_Input_29_Get();
        case 30: return Digital_Input_30_Get();
        case 31: return Digital_Input_31_Get();
        case 32: return Digital_Input_32_Get();
        case 33: return Digital_Input_33_Get();
        case 34: return Digital_Input_34_Get();
        case 35: return Digital_Input_35_Get();
        case 36: return Digital_Input_36_Get();
        case 37: return Digital_Input_37_Get();
        case 38: return Digital_Input_38_Get();
        case 39: return Digital_Input_39_Get();
        case 40: return Digital_Input_40_Get();
        case 41: return Digital_Input_41_Get();
        case 42: return Digital_Input_42_Get();
        case 43: return Digital_Input_43_Get();
        case 44: return Digital_Input_44_Get();
        case 45: return Digital_Input_45_Get();
        case 46: return Digital_Input_46_Get();
        case 47: return Digital_Input_47_Get();
        case 48: return Digital_Input_48_Get();
        case 49: return Digital_Input_49_Get();
        case 50: return Digital_Input_50_Get();
        case 51: return Digital_Input_51_Get();
        case 52: return Digital_Input_52_Get();
        case 53: return Digital_Input_53_Get();
        case 54: return Digital_Input_54_Get();
        case 55: return Digital_Input_55_Get();
        case 56: return Digital_Input_56_Get();
        case 57: return Digital_Input_57_Get();
        case 58: return Digital_Input_58_Get();
        case 59: return Digital_Input_59_Get();
        case 60: return Digital_Input_60_Get();

        // Digital Outputs (61–84)
        case 61: return Digital_Output_1_Get();
        case 62: return Digital_Output_2_Get();
        case 63: return Digital_Output_3_Get();
        case 64: return Digital_Output_4_Get();
        case 65: return Digital_Output_5_Get();
        case 66: return Digital_Output_6_Get();
        case 67: return Digital_Output_7_Get();
        case 68: return Digital_Output_8_Get();
        case 69: return Digital_Output_9_Get();
        case 70: return Digital_Output_10_Get();
        case 71: return Digital_Output_11_Get();
        case 72: return Digital_Output_12_Get();
        case 73: return Digital_Output_13_Get();
        case 74: return Digital_Output_14_Get();
        case 75: return Digital_Output_15_Get();
        case 76: return Digital_Output_16_Get();
        case 77: return Digital_Output_17_Get();
        case 78: return Digital_Output_18_Get();
        case 79: return Digital_Output_19_Get();
        case 80: return Digital_Output_20_Get();
        case 81: return Digital_Output_21_Get();
        case 82: return Digital_Output_22_Get();
        case 83: return Digital_Output_23_Get();
        case 84: return Digital_Output_24_Get();

        // Efuse + Relay
        case 85: return efuse1_in_Get();
        case 86: return efuse2_in_Get();
        case 87: return efuse3_in_Get();
        case 88: return efuse4_in_Get();
        case 89: return Relay_Output_1_Get();
        case 90: return Relay_Output_2_Get();

        default:
            *pErrorCode = errorInvalidPort;
            return 0;
    }
}

void GPIO_Write(uint16_t portNo, bool bPortVal, uint8_t *pErrorCode)
{
    if (portNo < 61 || portNo > 90)
    {
        *pErrorCode = errorInvalidPort;
        return;
    }

    switch (portNo)
    {
        case 61: bPortVal ? Digital_Output_1_Set()  : Digital_Output_1_Clear(); break;
        case 62: bPortVal ? Digital_Output_2_Set()  : Digital_Output_2_Clear(); break;
        case 63: bPortVal ? Digital_Output_3_Set()  : Digital_Output_3_Clear(); break;
        case 64: bPortVal ? Digital_Output_4_Set()  : Digital_Output_4_Clear(); break;
        case 65: bPortVal ? Digital_Output_5_Set()  : Digital_Output_5_Clear(); break;
        case 66: bPortVal ? Digital_Output_6_Set()  : Digital_Output_6_Clear(); break;
        case 67: bPortVal ? Digital_Output_7_Set()  : Digital_Output_7_Clear(); break;
        case 68: bPortVal ? Digital_Output_8_Set()  : Digital_Output_8_Clear(); break;
        case 69: bPortVal ? Digital_Output_9_Set()  : Digital_Output_9_Clear(); break;
        case 70: bPortVal ? Digital_Output_10_Set() : Digital_Output_10_Clear(); break;
        case 71: bPortVal ? Digital_Output_11_Set() : Digital_Output_11_Clear(); break;
        case 72: bPortVal ? Digital_Output_12_Set() : Digital_Output_12_Clear(); break;
        case 73: bPortVal ? Digital_Output_13_Set() : Digital_Output_13_Clear(); break;
        case 74: bPortVal ? Digital_Output_14_Set() : Digital_Output_14_Clear(); break;
        case 75: bPortVal ? Digital_Output_15_Set() : Digital_Output_15_Clear(); break;
        case 76: bPortVal ? Digital_Output_16_Set() : Digital_Output_16_Clear(); break;
        case 77: bPortVal ? Digital_Output_17_Set() : Digital_Output_17_Clear(); break;
        case 78: bPortVal ? Digital_Output_18_Set() : Digital_Output_18_Clear(); break;
        case 79: bPortVal ? Digital_Output_19_Set() : Digital_Output_19_Clear(); break;
        case 80: bPortVal ? Digital_Output_20_Set() : Digital_Output_20_Clear(); break;
        case 81: bPortVal ? Digital_Output_21_Set() : Digital_Output_21_Clear(); break;
        case 82: bPortVal ? Digital_Output_22_Set() : Digital_Output_22_Clear(); break;
        case 83: bPortVal ? Digital_Output_23_Set() : Digital_Output_23_Clear(); break;
        case 84: bPortVal ? Digital_Output_24_Set() : Digital_Output_24_Clear(); break;

        case 85: bPortVal ? efuse1_in_Set() : efuse1_in_Clear(); break;
        case 86: bPortVal ? efuse2_in_Set() : efuse2_in_Clear(); break;
        case 87: bPortVal ? efuse3_in_Set() : efuse3_in_Clear(); break;
        case 88: bPortVal ? efuse4_in_Set() : efuse4_in_Clear(); break;
        case 89: bPortVal ? Relay_Output_1_Set() : Relay_Output_1_Clear(); break;
        case 90: bPortVal ? Relay_Output_2_Set() : Relay_Output_2_Clear(); break;

        default:
            *pErrorCode = errorInvalidPort;
            return;
    }
}
/**
 * @brief Processes the parsed incoming packet.
 *
 * This function handles:
 * - **Digital input reads**
 * - **Relay output reads**
 * - **Digital output reads**
 * - **Digital/Relay output writes**
 *
 * @note The function **does not process** the packet if there are any prior errors.
 */
void ProcessInbPacket(uint8_t msgType,
                      uint16_t portNo,
                      uint8_t portVal,
                      uint8_t *pOutVal,
                      uint8_t *pErrorCode)
{
    if (*pErrorCode != errorNoError)
        return;

    if (msgType == msgSubTypeDigitalRead)
    {
        *pOutVal = GPIO_Read(portNo, pErrorCode);
    }
    else if (msgType == msgSubTypeDigitalWrite)
    {
        GPIO_Write(portNo, portVal, pErrorCode);
    }
    else
    {
        *pErrorCode = errorInvalidMsg;
    }
}

void ChargingControl(uint8_t u8DockNo, uint8_t action)
{
    // Placeholder for actual charging control logic
    //SYS_CONSOLE_PRINT("[CHARGING] Control for Dock %d, Action: %s\r\n", u8DockNo, action ? "START" : "STOP");
    static uint8_t u8PevChargingState[MAX_DOCKS] = {0}; // Assuming max 4 docks for example
    if (u8PevChargingState[u8DockNo] != action) {
        SYS_CONSOLE_PRINT("[CHARGING] Dock %d Charging State %s\r\n", u8DockNo, action ? "START" : "STOP");
        SESSION_SetAuthenticationCommand(u8DockNo, action);
        u8PevChargingState[u8DockNo] = action;
    }
}

bool bGPIO_Operation(GPIOOperation_e eGPIOType, uint8_t u8DockNo)
{
    bool bRet = true;
    uint8_t outbErrorCode = errorNoError;
    switch (eGPIOType)
    {
    case DO_AC_RELAY_ON:
        GPIO_Write(Gpio_conf.AC_Relay_Pin[u8DockNo], true, &outbErrorCode);
        bRet = true;
        break;
    case DO_AC_RELAY_OFF:
        GPIO_Write(Gpio_conf.AC_Relay_Pin[u8DockNo], false, &outbErrorCode);
        bRet = true;
        break;
    case DO_DC_RELAY_ON:
        GPIO_Write(Gpio_conf.DC_Relay_Pin[u8DockNo], true, &outbErrorCode);
        bRet = true;
        break;
    case DO_DC_RELAY_OFF:
        GPIO_Write(Gpio_conf.DC_Relay_Pin[u8DockNo], false, &outbErrorCode);
        bRet = true;
        break;
    case DO_SOLENOID_HIGH:
        GPIO_Write(Gpio_conf.Solenoid_PinHi[u8DockNo], true, &outbErrorCode);
        vTaskDelay(100); // Add delay if solenoid needs time to actuate
        GPIO_Write(Gpio_conf.Solenoid_PinHi[u8DockNo], false, &outbErrorCode);
        bRet = true;
        break;
    case DO_SOLENOID_LOW:
        GPIO_Write(Gpio_conf.Solenoid_PinLo[u8DockNo], true, &outbErrorCode);
        vTaskDelay(100); // Add delay if solenoid needs time to actuate
        GPIO_Write(Gpio_conf.Solenoid_PinLo[u8DockNo], false, &outbErrorCode);
        bRet = true;
        break;
    case DO_R_LED_HIGH:
        GPIO_Write(Gpio_conf.R_LED_Pin[u8DockNo], true, &outbErrorCode);
        bRet = true;
        break;
    case DO_R_LED_LOW:
        GPIO_Write(Gpio_conf.R_LED_Pin[u8DockNo], false, &outbErrorCode);
        bRet = true;
        break;
    case DO_G_LED_HIGH:
        GPIO_Write(Gpio_conf.G_LED_Pin[u8DockNo], true, &outbErrorCode);
        bRet = true;
        break;
    case DO_G_LED_LOW:
        GPIO_Write(Gpio_conf.G_LED_Pin[u8DockNo], false, &outbErrorCode);
        bRet = true;
        break;
    case DO_B_LED_HIGH:
        GPIO_Write(Gpio_conf.B_LED_Pin[u8DockNo], true, &outbErrorCode);
        bRet = true;
        break;
    case DO_B_LED_LOW:
        GPIO_Write(Gpio_conf.B_LED_Pin[u8DockNo], false, &outbErrorCode);
        bRet = true;
        break;
    case DO_DOCK_FAN_HIGH:
        GPIO_Write(Gpio_conf.Dock_Fan_Pin[u8DockNo], true, &outbErrorCode);
        bRet = true;
        break;
    case DO_DOCK_FAN_LOW:
        GPIO_Write(Gpio_conf.Dock_Fan_Pin[u8DockNo], false, &outbErrorCode);
        bRet = true;
        break;
    case DO_COMPARTMENT_FAN_HIGH:
        GPIO_Write(Gpio_conf.Compartment_Fan_Pin, true, &outbErrorCode);
        bRet = true;
        break;
    case DO_COMPARTMENT_FAN_LOW:
        GPIO_Write(Gpio_conf.Compartment_Fan_Pin, false, &outbErrorCode);
        bRet = true;
        break;
    case DI_E_STOP_STATUS:
        bRet = GPIO_Read(Gpio_conf.EStop_Pin, &outbErrorCode);
        break;
    case DI_DOOR_LOCK_STATUS:
        bRet = GPIO_Read(Gpio_conf.DoorLock_Pin[u8DockNo], &outbErrorCode);
        break;
    case DI_SOLENOID_LOCK_STATUS:
        bRet = GPIO_Read(Gpio_conf.SolenoidLock_Pin[u8DockNo], &outbErrorCode);
        break;
    default:
        SYS_CONSOLE_PRINT("[GPIO] Unknown GPIO Operation\r\n");
        bRet = false;
        break;
    }
}

/* *****************************************************************************
 End of File
 */

