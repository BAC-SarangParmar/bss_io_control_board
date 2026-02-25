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
const char msgSubTypeDigitalRead  = 0x01;
const char msgSubTypeDigitalWrite = 0x02;
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
uint32_t lastTick = 0;

// Create a buffer to hold the previous status of all pins
uint8_t prevDIQueueBuffer[NUM_DIGITAL_PINS];

void IsDigitalInputChanged();
void ReadAllAnalogInputPins();
bool IOExpander1_Configure_Ports(void);
bool IOExpander2_Configure_Ports(void);
void Active_Safety_Pins();
void Default_Pin_Status();
volatile  uint16_t currentTempAIQueueBuffer[NUM_TEMPERATURE_ANALOG_PINS];
volatile  uint16_t currentAIQueueBuffer[NUM_ANALOG_PINS];
uint16_t prevTempAIQueueBuffer[NUM_TEMPERATURE_ANALOG_PINS];
uint16_t prevAIQueueBuffer[NUM_ANALOG_PINS];
char messageAnalog[MESSAGE_BUFFER_SIZE];
uint32_t total_length;
volatile  uint8_t currentDIQueueBuffer[NUM_DIGITAL_PINS];
uint16_t CalculateCRC(uint8_t *data, uint16_t length);
static void SetStaticIPAddress(const char* ipStr);
volatile bool custom_message = false;
uint32_t *ramStart = (uint32_t *)BTL_TRIGGER_RAM_START;
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
        SYS_CONSOLE_PRINT("Restored Outputs - Digital: %08X, Relay: %04X\n", 
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
                    SYS_CONSOLE_PRINT("? Invalid Data Bits (%d) for RS485_%d\n", dataBits, uartIndex + 1);
                    continue;
            }

            USART_PARITY parity;
            switch (parityEncoded) {
                case 0: parity = USART_PARITY_NONE; break;
                case 1: parity = USART_PARITY_ODD; break;
                case 2: parity = USART_PARITY_EVEN; break;
                default:
                    SYS_CONSOLE_PRINT("? Invalid Parity (%d) for RS485_%d\n", parityEncoded, uartIndex + 1);
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
                    SYS_CONSOLE_PRINT("? RS485_1 setup from flash failed: Baud=%d Data=%d Parity=%d Stop=%d\n",
                                      baud, dataBits, parityEncoded, stopBits);
                } else {
                    SYS_CONSOLE_PRINT("RS485_1 setup from flash done: Baud=%d Data=%d Parity=%d Stop=%d\n",
                                      baud, dataBits, parityEncoded, stopBits);
                    uint8_t dummy;
                    bool result = SERCOM8_USART_Read(&dummy, 1);
                    SYS_CONSOLE_PRINT("RS485_1 RX resume: %s\n", result ? "OK" : "FAILED");
                }
            } else if (uartIndex == 1) {
                SERCOM9_USART_ReadAbort();
                if (!SERCOM9_USART_SerialSetup(&setup, 0)) {
                    SYS_CONSOLE_PRINT("? RS485_2 setup from flash failed: Baud=%d Data=%d Parity=%d Stop=%d\n",
                                      baud, dataBits, parityEncoded, stopBits);
                } else {
                    SYS_CONSOLE_PRINT("RS485_2 setup from flash done: Baud=%d Data=%d Parity=%d Stop=%d\n",
                                      baud, dataBits, parityEncoded, stopBits);
                    uint8_t dummy;
                    bool result = SERCOM9_USART_Read(&dummy, 1);
                    SYS_CONSOLE_PRINT("RS485_2 RX resume: %s\n", result ? "OK" : "FAILED");
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
#if HEV_IO_Aggregator
    TCPIP_TCP_Close(sCan0ServerSocket);
    sCan0ServerSocket = INVALID_SOCKET;
    TCPIP_TCP_Close(sCan1ServerSocket);
    sCan1ServerSocket = INVALID_SOCKET;  
    TCPIP_TCP_Close(sCan2ServerSocket);
    sCan2ServerSocket = INVALID_SOCKET;  
    TCPIP_TCP_Close(sCan3ServerSocket);
    sCan3ServerSocket = INVALID_SOCKET;  
    TCPIP_TCP_Close(sCan4ServerSocket);
    sCan4ServerSocket = INVALID_SOCKET;  
    TCPIP_TCP_Close(sCan5ServerSocket);
    sCan5ServerSocket = INVALID_SOCKET; 
                        
    // Print loaded or default configuration
    PrintLoadedConfig();
#endif    
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

    /* Reset globals at start of function */
    total_length = 0U;
    char *pWrite = messageAnalog; /* Safe buffer pointer */

    volatile uint32_t all_adc_data[ALL_ANALOG_PINS]; /* Store raw ADC values */
    bool dataChanged = false;   /* Flag to indicate if data has changed */

    uint8_t changedCount = 0U;
    uint8_t changedIndexes[NUM_TEMPERATURE_ANALOG_PINS]; /* Stores indexes of changed inputs */

    /* ---- Static sensor/channel mapping tables ---- */
    static const char* const sensor_names[ALL_ANALOG_PINS] = {
        "Analog Input1", "Analog Input2", "Analog Input3", "Analog Input4",
        "Analog Input5", "Analog Input6", "Analog Input7", "Analog Input8",
        "Analog Input9", "Analog Input10", "Analog Input11", "Analog Input12",
        "Analog Input13", "Analog Input14", "Analog Input15", "Analog Input16",
        "Analog Input17", "Analog Input18", "Analog Input19", "Analog Input20"
    };

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

            /* Convert temperature to 0.01 °C fixed-point format */
            uint16_t tempValue = (uint16_t)(temperature * 100.0F);
            if (tempValue > 65535U) { tempValue = 65535U; } /* Clamp to max */

            currentTempAIQueueBuffer[i] = tempValue;

            int32_t diff = (int32_t)tempValue - (int32_t)prevTempAIQueueBuffer[i];

            /* Detect change: either first read (0xFFFF) or >= 1.00 °C change */
            if ((prevTempAIQueueBuffer[i] == 0xFFFFU) || (abs(diff) >= 100))
            {
                dataChanged = true;
                changedIndexes[changedCount++] = i;
            }

            prevTempAIQueueBuffer[i] = tempValue;

            /* Log entry formatting */
            len = snprintf(pWrite, (size_t)(MESSAGE_BUFFER_SIZE - total_length),
                           "Core %d, Channel %d, Raw: %lu (%s): %.2f V, Temp: %.2f C\r\n",
                           (int)adc_cores[i], (int)adc_channels[i],
                           (unsigned long)all_adc_data[i],
                           sensor_names[i], adc_voltage, temperature);
        }
        else /* Non-temperature analog inputs */
        {
            /* Scale to 0.01 V units */
            uint16_t value16 = (uint16_t)(adc_voltage * 100.0F);
            if (value16 > 65535U) { value16 = 65535U; } /* Clamp */

            currentAIQueueBuffer[i - 16U] = value16;

            uint16_t prev = prevAIQueueBuffer[i - 16U];
            int32_t diff = (int32_t)value16 - (int32_t)prev;

            /* Detect change: first read or ?0.01 V difference */
            if ((prev == 0xFFFFU) || (abs(diff) >= 1))
            {
                dataChanged = true;
                changedIndexes[changedCount++] = i;
            }

            prevAIQueueBuffer[i - 16U] = value16;

            len = snprintf(pWrite, (size_t)(MESSAGE_BUFFER_SIZE - total_length),
                           "Core %d, Channel %d, Raw: %lu (%s): %.2f V\r\n",
                           (int)adc_cores[i],
                           (int)adc_channels[i],
                           (unsigned long)all_adc_data[i],
                           sensor_names[i],
                           adc_voltage);
        }

        /* Update message buffer safely */
        if ((len > 0) && ((total_length + (uint32_t)len) < MESSAGE_BUFFER_SIZE))
        {
            total_length += (uint32_t)len;
            pWrite += len;
        }
        else
        {
            break; /* Prevent buffer overflow */
        }
    }

    /* Optional UART logging if needed */
    if (dataChanged && (total_length > 0U))
    {
        /* (void)SERCOM0_USART_Write((uint8_t*)messageAnalog, total_length); */
    }

    /* ---- TCP Frame Send for changed inputs ---- */
    if (changedCount > 0U)
    {
        uint8_t processed = 0U;
        while (processed < changedCount)
        {
            /* Limit batch to 16 updates per frame */
            uint8_t batchCount = (changedCount - processed > 16U) ? 16U : (changedCount - processed);

            /* Construct message header */
            uint8_t header[4U];
            header[0U] = 0x00U;  /* Status: no errors */
            header[1U] = 0x02U;  /* Response type: analog data */
            header[2U] = msgSubTypeAnalogRead; /* Subtype */
            header[3U] = batchCount;

            /* Payload buffer: 6 bytes per channel */
            uint8_t payload[batchCount * 6U];

            for (uint8_t i = 0U; i < batchCount; i++)
            {
                uint8_t index = changedIndexes[processed + i];
                uint16_t port = GetPortFromSerialNumber((uint8_t)(index + 91U));

                uint16_t value;
                if (index < 16U)
                {
                    value = currentTempAIQueueBuffer[index];
                }
                else
                {
                    value = currentAIQueueBuffer[index - 16U];
                }

                /* Pack payload: Port (2B), Reserved (2B), Value (2B) */
                payload[(i * 6U) + 0U] = (uint8_t)((port >> 8U) & 0xFFU);
                payload[(i * 6U) + 1U] = (uint8_t)(port & 0xFFU);
                payload[(i * 6U) + 2U] = 0x00U;
                payload[(i * 6U) + 3U] = 0x00U;
                payload[(i * 6U) + 4U] = (uint8_t)((value >> 8U) & 0xFFU);
                payload[(i * 6U) + 5U] = (uint8_t)(value & 0xFFU);
            }

            /* Compute CRC over header+payload */
            uint16_t messageSize = (uint16_t)(sizeof(header) + (batchCount * 6U) + 2U);
            uint8_t message[messageSize];

            (void) memcpy(message, header, sizeof(header));
            (void) memcpy(&message[sizeof(header)], payload, (batchCount * 6U));

            uint16_t crc = CalculateCRC(message, (uint16_t)(sizeof(header) + (batchCount * 6U)));
            message[sizeof(header) + (batchCount * 6U)]     = (uint8_t)((crc >> 8U) & 0xFFU);
            message[sizeof(header) + (batchCount * 6U) + 1] = (uint8_t)(crc & 0xFFU);

            if (sIOServerSocket != INVALID_SOCKET)
            {
                SYS_CONSOLE_PRINT("Sending Analog Input Change Data with CRC to IO Socket\r\n");

                /* Send frame */
                (void) TCPIP_TCP_ArrayPut(sIOServerSocket, message, messageSize);
            }

            /* Update processed index */
            processed = (uint8_t)(processed + batchCount);
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
    
    //DEBUG
//        uint32_t now = SYS_TIME_CounterGet();   // current tick count
//    if (now - lastTick >= SYS_TIME_MSToCount(100))   // 1 sec elapsed
//    {
//        lastTick = now;
//        SYS_CONSOLE_PRINT("expanderData0: 0x%02X, expanderData1: 0x%02X\r\n", expanderData0, expanderData1);
//
//        /* Print individual pin states */
//        for (uint8_t i = 0; i < 8; i++)
//        {
//            SYS_CONSOLE_PRINT("P0_%d: %s\r\n", i, (expanderData0 & (1 << i)) ? "high" : "low");
//        }
//        for (uint8_t i = 0; i < 3; i++)
//        {
//            SYS_CONSOLE_PRINT("P1_%d: %s\r\n", i, (expanderData1 & (1 << i)) ? "high" : "low");
//        }
//    }
    
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
 
#if HEV_IO_Aggregator
    if (Board_Selection == false)
    {
        Board_Selection = true;
        /* Set Static IP */
        SetStaticIPAddress("192.168.1.231");
    }
#endif

#if Two_Wheeler_IO_Aggregator
    if (Board_Selection == false)
    {
        Board_Selection = true;

        if (Digital_Input_1_Get() == true)
        {
            Two_Wheeler_IO_Aggregator1 = true; /* If 1, set aggregator1 flag true */
            SYS_CONSOLE_PRINT("IO Aggregator1 Two Wheeler Application \r\n");
            SetStaticIPAddress("192.168.1.177");
        }
        else
        {
            Two_Wheeler_IO_Aggregator1 = false; /* If 0, set aggregator1 flag false */
            SYS_CONSOLE_PRINT("IO Aggregator2 Two Wheeler Application \r\n");
            SetStaticIPAddress("192.168.1.178");
        }

        Default_Pin_Status();
    }
#endif    
}

/**
 * @brief Updates and transmits the BDU frame.
 *
 * This function reads limit switch and ground sense data from the digital input buffer,
 * constructs a 4-word frame, converts it to Big-Endian format, and transmits the message
 * via TCP or UDP based on the platform configuration.
 *
 * MISRA C Compliance:
 * - Ensures proper type handling to avoid implicit promotions.
 * - Uses explicit type conversions to prevent unintended behavior.
 * - Includes necessary checks before transmitting data.
 * - Uses explicit function casting to avoid warnings.
 */
void updateBDUFrame(void) 
{
    uint8_t limitSwitchData = 0U;
    uint8_t groundSenseData = 0U;
    uint8_t i = 0U;
    
    /* Extract Limit Switch data (DI32 to DI39) */
    for (i = 0U; i < 8U; i++) 
    {
        limitSwitchData |= (uint8_t)((currentDIQueueBuffer[32U + i] & 0x01U) << i);
    }

    /* Extract Ground Sense data (DI40 to DI48) */
    for (i = 0U; i < 8U; i++) 
    {
        groundSenseData |= (uint8_t)((currentDIQueueBuffer[40U + i] & 0x01U) << i);
    }

    /* Frame structure */
    uint16_t frame[4] = { 0x0D11U, 0x0000U, 0x0000U, 0x0000U };
    frame[3] = (uint16_t)(((uint16_t)limitSwitchData << 8U) | (uint16_t)groundSenseData);

    /* Prepare message in Big-Endian format */
    uint8_t message[8];
    message[0] = (uint8_t)((frame[0] >> 8U) & 0xFFU);
    message[1] = (uint8_t)(frame[0] & 0xFFU);
    message[2] = (uint8_t)((frame[1] >> 8U) & 0xFFU);
    message[3] = (uint8_t)(frame[1] & 0xFFU);
    message[4] = (uint8_t)((frame[2] >> 8U) & 0xFFU);
    message[5] = (uint8_t)(frame[2] & 0xFFU);
    message[6] = (uint8_t)((frame[3] >> 8U) & 0xFFU);
    message[7] = (uint8_t)(frame[3] & 0xFFU);

    /* Send Data */
    #if HEV_IO_Aggregator
        TCPIP_TCP_ArrayPut(sIOServerSocket, message, sizeof(message));
    #elif Two_Wheeler_IO_Aggregator
        if (TCPIP_UDP_PutIsReady(sIOServerSocket) >= (int32_t)sizeof(message)) 
        {
            (void)TCPIP_UDP_ArrayPut(sIOServerSocket, message, sizeof(message));
            (void)TCPIP_UDP_Flush(sIOServerSocket);
            (void)TCPIP_UDP_Discard(sIOServerSocket);

            (void)SYS_CONSOLE_PRINT("Sent BDU Data via UDP\n");
        } 
        else 
        {
            (void)SYS_CONSOLE_PRINT("UDP Socket not ready\n");
        }
    #endif
}

/**
 * @brief Updates and transmits the Relay Output frame.
 *
 * This function reads the states of relay outputs, constructs a 4-word frame, converts it to Big-Endian format,
 * and transmits the message via TCP or UDP based on the platform configuration.
 *
 * MISRA C Compliance:
 * - Ensures explicit type handling and avoids implicit integer promotions.
 * - Uses explicit type conversions for bitwise operations.
 * - Prevents buffer overflows and ensures safe data transmission.
 */
void updateRelayOutputFrame() {    
    uint16_t frame[4];  // Buffer to hold the result
    // Frame format: 0C11 0000 yyyy xxxx
    uint16_t relayStatus = 0; // For Relay Output 1 to 15

    // Retrieve the Relay output states and pack them into the appropriate places in the frame
    // Packing for Relay Output 1 to Relay Output 6 (xxxx)
    relayStatus |= (efuse1_in_Get() << 0); 
    relayStatus |= (efuse2_in_Get() << 1); 
    relayStatus |= (efuse3_in_Get() << 2); 
    relayStatus |= (efuse4_in_Get() << 3);
    relayStatus |= (Relay_Output_1_Get() << 4);
    relayStatus |= (Relay_Output_2_Get() << 5);
    
    // Pack the frame: 0C11 0000 yyyy xxxx
    frame[0] = 0x0C11;   // Fixed header 0C11
    frame[1] = 0x0000;   // Reserved part (0000 0000)
    frame[2] = 0x0000;  // Relay Output 16 to 20 
    frame[3] = relayStatus;  // Relay Output 1 to 15 
    SYS_CONSOLE_PRINT("%04X %04X %04X %04X\n", frame[0], frame[1], frame[2], frame[3]);
    
    if (!TCPIP_UDP_IsConnected(sIOServerSocket))
    {
        SYS_CONSOLE_MESSAGE("Server Connection was closed\r\n");
        return;
    } 
    SYS_CONSOLE_PRINT("Sending Relay Output Data to IO Socket\r\n");

    // Prepare message in correct byte order (Big-Endian format)
    uint8_t message[8];
    message[0] = (frame[0] >> 8) & 0xFF;
    message[1] = frame[0] & 0xFF;
    message[2] = (frame[1] >> 8) & 0xFF;
    message[3] = frame[1] & 0xFF;
    message[4] = (frame[2] >> 8) & 0xFF;
    message[5] = frame[2] & 0xFF;
    message[6] = (frame[3] >> 8) & 0xFF;
    message[7] = frame[3] & 0xFF;

    /* Send data based on platform configuration */
    #if HEV_IO_Aggregator
        (void)TCPIP_TCP_ArrayPut(sIOServerSocket, message, (uint16_t)sizeof(message));
    #elif Two_Wheeler_IO_Aggregator
        if (TCPIP_UDP_PutIsReady(sIOServerSocket) >= (int32_t)sizeof(message)) 
        {
            (void)TCPIP_UDP_ArrayPut(sIOServerSocket, message, (uint16_t)sizeof(message));
            (void)TCPIP_UDP_Flush(sIOServerSocket);  /* Ensure data is sent */
            (void)TCPIP_UDP_Discard(sIOServerSocket);

            (void)SYS_CONSOLE_PRINT("Sent Relay Output Data via UDP\n");
        } 
        else 
        {
            (void)SYS_CONSOLE_PRINT("UDP Socket not ready\n");
        }
    #endif     
}

/**
 * @brief Sends the version number frame via UDP.
 *
 * This function prepares a small frame (3 bytes) containing version data and sends it 
 * over a UDP socket if the buffer is ready.
 *
 * Enhancements:
 * - Ensures safe socket operations.
 * - Adds proper debugging logs.
 * - Improves MISRA C compliance by explicit type casting.
 */
void updateFirmwareVersionNumberFrame() {
    uint8_t message[3];

    /* Extract Major, Minor, and Patch versions from SYS_FW_VERSION */
    message[0] = (SYS_FW_VERSION >> 16) & 0xFF;  // Major version
    message[1] = (SYS_FW_VERSION >> 8) & 0xFF;   // Minor version
    message[2] = (SYS_FW_VERSION) & 0xFF;        // Patch version
#if HEV_IO_Aggregator  
    /* Ensure the socket is ready to send data */
    if (TCPIP_TCP_IsConnected(sIOServerSocket))
    {
        TCPIP_TCP_ArrayPut(sIOServerSocket, message, (uint16_t)sizeof(message));
        TCPIP_TCP_Flush(sIOServerSocket);
        (void)SYS_CONSOLE_PRINT("Sent Version Number Data via TCP: %d.%d.%d\n",
                                message[0], message[1], message[2]);        
    }
    else 
    {
        (void)SYS_CONSOLE_PRINT("TCP Socket not ready\n");
    }
#endif  
#if Two_Wheeler_IO_Aggregator  
    /* Ensure the socket is ready to send data */
    if (TCPIP_UDP_PutIsReady(sIOServerSocket) >= (int32_t)sizeof(message)) 
    {
        (void)TCPIP_UDP_ArrayPut(sIOServerSocket, message, (uint16_t)sizeof(message));
        (void)TCPIP_UDP_Flush(sIOServerSocket);  /* Ensure data is sent */
        (void)TCPIP_UDP_Discard(sIOServerSocket);

        (void)SYS_CONSOLE_PRINT("Sent Version Number Data via UDP: %d.%d.%d\n",
                                message[0], message[1], message[2]);
    } 
    else 
    {
        (void)SYS_CONSOLE_PRINT("UDP Socket not ready\n");
    }
#endif    
}

/**
 * @brief Sends the Hardware version number frame via UDP.
 *
 * This function prepares a small frame (3 bytes) containing version data and sends it 
 * over a UDP socket if the buffer is ready.
 *
 * Enhancements:
 * - Ensures safe socket operations.
 * - Adds proper debugging logs.
 * - Improves MISRA C compliance by explicit type casting.
 */
void updateHardwareVersionNumberFrame() {
    uint8_t message[3];

    /* Extract Major, Minor, and Patch versions from SYS_FW_VERSION */
    message[0] = (SYS_HW_VERSION >> 16) & 0xFF;  // Major version
    message[1] = (SYS_HW_VERSION >> 8) & 0xFF;   // Minor version
    message[2] = (SYS_HW_VERSION) & 0xFF;        // Patch version

#if HEV_IO_Aggregator  
    /* Ensure the socket is ready to send data */
    if (TCPIP_TCP_IsConnected(sIOServerSocket))
    {
        TCPIP_TCP_ArrayPut(sIOServerSocket, message, (uint16_t)sizeof(message));
        TCPIP_TCP_Flush(sIOServerSocket);
        (void)SYS_CONSOLE_PRINT("Sent Version Number Data via TCP: %d.%d.%d\n",
                                message[0], message[1], message[2]);        
    }
    else 
    {
        (void)SYS_CONSOLE_PRINT("TCP Socket not ready\n");
    }
#endif
#if Two_Wheeler_IO_Aggregator      
    /* Ensure the socket is ready to send data */
    if (TCPIP_UDP_PutIsReady(sIOServerSocket) >= (int32_t)sizeof(message)) 
    {
        (void)TCPIP_UDP_ArrayPut(sIOServerSocket, message, (uint16_t)sizeof(message));
        (void)TCPIP_UDP_Flush(sIOServerSocket);  /* Ensure data is sent */
        (void)TCPIP_UDP_Discard(sIOServerSocket);

        (void)SYS_CONSOLE_PRINT("Sent Version Number Data via UDP: %d.%d.%d\n",
                                message[0], message[1], message[2]);
    } 
    else 
    {
        (void)SYS_CONSOLE_PRINT("UDP Socket not ready\n");
    }
#endif    
}
/**
 * @brief Sends the digital output states via UDP.
 *
 * This function reads 20 digital output states, packs them into a structured frame,
 * and sends the data over a UDP socket in Big-Endian format.
 *
 * Enhancements:
 * - Ensures proper bit-packing without misalignment.
 * - Uses loops for better maintainability.
 * - Adheres to MISRA C compliance.
 */
void updateDigitalOutputsFrame() {   
    uint16_t frame[4];  // Buffer to hold the result
    // Frame format: 0E11 0000 yyyy xxxx
    // We need to pack the digital output states from Digital_Output_1_Get() to Digital_Output_20_Get()

    uint16_t doStatus_1 = 0; // For Digital Output 1 to 15
    uint16_t doStatus_2 = 0; // For Digital Output 16 to 20

    // Retrieve the digital output states and pack them into the appropriate places in the frame
    // Packing for Digital Output 1 to Digital Output 15 (xxxx)
    doStatus_1 |= (Digital_Output_1_Get() << 0);  // Digital Output 1
    doStatus_1 |= (Digital_Output_2_Get() << 1);  // Digital Output 2
    doStatus_1 |= (Digital_Output_3_Get() << 2);  // Digital Output 3
    doStatus_1 |= (Digital_Output_4_Get() << 3);  // Digital Output 4
    doStatus_1 |= (Digital_Output_5_Get() << 4);  // Digital Output 5
    doStatus_1 |= (Digital_Output_6_Get() << 5);  // Digital Output 6
    doStatus_1 |= (Digital_Output_7_Get() << 6);  // Digital Output 7
    doStatus_1 |= (Digital_Output_8_Get() << 7);  // Digital Output 8
    doStatus_1 |= (Digital_Output_9_Get() << 8);  // Digital Output 9
    doStatus_1 |= (Digital_Output_10_Get() << 9); // Digital Output 10
    doStatus_1 |= (Digital_Output_11_Get() << 10); // Digital Output 11
    doStatus_1 |= (Digital_Output_12_Get() << 11); // Digital Output 12
    doStatus_1 |= (Digital_Output_13_Get() << 12); // Digital Output 13
    doStatus_1 |= (Digital_Output_14_Get() << 13); // Digital Output 14
    doStatus_1 |= (Digital_Output_15_Get() << 14); // Digital Output 15
    doStatus_1 |= (Digital_Output_16_Get() << 15); // Digital Output 16
    
    // Packing for Digital Output 16 to Digital Output 24 (yyyy)
//    doStatus_2 |= (Digital_Output_16_Get() << 0); // Digital Output 16
    doStatus_2 |= (Digital_Output_17_Get() << 0); // Digital Output 17
    doStatus_2 |= (Digital_Output_18_Get() << 1); // Digital Output 18
    doStatus_2 |= (Digital_Output_19_Get() << 2); // Digital Output 19
    doStatus_2 |= (Digital_Output_20_Get() << 3); // Digital Output 20
    doStatus_2 |= (Digital_Output_21_Get() << 4); // Digital Output 21
    doStatus_2 |= (Digital_Output_22_Get() << 5); // Digital Output 22
    doStatus_2 |= (Digital_Output_23_Get() << 6); // Digital Output 23
    doStatus_2 |= (Digital_Output_24_Get() << 7); // Digital Output 24
    
//    uint32_t reconstructedStatus = ((uint32_t)doStatus_2 << 16) | doStatus_1;
//    SYS_CONSOLE_PRINT("Reconstructed: %08X, Original: %08X\n", reconstructedStatus, doStatus);

    // Pack the frame: 0E11 0000 yyyy xxxx
    frame[0] = 0x0E11;   // Fixed header 0E11
    frame[1] = 0x0000;   // Reserved part (0000 0000)
    frame[2] = doStatus_2;  // Digital Output 17 to 24 
    frame[3] = doStatus_1;  // Digital Output 1 to 16 
    SYS_CONSOLE_PRINT("%04X %04X %04X %04X\n", frame[0], frame[1], frame[2], frame[3]);
    
    if (!TCPIP_UDP_IsConnected(sIOServerSocket))
    {
        SYS_CONSOLE_MESSAGE("Server Connection was closed\r\n");
        return;
    } 
    SYS_CONSOLE_PRINT("Sending Digital Output Data to IO Socket\r\n");

    // Prepare message in correct byte order (Big-Endian format)
    uint8_t message[8];
    message[0] = (frame[0] >> 8) & 0xFF;
    message[1] = frame[0] & 0xFF;
    message[2] = (frame[1] >> 8) & 0xFF;
    message[3] = frame[1] & 0xFF;
    message[4] = (frame[2] >> 8) & 0xFF;
    message[5] = frame[2] & 0xFF;
    message[6] = (frame[3] >> 8) & 0xFF;
    message[7] = frame[3] & 0xFF;

    // Call the TCPIP_TCP_ArrayPut function to send the data
    #if HEV_IO_Aggregator
        TCPIP_TCP_ArrayPut(sIOServerSocket, message, sizeof(message));
    #elif Two_Wheeler_IO_Aggregator
        if (TCPIP_UDP_PutIsReady(sIOServerSocket) >= sizeof(message)) {
            TCPIP_UDP_ArrayPut(sIOServerSocket, message, sizeof(message));
            TCPIP_UDP_Flush(sIOServerSocket);  // Ensure data is sent
            TCPIP_UDP_Discard(sIOServerSocket);

            SYS_CONSOLE_PRINT("Sent Digital Output Data via UDP\n");
        } else {
            SYS_CONSOLE_PRINT("UDP Socket not ready\n");
        }
    #endif     
}

/**
 * @brief Sends the analog input states via UDP.
 *
 * This function reads 15 analog input states, packs them into a structured frame,
 * and sends the data over a UDP socket in Big-Endian format.
 */
void updateAnalogInputsFrame()
{
    uint16_t frame[4];  // Buffer to hold the result
    // Convert the buffer into a 16-bit binary string
    unsigned int analogStatus = 0;
//    for (int i = 0; i < 15; i++) {
//        analogStatus |= (currentAIQueueBuffer[i] << (15 - i));  // Shift the status into the correct bit
//    }

    frame[0] = 0x0B11;   // Fixed header 0B11
    frame[1] = 0x0000;   // Reserved part (0000 0000)
    frame[2] = 0x0000;  // Reserved part (0000 0000) 
    frame[3] = analogStatus;  // Analog 1 to 15 
    SYS_CONSOLE_PRINT("%04X %04X %04X %04X\n", frame[0], frame[1], frame[2], frame[3]);
    
    if (!TCPIP_UDP_IsConnected(sIOServerSocket))
    {
        SYS_CONSOLE_MESSAGE("Server Connection was closed\r\n");
        return;
    } 
    
    SYS_CONSOLE_PRINT("Sending Analog Input Data to IO Socket\r\n");
    // Prepare message in correct byte order (Big-Endian format)
    uint8_t message[8];
    message[0] = (frame[0] >> 8) & 0xFF;
    message[1] = frame[0] & 0xFF;
    message[2] = (frame[1] >> 8) & 0xFF;
    message[3] = frame[1] & 0xFF;
    message[4] = (frame[2] >> 8) & 0xFF;
    message[5] = frame[2] & 0xFF;
    message[6] = (frame[3] >> 8) & 0xFF;
    message[7] = frame[3] & 0xFF;

    /* Send data over TCP or UDP */
    #if HEV_IO_Aggregator
        TCPIP_TCP_ArrayPut(sIOServerSocket, message, sizeof(message));
    #elif Two_Wheeler_IO_Aggregator
        if (TCPIP_UDP_PutIsReady(sIOServerSocket) >= (int32_t)sizeof(message))
        {
            (void)TCPIP_UDP_ArrayPut(sIOServerSocket, message, sizeof(message));
            (void)TCPIP_UDP_Flush(sIOServerSocket);  /* Ensure data is sent */
            (void)TCPIP_UDP_Discard(sIOServerSocket);

            (void)SYS_CONSOLE_PRINT("Sent Analog Input Data via UDP\n");
        }
        else
        {
            (void)SYS_CONSOLE_PRINT("UDP Socket not ready\n");
        }
    #endif     
}

/**
 * @brief Sends the digital input states via UDP.
 *
 * This function reads 60 digital input states, packs them into a structured frame,
 * and sends the data over a UDP socket in Big-Endian format.
 */
void updateDigitalInputsFrame() {    
    uint16_t frame[4];  // Buffer to hold the result
    // Create the frame in the structure 0A11 Digital_Input1 Digital_Input2 Digital_Input3
    // Digital_Input1: Inputs 0 to 15
    // Digital_Input2: Inputs 16 to 31
    // Digital_Input3: Inputs 32 to 47

    uint16_t Digital_Input1 = 0, Digital_Input2 = 0, Digital_Input3 = 0;

    // Process the first 16 digital inputs (Inputs 0 to 15)
    for (int i = 0; i < 16; i++) {
        Digital_Input1 |= (currentDIQueueBuffer[i] << (15 - i));  // Shift and OR to build the 16-bit value
    }

    // Process the next 16 digital inputs (Inputs 16 to 31)
    for (int i = 16; i < 32; i++) {
        Digital_Input2 |= (currentDIQueueBuffer[i] << (31 - i));  // Shift and OR to build the 16-bit value
    }

    // Process the last 16 digital inputs (Inputs 32 to 47)
    for (int i = 32; i < 48; i++) {
        Digital_Input3 |= (currentDIQueueBuffer[i] << (47 - i));  // Shift and OR to build the 16-bit value
    }

    // Pack the frame: 0A11 XXXX YYYY ZZZZ
    frame[0] = 0x0A11;    // Fixed header 0A11
    frame[1] = Digital_Input1;      // 16 bits for the first part (XXXX)
    frame[2] = Digital_Input2;      // 16 bits for the second part (YYYY)
    frame[3] = Digital_Input3;      // 16 bits for the third part (ZZZZ)
    SYS_CONSOLE_PRINT("%04X %04X %04X %04X\n", frame[0], frame[1], frame[2], frame[3]);
    
    /* Ensure socket is connected before sending */
    if (!TCPIP_UDP_IsConnected(sIOServerSocket))
    {
        SYS_CONSOLE_MESSAGE("Server Connection was closed\r\n");
        return;
    }

    SYS_CONSOLE_PRINT("Sending Digital Input Data to IO Socket\r\n");

    // Prepare message in correct byte order (Big-Endian format)
    uint8_t message[8];
    message[0] = (frame[0] >> 8) & 0xFF;
    message[1] = frame[0] & 0xFF;
    message[2] = (frame[1] >> 8) & 0xFF;
    message[3] = frame[1] & 0xFF;
    message[4] = (frame[2] >> 8) & 0xFF;
    message[5] = frame[2] & 0xFF;
    message[6] = (frame[3] >> 8) & 0xFF;
    message[7] = frame[3] & 0xFF;

    /* Send data over TCP or UDP */
    #if HEV_IO_Aggregator
        TCPIP_TCP_ArrayPut(sIOServerSocket, message, sizeof(message));
    #elif Two_Wheeler_IO_Aggregator
        if (TCPIP_UDP_PutIsReady(sIOServerSocket) >= (int32_t)sizeof(message))
        {
            (void)TCPIP_UDP_ArrayPut(sIOServerSocket, message, sizeof(message));
            (void)TCPIP_UDP_Flush(sIOServerSocket);
            (void)TCPIP_UDP_Discard(sIOServerSocket);

            (void)SYS_CONSOLE_PRINT("Sent Digital Input Data via UDP\n");
        }
        else
        {
            (void)SYS_CONSOLE_PRINT("UDP Socket not ready\n");
        }
    #endif  
}

/**
 * @brief Detects changes in digital input states and sends updated values via TCP.
 *
 * This function compares the current digital input buffer (`currentDIQueueBuffer`)
 * with the previous state (`prevDIQueueBuffer`) to determine which inputs have changed.
 * If changes are detected, it sends the updated values in packets with a 
 * predefined structure, including a CRC for data integrity.
 *
 * The data is transmitted in **batches of 16 changes** (due to payload limitations).
 * Each batch consists of a **header, payload (each input change as a tuple), and CRC**.
 *
 * ## Message Structure:
 * - **Header (4 bytes)**:
 *   - Byte 0: Error Code (0x00 for no errors)
 *   - Byte 1: Message Type (0x02 for response)
 *   - Byte 2: Message Subtype (`msgSubTypeDigitalRead`)
 *   - Byte 3: Number of tuples in this batch
 * 
 * - **Payload (Each tuple is 6 bytes, max 16 tuples per batch)**:
 *   - Byte 0-1: Digital Input Port (Big-Endian)
 *   - Byte 2-3: Reserved (0x00 0x00)
 *   - Byte 4: Digital Input Value (1 or 0)
 *   - Byte 5: Reserved (0x00)
 *
 * - **CRC (2 bytes, at the end of the message)**:
 *   - Ensures data integrity for the entire message (excluding the CRC itself).
 *
 * @note If no digital inputs have changed, the function exits without sending data.
 * @note The function ensures data transmission by calling `TCPIP_TCP_Flush()` after sending.
 */
void IsDigitalInputChanged(void)
{
    uint8_t changedCount = 0;
    uint8_t changedIndexes[NUM_DIGITAL_PINS];

    /* Identify changed digital inputs */
    for (uint8_t i = 0; i < NUM_DIGITAL_PINS; i++)
    {
        if (currentDIQueueBuffer[i] != prevDIQueueBuffer[i])
        {
            changedIndexes[changedCount++] = i;
        }
    }

    /* Exit if no changes */
    if (changedCount == 0U)
    {
        return;
    }

    uint8_t processed = 0;

    while (processed < changedCount)
    {
        uint8_t batchCount = (changedCount - processed > MAX_BATCH_SIZE) ? MAX_BATCH_SIZE : (changedCount - processed);

        /* Prepare the message header */
        uint8_t header[HEADER_SIZE] = { 0x00, 0x02, msgSubTypeDigitalRead, batchCount };

        /* Use static arrays to avoid stack overflow */
        static uint8_t payload[MAX_BATCH_SIZE * TUPLE_SIZE];
        static uint8_t message[MESSAGE_MAX_SIZE];

        /* Prepare the payload */
        for (uint8_t i = 0; i < batchCount; i++)
        {
            uint8_t index = changedIndexes[processed + i];
            uint16_t port = GetPortFromSerialNumber(index + 1U); /* Convert serial number to port */

            payload[i * TUPLE_SIZE] = (port >> 8U) & 0xFFU;   /* Port high byte */
            payload[i * TUPLE_SIZE + 1U] = port & 0xFFU;     /* Port low byte */
            payload[i * TUPLE_SIZE + 2U] = 0x00U;           /* Reserved */
            payload[i * TUPLE_SIZE + 3U] = 0x00U;           /* Reserved */
            payload[i * TUPLE_SIZE + 4U] = currentDIQueueBuffer[index]; /* Digital value */
            payload[i * TUPLE_SIZE + 5U] = 0x00U;           /* Reserved */
        }

        /* Calculate message size dynamically */
        uint16_t messageSize = HEADER_SIZE + (batchCount * TUPLE_SIZE) + CRC_SIZE;

        /* Copy header & payload into final message */
        memcpy(message, header, HEADER_SIZE);
        memcpy(message + HEADER_SIZE, payload, batchCount * TUPLE_SIZE);

        /* Calculate CRC */
        uint16_t crc = CalculateCRC(message, messageSize - CRC_SIZE);
        message[messageSize - 2U] = (crc >> 8U) & 0xFFU;
        message[messageSize - 1U] = crc & 0xFFU;

        /* Send message */
        if (sIOServerSocket != INVALID_SOCKET)
        {
            SYS_CONSOLE_PRINT("Sending Digital Input Change Data with CRC to IO Socket\r\n");
            TCPIP_TCP_ArrayPut(sIOServerSocket, message, messageSize);
            TCPIP_TCP_Flush(sIOServerSocket); /* Ensure immediate transmission */
        }

        /* Move to the next batch */
        processed += batchCount;
    }

    /* Update previous state buffer */
//    memcpy(prevDIQueueBuffer, currentDIQueueBuffer, sizeof(prevDIQueueBuffer));
    for (size_t i = 0; i < sizeof(prevDIQueueBuffer); i++) {
        prevDIQueueBuffer[i] = currentDIQueueBuffer[i]; 
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
uint16_t CalculateCRC(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;      // Initial CRC value (all bits set to 1)
    uint16_t polynomial = 0x8005;  // CRC-16 polynomial

    for (uint16_t i = 0; i < length; i++) {
        crc ^= (data[i] << 8);  // Load byte into the CRC register (shifted left by 8 bits)

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {  // Check if the MSB is set
                crc = (crc << 1) ^ polynomial;  // Shift left and XOR with the polynomial
            } else {
                crc <<= 1;  // Otherwise, just shift left
            }
        }
    }

    return crc;  // Return the calculated CRC
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
void ParseInbPacket(uint8_t u8IOBuffer[], uint32_t inbPacketSz) {
    // Initialize variables
    inbMsgSubType = 0x00;
    inbNumTuples = 0;
    inbPortNo = -1;
    inbPortVal = -1;
    inbCrc = -1;

    SYS_CONSOLE_PRINT("Parsing incoming packet...\n");

    // Check if input buffer is NULL
    if (u8IOBuffer == NULL) {
        SYS_CONSOLE_PRINT("Error: Input buffer is NULL\n");
        return;
    }   
    if (u8IOBuffer[0] == 0x02) {
        custom_message = true;
        SYS_CONSOLE_MESSAGE("\n\r####### Bootloader Triggered #######\n\r");

        SYS_CONSOLE_MESSAGE("\n\r####### Program new firmware from Bootloader #######\n\r");

        ramStart[0] = BTL_TRIGGER_PATTERN;
        ramStart[1] = BTL_TRIGGER_PATTERN;
        ramStart[2] = BTL_TRIGGER_PATTERN;
        ramStart[3] = BTL_TRIGGER_PATTERN;

        DCACHE_CLEAN_BY_ADDR(ramStart, 16);

        SYS_RESET_SoftwareReset(); 
        return;        
    }
    if (u8IOBuffer[0] == 0x03) {
        custom_message = true;
        SYS_CONSOLE_PRINT("Soft Reset\n");
        NVIC_SystemReset();
        return;        
    } 
    if (u8IOBuffer[0] == 0x04) {//Serial number
        if (inbPacketSz >= 3) { // Ensure we have enough data
            serialnum = ((uint16_t)u8IOBuffer[1] << 8) | u8IOBuffer[2];
            custom_message = true;
            SYS_CONSOLE_PRINT("Received Serial Number: 0x%04X (%u)\n", serialnum, serialnum);
            saveOutputsToFlash(doStatus, relayStatus,serialnum);
        } else {
            SYS_CONSOLE_PRINT("Error: Incomplete serial number data\n");
        }
        return;
    }
    else if (u8IOBuffer[0] == 0x10)    /* --- ?Give me SN & FW? ----------- */
    {
        custom_message = true;
        /* Assemble reply: 2-byte serial number + 3-byte FW version */
        uint8_t txBuf[5U];

        txBuf[0] = (uint8_t)(serialnum >> 8U);  /* MSB first */
        txBuf[1] = (uint8_t)(serialnum & 0xFFU);

        txBuf[2] = (uint8_t)((SYS_FW_VERSION >> 16U) & 0xFFU); /* Major */
        txBuf[3] = (uint8_t)((SYS_FW_VERSION >>  8U) & 0xFFU); /* Minor */
        txBuf[4] = (uint8_t)( SYS_FW_VERSION         & 0xFFU); /* Patch */
        /* Ensure the socket is ready to send data */
        if (TCPIP_TCP_IsConnected(sIOServerSocket))
        {
            TCPIP_TCP_ArrayPut(sIOServerSocket, txBuf, (uint16_t)sizeof(txBuf));
            TCPIP_TCP_Flush(sIOServerSocket);
            (void)SYS_CONSOLE_PRINT("Sent SN+FW via TCP: SN=0x%04X, FW=%u.%u.%u\n",
                                serialnum,
                                txBuf[2], txBuf[3], txBuf[4]);     
        }
        else 
        {
            (void)SYS_CONSOLE_PRINT("TCP Socket not ready\n");
        }
        return;        
    }    
    // Extract and validate error code
    uint8_t errorCode = u8IOBuffer[0];
    if (errorCode != 0) {
        SYS_CONSOLE_PRINT("Error code: %u\n", errorCode);
    }    
    // Extract and validate message type
    uint8_t messageType = u8IOBuffer[1];
    if (messageType == MSG_TYPE_REQUEST) {
        SYS_CONSOLE_PRINT("Request Message\n");
    } else if (messageType == MSG_TYPE_RESPONSE) {
        SYS_CONSOLE_PRINT("Response Message\n");
    } else {
        SYS_CONSOLE_PRINT("Error: Unknown Message Type\n");
        outbErrorCode = errorInvalidMsg;
        return;
    }

    // Extract and validate message subtype
    inbMsgSubType = u8IOBuffer[2];
    outbMsgSubType = inbMsgSubType;
    SYS_CONSOLE_PRINT("Message Subtype: 0x%02X\n", inbMsgSubType);

    if ((inbMsgSubType != msgSubTypeDigitalRead) && (inbMsgSubType != msgSubTypeDigitalWrite)) {
        SYS_CONSOLE_PRINT("Invalid Message Subtype\n");
        outbErrorCode = errorInvalidMsg;
        return;
    }

    // Extract and validate the number of tuples
    inbNumTuples = u8IOBuffer[3];
    outbNumTuples = inbNumTuples;
    SYS_CONSOLE_PRINT("Number of Tuples: %u\n", inbNumTuples);

    if (inbNumTuples != 1) {
        SYS_CONSOLE_PRINT("Error: Invalid number of tuples\n");
        outbErrorCode = errorInvalidTuples;
        return;
    }

    // Validate the packet size
    if (inbPacketSz != (4 + (inbNumTuples * 6) + 2)) {
        SYS_CONSOLE_PRINT("Error: Invalid packet size\n");
        outbErrorCode = errorInvalidMsg;
        return;
    }

    // Extract and validate the port number
    inbPortNo = (u8IOBuffer[4] << 8) | u8IOBuffer[5];
    outbPortNo = inbPortNo;
    SYS_CONSOLE_PRINT("Port Number: %d\n", inbPortNo);

    // Check if the port number falls within valid ranges
    if (!(inbPortNo >= 1 && inbPortNo <= 110)) {
        SYS_CONSOLE_PRINT("Error: Invalid Port Number\n");
        outbErrorCode = errorInvalidPort;
        return;
    }

    // Extract and validate the port value
    inbPortVal = (u8IOBuffer[6] << 24) | (u8IOBuffer[7] << 16) | (u8IOBuffer[8] << 8) | u8IOBuffer[9];
    SYS_CONSOLE_PRINT("Port Value: %d\n", inbPortVal);

    if ((inbPortVal != 0) && (inbPortVal != 1)) {
        SYS_CONSOLE_PRINT("Error: Invalid Port Value\n");
        outbErrorCode = errorInvalidValue;
        return;
    }

    // Extract and validate CRC
    inbCrc = (u8IOBuffer[10] << 8) | u8IOBuffer[11];
    SYS_CONSOLE_PRINT("CRC: 0x%04X\n", inbCrc);

    // TODO: Implement CRC validation logic if required

    // Parsing completed successfully
    outbErrorCode = errorNoError;
    SYS_CONSOLE_PRINT("Packet parsing completed successfully\n");
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
void ProcessInbPacket() 
{  
    // No processing for any errors thus far
    if (outbErrorCode != errorNoError) {
        return;
    }
    
    if (inbMsgSubType == msgSubTypeDigitalRead) 
    {
        switch (inbPortNo) {
            // **Digital Input Read Handling (DI1 - DI60)**
            case 1:  outbPortVal = Digital_Input_1_Get();  break;
            case 2:  outbPortVal = Digital_Input_2_Get();  break;
            case 3:  outbPortVal = Digital_Input_3_Get();  break;
            case 4:  outbPortVal = Digital_Input_4_Get();  break;
            case 5:  outbPortVal = Digital_Input_5_Get();  break;
            case 6:  outbPortVal = Digital_Input_6_Get();  break;
            case 7:  outbPortVal = Digital_Input_7_Get();  break;
            case 8:  outbPortVal = Digital_Input_8_Get();  break;
            case 9:  outbPortVal = Digital_Input_9_Get();  break;
            case 10: outbPortVal = Digital_Input_10_Get(); break;
            case 11: outbPortVal = Digital_Input_11_Get(); break;
            case 12: outbPortVal = Digital_Input_12_Get(); break;
            case 13: outbPortVal = Digital_Input_13_Get(); break;
            case 14: outbPortVal = Digital_Input_14_Get(); break;
            case 15: outbPortVal = Digital_Input_15_Get(); break;
            case 16: outbPortVal = Digital_Input_16_Get(); break;
            case 17: outbPortVal = Digital_Input_17_Get(); break;
            case 18: outbPortVal = Digital_Input_18_Get(); break;
            case 19: outbPortVal = Digital_Input_19_Get(); break;
            case 20: outbPortVal = Digital_Input_20_Get(); break;
            case 21: outbPortVal = Digital_Input_21_Get(); break;
            case 22: outbPortVal = Digital_Input_22_Get(); break;
            case 23: outbPortVal = Digital_Input_23_Get(); break;
            case 24: outbPortVal = Digital_Input_24_Get(); break;
            case 25: outbPortVal = Digital_Input_25_Get(); break;
            case 26: outbPortVal = Digital_Input_26_Get(); break;
            case 27: outbPortVal = Digital_Input_27_Get(); break;
            case 28: outbPortVal = Digital_Input_28_Get(); break;
            case 29: outbPortVal = Digital_Input_29_Get(); break;
            case 30: outbPortVal = Digital_Input_30_Get(); break;
            case 31: outbPortVal = Digital_Input_31_Get(); break;
            case 32: outbPortVal = Digital_Input_32_Get(); break;
            case 33: outbPortVal = Digital_Input_33_Get(); break;
            case 34: outbPortVal = Digital_Input_34_Get(); break;
            case 35: outbPortVal = Digital_Input_35_Get(); break;
            case 36: outbPortVal = Digital_Input_36_Get(); break;
            case 37: outbPortVal = Digital_Input_37_Get(); break;
            case 38: outbPortVal = Digital_Input_38_Get(); break;
            case 39: outbPortVal = Digital_Input_39_Get(); break;
            case 40: outbPortVal = Digital_Input_40_Get(); break;
            case 41: outbPortVal = Digital_Input_41_Get(); break;
            case 42: outbPortVal = Digital_Input_42_Get(); break;
            case 43: outbPortVal = Digital_Input_43_Get(); break;
            case 44: outbPortVal = Digital_Input_44_Get(); break;
            case 45: outbPortVal = Digital_Input_45_Get(); break;
            case 46: outbPortVal = Digital_Input_46_Get(); break;
            case 47: outbPortVal = Digital_Input_47_Get(); break;
            case 48: outbPortVal = Digital_Input_48_Get(); break;
            case 49: outbPortVal = Digital_Input_49_Get(); break;
            case 50: outbPortVal = Digital_Input_50_Get(); break;
            case 51: outbPortVal = Digital_Input_51_Get(); break;
            case 52: outbPortVal = Digital_Input_52_Get(); break;
            case 53: outbPortVal = Digital_Input_53_Get(); break;
            case 54: outbPortVal = Digital_Input_54_Get(); break;
            case 55: outbPortVal = Digital_Input_55_Get(); break;
            case 56: outbPortVal = Digital_Input_56_Get(); break;
            case 57: outbPortVal = Digital_Input_57_Get(); break;
            case 58: outbPortVal = Digital_Input_58_Get(); break;
            case 59: outbPortVal = Digital_Input_59_Get(); break;
            case 60: outbPortVal = Digital_Input_60_Get(); break;
            
            // **Digital Output Handling (DO1 - DO20) with Read**
            case 61: outbPortVal = Digital_Output_1_Get();  break;
            case 62: outbPortVal = Digital_Output_2_Get();  break;
            case 63: outbPortVal = Digital_Output_3_Get();  break;
            case 64: outbPortVal = Digital_Output_4_Get();  break;
            case 65: outbPortVal = Digital_Output_5_Get();  break;
            case 66: outbPortVal = Digital_Output_6_Get();  break;
            case 67: outbPortVal = Digital_Output_7_Get();  break;
            case 68: outbPortVal = Digital_Output_8_Get();  break;
            case 69: outbPortVal = Digital_Output_9_Get();  break;
            case 70: outbPortVal = Digital_Output_10_Get(); break;
            case 71: outbPortVal = Digital_Output_11_Get(); break;
            case 72: outbPortVal = Digital_Output_12_Get(); break;
            case 73: outbPortVal = Digital_Output_13_Get(); break;
            case 74: outbPortVal = Digital_Output_14_Get(); break;
            case 75: outbPortVal = Digital_Output_15_Get(); break;
            case 76: outbPortVal = Digital_Output_16_Get(); break;
            case 77: outbPortVal = Digital_Output_17_Get(); break;
            case 78: outbPortVal = Digital_Output_18_Get(); break;
            case 79: outbPortVal = Digital_Output_19_Get(); break;
            case 80: outbPortVal = Digital_Output_20_Get(); break;
            case 81: outbPortVal = Digital_Output_21_Get(); break;
            case 82: outbPortVal = Digital_Output_22_Get(); break;
            case 83: outbPortVal = Digital_Output_23_Get(); break;
            case 84: outbPortVal = Digital_Output_24_Get(); break;   
            
            // **Relay Output Handling (R1 - R6) with Read**
            case 85:  outbPortVal = efuse1_in_Get();  break;
            case 86:  outbPortVal = efuse2_in_Get();  break;
            case 87:  outbPortVal = efuse3_in_Get();  break;
            case 88:  outbPortVal = efuse4_in_Get();  break;
            case 89:  outbPortVal = Relay_Output_1_Get();  break;
            case 90:  outbPortVal = Relay_Output_2_Get();  break;            
        }
    }
    else if (inbMsgSubType == msgSubTypeDigitalWrite)
    {
        if (inbPortNo >= 61 && inbPortNo <= 90)  
        {
            switch (inbPortNo) {
                //Digital Output
                case 61:  inbPortVal ? Digital_Output_1_Set()  : Digital_Output_1_Clear();  break;
                case 62:  inbPortVal ? Digital_Output_2_Set()  : Digital_Output_2_Clear();  break;
                case 63:  inbPortVal ? Digital_Output_3_Set()  : Digital_Output_3_Clear();  break;
                case 64:  inbPortVal ? Digital_Output_4_Set()  : Digital_Output_4_Clear();  break;
                case 65:  inbPortVal ? Digital_Output_5_Set()  : Digital_Output_5_Clear();  break;
                case 66:  inbPortVal ? Digital_Output_6_Set()  : Digital_Output_6_Clear();  break;
                case 67:  inbPortVal ? Digital_Output_7_Set()  : Digital_Output_7_Clear();  break;
                case 68:  inbPortVal ? Digital_Output_8_Set()  : Digital_Output_8_Clear();  break;
                case 69:  inbPortVal ? Digital_Output_9_Set()  : Digital_Output_9_Clear();  break;
                case 70:  inbPortVal ? Digital_Output_10_Set() : Digital_Output_10_Clear(); break;
                case 71:  inbPortVal ? Digital_Output_11_Set() : Digital_Output_11_Clear(); break;
                case 72:  inbPortVal ? Digital_Output_12_Set() : Digital_Output_12_Clear(); break;
                case 73:  inbPortVal ? Digital_Output_13_Set() : Digital_Output_13_Clear(); break;
                case 74:  inbPortVal ? Digital_Output_14_Set() : Digital_Output_14_Clear(); break;
                case 75:  inbPortVal ? Digital_Output_15_Set() : Digital_Output_15_Clear(); break;
                case 76:  inbPortVal ? Digital_Output_16_Set() : Digital_Output_16_Clear(); break;
                case 77:  inbPortVal ? Digital_Output_17_Set() : Digital_Output_17_Clear(); break;
                case 78:  inbPortVal ? Digital_Output_18_Set() : Digital_Output_18_Clear(); break;
                case 79:  inbPortVal ? Digital_Output_19_Set() : Digital_Output_19_Clear(); break;
                case 80:  inbPortVal ? Digital_Output_20_Set() : Digital_Output_20_Clear(); break;
                case 81:  inbPortVal ? Digital_Output_21_Set() : Digital_Output_21_Clear(); break;
                case 82:  inbPortVal ? Digital_Output_22_Set() : Digital_Output_22_Clear(); break;
                case 83:  inbPortVal ? Digital_Output_23_Set() : Digital_Output_23_Clear(); break;
                case 84:  inbPortVal ? Digital_Output_24_Set() : Digital_Output_24_Clear(); break;
                
                //Relay Output
                case 85:  inbPortVal ? efuse1_in_Set()  : efuse1_in_Clear();  break;
                case 86:  inbPortVal ? efuse2_in_Set()  : efuse2_in_Clear();  break;
                case 87:  inbPortVal ? efuse3_in_Set()  : efuse3_in_Clear();  break;
                case 88:  inbPortVal ? efuse4_in_Set()  : efuse4_in_Clear();  break;
                case 89:  inbPortVal ? Relay_Output_1_Set()  : Relay_Output_1_Clear();  break;
                case 90:  inbPortVal ? Relay_Output_2_Set()  : Relay_Output_2_Clear();  break;                
                default:
                    outbErrorCode = errorInvalidPort;
                    return;
            }        
        }
    }
    return;
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
    outbPacketBuf[0] = outbErrorCode;
    outbPacketBuf[1] = 0x02;
    outbPacketBuf[2] = outbMsgSubType;
    outbPacketBuf[3] = outbNumTuples;
    outbPacketBuf[4] = (outbPortNo >> 8) & 0xFF;
    outbPacketBuf[5] = outbPortNo & 0xFF;
    outbPacketBuf[6] = 0x00;
    outbPacketBuf[7] = 0x00;
    outbPacketBuf[8] = 0x00;
    if (inbMsgSubType == msgSubTypeDigitalWrite){
        outbPacketBuf[9] = (inbPortVal != 0) ? 0x01 : 0x00;
    }
    else{
        outbPacketBuf[9] = (outbPortVal != 0) ? 0x01 : 0x00;
    }
    outbPacketBuf[10] = 0x00;
    outbPacketBuf[11] = 0x00;

    outbPacketSz = 4 + (outbNumTuples * 6) + 2;

    // Calculate CRC over the packet excluding the last two bytes
    uint16_t crc = CalculateCRC((uint8_t *)outbPacketBuf, outbPacketSz - 2);  // Exclude CRC bytes

    // Update the last two bytes with the CRC value
    outbPacketBuf[outbPacketSz - 2] = (crc >> 8) & 0xFF;  // MSB
    outbPacketBuf[outbPacketSz - 1] = crc & 0xFF;         // LSB
    
    // Print outbPacketBuf contents as hex for debugging
    SYS_CONSOLE_PRINT("outbPacketBuf: ");
    for (uint32_t i = 0; i < outbPacketSz; i++)
    {
        SYS_CONSOLE_PRINT("%02X ", outbPacketBuf[i]);
    }
    SYS_CONSOLE_PRINT("\n");

    // Transmit the packet over TCP
    TCPIP_TCP_ArrayPut(sIOServerSocket, (const uint8_t*)outbPacketBuf, outbPacketSz);
}

/**
 * @brief Parses received data and executes corresponding actions.
 *
 * This function processes the received `u8IOBuffer` and performs actions 
 * based on predefined command types. It supports handling commands for:
 * - Digital input request
 * - Analog input request
 * - Relay output request
 * - BDU pin request
 * - Digital output request
 * - Version number retrieval
 * 
 * Additionally, it handles specific commands:
 * - `0xCC` (Relay_Write Command): Controls 15 relay outputs.
 * - `0xAA` (Digital Write Command): Controls 20 digital outputs.
 *
 * After processing, it sends an acknowledgment via UDP for `0xCC` and `0xAA` commands.
 *
 * @param[in] u8IOBuffer Received data buffer containing the command and parameters.
 */
void Parse_Wheeler_Received_Data(uint8_t u8IOBuffer[]) {
    if (u8IOBuffer[1] == 0xA1) {
        SYS_CONSOLE_PRINT("Digital Input Request Command Received\n");
        updateDigitalInputsFrame();
    }
    if (u8IOBuffer[1] == 0xB1) {
        SYS_CONSOLE_PRINT("Analog Input Request Command Received\n");
        updateAnalogInputsFrame();
    }
    if (u8IOBuffer[1] == 0xC1) {
        SYS_CONSOLE_PRINT("Relay Output Request Command Received\n");
        updateRelayOutputFrame();
    }
    if (u8IOBuffer[1] == 0xD1) {
        SYS_CONSOLE_PRINT("BDU pin Request Command Received\n");
        updateBDUFrame();
    }    
    if (u8IOBuffer[1] == 0xE1) {
        SYS_CONSOLE_PRINT("Digital Output Request Command Received\n");
        updateDigitalOutputsFrame();
    }    
    // Check if the first byte is the fixed header 0xCC (Relay Command)
    if (u8IOBuffer[0] == 0xCC) {
        SYS_CONSOLE_PRINT("Relay_Write Command Received\n");

      // Extract relay control data (the next 4 bytes: "CC00 0000 0000 XXXX")
      uint16_t relayStatus = (u8IOBuffer[6] << 8) | u8IOBuffer[7]; // XXXX from u8IOBuffer
        
        // Iterate through each relay (assuming 15 relays based on the example)
        for (int i = 0; i < RELAY_COUNT; i++) {
            // Check if the corresponding bit is 1 or 0
            if (relayStatus & (1 << i)) {
                // Relay is ON
                switch (i) {
                    case 0: efuse1_in_Set(); break;
                    case 1: efuse2_in_Set(); break;
                    case 2: efuse3_in_Set(); break;
                    case 3: efuse4_in_Set(); break;
                    case 4: Relay_Output_1_Set(); break;
                    case 5: Relay_Output_2_Set(); break;                    
                    default: break;
                }
            } else {
                // Relay is OFF
                switch (i) {
                    case 0: efuse1_in_Clear(); break;
                    case 1: efuse2_in_Clear(); break;
                    case 2: efuse3_in_Clear(); break;
                    case 3: efuse4_in_Clear(); break;
                    case 4: Relay_Output_1_Clear(); break;
                    case 5: Relay_Output_2_Clear(); break;                    
                    default: break;
                }
            }
        }
      // Send acknowledgment via UDP
        uint8_t message[2] = {0xCC, 0xCC};  // Data to be sent

        if (TCPIP_UDP_PutIsReady(sIOServerSocket) >= sizeof(message)) {
            TCPIP_UDP_ArrayPut(sIOServerSocket, message, sizeof(message));
            TCPIP_UDP_Flush(sIOServerSocket);  // Ensure data is sent
            TCPIP_UDP_Discard(sIOServerSocket);

            SYS_CONSOLE_PRINT("Sent Data via UDP: CCCC\n");
        } else {
            SYS_CONSOLE_PRINT("UDP Socket not ready\n");
        }
    }
    
    // Check if the first byte is the fixed header 0xAA (Digital Output Command)
    if (u8IOBuffer[0] == 0xAA) {
        SYS_CONSOLE_PRINT("Digital Write Command Received\n");

        // Extract the 16-bit digital output control data (the last 2 bytes: "XXXX")
        uint16_t digitalWriteStatus = (u8IOBuffer[6] << 8) | u8IOBuffer[7]; // XXXX from u8IOBuffer[6] and u8IOBuffer[7]

        // We now have the first 16 bits controlling digital outputs 1-16
        // Iterate through each digital output (assuming 20 digital outputs)
        for (int i = 0; i < 16; i++) {
            // Check if the corresponding bit is 1 or 0 (0-15 bits)
            if (digitalWriteStatus & (1 << i)) {
                // Digital Output is ON
                switch (i) {
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
                    default: break;
                }
            } else {
                // Digital Output is OFF
                switch (i) {
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
                    default: break;
                }
            }
        }

        // Extract the byte X (the 6th byte, u8IOBuffer[5])
        uint8_t extraControl = u8IOBuffer[5];  // This is the byte X that controls outputs 17-20

        // Iterate over the bits of extraControl and control the corresponding digital outputs (17 to 20)
        for (int i = 0; i < 4; i++) {
            // Shift the appropriate bit into the least significant position
            if (extraControl & (1 << i)) {
                // If the bit is 1, turn the corresponding digital output ON
                switch (i) {
                    case 0: Digital_Output_17_Set(); break;
                    case 1: Digital_Output_18_Set(); break;
                    case 2: Digital_Output_19_Set(); break;
                    case 3: Digital_Output_20_Set(); break;
                    default: break;
                }
            } else {
                // If the bit is 0, turn the corresponding digital output OFF
                switch (i) {
                    case 0: Digital_Output_17_Clear(); break;
                    case 1: Digital_Output_18_Clear(); break;
                    case 2: Digital_Output_19_Clear(); break;
                    case 3: Digital_Output_20_Clear(); break;
                    default: break;
                }
            }
        }
        // Send acknowledgment via UDP
        uint8_t message[2] = {0xAA, 0xAA};  // Data to be sent

        if (TCPIP_UDP_PutIsReady(sIOServerSocket) >= sizeof(message)) {
            TCPIP_UDP_ArrayPut(sIOServerSocket, message, sizeof(message));
            TCPIP_UDP_Flush(sIOServerSocket);  // Ensure data is sent
            TCPIP_UDP_Discard(sIOServerSocket);

            SYS_CONSOLE_PRINT("Sent Data via UDP: AAAA\n");
        } else {
            SYS_CONSOLE_PRINT("UDP Socket not ready\n");
        }        
    }
}

/**
 * @brief   Monitors specific digital input pins and controls an output pin based on their state.
 * 
 * This function checks if the predefined digital input pins are active (logic HIGH). 
 * If any of them are active, it sets Digital_Output_1 HIGH and waits for 1 second.
 * If none are active, it clears Digital_Output_1 and waits for 10 seconds.
 * 
 * If `Two_Wheeler_IO_Aggregator1` is true, the function ensures Digital_Output_1 is OFF.
 * 
 * Note: The function uses `vTaskDelay`, which blocks execution for the specified time.
 *       Consider optimizing with an event-driven approach if needed.
 */
void Active_Safety_Pins()
{
    if (!Two_Wheeler_IO_Aggregator1)  // Avoid redundant comparisons (== false)
    {
        // Read all digital inputs once
        bool input_active = Digital_Input_1_Get() || Digital_Input_2_Get() || 
                            Digital_Input_3_Get() || Digital_Input_8_Get() || 
                            Digital_Input_10_Get() || Digital_Input_12_Get() || 
                            Digital_Input_16_Get() || Digital_Input_17_Get() || 
                            Digital_Input_18_Get();

        if (input_active)
        {    
            Digital_Output_1_Set();  // Activate the output if any input is active
            vTaskDelay(1000);
        }
        else
        {
            Digital_Output_1_Clear();
            vTaskDelay(10000);
        }
    }
    else 
    {
        Digital_Output_1_Clear();
        vTaskDelay(10000);
    }
}

/**
 * @brief   Configures the default state of digital and relay outputs based on the aggregator type.
 * 
 * This function checks whether the `Two_Wheeler_IO_Aggregator` is defined and applies 
 * the appropriate default pin configurations. It also updates the version number frame.
 * 
 * - If `Two_Wheeler_IO_Aggregator1` is **true**, it enables **Digital_Output_1** 
 *   and sets all **15 relay outputs**.
 * - If `Two_Wheeler_IO_Aggregator1` is **false**, it sets multiple **digital outputs** 
 *   (excluding Digital_Output_1).
 * 
 * @note The function currently does nothing if `HEV_IO_Aggregator` is defined. 
 *       Future functionality may be added inside its preprocessor block.
 */
void Default_Pin_Status()
{
#if HEV_IO_Aggregator

#endif
    
#if Two_Wheeler_IO_Aggregator
    updateFirmwareVersionNumberFrame();
    if(Two_Wheeler_IO_Aggregator1) {    
        // Enable Digital Output 1
        Digital_Output_1_Set();

        // Enable all relay outputs (1-15)
        Relay_Output_1_Set();
        Relay_Output_2_Set();
        Relay_Output_3_Set();
        Relay_Output_4_Set();
        Relay_Output_5_Set();
        Relay_Output_6_Set();
        Relay_Output_7_Set();
        Relay_Output_8_Set();
        Relay_Output_9_Set();
        Relay_Output_10_Set();
        Relay_Output_11_Set();
        Relay_Output_12_Set();
        Relay_Output_13_Set();
        Relay_Output_14_Set();
        Relay_Output_15_Set();
    }
    else {
        // Enable multiple digital outputs except Digital_Output_1
        Digital_Output_2_Set();
        Digital_Output_3_Set();
        Digital_Output_4_Set();
        Digital_Output_5_Set();
        Digital_Output_6_Set();
        Digital_Output_7_Set();
        Digital_Output_8_Set();
        Digital_Output_9_Set();
        Digital_Output_10_Set();
        Digital_Output_11_Set();
        Digital_Output_12_Set();
        Digital_Output_13_Set();
        Digital_Output_14_Set();
        Digital_Output_15_Set();
        Digital_Output_16_Set();
        Digital_Output_18_Set();
        Digital_Output_19_Set();
    }
#endif
}

void SendFixedUDPTestData(void)
{
    const char testMessage[] = "Test UDP Data";  // Fixed message to send
    uint16_t messageLength = sizeof(testMessage) - 1;  // Exclude null terminator

    if (!TCPIP_UDP_IsConnected(sIOServerSocket))
    {
        SYS_CONSOLE_MESSAGE("Server Connection was closed\r\n");
        return;
    }

    // Check if we can send data
    int16_t wMaxPut = TCPIP_UDP_PutIsReady(sIOServerSocket);
    if (wMaxPut < messageLength)
    {
        SYS_CONSOLE_MESSAGE("UDP TX buffer is full, cannot send test data\r\n");
        return;
    }

    // Send the fixed test message
    TCPIP_UDP_ArrayPut(sIOServerSocket, (uint8_t*)testMessage, messageLength);
    TCPIP_UDP_Flush(sIOServerSocket);  // Ensure data is sent immediately

    SYS_CONSOLE_PRINT("Sent test UDP message: '%s'\r\n", testMessage);
//    vTaskDelay(1000);
}

/**
 * @brief   Receives and processes incoming UDP packets.
 * 
 * This function checks for available data in the UDP socket buffer. If data is available:
 * - It reads the incoming bytes into a buffer (`u8IOBuffer`).
 * - The received data is null-terminated to ensure proper string handling.
 * - The data is printed to the console for debugging.
 * - The function `Parse_Wheeler_Received_Data()` is called to process the received data.
 * 
 * After processing, the UDP receive buffer is discarded to free up space for new data.
 */
void UDP_Receive()
{
    int16_t availableBytes = TCPIP_UDP_GetIsReady(sIOServerSocket);
    
    if (availableBytes > 0) 
    {
        uint8_t u8IOBuffer[64];  // Adjust buffer size as needed
        memset(u8IOBuffer, 0, sizeof(u8IOBuffer));

        int bytesReceived = TCPIP_UDP_ArrayGet(sIOServerSocket, u8IOBuffer, sizeof(u8IOBuffer) - 1);

        if (bytesReceived > 0) 
        {
            u8IOBuffer[bytesReceived] = '\0';  // Null-terminate the received data
            SYS_CONSOLE_PRINT("Received UDP Data: %s (Length: %d)\r\n", u8IOBuffer, bytesReceived);
            Parse_Wheeler_Received_Data(u8IOBuffer);
        }

        TCPIP_UDP_Discard(sIOServerSocket);  // Free the RX buffer
    }
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
void vIOHandlerServerTask(void *pvParameters) {    
    // Configure IO expanders
    vConfigureIOexpanders();

    // Read stored output states from Flash memory
    SYS_CONSOLE_PRINT("Reading stored output states from Flash\n");
    readOutputsFromFlash();
    
    // Print function entry for debugging
    SYS_CONSOLE_PRINT("In Function: %s\r\n", __FUNCTION__);  

    // Open server socket based on the aggregator type
    #if HEV_IO_Aggregator
        sIOServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, IO_SERVER_PORT_HEV, 0);
    #elif Two_Wheeler_IO_Aggregator
        sIOServerSocket = TCPIP_UDP_ServerOpen(IP_ADDRESS_TYPE_IPV4, IO_SERVER_PORT_Wheeler, 0);
    #endif    
    
    if (sIOServerSocket == INVALID_SOCKET) {
        SYS_CONSOLE_PRINT("Error opening IO server socket\r\n");
        vTaskDelete(NULL);
        return;
    }
for (uint8_t i = 0; i < (NUM_ANALOG_PINS - NUM_TEMPERATURE_ANALOG_PINS); i++)
{
    prevAIQueueBuffer[i] = 0xFFFFU;
}

    while (true) {
        // Read digital and analog inputs
        readDigitalInputs();
        ReadAllAnalogInputPins();
        #if HEV_IO_Aggregator
            uint8_t u8IOBuffer[256] = {0};
            if (TCPIP_TCP_IsConnected(sIOServerSocket)) {
                int16_t bytesRead = TCPIP_TCP_ArrayGet(sIOServerSocket, u8IOBuffer, sizeof(u8IOBuffer));

                if (bytesRead > 0) {
                    SYS_CONSOLE_PRINT("Got Data on IO Server Handler (TCP)\r\n");
                    // Read and process incoming commands if any
                    ParseInbPacket(u8IOBuffer,bytesRead);  
                    if(custom_message == false)
                    {
                        ProcessInbPacket();

                        // Prepare and dispatch response to incoming commands
                        PrepareDispatchOutbPacket();

                        // Reset incoming/outgoing data buffers before ending the loop
                        ResetInbOutbData();
                        StoreRelay_DigitalOutputsFrame();
                    }
                    custom_message = false;
                }

                if (!TCPIP_TCP_IsConnected(sIOServerSocket) || TCPIP_TCP_WasDisconnected(sIOServerSocket)) {
                    SYS_CONSOLE_PRINT("\r\nTCP Connection Closed\r\n");
                    TCPIP_TCP_Close(sIOServerSocket);
                    sIOServerSocket = INVALID_SOCKET;
                }
            }
            // Ensure reconnection logic is handled correctly
            if (sIOServerSocket == INVALID_SOCKET) {
                SYS_CONSOLE_PRINT("TCP server socket disconnected, attempting to reconnect...\r\n");

                // Attempt to open the socket again
                sIOServerSocket = TCPIP_TCP_ServerOpen(IP_ADDRESS_TYPE_IPV4, IO_SERVER_PORT_HEV, 0);

                if (sIOServerSocket == INVALID_SOCKET) {
                    SYS_CONSOLE_PRINT("Failed to reopen IO socket, retrying...\r\n");
                } else {
                    SYS_CONSOLE_PRINT("Reconnected successfully IO Socket!\r\n");
                }

                vTaskDelay(RECONNECT_DELAY_MS);
            }   
            // Check if digital or analog input state has changed
            IsDigitalInputChanged();
        #endif

        #if Two_Wheeler_IO_Aggregator
            // Handle UDP-based input processing
            UDP_Receive();
//                IOExpander1_00_Data();
//                SendFixedUDPTestData();            
            // Update input and output frames
            updateDigitalInputsFrame();
            updateAnalogInputsFrame();
            updateRelayOutputFrame();
            updateDigitalOutputsFrame();
            updateBDUFrame();
            
            // Store relay and digital output states
            StoreRelay_DigitalOutputsFrame();
            
            // Activate safety-related pins
            Active_Safety_Pins();                
        #endif       
        vTaskDelay(800); // Add a small delay to avoid consuming too much CPU time
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
/* *****************************************************************************
 End of File
 */
