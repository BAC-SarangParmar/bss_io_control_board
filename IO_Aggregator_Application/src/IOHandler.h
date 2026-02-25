/*******************************************************************************
 * AppIOHandler Application Header File
 *
 * Company:
 *   Bacancy - SunMobility
 *
 * File Name:
 *   AppIOHandler.h
 *
 * Summary:
 *   This header file provides function prototypes and definitions for IO handling.
 *
 * Description:
 *   This header file includes function prototypes, macros, and global variables
 *   required for IO Aggregator operations. It defines configurations for
 *   I2C-based IO expanders (TCA9539), ADC handling, and digital input/output control.
 *******************************************************************************/

#ifndef _APP_IO_HANDLER_H
#define _APP_IO_HANDLER_H

// *****************************************************************************
// Section: Included Files
// *****************************************************************************
#include <stddef.h>  // Defines NULL
#include <stdbool.h> // Defines true

// *****************************************************************************
// IO Server Task Configuration
// *****************************************************************************
#define IO_SERVER_HANDLER_HEAP_DEPTH      1024  // Heap size for IO handler task
#define IO_SERVER_HANDLER_TASK_PRIORITY   ((tskIDLE_PRIORITY + 2))       // Lowest priority   // Task priority level

// *****************************************************************************
// IO Aggregator Configuration
// *****************************************************************************
#define HEV_IO_Aggregator           1 // Flag for HEV IO Aggregator mode
#define Two_Wheeler_IO_Aggregator   0  // Flag for Two-Wheeler IO Aggregator mode

#if HEV_IO_Aggregator
#define SYS_VERSION_STR     "0.0.1"
#define SYS_FW_VERSION      0x000001U
#define SYS_HW_VERSION      0x000102U
#endif
    
#if Two_Wheeler_IO_Aggregator 
#define SYS_VERSION_STR     "0.1.3"
#define SYS_FW_VERSION      0x000103U
#define SYS_HW_VERSION      0x000102U
#endif
// IO Server Port Configuration (Same port used for both configurations)
#define IO_SERVER_PORT_HEV          8888
#define IO_SERVER_PORT_Wheeler      8888

// Delay for reattempting connection (in milliseconds)
#define RECONNECT_DELAY_MS          5000  

// *****************************************************************************
// I2C Address Definitions for IO Expander (TCA9539)
// *****************************************************************************
#define TCA9539_I2C_ADDRESS     0x75 // I2C address of first IO Expander

// *****************************************************************************
// TCA9539 Register Definitions
// *****************************************************************************

// Input Port Registers (Read Input States)
#define INPUT_PORT_REG     0x00  // Register for input port (P00 to P07)
#define INPUT_PORT_REG_2   0x01  // Register for second input port (P10 to P17)

// Output Port Registers (Control Output States)
#define TCA9539_OUTPUT_PORT_0  0x02  // Port 0 output register
#define TCA9539_OUTPUT_PORT_1  0x03  // Port 1 output register

// config Port Registers 
#define TCA9539_CONFIG_PORT0      0x06
#define TCA9539_CONFIG_PORT1      0x07

// *****************************************************************************
// ADC Configuration
// *****************************************************************************
#define ADC_VREF             (3.3f)  // ADC reference voltage
#define REF_RESISTOR    4700U
#define COEFF_A    0.0011279f
#define COEFF_B    0.00023429f
#define COEFF_C    0.000000087298f
#define ADC_TO_VOLT_CONVERTER(x) (x * (ADC_VREF / ADC_MAX_VALUE))
#define NUM_TEMPERATURE_ANALOG_PINS (16)
#define NUM_ANALOG_PINS      (4)    // Total number of ADC channels

#define PT100_Sensor    1
#define NTC_Sensor      0
#define ALL_ANALOG_PINS (20U)
#define MESSAGE_BUFFER_SIZE (2048U)

#define BTL_TRIGGER_PATTERN (0x5048434DUL)
#define BTL_TRIGGER_RAM_START  0x20020000U
extern uint32_t *ramStart;
// External declarations for ADC handling
extern const char* sensor_names[NUM_ANALOG_PINS];
extern const ADC_CORE_NUM adc_cores[NUM_ANALOG_PINS];
extern const ADC_CHANNEL_NUM adc_channels[NUM_ANALOG_PINS];
extern volatile uint32_t adc_data[NUM_ANALOG_PINS]; // Array to store ADC results

///IRC expander handling 
extern uint8_t i2cTxBuf[2];
extern uint8_t i2cRxBuf[2];
extern bool i2cTransferDone;
extern uint32_t lastTick;
extern char messageAnalog[MESSAGE_BUFFER_SIZE];
extern uint32_t total_length;
// *****************************************************************************
// Function Prototypes
// *****************************************************************************
extern uint32_t digitalOutputs;
extern uint16_t relayOutputs;

typedef struct {
    uint32_t magic;
    uint32_t digitalOutputs;
    uint16_t relayOutputs;
    uint16_t serialNumber;  //Serial number
    //CAN
    uint16_t canPorts[6];
    uint32_t canBaudRates[6];
    //RS485
    uint16_t rs485Ports[2];
    struct {
        uint32_t baudRate;
        uint8_t dataBits;
        char parity;
        uint8_t stopBits;
    } rs485Config[2];
} flash_data_t;

// Extern declaration
extern flash_data_t writeData;

extern uint32_t doStatus;
extern uint16_t relayStatus;
extern uint16_t serialnum;
void saveOutputsToFlash(uint32_t digital, uint16_t relay,uint16_t serialnum);
uint16_t MovingAverage_Update(uint8_t channel, uint16_t newValue);
/**
 * @brief Writes data to the TCA9539 IO Expander via I2C.
 *
 * @param address The I2C address of the IO expander.
 * @param wrData Pointer to the data buffer to be written.
 * @param wrLength Number of bytes to write.
 * @return True if the write operation is successful, false otherwise.
 */
bool SERCOM7_I2C_Write(uint16_t address, uint8_t* wrData, uint32_t wrLength);

/**
 * @brief Reads data from the TCA9539 IO Expander via I2C.
 *
 * @param address The I2C address of the IO expander.
 * @param rdData Pointer to the buffer where read data will be stored.
 * @param rdLength Number of bytes to read.
 * @return True if the read operation is successful, false otherwise.
 */
bool SERCOM7_I2C_Read(uint16_t address, uint8_t* rdData, uint32_t rdLength);

/**
 * @brief Initializes and manages IO operations.
 */
void vIOHandler(void);

/**
 * @brief Checks for changes in digital input values.
 */
void IsDigitalInputChanged(void);

/**
 * @brief Reads the analog values from all configured ADC channels.
 */
void ReadAllAnalogInputPins(void);

/**
 * @brief Configures IO Expander 1 ports.
 */
bool IOExpander1_Configure_Ports(void);

/**
 * @brief Reads and processes data from IO Expander 1.
 */
void IOExpander1_Data(void);

/**
 * @brief Checks the input state of the IO expander.
 *
 * @param expanderData Pointer to buffer storing expander input states.
 */
void checkIOExpanderInput(uint8_t *expanderData);

/**
 * @brief Activates safety-related pins.
 */
void Active_Safety_Pins(void);

/**
 * @brief  Reads an 8-bit register value from the TCA9539 I/O Expander.
 *
 * @param  reg  Register address of the TCA9539 (e.g., input port, output port, polarity, configuration).
 * @return uint8_t  Value read from the specified register.
 */
uint8_t TCA9539_ReadRegister(uint8_t reg);

#endif /* _APP_IO_HANDLER_H */

/*******************************************************************************
 * End of File
 *******************************************************************************/
