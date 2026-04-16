
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "definitions.h"
// Function to parse CAN configuration
void parseCanConfig(cJSON *canConfigArray) {
    int size = cJSON_GetArraySize(canConfigArray);
    for (int i = 0; i < size; i++) {
        cJSON *canConfig = cJSON_GetArrayItem(canConfigArray, i);

        // Parse "can" and "canSpeed"
        cJSON *can = cJSON_GetObjectItem(canConfig, "can");
        cJSON *canSpeed = cJSON_GetObjectItem(canConfig, "canSpeed");

        // If the key is present, print the value
        if (can && canSpeed) {
            SYS_CONSOLE_PRINT("CAN: %s, CAN Speed: %s\n", can->valuestring, canSpeed->valuestring);
        } else {
            SYS_CONSOLE_PRINT("Some fields are missing in CAN config.\n");
        }
    }
}

// Function to parse Mapping configuration
void parseMappingConfig(cJSON *mappingArray) {
    int size = cJSON_GetArraySize(mappingArray);
    for (int i = 0; i < size; i++) {
        cJSON *mapping = cJSON_GetArrayItem(mappingArray, i);

        // Parse "type", "interface", and "tcpPort"
        cJSON *type = cJSON_GetObjectItem(mapping, "type");
        cJSON *interface = cJSON_GetObjectItem(mapping, "interface");
        cJSON *tcpPort = cJSON_GetObjectItem(mapping, "tcpPort");

        if (type && interface && tcpPort) {
            SYS_CONSOLE_PRINT("Type: %s, Interface: %s, TCP Port: %s\n", type->valuestring, interface->valuestring, tcpPort->valuestring);
        } else {
            SYS_CONSOLE_PRINT("Some fields are missing in Mapping config.\n");
        }
    }
}

// Function to parse UART configuration
void parseUartConfig(cJSON *uartConfigArray) {
    int size = cJSON_GetArraySize(uartConfigArray);
    for (int i = 0; i < size; i++) {
        cJSON *uartConfig = cJSON_GetArrayItem(uartConfigArray, i);

        // Parse UART configuration fields
        cJSON *uart = cJSON_GetObjectItem(uartConfig, "uart");
        cJSON *baudRate = cJSON_GetObjectItem(uartConfig, "baudRate");
        cJSON *dataBits = cJSON_GetObjectItem(uartConfig, "dataBits");
        cJSON *parity = cJSON_GetObjectItem(uartConfig, "parity");
        cJSON *stopBits = cJSON_GetObjectItem(uartConfig, "stopBits");

        if (uart && baudRate && dataBits && parity && stopBits) {
            SYS_CONSOLE_PRINT("UART: %s, Baud Rate: %s, Data Bits: %s, Parity: %s, Stop Bits: %s\n",
                   uart->valuestring, baudRate->valuestring, dataBits->valuestring, parity->valuestring, stopBits->valuestring);
        } else {
            SYS_CONSOLE_PRINT("Some fields are missing in UART config.\n");
        }
    }
}

// Function to parse the overall configuration
void parseConfig(const char *jsonString) {
    cJSON *json = cJSON_Parse(jsonString);
    if (!json) {
        SYS_CONSOLE_PRINT("Error parsing JSON\n");
        return;
    }

    // Parse CAN Config
    cJSON *canConfigArray = cJSON_GetObjectItem(json, "canConfig");
    if (canConfigArray) {
        parseCanConfig(canConfigArray);
    }

    // Parse Mapping
    cJSON *mappingArray = cJSON_GetObjectItem(json, "mapping");
    if (mappingArray) {
        parseMappingConfig(mappingArray);
    }

    // Parse UART Config
    cJSON *uartConfigArray = cJSON_GetObjectItem(json, "uartConfig");
    if (uartConfigArray) {
        parseUartConfig(uartConfigArray);
    }

    // Free JSON object
    cJSON_Delete(json);
}
