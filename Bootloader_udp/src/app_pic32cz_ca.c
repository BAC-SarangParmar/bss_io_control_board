/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_pic32cz_ca.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "app_pic32cz_ca.h"
#include "definitions.h"
#define PHASE_1 0
#define PHASE_2 1
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_PIC32CZ_CA_Initialize function.

    Application strings and buffers are be defined outside this structure.
*/

APP_PIC32CZ_CA_DATA app_pic32cz_caData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/

#define BTL_TRIGGER_PATTERN (0x5048434DUL)

static uint32_t *ramStart = (uint32_t *)BTL_TRIGGER_RAM_START;

bool bootloader_Trigger(void)
{
    uint32_t i;

    // Cheap delay. This should give at leat 1 ms delay.
    for (i = 0; i < 2000; i++)
    {
        asm("nop");
    }

    /* Check for Bootloader Trigger Pattern in first 16 Bytes of RAM to enter
     * Bootloader.
     */
    if (BTL_TRIGGER_PATTERN == ramStart[0] && BTL_TRIGGER_PATTERN == ramStart[1] &&
        BTL_TRIGGER_PATTERN == ramStart[2] && BTL_TRIGGER_PATTERN == ramStart[3])
    {
        ramStart[0] = 0;

        DCACHE_CLEAN_BY_ADDR(ramStart, 4);

        return true;
    }

    /* Check for Switch press to enter Bootloader */
    if (BUTTON_Get() == 1)
    {
        return true;
    }

    return false;
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_PIC32CZ_CA_Initialize ( void )

  Remarks:
    See prototype in app_pic32cz_ca.h.
 */

void APP_PIC32CZ_CA_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    app_pic32cz_caData.state = APP_PIC32CZ_CA_STATE_INIT;



    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_PIC32CZ_CA_Tasks ( void )

  Remarks:
    See prototype in app_pic32cz_ca.h.
 */

void APP_PIC32CZ_CA_Tasks ( void )
{
    static uint32_t lastTick = 0;
    uint32_t currentTick = SYS_TIME_CounterGet();   // Harmony system time counter
    
    /* Check the application's current state. */
    switch ( app_pic32cz_caData.state )
    {
        /* Application's initial state. */
        case APP_PIC32CZ_CA_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized)
            {

                app_pic32cz_caData.state = APP_PIC32CZ_CA_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_PIC32CZ_CA_STATE_SERVICE_TASKS:
        {
            // Toggle LED every 1 second
            if ((currentTick - lastTick) >= SYS_TIME_MSToCount(1000))   // 1000 ms
            {
                LED_Toggle();
                WDT_Clear();
                lastTick = currentTick;
            }
            break;
        }

        /* TODO: implement your application state machine.*/


        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

void AppKSZ8863ResetFunction(const struct DRV_ETHPHY_OBJECT_BASE_TYPE* pBaseObj, DRV_HANDLE handle)
{

    uint32_t delay = 50000;
#if PHASE_1
    Ethernet_Reset_Clear();
#endif  
#if PHASE_2    
    Ethernet_Reset_Set();
#endif
  
    while (delay)
    {
        delay--;
    }
#if PHASE_1
    Ethernet_Reset_Set();
#endif
#if PHASE_2     
    Ethernet_Reset_Clear();
#endif
  
    delay = 500;
    while (delay)
    {
        delay--;
    }

}
/*******************************************************************************
 End of File
 */
