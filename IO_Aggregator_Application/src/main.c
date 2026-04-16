/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );
#if HEV_IO_Aggregator
    SYS_CONSOLE_MESSAGE(" ------------------------------ \r\n");
    SYS_CONSOLE_PRINT(" IO Aggregator HEV Application Ver:%s [0x%08x]\r\n",SYS_VERSION_STR, SYS_FW_VERSION);
    SYS_CONSOLE_MESSAGE(" ------------------------------ \r\n"); 
#endif
    
#if Two_Wheeler_IO_Aggregator    
    SYS_CONSOLE_MESSAGE(" ------------------------------ \r\n");
    SYS_CONSOLE_PRINT(" IO Aggregator Two Wheeler Application Ver:%s [0x%08x]\r\n",SYS_VERSION_STR, SYS_FW_VERSION);
    SYS_CONSOLE_MESSAGE(" ------------------------------ \r\n");    
#endif

     /* Set WDT timeout period (e.g., PER = CYC16384) */
    WDT_TimeoutPeriodSet(WDT_CONFIG_PER_CYC16384_Val);
   
     /* Enable the Watchdog Timer */
    WDT_Enable();
    
    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

