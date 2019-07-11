//----------------------------------------------------------------------------------
//  FILE:           ILBST-Main.C
//
//  Description:    2-Ph Interleaved boost
//                  The file drives duty on PWM1A and PWM1B using C28x
//                  C28x ISR is triggered by the (PWM 2) CPU Timer interrupt
//
//  Version:        1.1
//
//  Target:         TMS320F2803x(PiccoloB),
//
//----------------------------------------------------------------------------------
//  Copyright Texas Instruments 02-15-2015
//---------------------------------------------------------------------------------
//  Revision History:
//----------------------------------------------------------------------------------
//  Date      | Description / Status
//----------------------------------------------------------------------------------
// Feb 13 , 2013  - File created (SC)
// Sept 15, 2015 - ILPFC PowerSUITE related changes
//----------------------------------------------------------------------------------
//
// PLEASE READ - Useful notes about this Project

// Although this project is made up of several files, the most important ones are:
//   "main.C" - this file
//      - Application Initialization, Peripheral config,
//      - Application management
//      - Slower background code loops and Task scheduling
//   "device.C
//      - Device Initialization, e.g. Clock, PLL, WD, GPIO mapping
//      - Peripheral clock enables
//      - DevInit file will differ per each F28xxx device series, e.g. F280x, F2833x,
//   "ILBST-Settings.h"
//      - Global defines (settings) project selections are found here
//      - This file is referenced by both C and ASM files.
//
// Code is made up of sections, e.g. "FUNCTION PROTOTYPES", "VARIABLE DECLARATIONS" ,..etc
//  each section has FRAMEWORK and USER areas.
//  FRAMEWORK areas provide useful ready made "infrastructure" code which for the most part
//  does not need modification, e.g. Task scheduling, ISR call, GUI interface support,...etc
//  USER areas have functional example code which can be modified by USER to fit their appl.
//
// Code can be compiled with various build options (Incremental Builds IBx), these
//  options are selected in file "ILPFC-Settings.h".  Note: "Rebuild All" compile
//  tool bar button must be used if this file is modified.
//----------------------------------------------------------------------------------

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "ringbuffer.h"
#include "commands.h"
#include "application.h"
#include "device.h"



// Prototype statements for functions found within this file.


// Global variables used in this example
void main(void)
{

    int32_t err = NO_ERROR;

    // Step 1. Initialize System Control:
    err = device_init();   // Device Life support & GPIO
    application_init(); // application initial values

    // Step 6. IDLE loop. Just sit and loop forever (optional):
    for(;;)
    {

        err = application_serialHandler();

    };
}



//===========================================================================
// No more.
//===========================================================================
