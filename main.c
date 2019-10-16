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


#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

#include "ringbuffer.h"
#include "commands.h"
#include "application.h"
#include "device.h"
#include "DCL.h"



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
