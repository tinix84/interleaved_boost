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
#include "DSP2803x_Cla_defines.h"
#include "ringbuffer.h"
#include "commands.h"
#include "application.h"
#include "device.h"
#include "shared.h"
#include "DCL.h"


#define CLARAM0_ENABLE  1
#define CLARAM1_ENABLE  1

//CLA ISRs

//Linker defined vars
extern Uint16 Cla1Prog_Start;
extern Uint16 Cla1funcsLoadStart;
extern Uint16 Cla1funcsLoadEnd;
extern Uint16 Cla1funcsRunStart;
extern Uint16 Cla1funcsLoadSize;
extern Uint16 Cla1mathTablesLoadStart;
extern Uint16 Cla1mathTablesRunStart;
extern Uint16 Cla1mathTablesLoadSize;

// Prototype statements for functions found within this file.


// Global variables used in this example
void main(void)
{


    int32_t err = NO_ERROR;

    // Step 1. Initialize System Control:
    err = device_init();   // Device Life support & GPIO

    //Copy over the CLA code and Tables
    memcpy(&Cla1funcsRunStart, &Cla1funcsLoadStart, (Uint32)&Cla1funcsLoadSize);
    memcpy(&Cla1mathTablesRunStart, &Cla1mathTablesLoadStart, (Uint32)&Cla1mathTablesLoadSize);

    /*  Compute all CLA task vectors */
    EALLOW;
    Cla1Regs.MVECT1 = (Uint16)((Uint32)&Cla1Task1 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT2 = (Uint16)((Uint32)&Cla1Task2 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT3 = (Uint16)((Uint32)&Cla1Task3 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT4 = (Uint16)((Uint32)&Cla1Task4 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT5 = (Uint16)((Uint32)&Cla1Task5 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT6 = (Uint16)((Uint32)&Cla1Task6 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT7 = (Uint16)((Uint32)&Cla1Task7 - (Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT8 = (Uint16)((Uint32)&Cla1Task8 - (Uint32)&Cla1Prog_Start);
    EDIS;

    //  Step 3 : Mapping CLA tasks
    /*  All tasks are enabled and will be started by an ePWM trigger
     *  Map CLA program memory to the CLA and enable software breakpoints
     */
    EALLOW;
    Cla1Regs.MPISRCSEL1.bit.PERINT1SEL   = CLA_INT1_NONE;
    Cla1Regs.MPISRCSEL1.bit.PERINT2SEL   = CLA_INT2_NONE;
    Cla1Regs.MPISRCSEL1.bit.PERINT3SEL   = CLA_INT3_NONE;
    Cla1Regs.MPISRCSEL1.bit.PERINT4SEL   = CLA_INT4_NONE;
    Cla1Regs.MPISRCSEL1.bit.PERINT5SEL   = CLA_INT5_NONE;
    Cla1Regs.MPISRCSEL1.bit.PERINT6SEL   = CLA_INT6_NONE;
    Cla1Regs.MPISRCSEL1.bit.PERINT7SEL   = CLA_INT7_NONE;
    Cla1Regs.MPISRCSEL1.bit.PERINT8SEL   = CLA_INT8_NONE;
    Cla1Regs.MIER.all                    = 0x00FF;
    EDIS;


    /* Switch the CLA program space to the CLA and enable software forcing
     * Also switch over CLA data ram 0 and 1
     */
    EALLOW;
    Cla1Regs.MMEMCFG.bit.PROGE   = 1;
    Cla1Regs.MCTL.bit.IACKE  = 1;
    Cla1Regs.MMEMCFG.bit.RAM0E   = CLARAM0_ENABLE;
    Cla1Regs.MMEMCFG.bit.RAM1E   = CLARAM1_ENABLE;
    EDIS;


    application_init(); // application initial values

    // Step 6. IDLE loop. Just sit and loop forever (optional):
    for(;;)
    {

        err = application_serialHandler();

        Cla1ForceTask1andWait();

    };
}



//===========================================================================
// No more.
//===========================================================================
