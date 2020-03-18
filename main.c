
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "ringbuffer.h"
#include "commands.h"
#include "application.h"
#include "device.h"
#include "DCL.h"


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
