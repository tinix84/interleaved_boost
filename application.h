/*
 * control.h
 *
 *  Created on: 23.07.2018
 *      Author: ies
 */

#ifndef APPLICATION_H_
#define APPLICATION_H_

/* Standard imports */
#include <stdint.h>
/* Common imports */
#include "ringbuffer.h"
#include "commands.h"
#include "error.h"

/* Project imports */
//#include "ILBST_Base-Settings.h"
#include "defines.h"
#include "device.h"

/* Serial frame state machine enum */
typedef enum __commFrameStates_e{
    state_idle, state_start, state_command,
    state_length, state_data, state_checksum, state_abort
}commFrameStates_e;


/*
 * Serial Frame typedef
 *
 * Byte 0: '0x55' Start Byte
 * Byte 3: 0x01 Command
 * Byte 1: 0x00 Data Length (<256)
 * Byte 4: 0x02 Data Byte 0
 * Byte 5: 0x03 Data Byte 1
 * Byte 6: 0x02 Data Byte 2
 * Byte 7: 0x03 Data Byte 3
 * ...
 * Byte x: Checksum (XOR of all bytes)
 */
typedef struct __commSerialFrame_t{
    uint16_t start;
    commSerialCommands_t command;
    uint16_t length;
    uint16_t data[COMM_MAX_DATA_LENGTH];
    uint16_t checksum;
    uint16_t checksumFlag;
    uint16_t transferFlag;
    uint16_t processedFlag;
}commSerialFrame_t;


/* General typedefs */



typedef struct __array4{
    uint16_t data0;
    uint16_t data1;
    uint16_t data2;
    uint16_t data3;
} array4;

extern int32_t application_init(void);
extern int32_t application_serialHandler(void);


/*
 ****************************************************************************
 * General setter and getter functions
 ****************************************************************************
 */


/*
 ****************************************************************************
 * Auxiliary functions
 ****************************************************************************
 */
static inline int32_t application_blink(void)
{
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    return 0;
}



#endif /* APPLICATION_H_ */

