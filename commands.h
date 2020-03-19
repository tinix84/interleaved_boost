#ifndef COMMANDS_H_
#define COMMANDS_H_

typedef enum {
    echo = 0,
    multi = 1,
    /* General commands (0 - 49) */
    setFrequency = 2,
    setDuty = 3,
    enablePhaseAll = 4,
    disablePhaseAll = 5,
    enableULS = 6,
    enableUHS = 7,
    enableVLS = 8,
    enableVHS = 9,
    disableULS = 10,
    disableUHS = 11,
    disableVLS = 12,
    disableVHS = 13,
    setDeadtime = 14,
    setOutputVoltage = 15,
    disableU = 16,
    disableV = 17,
    /* ERROR commands (250-255)*/
    errorLength = 250,
    errorChecksum = 251,
    error = 255
}commSerialCommands_t;

#endif


