#ifndef COMMANDS_H_
#define COMMANDS_H_

typedef enum {
    echo = 0,
    multi = 1,
    /* General commands (0 - 49) */
    setFrequency = 2,
    setDuty = 3,
    /* ERROR commands (250-255)*/
    errorLength = 250,
    errorChecksum = 251,
    error = 255
}commSerialCommands_t;

#endif
