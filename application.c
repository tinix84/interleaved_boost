/*
 * application.c
 *
 *  Created on: 09.05.2019
 *      Author: tinivella
 */
#include <stdint.h>

#include "application.h"



/* Private functions */
/* Initialization functions */
static int32_t application_initSerial(void);

/* Serial driver functions */
static int32_t application_processFrame(commSerialFrame_t *frame);
static int32_t application_getFrame(ringbuffer_t *rbuf, commSerialFrame_t *frame);
static int32_t application_putFrame(ringbuffer_t *rbuf, commSerialFrame_t *frame);

/* Miscellaneous functions */

/*
 * Initialization sequence for the control module.
 * Initialize all state machines and set them to startup mode.
 */
int32_t application_init(void)
{
    extern uint32_t actual_duty_cnt;
    extern uint32_t actual_pwm_period;
    extern uint32_t actual_phdly_cnt;
    extern uint32_t actual_deadtime_cnt;

    int32_t err = NO_ERROR;

    /* Initialize system monitor handler. */
    if (err == NO_ERROR)
    {
        //here all the measurements are init
    }
    /* Initialize serial handler. */
    if (err == NO_ERROR)
    {
        err = application_initSerial();
    }

    /* Initialize variable of application code. */
    if (err == NO_ERROR)
    {
        //Here each FSM is init
        actual_duty_cnt = EPwm1Regs.CMPA.half.CMPA;
        actual_phdly_cnt = EPwm1Regs.TBPHS.half.TBPHS;
        actual_pwm_period = EPwm1Regs.TBPRD;
        actual_deadtime_cnt = EPwm1Regs.DBFED;
    }

    /* Error handler for the above */
    if (err != NO_ERROR)
    {
        //application_errorHandler(err);
        for (;;);
        /* watchdog will bite here */
    }

    /* From here the global interrupts can get enabled. */

    return err;
}

/*
 * Initialize serial interface state machine
 */
static int32_t application_initSerial(void)
{

    extern commSerialFrame_t serialFrame;
    extern commFrameStates_e rxstate;
    extern commFrameStates_e txstate;

    rxstate = state_idle;
    txstate = state_idle;

    serialFrame.start = 0U;
    serialFrame.command = echo;
    serialFrame.checksum = 0U;
    serialFrame.checksumFlag = 0U;
    serialFrame.transferFlag = 0U;
    serialFrame.processedFlag = 1U; /* prevent processing at startup */

    return NO_ERROR;
}

/*
 * Serial handler:
 * - Receive serial data from SCI
 * - Transmit serial data to SCI
 * -
 */
int32_t application_serialHandler(void)
{

    extern commSerialFrame_t serialFrame;
    extern ringbuffer_t SCIATXBufferStruct;
    extern ringbuffer_t SCIARXBufferStruct;

    static int32_t status = 1;
    static uint16_t foo = 0;

    /* Receive data from SCI in interrupt routine*/
    if (ringbuffer_length(&SCIARXBufferStruct) >= 8)
    {
        /* Process received data (if any) */
        application_getFrame(&SCIARXBufferStruct, &serialFrame);

        application_processFrame(&serialFrame);

        /* Process data for transmitting (if any) */
        application_putFrame(&SCIATXBufferStruct, &serialFrame);
    }

    /* Transmit data to SCI */
    if (!ringbuffer_empty(&SCIATXBufferStruct) && (SciaRegs.SCICTL2.bit.TXRDY==1))
    {
        //TODO: post process
        ringbuffer_get(&SCIATXBufferStruct, &foo);
        SciaRegs.SCITXBUF=foo; // Send data
    }

    status = 0;
    return status;
}





/*
 * Frame processing
 *
 * Each frame has to be processede only once
 */
int32_t application_processFrame(commSerialFrame_t *frame)
{
    extern uint32_t actual_duty_cnt;
    extern uint32_t actual_pwm_period;
    extern uint32_t actual_phdly_cnt;
    extern uint32_t actual_deadtime_cnt;
    extern uint32_t foo2;

    uint32_t new_duty_cnt = 0;
    uint32_t new_pwm_period = 0;
    uint32_t new_phdly_cnt = 0;
    uint32_t new_deadtime_cnt = 0;

    array4 temp_4;
    uint16_t data_offset = 0;
    uint16_t command_offset = 0;
    commSerialCommands_t command;
    uint16_t * data_address_p;

    /*
     * Process a single command or loop for multi commands
     */

    /* reset variables for multi command */
    //    data_offset = 0U;
    //    command_offset = 0U;

    while(frame->processedFlag == 0U)
    {

        if (frame->checksumFlag == 1U)
        {
            /* Normal operation */
            if (frame->length == 4U)
            {
                data_offset = 0;
                command_offset = 0;
                command = frame->command;
            }
            else
            {
                command = errorLength;
            }
        }
        else
        {
            command = errorChecksum;
        }
        /* Set data address pointer to point to first address location */
        data_address_p = (frame->data + data_offset);

        /* Execute command */
        switch(command)
        {
        case echo:
            /* send back same frame */
            break;
            /*
             ******************************************************************
             * General commands
             ******************************************************************
             */
        case setFrequency:
            //TODO: EPwm1Regs.TBPRD
            new_pwm_period = (EPWM_A_INIT_PERIOD*25/data_address_p[0]);
            EPwm1Regs.TBPRD = (uint16_t) new_pwm_period;
            EPwm2Regs.TBPRD = (uint16_t) new_pwm_period;
            new_duty_cnt = (actual_duty_cnt*new_pwm_period/actual_pwm_period);
            EPwm1Regs.CMPA.half.CMPA = (uint16_t)new_duty_cnt; // adjust duty for output EPWM1A
            EPwm2Regs.CMPA.half.CMPA = (uint16_t)new_duty_cnt; // adjust duty for output EPWM2A
            new_phdly_cnt = new_pwm_period;
            EPwm2Regs.TBPHS.half.TBPHS = new_phdly_cnt;
            actual_duty_cnt = new_duty_cnt;
            actual_pwm_period = new_pwm_period;
            actual_phdly_cnt = new_phdly_cnt;

            break;

        case setDuty:
            new_duty_cnt = (uint32_t)(data_address_p[0])*actual_pwm_period/100;
            EPwm1Regs.CMPA.half.CMPA = (uint16_t)new_duty_cnt;
            EPwm2Regs.CMPA.half.CMPA = (uint16_t)new_duty_cnt;
            actual_duty_cnt = new_duty_cnt;
            break;

        case setDeadtime:
            foo2 = ((uint32_t)data_address_p[0]+256*(uint32_t)data_address_p[1]);
            new_deadtime_cnt = (uint32_t)( foo2 / EPWM_CLK_PERIOD_NS);
            EPwm1Regs.DBFED = (uint16_t)new_deadtime_cnt;
            EPwm1Regs.DBRED = (uint16_t)new_deadtime_cnt;
            EPwm2Regs.DBFED = (uint16_t)new_deadtime_cnt;; // FED = 20 TBCLKs
            EPwm2Regs.DBRED = (uint16_t)new_deadtime_cnt;; // RED = 20 TBCLKs
            actual_deadtime_cnt = new_deadtime_cnt;
            break;

        case enablePhaseAll:
            device_setALLDriverEnable();
            break;

        case disablePhaseAll:
            device_setALLDriverDisable();
            break;

        case enableULS:
            device_driverEnableULS();
            break;

        case enableUHS:
            device_driverEnableUHS();
            break;

        case enableVLS:
            device_driverEnableVLS();
            break;

        case enableVHS:
            device_driverEnableVHS();
            break;

        case disableULS:
            device_driverDisableULS();
            break;

        case disableUHS:
            device_driverDisableUHS();
            break;

        case disableVLS:
            device_driverDisableVLS();
            break;

        case disableVHS:
            device_driverDisableVHS();
            break;


            /*
             ******************************************************************
             * ERROR commands
             ******************************************************************
             */
        case errorLength:
            frame->command = errorLength;
            frame->length = 4;
            temp_4.data0 = 0U;
            temp_4.data1 = 0U;
            temp_4.data2 = 0U;
            temp_4.data3 = 0U;
            memcpy(frame->data, &temp_4, 4U);
            break;
        case errorChecksum:
            frame->command = errorChecksum;
            frame->length = 4;
            temp_4.data0 = 0U;
            temp_4.data1 = 0U;
            temp_4.data2 = 0U;
            temp_4.data3 = 0U;
            memcpy(frame->data, &temp_4, 4U);
            break;
        default:
            frame->command = multi;
            frame->length = 4;
            temp_4.data0 = 0U;
            temp_4.data1 = 0U;
            temp_4.data2 = 0U;
            temp_4.data3 = 0U;
            memcpy(frame->data, &temp_4, 4U);
            break;
        } /* end switch */


        frame->start = COMM_START;
        frame->transferFlag = 1U;
        frame->processedFlag = 1U;
        command_offset = 0U;
        data_offset = 0U;
        break; /* break while */
    } /* end while */
    return 0;
}

int32_t application_getFrame(ringbuffer_t *rbuf, commSerialFrame_t *frame)
{
    extern commFrameStates_e rxstate;
    extern commFrameStates_e txstate;


    static int16_t ret = 1;
    static uint16_t temp, idx, checksum;

    /* While there is data in the ringbuffer, populate frame. */
    while(!ringbuffer_empty(rbuf))
    {
        switch(rxstate)
        {
        case state_idle:
            /* get tail data of RX buffer */
            ringbuffer_data(rbuf, &temp, rbuf->tail);
            /* check if it is a start byte */
            if (temp == COMM_START)
            {
                rxstate = state_start;
                frame->checksumFlag = 0U;
                frame->transferFlag = 0U;
                frame->processedFlag = 0U;
                checksum = 0U;
                idx = 0;
            }
            else
            {
                /* error, no valid frame, throw data away*/
                ringbuffer_get(rbuf, &temp);
            }
            break;
        case state_start:
            /* get start byte */
            ringbuffer_get(rbuf, &temp);
            frame->start = temp;
            checksum ^= temp;
            rxstate = state_command;
            break;
        case state_command:
            /* get command byte */
            ringbuffer_get(rbuf, &temp);
            frame->command = (commSerialCommands_t)temp;
            checksum ^= temp;
            rxstate = state_length;
            break;
        case state_length:
            /* get length byte */
            ringbuffer_get(rbuf, &temp);
            frame->length = temp;
            checksum ^= temp;
            rxstate = state_data;
            if (frame->length == 0U)
            {
                rxstate = state_checksum;
            }
            if (frame->length > COMM_MAX_DATA_LENGTH)
            {
                /* Error! Data too long for buffer */
                frame->command = errorLength;
                rxstate = state_abort;
            }
            break;
        case state_data:
            while (idx<frame->length)
            {
                if (ringbuffer_empty(rbuf)) break;
                ringbuffer_get(rbuf, &temp);
                frame->data[idx] = temp;
                checksum ^= temp;
                idx++;
            }
            /* leave state only when all data was received */
            if(idx >= (frame->length))
            {
                rxstate = state_checksum;
            }
            break;
        case state_checksum:
            /* get checksum byte */
            ringbuffer_get(rbuf, &temp);
            frame->checksum = temp;
            if (checksum == frame->checksum)
            {
                frame->checksumFlag = 1U;
                ret = 0;
            }
            else
            {
                frame->checksumFlag = 0U;
                ret = 1;
            }
            rxstate = state_idle;
            break;
        case state_abort:
            while (!ringbuffer_empty(rbuf))
            {
                ringbuffer_get(rbuf, &temp);
                /* stop at frame start */
                ringbuffer_data(rbuf, &temp, rbuf->tail);
                if (temp == COMM_START) break;
            }
            rxstate = state_idle;
            break;
        }

        /* Break while loop */
        if (rxstate == state_idle)
        {
            break;
        }
    }

    return ret;
}


/*
 * Decode complete frame into ringbuffer.
 */
int32_t application_putFrame(ringbuffer_t *rbuf, commSerialFrame_t *frame)
{

    extern commFrameStates_e rxstate;
    extern commFrameStates_e txstate;

    int32_t ret = 1;
    uint16_t temp, idx, checksum;

    if(rbuf && frame)
    {

        /* While there is data in the ringbuffer, populate frame. */
        while(!ringbuffer_full(rbuf))
        {
            switch(txstate)
            {
            case state_idle:
                /* check if frame is ready to transfer */
                if ((frame->transferFlag == 1U))
                {
                    txstate = state_start;
                    frame->transferFlag = 0U;
                    checksum = 0U;
                    idx = 0U;
                }
                break;
            case state_start:
                /* put start byte */
                temp = frame->start;
                checksum ^= temp;
                ringbuffer_put(rbuf, temp);
                txstate = state_command;
                break;
            case state_command:
                /* put command byte */
                temp = (uint16_t)frame->command;
                checksum ^= temp;
                ringbuffer_put(rbuf, temp);
                txstate = state_length;
                break;
            case state_length:
                /* put length byte */
                temp = frame->length;
                checksum ^= temp;
                ringbuffer_put(rbuf, temp);
                txstate = state_data;
                if (frame->length == 0U)
                {
                    txstate = state_checksum;
                }
                break;
            case state_data:
                while(idx < frame->length)
                {
                    if (ringbuffer_full(rbuf)) break;
                    temp = frame->data[idx];
                    checksum ^= temp;
                    ringbuffer_put(rbuf, temp);
                    idx++;
                }
                if(idx >= (frame->length))
                {
                    txstate = state_checksum;
                }
                break;
            case state_checksum:
                /* put checksum byte */
                temp = checksum;
                ringbuffer_put(rbuf, temp);
                txstate = state_idle;
                break;
            default:
                txstate = state_idle;
                break;
            }

            /* Break while loop */
            if (txstate == state_idle)
            {
                break;
            }
        }
        ret = 0;
    }
    else /* Null pointer passed */
    {
        ret = 1;
    }


    return ret;
}


