/*
 * ringbuffer.c
 *
 *  Created on: Jul 19, 2018
 *      Author: fkyburz
 *
 *      data in rbuf[head] is not valid (post imcrement)
 *      data in rbuf[tail] is valid
 *
 */

//#include "ringbuffer.h"

#include "ringbuffer.h"
//#include "device.h"

int32_t ringbuffer_reset(ringbuffer_t * rbuf);
int32_t ringbuffer_put(ringbuffer_t * rbuf, uint16_t data);
int32_t ringbuffer_get(ringbuffer_t * rbuf, uint16_t * data);
int32_t ringbuffer_empty(ringbuffer_t * rbuf);
int32_t ringbuffer_full(ringbuffer_t * rbuf);


int32_t ringbuffer_reset(ringbuffer_t * rbuf)
{
    int32_t status = -1;

    if(rbuf)
    {
        rbuf->head = 0;
        rbuf->tail = 0;
        status = 0;
    }

    return status;
}



int32_t ringbuffer_empty(ringbuffer_t * rbuf)
{
    // define empty as head == tail
    return (rbuf->head == rbuf->tail);
}


int32_t ringbuffer_length(ringbuffer_t * rbuf)
{
    int32_t status;
    if (rbuf->head >= rbuf->tail)
    {
        status = ((int16_t)(rbuf->head) - (int16_t)(rbuf->tail));
    }
    else
    {
        status = (((int16_t)rbuf->size - (int16_t)rbuf->tail) +
                  (int16_t)rbuf->head);
    }
    return status;
}

int32_t ringbuffer_full(ringbuffer_t * rbuf)
{
    // We determine "full" case by head being one position behind the tail
    // Note that this means we are wasting one space in the buffer!
    // Instead, you could have an "empty" flag and determine buffer full that way
    return ((rbuf->head + 1) % rbuf->size) == rbuf->tail;
}

int32_t ringbuffer_put(ringbuffer_t * rbuf, uint16_t data)
{
    int32_t status = -1;

    if(rbuf)
    {
        rbuf->buffer[rbuf->head] = data;
        rbuf->head = (rbuf->head + 1) % rbuf->size;

        if(rbuf->head == rbuf->tail)
        {
            rbuf->tail = (rbuf->tail + 1) % rbuf->size;
        }

        status = 0;
    }

    return status;
}

int32_t ringbuffer_put_array(ringbuffer_t * rbuf, uint16_t * data, uint16_t size)
{
    int32_t status = -1;

    if (rbuf)
    {
        uint16_t i;
        for(i = 0; i < size; i++)
        {
            ringbuffer_put(rbuf, data[i]);
        }

        status = 0;
    }

    return status;
}


int32_t ringbuffer_get(ringbuffer_t * rbuf, uint16_t * data)
{
    int32_t status = -1;

    if(rbuf && data && !ringbuffer_empty(rbuf))
    {
        *data = rbuf->buffer[rbuf->tail];
        rbuf->tail = (rbuf->tail + 1) % rbuf->size;

        status = 0;
    }

    return status;
}

int32_t ringbuffer_data(ringbuffer_t * rbuf, uint16_t * data, uint16_t pos)
{
    int32_t status = -1;
    if (rbuf && data && !ringbuffer_empty(rbuf))
    {
        *data = rbuf->buffer[pos];
        status = 0;
    }
    return status;
}



