/*
 * ringbuffer.h
 *
 *  Created on: Jul 19, 2018
 *      Author: fkyburz
 */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

//#include "device.h"
#include <stdint.h>

typedef struct {
    uint16_t * buffer;
    uint16_t head;
    uint16_t tail;
    uint16_t size;
} ringbuffer_t;

extern int32_t ringbuffer_reset(ringbuffer_t * rbuf);
extern int32_t ringbuffer_put(ringbuffer_t * rbuf, uint16_t data);
extern int32_t ringbuffer_put_array(ringbuffer_t * rbuf,
                                    uint16_t * data, uint16_t size);
extern int32_t ringbuffer_get(ringbuffer_t * rbuf, uint16_t * data);
extern int32_t ringbuffer_empty(ringbuffer_t * rbuf);
extern int32_t ringbuffer_full(ringbuffer_t * rbuf);
extern int32_t ringbuffer_length(ringbuffer_t * rbuf);
extern int32_t ringbuffer_data(ringbuffer_t * rbuf,
                               uint16_t *data, uint16_t pos);



#endif /* RINGBUFFER_H_ */
