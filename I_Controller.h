/*
 * I_Controller.h
 *
 *  Created on: 18.02.2019
 *      Author: sklammer
 */

#ifndef I_CONTROLLER_H_
#define I_CONTROLLER_H_


#include <stdint.h>
#include "F2806x_Cla_typedefs.h"
#include "F2806x_Cla_defines.h"
#include "F2806x_Cla.h"

#ifndef C2000_IEEE754_TYPES
#define C2000_IEEE754_TYPES
typedef float float32_t;
#endif

typedef volatile struct {
    float32_t Vi;       //!< Integral Gain
    float32_t Vr;       //!< Gain from Feedback (Saturation)
    float32_t u;        //!< Input
    float32_t e;        //!< Saturation Difference
    float32_t y;        //!< unsaturated Output
    float32_t y1;       //!< old output value
    float32_t ys;       //!< saturated Output
    float32_t sat_min;  //!< Lower saturation limit
    float32_t sat_max;  //!< Upper saturation limit
} I_CONTROLLER;

typedef volatile struct {
    float32_t Vi;       //!< Integral Gain
    float32_t Vp;       //!< Proportional Gain
    float32_t Vr;       //!< Gain from Feedback (Saturation)
    float32_t u;        //!< Input
    float32_t e;        //!< Saturation Difference
    float32_t y;        //!< unsaturated Output
    float32_t y1;       //!< old output value
    float32_t ys;       //!< saturated Output
    float32_t sat_min;  //!< Lower saturation limit
    float32_t sat_max;  //!< Upper saturation limit
} PI_CONTROLLER;

//! \brief          Defines default values to initialise the DCL_PID structure
//!
#define I_DEFAULTS {0.1f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 20.0f}

//! \brief          Executes an inline ideal form PID controller on the FPU32
//! \param[in] cntl    Pointer to the DCL_PID structure
//! \param[in] ref   The controller set-point reference
//! \param[in] value   The measured feedback value
//! \return         The control effort
//!
float32_t run_I_CONTROLLER(I_CONTROLLER *cntl, float32_t ref, float32_t value);

void  init_I_CONTROLLER(I_CONTROLLER *cntl, float32_t V_i, float32_t V_r, float32_t min, float32_t max);

float32_t run_PI_CONTROLLER(PI_CONTROLLER *cntl, float32_t ref, float32_t value);

void  init_PI_CONTROLLER(PI_CONTROLLER *cntl, float32_t V_p, float32_t V_i, float32_t V_r, float32_t min, float32_t max);

/**
 * Macro to calculate a integral controller
 * @param cntl - I_CONTROLLER structure, ref - Reference Value, value - actual value to compare
 * @return I_CONTROLLER structure
 */
#define run_I_CONTROLLER_CLA_MACRO(cntl, ref, value)            \
        cntl.u = ref - value;                                   \
        cntl.e = cntl.ys - cntl.y;                              \
        cntl.y = cntl.y1 + cntl.Vi * cntl.u + cntl.Vr * cntl.e; \
        cntl.ys = __mmaxf32(cntl.y, cntl.sat_min);              \
        cntl.ys = __mminf32(cntl.ys, cntl.sat_max);             \
        cntl.y1 = cntl.y;                                      \


#endif /* I_CONTROLLER_H_ */
