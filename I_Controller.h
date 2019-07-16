/*
 * I_Controller.h
 *
 *  Created on: 18.02.2019
 *      Author: sklammer
 */

#ifndef I_CONTROLLER_H_
#define I_CONTROLLER_H_


#include <stdint.h>
#include "DSP2803x_Cla_typedefs.h"
#include "DSP2803x_Cla_defines.h"
#include "DSP2803x_Cla.h"


typedef volatile struct {
    float Vi;       //!< Integral Gain
    float Vr;       //!< Gain from Feedback (Saturation)
    float u;        //!< Input
    float e;        //!< Saturation Difference
    float y;        //!< unsaturated Output
    float y1;       //!< old output value
    float ys;       //!< saturated Output
    float sat_min;  //!< Lower saturation limit
    float sat_max;  //!< Upper saturation limit
} I_CONTROLLER;

//! \brief          Defines default values to initialise the DCL_PID structure
//!
#define I_DEFAULTS {0.1f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 20.0f}

//! \brief          Executes an inline ideal form PID controller on the FPU32
//! \param[in] cntl    Pointer to the DCL_PID structure
//! \param[in] ref   The controller set-point reference
//! \param[in] value   The measured feedback value
//! \return         The control effort
//!
float run_I_CONTROLLER(I_CONTROLLER *cntl, float ref, float value);

void  init_I_CONTROLLER(I_CONTROLLER *cntl, float V_i, float V_r, float min, float max);


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
