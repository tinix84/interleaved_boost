//###########################################################################
// Description:
//
// Declare shared variables
//
//###########################################################################
// $TI Release: F2803x C/C++ Header Files and Peripheral Examples V130 $
// $Release Date: May  8, 2015 $
// $Copyright: Copyright (C) 2009-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#ifndef CLA_SHARED_H_
#define CLA_SHARED_H_

#include "DSP2803x_Cla_typedefs.h"
#include "DSP2803x_Cla_defines.h"
#include "DSP2803x_Cla.h"
#include "DCL.h"
#include "I_Controller.h"

#ifdef __cplusplus
extern "C" {
#endif

//Task 1 (C) Variables
extern float cla_VrefU;
extern float cla_VrefV;
extern float cla_VrefW;
extern float cla_VoutU;
extern float cla_VoutV;
extern float cla_VoutW;

extern float cla_IoutU;
extern float cla_IoutV;
extern float cla_IoutW;

extern float cla_Ramp_Volt;

extern uint32_t actual_pwm_period;
extern uint32_t actual_duty_cnt;
extern float min_duty_f;

extern DCL_PID pid1_Volt;
extern I_CONTROLLER pi1_Volt;

extern float cla_CC_Ki;
extern float cla_CC_Kr;


//Task 2 (C) Variables

//Task 3 (C) Variables

//Task 4 (C) Variables

//Task 5 (C) Variables

//Task 6 (C) Variables

//Task 7 (C) Variables

//Task 8 (C) Variables

//Common (C) Variables

//CLA C Tasks
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
//__interrupt void Cla1Task3();
//__interrupt void Cla1Task4();
//__interrupt void Cla1Task5();
//__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();

//C Function Prototypes

#ifdef __cplusplus
}
#endif /* extern "C" */
#endif /*CLA_SHARED_H_*/
