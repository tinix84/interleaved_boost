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

#ifndef SHELLSORT_SHARED_H_
#define SHELLSORT_SHARED_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "IQmathLib.h"
#include "DSP2803x_Cla_typedefs.h"
#include "DSP2803x_Cla_defines.h"
#include "DSP2803x_Cla.h"
#include "DCL.h"
#include "I_Controller.h"


//Task 1 (C) Variables

//Task 2 (C) Variables

extern float cla_Vref1;
extern float cla_Vref2;
extern float cla_Vref3;
extern float cla_Vout1;
extern float cla_Vout2;
extern float cla_Vout3;

extern float cla_Iref1;
extern float cla_Iref2;
extern float cla_Iref3;
extern float cla_Iout1;
extern float cla_Iout2;
extern float cla_Iout3;

extern float cla_Ramp1_Volt;
extern float cla_Ramp2_Volt;
extern float cla_Ramp3_Volt;

extern float cla_Ramp1_Curr;
extern float cla_Ramp2_Curr;
extern float cla_Ramp3_Curr;

extern DCL_PI pi1_Volt;

extern I_CONTROLLER i1_Curr;
extern I_CONTROLLER i2_Curr;
extern I_CONTROLLER i3_Curr;

extern uint16_t cla_Current_Controller_Disable;
extern float cla_Current_Controller_Correction;

extern float cla_CC_Ki;
extern float cla_CC_Kr;

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


//CLA ISRs
__interrupt void cla1_isr2(void);

//Linker defined vars
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
//__interrupt void Cla1Task3();
//__interrupt void Cla1Task4();
//__interrupt void Cla1Task5();
//__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();

extern Uint16 Cla1Prog_Start;
extern Uint16 Cla1funcsLoadStart;
extern Uint16 Cla1funcsLoadSize;
extern Uint16 Cla1funcsRunStart;

extern Uint32 Cla1T1End;
extern Uint32 Cla1T2End;
extern Uint32 Cla1T3End;
extern Uint32 Cla1T4End;
extern Uint32 Cla1T5End;
extern Uint32 Cla1T6End;
extern Uint32 Cla1T7End;
extern Uint32 Cla1T8End;


//C Function Prototypes

#ifdef __cplusplus
}
#endif /* extern "C" */
#endif /*EXP2_SHARED_C_H_*/
