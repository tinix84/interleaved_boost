/* ==================================================================== */
/* ========================== include files =========================== */
/* ==================================================================== */

//#include "application.h"
#include <stdint.h>
#include "device.h"
#include "application.h"

#include "DSP2803x_Device.h"      // DSP2803x Headerfile Include File
#include "DSP2803x_Cla_typedefs.h"// DSP2803x CLA Type definitions
#include "DSP2803x_Cla_defines.h"
#include "DSP2803x_Cla.h"
#include "DCL.h"

/* ==================================================================== */
/* ============================ constants ============================= */
/* ==================================================================== */
/* none */

/* ==================================================================== */
/* ======================== global variables ========================== */
/* ==================================================================== */

//===============================================
// Variables - CPU to CLA
//===============================================

//Reference Values - Voltage
#pragma DATA_SECTION(cla_Vref1,"CpuToCla1MsgRAM");
float cla_Vref1=Vref_default;
#pragma DATA_SECTION(cla_Vref2,"CpuToCla1MsgRAM");
float cla_Vref2=Vref_default;

//Reference Values - Current
#pragma DATA_SECTION(cla_Iref1,"CpuToCla1MsgRAM");
float cla_Iref1=Iref_default;
#pragma DATA_SECTION(cla_Iref2,"CpuToCla1MsgRAM");
float cla_Iref2=Iref_default;
#pragma DATA_SECTION(cla_Iref3,"CpuToCla1MsgRAM");
float cla_Iref3=Iref_default;

//Controller Coefficients
#pragma DATA_SECTION(cla_Kp_Volt_1,"CpuToCla1MsgRAM");
float cla_Kp_Volt_1=Kp_Volt_default;

#pragma DATA_SECTION(cla_Ki_Volt_1,"CpuToCla1MsgRAM");
float cla_Ki_Volt_1=Ki_Volt_default;

//===============================================
// Variables - CLA to CPU
//===============================================

//Measured Values - Voltage
#pragma DATA_SECTION(cla_Vout1,"Cla1ToCpuMsgRAM");
float cla_Vout1=5.0;
#pragma DATA_SECTION(cla_Vout2,"Cla1ToCpuMsgRAM");
float cla_Vout2=5.0;
#pragma DATA_SECTION(cla_Vout3,"Cla1ToCpuMsgRAM");
float cla_Vout3=5.0;

//Measured Values - Voltage
#pragma DATA_SECTION(cla_Iout1,"Cla1ToCpuMsgRAM");
float cla_Iout1=5.0;
#pragma DATA_SECTION(cla_Iout2,"Cla1ToCpuMsgRAM");
float cla_Iout2=5.0;
#pragma DATA_SECTION(cla_Iout3,"Cla1ToCpuMsgRAM");
float cla_Iout3=5.0;
#pragma DATA_SECTION(cla_Iout123,"Cla1ToCpuMsgRAM");
float cla_Iout123=5.0;


//Ramp Values Current
#pragma DATA_SECTION(cla_Ramp1_Curr,"Cla1ToCpuMsgRAM");
float cla_Ramp1_Curr=5.0;
#pragma DATA_SECTION(cla_Ramp2_Curr,"Cla1ToCpuMsgRAM");
float cla_Ramp2_Curr=5.0;
#pragma DATA_SECTION(cla_Ramp3_Curr,"Cla1ToCpuMsgRAM");
float cla_Ramp3_Curr=5.0;

//#pragma DATA_SECTION(i1_Curr, "Cla1ToCpuMsgRAM")
//I_CONTROLLER i1_Curr;
//#pragma DATA_SECTION(i2_Curr, "Cla1ToCpuMsgRAM")
//I_CONTROLLER i2_Curr;
//#pragma DATA_SECTION(i3_Curr, "Cla1ToCpuMsgRAM")
//I_CONTROLLER i3_Curr;

//===============================================
// Variables - CLA
//===============================================

//Ramp Values Voltage
#pragma DATA_SECTION(cla_Ramp1_Volt,"Cla1DataRam0");
float cla_Ramp1_Volt=5.0;
#pragma DATA_SECTION(cla_Ramp2_Volt,"Cla1DataRam0");
float cla_Ramp2_Volt=5.0;
#pragma DATA_SECTION(cla_Ramp3_Volt,"Cla1DataRam0");
float cla_Ramp3_Volt=5.0;

// Voltage Controller
#pragma DATA_SECTION(pi1_Volt, "Cla1DataRam0")
DCL_PI pi1_Volt = PI_DEFAULTS;
#pragma DATA_SECTION(pi2_Volt, "Cla1DataRam0")
DCL_PI pi2_Volt = PI_DEFAULTS;
#pragma DATA_SECTION(pi3_Volt, "Cla1DataRam0")
DCL_PI pi3_Volt = PI_DEFAULTS;

// Debug Variables
#pragma DATA_SECTION(pi_debug_Volt_max, "Cla1DataRam0")
float pi_debug_Volt_max;
#pragma DATA_SECTION(pi_debug_Volt_min, "Cla1DataRam0")
float pi_debug_Volt_min;
#pragma DATA_SECTION(pi_debug_Curr_max, "Cla1DataRam0")
float pi_debug_Curr_max;
#pragma DATA_SECTION(pi_debug_Curr_min, "Cla1DataRam0")
float pi_debug_Curr_min;


//===============================================
// Controller
//===============================================

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

//===============================================
// System Monitor data
//===============================================
uint16_t Timer1IntFlg;

//===============================================
// Serial driver data
//===============================================
commSerialFrame_t serialFrame;
commFrameStates_e rxstate;
commFrameStates_e txstate;

//===============================================
// Ring buffer structures
//===============================================
ringbuffer_t SCIATXBufferStruct;
ringbuffer_t SCIARXBufferStruct;
uint16_t SCIATXBuffer[SCI_BUFFER_SIZE];
uint16_t SCIARXBuffer[SCI_BUFFER_SIZE];

//===============================================
// timer vars */
//===============================================
uint32_t  EPwm1TimerIntCount;
uint32_t  EPwm2TimerIntCount;
uint32_t  EPwm3TimerIntCount;
uint16_t  EPwm1_DB_Direction;
uint16_t  EPwm2_DB_Direction;
uint16_t  EPwm3_DB_Direction;

//===============================================
// PWM data
//===============================================

uint32_t actual_duty_cnt = 0;
uint32_t actual_phdly_cnt = 0;
uint32_t actual_pwm_period = 0;
uint32_t actual_deadtime_cnt = 0;
uint32_t new_deadtime_ns = 0;


/* ==================================================================== */
/* ========================== private data ============================ */
/* ==================================================================== */

/* none */

/* ==================================================================== */
/* ====================== private functions =========================== */
/* ==================================================================== */

/* none */

/* ==================================================================== */
/* ===================== All functions by section ===================== */
/* ==================================================================== */

/* none */
