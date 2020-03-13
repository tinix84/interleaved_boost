/* ==================================================================== */
/* ========================== include files =========================== */
/* ==================================================================== */

//#include "application.h"
#include <stdint.h>
#include "device.h"
#include "application.h"
#include "F2806x_Device.h"      // DSP2803x Headerfile Include File
#include "F2806x_Cla_typedefs.h"// DSP2803x CLA Type definitions
#include "F2806x_Cla_defines.h"
#include "F2806x_Cla.h"
#include "DCL.h"
#include "I_Controller.h"

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
#pragma DATA_SECTION(cla_VrefU,"CpuToCla1MsgRAM");
float cla_VrefU=Vref_default;
#pragma DATA_SECTION(cla_VrefV,"CpuToCla1MsgRAM");
float cla_VrefV=Vref_default;
#pragma DATA_SECTION(cla_VrefW,"CpuToCla1MsgRAM");
float cla_VrefW=Vref_default;

//Reference Values - Current
#pragma DATA_SECTION(cla_IrefU,"CpuToCla1MsgRAM");
float cla_IrefU=Iref_default;
#pragma DATA_SECTION(cla_IrefV,"CpuToCla1MsgRAM");
float cla_IrefV=Iref_default;
#pragma DATA_SECTION(cla_IrefW,"CpuToCla1MsgRAM");
float cla_IrefW=Iref_default;

#pragma DATA_SECTION(actual_pwm_period,"CpuToCla1MsgRAM");
uint32_t actual_pwm_period = EPWM_A_INIT_PERIOD;
#pragma DATA_SECTION(actual_duty_cnt,"CpuToCla1MsgRAM");
uint32_t actual_duty_cnt = EPWM_A_INIT_CMPA;

//Controller Coefficients
#pragma DATA_SECTION(cla_CC_Ki,"Cla1DataRam0");
float cla_CC_Ki;
#pragma DATA_SECTION(cla_CC_Kr,"Cla1DataRam0");
float cla_CC_Kr;

//===============================================
// Variables - CLA to CPU
//===============================================

//Measured Values - Voltage
#pragma DATA_SECTION(cla_VoutU,"Cla1ToCpuMsgRAM");
float cla_VoutU=20.0;
#pragma DATA_SECTION(cla_VoutV,"Cla1ToCpuMsgRAM");
float cla_VoutV=20.0;
#pragma DATA_SECTION(cla_VoutW,"Cla1ToCpuMsgRAM");
float cla_VoutW=20.0;

//Measured Values - Current
#pragma DATA_SECTION(cla_IoutU,"Cla1ToCpuMsgRAM");
float cla_IoutU=5.0;
#pragma DATA_SECTION(cla_IoutV,"Cla1ToCpuMsgRAM");
float cla_IoutV=5.0;
#pragma DATA_SECTION(cla_IoutW,"Cla1ToCpuMsgRAM");
float cla_IoutW=5.0;


//===============================================
// Variables - CLA
//===============================================

//Ramp Values Voltage
#pragma DATA_SECTION(cla_Ramp_Volt,"Cla1DataRam0");
float cla_Ramp_Volt=0.0f;
#pragma DATA_SECTION(min_duty_f,"Cla1DataRam0");
float min_duty_f=0.0f;

// I Controller Section
#pragma DATA_SECTION(cla_CC_Ki,"Cla1DataRam0");
float cla_CC_Ki;
#pragma DATA_SECTION(cla_CC_Kr,"Cla1DataRam0");
float cla_CC_Kr;

// Voltage Controller
//#pragma DATA_SECTION(pi1_Volt, "Cla1DataRam0")
//DCL_PI pi1_Volt = PI_DEFAULTS;
#pragma DATA_SECTION(pi1_Volt, "Cla1ToCpuMsgRAM")
I_CONTROLLER pi1_Volt;

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


uint32_t actual_phdly_cnt = EPWM_A_INIT_PHASE;

uint32_t actual_deadtime_cnt = EPWM_A_INIT_DEADBAND;
uint32_t new_deadtime_ns = 0;
float new_Vout = 0.0f;
float max_Vout_step = 10.0f;


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
