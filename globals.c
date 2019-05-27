/* =====================================================================*/
/* Program: globals.c                                                   */
/*                                                                      */
/* Project: PV-Batteriewechselrichter                                   */
/*                                                                      */
/* Description: Global variables only. All globals must be placed in    */
/*              this file. Initialize everything to zero.                                             */
/*                                                                      */
/* Last Edited: 2018-11-06                                              */
/*                                                                      */
/* Author: Falk Kyburz                                                  */
/*                                                                      */
/* ==================================================================== */
/*    (c) 2018 by Interstaatliche Hochschule für Technik Buchs NTB      */
/* ==================================================================== */


/* ==================================================================== */
/* ========================== include files =========================== */
/* ==================================================================== */

//#include "application.h"
#include <stdint.h>
#include "device.h"
#include "application.h"
#include "DSP2803x_Cla_typedefs.h"// DSP2803x CLA Type definitions
#include "DSP2803x_Device.h"      // DSP2803x Headerfile Include File

/* ==================================================================== */
/* ============================ constants ============================= */
/* ==================================================================== */

/* none */

/* ==================================================================== */
/* ======================== global variables ========================== */
/* ==================================================================== */

/* General */

/* ADC converter data structs */
int ChSel[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int TrigSel[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int ACQPS[16] = {6,6,6,6,6,6,6,6,6,6,6,6,6,6,6,6};//For 8x PFC current oversampling

/* EPWM data structs */
// Used to indirectly access all EPWM modules
volatile struct EPWM_REGS *ePWM[] =
                  { &EPwm1Regs,         //intentional: (ePWM[0] not used)
                    &EPwm1Regs,
                    &EPwm2Regs,
                    &EPwm3Regs,
                    &EPwm4Regs,
                    &EPwm5Regs,
                    &EPwm6Regs,
                    &EPwm7Regs,
                  };

// Used to indirectly access all Comparator modules
volatile struct COMP_REGS *Comp[] =
                  { &Comp1Regs,         //intentional: (Comp[0] not used)
                    &Comp1Regs,
                    &Comp2Regs,
                    &Comp3Regs
                  };

/* System Monitor data */


/* Serial driver data */
commSerialFrame_t serialFrame;
commFrameStates_e rxstate;
commFrameStates_e txstate;
ringbuffer_t SCIATXBufferStruct;
ringbuffer_t SCIARXBufferStruct;
uint16_t SCIATXBuffer[SCI_BUFFER_SIZE];
uint16_t SCIARXBuffer[SCI_BUFFER_SIZE];

/* Ring buffer structures */
ringbuffer_t SCIATXBufferStruct;
ringbuffer_t SCIARXBufferStruct;

/* timer vars */
uint32_t  EPwm1TimerIntCount;
uint32_t  EPwm2TimerIntCount;
uint32_t  EPwm3TimerIntCount;
uint16_t  EPwm1_DB_Direction;
uint16_t  EPwm2_DB_Direction;
uint16_t  EPwm3_DB_Direction;

/* PWM data */
uint32_t actual_duty_cnt = 0;
uint32_t actual_phdly_cnt = 0;
uint32_t actual_pwm_period = 0;
uint32_t actual_deadtime_cnt = 0;

uint32_t foo2 = 0;

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
