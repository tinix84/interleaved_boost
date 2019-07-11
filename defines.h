/* =====================================================================*/
/* Program: defines.h                                                   */
/*                                                                      */
/* Project: PV_Batteriewechselrichter                                   */
/*                                                                      */
/* Description: Global defines (adresses, init values). Do not put here */
/*              if value could change during program execution.         */
/*                                                                      */
/* Last Edited: 2018-11-06                                              */
/*                                                                      */
/* Author: Falk Kyburz                                                  */
/*                                                                      */
/* ==================================================================== */
/*    (c) 2018 by Interstaatliche Hochschule für Technik Buchs NTB      */
/* ==================================================================== */

#ifndef DEFINES_H_
#define DEFINES_H_

/* ==================================================================== */
/* ========================== include files =========================== */
/* ==================================================================== */

#include "DSP2803x_Cla_typedefs.h"// DSP2803x CLA Type definitions
#include "DSP2803x_Device.h"     //DSP2803x Headerfile Include File
#include "DSP2803x_GlobalPrototypes.h"         // Prototypes for global functions within the
                                               // .c files.
#include "DSP2803x_EPwm_defines.h"             // Macros used for PWM examples.
#include "DSP2803x_I2c_defines.h"              // Macros used for I2C examples.
#include "DSP2803x_Cla_defines.h"              // Macros used for CLA examples.


/* ==================================================================== */
/* ==================================================================== */

/* Defines */

// The following pointer to a function call calibrates the ADC and internal oscillators
#define Device_cal (void   (*)(void))0x3D7C80

#define   PARTNO_28030PAG   0xAA
#define   PARTNO_28030PN    0xAB

#define   PARTNO_28031PAG   0xAE
#define   PARTNO_28031PN    0xAF

#define   PARTNO_28032PAG   0xB2
#define   PARTNO_28032PN    0xB3

#define   PARTNO_28033PAG   0xB6
#define   PARTNO_28033PN    0xB7

#define   PARTNO_28034PAG   0xBA
#define   PARTNO_28034PN    0xBB

#define   PARTNO_28035PAG   0xBE
#define   PARTNO_28035PN    0xBF


// Include files not used with DSP/BIOS
#ifndef DSP28_BIOS
#include "DSP2803x_DefaultISR.h"
#endif

// DO NOT MODIFY THIS LINE.
#define DELAY_US(A)  DSP28x_usDelay(((((long double) A * 1000.0L) / (long double)CPU_RATE) - 9.0L) / 5.0L)

/* Compilation define */
#define INCR_BUILD 0
#define BUILD2_SELECT 0
#define CNTRL_ISR_FREQ_RATIO    2
#define VOLTAGE_LOOP_RUN_RATIO  2

/* System control define */
/* Specify the PLL control register (PLLCR) and divide select (DIVSEL) value.*/
//#define DSP28_DIVSEL   0 // Enable /4 for SYSCLKOUT
//#define DSP28_DIVSEL   1 // Disable /4 for SYSCKOUT
#define DSP28_DIVSEL   2 // Enable /2 for SYSCLKOUT
//#define DSP28_DIVSEL   3 // Enable /1 for SYSCLKOUT

#define DSP28_PLLCR   12    // Uncomment for 60 MHz devices [60 MHz = (10MHz * 12)/2]
//#define DSP28_PLLCR   11
//#define DSP28_PLLCR   10
//#define DSP28_PLLCR    9
//#define DSP28_PLLCR    8      // Uncomment for 40 MHz devices [40 MHz = (10MHz * 8)/2]
//#define DSP28_PLLCR    7
//#define DSP28_PLLCR    6
//#define DSP28_PLLCR    5
//#define DSP28_PLLCR    4
//#define DSP28_PLLCR    3
//#define DSP28_PLLCR    2
//#define DSP28_PLLCR    1
//#define DSP28_PLLCR    0  // PLL is bypassed in this mode
/*-----------------------------------------------------------------------------
      Specify the clock rate of the CPU (SYSCLKOUT) in nS.
      Only one statement should be uncommented.
-----------------------------------------------------------------------------*/

#define CPU_RATE   16.667L   // for a 60MHz CPU clock speed (SYSCLKOUT)
//#define CPU_RATE   20.000L   // for a 50MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   25.000L   // for a 40MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   33.333L   // for a 30MHz CPU clock speed  (SYSCLKOUT)

#define DEVICE_SYSCLK_FREQ 60000000U

/* EPWM control defines */
#define INTERLEAVE          2
#define UPDOWN_FACTOR 2
#define UP_FACTOR 1

#define CARRIER_FACTOR UPDOWN_FACTOR


#define EPWM_CLK_FREQ          DEVICE_SYSCLK_FREQ
#define EPWM_CLK_PERIOD_NS     (1000000000U / EPWM_CLK_FREQ)
#define EPWM_PWM_DEAD_TIME_NS  1000U
#define EPWM_PWM_FREQ_25KHZ    25000U

#define EPWM_PERIOD_COUNT_25KHZ (EPWM_CLK_FREQ / EPWM_PWM_FREQ_25KHZ)
#define EPWM_PERIOD_COUNT_25KHZ_F 4000.0f
#define EPWM_COUNT_25KHZ_DUTY50 (EPWM_PERIOD_COUNT_25KHZ / 2U)
#define EPWM_COUNT_25KHZ_DUTY10 (EPWM_PERIOD_COUNT_25KHZ / 10U)

#define EPWM_PWM_DEAD_TIME_COUNT (EPWM_PWM_DEAD_TIME_NS / EPWM_CLK_PERIOD_NS)

/* EPWM defines for initialization */
#define EPWMx_INIT_PERIOD    (uint16_t)(EPWM_PERIOD_COUNT_25KHZ / CARRIER_FACTOR)
#define EPWMx_INIT_PHASE     (uint16_t)(EPWM_PERIOD_COUNT_25KHZ / INTERLEAVE)
#define EPWMx_INIT_CMPA      (uint16_t)(EPWM_COUNT_25KHZ_DUTY10 / CARRIER_FACTOR)
#define EPWMx_INIT_DEADBAND  (uint16_t)(EPWM_PWM_DEAD_TIME_COUNT)

#define EPWM_A_INIT_PERIOD       EPWMx_INIT_PERIOD
#define EPWM_A_INIT_PHASE        0U
#define EPWM_A_INIT_CMPA         EPWMx_INIT_CMPA
#define EPWM_A_INIT_DEADBAND     EPWMx_INIT_DEADBAND

#define EPWM_B_INIT_PERIOD       EPWMx_INIT_PERIOD
#define EPWM_B_INIT_PHASE        EPWMx_INIT_PHASE
#define EPWM_B_INIT_CMPA         EPWMx_INIT_CMPA
#define EPWM_B_INIT_DEADBAND     EPWMx_INIT_DEADBAND

/* Low frequency peripheries */
#define LSPCLK_FREQ DEVICE_SYSCLK_FREQ/4
#define SCI_FREQ    100E3
#define SCI_PRD     (LSPCLK_FREQ/(SCI_FREQ*8))-1

/* Serial communication */
#define COMM_MAX_DATA_LENGTH 4U
#define COMM_START 0x55U
#define SCI_BUFFER_SIZE 16

/* Control loop define */
#define PWM_NO              1
#define ADC_TRIG_SOURCE     5
#define ADC_PIN_VOUT        2
#define ADC_PIN_VIN_L       10
#define ADC_PIN_VIN_N       8
#define ADC_PIN_IL_AVG      4
#define ADC_IL_COMPARATOR   2
#define SFRA_TYPE           1

#define OUTPUT_VOLTAGE      390
#define VIN_MAX_SENSE       402.07
#define VBUS_MAX_SENSE      510.99
#define IL_MAX_SENSE        19.8
#define VBUS_RATED_VOLTS    390.0

#define VBUS_MIN ((int32)((160.0/VBUS_MAX_SENSE)*4095*4095))
#define VBUS_OVP_THRSHLD ((int32)((435.0/VBUS_MAX_SENSE)*4095*4095)) //435V

#define VBUS_TARGET         ((int32)((VBUS_RATED_VOLTS/VBUS_MAX_SENSE)*4095*4095)) //395V
#define VBUS_ERROR_NL_CNTRL_THRSHLD ((int32)((15.0/VBUS_MAX_SENSE)*4095*4095)) //Vbus error threshold to activate NL Vloop control

#if (INCR_BUILD == 1)
#define SFRA_FREQ_START 10
#define SFREQ_STEP_MULTIPLY (float)1.08902296
#elif (INCR_BUILD == 2)
    #if(BUILD2_SELECT == 0)
    #define SFRA_FREQ_START 10
    #define SFREQ_STEP_MULTIPLY (float)1.08902296
    #elif(BUILD2_SELECT == 1)
    #define SFRA_FREQ_START 150
    #define SFREQ_STEP_MULTIPLY (float)1.05925373
    #endif
#elif (INCR_BUILD == 3)
    #if (SFRA_TYPE == 1)
    #define SFRA_FREQ_START 150
    #define SFREQ_STEP_MULTIPLY (float)1.05925373
    #elif (SFRA_TYPE == 0)
    #define SFRA_FREQ_START 2
    #define SFREQ_STEP_MULTIPLY (float) 1.0660505
    #endif
#endif

/* ==================================================================== */
/* ======================== macros           ========================== */
/* ==================================================================== */

/* Int macros */
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define ABS(a)   ((((a)>(0))?(a):(-a)))

/* float macros */
#define MINF32(a, b) __fmin((a), (b))
#define MAXF32(a, b) __fmax((a), (b))
#define ABSF32(a) __fmax((-a),(a))
#define SATF32(a, amax, amin) __fmax(__fmin((a), (amax)), (amin));
#define RATEF32(target, var, step) ((var) + __fmax(__fmin(((target)-(var)), (step)), -(step)))

/* Macro for 1us_delay */





/* ==================================================================== */
/* ======================== global variables ========================== */
/* ==================================================================== */

/* Global variables definitions go here */

/* ==================================================================== */
/* ========================== private data ============================ */
/* ==================================================================== */

/* Definition of private datatypes go here */

/* ==================================================================== */
/* ====================== private functions =========================== */
/* ==================================================================== */

/* Function prototypes for private (static) functions go here */

/* ==================================================================== */
/* ===================== All functions by section ===================== */
/* ==================================================================== */

//==================================================================================
// Comp Settings
//----------------------------------------------------------------------------------

#define ACTIVE_COMP 1
#define CNTL_3p3z_A1_1 1.1159959670
#define CNTL_3p3z_A2_1 -0.1159959670
#define CNTL_3p3z_A3_1 0.0000000000
#define CNTL_3p3z_B0_1 0.4104341549
#define CNTL_3p3z_B1_1 -0.3822445616
#define CNTL_3p3z_B2_1 0.0712511235
#define CNTL_3p3z_B3_1 0.0000000000
#define CNTL_3p3z_IMin_1 _IQ24(-0.1);
#define CNTL_3p3z_Max_1 _IQ24(0.9);
#define CNTL_3p3z_Min_1 _IQ24(0.0);

#define CNTL_3p3z_A1_2 0.9999998212
#define CNTL_3p3z_A2_2 0.0000001788
#define CNTL_3p3z_A3_2 0.0000000000
#define CNTL_3p3z_B0_2 1.0239908909
#define CNTL_3p3z_B1_2 -0.4166883042
#define CNTL_3p3z_B2_2 -0.0000000745
#define CNTL_3p3z_B3_2 0.0000000000
#define CNTL_3p3z_IMin_2 _IQ24(-0.1);
#define CNTL_3p3z_Max_2 _IQ24(0.9);
#define CNTL_3p3z_Min_2 _IQ24(0.0);


#define CNTL_3p3z_A1_3 0.9999998212
#define CNTL_3p3z_A2_3 0.0000001788
#define CNTL_3p3z_A3_3 0.0000000000
#define CNTL_3p3z_B0_3 1.5359818835
#define CNTL_3p3z_B1_3 -0.6250306444
#define CNTL_3p3z_B2_3 -0.0000001117
#define CNTL_3p3z_B3_3 0.0000000000
#define CNTL_3p3z_IMin_3 _IQ24(-0.1);
#define CNTL_3p3z_Max_3 _IQ24(0.9);
#define CNTL_3p3z_Min_3 _IQ24(0.0);

#define CNTL_3p3z_A1_4 1.3479999276
#define CNTL_3p3z_A2_4 -0.3479999276
#define CNTL_3p3z_A3_4 0.0000000000
#define CNTL_3p3z_B0_4 0.6304931601
#define CNTL_3p3z_B1_4 -0.3200911809
#define CNTL_3p3z_B2_4 0.0143992349
#define CNTL_3p3z_B3_4 0.0000000000
#define CNTL_3p3z_IMin_4 _IQ24(-0.1);
#define CNTL_3p3z_Max_4 _IQ24(0.9);
#define CNTL_3p3z_Min_4 _IQ24(0.0);


#define CNTL_3p3z_A1_5 0.9999998212
#define CNTL_3p3z_A2_5 0.0000001788
#define CNTL_3p3z_A3_5 0.0000000000
#define CNTL_3p3z_B0_5 0.7824251528
#define CNTL_3p3z_B1_5 -0.4449240626
#define CNTL_3p3z_B2_5 -0.0000000795
#define CNTL_3p3z_B3_5 0.0000000000
#define CNTL_3p3z_IMin_5 _IQ24(-0.1);
#define CNTL_3p3z_Max_5 _IQ24(0.9);
#define CNTL_3p3z_Min_5 _IQ24(0.0);

#define CNTL_3p3z_A1_6 0.9999998212
#define CNTL_3p3z_A2_6 0.0000001788
#define CNTL_3p3z_A3_6 0.0000000000
#define CNTL_3p3z_B0_6 1.0239908909
#define CNTL_3p3z_B1_6 -0.4166883042
#define CNTL_3p3z_B2_6 -0.0000000745
#define CNTL_3p3z_B3_6 0.0000000000
#define CNTL_3p3z_IMin_6 _IQ24(-0.1);
#define CNTL_3p3z_Max_6 _IQ24(0.9);
#define CNTL_3p3z_Min_6 _IQ24(0.0);

#define CNTL_3p3z_A1_7 0.9999998212
#define CNTL_3p3z_A2_7 0.0000001788
#define CNTL_3p3z_A3_7 0.0000000000
#define CNTL_3p3z_B0_7 1.5979869793
#define CNTL_3p3z_B1_7 -0.1597843239
#define CNTL_3p3z_B2_7 -0.0000000286
#define CNTL_3p3z_B3_7 0.0000000000
#define CNTL_3p3z_IMin_7 _IQ24(-0.1);
#define CNTL_3p3z_Max_7 _IQ24(0.9);
#define CNTL_3p3z_Min_7 _IQ24(0.0);

#define CNTL_3p3z_A1_8 1.5000116470
#define CNTL_3p3z_A2_8 -0.5000116470
#define CNTL_3p3z_A3_8 0.0000000000
#define CNTL_3p3z_B0_8 1.9909697624
#define CNTL_3p3z_B1_8 -1.0589938796
#define CNTL_3p3z_B2_8 0.1040005785
#define CNTL_3p3z_B3_8 0.0000000000
#define CNTL_3p3z_IMin_8 _IQ24(-0.1);
#define CNTL_3p3z_Max_8 _IQ24(0.9);
#define CNTL_3p3z_Min_8 _IQ24(0.0);

#define CNTL_3p3z_A1_9 1.0000000000
#define CNTL_3p3z_A2_9 0.0000000000
#define CNTL_3p3z_A3_9 0.0000000000
#define CNTL_3p3z_B0_9 4.5050000000
#define CNTL_3p3z_B1_9 -4.4950000000
#define CNTL_3p3z_B2_9 0.0000000000
#define CNTL_3p3z_B3_9 0.0000000000
#define CNTL_3p3z_IMin_9 _IQ24(-0.1);
#define CNTL_3p3z_Max_9 _IQ24(0.9);
#define CNTL_3p3z_Min_9 _IQ24(0.0);

#define CNTL_3p3z_A1_10 1.0000000000
#define CNTL_3p3z_A2_10 0.0000000000
#define CNTL_3p3z_A3_10 0.0000000000
#define CNTL_3p3z_B0_10 0.2505000000
#define CNTL_3p3z_B1_10 -0.2495000000
#define CNTL_3p3z_B2_10 0.0000000000
#define CNTL_3p3z_B3_10 0.0000000000
#define CNTL_3p3z_IMin_10 _IQ24(-0.1);
#define CNTL_3p3z_Max_10 _IQ24(0.9);
#define CNTL_3p3z_Min_10 _IQ24(0.0);

//==================================================================================
// System Settings
//----------------------------------------------------------------------------------
//Add any system specific setting below
#define HistorySize     8   // Number of samples averaged for use in GUI
#define DLOG_SIZE       400

//==================================================================================
// Interrupt Framework options
//==================================================================================

#define EPWMn_DPL_ISR   1   // for EPWM triggered ISR set as 1
#define ADC_DPL_ISR     0   // for ADC INT 1 triggered ISR set as 1
#define CLAn_DPL_ISR    0   // for CLA Task n Triggered ISR set as 1

//----------------------------------------------------------------------------------
// If EPWMn_DPL_ISR = 1, then choose which module
//----------------------------------------------------------------------------------
#define EPWM1           1   // EPWM1 provides ISR trigger
#define EPWM2           1   // EPWM2 provides ISR trigger
#define EPWM3           0   // EPWM3 provides ISR trigger
#define EPWM4           0   // EPWM4 provides ISR trigger
#define EPWM5           0   // EPWM5 provides ISR trigger
#define EPWM6           0   // EPWM6 provides ISR trigger


#endif /* DEFINES_H_ */
