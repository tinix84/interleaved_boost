#ifndef DEFINES_H_
#define DEFINES_H_

/* ==================================================================== */
/* ========================== include files =========================== */
/* ==================================================================== */

#include "DSP2803x_Cla_typedefs.h"              // DSP2803x CLA Type definitions
#include "DSP2803x_Device.h"                    //DSP2803x Headerfile Include File
#include "DSP2803x_Examples.h"                  // Examples Include File

#include "DSP2803x_GlobalPrototypes.h"          // Prototypes for global functions within the
                                                // .c files.
#include "DSP2803x_EPwm_defines.h"             // Macros used for PWM examples.
#include "DSP2803x_I2c_defines.h"              // Macros used for I2C examples.
#include "DSP2803x_Cla_defines.h"              // Macros used for CLA examples.


//// The following pointer to a function call calibrates the ADC and internal oscillators
//#define Device_cal (void   (*)(void))0x3D7C80
//#define   PARTNO_28035PAG   0xBE
//#define   PARTNO_28035PN    0xBF

//// Include files not used with DSP/BIOS
//#ifndef DSP28_BIOS
//#include "DSP2803x_DefaultISR.h"
//#endif
//// DO NOT MODIFY THIS LINE.
//#define DELAY_US(A)  DSP28x_usDelay(((((long double) A * 1000.0L) / (long double)CPU_RATE) - 9.0L) / 5.0L)

/* Compilation define */
#define CLOSE_LOOP 0
//#define INVERTEDPOWER

/* System control define */
///* Specify the PLL control register (PLLCR) and divide select (DIVSEL) value.*/
//#define DSP28_DIVSEL   2 // Enable /2 for SYSCLKOUT
//#define DSP28_PLLCR   12    // Uncomment for 60 MHz devices [60 MHz = (10MHz * 12)/2]
#define CPU_RATE   16.667L   // for a 60MHz CPU clock speed (SYSCLKOUT)
#define DEVICE_SYSCLK_FREQ (60000000U)

/* EPWM control defines */
#define INTERLEAVE 2 //interleave half the switching frequency
#define UPDOWN_FACTOR 2 //symmetrical carrier half the switching frequency
#define UP_FACTOR 1 //unipolar carrier half the switching frequency

#ifdef INVERTEDPOWER
#define CARRIER_FACTOR UP_FACTOR
#else
#define CARRIER_FACTOR UPDOWN_FACTOR
#endif

      // Clock ratio to SYSCLKOUT
#define EPWM_CLK_FREQ          DEVICE_SYSCLK_FREQ //HSPCLKDIV = TB_DIV1; CLKDIV = TB_DIV1;
#define EPWM_CLK_PERIOD_NS     (1000000000U / EPWM_CLK_FREQ)
#define EPWM_PWM_DEAD_TIME_NS  500U
#define EPWM_PWM_FREQ_25KHZ    25000U

#define EPWM_PERIOD_COUNT_25KHZ (EPWM_CLK_FREQ / EPWM_PWM_FREQ_25KHZ)
//#define EPWM_PERIOD_COUNT_25KHZ_F 4000.0f
#define EPWM_COUNT_25KHZ_DUTY50 (EPWM_PERIOD_COUNT_25KHZ / 2U)
#define EPWM_COUNT_25KHZ_DUTY75 (EPWM_PERIOD_COUNT_25KHZ * 3U / 4U)

#define EPWM_PWM_DEAD_TIME_COUNT (EPWM_PWM_DEAD_TIME_NS / EPWM_CLK_PERIOD_NS)

/* EPWM defines for initialization, based on section 3.2.2.3 reference manual */
#define EPWMx_INIT_PERIOD    (uint16_t)(EPWM_PERIOD_COUNT_25KHZ / CARRIER_FACTOR)
#define EPWMx_INIT_PHASE     (uint16_t)(EPWM_PERIOD_COUNT_25KHZ / INTERLEAVE)
#define EPWMx_INIT_CMPA      (uint16_t)(EPWM_COUNT_25KHZ_DUTY75 / CARRIER_FACTOR)
#define EPWMx_INIT_DEADBAND  (uint16_t)(EPWM_PWM_DEAD_TIME_COUNT)

#define EPWM_A_INIT_PERIOD       EPWMx_INIT_PERIOD
#define EPWM_A_INIT_PHASE        0U
#define EPWM_A_INIT_CMPA         EPWMx_INIT_CMPA
#define EPWM_A_INIT_DEADBAND     EPWMx_INIT_DEADBAND

#define EPWM_B_INIT_PERIOD       EPWMx_INIT_PERIOD
#define EPWM_B_INIT_PHASE        EPWMx_INIT_PHASE
#define EPWM_B_INIT_CMPA         EPWMx_INIT_CMPA
#define EPWM_B_INIT_DEADBAND     EPWMx_INIT_DEADBAND

///* Low frequency peripheries */
//#define LSPCLK_FREQ DEVICE_SYSCLK_FREQ/4
//#define SCI_FREQ    100E3
//#define SCI_PRD     (LSPCLK_FREQ/(SCI_FREQ*8))-1

/* Serial communication */
#define COMM_MAX_DATA_LENGTH 4U
#define COMM_START 0x55U
#define SCI_BUFFER_SIZE 16

///*ADC defines*/
//#define ADC_usDELAY  1000L

// ==================================================================== */
// macros
// ==================================================================== */
/* Int macros */
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define ABS(a)   (((a)>(0))?(a):(-(a)))
#define SAT(a, amax, amin) max(min((a), (amax)), (amin));
/* float macros for CPU */
#define MIN32(a, b) __min((a), (b))
#define MAX32(a, b) __max((a), (b))
#define ABS32(a) __max((-a),(a))
#define SAT32(a, amax, amin) __max(__min((a), (amax)), (amin));
#define RATE32(target, var, step) ((var) + __max(__min(((target)-(var)), (step)), -(step)))

//==================================================================================
// Comp Settings
//----------------------------------------------------------------------------------
#define Vref_default 10.0f
#define Iref_default 0.5f


#endif /* DEFINES_H_ */
