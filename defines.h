#ifndef DEFINES_H_
#define DEFINES_H_

/* ==================================================================== */
/* ========================== include files =========================== */
/* ==================================================================== */

#include "DSP2803x_Cla_typedefs.h"              // DSP2803x CLA Type definitions
#include "DSP2803x_Device.h"                    //DSP2803x Headerfile Include File
#include "DSP2803x_GlobalPrototypes.h"          // Prototypes for global functions within the
                                                // .c files.
#include "DSP2803x_EPwm_defines.h"             // Macros used for PWM examples.
#include "DSP2803x_I2c_defines.h"              // Macros used for I2C examples.
#include "DSP2803x_Cla_defines.h"              // Macros used for CLA examples.


/* ==================================================================== */
/* ==================================================================== */


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
#define EPWMx_INIT_CMPA      (uint16_t)(EPWM_COUNT_25KHZ_DUTY50 / CARRIER_FACTOR)
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

///* Control loop define */
//#define PWM_NO              1
//#define ADC_TRIG_SOURCE     5
//#define ADC_PIN_VOUT        2
//#define ADC_PIN_VIN_L       10
//#define ADC_PIN_VIN_N       8
//#define ADC_PIN_IL_AVG      4
//#define ADC_IL_COMPARATOR   2
//#define SFRA_TYPE           1
//
//#define VIN_MAX_SENSE_V       550
//#define VBUS_MAX_SENSE_V      510.99
//#define IL_MAX_SENSE_V        19.8
//
//#define VBUS_MIN_VOLTS    600.0
//#define VBUS_RATED_VOLTS    750.0
//#define VBUS_OVP_THRSHLD_VOLTS    800.0
//
//#define VBUS_MIN_LSB ((int32)((100.0/VBUS_MAX_SENSE)*4095*4095))
//#define VBUS_OVP_THRSHLD_LSB ((int32)((435.0/VBUS_MAX_SENSE)*4095*4095)) //435V
//
//#define VBUS_TARGET         ((int32)((VBUS_RATED_VOLTS/VBUS_MAX_SENSE)*4095*4095)) //395V
//#define VBUS_ERROR_NL_CNTRL_THRSHLD ((int32)((15.0/VBUS_MAX_SENSE)*4095*4095)) //Vbus error threshold to activate NL Vloop control

/*ADC defines*/
#define ADC_usDELAY  1000L

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
#define Vref_default 10.0f
#define Iref_default 0.5f


//==================================================================================
// System Settings
//----------------------------------------------------------------------------------


//==================================================================================
// Interrupt Framework options
//==================================================================================


//----------------------------------------------------------------------------------
// If EPWMn_DPL_ISR = 1, then choose which module
//----------------------------------------------------------------------------------


#endif /* DEFINES_H_ */
