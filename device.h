/* =====================================================================*/
/* Program: template.h                                                  */
/*                                                                      */
/* Project: Template-Project                                            */
/*                                                                      */
/* Description: Template Description                                    */
/*                                                                      */
/* Last Edited: 2018-11-06                                              */
/*                                                                      */
/* Author: Falk Kyburz                                                  */
/*                                                                      */
/* ==================================================================== */
/*    (c) 2018 by Interstaatliche Hochschule für Technik Buchs NTB      */
/* ==================================================================== */


#ifndef DEVICE_H
#define DEVICE_H




/* ==================================================================== */
/* ========================== include files =========================== */
/* ==================================================================== */
#include <stdint.h>
#include "ringbuffer.h"
//#include "stdbool.h"
#include "defines.h"
#include "DSP2803x_Cla_typedefs.h"// DSP2803x CLA Type definitions
#include "DSP2803x_Device.h"      // DSP2803x Headerfile Include File

// Functions that will be run from RAM need to be assigned to
// a different section.  This section will then be mapped to a load and
// run address using the linker cmd file.
#pragma CODE_SECTION(InitFlash, "ramfuncs");
#define Device_cal (void   (*)(void))0x3D7C80


/* ==================================================================== */
/* ============================ constants ============================= */
/* ==================================================================== */

/* #define and enum statements go here */

/* ==================================================================== */
/* ========================== public data ============================= */
/* ==================================================================== */

/* Definition of public (external) data types go here */

/* ==================================================================== */
/* ======================= public functions =========================== */
/* ==================================================================== */

extern int32_t device_init(void);
extern void updateDutyEPwm(uint16_t duty);


/* ==================================================================== */
/* =================== public inline functions ======================== */
/* ==================================================================== */

extern inline int32_t device_ePWMSetPeriodAll(uint16_t count)
{
//    EPWM_setTimeBasePeriod(EPWM_GRID_1_BASE, count);
//    EPWM_setTimeBasePeriod(EPWM_GRID_2_BASE, count);
    return 0;
}

/*
 * Enable  ePWM outputs.
 * !!Use this for testing only!!
 * Registers are loaded on counter period.
 */
extern inline int32_t device_ePWMEnablePhaseU(void)
{
//    /* Remove Action Qualifier continuous software force */
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_A_BASE, EPWM_AQ_OUTPUT_A,
//                                             EPWM_AQ_SW_DISABLED);
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_A_BASE, EPWM_AQ_OUTPUT_B,
//                                             EPWM_AQ_SW_DISABLED);
//    /* Remove Action Qualifier continuous software force */
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_B_BASE, EPWM_AQ_OUTPUT_A,
//                                             EPWM_AQ_SW_DISABLED);
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_B_BASE, EPWM_AQ_OUTPUT_B,
//                                             EPWM_AQ_SW_DISABLED);
    return 0;
}

/*
 * Enable Phase V ePWM outputs.
 * !!Use this for testing only!!
 * Registers are loaded on counter period.
 */
extern inline int32_t device_ePWMEnablePhaseV(void)
{
//    /* Remove Action Qualifier continuous software force */
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_A_BASE, EPWM_AQ_OUTPUT_A,
//                                             EPWM_AQ_SW_DISABLED);
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_A_BASE, EPWM_AQ_OUTPUT_B,
//                                             EPWM_AQ_SW_DISABLED);
//    /* Remove Action Qualifier continuous software force */
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_B_BASE, EPWM_AQ_OUTPUT_A,
//                                             EPWM_AQ_SW_DISABLED);
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_B_BASE, EPWM_AQ_OUTPUT_B,
//                                             EPWM_AQ_SW_DISABLED);
    return 0;
}


/*
 * Enable Phase V ePWM outputs.
 * !!Use this for testing only!!
 * Registers are loaded on counter period.
 */
extern inline int32_t device_ePWMEnablePhaseAll(void)
{
//    device_ePWMEnablePhaseV(void);
//    device_ePWMEnablePhaseW(void);
    return 0;
}

/*
 * Enable  ePWM outputs.
 * !!Use this for testing only!!
 * Registers are loaded on counter period.
 */
extern inline int32_t device_ePWMDisablePhaseU(void)
{
//    /* Remove Action Qualifier continuous software force */
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_A_BASE, EPWM_AQ_OUTPUT_A,
//                                             EPWM_AQ_SW_DISABLED);
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_A_BASE, EPWM_AQ_OUTPUT_B,
//                                             EPWM_AQ_SW_DISABLED);
//    /* Remove Action Qualifier continuous software force */
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_B_BASE, EPWM_AQ_OUTPUT_A,
//                                             EPWM_AQ_SW_DISABLED);
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_B_BASE, EPWM_AQ_OUTPUT_B,
//                                             EPWM_AQ_SW_DISABLED);
    return 0;
}

/*
 * Enable Phase V ePWM outputs.
 * !!Use this for testing only!!
 * Registers are loaded on counter period.
 */
extern inline int32_t device_ePWMDisablePhaseV(void)
{
//    /* Remove Action Qualifier continuous software force */
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_A_BASE, EPWM_AQ_OUTPUT_A,
//                                             EPWM_AQ_SW_DISABLED);
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_A_BASE, EPWM_AQ_OUTPUT_B,
//                                             EPWM_AQ_SW_DISABLED);
//    /* Remove Action Qualifier continuous software force */
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_B_BASE, EPWM_AQ_OUTPUT_A,
//                                             EPWM_AQ_SW_DISABLED);
//    EPWM_setActionQualifierContSWForceAction(EPWM_PV2_B_BASE, EPWM_AQ_OUTPUT_B,
//                                             EPWM_AQ_SW_DISABLED);
    return 0;
}





/*
 * !!Use this for testing only!!
 * Registers are loaded on counter period.
 */
extern inline int32_t device_ePWMEnableAll(void)
{
//    device_ePWMEnablePhaseV(void);
//    device_ePWMEnablePhaseU(void);
    return 0;
}

extern inline int32_t device_ePWMDisableAll(void)
{
//    device_ePWMDisablePhaseV(void);
//    device_ePWMDisablePhaseW(void);
    return 0;
}


/*
 * Enable gate drivers for Phase U
 */
extern inline int32_t device_driverEnableVLS(void)
{
    GpioDataRegs.GPADAT.bit.GPIO11 = 0;
    return 0;
}

extern inline int32_t device_driverEnableVHS(void)
{
    GpioDataRegs.GPBDAT.bit.GPIO43 = 0;
    return 0;
}

/*
 * Enable gate drivers for Phase V
 */
extern inline int32_t device_driverEnableULS(void)
{
    GpioDataRegs.GPADAT.bit.GPIO4 = 0;
    return 0;
}

extern inline int32_t device_driverEnableUHS(void)
{
    GpioDataRegs.GPADAT.bit.GPIO5 = 0;
    return 0;
}

/*
 * Enable gate drivers for Phase U
 */
extern inline int32_t device_driverDisableVLS(void)
{
    GpioDataRegs.GPADAT.bit.GPIO11 = 1;
    return 0;
}

extern inline int32_t device_driverDisableVHS(void)
{
    GpioDataRegs.GPBDAT.bit.GPIO43 = 1;
    return 0;
}

/*
 * Enable gate drivers for Phase V
 */
extern inline int32_t device_driverDisableULS(void)
{
    GpioDataRegs.GPADAT.bit.GPIO4 = 1;
    return 0;
}

extern inline int32_t device_driverDisableUHS(void)
{
    GpioDataRegs.GPADAT.bit.GPIO5 = 1;
    return 0;
}

extern inline int32_t device_setALLDriverDisable(void)
{
    device_driverDisableULS();
    device_driverDisableVLS();
    device_driverDisableUHS();
    device_driverDisableVHS();

    return 0;
}

extern inline int32_t device_setALLDriverEnable(void)
{
    device_driverEnableULS();
    device_driverEnableVLS();
    device_driverEnableUHS();
    device_driverEnableVHS();

    return 0;
}


#endif // DEVICE_H


