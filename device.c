/* =====================================================================*/
/* Program: device.c                                                    */
/*                                                                      */
/* Project:                                                             */
/*                                                                      */
/* Description: Hardware driver                                         */
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
#include <stdbool.h>
#include <stddef.h>
#include "application.h"
#include "device.h"
#include "defines.h"
//#include "ringbuffer.h"

#include "DSP2803x_Cla_typedefs.h"// DSP2803x CLA Type definitions
#include "DSP2803x_Device.h"      // DSP2803x Headerfile Include File
#include "CLAShared.h"
#include "DSP2803x_Adc.h"

/* ==================================================================== */
/* ====================== private functions =========================== */
/* ==================================================================== */
static int32_t device_initSysCtl(void);
static int32_t device_initCPUTimer(void);
static int32_t device_initCPUMemory(void);

static int32_t device_initCLA(void);
static int32_t device_initSCIA(void);


#ifdef INVERTEDPOWER
static int32_t device_initEPWM3phNIBBSpecialModulation(void);
#else
static int32_t device_initEPWM(void);
#endif

static int32_t device_initADC(void);
static int32_t device_initGPIO(void);

void __error__(char *filename, uint32_t line);



/* ==================================================================== */
/* ================= interrupt service functions ====================== */
/* ==================================================================== */

__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void AdcInterruptISR(void);
//__interrupt void sciaTxIsr(void);
__interrupt void sciaRXISR(void);
__interrupt void ISR_ILLEGAL(void);


/* ==================================================================== */
/* ===================== All functions by section ===================== */
/* ==================================================================== */

/*
 * Perform initialization of all peripherals in the correct order
 */
int32_t device_init(void)
{
    int32_t err = NO_ERROR;

    // Step 1. Initialize System Control:
    /* Initialize PLL, calibrations and clock system*/
    device_initSysCtl();
    /* Initialize CPU Memory */
    device_initCPUMemory();

    // Step 2. Initialize GPIO:
    /* Initialize board related peripherals */
    // Connect ePWM1, ePWM2, ePWM3 to GPIO pins, so that in not necessary toggle function in ISR
    device_initGPIO();
    //    // Setup only the GPI/O only for SCI-A and SCI-B functionality
    //    // This function is found in DSP2803x_Sci.c
    //    InitSciaGpio();

    /* Initialize the CLA Memory */
    device_initCLA();
    /* Init CPU Timers */
    device_initCPUTimer();

    /* Init watchdog */

    /* Initialize all peripherals */
    device_initSCIA();
#ifdef INVERTEDPOWER
    device_initEPWM3phNIBBSpecialModulation();
#else
    device_initEPWM();
#endif

    device_initADC();
    //device_initDAC();

    DINT;
    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags are cleared.
    InitPieCtrl();
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    InitPieVectTable();

    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.TINT1 = &cpu_timer1_isr;

    PieVectTable.SCIRXINTA = &sciaRXISR;
    PieVectTable.ADCINT1 = &AdcInterruptISR;

    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // ADCINT1
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1; // Enable TINT0 in the PIE: Group 1 interrupt 7
    // Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;

    // ????
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER9.bit.INTx1=1;     // PIE Group 9, INT1
    PieCtrlRegs.PIEIER9.bit.INTx2=1;     // PIE Group 9, INT2

    IER = 0x100; // Enable CPU INT
    // Enable CPU INT1 which is connected to CPU-Timer 0:
    IER |= M_INT1;
    IER |= M_INT2;
    // Enable CPU INT3 which is connected to EPWM1-3 INT:
    IER |= M_INT3;
    // Enable CPU INT13 which is connected to ADC INT:
    IER |= M_INT13;


    // Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM
    EDIS;    // This is needed to disable write to EALLOW protected registers

    return err;
}

static int32_t device_initSysCtl(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    InitSysCtrl();
    InitPeripheralClocks();
    return NO_ERROR;
}

/* it starts also the timers */
static int32_t device_initCPUTimer(void)
{
    EALLOW;
    // Timing sync for background loops
    // Timer period definitions found in PeripheralHeaderIncludes.h
    CpuTimer0Regs.PRD.all =  mSec0_5;   // A tasks
    CpuTimer1Regs.PRD.all =  mSec500;    // B tasks
    CpuTimer2Regs.PRD.all =  mSec500;    // C tasks

    CpuTimer0Regs.TCR.bit.TIE = 1;
    CpuTimer1Regs.TCR.bit.TIE = 1;
    CpuTimer2Regs.TCR.bit.TIE = 1;

    EDIS;

    return NO_ERROR;

}

/*
 * Set CPU Memory and copy RAM functions to RAM.
 */
static int32_t device_initCPUMemory(void)
{
    extern Uint16 RamfuncsLoadStart;
    extern Uint16 RamfuncsLoadSize;
    extern Uint16 RamfuncsRunStart;

    // Only used if running from FLASH
    // Note that the variable FLASH is defined by the compiler with -d FLASH
#ifdef FLASH
    // Copy time critical code and Flash setup code to RAM
    // The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the linker files.
    memcpy((uint16_t *)&RamfuncsRunStart,(uint16_t *)&RamfuncsLoadStart, (unsigned long)&RamfuncsLoadSize);;
    //    memcpy((uint16_t *)&Cla1funcsLoadStart, (uint16_t *)&Cla1funcsRunStart, (unsigned long)&Cla1funcsLoadSize);

    // Call Flash Initialization to setup flash waitstates
    // This function must reside in RAM
    InitFlash();    // Call the flash wrapper init function
#endif //(FLASH)

    return NO_ERROR;
}

/*
 * SCIA is used for communication to the PC
 * A ring buffer is used without the internal FIFO
 */
static int32_t device_initSCIA(void)
{
    extern ringbuffer_t SCIATXBufferStruct;
    extern ringbuffer_t SCIARXBufferStruct;
    extern uint16_t SCIATXBuffer[SCI_BUFFER_SIZE];
    extern uint16_t SCIARXBuffer[SCI_BUFFER_SIZE];

    /*init peripherical*/
    SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback  No parity,8 char bits, async mode, idle-line protocol
    SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
    // Disable RX ERR, SLEEP, TXWAKE
    //SciaRegs.SCICTL2.bit.TXINTENA =0;
    SciaRegs.SCICTL2.bit.RXBKINTENA =1;
    SciaRegs.SCIHBAUD = 0x0000;
    SciaRegs.SCILBAUD = 0x00C2; //9600;
    SciaRegs.SCICCR.bit.LOOPBKENA =0; // Enable loop back
    SciaRegs.SCICTL1.all = 0x0023;     // Relinquish SCI from Reset

    /* Initialize ring buffer */
    SCIATXBufferStruct.buffer = SCIATXBuffer;
    SCIARXBufferStruct.buffer = SCIARXBuffer;
    SCIATXBufferStruct.size = SCI_BUFFER_SIZE;
    SCIARXBufferStruct.size = SCI_BUFFER_SIZE;
    ringbuffer_reset(&SCIATXBufferStruct);
    ringbuffer_reset(&SCIARXBufferStruct);

    return NO_ERROR;
}

#ifdef INVERTEDPOWER
static int32_t device_initEPWM3phNIBBSpecialModulation(void)
{
    //=====================================================================
    // (Note: code for only 3 modules shown)
    // Initialization Time
    //========================
    // EPWM Module 1 config
//    EPwm1Regs.CMPA.half.CMPA = EPWM_B_INIT_DEADBAND; // adjust duty for output EPWM1A
//    EPwm2Regs.CMPA.half.CMPA = EPWM_B_INIT_DEADBAND;
//    EPwm3Regs.CMPA.half.CMPA = EPWM_B_INIT_DEADBAND;

    EPwm1Regs.CMPB = EPWM_A_INIT_CMPA; // adjust duty for output EPWM1A
    EPwm2Regs.CMPB = EPWM_A_INIT_CMPA; // adjust duty for output EPWM2A


    EPwm1Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;
    EPwm2Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;


    EPwm1Regs.TBPRD = EPWM_A_INIT_PERIOD; // Period = TBCLK counts
    EPwm2Regs.TBPRD = EPWM_A_INIT_PERIOD; // Period = 1401 TBCLK counts


    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Asymmetrical mode
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Asymmetrical mode


    EPwm1Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
    EPwm2Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero


    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Phase loading disabled
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Phase loading enable
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Phase loading disabled

    EPwm1Regs.TBCTL.bit.PHSDIR = TB_DOWN;
    EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN;


    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_CMPB;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_CTR_CMPB;

//    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; // every period set PWM
//    EPwm1Regs.AQCTLA.bit.CBU = AQ_CLEAR ; //if PWMA=CMPA on rising edge disable PWM
//    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET; // every period set PWM
//    EPwm2Regs.AQCTLA.bit.CBU = AQ_CLEAR ; //if PWMA=CMPA on rising edge disable PWM

    EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR; // every period set PWM
    EPwm1Regs.AQCTLA.bit.CBU = AQ_SET ; //if PWMA=CMPA on rising edge disable PWM
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR; // every period set PWM
    EPwm2Regs.AQCTLA.bit.CBU = AQ_SET ; //if PWMA=CMPA on rising edge disable PWM

    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active Hi complementary
    EPwm1Regs.DBFED = EPWM_B_INIT_DEADBAND; // FED = 20 TBCLKs
    EPwm1Regs.DBRED = EPWM_B_INIT_DEADBAND; // RED = 20 TBCLKs

    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable dead-band module
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active Hi Complementary
    EPwm2Regs.DBFED = EPWM_B_INIT_DEADBAND; // FED = 20 TBCLKs
    EPwm2Regs.DBRED = EPWM_B_INIT_DEADBAND; // RED = 20 TBCLKs


    // ADC trigger
    EPwm1Regs.ETSEL.bit.SOCAEN   = 1; // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL  = ET_CTR_ZERO;
    EPwm1Regs.ETPS.bit.SOCAPRD   = ET_1ST;
    EPwm1Regs.ETSEL.bit.SOCBEN   = 1;        // Enable SOC on B group
    EPwm1Regs.ETSEL.bit.SOCBSEL  = ET_CTR_ZERO;        // Select SOC from from CPMA on upcount
    EPwm1Regs.ETPS.bit.SOCBPRD   = ET_1ST;        // Generate pulse on 1st event

    return NO_ERROR;
}

#else

static int32_t device_initEPWM(void)
{
    //=====================================================================
    // Config
    // Initialization Time
    //===========================================================================
    // EPWM Module 1 config
    EPwm1Regs.TBPRD = EPWM_A_INIT_PERIOD; // Period = 900 TBCLK counts
    EPwm1Regs.TBPHS.half.TBPHS = EPWMx_INIT_PHASE; // Set Phase register to zero
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Master module
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // Sync down-stream module
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // TBCLK = SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM1A
    EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable Dead-band module
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active Hi complementary
    EPwm1Regs.DBFED = EPWM_A_INIT_DEADBAND; // FED = 20 TBCLKs
    EPwm1Regs.DBRED = EPWM_A_INIT_DEADBAND; // RED = 20 TBCLKs

    // ADC trigger
    EPwm1Regs.ETSEL.bit.SOCAEN   = 1; // Enable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL  = ET_CTR_ZERO;
    EPwm1Regs.ETPS.bit.SOCAPRD   = ET_1ST;
    EPwm1Regs.ETSEL.bit.SOCBEN   = 1;        // Enable SOC on B group
    EPwm1Regs.ETSEL.bit.SOCBSEL  = ET_CTR_ZERO;        // Select SOC from from CPMA on upcount
    EPwm1Regs.ETPS.bit.SOCBPRD   = ET_1ST;        // Generate pulse on 1st event

    // EPWM Module 2 config
    EPwm2Regs.TBPRD = EPWM_B_INIT_PERIOD; // Period = 900 TBCLK counts
    EPwm2Regs.TBPHS.half.TBPHS = EPWM_B_INIT_PHASE; // Phase = 300/900 * 360 = 120 deg
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
    EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN; // Count DOWN on sync (=120 deg)
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // TBCLK = SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
    EPwm2Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM2A
    EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // enable dead-band module
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // Active Hi Complementary
    EPwm2Regs.DBFED = EPWM_B_INIT_DEADBAND; // FED = 20 TBCLKs
    EPwm2Regs.DBRED = EPWM_B_INIT_DEADBAND; // RED = 20 TBCLKs

    // Run Time (Note: Example execution of one run-time instant)
    //===========================================================
    EPwm1Regs.CMPA.half.CMPA = EPWM_A_INIT_CMPA; // adjust duty for output EPWM1A
    EPwm2Regs.CMPA.half.CMPA = EPWM_B_INIT_CMPA; // adjust duty for output EPWM2A

    return NO_ERROR;
}
#endif

static int32_t device_initGPIO(void)
{
    EALLOW;
    //--------------------------------------------------------------------------------------
    // GPIO (GENERAL PURPOSE I/O) CONFIG
    //--------------------------------------------------------------------------------------
    //  GPIO-00 - PIN FUNCTION = PWM_ULS
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;     // 0=GPIO,  1=EPWM1A,  2=Resv,  3=Resv
    //  GPIO-01 - PIN FUNCTION = PWM_UHS
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;     // 0=GPIO,  1=EPWM1B,  2=Resv,  3=COMP1OUT
    //  GPIO-02 - PIN FUNCTION = PWM_VLS
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;     // 0=GPIO,  1=EPWM2A,  2=Resv,  3=Resv
    //  GPIO-03 - PIN FUNCTION = PWM_VHS
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;     // 0=GPIO,  1=EPWM2B,  2=SPISOMI-A,  3=COMP2OUT
    //  GPIO-04 - PIN FUNCTION = RST_ULS
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;     // 0=GPIO,  1=EPWM3B,  2=Resv,  3=COMP1OUT
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;      // 1=OUTput,  0=INput
    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;    // uncomment if --> Set Low initially
    //  GPIO-05 - PIN FUNCTION = RST_UHS
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;     // 0=GPIO,  1=EPWM3B,  2=SPISOMI-A,  3=COMP2OUT
    GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;      // 1=OUTput,  0=INput
    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;    // uncomment if --> Set Low initially
    //  GPIO-06 - PIN FUNCTION = Enable PWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;     // 0=GPIO,  1=EPWM4A,  2=SYNCI,  3=SYNCO
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;      // 1=OUTput,  0=INput
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;    // uncomment if --> Set Low initially
    //  GPIO-07 - PIN FUNCTION = Enable PWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;     // 0=GPIO,  1=EPWM4B,  2=SCIRX-A,  3=Resv
    //  GPIO-08 - PIN FUNCTION = --Spare--
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;     // 0=GPIO,  1=EPWM5A,  2=Resv,  3=ADCSOC-A
    //  GPIO-09 - PIN FUNCTION = --Spare--
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;     // 0=GPIO,  1=EPWM5B,  2=LINTX-A,  3=Resv
    //  GPIO-10 - PIN FUNCTION = --Spare--
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;    // 0=GPIO,  1=EPWM6A,  2=Resv,  3=ADCSOC-B
    GpioCtrlRegs.GPADIR.bit.GPIO10 = 0;     // 1=OUTput,  0=INput
    //  GPIO-11 - PIN FUNCTION = RST_VLS
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;    // 0=GPIO,  1=EPWM6B,  2=LINRX-A,  3=Resv
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;     // 1=OUTput,  0=INput
    GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;   // uncomment if --> Set Low initially
    //  GPIO-28 - PIN FUNCTION = SCI-RX
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SCIRXDA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;    // 0=GPIO,  1=SCIRX-A,  2=I2CSDA-A,  3=TZ2
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
    //  GPIO-29 - PIN FUNCTION = SCI-TX
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up for GPIO29 (SCITXDA)
    GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;    // 0=GPIO,  1=SCITXD-A,  2=I2CSCL-A,  3=TZ3
    //  GPIO-34 - PIN FUNCTION = LED3 on controlCARD
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;    // 0=GPIO,  1=Resv,  2=Resv,  3=Resv
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;     // 1=OUTput,  0=INput
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;     // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    // GPIO 35-38 are defaulted to JTAG usage, and are not shown here to enforce JTAG debug
    // usage.
    //--------------------------------------------------------------------------------------
    //  GPIO-39 - PIN FUNCTION = --Spare--
    GpioCtrlRegs.GPBMUX1.bit.GPIO39 = 0;    // 0=GPIO,  1=Resv,  2=Resv,  3=Resv
    GpioCtrlRegs.GPBDIR.bit.GPIO39 = 0;     // 1=OUTput,  0=INput
    //  GPIO-40 - PIN FUNCTION = --Spare--
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 0;    // 0=GPIO,  1=EPWM7A,  2=Resv,  3=Resv
    GpioCtrlRegs.GPBDIR.bit.GPIO40 = 0;     // 1=OUTput,  0=INput
    //  GPIO-41 - PIN FUNCTION = --Spare--
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 0;    // 0=GPIO,  1=EPWM7B,  2=Resv,  3=Resv
    GpioCtrlRegs.GPBDIR.bit.GPIO41 = 0;     // 1=OUTput,  0=INput
    //  GPIO-42 - PIN FUNCTION = LED2
    GpioCtrlRegs.GPBMUX1.bit.GPIO42 = 0;    // 0=GPIO,  1=Resv,  2=Resv,  3=COMP1OUT
    GpioCtrlRegs.GPBDIR.bit.GPIO42 = 1;     // 1=OUTput,  0=INput
    GpioDataRegs.GPBSET.bit.GPIO42 = 1;     // uncomment if --> Set High initially
    //  GPIO-43 - PIN FUNCTION = RST_VHS
    GpioCtrlRegs.GPBMUX1.bit.GPIO43 = 0;    // 0=GPIO,  1=Resv,  2=Resv,  3=COMP2OUT
    GpioCtrlRegs.GPBDIR.bit.GPIO43 = 1;     // 1=OUTput,  0=INput
    GpioDataRegs.GPBCLEAR.bit.GPIO43 = 1;   // uncomment if --> Set Low initially
    //  GPIO-44 - PIN FUNCTION = LED1
    GpioCtrlRegs.GPBMUX1.bit.GPIO44 = 0;    // 0=GPIO,  1=Resv,  2=Resv,  3=Resv
    GpioCtrlRegs.GPBDIR.bit.GPIO44 = 1;     // 1=OUTput,  0=INput
    GpioDataRegs.GPBSET.bit.GPIO44 = 1;     // uncomment if --> Set High initially
    EDIS;

    return NO_ERROR;
}

static int32_t device_initADC(void)
{
    // Assumes ADC clock is already enabled in InitSysCtrl();
    // Call the InitAdc function in the DSP2803x_Adc.c file
    // This function calibrates and powers up the ADC to
    // into a known state.

    InitAdc();
    AdcOffsetSelfCal();

    EALLOW;
    AdcRegs.ADCCTL2.bit.CLKDIV2EN = 0;  // ADC Clock - 60 MHz
    EDIS;
//    DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels

    // Configure ADC
    EALLOW;
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;  // Enable non-overlap mode
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;    // ADCINT1 trips after AdcResults latch
    AdcRegs.INTSEL1N2.bit.INT1E     = 1;    // Enabled ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1SEL   = 1;    // setup EOC1 to trigger ADCINT1 to fire

    AdcRegs.ADCINTFLG.bit.ADCINT1 = 0;      // clear interrupt flag for ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT = 0;     // set ADCInterrupt 1 to auto clr
    AdcRegs.ADCINTSOCSEL1.all = 0x0000;     // No ADCInterrupt will trigger SOCx
    AdcRegs.ADCINTSOCSEL2.all = 0x0000;

    AdcRegs.ADCSOC0CTL.bit.CHSEL    = 2;    // A2 - Ifb_Sum
    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 5;    // A2 - Ifb_Sum -> double sampling for errata silicon
    AdcRegs.ADCSOC2CTL.bit.CHSEL    = 5;    // A5 - Ifb_W
    AdcRegs.ADCSOC3CTL.bit.CHSEL    = 6;    // A6 - Vfb_Bus
    AdcRegs.ADCSOC4CTL.bit.CHSEL    = 7;    // A7 - Vfb_Bus
    AdcRegs.ADCSOC5CTL.bit.CHSEL    = 11;   // B3 - Ifb_U
    AdcRegs.ADCSOC6CTL.bit.CHSEL    = 12;   // B4 - Vfb_W
    AdcRegs.ADCSOC7CTL.bit.CHSEL    = 13;   // B5 - Ifb_V
    AdcRegs.ADCSOC8CTL.bit.CHSEL    = 14;   // B6 - Vfb_V
    AdcRegs.ADCSOC9CTL.bit.CHSEL    = 15;   // B7 - Vfb_U

    // ADCTRIG5 – ePWM1, ADCSOCA
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC6CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC7CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC8CTL.bit.TRIGSEL  = 5;
    AdcRegs.ADCSOC9CTL.bit.TRIGSEL  = 5;

    // set SOC0 S/H Window to 9 ADC Clock Cycles, (8 ACQPS plus 1)
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 8;
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 8;
    AdcRegs.ADCSOC2CTL.bit.ACQPS    = 8;
    AdcRegs.ADCSOC3CTL.bit.ACQPS    = 8;
    AdcRegs.ADCSOC4CTL.bit.ACQPS    = 8;
    AdcRegs.ADCSOC5CTL.bit.ACQPS    = 8;
    AdcRegs.ADCSOC6CTL.bit.ACQPS    = 8;
    AdcRegs.ADCSOC7CTL.bit.ACQPS    = 8;
    AdcRegs.ADCSOC8CTL.bit.ACQPS    = 8;
    AdcRegs.ADCSOC9CTL.bit.ACQPS    = 8;
    EDIS;

    return NO_ERROR;
}

static int32_t device_initCLA(void)
{
    //
    // These are defined by the linker file and used to copy
    // the CLA code from its load address to its run address
    // in CLA program memory
    //
    extern Uint16 Cla1funcsLoadStart;
    extern Uint16 Cla1funcsLoadEnd;
    extern Uint16 Cla1funcsLoadSize;
    extern Uint16 Cla1funcsRunStart;
    extern Uint16 Cla1Prog_Start;

    // This code assumes the CLA clock is already enabled in
    // the call to DevInit();
    // The symbols used in this calculation are defined in the CLA
    // assembly code and in the CLAShared.h header file

    EALLOW;
    Cla1Regs.MVECT1 = (Uint16)((Uint32)&Cla1Task1 -(Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT7 = (Uint16)((Uint32)&Cla1Task7 -(Uint32)&Cla1Prog_Start);
    Cla1Regs.MVECT8 = (Uint16)((Uint32)&Cla1Task8 -(Uint32)&Cla1Prog_Start);


    // Copy the CLA program code from its load address to the CLA program memory
    // Once done, assign the program memory to the CLA
    //
    // Make sure there are at least two SYSCLKOUT cycles between assigning
    // the memory to the CLA and when an interrupt comes in
    // Call this function even if Load and Run address is the same!
    memcpy((uint16_t *)&Cla1funcsRunStart,(uint16_t *)&Cla1funcsLoadStart, (unsigned long)&Cla1funcsLoadSize);

    asm("   RPT #3 || NOP");

    Cla1Regs.MMEMCFG.bit.PROGE = 1;          // Map CLA program memory to the CLA
    Cla1Regs.MMEMCFG.bit.RAM0E   = 1;
    Cla1Regs.MMEMCFG.bit.RAM1E   = 1;

    // Enable the IACK instruction to start a task
    // Enable the CLA interrupt 8 and interrupt 2
    asm("   RPT #3 || NOP");

    Cla1Regs.MCTL.bit.IACKE = 1;             // Enable IACK to start tasks via software
    Cla1Regs.MPISRCSEL1.bit.PERINT1SEL  = CLA_INT1_ADCINT1;
    Cla1Regs.MIER.all = (M_INT8 | M_INT7 | M_INT1);   // Enable Task 8 , Task 7 and Task 1

    asm("   RPT #3 || NOP"); 

    Cla1ForceTask8();

    EDIS;

    return NO_ERROR;
}


__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// interrupt routine for led blinking and rate limiter
__interrupt void cpu_timer1_isr(void)
{
    extern uint16_t Timer1IntFlg;
    extern float new_Vout;
    extern uint32_t max_Vout_step;
    extern float cla_VoutU;
    extern float cla_VrefU;
    int16_t direction = 1;
    float abs_dv = 0;


    CpuTimer1.InterruptCount++;
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Toggle GPIO34 once per 500 milliseconds

    if (new_Vout <= cla_VoutU)
        direction = -1;
    else
        direction = 1;

    abs_dv=(ABS(new_Vout-cla_VoutU));

    if ( abs_dv > max_Vout_step)
    {
        cla_VrefU = (float)((cla_VoutU + max_Vout_step*direction));
        cla_VrefV = (float)((cla_VoutV + max_Vout_step*direction));
        cla_VrefW = (float)((cla_VoutW + max_Vout_step*direction));
    }
    else
    {
        cla_VrefU = new_Vout;
        cla_VrefV = new_Vout;
        cla_VrefW = new_Vout;
    }

    // Clear INT flag for this timer
    EPwm1Regs.ETCLR.bit.INT = 1;
    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void AdcInterruptISR(void) {
    GpioDataRegs.GPASET.bit.GPIO18 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    return;
}

/* sciaRXFIFOISR - SCIA Receive FIFO ISR */
__interrupt void sciaRXISR(void)
{
    extern ringbuffer_t SCIARXBufferStruct;
    uint16_t data;

    /* Read data from RX FIFO */
    data = SciaRegs.SCIRXBUF.all;
    /* Write data to ring buffer */
    ringbuffer_put(&SCIARXBufferStruct, data);

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
}

__interrupt void ISR_ILLEGAL(void)   // Illegal operation TRAP
{
    // Insert ISR Code here
    //TODO: all gate drivers off

    // Next two lines for debug only to halt the processor here
    // Remove after inserting ISR Code
    asm("          ESTOP0");
    for(;;);

}
