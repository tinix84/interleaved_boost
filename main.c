//###########################################################################
// Description:
//
//
//###########################################################################
// $TI Release: F2803x C/C++ Header Files and Peripheral Examples V130 $
// $Release Date: May  8, 2015 $
// $Copyright: Copyright (C) 2009-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "ringbuffer.h"
#include "commands.h"


// Prototype statements for functions found within this file.
__interrupt void cpu_timer0_isr(void);
__interrupt void epwm1_isr(void);
__interrupt void epwm2_isr(void);
__interrupt void epwm3_isr(void);
//__interrupt void sciaTxIsr(void);
__interrupt void sciaRxIsr(void);
void scia_init(void);

void error1(void);
void InitGpioForThreePhaseMode(void);
void InitEPwm2phInterleaved(void);
void UpdateDutyEPwm2phInterleaved(uint16_t duty);

typedef struct __array4{
    uint16_t data0;
    uint16_t data1;
    uint16_t data2;
    uint16_t data3;
} array4;

#define COMM_MAX_DATA_LENGTH 4U
#define COMM_START 0x55U


#define CPU_FREQ    60E6
#define LSPCLK_FREQ CPU_FREQ/4
#define SCI_FREQ    100E3
#define SCI_PRD     (LSPCLK_FREQ/(SCI_FREQ*8))-1

#define SCI_BUFFER_SIZE 16

// Maximum Duty values
#define CPU_SYS_CLOCK 60000
#define DT .5 //2us
#define PWM_SWITCHING_FREQUENCY 20 //kHz
#define PWM_PERIOD (CPU_SYS_CLOCK)/(PWM_SWITCHING_FREQUENCY)
#define ISR_CONTROL_FREQUENCY (PWM_SWITCHING_FREQUENCY)/(CNTRL_ISR_FREQ_RATIO)

#define EPWM_MAX_duty   0x03FF //max value 1023
#define EPWM_MIN_duty   0

// To keep track of which way the Dead Band is moving
#define DB_UP   1
#define DB_DOWN 0

// define PWM parameters
#define NPHASES 2
#define TBPRD_TBCLKs_INIT PWM_PERIOD/2    // Period = 900 TBCLK counts
#define DT_TBCLKs CPU_SYS_CLOCK*DT/1000/2       // dead time express in TBCLKs ticks
#define PHDLY_TBCLKs_INIT PWM_PERIOD/NPHASES    // Phase = 300/900 * 360 = 120 deg
#define DUTY_TBCLKs_INIT PWM_PERIOD/10     //duty=285/900=31.7%



// Global variables used in this example
Uint32  EPwm1TimerIntCount;
Uint32  EPwm2TimerIntCount;
Uint32  EPwm3TimerIntCount;
Uint16  EPwm1_DB_Direction;
Uint16  EPwm2_DB_Direction;
Uint16  EPwm3_DB_Direction;

uint16_t duty, incr_duty, phdly, pwm_period;


/* Serial driver data */
ringbuffer_t SCIATXBufferStruct;
ringbuffer_t SCIARXBufferStruct;

uint16_t SCIATXBuffer[SCI_BUFFER_SIZE];
uint16_t SCIARXBuffer[SCI_BUFFER_SIZE];


/* Serial frame state machine enum */
typedef enum __commFrameStates_e{
    state_idle, state_start, state_command,
    state_length, state_data, state_checksum, state_abort
}commFrameStates_e;


/*
 * Serial Frame typedef
 *
 * Byte 0: '0x55' Start Byte
 * Byte 3: 0x01 Command
 * Byte 1: 0x00 Data Length (<256)
 * Byte 4: 0x02 Data Byte 0
 * Byte 5: 0x03 Data Byte 1
 * Byte 6: 0x02 Data Byte 2
 * Byte 7: 0x03 Data Byte 3
 * ...
 * Byte x: Checksum (XOR of all bytes)
 */
typedef struct __commSerialFrame_t{
    uint16_t start;
    commSerialCommands_t command;
    uint16_t length;
    uint16_t data[4];
    uint16_t checksum;
    uint16_t checksumFlag;
    uint16_t transferFlag;
    uint16_t processedFlag;
}commSerialFrame_t;

/* Serial driver data */
commSerialFrame_t serialFrame;
commFrameStates_e rxstate;
commFrameStates_e txstate;


/* Serial driver functions */
int32_t application_processFrame(commSerialFrame_t *frame);
int32_t application_getFrame(ringbuffer_t *rbuf, commSerialFrame_t *frame);
int32_t application_putFrame(ringbuffer_t *rbuf, commSerialFrame_t *frame);

uint16_t foo2=0;

void main(void)
{
    uint16_t foo;

    serialFrame.start = 0U;
    serialFrame.command = echo;
    serialFrame.checksum = 0U;
    serialFrame.checksumFlag = 0U;
    serialFrame.transferFlag = 0U;
    serialFrame.processedFlag = 1U; /* prevent processing at startup */
    rxstate = state_idle;
    txstate = state_idle;

    pwm_period = TBPRD_TBCLKs_INIT;
    duty = DUTY_TBCLKs_INIT;
    phdly =PHDLY_TBCLKs_INIT;


    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    InitSysCtrl();

    // Step 2. Initalize GPIO:
    // Connect ePWM1, ePWM2, ePWM3 to GPIO pins, so that in not necessary toggle function in ISR
    InitGpioForThreePhaseMode();
    // Setup only the GPI/O only for SCI-A and SCI-B functionality
    // This function is found in DSP2803x_Sci.c
    InitSciaGpio();

    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags are cleared.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.SCIRXINTA = &sciaRxIsr;
    //PieVectTable.SCITXINTA = &sciaTxIsr;
    PieVectTable.EPWM1_INT = &epwm1_isr;
    PieVectTable.EPWM2_INT = &epwm2_isr;
    PieVectTable.EPWM3_INT = &epwm3_isr;
    PieVectTable.TINT0 = &cpu_timer0_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    // Step 4. Initialize all the Device Peripherals.
    InitCpuTimers();   // For this example, only initialize the Cpu Timers
    // Configure CPU-Timer 0 to interrupt every 500 milliseconds:
    ConfigCpuTimer(&CpuTimer0, 60, 500000);
    CpuTimer0Regs.TCR.all = 0x4001;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    InitEPwm2phInterleaved();

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    scia_init();  // Init SCI-A


    // Step 5. User specific code, enable interrupts

    // Enable CPU INT1 which is connected to CPU-Timer 0:
    IER |= M_INT1;
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    // Initialize counters

    EPwm1TimerIntCount = 0;
    EPwm2TimerIntCount = 0;
    EPwm3TimerIntCount = 0;
    // Enable CPU INT3 which is connected to EPWM1-3 INT:
    IER |= M_INT3;
    // Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;

    // Enable interrupts required for this example
    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER9.bit.INTx1=1;     // PIE Group 9, INT1
    PieCtrlRegs.PIEIER9.bit.INTx2=1;     // PIE Group 9, INT2
    IER = 0x100; // Enable CPU INT
    EINT;


    // Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    // Step 6. IDLE loop. Just sit and loop forever (optional):
    for(;;)
    {

        if (ringbuffer_length(&SCIARXBufferStruct) >= 8)
        {
            /* Process received data (if any) */
            application_getFrame(&SCIARXBufferStruct, &serialFrame);

            application_processFrame(&serialFrame);

            /* Process data for transmitting (if any) */
            application_putFrame(&SCIATXBufferStruct, &serialFrame);
        }

        /* Transmit data to SCI */
        if (!ringbuffer_empty(&SCIATXBufferStruct) && (SciaRegs.SCICTL2.bit.TXRDY==1))
        {
            //TODO: post process
            ringbuffer_get(&SCIATXBufferStruct, &foo);
            SciaRegs.SCITXBUF=foo; // Send data
        }

    };
}

// interrupt routine for led blinking
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Toggle GPIO34 once per 500 milliseconds
    if(incr_duty < duty)
    {
        incr_duty++;
        UpdateDutyEPwm2phInterleaved(incr_duty);
    }
    else
        if(incr_duty > duty)
        {
            incr_duty--;
            UpdateDutyEPwm2phInterleaved(incr_duty);
        }
    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


__interrupt void epwm1_isr(void)
{
    EPwm1TimerIntCount++;
    // Clear INT flag for this timer
    EPwm1Regs.ETCLR.bit.INT = 1;
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

__interrupt void epwm2_isr(void)
{
    EPwm2TimerIntCount++;
    // Clear INT flag for this timer
    EPwm2Regs.ETCLR.bit.INT = 1;
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}

__interrupt void epwm3_isr(void)
{
    EPwm3TimerIntCount++;
    // Clear INT flag for this timer
    EPwm3Regs.ETCLR.bit.INT = 1;
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

}

void UpdateDutyEPwm2phInterleaved(uint16_t duty)
{
    EPwm1Regs.CMPA.half.CMPA = duty;
    EPwm2Regs.CMPA.half.CMPA = duty;
    EPwm3Regs.CMPA.half.CMPA = duty;
}

void InitEPwm2phInterleaved(void)
{
    //=====================================================================
    // Config
    // Initialization Time
    //===========================================================================
    // EPWM Module 1 config
    EPwm1Regs.TBPRD = TBPRD_TBCLKs_INIT; // Period = 900 TBCLK counts
    EPwm1Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
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
    EPwm1Regs.DBFED = DT_TBCLKs; // FED = 20 TBCLKs
    EPwm1Regs.DBRED = DT_TBCLKs; // RED = 20 TBCLKs

    // EPWM Module 2 config
    EPwm2Regs.TBPRD = TBPRD_TBCLKs_INIT; // Period = 900 TBCLK counts
    EPwm2Regs.TBPHS.half.TBPHS = PHDLY_TBCLKs_INIT; // Phase = 300/900 * 360 = 120 deg
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
    EPwm2Regs.DBFED = DT_TBCLKs; // FED = 20 TBCLKs
    EPwm2Regs.DBRED = DT_TBCLKs; // RED = 20 TBCLKs

    // Run Time (Note: Example execution of one run-time instant)
    //===========================================================
    EPwm1Regs.CMPA.half.CMPA = DUTY_TBCLKs_INIT; // adjust duty for output EPWM1A
    EPwm2Regs.CMPA.half.CMPA = DUTY_TBCLKs_INIT; // adjust duty for output EPWM2A
}


void error1(void)
{
    __asm("     ESTOP0"); // Test failed!! Stop!
    for (;;);
}

//__interrupt void sciaTxFifoIsr(void)
//{
//    Uint16 i;
//    for(i=0; i< 2; i++)
//    {
//        SciaRegs.SCITXBUF=sdataA[i];     // Send data
//    }
//
//    for(i=0; i< 2; i++)                 //Increment send data for next cycle
//    {
//        sdataA[i] = (sdataA[i]+1) & 0x00FF;
//    }
//
//    SciaRegs.SCIFFTX.bit.TXFFINTCLR=1;  // Clear SCI Interrupt flag
//    PieCtrlRegs.PIEACK.all|=0x100;      // Issue PIE ACK
//}

__interrupt void sciaRxIsr(void)
{
    //    Uint16 i;
    //    for(i=0;i<2;i++)
    //    {
    //        rdataA[i]=SciaRegs.SCIRXBUF.all;  // Read data
    //    }
    //    for(i=0;i<2;i++)                     // Check received data
    //    {
    //        if(rdataA[i] != ( (rdata_pointA+i) & 0x00FF) ) error();
    //    }
    //    rdata_pointA = (rdata_pointA+1) & 0x00FF;

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

void scia_init()
{
    extern ringbuffer_t SCIATXBufferStruct;
    extern ringbuffer_t SCIARXBufferStruct;

    extern uint16_t SCIATXBuffer[SCI_BUFFER_SIZE];
    extern uint16_t SCIARXBuffer[SCI_BUFFER_SIZE];

    /* Initialize ring buffer */
    SCIATXBufferStruct.buffer = SCIATXBuffer;
    SCIARXBufferStruct.buffer = SCIARXBuffer;

    SCIATXBufferStruct.size = SCI_BUFFER_SIZE;
    SCIARXBufferStruct.size = SCI_BUFFER_SIZE;

    ringbuffer_reset(&SCIATXBufferStruct);
    ringbuffer_reset(&SCIARXBufferStruct);

    SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback  No parity,8 char bits, async mode, idle-line protocol
    SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
    // Disable RX ERR, SLEEP, TXWAKE
    //SciaRegs.SCICTL2.bit.TXINTENA =0;
    SciaRegs.SCICTL2.bit.RXBKINTENA =1;
    SciaRegs.SCIHBAUD = 0x0000;
    SciaRegs.SCILBAUD = 0x00C2; //9600;
    SciaRegs.SCICCR.bit.LOOPBKENA =0; // Enable loop back
    //SciaRegs.SCIFFTX.all=0xC022;
    //SciaRegs.SCIFFRX.all=0x0022;
    //SciaRegs.SCIFFCT.all=0x00;

    SciaRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset
    //SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
    //SciaRegs.SCIFFRX.bit.RXFIFORESET=1;
}

void InitGpioForThreePhaseMode(void)
{
    EALLOW;
    //--------------------------------------------------------------------------------------
    // GPIO (GENERAL PURPOSE I/O) CONFIG
    //--------------------------------------------------------------------------------------
    //-----------------------
    // QUICK NOTES on USAGE:
    //-----------------------
    // If GpioCtrlRegs.GP?MUX?bit.GPIO?= 1, 2 or 3 (i.e. Non GPIO func), then leave
    //  rest of lines commented
    // If GpioCtrlRegs.GP?MUX?bit.GPIO?= 0 (i.e. GPIO func), then:
    //  1) uncomment GpioCtrlRegs.GP?DIR.bit.GPIO? = ? and choose pin to be IN or OUT
    //  2) If IN, can leave next to lines commented
    //  3) If OUT, uncomment line with ..GPACLEAR.. to force pin LOW or
    //             uncomment line with ..GPASET.. to force pin HIGH or
    //--------------------------------------------------------------------------------------
    //--------------------------------------------------------------------------------------
    //  GPIO-00 - PIN FUNCTION = PWM1A
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;     // 0=GPIO,  1=EPWM1A,  2=Resv,  3=Resv
    //  GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;      // 1=OUTput,  0=INput
    //  GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;    // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO0 = 1;      // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    //  GPIO-01 - PIN FUNCTION = PWM1B
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;     // 0=GPIO,  1=EPWM1B,  2=Resv,  3=COMP1OUT
    //  GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;      // 1=OUTput,  0=INput
    //  GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;    // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO1 = 1;      // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    //  GPIO-02 - PIN FUNCTION = PWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;     // 0=GPIO,  1=EPWM2A,  2=Resv,  3=Resv
    //  GpioCtrlRegs.GPADIR.bit.GPIO2 = 0;      // 1=OUTput,  0=INput
    //  GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;    // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO2 = 1;      // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    //  GPIO-03 - PIN FUNCTION = PWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;     // 0=GPIO,  1=EPWM2B,  2=SPISOMI-A,  3=COMP2OUT
    //  GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;      // 1=OUTput,  0=INput
    //  GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;    // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO3 = 1;      // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    //  GPIO-04 - PIN FUNCTION = PWM3A
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;     // 0=GPIO,  1=EPWM3A,  2=Resv,  3=Resv
    //  GpioCtrlRegs.GPADIR.bit.GPIO4 = 0;      // 1=OUTput,  0=INput
    //  GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;    // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO4 = 1;      // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    //  GPIO-05 - PIN FUNCTION = PWM3B
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;     // 0=GPIO,  1=EPWM3B,  2=SPISIMO-A,  3=ECAP1
    //  GpioCtrlRegs.GPADIR.bit.GPIO5 = 0;      // 1=OUTput,  0=INput
    //  GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;    // uncomment if --> Set Low initially
    //  GpioDataRegs.GPASET.bit.GPIO5 = 1;      // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    //  GPIO-11 - PIN FUNCTION = gate driver reset disable
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;    // 0=GPIO RST,  1=EPWM6B,  2=LINRX-A,  3=Resv
    GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;     // 1=OUTput,  0=INput
    //  GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;   // uncomment if --> Set Low initially
    GpioDataRegs.GPASET.bit.GPIO11 = 1;     // uncomment if --> Set High initially
    //--------------------------------------------------------------------------------------
    EDIS;   // Disable register access
}


/*
 * Frame processing
 *
 * Each frame has to be processede only once
 */
int32_t application_processFrame(commSerialFrame_t *frame)
{

    array4 temp_4;
    uint16_t data_offset = 0;
    uint16_t command_offset = 0;
    commSerialCommands_t command;
    uint16_t * data_address_p;

    /*
     * Process a single command or loop for multi commands
     */

    /* reset variables for multi command */
    //    data_offset = 0U;
    //    command_offset = 0U;

    while(frame->processedFlag == 0U)
    {

        if (frame->checksumFlag == 1U)
        {
            /* Normal operation */
            if (frame->length == 4U)
            {
                data_offset = 0;
                command_offset = 0;
                command = frame->command;
            }
            else
            {
                command = errorLength;
            }
        }
        else
        {
            command = errorChecksum;
        }
        /* Set data address pointer to point to first address location */
        data_address_p = (frame->data + data_offset);

        /* Execute command */
        switch(command)
        {
        case echo:
            /* send back same frame */
            break;
            /*
             ******************************************************************
             * General commands
             ******************************************************************
             */
        case setFrequency:
            //TODO: EPwm1Regs.TBPRD

            EPwm1Regs.TBPRD = (CPU_SYS_CLOCK)/(data_address_p[0])/2;
            EPwm2Regs.TBPRD = (CPU_SYS_CLOCK)/(data_address_p[0])/2;
            EPwm1Regs.CMPA.half.CMPA = duty*(CPU_SYS_CLOCK)/(data_address_p[0])/2/pwm_period; // adjust duty for output EPWM1A
            EPwm2Regs.CMPA.half.CMPA = duty*(CPU_SYS_CLOCK)/(data_address_p[0])/2/pwm_period; // adjust duty for output EPWM2A
            EPwm2Regs.TBPHS.half.TBPHS = (CPU_SYS_CLOCK)/(data_address_p[0])/NPHASES;
            foo2 = 1;
            break;

        case setDuty:
            duty = pwm_period/(data_address_p[0]);
            EPwm1Regs.CMPA.half.CMPA = duty;
            EPwm2Regs.CMPA.half.CMPA = duty;
            break;

            /*
             ******************************************************************
             * ERROR commands
             ******************************************************************
             */
        case errorLength:
            frame->command = errorLength;
            frame->length = 4;
            temp_4.data0 = 0U;
            temp_4.data1 = 0U;
            temp_4.data2 = 0U;
            temp_4.data3 = 0U;
            memcpy(frame->data, &temp_4, 4U);
            break;
        case errorChecksum:
            frame->command = errorChecksum;
            frame->length = 4;
            temp_4.data0 = 0U;
            temp_4.data1 = 0U;
            temp_4.data2 = 0U;
            temp_4.data3 = 0U;
            memcpy(frame->data, &temp_4, 4U);
            break;
        default:
            frame->command = 1;
            frame->length = 4;
            temp_4.data0 = 0U;
            temp_4.data1 = 0U;
            temp_4.data2 = 0U;
            temp_4.data3 = 0U;
            memcpy(frame->data, &temp_4, 4U);
            break;
        } /* end switch */


        frame->start = COMM_START;
        frame->transferFlag = 1U;
        frame->processedFlag = 1U;
        command_offset = 0U;
        data_offset = 0U;
        break; /* break while */
    } /* end while */
    return 0;
}

int32_t application_getFrame(ringbuffer_t *rbuf, commSerialFrame_t *frame)
{
    extern commFrameStates_e rxstate;
    extern commFrameStates_e txstate;


    static int16_t ret = 1;
    static uint16_t temp, idx, checksum;

    /* While there is data in the ringbuffer, populate frame. */
    while(!ringbuffer_empty(rbuf))
    {
        switch(rxstate)
        {
        case state_idle:
            /* get tail data of RX buffer */
            ringbuffer_data(rbuf, &temp, rbuf->tail);
            /* check if it is a start byte */
            if (temp == COMM_START)
            {
                rxstate = state_start;
                frame->checksumFlag = 0U;
                frame->transferFlag = 0U;
                frame->processedFlag = 0U;
                checksum = 0U;
                idx = 0;
            }
            else
            {
                /* error, no valid frame, throw data away*/
                ringbuffer_get(rbuf, &temp);
            }
            break;
        case state_start:
            /* get start byte */
            ringbuffer_get(rbuf, &temp);
            frame->start = temp;
            checksum ^= temp;
            rxstate = state_command;
            break;
        case state_command:
            /* get command byte */
            ringbuffer_get(rbuf, &temp);
            frame->command = (commSerialCommands_t)temp;
            checksum ^= temp;
            rxstate = state_length;
            break;
        case state_length:
            /* get length byte */
            ringbuffer_get(rbuf, &temp);
            frame->length = temp;
            checksum ^= temp;
            rxstate = state_data;
            if (frame->length == 0U)
            {
                rxstate = state_checksum;
            }
            if (frame->length > COMM_MAX_DATA_LENGTH)
            {
                /* Error! Data too long for buffer */
                frame->command = errorLength;
                rxstate = state_abort;
            }
            break;
        case state_data:
            while (idx<frame->length)
            {
                if (ringbuffer_empty(rbuf)) break;
                ringbuffer_get(rbuf, &temp);
                frame->data[idx] = temp;
                checksum ^= temp;
                idx++;
            }
            /* leave state only when all data was received */
            if(idx >= (frame->length))
            {
                rxstate = state_checksum;
            }
            break;
        case state_checksum:
            /* get checksum byte */
            ringbuffer_get(rbuf, &temp);
            frame->checksum = temp;
            if (checksum == frame->checksum)
            {
                frame->checksumFlag = 1U;
                ret = 0;
            }
            else
            {
                frame->checksumFlag = 0U;
                ret = 1;
            }
            rxstate = state_idle;
            break;
        case state_abort:
            while (!ringbuffer_empty(rbuf))
            {
                ringbuffer_get(rbuf, &temp);
                /* stop at frame start */
                ringbuffer_data(rbuf, &temp, rbuf->tail);
                if (temp == COMM_START) break;
            }
            rxstate = state_idle;
            break;
        }

        /* Break while loop */
        if (rxstate == state_idle)
        {
            break;
        }
    }

    return ret;
}


/*
 * Decode complete frame into ringbuffer.
 */
int32_t application_putFrame(ringbuffer_t *rbuf, commSerialFrame_t *frame)
{

    extern commFrameStates_e rxstate;
    extern commFrameStates_e txstate;

    int32_t ret = 1;
    uint16_t temp, idx, checksum;

    if(rbuf && frame)
    {

        /* While there is data in the ringbuffer, populate frame. */
        while(!ringbuffer_full(rbuf))
        {
            switch(txstate)
            {
            case state_idle:
                /* check if frame is ready to transfer */
                if ((frame->transferFlag == 1U))
                {
                    txstate = state_start;
                    frame->transferFlag = 0U;
                    checksum = 0U;
                    idx = 0U;
                }
                break;
            case state_start:
                /* put start byte */
                temp = frame->start;
                checksum ^= temp;
                ringbuffer_put(rbuf, temp);
                txstate = state_command;
                break;
            case state_command:
                /* put command byte */
                temp = (uint16_t)frame->command;
                checksum ^= temp;
                ringbuffer_put(rbuf, temp);
                txstate = state_length;
                break;
            case state_length:
                /* put length byte */
                temp = frame->length;
                checksum ^= temp;
                ringbuffer_put(rbuf, temp);
                txstate = state_data;
                if (frame->length == 0U)
                {
                    txstate = state_checksum;
                }
                break;
            case state_data:
                while(idx < frame->length)
                {
                    if (ringbuffer_full(rbuf)) break;
                    temp = frame->data[idx];
                    checksum ^= temp;
                    ringbuffer_put(rbuf, temp);
                    idx++;
                }
                if(idx >= (frame->length))
                {
                    txstate = state_checksum;
                }
                break;
            case state_checksum:
                /* put checksum byte */
                temp = checksum;
                ringbuffer_put(rbuf, temp);
                txstate = state_idle;
                break;
            default:
                txstate = state_idle;
                break;
            }

            /* Break while loop */
            if (txstate == state_idle)
            {
                break;
            }
        }
        ret = 0;
    }
    else /* Null pointer passed */
    {
        ret = 1;
    }


    return ret;
}

//===========================================================================
// No more.
//===========================================================================
