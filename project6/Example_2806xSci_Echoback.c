//###########################################################################
//
//!  \addtogroup f2806x_example_list
//!  <h1>SCI Echo Back(sci_echoback)</h1>
//!
//!  This test receives and echo-backs data through the SCI-A port.
//!
//!  The PC application 'hypterterminal' can be used to view the data
//!  from the SCI and to send information to the SCI.  Characters received
//!  by the SCI port are sent back to the host.
//!
//!  \b Running \b the \b Application
//!  -# Configure hyperterminal:
//!  Use the included hyperterminal configuration file SCI_96.ht.
//!  To load this configuration in hyperterminal
//!    -# Open hyperterminal
//!    -# Go to file->open
//!    -# Browse to the location of the project and
//!       select the SCI_96.ht file.
//!  -# Check the COM port.
//!  The configuration file is currently setup for COM1.
//!  If this is not correct, disconnect (Call->Disconnect)
//!  Open the File-Properties dialog and select the correct COM port.
//!  -# Connect hyperterminal Call->Call
//!  and then start the 2806x SCI echoback program execution.
//!  -# The program will print out a greeting and then ask you to
//!  enter a character which it will echo back to hyperterminal.
//!
//!  \note If you are unable to open the .ht file, you can create
//!  a new one with the following settings
//!  -  Find correct COM port
//!  -  Bits per second = 9600
//!  -  Date Bits = 8
//!  -  Parity = None
//!  -  Stop Bits = 1
//!  -  Hardware Control = None
//!
//!  \b Watch \b Variables \n
//!  - \b LoopCount, for the number of characters sent
//!  - ErrorCount
//!
//! \b External \b Connections \n
//!  Connect the SCI-A port to a PC via a transceiver and cable.
//!  - GPIO28 is SCI_A-RXD (Connect to Pin3, PC-TX, of serial DB9 cable)
//!  - GPIO29 is SCI_A-TXD (Connect to Pin2, PC-RX, of serial DB9 cable)
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V151 $
// $Release Date: February  2, 2016 $
// $Copyright: Copyright (C) 2011-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "string.h"
#include "stdio.h"
// Prototype statements for functions found within this file.
__interrupt void cpu_timer0_isr(void);
void scia_echoback_init(void);
void scia_fifo_init(void);
void scia_xmit(int a);
//void scia_msg(char *msg, int x);
__interrupt void adc_isr(void);
void Adc_Config(void);

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;
// Global counts used in this example
Uint16 LoopCount;
Uint16 ErrorCount;
Uint16 ConversionCount;
//Uint16 Voltage1[10];
//Uint16 Voltage2[10];
Uint16 Voltage[16];

Uint16 prev_adc_value[16];

void main(void)
{

//    Uint16 ReceivedChar;
//    char *msg;

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2806x_SysCtrl.c file.
   InitSysCtrl();

//       EALLOW;
//       GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;  // Enable pull-up on GPIO0 (button)
//       GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0; // GPIO0 is GPIO
//       GpioCtrlRegs.GPADIR.bit.GPIO0 = 0;  // GPIO0 is an input
//       EDIS;
   EALLOW;
   GpioCtrlRegs.GPAPUD.all = 0;
   GpioCtrlRegs.GPBPUD.all = 0;

   GpioCtrlRegs.GPAMUX1.all = 0x00000000;  // All GPIO
   GpioCtrlRegs.GPAMUX2.all = 0x00000000;  // All GPIO
   GpioCtrlRegs.GPBMUX1.all = 0x00000000;  // All GPIO
   GpioCtrlRegs.GPBMUX2.all = 0x00000000;  // All GPIO

   GpioCtrlRegs.GPADIR.all = 0x00000000;   // All inputs
   GpioCtrlRegs.GPBDIR.all = 0x00000000;   // All inputs

   EDIS;
// Step 2. Initalize GPIO:
// This example function is found in the F2806x_Gpio.c file and
// illustrates how to set the GPIO to its default state.
   // InitGpio(); Skipped for this example

// For this example, only init the pins for the SCI-A port.
// This function is found in the F2806x_Sci.c file.
   InitSciaGpio();
   InitSysCtrl();

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2806x_PieCtrl.c file.
   InitPieCtrl();
   memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32)&RamfuncsLoadSize);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
   InitFlash();
// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2806x_DefaultIsr.c.
// This function is found in F2806x_PieVect.c.
   InitPieVectTable();
   EALLOW;  // This is needed to write to EALLOW protected register
   PieVectTable.ADCINT1 = &adc_isr;
   EDIS;    // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in F2806x_InitPeripherals.c
// InitPeripherals(); // Not required for this example

// Step 5. User specific code:
   InitAdc();  // For this example, init the ADC
   AdcOffsetSelfCal();

// Step 5. User specific code, enable interrupts:

// Enable ADCINT1 in PIE
   PieCtrlRegs.PIEIER1.bit.INTx1 = 1;   // Enable INT 1.1 in the PIE
   IER |= M_INT1;                       // Enable CPU Interrupt 1
   EINT;                                // Enable Global interrupt INTM
   ERTM;                                // Enable Global realtime interrupt DBGM

   LoopCount = 0;
   ConversionCount = 0;

// Configure ADC
    EALLOW;
    AdcRegs.ADCCTL2.bit.ADCNONOVERLAP = 1;  // Enable non-overlap mode
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;    // ADCINT1 trips after AdcResults latch
    AdcRegs.INTSEL1N2.bit.INT1E     = 1;    // Enabled ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;    // Disable ADCINT1 Continuous mode
    AdcRegs.INTSEL1N2.bit.INT1SEL   = 1;    // setup EOC1 to trigger ADCINT1 to fire
    AdcRegs.ADCSOC0CTL.bit.CHSEL    = 0;    // set SOC0 channel select to ADCINA4
    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 1;    // set SOC1 channel select to ADCINA2
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 5;    // set SOC0 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 5;    // set SOC1 start trigger on EPWM1A, due to round-robin SOC0 converts first then SOC1
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 6;    // set SOC0 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 6;    // set SOC1 S/H Window to 7 ADC Clock Cycles, (6 ACQPS plus 1)

    AdcRegs.ADCSOC2CTL.bit.CHSEL = 2;
    AdcRegs.ADCSOC2CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 5;

    AdcRegs.ADCSOC3CTL.bit.CHSEL = 3;
    AdcRegs.ADCSOC3CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 5;

    AdcRegs.ADCSOC4CTL.bit.CHSEL = 4;
    AdcRegs.ADCSOC4CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC4CTL.bit.TRIGSEL = 5;

    AdcRegs.ADCSOC5CTL.bit.CHSEL = 5;
    AdcRegs.ADCSOC5CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC5CTL.bit.TRIGSEL = 5;

    AdcRegs.ADCSOC6CTL.bit.CHSEL = 6;
    AdcRegs.ADCSOC6CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC6CTL.bit.TRIGSEL = 5;

    AdcRegs.ADCSOC7CTL.bit.CHSEL = 7;
    AdcRegs.ADCSOC7CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC7CTL.bit.TRIGSEL = 5;

    AdcRegs.ADCSOC8CTL.bit.CHSEL = 8;
    AdcRegs.ADCSOC8CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC8CTL.bit.TRIGSEL = 5;

    AdcRegs.ADCSOC9CTL.bit.CHSEL = 9;
    AdcRegs.ADCSOC9CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC9CTL.bit.TRIGSEL = 5;

    AdcRegs.ADCSOC10CTL.bit.CHSEL = 10;
    AdcRegs.ADCSOC10CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC10CTL.bit.TRIGSEL = 5;

    AdcRegs.ADCSOC11CTL.bit.CHSEL = 11;
    AdcRegs.ADCSOC11CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC11CTL.bit.TRIGSEL = 5;

    AdcRegs.ADCSOC12CTL.bit.CHSEL = 12;
    AdcRegs.ADCSOC12CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC12CTL.bit.TRIGSEL = 5;

    AdcRegs.ADCSOC13CTL.bit.CHSEL = 13;
    AdcRegs.ADCSOC13CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC13CTL.bit.TRIGSEL = 5;

    AdcRegs.ADCSOC14CTL.bit.CHSEL = 14;
    AdcRegs.ADCSOC14CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC14CTL.bit.TRIGSEL = 5;

    AdcRegs.ADCSOC15CTL.bit.CHSEL = 15;
    AdcRegs.ADCSOC15CTL.bit.ACQPS = 6;
    AdcRegs.ADCSOC15CTL.bit.TRIGSEL = 5;

    EDIS;

// Assumes ePWM1 clock is already enabled in InitSysCtrl();
   EPwm1Regs.ETSEL.bit.SOCAEN   = 1;        // Enable SOC on A group
   EPwm1Regs.ETSEL.bit.SOCASEL  = 4;        // Select SOC from CMPA on upcount
   EPwm1Regs.ETPS.bit.SOCAPRD   = 1;        // Generate pulse on 1st event
   EPwm1Regs.CMPA.half.CMPA     = 0x0080;   // Set compare A value
   EPwm1Regs.TBPRD              = 0xFFFF;   // Set period for ePWM1
   EPwm1Regs.TBCTL.bit.CTRMODE  = 0;        // count up and start

    LoopCount = 0;
    ErrorCount = 0;

    scia_fifo_init();	   // Initialize the SCI FIFO
    scia_echoback_init();  // Initalize SCI for echoback

	for(;;)
    {
       //msg = "\r\nEnter a character: \0";
       //scia_msg(msg);

       // Wait for inc character
       //while(SciaRegs.SCIFFRX.bit.RXFFST !=1) { } // wait for XRDY =1 for empty state

       // Get character
       //ReceivedChar = SciaRegs.SCIRXBUF.all;

       // Echo character back
       //msg = "  You sent: \0";
       //scia_msg(msg);
       //scia_xmit(ReceivedChar);

       LoopCount++;
    }

}

__interrupt void cpu_timer0_isr(void)
{
   CpuTimer0.InterruptCount++;

   // Acknowledge this interrupt to receive more interrupts from group 1
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// Test 1,SCIA  DLB, 8-bit word, baud rate 0x0103, default, 1 STOP bit, no parity
void scia_echoback_init()
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

 	SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE

	SciaRegs.SCICTL2.bit.TXINTENA =1;
	SciaRegs.SCICTL2.bit.RXBKINTENA =1;

	SciaRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK).
    SciaRegs.SCILBAUD    =0x0024;

	SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

// Transmit a character from the SCI
void scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF=a;

}

//void scia_msg(char * msg, int x)
//{
//    int i;
//    i = 0;
//    while(msg[i] != '\0')
//    {
//        scia_xmit(msg[i]);
//        i++;
//    }
//
//    char str[4];
//    i = 0;
//
//    while(i !=4){
//        //scia_xmit((x%10)+48);
//        str[i] = 48+(x%10);
//        x = x/10;
//        i++;
//    }
//
//    i = 3;
//
//    while(str[i] != '\0')
//    {
//        scia_xmit(str[i]);
//        i--;
//    }
//    scia_xmit('\n');
//
//}

// Initalize the SCI FIFO
void scia_fifo_init()
{
    SciaRegs.SCIFFTX.all=0xE040;
    SciaRegs.SCIFFRX.all=0x2044;
    SciaRegs.SCIFFCT.all=0x0;

}

double filter_adc(Uint16 new_value, Uint16 prev_value)
{
    return 0.1 * new_value + (1 - 0.1) * prev_value;
}

void scia_msg(Uint16 value[])
{
    // Function to send message over SCI
    char buffer[200];
    int i = 0;
    sprintf(buffer, "%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %lu %lu\n",
            value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7],
            value[8], value[9], value[10], value[11], value[12], value[13], value[14], value[15],
            GpioDataRegs.GPADAT.all, GpioDataRegs.GPBDAT.all);
//    sprintf(buffer, "%u %u %lu %lu", value[0], value[1], GpioDataRegs.GPADAT.all, GpioDataRegs.GPBDAT.all);
    while (buffer[i] != '\0')
    {
        scia_xmit(buffer[i]);
        i++;
    }
}

__interrupt void  adc_isr(void)
{

    Voltage[0]  = AdcResult.ADCRESULT0;  // Result from ADCSOC0
    Voltage[1]  = AdcResult.ADCRESULT1;  // Result from ADCSOC1
    Voltage[2]  = AdcResult.ADCRESULT2;  // Result from ADCSOC2
    Voltage[3]  = AdcResult.ADCRESULT3;  // Result from ADCSOC3
    Voltage[4]  = AdcResult.ADCRESULT4;  // Result from ADCSOC4
    Voltage[5]  = AdcResult.ADCRESULT5;  // Result from ADCSOC5
    Voltage[6]  = AdcResult.ADCRESULT6;  // Result from ADCSOC6
    Voltage[7]  = AdcResult.ADCRESULT7;  // Result from ADCSOC7
    Voltage[8]  = AdcResult.ADCRESULT8;  // Result from ADCSOC8
    Voltage[9]  = AdcResult.ADCRESULT9;  // Result from ADCSOC9
    Voltage[10] = AdcResult.ADCRESULT10; // Result from ADCSOC10
    Voltage[11] = AdcResult.ADCRESULT11; // Result from ADCSOC11
    Voltage[12] = AdcResult.ADCRESULT12; // Result from ADCSOC12
    Voltage[13] = AdcResult.ADCRESULT13; // Result from ADCSOC13
    Voltage[14] = AdcResult.ADCRESULT14; // Result from ADCSOC14
    Voltage[15] = AdcResult.ADCRESULT15; // Result from ADCSOC15
//    int i;
//    for (i = 0; i < 16; i++) {
//        Voltage[i] = (*(&AdcResult.ADCRESULT0 + i));
//    }

//    Uint16 filtered_value[16];
//
//    filtered_value[0]  = filter_adc(Voltage[0], prev_adc_value[0]);
//    prev_adc_value[0] = filtered_value[0];
//
//    filtered_value[1]  = filter_adc(Voltage[1], prev_adc_value[1]);
//    prev_adc_value[1] = filtered_value[1];
//
//    filtered_value[2]  = filter_adc(Voltage[2], prev_adc_value[2]);
//    prev_adc_value[2] = filtered_value[2];
//
//    filtered_value[3]  = filter_adc(Voltage[3], prev_adc_value[3]);
//    prev_adc_value[3] = filtered_value[3];
//
//    filtered_value[4]  = filter_adc(Voltage[4], prev_adc_value[4]);
//    prev_adc_value[4] = filtered_value[4];
//
//    filtered_value[5]  = filter_adc(Voltage[5], prev_adc_value[5]);
//    prev_adc_value[5] = filtered_value[5];
//
//    filtered_value[6]  = filter_adc(Voltage[6], prev_adc_value[6]);
//    prev_adc_value[6] = filtered_value[6];
//
//    filtered_value[7]  = filter_adc(Voltage[7], prev_adc_value[7]);
//    prev_adc_value[7] = filtered_value[7];
//
//    filtered_value[8]  = filter_adc(Voltage[8], prev_adc_value[8]);
//    prev_adc_value[8] = filtered_value[8];
//
//    filtered_value[9]  = filter_adc(Voltage[9], prev_adc_value[9]);
//    prev_adc_value[9] = filtered_value[9];
//
//    filtered_value[10] = filter_adc(Voltage[10], prev_adc_value[10]);
//    prev_adc_value[10] = filtered_value[10];
//
//    filtered_value[11] = filter_adc(Voltage[11], prev_adc_value[11]);
//    prev_adc_value[11] = filtered_value[11];
//
//    filtered_value[12] = filter_adc(Voltage[12], prev_adc_value[12]);
//    prev_adc_value[12] = filtered_value[12];
//
//    filtered_value[13] = filter_adc(Voltage[13], prev_adc_value[13]);
//    prev_adc_value[13] = filtered_value[13];
//
//    filtered_value[14] = filter_adc(Voltage[14], prev_adc_value[14]);
//    prev_adc_value[14] = filtered_value[14];
//
//    filtered_value[15] = filter_adc(Voltage[15], prev_adc_value[15]);
//    prev_adc_value[15] = filtered_value[15];


//  int number = Voltage2[ConversionCount];
//  char str[20];
//  char str2[80];
//
//  sprintf(str, "%d", number);
//  strcat(str2, "hi ");
//  strcat(str2, str);
//  strcat(str2, " done\n\0");

  if((LoopCount % 10000) == 0)
  {
      scia_msg(Voltage);
  }
 // printf("ok %d\n", Voltage2[ConversionCount]);

//  if(Voltage2[ConversionCount] > 3000){
//      GpioDataRegs.GPATOGGLE.bit.GPIO0 = 1;
//  }

  // If 20 conversions have been logged, start over
//  if(ConversionCount == 9)
//  {
//     ConversionCount = 0;
//  }
//  else ConversionCount++;

  AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;     //Clear ADCINT1 flag reinitialize for next SOC
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE

  return;
}



//===========================================================================
// No more.
//===========================================================================

