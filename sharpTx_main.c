/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                             //
//                                              SharpTxII.c  ver1.0                                            //
//                                                                                                             //
//                                      Author: Mark Patten, Senior Engineer,                                    //
//                                                                                                             //
//                              C Firmware code for the 001298xXXX PCB Rev  wide voltage range                 //
//                                                                                                             //
//                                        Written for the PICC Compiler                                        //
//             (CCS Information Systems, www.ccsinfo.com, Brookfield, WI 53008, Tel. (262) 522-6500)           //
//                                                                                                             //
//                                     Program commencement date: Jan 2010                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/* MODIFICATIONS
  A2100503w    8/03/17   Control Vtx regulators on Pin 20 (PIN_B2)
  A2100503u    7/06/17   Comment out adjust pot routine (causing bias to decrement each on/off cycle), set AD source by MCLR input (hi=ext, low=mic), set B7 (PGD) hi-Z
  A2100503t    3/24/17   Put AD in play mode if recorder playing (i.e. after turn-off and on), reduce cut-off voltage to 2.9V (from 3.1 V), and extend receiver look time to 4.5 seconds (to help stop double received commands)
  A2100503s    3/03/17   Block repeated receiver commands, stop RS-232 pinging when transmitter is off, set audio mode to mic when status is not playing
  A2100503r   11/10/16   Added RS-232 receive capabilityalso playback sets AD to   
  A2100503q   10/06/16   Added volume message to recorder before every playback, also playback sets AD to AUX mode (stop sets it back to MIC)
  A2100503p3   9/30/16   Added Flyron playback
  A2100503p2   9/29/16   Added ability to select source for stealth record Flyron
*/
                                                                                                             
//GENERAL NOTES
//
//The software design is a state flow with two major states RX and TX.  Both the RX and TX states have
//sub-states.  RX has six sub-states and TX has seven.  The Main_Mode_Type struct defines the byte that 
//indicates the state of the SharpTx at any given instant.
//
//When the TX is not active the state remains RX.  When the TX is active the state alternates between
//the RX and TX states.  Only one sub-state is executed each time a TX or RX state is executed.  This  
//allows a controlled processor bandwidth sharing between the RX and TX states.
//
//In addtion to processing the RX and TX states, the processor processes interrupts for the subchannel
//and the timer update.
//
//
///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                                  COMPILER INSTRUCTIONS                                    //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
// select either 18LF26K22 or 18F2520
#define USING_PIC18LF26K22 0

#include "testModes.h"

#include <SharpTxII.h>
#include <SharpTxII_Config_Options.h>
//#include <SharpTxII_IDs.h>
//#include <stdlib.h>
#include "SharpTx.h"
#include "AKM5356.h"
#include <configuration_options.h>
#include "freq_data_hi.h"
#include "freq_data_lo.h"
#include "DS1845.h"
#include "buttonAdjustModPwr.h"
#include "recorderControl.c"

//#include "VtxAdjust.h"


#use standard_io(A)  // with 'standard_io' compiler preceded any pin state change with correct TRIS
#use standard_io(B)
//#use standard_io(C)
//#use standard_io(D)
//#use standard_io(E)

#if CONFIG_UART_IN_HARDWARE==1
//   #use rs232(baud=9600,xmit=PIN_C6)
#else
   #use rs232(baud=9600,xmit=PIN_B7,parity=N,bits=8)
#endif

// Definitions for direct access to PIC registers

#byte TMR1L=0x0FCE

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                                  LOOK-UP TABLE(S)                                         //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
signed int CONST nDecode6Bto4B[64] = {   // A look up table of # 1's in an address value

/*0 */   -1, -1, -1, -1, -1, -1, -1, -1,  // any negative value indicates an illegal value
/*8*/    -1, -1, -1, -1, -1,  0,  1, -1,
/*16*/   -1, -1, -1,  2, -1,  3,  4, -1,
/*24*/   -1,  5,  6, -1,  7, -1, -1, -1,
/*32*/   -1, -1, -1,  8, -1,  9, 10, -1,
/*40*/   -1, 11, 12, -1, 13, -1, -1, -1,
/*48*/   -1, -1, 14, -1, 15, -1, -1, -1,
/*56*/   -1, -1, -1, -1, -1, -1, -1, -1
};
///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                                  FUNCTION PROTOTYPES                                      //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
char go_to_sleep(void);
char sig_chk(void);
char Rcv(void);
char Processf(void);
char Actionf(void);
char Schedulef(void);
char Tx_OfftoOn_Vtxf(void);
char Tx_OfftoOn_InitTxf(void);
char Tx_On_Chk_Adj_PSf(void);
char Tx_On_MeasureBatVf(void);
char Tx_On_Changef(void);
char Tx_OntoOffF(void);
char Tx_Pot_AdjF(void);
void TEST_PULSE(long length,N);
void delay_usec( signed int n);
void delay_msec(int n);
void determine_channel();
void determine_stereo();
//void change_stereo_mono();
void send_byte(char b);
void Change_Frequency();
void Send_Synth_Msg();
//void set_config_info();
void Turn_REMOTE_CTL_ON_OFF();

void send_R(void);
void send_N(void);
void set_FOLD_HIGH(void);
void set_FOLD_LOW(void);

void Send_AD_Msg(int nReg_Addr, nAD_Data);
  BOOLEAN Send_Data(unsigned int nWhat_to_Send, nPot_Address, nValue, nTiming_Delay);
BOOLEAN set_pot(unsigned int nValue, nPot_Number);
void set_data_speed_and_polarity();

void update_elapsed_time_counters();
BOOLEAN initial_RF_activity_search();
void wait_8_32KHz_cycles();
#inline void hi_speed_data_log();
void clear_squelch_counters();
BOOLEAN verify_customer_ID();
BOOLEAN verify_receiver_ID();
BOOLEAN sample_and_store();
BOOLEAN in_message_squelch_test();
BOOLEAN hi_speed_squelch_test();
void phase_lock_to_sample_stream();
void determine_encoded_bit_stream();
BOOLEAN locate_start_pattern();
BOOLEAN decode_data();
void determine_io();
void determine_output_control();
void determine_actions();
BOOLEAN is_long_msg_for_other_unit_imminent();
unsigned int get_action_nibble();
void configure_lines();
void tristate_inputs();
void configure_outputs();
unsigned int determine_output_state(unsigned int nOutput_State);
// void determine_encoded_msg_lsb_position();
void update_schedule_timer();
void store_schedule_periods_in_EEPROM();
void store_schedule_timer_in_EEPROM();
unsigned int determine_message_type_to_follow();
unsigned long  retrieve_2byte_period(unsigned int nNibble_Offset_Index);
unsigned int32 retrieve_3byte_period(unsigned int nNibble_Offset_Index);
void set_config_info();
void determine_power_level();

// Functions from threshold.c
void initialize_threshold();
void calculate_limits(unsigned long *plLower_Limit, unsigned long *plUpper_Limit);
void adjust_bias_voltage();
void store_bias_voltage();
void initialize_AD();
void shutdown_AD_and_restore_data_input_pin();
// void establish_C12_charge_time();
void enable_RFM();
void disable_RFM();
void increment_bias_voltage();
void decrement_bias_voltage();
int determine_special_remote_cmd();



#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
   void updatePSVoltageTarget();
#endif

#if SAVE_OSC_AND_PR2_IN_EEPROM==1
   int testVal=0;
   int eepromWriteNum=0;
#endif

#if SAVE_PWM_DUTY_IN_EEPROM==1
   int incrementCount=0;
   int decrementCount=0;
   int savEEPROMIndex=0x60;
#endif

#if SAVE_SQUELCH_TEST_VALUES==1
   int saveSquelchIndex=0;
   unsigned long squelchSampleCount=0;
#endif

#if SAVE_BIAS_ADJUST_VALUES==1
   int PWMSaveIndex=0;
#endif

// #separate float crude_log2(float fValue);
// void display_long_int_on_Line_6(unsigned long lVariable); // DEBUGGING CODE ONLY

//void pulse_pinE(unsigned int nPin_No, nDly);	//DEBUGGING ONLY
//void pulse_FETGate(nDly);						//DEBUGGING ONLY
///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                                  GLOBAL VARIABLES                                         //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
//*********************************ASC*********************************************************
//			New SharpTx

 char remoteChannel_Set,remoteChannel_Select;
char Channel = 0;
 char remoteLeft_Set, remoteRight_Set;
//*******************************Remote Control********************************************************
/////////////////////////////////////////////////////////////////////////////////////////


// STATIC (GLOBAL) VARIABLE DEFINITIONS

static unsigned int GnCurrent_Ch_Bias, GnCurrent_Ch_Bias_EEPROM, GnCurrent_Ch_ModIndx, GnCurrent_Ch_ModIndx_EEPROM;
static unsigned int GnSynth_Msg[3] = {0,0,0};


//static BOOLEAN GbAudio_Mode=STEREO; //initialize as Stereo
static BOOLEAN GbSubData_Msg_Complete, Bit_State = FALSE;
static BOOLEAN GbAudio_Left=TRUE;
      // arrays
unsigned int nSample_Buffer[SAMPLE_BUFFER_SIZE_BYTES];
unsigned int nAveraged_Sample_Buffer[ENCODED_MESSAGE_SIZE_BYTES];

unsigned int nRequired_Output_Control[4], nActual_Output_Control[4]; // holds the required and actual output control states
unsigned int N_BYTE1[8], N_BYTE2[8], N_BYTE3[8], R_BYTE2[8], R_BYTE3[8];

      //integers
unsigned long Vbat_ADC; //Measured Battery Voltage 10bits
unsigned long Vtx_PWM_Duty; // PWM duty factor for Tx power supply control

unsigned int COMMON_R1;
unsigned int   nMsg_Fail_Counter = 0, nWakeup_Counter_100ms = 0;
unsigned int   nRequired_IO_Direction=0xFF, nActual_IO_Direction=0xFF;   //by default all i/o lines to be forced to input
unsigned int   nZero_Crossing_Count, nOnes_Count;
unsigned long  lHi_Speed_Sample_Counter, lHi_Speed_Ones_Counter, lHi_Speed_0_Xing_Counter;
unsigned int   nFrame_Number, nStart_Frame_Number, nNumber_of_Frames_to_Analyze;
// unsigned int   nLast_Bit_Byte_Index, nLast_Encoded_Bit_BitIndex;
unsigned int   nLast_Timer1_Lower_Byte;
// unsigned long  lTimer1_On_Wake;
unsigned int   nListen_Interval = LISTEN_INTERVAL;
unsigned int Charge_Time = 0; 
unsigned int Iavg = 0, i;
unsigned long PWM_Duty_Array[11];
unsigned int  Ipoint = 0;
unsigned long Sum_PWM_Duty = 0;

unsigned int IRavg = 0;
unsigned long lPWM_Duty_Array[16];
unsigned int  IRpoint = 0;
unsigned long Sum_lPWM_Duty = 0;

unsigned int IRonavg = 0;
unsigned long lPWM_ON_Duty_Array[16];
unsigned int  IRonpoint = 0;
unsigned long Sum_lPWM_ON_Duty = 0;

signed int Duty_ON_Offset = -6;
signed int Duty_Offset = 5;
 
unsigned int   nElapsed_100ms_Cntr;
unsigned int   nElapsed_Second = 0;
signed int     nSample_Bit_Index;
 signed int    nFollow_On_TimeOut = 0;
unsigned int   nMessage_Type;
unsigned int   nConfig;

unsigned int32 dlOff_Period_Minutes;
  signed int32 dlSchedule_Minute_Timer; // values sent are 3 bytes, therefore sign does not reduce range
unsigned long  lOn_Period_Minutes;

unsigned long  lFrame_Sample_Index;
signed long	   lSample_Byte_Index, lView_Period_Countdown;
unsigned int   I8XSample;
unsigned int Power_Setting = 0x0;//Initialized for low power
int VtxDeadZoneRadius;
int delayBeforeQualify;
unsigned int Vbat_min_voltage;
unsigned int Vbat_low;
unsigned int Vbat_reset;

   // Flags
BOOLEAN  First_Search = TRUE;
BOOLEAN  bRFM_Enabled = FALSE;
BOOLEAN  bOn_Off = OFF;
//BOOLEAN  bOn_Off_State = OFF;
BOOLEAN  b5ms_Qualification_Failed = FALSE;
BOOLEAN  bValid_Message_Received = FALSE, bFirst_Frame_Failed = FALSE; //
BOOLEAN  bInitial_Activity;  
BOOLEAN  bLast_Received_Bit_Sample = FALSE, bLast_Hi_Speed_Bit_Sample = FALSE;
BOOLEAN  bSampling_Complete = NO;
BOOLEAN  bStart_Marker = NORMAL; // = INVERTED when looking for a follow-on message
BOOLEAN  bScheduling, bLong_Message_Flag, bStore_Timer_Flag = FALSE, bSchedule_State;
BOOLEAN  first_threshold_OFF = FALSE, first_threshold_ON = FALSE;
BOOLEAN  EncoderA_Flag = FALSE;
BOOLEAN  Bat_Lo_Flag = TRUE;
BOOLEAN  Change_ON_off = FALSE;
BOOLEAN  First_Invert = TRUE;

// ID Variables
unsigned long CUSTOMER_ID, RECEIVER_ID;

// Global variables associated with threshold setting
unsigned long lPWM_Duty;
#if BIAS_SAME_FOR_ON_OFF==0
  unsigned long lPWM_ON_Duty;
#endif
unsigned int  nDuty_Cycle_Step_Size;

//Test variable
unsigned int tmr1_count = 0;

#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
   unsigned long PS_Voltage_Target;
#endif

#define SAVE_FLAG_OF_CURRENT_STATE 0
#define SAVE_VTX_DUTY 0
#if SAVE_VTX_DUTY==1
char loByte;
char hiByte;
#endif

//#if SAVE_CLOCK_REGS_IN_EEPROM==1
   #byte CCP1CON=0x0FBD
   #byte CCPR1L=0x0FBE
   #byte TRISA=0x0F92
   char clkRegTemp;
//#endif

#if TOGGLE_LSB_BIAS_PWM==1
   int toggleLSBBiasPWM=0;
#endif

int playOrLiveCmdFlag=0;

//int wasOffNowOnFlag=0;
//int wasLowPowerNowHighFlag=0;
///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                              INTERRUPT SERVICE ROUTINES                                   //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                              SUBCHANNEL INTERRUPT SERVICE ROUTINE                         //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

unsigned int GnSubDataTemp[4];
int rfmJustEnabledFlag=0;
unsigned int16 baudRate;

//--------------------------------------------------------------------------------------------------------------------
#int_rda
void get_com_data() {
   rcreg=0x0fae;
   rx_contents= (char) *rcreg;  // read Rx COM port data

//   putcharx((unsigned char)rx_contents+1);  // this is an echo function for testing

   if (rxBufSize<RX_BUFF_SIZE_MAX) { // if there is room in buffer...
      inRxBuff[rxBufIndex++]=rx_contents; // put data in the buffer
      if (rxBufIndex>=RX_BUFF_SIZE_MAX) { // wrap index if necessary
         rxBufIndex=0;
      }
      rxBufSize++; // increment buffer size
   } else {
      rxBufOverflow++; // there was no room in buffer so increment overflow counter
   }
}
//--------------------------------------------------------------------------------------------------------------------

#int_EXT high    //SubData request
void EXT_isr()
{
//   unsigned int nCount;
   BOOLEAN bitFlag;
   unsigned long testval=0x00da;

   if (GbSubData_Msg_Complete)
       // If full message has been sent, send sync pulses to FPGA
   {
      output_high(SUBDATA);
      output_low(SUBDATA);
      output_high(SUBDATA);
      output_low(SUBDATA);
      output_high(SUBDATA);
      output_low(SUBDATA);

      GbSubData_Msg_Complete=FALSE;

   }  // end of 'if (GbSubData_Msg_Complete)'

   else
   {

      // A SubData logic zero is coded as a single pulse, a logic one as two pulses

   if (rfmJustEnabledFlag==1 ) {
      if (GnSubData_Byte_Index==0) {
         bitFlag=bit_test(testval,GnSubData_Bit_Index);
      } else {
         bitFlag=Bit_State;
      }
      if (GnSubData_Bit_Index==7) {
         rfmJustEnabledFlag=0;
      }
   } else {
      bitFlag=Bit_State;
   }

      if (bitFlag) // ..then determine whether a second pulse should be sent
      {
         output_high(SUBDATA);
         output_low(SUBDATA);
         output_high(SUBDATA);
         output_low(SUBDATA);
      }
      else
	  {
         output_high(SUBDATA);
         output_low(SUBDATA);
      }

      GnSubData_Bit_Index++;

      if (GnSubData_Bit_Index == 8)
         {
         GnSubData_Bit_Index = 0;
         GnSubData_Byte_Index++;

         if (GnSubData_Byte_Index == SUBDATA_MSG_LNGTH)
            {
            GnSubData_Byte_Index = 0;
            GbSubData_Msg_Complete=TRUE;
         }
      } //end of if (GnSubData_Bit_Index == 8)

   } //end of 'else'

//Setup for the next Subdata Interrupt
 

   Bit_State = bit_test(GnSubData[GnSubData_Byte_Index],GnSubData_Bit_Index);
   clear_interrupt(SUBDATA_CLOCK_INTERRUPT);     //reenable SubData Clock Interrupt by clearing Interrupt Flag
} //end of '#int_EXT'


////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                            //
//                              DIGITAL POT ADJUST INTERRUPT SERVICE ROUTINE                  //
//                             Note the digital pot ISR is cleared in the Tx_Adj_Pot function //
//                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////

//#ifndef NO_DIGITAL_POTENTIOMETER
#int_EXT1 noclear  //Rotary Encoder interrupt, falling edge; 'noclear' is to prevent nested triggers
void EXT1_isr()
   {
    EncoderA_Flag = TRUE;
//    Main_Mode.Tx_Mode=Tx_Pot_Adj;
    clear_interrupt(int_ext1);     //reenable SubData Clock Interrupt by clearing Interrupt Flag

} //end of #int_EXT1
//#endif

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                              TIMER 1 INTERRUPT SERVICE ROUTINE                            //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

#int_TIMER1
void  TIMER1_isr()
{	// NB Timer 1 has value 0 at this point
   unsigned int Current_TMR1L;
   unsigned long lCorrected_Timer1;

   disable_interrupts(INT_TIMER1);   //this will be enabled before sleep

#if SAVE_OSC_AND_PR2_IN_EEPROM==1
   if (eepromWriteNum<20) {
      testVal= *0x0FCB;
      write_eeprom(0x60+eepromWriteNum,0x01);
      write_eeprom(0x61+eepromWriteNum,testVal);
      eepromWriteNum++;
      eepromWriteNum++;
   }
#endif


//Reset timer by including the time taken to wakeup and service the interrupt
   Current_TMR1L = TMR1L;
   while(TMR1L == Current_TMR1L);	// Wait until TMR1 changes (to ensure it doesn't change during correction operation)
   lCorrected_Timer1 = get_timer1()+TMR1_INIT;   //NB Timer 1 should have a value of the time since rollover at this point
   set_timer1(lCorrected_Timer1);

   nWakeup_Counter_100ms++; // Increment this counter (inc's every 100ms)
   if(nFollow_On_TimeOut>0)
      nFollow_On_TimeOut--;

   if(++nElapsed_100ms_Cntr > 9)
   {
      nElapsed_100ms_Cntr = 0;

	  // now advance Timer1 by 2 ticks to account for error that 100ms is not an exact multiple of 3277 x 32768Hz ticks
	  // (3277 x 10 = 32770 i.e. 2 ticks two long)
	 Current_TMR1L = TMR1L;
	 while(TMR1L == Current_TMR1L);	// Wait until TMR1 changes (to ensure it doesn't change during correction operation)

	 lCorrected_Timer1 = get_timer1() + 2;   //NB Timer 1 should have a value of 0x0001 at this point
	 set_timer1(lCorrected_Timer1);


      if(++nElapsed_Second > 59)
      {
         nElapsed_Second = 0;
         dlSchedule_Minute_Timer--;
         bStore_Timer_Flag = TRUE;

      } // end of 'if(++Elapsed_Sec_Cntr > 59)'
   } // end of 'if(++100ms_Elapsed_cntr > 9)'

 if(dlSchedule_Minute_Timer <= 0)
   {
   Change_ON_off = TRUE;
   if(bSchedule_State)  //if we are currently in an 'On' period...
     dlSchedule_Minute_Timer = dlOff_Period_Minutes; 
   else
     dlSchedule_Minute_Timer = lOn_Period_Minutes; 
   }
tmr1_count++;
clear_interrupt(INT_TIMER1);
enable_interrupts(INT_timer1);
}

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                                  MAIN CODE                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

// INITIALIZATION.....

void main()
{
   unsigned int nCount, nValue;

   setup_oscillator(OSC_32MHZ);

//Initialize the interrupts

   disable_interrupts(int_ext);
   disable_interrupts(int_ext1);

   //Leave all interrupts disabled (by default on power-up) until required
   ext_int_edge(0,L_TO_H);    //set RB0's interrupt to rising edge (SubData call from FPGA)
   ext_int_edge(1,H_TO_L);    //set RB1's interrupt to rising edge (Encoder A input)



#ifdef LOW_SENSITIVITY_AND_CURRENT
	set_tris_A(TRIS_A_LOW_CURRENT);     // "                 "                 '              "
#else
	set_tris_A(TRIS_A_HIGH_SENSITIVITY);
#endif
   set_tris_B(TRIS_B);     //             "                 "                 '              "
   set_tris_C(TRIS_C);     //These TRIS's result in all lines powering up as tristate (hi impedance)
   set_tris_E(TRIS_E);     //             "                 "                 '              "
   port_b_pullups(PORTB_PULLUPS_STATE);
#if USING_PIC18LF26K22==1
   setup_adc_ports(sAN0|sAN1|sAN2,VSS_VDD);
#else
   setup_adc_ports(AN0_TO_AN2|VSS_VDD);
#endif
   setup_adc(ADC_CLOCK_INTERNAL|ADC_TAD_MUL_20);
   setup_spi(FALSE);
   setup_spi(SPI_SS_DISABLED);
   setup_wdt(WDT_OFF);
   setup_timer_0(RTCC_OFF);
   setup_CCP2 (CCP_OFF);

#if USING_PIC18LF26K22==1
   setup_timer_1(T1_EXTERNAL|T1_DIV_BY_1|T1_ENABLE_SOSC);
#else
   setup_timer_1(T1_EXTERNAL|T1_DIV_BY_1|T1_CLK_OUT);
#endif
   delay_msec(50);	// Give Timer 1 time to get up and running

   setup_timer_2(T2_DIV_BY_1,PR2,1);   // For 32MHz clk PR2 = 127, for pwm period of 62.5KHz

//while (TRUE) output_low(SUBDATA);//DEBUG ONLY

// Initialize TX to off
//   output_low(REMOTE_RCVR_PWR_CTL); no power control on Flyron version, commented out 7-12-17 MHPatten
   output_low(VTX_PWR_CTL);
   output_float(SDA);
   output_float(AD_CHIP_SLCT);
   output_float(SUBDATA);
   output_float(RIGHT_PIN);
   output_float(SCL);
   output_float(WP);
   output_float(LEFT_PIN);
   output_float(SYNTH_LE);
   output_float(TX_PWR_PWM_PIN);
#if OSC_DIV_4_ON_RA7==0
   output_float(RX_SENSITIVITY_CTL_PIN);   // .. set the Sensitivity to High (when RFM is enabled)
#endif
//   output_float(REMOTE_RCVR_PWR_CTL); no power control on Flyron version, commented out 7-12-17 MHPatten
   Main_Mode.Tx_Mode = Tx_Off;
   set_timer1(TMR1_INIT);           //intialize the timer for a 100ms wakeup tick

#if CONFIG_UART_IN_HARDWARE==1
    SPBRGH=0x00;
    SPBRG=0x33;
	RCSTA=0x90;
    TXSTA|=0x20;
#endif
//Debug only********************
//while (TRUE) Main_Mode.Tx_Mode = Tx_Off;
 
//   write_eeprom(0x70, 0); // test register to see is synth initializes (3/26/14 MPatten)
#if SAVE_FLAG_OF_CURRENT_STATE==1
   write_eeprom(0x4a,0xEE); // test register to see is synth initializes (3/26/14 MPatten)
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                  INITIALIZE TRANSMITTER FREQUENCY TABLE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

   if (READ_EEPROM(EEPROM_FREQ_SOURCE)==0)
	  {
	  COMMON_R1=READ_EEPROM(EEPROM_COMMON_R1_ADDRESS);
	  for (nCount=0; nCount<8; nCount++)
		 {
		 N_BYTE1[nCount]=READ_EEPROM(EEPROM_N_BYTE1_BASE+nCount);
		 N_BYTE2[nCount]=READ_EEPROM(EEPROM_N_BYTE2_BASE+nCount);
		 N_BYTE3[nCount]=READ_EEPROM(EEPROM_N_BYTE3_BASE+nCount);
		 R_BYTE2[nCount]=READ_EEPROM(EEPROM_R_BYTE2_BASE+nCount);
		 R_BYTE3[nCount]=READ_EEPROM(EEPROM_R_BYTE3_BASE+nCount);
		 }
	  }
   else if (READ_EEPROM(EEPROM_FREQ_SOURCE)==1)
	  {
	  COMMON_R1=COMMON_360_R1;
	  for (nCount=0; nCount<8; nCount++)
		 {
		 N_BYTE1[nCount]=COMMON_360_N1;
		 N_BYTE2[nCount]=N_360_BYTE2[nCount];
		 N_BYTE3[nCount]=N_360_BYTE3[nCount];
		 R_BYTE2[nCount]=R_360_BYTE2[nCount];
		 R_BYTE3[nCount]=R_360_BYTE3[nCount];
		 }
	   }
	else
	   {
	   COMMON_R1=COMMON_900_R1;
	   for (nCount=0; nCount<8; nCount++)
		  {
		  N_BYTE1[nCount]=COMMON_900_N1;
		  N_BYTE2[nCount]=N_900_BYTE2[nCount];
		  N_BYTE3[nCount]=N_900_BYTE3[nCount];
		  R_BYTE2[nCount]=R_900_BYTE2[nCount];
		  R_BYTE3[nCount]=R_900_BYTE3[nCount];
		  }
	   }

   baudRate = read_eeprom(EEPROM_BAUD_RATE)*256+read_eeprom(EEPROM_BAUD_RATE+1);
//#use rs232()

//////////////////////////////////////////////////////////////////////////////
//Read Cutomer ID and Receiver ID from EEPROM
/////////////////////////////////////////////////////////////////////////////
   CUSTOMER_ID = read_eeprom(EEPROM_CUSTOMER_ID_BASE)+256*(READ_EEPROM(EEPROM_CUSTOMER_ID_BASE+1));
   RECEIVER_ID = read_eeprom(EEPROM_RECEIVER_ID_BASE)+256*(READ_EEPROM(EEPROM_RECEIVER_ID_BASE+1));

   VtxDeadZoneRadius=read_eeprom(EEPROM_DEAD_ZONE_RADIUS_ADDRESS);
   delayBeforeQualify=read_eeprom(EEPROM_DELAY_MS_BEFORE_QUAL_ADDRESS);
   Vbat_min_voltage=read_eeprom(EEPROM_VBAT_MIN_VOLTAGE_ADDRESS);
   Vbat_low=read_eeprom(EEPROM_VBAT_LOW_VOLTAGE_ADDRESS);
   Vbat_reset=read_eeprom(EEPROM_VBAT_RESET_VOLTAGE_ADDRESS);

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                  INITIALIZATION OF UNIT STATE                                          //
   ////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// set the required line control statuses

//#ifdef DISABLE_SCHEDULE_ON_POWERUP
//      nConfig = EEPROM_INITIAL_CONFIG_STATE;
//#else
      nConfig = read_eeprom(EEPROM_CONFIG_STATE);
//#endif


   // set the schedule configuration


   dlSchedule_Minute_Timer = make32( read_eeprom(EEPROM_MINUTE_TIMER_BYTE4of4),
                                     read_eeprom(EEPROM_MINUTE_TIMER_BYTE3of4),
                                     read_eeprom(EEPROM_MINUTE_TIMER_BYTE2of4),
                                     read_eeprom(EEPROM_MINUTE_TIMER_BYTE1of4) );

   dlOff_Period_Minutes = make32( read_eeprom(EEPROM_OFF_PERIOD_BYTE3of3),
                                  read_eeprom(EEPROM_OFF_PERIOD_BYTE2of3),
                                  read_eeprom(EEPROM_OFF_PERIOD_BYTE1of3) );

   lOn_Period_Minutes = make32( read_eeprom(EEPROM_ON_PERIOD_BYTE2of2),
                                  read_eeprom(EEPROM_ON_PERIOD_BYTE1of2) );

   if(bit_test(nConfig,STORED_SCHEDULE_STATUS_BIT))
         bScheduling = ON;
   else  bScheduling = OFF;

   if(bit_test(nConfig,STORED_SCHEDULE_STATE_BIT))
         bSchedule_State = ON;
   else  bSchedule_State = OFF;

   set_data_speed_and_polarity(); // set the data speed and polarity in the FPGA
   remoteChannel_Select = read_eeprom(0XF0);
   remoteChannel_Set = 0Xff;  // ***ASC*** ensure channel change
   Channel = 7-remoteChannel_Select;
   GbAudio_Mode = read_eeprom(0XF1);
   GbAudio_Left = read_eeprom(0XF2);
  
///////////////////////////////////////////////
//Initialize TX Subchannel
///////////////////////////////////////////////
   //Copy EEPROM Sub-Data information to RAM
      For (nValue=0; nValue<= (SUBDATA_MSG_LNGTH-1); nValue++)  // array size = SUBDATA_MSG_LNGTH,
                                                                // last element (SUBDATA_MSG_LNGTH-1) = Battery Voltage
															    // DAC mod to get byte 0 - checksum bytes 1 & 2 ID from EEPROM
         GnSubData[nValue]= read_eeprom((SUBDATA_VALUE_BASE_ADDRESS+nValue));

      GbSubData_Msg_Complete=TRUE; //Start Subchannel with sync bit

////////////////////////////////////////
//Initialize PWM Offsets
////////////////////////////////////////

     Duty_ON_Offset = read_eeprom(PWM_ON_OFFSET_ADD);
     Duty_Offset = read_eeprom(PWM_OFFSET_ADD);

 

   ////////////////////////////////////////////////////////////////////////////////////////////////////////////

   //restart_wdt();
     delay_msec(RFM_PWR_UP_DELAY_MS);    // give the receiver module time to stabilize
   //restart_wdt();


    First_Search = TRUE;
    disable_RFM();

////////////////////////////////////////////////////////////////////////////////
//Initialize the min and max Threshold measurements - for test only
////////////////////////////////////////////////////////////////////////////////
/*		write_eeprom(EEPROM_MIN_PWM_HI_BYTE,make8(lPWM_Duty,1));
		write_eeprom(EEPROM_MIN_PWM_LO_BYTE,make8(lPWM_Duty,0));
		write_eeprom(EEPROM_MAX_PWM_HI_BYTE,make8(lPWM_Duty,1));
		write_eeprom(EEPROM_MAX_PWM_LO_BYTE,make8(lPWM_Duty,0));
		write_eeprom(EEPROM_MIN_PWM_ON_HI_BYTE,make8(lPWM_Duty,1));
		write_eeprom(EEPROM_MIN_PWM_ON_LO_BYTE,make8(lPWM_Duty,0));
		write_eeprom(EEPROM_MAX_PWM_ON_HI_BYTE,make8(lPWM_Duty,1));
		write_eeprom(EEPROM_MAX_PWM_ON_LO_BYTE,make8(lPWM_Duty,0));
	
*/ 
////////////////////////////////////////////////////////////////////////////////
//Initialize the Transmit and Receive State Machines
////////////////////////////////////////////////////////////////////////////////

   if(bit_test(nConfig,OUTPUT_STATE_BIT))
     {
      bOn_Off = ON;
      Main_Mode.Tx_Mode = Tx_OfftoOn_Vtx; //Start to turn on transmitter
     } 
   else
     {
      bOn_Off = OFF;
      Main_Mode.Tx_Mode = Tx_Off;  //Leave transmitter off
     } 

   Main_Mode.Rx_Mode = Sig_Check; //Start receive cycle


   // Initialize the RX6000/RX5500 CMPIN bias voltage setting
#if BIAS_SAME_FOR_ON_OFF==0
        lPWM_ON_Duty = make16(read_eeprom(EEPROM_PWM_ON_SETTING_HI_BYTE), read_eeprom(EEPROM_PWM_ON_SETTING_LO_BYTE));
#endif
        lPWM_Duty = make16(read_eeprom(EEPROM_PWM_SETTING_HI_BYTE), read_eeprom(EEPROM_PWM_SETTING_LO_BYTE));

//////////////////////////////////////////////////////////////////////
//            Zero PWM Averaging Arrays
//////////////////////////////////////////////////////////////////////

 #if BIAS_SAME_FOR_ON_OFF==0
     for (i=0; i<16; i++) lPWM_ON_Duty_Array[i]=lPWM_ON_Duty;
#endif
      for (i=0; i<16; i++) lPWM_Duty_Array[i]=lPWM_Duty;
      IRavg = 15;
       IRonavg = 15;
       Sum_lPWM_Duty = lPWM_Duty*15;
#if BIAS_SAME_FOR_ON_OFF==0
       Sum_lPWM_ON_Duty = lPWM_ON_Duty*15;
#endif

    first_threshold_OFF = FALSE;
    first_threshold_ON = FALSE;
//	initialize_threshold();
    disable_RFM(); //RF signal search not successful continue signal search

    clear_squelch_counters();

   Power_Setting = read_eeprom(INIT_PWR_ADD);

#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
   updatePSVoltageTarget();
#endif
//Enable Interrupts before entering main loop

   clear_interrupt(INT_EXT);
   enable_interrupts(INT_TIMER1);   //make sure we get time ticks and wakeup calls

   if (bOn_Off)
     {
      clear_interrupt(INT_EXT);
      delay_usec(100);
      enable_interrupts(INT_EXT);   //enables subdata
      clear_interrupt(INT_EXT1);
      enable_interrupts(INT_EXT1);   //enables external encoder
      delay_usec(100);
      clear_interrupt(INT_EXT1);
      delay_usec(100);
     } 
   enable_interrupts(int_rda);
   enable_interrupts(GLOBAL);


// New from Mark 11-10-12: will this succeed in initializing the power level correctly?

  if (bOn_Off == ON) {  // only do it if transmitter is meant to be on
#ifdef COMMON_CHANNEL_BIAS
      GnCurrent_Ch_Bias_EEPROM = read_eeprom(BIAS_VALUE_BASE_ADDRESS);
#else
      if (Power_Setting==0) {
         GnCurrent_Ch_Bias_EEPROM = read_eeprom(BIAS_VALUE_BASE_ADDRESS + Channel+8);
      } else {
         GnCurrent_Ch_Bias_EEPROM = read_eeprom(BIAS_VALUE_BASE_ADDRESS + Channel);
      }
#endif
//      GnCurrent_Ch_Bias 		= GnCurrent_Ch_Bias_EEPROM & Power_Setting;
      GnCurrent_Ch_Bias 		= GnCurrent_Ch_Bias_EEPROM; // & Power_Setting;
      set_pot(GnCurrent_Ch_Bias, POT0);
  } 


#if USE_FIXED_BIAS==1
          set_pwm1_duty(lPWM_Duty);

          
#endif

//   disable_interrupts(INT_EXT1);//*****Note comment out this line to enable bias / mod adjustments
                                //          Plus two other places find enable_interrupts(INT_EXT1)

   adSource=2; // force AD source to be reset
   readMclrSetADSource();


///////////////////////////////////////////////////////////////////////////////////////////////
// -------------------------------> MAIN LOOP STARTS HERE <------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////

#if USE_SHAFT_ENCODER_FOR_TEST==1
   output_low(ENC_CH_A);
#endif

   While(TRUE)
   {

/*
while(TRUE) {
      putchar('M'); //copy byte at current read index to transmit buffer
      putchar('a'); //copy byte at current read index to transmit buffer
      putchar('r'); //copy byte at current read index to transmit buffer
      putchar('k'); //copy byte at current read index to transmit buffer
      putchar(' '); //copy byte at current read index to transmit buffer
      putchar('P'); //copy byte at current read index to transmit buffer
      putchar('a'); //copy byte at current read index to transmit buffer
      putchar('t'); //copy byte at current read index to transmit buffer
      putchar('t'); //copy byte at current read index to transmit buffer
      putchar('e'); //copy byte at current read index to transmit buffer
      putchar('n'); //copy byte at current read index to transmit buffer
      putchar(32); //copy byte at current read index to transmit buffer
}
*/

#if USE_SHAFT_ENCODER_FOR_TEST==1
   if (toggleHere==0) {
      output_high(ENC_CH_A);
      toggleHere=1;
   } else {
      output_low(ENC_CH_A);
      toggleHere=0;
   }
#endif


#if USE_PUSHBUTTON_MOD_PWR_ADJ==1
     scan_pwr_mod_adjust_buttons();
#endif

///////////////////////////////////////
// Start Main State Machine
///////////////////////////////////////

#if SAVE_CLOCK_REGS_IN_EEPROM==1
   clkRegTemp= CCP1CON;
   if (read_eeprom(0x98)==0xff) {
      write_eeprom(0x98,clkRegTemp);
   }

   clkRegTemp= CCPR1L;	
   if (read_eeprom(0x99)==0xff) {
      write_eeprom(0x99,clkRegTemp);
   }
#endif

   outputRecStateOnSubchan();

// Rx State

   switch (Main_Mode.Rx_Mode)
      {
	  case sleep_mode:
         Main_Mode.Rx_Mode=go_to_sleep();
         break;
	  case sig_check:

#if SAVE_CLOCK_REGS_IN_EEPROM==1
   clkRegTemp= CCP1CON;
   if (read_eeprom(0x9a)==0xff) {
      write_eeprom(0x9a,clkRegTemp);
   }

   clkRegTemp= CCPR1L;	
   if (read_eeprom(0x9b)==0xff) {
      write_eeprom(0x9b,clkRegTemp);
   }
#endif

         Main_Mode.Rx_Mode=sig_chk();
         break;
      case Rx:


         Main_Mode.Rx_Mode=Rcv();
         break;
      case process:
         Main_Mode.Rx_Mode=processf();
         break;
      case action:
         Main_Mode.Rx_Mode=actionf();
         break;
      case schedule:
         Main_Mode.Rx_Mode=schedulef();
         break;
      }

// Tx State

   switch (Main_Mode.Tx_Mode)
      {
      case Tx_Bypass:
		 Main_Mode.Tx_Mode=TxMode_Save;
		 break;
      case Tx_OfftoOn_Vtx:
         Main_Mode.Tx_Mode=Tx_OfftoOn_Vtxf();
         break;
      case Tx_OfftoOn_InitTx:
         Main_Mode.Tx_Mode=Tx_OfftoOn_InitTxf();
         break;
      case Tx_On_Change:
         Main_Mode.Tx_Mode=Tx_On_Changef();
         break;
      case Tx_On_Chk_adj_PS:
         Main_Mode.Tx_Mode=Tx_On_Chk_adj_PSf();
         break;
      case Tx_On_MeasureBatV:
         Main_Mode.Tx_Mode=Tx_On_MeasureBatVf();
         break;
      case Tx_OntoOff:
         Main_Mode.Tx_Mode=Tx_OntoOffF();
         break;
      case Tx_Pot_Adj:
#if USE_SHAFT_ENCODER_FOR_TEST==0
         Main_Mode.Tx_Mode=Tx_Pot_AdjF();
#endif
         break;
      }


   } //End of While(TRUE)
} //End of Main
///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                                  END OF MAIN()                                            //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
   void updatePSVoltageTarget() {
      unsigned int PSTargetAddress; 
      PSTargetAddress=VTX_VALUE_BASE_ADDRESS+Channel;
      if (Power_Setting==0) {
        PSTargetAddress+=8;
      }
      PS_Voltage_Target = (unsigned long)read_eeprom(PSTargetAddress);
      if (PS_Voltage_Target > PS_VOLTAGE_MAX_SETTING) { // safety in case EEPROM has too high voltage setting (0xFF for example) 9-15-13 MPatten
         PS_Voltage_Target = (unsigned long)PS_VOLTAGE_MAX_SETTING;  // set to value for low voltage
         write_eeprom(PSTargetAddress,(int8)(PS_Voltage_Target&0x0ff));
      }
   }
#endif


///////////////////////////////////////////////////////////////////////////////////////////
//
//                Signal Check State
//
//    Enables the receiver, set last threshold setting and checks for signal
//    Returns next state
//
///////////////////////////////////////////////////////////////////////////////////////////

char sig_chk()

{   
    BOOLEAN result;
/*    if (bOn_Off)
      if (first_threshold_ON == FALSE) initialize_threshold();
    else
      if (first_threshold_OFF == FALSE) initialize_threshold();

*/

      bValid_Message_Received = FALSE;		
	  bInitial_Activity = FALSE;

	  clear_squelch_counters();
       nMsg_Fail_Counter = 0;
/*
//            Zero PWM Averaging Arrays
       for (i=0; i<16; i++) lPWM_ON_Duty_Array[i]=0;
       for (i=0; i<16; i++) lPWM_Duty_Array[i]=0;
       IRonpoint = 0;
       IRpoint = 0;
       IRavg = 0;
       IRonavg = 0;
       Sum_lPWM_Duty = 0;
       Sum_lPWM_ON_Duty = 0;
*/

      while (nMsg_Fail_Counter < MAX_NUMBER_TRIES)
      {

   result=initial_RF_activity_search();
   if (result)
         {

//         nMsg_Fail_Counter = 0;  
#use delay(clock=32000000)
         setup_oscillator(OSC_32MHZ);


         return Rx;  //RF signal search successful go to next Rx state
         }
#if USE_FIXED_BIAS==0
      if(nMsg_Fail_Counter == 0)

        adjust_bias_voltage();

#endif
      nMsg_Fail_Counter++;
      } 

//        if(!hi_speed_squelch_test())  // If true 5ms sample is noise
      store_bias_voltage();	// stores if current lPWM_Duty differs from stored value by > 2
      
      disable_RFM(); //RF signal search not successful continue signal search
      nZero_Crossing_Count = 0;
      nOnes_Count = 0;
//	  clear_squelch_counters();
//      return schedule;//Process schedule then go to sleep 
      nWakeup_Counter_100ms = 0;
      nListen_Interval = LISTEN_INTERVAL;  //ensure nListen_Interval is reset after a long sleep (ignoring non-target schedule data)
      return schedule;//Process schedule then go to sleep 
          
 } //End of Signal Check State Processing


////////////////////////////////////////////////////////////////////////////////////
//
//                   Receive State
//
//  Gather receive data, check for valid data, set next state
//
////////////////////////////////////////////////////////////////////////////////////

char Rcv()
  {
      unsigned int nin_Msg_Fail_Counter, nin_Msg_Fail_Tries;

      if (bOn_Off)
        {
          nin_Msg_Fail_Tries = MAX_NUMBER_TRIES + 5;
//	      lPWM_ON_Duty=lPWM_ON_Duty+Duty_ON_Offset;// R7 100 C7 1000p
#if USE_FIXED_BIAS==0
  #if BIAS_SAME_FOR_ON_OFF==1
          set_pwm1_duty(lPWM_Duty+Duty_Offset);
  #else
          set_pwm1_duty(lPWM_ON_Duty+Duty_ON_Offset);
  #endif
#endif
/*
//            Zero PWM Averaging Arrays
          for (i=0; i<16; i++) lPWM_ON_Duty_Array[i]=0;
          for (i=0; i<16; i++) lPWM_Duty_Array[i]=0;
          IRonpoint = 0;
          IRpoint = 0;
*/
        }
        else
        {
        nin_Msg_Fail_Tries = MAX_NUMBER_TRIES;
#if USE_FIXED_BIAS==0
#if DONT_CHANGE_PWM_DURING_RCV==0
        set_pwm1_duty(lPWM_Duty+Duty_Offset);
#endif
#endif
//        set_pwm1_duty(lPWM_Duty-2);//R7 100 C7 1000p
        }

      I8XSample=0; //Zero count required before threshold adjustment during data RCV

      while (nMsg_Fail_Counter < nin_Msg_Fail_Tries) // increments AFTER each attempt
      {
       
		bInitial_Activity = TRUE;
        bFirst_Frame_Failed = FALSE;


//         nin_Msg_Fail_Counter=0;
         do
         {
             // start collecting samples and perform running tach squelch test during first frame
            //restart_wdt();
            bSampling_Complete = sample_and_store();
            if((nFrame_Number == 1) && ((lFrame_Sample_Index & 0x001F) == 0))	//triggers every 32 samples
            {
               if(!in_message_squelch_test())
               {
//                 if(nin_Msg_Fail_Counter>3)
                   bfirst_Frame_Failed = TRUE;

/*                   bSampling_Complete = FALSE; //Reset
                   lFrame_Sample_Index = 0;	// reset
                   nFrame_Number = 1;		// reset
                   lSample_Byte_Index = SAMPLE_BUFFER_SIZE_BYTES -1;   //set to top byte of array
                   nSample_Bit_Index = 7;
                   
                  nin_Msg_Fail_Counter++;  */
               }
               nZero_Crossing_Count = 0;
               nOnes_Count = 0;
            }
           } while(!bSampling_Complete && !bFirst_Frame_Failed);


         if(!bFirst_Frame_Failed & bSampling_Complete) // this single amperstand seems suspicious, I tried a double "logical and" but didn't work the same 9-23-13 MPatten
            {
            return Process; // Successfully received all frames of rx data
            }
	     else
           {

            if(nFollow_On_TimeOut <= 0)
             {
              bStart_Marker = NORMAL;     // if the wait for a follow on message has timed out, revert to waiting for normal messages
		 	  nMsg_Fail_Counter++;
              initial_RF_activity_search(); //Re-init for next try
             }
            else
             {
              initial_RF_activity_search(); //Re-init for next try while waiting for schedule message
              return Rx;
             }
           } 
      } //End of nMsg_Fail_Counter < MAX_NUMBER_TRIES

   // Failed trying to receive stream re-initialize the RX6000/RX5500 CMPIN bias voltage setting for the next

        disable_RFM(); //RF signal search not successful continue signal search
//        lPWM_ON_Duty = Sum_lPWM_ON_Duty/IRonavg;
//        lPWM_Duty = Sum_lPWM_Duty/IRavg;
//        lPWM_ON_Duty = make16(read_eeprom(EEPROM_PWM_ON_SETTING_HI_BYTE), read_eeprom(EEPROM_PWM_ON_SETTING_LO_BYTE));
//        lPWM_Duty = make16(read_eeprom(EEPROM_PWM_SETTING_HI_BYTE), read_eeprom(EEPROM_PWM_SETTING_LO_BYTE));


      return schedule;  // Receive data unsuccessful

   } //End of Receive State


////////////////////////////////////////////////////////////////////////////////////////
//
//                    Process Received Data
//
//   Phase locks to sample stream of data
//   determines encoded bit stream 
//   does frame averaging result in averaged buffer
//   locates start bit pattern & positions start pattern in 2 LSB's of
//     nAveraged_Sample_Buffer
//   does 4b6b decoding
//
////////////////////////////////////////////////////////////////////////////////////////

char Processf()

         {

         // Analyze the gathered data
	      nStart_Frame_Number = 1;  // nStart_Frame_Number range is 1 to NUMBER_FRAMES_TO_SAMPLE
          bValid_Message_Received = FALSE;
 
            do
            {
               nNumber_of_Frames_to_Analyze = NUMBER_FRAMES_TO_SAMPLE - (nStart_Frame_Number-1);

               phase_lock_to_sample_stream();	//performed once for each start frame increment (lock to start frame)
               do
               {
                  determine_encoded_bit_stream();
                  if(locate_start_pattern() )
                     bValid_Message_Received = decode_data();

               } while(!bValid_Message_Received && (--nNumber_of_Frames_to_Analyze > 0));
            }	while(!bValid_Message_Received && (++nStart_Frame_Number <= NUMBER_FRAMES_TO_SAMPLE));

            if(!bValid_Message_Received)
               {
                if (bStart_Marker == INVERTED)
                  {
                   initial_RF_activity_search(); //Re-init for next try
                   return Rx;
                  } 
                else 
                  {
		       	   nMsg_Fail_Counter++;
                   disable_RFM(); //RF signal search not successful continue signal search
//                   lPWM_ON_Duty = Sum_lPWM_ON_Duty/IRonavg;
//                   lPWM_Duty = Sum_lPWM_Duty/IRavg;
//                   lPWM_ON_Duty = make16(read_eeprom(EEPROM_PWM_ON_SETTING_HI_BYTE), read_eeprom(EEPROM_PWM_ON_SETTING_LO_BYTE));
//                   lPWM_Duty = make16(read_eeprom(EEPROM_PWM_SETTING_HI_BYTE), read_eeprom(EEPROM_PWM_SETTING_LO_BYTE));
                   return schedule;
                  }
               }
                else
               {
                if(First_Invert) // First invert and start marker inverted.  Go back and receive another set of data to esure follow on message is not corupted by normal message
                 {
                  First_Invert = FALSE;
                  initial_RF_activity_search(); //Re-init for next try
                  return Rx;
                 }
                else
                 {
                  return action;
                 } 
               }
         } // End of Process received sample stream
 
//////////////////////////////////////////////////////////////////////////////////////////////////
//
//                             Action State
//
//  Verifies customer ID and receiver ID
//  Test for Tx on/off status
//  Test for Left, Right, Stereo change
//  Test for Channel change
//  Sets Tx mode for required actions
//
//////////////////////////////////////////////////////////////////////////////////////////////////
char Actionf()

   {
   int returnval;

          //Message received   - now decode it

         if(bStart_Marker == NORMAL)
         {
           if(verify_customer_ID() && verify_receiver_ID() )   //verify 8-bit Customer ID & 12-bit receiver ID
             {
               returnval=determine_special_remote_cmd();
               if (returnval!=0) {

                  if (returnval==1) {
                     processSpecialremoteCmd();
                  }

                  disable_RFM(); //RF signal search not successful continue signal search
                  return schedule; // next state process schedule
               }
   			   determine_actions();		  // Retrieve the 'Action' nibble and set flags and/or act accordingly
			   determine_stereo();
			   determine_channel();
			   determine_power_level();

               if(bLong_Message_Flag) // if extended message is expected, setup to decode and act on it
                 {
                  nMessage_Type = determine_message_type_to_follow();
                  bStart_Marker = INVERTED;
                  nFollow_On_TimeOut = FOLLOWING_MESSAGE_TIMEOUT_x100MS; // Initialize the Time-Out period for the follow-on message
//----------------------------------------------------------------------------- 
                  First_Invert = TRUE;
//                  nListen_Interval = LISTEN_INTERVAL + 20;  // Extend next sleep period by 2 secs (one time only)
                  nMsg_Fail_Counter = 0;
	              clear_squelch_counters();
				  TxMode_Save = Main_Mode.Tx_Mode; //Necessary to bypass tx to receive rest of message
			   	  Main_Mode.Tx_Mode = Tx_Bypass;			
                  initial_RF_activity_search(); //Re-init for rest of long tx
                  return Rx;
//----------------------------------------------------------Added from below
                 } // end of 'if(bLong_Message_Flag) / else...'
             }  //end of if (verify_customer_ID ...  
/*
             else // Check to see if Schedule cmd is imminent; if so, switch off remote for Listen Interval + 2 secs (one time only)
             {
               if(is_long_msg_for_other_unit_imminent())
                  nListen_Interval = LISTEN_INTERVAL + 20;  // Extend next sleep period by 2 secs (one time only)

             } //end of else if (verify_customer_ID ...

            if(bLong_Message_Flag) // if extended message is expected, setup to decode and act on it
               {
                nMsg_Fail_Counter = 0;
	            clear_squelch_counters();
				TxMode_Save = Main_Mode.Tx_Mode; //Necessary to bypass tx to receive rest of message
				Main_Mode.Tx_Mode = Tx_Bypass;			
                initial_RF_activity_search(); //Re-init for rest of long tx
                return Rx;
               } 
*/
           } // end of 'if(bStart_Marker == NORMAL)'

           else  // bStart_Marker == INVERTED:- The data in the message is follow-on data.
             {
		      dlSchedule_Minute_Timer = retrieve_3byte_period(SCHEDULE_START_TIME_OFFSET_NIBBLE_INDEX);
              nElapsed_Second = 0;
              nElapsed_100ms_Cntr = 0;

		      lOn_Period_Minutes = retrieve_2byte_period(SCHEDULE_ON_PERIOD_OFFSET_NIBBLE_INDEX);

		      if(!((dlSchedule_Minute_Timer == 0) && (lOn_Period_Minutes == 0))) //if either dlSchedule_Minute_Timer OR lOn_Period_Minutes are NOT zero..
                {
                 dlOff_Period_Minutes = retrieve_3byte_period(SCHEDULE_REPEAT_PERIOD_OFFSET_NIBBLE_INDEX);	// actually retrieves repeat period

			     if(dlOff_Period_Minutes > lOn_Period_Minutes)
					dlOff_Period_Minutes = dlOff_Period_Minutes - lOn_Period_Minutes;	// Off = Repeat - On
		  	     else dlOff_Period_Minutes = 0;

			     bScheduling = ON;
                 bSchedule_State = OFF;

                 bOn_Off = OFF;          // As per Bill M's e-mail 6/29/05 (point #2)
                 Main_Mode.Tx_Mode = Tx_OntoOff;
                 
			     store_schedule_periods_in_EEPROM();
	             store_schedule_timer_in_EEPROM();

                 }  // end of 'if((dlSchedule_Minute_Timer <= 0) || (lOn_Period_Minutes != 0))'

               bStart_Marker = NORMAL;  // Long Message wait has ended, back to waiting for 'Normal' messages

               } // end of 'else  // bStart_Marker = INVERTED:- The data is the message is follow-on data'

            // Finished receiving both regular and long re-initialize the RX6000/RX5500 CMPIN bias voltage setting for the next stream
              disable_RFM(); //RF signal search not successful continue signal search
//              lPWM_ON_Duty = Sum_lPWM_ON_Duty/IRonavg;
//              lPWM_Duty = Sum_lPWM_Duty/IRavg;
//              lPWM_ON_Duty = make16(read_eeprom(EEPROM_PWM_ON_SETTING_HI_BYTE), read_eeprom(EEPROM_PWM_ON_SETTING_LO_BYTE));
//              lPWM_Duty = make16(read_eeprom(EEPROM_PWM_SETTING_HI_BYTE), read_eeprom(EEPROM_PWM_SETTING_LO_BYTE));

         return schedule; // next state process schedule

   } // end of action()


////////////////////////////////////////////////////////////////////////////////////////////////
//
//                           Activate or Process Schedule
//
////////////////////////////////////////////////////////////////////////////////////////////////
char Schedulef()

   {

    if(bStart_Marker == NORMAL)
        {

         if(bScheduling)
         {

//            if(dlSchedule_Minute_Timer <= 0)
            if(Change_ON_off)
            {
               disable_interrupts(INT_TIMER1);
               if(bSchedule_State)  //if we are currently in an 'On' period...
               {
                  bOn_Off = OFF;
                  Main_Mode.Tx_Mode = Tx_OntoOff;
                  bSchedule_State = OFF;
                  if(dlOff_Period_Minutes == 0)   // disable Scheduling if Repeat period <= On Period
                                                  // As per Bill M's e-mail 6/29/05 (point #4)
                    bScheduling = FALSE;
               }
               else // we are currently in an 'Off' period; switch to 'On' state
               {
#use delay(clock=32000000)
                  setup_oscillator(OSC_32MHZ);
                  bOn_Off = ON;
                  Main_Mode.Tx_Mode = Tx_OfftoOn_Vtx;
                  bSchedule_State = ON ;
                  if(lOn_Period_Minutes == 0)   // disable Scheduling if lOn_Period_Minutes is 0
						bScheduling = FALSE;	// As per Bill M's e-mail 6/29/05 (point #4)
               }  // end of 'if/else (bSchedule_State)'


               Change_ON_off = FALSE;//Reset Minute timer for next schedule change

               enable_interrupts(INT_TIMER1);   //re-enable

   //            configure_outputs();
               set_config_info();
               store_schedule_timer_in_EEPROM();

            } // end of 'if(dlSchedule_Minute_Timer <= 0)'

//            if(bStore_Timer_Flag)
//               store_schedule_timer_in_EEPROM();


         } // end of 'if(bScheduling)'

//         if(nMsg_Fail_Counter >= MAX_NUMBER_TRIES)
//            {
            return sleep_mode;
//            } 

    } //end of if( bStart_Marker == NORMAL)
    else // bStart_Marker == INVERTED
      {
       if(nFollow_On_TimeOut <= 0)
        {
         bStart_Marker = NORMAL;  // The wait for follow-on message has timed out go to sleep, wait for next tx
         return sleep_mode;
        }
       initial_RF_activity_search(); //Re-init for rest of long tx
       return Rx;
//       return sig_check; // Still waiting for follow-on message go to signal check
      }

    } // end of Schedule()


////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                    Switch Tx Power Supply ON
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
#if SAVE_VOLTAGES_TO_EEPROM==1
   int SAV_PS_voltage=0x70;
   int SAV_bat_voltage=0x80;
#endif

   char Tx_OfftoOn_Vtxf()
     {
      unsigned long long Vtxsquare;
      int i=0;
      unsigned long Vbatt_tenths_of_volts;

//   set_uart_speed(11100);


#if USING_PIC18LF26K22==1
      setup_adc_ports(sAN0|sAN1|sAN2 , VSS_VDD);  // AN5 is used to read C12
                                                  // A/D range is 0 - Vdd
#else
      setup_adc_ports(AN0_TO_AN2 | VSS_VDD);  // AN5 is used to read C12
                                                  // A/D range is 0 - Vdd
#endif
      setup_adc(ADC_CLOCK_DIV_32 | ADC_TAD_MUL_20); // A/D clock source period = 1us;
                                                        // Tacq = 18us [Rsource = 90 Kohms 
                                                        // Acquisition time = 20 x 1us = 20us (before conversion) (11% safety factor = 20/18)
                                                        // Measurement Time = 20 + (11 x 1) = 31us (sampling + conversion)
       set_adc_channel(2);  //set AD Channel 
       delay_msec(1);
       Vbat_ADC = read_adc(); //read the battery voltage
#if SAVE_VOLTAGES_TO_EEPROM==1
        write_eeprom(SAV_bat_voltage,Vbat_ADC);
        SAV_bat_voltage++;
        if (SAV_bat_voltage>0x8F) SAV_bat_voltage=0x80;
#endif
       GnSubData[BATTVOLT_BYTE] = ((Vbat_ADC>>2)&0xff)+4;        // read the battery voltage (circuit resistor ratio=.1, add 4 for .4v drop across diode)
//       if(GnSubData[BATTVOLT_BYTE]< 35) { // used to be 35
//       if(Vbat_ADC < 117) { // 117: 3.15 Volts
       if(Vbat_ADC < Vbat_min_voltage) { // 117: 3.15 Volts
//       if(Vbat_ADC < 108) { // 108: 2.9 Volts
#if SAVE_FLAG_OF_CURRENT_STATE==1
      if (read_eeprom(0x4a)!=0x18) {
        write_eeprom(0x4a,0x18);
     }
#endif
          return Tx_OfftoOn_Vtx; //re-initializes Tx when battery is too low
       }

#define NEW_VBATT_INITIALIZATION 0
#if NEW_VBATT_INITIALIZATION==1
       Vbatt_tenths_of_volts=GnSubData[BATTVOLT_BYTE];
       if (Vbatt_tenths_of_volts<50) {
         Vtx_PWM_Duty=240;
       } else if (Vbatt_tenths_of_volts<60) {
         Vtx_PWM_Duty=200;
       } else if (Vbatt_tenths_of_volts<70) {
         Vtx_PWM_Duty=137;
       } else if (Vbatt_tenths_of_volts<80) {
         Vtx_PWM_Duty=112;
       } else if (Vbatt_tenths_of_volts<	90) {
         Vtx_PWM_Duty=96;
       } else if (Vbatt_tenths_of_volts<100) {
         Vtx_PWM_Duty=76;
       } else {
         Vtx_PWM_Duty=10;
       }

    
#else
       Vtxsquare = 5031424/Vbat_ADC;
       Vtxsquare = Vtxsquare/Vbat_ADC;
       Vtx_PWM_Duty = Vtxsquare; 
       Vtx_PWM_Duty = Vtx_PWM_Duty+1705/Vbat_ADC+14; // Calculate the initial duty factor for the Tx power supply
 //      Vtx_PWM_Duty = 58254/Vbat_ADC;
#endif
 
     // Initialize Tx Power Supply PWM
//	   output_low(RIGHT_PIN);//This is a test to see if this affects mono on Actel
//	   output_high(LEFT_PIN);

//      output_low(REMOTE_RCVR_PWR_CTL); //need to force low before power applied,  no power control on Flyron version, commented out 7-12-17 MHPatten
      output_low(VTX_PWR_CTL); //need to force low before power applied,  no power control on Flyron version, commented out 7-12-17 MHPatten

      setup_timer_2(T2_DIV_BY_1,PR2,1);   // For 32MHz clk PR2 = 127, for pwm period of 62.5KHz
      set_pwm2_duty (Vtx_PWM_Duty);

#if SAVE_VTX_DUTY==1
      loByte=(char)(Vtx_PWM_Duty&0x0ff);
      hiByte=(char)((Vtx_PWM_Duty>>8)&0x0ff);
      if (read_eeprom(0x4b)!=hiByte) {
         write_eeprom(0x4b,hiByte);
      }
      if (read_eeprom(0x4c)!=loByte) {
         write_eeprom(0x4c,loByte);
      }
#endif

       // Turn on the Tx regulators
//      output_high(REMOTE_RCVR_PWR_CTL);  no power control on Flyron version, commented out 7-12-17 MHPatten
      output_high(VTX_PWR_CTL);
      setup_CCP2 (CCP_PWM);

// Zero variables used to compute running Duty Cycle average for power supply PWM

      Iavg = 0;
      Ipoint = 0;
      Sum_PWM_Duty = 0;
      for (i=0; i<11; i++) PWM_Duty_Array[i]=0;

//      return Tx_OfftoOn_tx; //for test
#if SAVE_FLAG_OF_CURRENT_STATE==1
      if (read_eeprom(0x4a)!=0x19) {
        write_eeprom(0x4a,0x19);
     }
#endif
      return Tx_OfftoOn_InitTx;
     }

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                    Initialize Tx to On State
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

   char Tx_OfftoOn_InitTxf()
      {
       unsigned long PS_Voltage;
       int pot_init_value = 0x00;

      delay_msec(100); //for test

       if (Vbat_ADC > Vbat_low)   //Check that battery voltage is greater than 3.3 volts (used to be "123" but measured values make me think 121 is closer to 3.3 V
//       if (Vbat_ADC > 121)   //Check that battery voltage is greater than 3.3 volts (used to be "123" but measured values make me think 121 is closer to 3.3 V
       {
#if USING_PIC18LF26K22==1
      setup_adc_ports(sAN0|sAN1|sAN2 , VSS_VDD);  // AN1 is used to read power supply
                                              // A/D range is 0 - Vdd
#else
      setup_adc_ports(AN0_TO_AN2 | VSS_VDD);  // AN1 is used to read power supply
                                              // A/D range is 0 - Vdd
#endif
        setup_adc(ADC_CLOCK_DIV_32 | ADC_TAD_MUL_20); // A/D clock source period = 1us;
                                                    // Tacq = 18us [Rsource = 90 Kohms 
                                                    // Acquisition time = 20 x 1us = 20us (before conversion) (11% safety factor = 20/18)
                                                    // Measurement Time = 20 + (11 x 1) = 31us (sampling + conversion)
        set_adc_channel(1);  //set AD Channel 
        delay_msec(1);
        PS_Voltage = read_adc();      // read the voltage   NB Must have a '#device adc=10' to force A/D to 10-bit operation 
#if SAVE_VOLTAGES_TO_EEPROM==1
        write_eeprom(SAV_PS_voltage,PS_voltage);
        SAV_PS_voltage++;
        if (SAV_PS_voltage>0x7F) SAV_PS_voltage=0x70;
#endif

#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
/* ---this was the code that was causing it not to regulate properly on turn-on 3-20-14 MPatten -------
       if (PS_Voltage > PS_VOLTAGE_MAX) {
          if (read_eeprom(0x4a)!=0x15) {
             write_eeprom(0x4a,0x15);
          }
           return Tx_OfftoON_InitTx; // Make sure Power supply voltage < max (currently 152 = 4.02 V)
       }
*/
#else
       if (PS_Voltage > 132) {
#if SAVE_FLAG_OF_CURRENT_STATE==1
      if (read_eeprom(0x4a)!=0x17) {
        write_eeprom(0x4a,0x17);
     }
#endif

          return Tx_OfftoON_InitTx; // Make sure Power supply voltage < 3.5v (was 131, but I think 132 is closer to 3.5 V)
       }
#endif

      //Increase Duty factor if voltage is less  than 2.9v
#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
      if (PS_Voltage < PS_Voltage_Target - VtxDeadZoneRadius) // For [6A, 88] and hysteresis=2, resulting voltages=[2.56,3.57}
#else	
      if (PS_Voltage < 109) // corresponds to 2.7 Volts
#endif
         {
         if (Vtx_PWM_Duty++ > MAX_VTX_PWM_DUTY) Vtx_PWM_Duty = MAX_VTX_PWM_DUTY;  // VpwmDuty > MAX_VTX_PWM_DUTY (493) is 100% duty factor
//          if (Vtx_PWM_Duty++ > 480) Vtx_PWM_Duty = 500;  // VpwmDuty > 450 is 100% duty factor for test
          set_pwm2_duty(Vtx_PWM_Duty);

#if SAVE_VTX_DUTY==1
      loByte=(char)(Vtx_PWM_Duty&0x0ff);
      hiByte=(char)((Vtx_PWM_Duty>>8)&0x0ff);
      if (read_eeprom(0x4b)!=hiByte) {
         write_eeprom(0x4b,hiByte);
      }
      if (read_eeprom(0x4c)!=loByte) {
         write_eeprom(0x4c,loByte);
      }

#endif

#define USE_NEW_STATE_FLOW_OFF2ON_INITTX 1
#if USE_NEW_STATE_FLOW_OFF2ON_INITTX!=1
#if SAVE_FLAG_OF_CURRENT_STATE==1
          if (read_eeprom(0x4a)!=0x01) {
             write_eeprom(0x4a,0x01);
          }
#endif

          return Tx_OfftoON_InitTx;
#endif
         } 
      //Decrease Duty factor if voltage is greater than 3.1v
#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
      if (PS_Voltage > PS_Voltage_Target + VtxDeadZoneRadius)  // For [6A, 88] and hysteresis=2, resulting voltages=[2.70,3.70}
#else
      if (PS_Voltage > 116)
#endif
         {
          if (Vtx_PWM_Duty-- < 1) Vtx_PWM_Duty = 0;  //VpwmDuty cannot be less than 0
          set_pwm2_duty(Vtx_PWM_Duty);

#if USE_NEW_STATE_FLOW_OFF2ON_INITTX!=1
#if SAVE_FLAG_OF_CURRENT_STATE==1
          if (read_eeprom(0x4a)!=0x11) {
             write_eeprom(0x4a,0x11);
          }
#endif
          return Tx_OfftoON_InitTx;
#endif
         } 
      }
      else
      {
      setup_CCP2 (CCP_OFF); //Turn on Tx Power Supply switch since battery voltage is less than 3.3v
      output_high(TX_PWR_PWM_PIN);
      Bat_Lo_Flag = TRUE;
      }

       set_adc_channel(1);  //set AD Channel 
       delay_usec(20);
       PS_Voltage = read_adc();      // read the voltage   NB Must have a '#device adc=10' to force A/D to 10-bit operation 

//#if SAVE_VOLTAGES_TO_EEPROM==1
//       write_eeprom(SAV_PS_voltage,PS_voltage);
//       SAV_PS_voltage++;
//       if (SAV_PS_voltage>0x7F) SAV_PS_voltage=0x70;
//#endif

#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
       if (PS_Voltage > PS_VOLTAGE_MAX) {

#if SAVE_FLAG_OF_CURRENT_STATE==1
      if (read_eeprom(0x4a)!=0x54) {
        write_eeprom(0x4a,0x54);
     }
#endif


          return Tx_OfftoON_Vtx; // Make sure Power supply voltage < max (currently 152 = 4.02 V)
#if USE_NEW_STATE_FLOW_OFF2ON_INITTX==1
       } else if (PS_Voltage<96) { // if Vtx less than 2.3 volts
#if SAVE_FLAG_OF_CURRENT_STATE==1
          if (read_eeprom(0x4a)!=0x27) {
            write_eeprom(0x4a,0x27);
          }

#endif
          return Tx_OfftoON_InitTx; // Make sure Power supply voltage < 3.5v (was 131, but I think 132 is closer to 3.5 V)
#endif
       }

#else
       if (PS_Voltage > 135) return Tx_OfftoON_Vtx; // Make sure Power supply voltage < 3.8v
#endif

       // Turn on the Tx regulators

//	   output_high(REMOTE_RCVR_PWR_CTL); 
	   output_high(VTX_PWR_CTL); 
       delay_msec(1);
       port_b_pullups(ON);

       set_pot(pot_init_value, POT0); //Initalize bias to min
       delay_msec(1);

	   Init_Synth(0,0);  //initialize the external Synthesize
       delay_msec(1);
	   remoteChannel_Set = remoteChannel_Select;
	   Channel = 7-remoteChannel_Select;
#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
       updatePSVoltageTarget();
#endif
	   Change_Frequency();  //set frequency to channel

#if AD_SOURCE_SET_BY_MCLR==0
       if (playOrLiveCmdFlag==1) {
          playOrLiveCmdFlag=0; // don;t turn on A/D if this is turning on as a result of a "Play" or "Live" special remote command
  	      Init_extAD(4,5,0);  //initialize the external A/D
       } else {
  	      Init_extAD(4,5,3);  //initialize the external A/D
       }
#else
       adSource=2; // force AD source to be reset
       readMclrSetADSource();
#endif

       set_data_speed_and_polarity(); // set the data speed and polarity in the FPGA

//       if (wasOffNowOnFlag==1 && wasLowPowerNowHighFlag==1) {
//          PS_Voltage_Target = (unsigned long)read_eeprom(PS_THRESHOLD_HIGHPOWER);
//          if (PS_Voltage_Target > PS_VOLTAGE_MAX) { // safety in case EEPROM has too high voltage setting (0xFF for example) 9-15-13 MPatten
//           PS_Voltage_Target = (unsigned long)0x71;  // set to value for low voltage
//            write_eeprom(PS_THRESHOLD_HIGHPOWER,(int8)(PS_Voltage_Target&0x0ff));
//          }
//       }
//       wasOffNowOnFlag=0;
//       wasLowPowerNowHighFlag=0;         

 
       enable_interrupts(INT_EXT); //enable FPGA SubClock interrupt
       enable_interrupts(INT_EXT1);   //enables external encoder

//Reset FPGA
 
	   output_high(LEFT_PIN); 
   	   output_high(RIGHT_PIN); 
	   delay_msec(2);
	   output_high(LEFT_PIN); 
   	   output_low(RIGHT_PIN); 
	   delay_msec(2);

  	   if ( GbAudio_Mode == STEREO)
	     {
	      output_high(LEFT_PIN); 
   	      output_high(RIGHT_PIN); 
	     }
	   else 
	     {
	      if (GbAudio_Left)
	        {
	        output_high(LEFT_PIN);
	        output_low(RIGHT_PIN);
	        }
  	      else
	        {
	         output_low(LEFT_PIN);
	         output_high(RIGHT_PIN);
             }
          }

//  Start the subclk as it stays high indicating the PIC should send Subchannel data
//  but the interrupt for sending the subchannel data is rising edge triggered.  To get the process
//  started a pulse is sent to the FPGA

         output_high(SUBDATA);
         output_low(SUBDATA);

#if SAVE_FLAG_OF_CURRENT_STATE==1
      if (read_eeprom(0x4a)!=0x12) {
        write_eeprom(0x4a,0x12);
     }
#endif
      return Tx_On_Chk_Adj_PS; //Start normal Tx On cycle 
     }        


////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                    Check and Adjust Power Supply Voltage
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

   char Tx_On_Chk_Adj_PSf()
     {

      long long Vtxsquare;
      unsigned long PS_Voltage;
      long PWM_Duty;
 
      if (EncoderA_Flag == TRUE) return Tx_Pot_Adj; //if pot needs adjusting do it

      //Read the Power Supply Voltage

#if USING_PIC18LF26K22==1
      setup_adc_ports(sAN0|sAN1|sAN2 , VSS_VDD);  // AN1 is used to read power supply
                                              // A/D range is 0 - Vdd
#else
      setup_adc_ports(AN0_TO_AN2 | VSS_VDD);  // AN1 is used to read power supply
                                              // A/D range is 0 - Vdd
#endif
      setup_adc(ADC_CLOCK_DIV_32 | ADC_TAD_MUL_20); // A/D clock source period = 1us;
                                                    // Tacq = 18us [Rsource = 90 Kohms 
                                                    // Acquisition time = 20 x 1us = 20us (before conversion) (11% safety factor = 20/18)
                                                    // Measurement Time = 20 + (11 x 1) = 31us (sampling + conversion)
      set_adc_channel(1);  //set AD Channel 
      delay_usec(200);
      PS_Voltage = read_adc();      // read the voltage   NB Must have a '#device adc=10' to force A/D to 10-bit operation 
#if SAVE_VOLTAGES_TO_EEPROM==1
        write_eeprom(SAV_PS_voltage,PS_voltage);
        SAV_PS_voltage++;
        if (SAV_PS_voltage>0x7F) SAV_PS_voltage=0x70;
#endif

//Check that Voltage is high enough to run Tx

//       if (Vbat_ADC < 116)  //If battery voltage is less than 3.1v turn switch on continuously
//       if (Vbat_ADC < 121)  //If battery voltage is less than 3.3v turn switch on continuously
       if (Vbat_ADC < Vbat_low)  //If battery voltage is less than 3.3v turn switch on continuously
        {
      setup_CCP2 (CCP_OFF); //Turn on Tx Power Supply switch since battery voltage is less than 3.3v
      output_high(TX_PWR_PWM_PIN);
      Bat_Lo_Flag = TRUE;
/*         disable_interrupt	s(INT_EXT);
	     disable_interrupts(INT_EXT1);
	     output_float(REMOTE_RCVR_PWR_CTL);
         setup_CCP2 (CCP_OFF); //Turn off Tx Power Supply

	     output_float(SDA); //These outputs requied to be low when TX off else blow LMX
	     output_float(AD_CHIP_SLCT);
	     output_float(SUBDATA);
	     output_float(RIGHT_PIN);
	     output_float(SCL);
	     output_float(WP);
	     output_float(LEFT_PIN);
	     output_float(SYNTH_LE);
         output_float(TX_PWR_PWM_PIN);
         port_b_pullups(OFF);
 
         return Tx_OfftoOn_Vtx; // Make sure Power supply voltage > 2.5v  */
        }
      if (!Bat_Lo_Flag)  //Check to see if PS switcher is in continuous mode
      {
      //Increase Duty factor if voltage is less  than 2.9v
#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
      if (PS_Voltage < PS_Voltage_Target + VtxDeadZoneRadius)
#else
      if (PS_Voltage < 126) 
#endif
         {
          if (Vtx_PWM_Duty++ > MAX_VTX_PWM_DUTY) Vtx_PWM_Duty = MAX_VTX_PWM_DUTY;  // VpwmDuty > MAX_VTX_PWM_DUTY (493) is 100% duty factor
//          if (Vtx_PWM_Duty++ > 480) Vtx_PWM_Duty = 500;  // VpwmDuty > 450 is 100% duty factor for test
//          set_pwm2_duty(Vtx_PWM_Duty);
         } 
      //Decrease Duty factor if voltage is greater than 3.1v
#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
      if (PS_Voltage > PS_Voltage_Target - VtxDeadZoneRadius)
#else
      if (PS_Voltage > 100)
#endif
         {
          if (Vtx_PWM_Duty-- < 1) Vtx_PWM_Duty = 0;  //VpwmDuty cannot be less than 0
         } 
/*
     PWM_Duty_Array[Ipoint] = Vtx_PWM_Duty;
     if (Ipoint == 10)
        Sum_PWM_Duty =Sum_PWM_Duty + Vtx_PWM_Duty - PWM_Duty_Array[0];
     else
        Sum_PWM_Duty =Sum_PWM_Duty + Vtx_PWM_Duty - PWM_Duty_Array[Ipoint+1];
     if (Iavg == 10)
       {
       if (Ipoint == 10)
         Ipoint = 0;
       else
         Ipoint++;
       }
     else 
       {
       Ipoint++;
       Iavg++;
       }
     PWM_Duty = Sum_PWM_Duty/Iavg;
     set_pwm2_duty(PWM_Duty);
*/
     set_pwm2_duty(Vtx_PWM_Duty);

#if SAVE_VTX_DUTY==1
      loByte=(char)(Vtx_PWM_Duty&0x0ff);
      hiByte=(char)((Vtx_PWM_Duty>>8)&0x0ff);
      if (read_eeprom(0x4b)!=hiByte) {
         write_eeprom(0x4b,hiByte);
      }
      if (read_eeprom(0x4c)!=loByte) {
         write_eeprom(0x4c,loByte);
      }
#endif

      }
      else
      {
      if (Vbat_ADC > 127)  //if Power Supply voltage > 3.5v then turn PWM back on (used to be 124 but I think 127 is closer to 3.5 v)
       {
//       Vtxsquare = 5031424/Vbat_ADC;
//       Vtxsquare = Vtxsquare/Vbat_ADC;
//       Vtx_PWM_Duty = Vtxsquare; 
//       Vtx_PWM_Duty = Vtx_PWM_Duty+1705/Vbat_ADC+14; // Calculate the initial duty factor for the Tx power supply
//         Vtx_PWM_Duty = 58254/Vbat_ADC;
      // Initialize Tx Power Supply PWM
         setup_timer_2(T2_DIV_BY_1,PR2,1);   // For 32MHz clk PR2 = 127, for pwm period of 62.5KHz
         set_pwm2_duty (Vtx_PWM_Duty);

#if SAVE_VTX_DUTY==1
      loByte=(char)(Vtx_PWM_Duty&0x0ff);
      hiByte=(char)((Vtx_PWM_Duty>>8)&0x0ff);
      if (read_eeprom(0x4b)!=hiByte) {
         write_eeprom(0x4b,hiByte);
      }
      if (read_eeprom(0x4c)!=loByte) {
         write_eeprom(0x4c,loByte);
      }
#endif


         setup_CCP2 (CCP_PWM);
         Bat_Lo_Flag = FALSE;
// return to initialize power, synth, and A/D

#if SAVE_FLAG_OF_CURRENT_STATE==1
      if (read_eeprom(0x4a)!=0x31) {
        write_eeprom(0x4a,0x31);
     }
#endif

         return Tx_OfftoOn_InitTx;
       }
      }

#if SAVE_FLAG_OF_CURRENT_STATE==1
      if (read_eeprom(0x4a)!=0x13) {
        write_eeprom(0x4a,0x13);
     }
#endif

      return Tx_On_MeasureBatV;

     }//End of Tx_On_Chk_Adj_PS
       


////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                    Measure Battery Voltage State
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

   char Tx_On_MeasureBatVf()
     {
     static unsigned int sec_count, Prev_Bit_Index ;

     if (EncoderA_Flag == TRUE) {

#if SAVE_FLAG_OF_CURRENT_STATE==1
       if (read_eeprom(0x4a)!=0x14) {
           write_eeprom(0x4a,0x14);
        }
#endif

        return Tx_Pot_Adj; //if pot needs adjusting do it
     }


   // Check once each second that the subclock is still happening

     if (nElapsed_Second!=sec_count)
       {
       sec_count = nElapsed_Second;
       if (Prev_Bit_Index == GnSubData_Bit_Index)
        {
         Prev_Bit_Index = GnSubData_Bit_Index;
         if (input(SUBCLK))  //Re-start Subchannel if ready
          {
           output_high(SUBDATA);
           output_low(SUBDATA);
          }
        }
       else Prev_Bit_Index = GnSubData_Bit_Index;

       } 

     //Now read the battery voltage for the appropriate SubData byte when voltage not currently sending value

     if (GnSubData_Byte_Index == 0)
       {
#if USING_PIC18LF26K22==0
          setup_adc_ports(AN0_TO_AN2 | VSS_VDD);  // AN2 is used to read battery
#else
          setup_adc_ports(sAN1|sAN2 , VSS_VDD);  // AN2 is used to read battery
#endif
                                                  // A/D range is 0 - Vdd
          setup_adc(ADC_CLOCK_DIV_32 | ADC_TAD_MUL_20); // A/D clock source period = 1us;
                                                        // Tacq = 18us [Rsource = 90 Kohms 
                                                        // Acquisition time = 20 x 1us = 20us (before conversion) (11% safety factor = 20/18)
                                                        // Measurement Time = 20 + (11 x 1) = 31us (sampling + conversion)
           set_adc_channel(2);  //set AD Channel 
           delay_usec(20);
           Vbat_ADC = read_adc(); //read the battery voltage
//#if SAVE_VOLTAGES_TO_EEPROM==1
//        write_eeprom(SAV_bat_voltage,Vbat_ADC);
//        SAV_bat_voltage++;
//        if (SAV_bat_voltage>0x8F) SAV_bat_voltage=80;
//#endif
           GnSubData[BATTVOLT_BYTE] = ((Vbat_ADC>>2)&0xff)+4;        // read the battery voltage (circuit resistor ratio=.1, add 4 for .4v drop across diode)
           if(GnSubData[BATTVOLT_BYTE]< Vbat_reset) reset_cpu(); //re-initializes Tx when battery is too low
//           if(GnSubData[BATTVOLT_BYTE]< 32) reset_cpu(); //re-initializes Tx when battery is too low
      }                                                       //NB Must have a '#device adc=10' to force A/D to 10-bit operation

#if SAVE_FLAG_OF_CURRENT_STATE==1
       if (read_eeprom(0x4a)!=0x22) {
           write_eeprom(0x4a,0x22);
        }
#endif

      return Tx_On_Chk_adj_PS;

     }//End of Tx_On_MeasureBatV
       

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                Tramnsmit On Change Mode
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////

   char Tx_On_Changef()

      {

// Added this reinitialization of synth after every channel change 3-12-14 MPatten
		   Init_Synth(0,0);  //initialize the external Synthesize
    	   delay_msec(1);

		   remoteChannel_Set = remoteChannel_Select;
		   Channel = 7-remoteChannel_Select;
		   Change_Frequency();
#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
           updatePSVoltageTarget();
#endif

  	    if ( GbAudio_Mode == STEREO)
	      {
	       output_high(LEFT_PIN); 
   	       output_high(RIGHT_PIN); 
	      }
	    else 
	      {
	       if (GbAudio_Left)
	         {
	         output_high(LEFT_PIN);
	         output_low(RIGHT_PIN);
	         }
  	       else
	         {
	          output_low(LEFT_PIN);
	          output_high(RIGHT_PIN);
	         }
	       }

//  Start the subclk as it stays high indicating the PIC should send Subchannel data
//  but the interrupt for sending the subchannel data is rising edge triggered.  To get the process
//  started a pulse is sent to the FPGA

         output_high(SUBDATA);
         output_low(SUBDATA);

      if (bON_OFF) {

#if SAVE_FLAG_OF_CURRENT_STATE==1
       if (read_eeprom(0x4a)!=0x51) {
           write_eeprom(0x4a,0x51);
        }
#endif

         return Tx_On_MeasureBatV;
      }  
      else {

#if SAVE_FLAG_OF_CURRENT_STATE==1
       if (read_eeprom(0x4a)!=0x61) {
           write_eeprom(0x4a,0x61);
        }
#endif

         return Tx_OntoOff;
      }
      } // End of Tx_On_Change()

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                    Tx On to Off State
//
////////////////////////////////////////////////////////////////////////////////////////////////////////


   char Tx_OntoOffF()
      {
       char txStatus;

	   disable_interrupts(INT_EXT);
	   disable_interrupts(INT_EXT1);


       txStatus=TXSTA;
       while ((txStatus&0x02)==0) {
                txStatus=TXSTA;   // wait for tx shift register clear
       }

//       write_eeprom(0x8e,SPBRG);
//       write_eeprom(0x8f,SPBRGH);
//       SPBRG=32;
//       SPBRGH=0;

//       output_low(REMOTE_RCVR_PWR_CTL); no power control on Flyron version, commented out 7-12-17 MHPatten
       output_low(VTX_PWR_CTL);
       setup_CCP2 (CCP_OFF); //Turn off Tx Power Supply

	   output_float(SDA); //These outputs requied to be low when TX off else blow LMX
	   output_float(AD_CHIP_SLCT);
	   output_float(SUBDATA);
	   output_float(RIGHT_PIN);
	   output_float(SCL);
	   output_float(WP);
	   output_float(LEFT_PIN);
	   output_float(SYNTH_LE);
       output_float(TX_PWR_PWM_PIN);
       port_b_pullups(OFF);
//	   output_float(REMOTE_RCVR_PWR_CTL);  no power control on Flyron version, commented out 7-12-17 MHPatten
       output_float(PIN_B7);

#if SAVE_FLAG_OF_CURRENT_STATE==1
       if (read_eeprom(0x4a)!=0x71) {
           write_eeprom(0x4a,0x71);
        }
#endif

       return Tx_Off; // Set the Tx state to off
      }

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                    Tx Program Digital Potentiometer Values
//
//                    Read Shaft encoder
//                    Adjust Values for Bias Adjustment per channel
//                    Adjust Values for Mod Index Adjustment per channel
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

   char Tx_Pot_AdjF()
      {

       BOOLEAN bIncrease,bChange_Pot = FALSE;   //Assume Increase = CW motion of encoder (Decrease = ACW)
       unsigned int nPrevious_Pot_Value, nEEPROM_Write_Address;

       disable_interrupts(INT_EXT1);

       delay_msec(ROT_ENC_DBNCE_DLY);

       if(!input(ENC_CH_A))	// ensure interrupt pin RB1 is still low (glitch avoidance, in combination with preceding delay)
         {
// Now using pin 20 for Vtx, not ENC_CH_B anymore 8/2/17 MPatten
//          if (input(ENC_CH_B)) 		bIncrease=TRUE;
//          else						bIncrease=FALSE;
			bIncrease=FALSE;

          if (input(F2_FN_SLCT)== BIAS)  //Disable for debug
//          if (POT_SLCT== BIAS)  //Enable for debug
            {
//           if (bIncrease && (GnCurrent_Ch_Bias < POT0_MAX))	// this way round, turning the encoder c/w DECREASES the current
             if (!bIncrease && (GnCurrent_Ch_Bias < POT0_MAX))	// this way round, turning the encoder c/w INCREASES the current
               {
                nPrevious_Pot_Value = GnCurrent_Ch_Bias;
                GnCurrent_Ch_Bias++;
                bChange_Pot=TRUE;
               }
//           if (!bIncrease && (GnCurrent_Ch_Bias != 0 ))	// this way round, turning the encoder anti-c/w INCREASES the current
             if (bIncrease && (GnCurrent_Ch_Bias != 0 ))	// this way round, turning the encoder anti-c/w DECREASES the current
               {
                nPrevious_Pot_Value = GnCurrent_Ch_Bias;
                GnCurrent_Ch_Bias--;
                bChange_Pot=TRUE;
               }

             if (bChange_Pot)

               if(!set_pot(GnCurrent_Ch_Bias,POT0)) // Comment out to accomodate lack of dig pot
                 GnCurrent_Ch_Bias = nPrevious_Pot_Value;  // if Potentiometer setting failed, restore old value

             if (!input(Program_Mode_Pin))
              {
               if (GnCurrent_Ch_Bias != GnCurrent_Ch_Bias_EEPROM)
                 {
         // SINGLE BIAS VALUE The following line has been commented out and the one below substituted as it was decided to have only
         // one bias setting for all frequencies

#ifdef COMMON_CHANNEL_BIAS
                  nEEPROM_Write_Address = BIAS_VALUE_BASE_ADDRESS;
#else
      if (Power_Setting==0) {
          nEEPROM_Write_Address = BIAS_VALUE_BASE_ADDRESS+Channel+8;   //read the Channel's Bias Pot setting
      } else {
          nEEPROM_Write_Address = BIAS_VALUE_BASE_ADDRESS+Channel;   //read the Channel's Bias Pot setting
      }
#endif
//                  write_eeprom(nEEPROM_Write_Address,GnCurrent_Ch_Bias);
                 }
              }
           } //end of 'if input(F2_FN_SLCT==BIAS)'


         else // F2_FN_SLCT=Modulation Index
            {

             if (bIncrease && (GnCurrent_Ch_ModIndx < POT1_MAX))
               {
                nPrevious_Pot_Value = GnCurrent_Ch_ModIndx;
                GnCurrent_Ch_ModIndx++;
                bChange_Pot=TRUE;
                }
             if (!bIncrease && (GnCurrent_Ch_ModIndx != 0 ))
               {
                nPrevious_Pot_Value = GnCurrent_Ch_ModIndx;
                GnCurrent_Ch_ModIndx--;
                bChange_Pot=TRUE;
               }

         	 if (bChange_Pot)
               if(!set_pot(GnCurrent_Ch_ModIndx, POT1))
               		GnCurrent_Ch_ModIndx = nPrevious_Pot_Value;  // if Potentiometer setting failed, restore old value

             if (!input(Program_Mode_Pin))
              {
              if (GnCurrent_Ch_ModIndx != GnCurrent_Ch_ModIndx_EEPROM)
                {
                 if (Power_Setting==0) {
                    nEEPROM_Write_Address = MODINDX_VALUE_BASE_ADDRESS + Channel+8;
                 } else {
                    nEEPROM_Write_Address = MODINDX_VALUE_BASE_ADDRESS + Channel;
                 }
//                 write_eeprom(nEEPROM_Write_Address,GnCurrent_Ch_ModIndx);  //Disable for no write to EEPROM
                }
              }
           } //end of 'else // F2_FN_SLCT=Modulation Index'
        } //end of 'if(!input(ENC_CH_A))'

      EncoderA_Flag = FALSE;
      clear_interrupt(INT_EXT1); //reenable the interrupt
      enable_interrupts(INT_EXT1);

#if SAVE_FLAG_OF_CURRENT_STATE==1
       if (read_eeprom(0x4a)!=0x81) {
           write_eeprom(0x4a,0x81);
        }
#endif

      return Tx_On_MeasureBatV;
      }



//==============================================================================================
char go_to_sleep()
{
#if SAVE_OSC_AND_PR2_IN_EEPROM==1
   if (eepromWriteNum<20) {
      testVal= *0x0FCB;
      write_eeprom(0x60+eepromWriteNum,0x02);
      write_eeprom(0x61+eepromWriteNum,testVal);
      eepromWriteNum++;
      eepromWriteNum++;
   }
#endif


   if (Main_Mode.Tx_Mode==0)
     {
#if TURN_OFF_TX_BIAS_PWM!=1
      setup_ccp1 (CCP_OFF);
#endif
      output_float(BIAS_SETTING_PIN);

#if DONT_CHANGE_CLOCK_SPEEDS==0
#use delay(clock=4000000)
      setup_oscillator(OSC_4MHZ);
#endif
#if TOGGLE_LSB_BIAS_PWM==1
   if (toggleLSBBiasPWM==1) {
      toggleLSBBiasPWM=0;
   } else {
      toggleLSBBiasPWM=1;
   }
#endif


      sleep();    //LISTEN_INTERVAL is multiples of 100ms
      setup_timer_2(T2_DIV_BY_1,PR2S,1);   // 4MHz clk PR2 = 15, for pwm period of 62.5KHz
     }

   if (nWakeup_Counter_100ms >= nListen_Interval)
      {
      nWakeup_Counter_100ms = 0;
      Charge_Time = 0;
      nListen_Interval = LISTEN_INTERVAL;  //ensure nListen_Interval is reset after a long sleep (ignoring non-target schedule data)
      return sig_check;
      }
  return schedule;
} // end of 'go_to_sleep()'
//==============================================================================================
void update_elapsed_time_counters()
// This function is necessary to keep accurate schedule time
{
   unsigned int  nCurrent_TMR1L;
   unsigned long lTime_Elapsed;
   signed int32 dTime_Elapsed;


   nCurrent_TMR1L = TMR1L;
   while(TMR1L == nCurrent_TMR1L);  // Wait here until Timer 1 Lo byte changes (max 31 us)

   lTime_Elapsed =  get_timer1() - TMR1_INIT;	// have to adjust because Timer 1 ISR initializes Timer 1 to F333
   dTime_Elapsed = lTime_Elapsed;

   while((dTime_Elapsed - (0x10000 - TMR1_INIT)) >= 0)	// if Time Elapsed > 100ms..
   {
       dTime_Elapsed -= (0x10000 - TMR1_INIT);			// Subtract 100ms from Time_Elapsed
       nElapsed_100ms_Cntr++;
   }

   // At this point dTime_Elapsed < 100ms
   set_timer1(TMR1_INIT + (unsigned long)dTime_Elapsed);      //Load Timer 1

   while(nElapsed_100ms_Cntr > 9)
   {
      nElapsed_100ms_Cntr -= 10; // decrement the 100ms by one second

      if(++nElapsed_Second > 59) // increment the elapsed counter and check for overflow
      {
         nElapsed_Second = 0;
         dlSchedule_Minute_Timer--;
         bStore_Timer_Flag = TRUE;

      } // end of 'if(++Elapsed_Sec_Cntr > 59)'
   } // end of 'while(nElapsed_100ms_Cntr > 9)'

} // end of 'update_elapsed_time_counters()'
//==============================================================================================
BOOLEAN initial_RF_activity_search()

// Collect for 5ms then test sample from min # of 1's, and against min and max zero-crossing thresholds
{
   BOOLEAN result;
   bLast_Received_Bit_Sample = FALSE;
   bLast_Hi_Speed_Bit_Sample = FALSE;

   nZero_Crossing_Count = 0;
   nOnes_Count = 0;
//   clear_squelch_counters();

   if((bStart_Marker == NORMAL) & (nMsg_Fail_Counter==0)) //Already enabled if receiving long packet or nMsg_Fail_Counter>0
      enable_RFM();

//      delay_ms(2); // This is how it is in Andy's version --- I think too soon 7-7-18 MPatten


   bSampling_Complete = FALSE;
   lFrame_Sample_Index = 0;	// reset
   nFrame_Number = 1;		// reset

   lSample_Byte_Index = SAMPLE_BUFFER_SIZE_BYTES -1;   //set to top byte of array
   nSample_Bit_Index = 7;

   lView_Period_Countdown = INITIAL_ACTIVITY_SEARCH_PERIOD;  //msd in 32768Hz clock cycles
//   lView_Period_Countdown = 64;  //msd in 32768Hz clock cycles

//   set_timer1(0);             //set Timer 1 to zero
   while((TMR1L & 0x07) !=0);  // Wait here until TMR1L = 0bxxxxx000

   nLast_Timer1_Lower_Byte = 0xFF;	// and initialize this for wait_8_32KHz cycles

   while (lView_Period_Countdown >= 0)
      sample_and_store();



  b5ms_Qualification_Failed = TRUE;   // first, set this flag to default

#if SAVE_SQUELCH_TEST_VALUES==1
   if (saveSquelchIndex>30) {
      saveSquelchIndex=0;
   }
   write_eeprom(0x90,saveSquelchIndex);
   write_eeprom(0x91,(int)(squelchSampleCount&0x0FF));
   write_eeprom(0x92,(int)((squelchSampleCount>>8)&0x0FF));
   squelchSampleCount=0;
   write_eeprom(0x70+saveSquelchIndex,nOnes_Count);
   saveSquelchIndex++;
   write_eeprom(0x70+saveSquelchIndex,nZero_Crossing_Count);
   saveSquelchIndex++;
#endif


	   //Now perform a Tach Squelch
//   if(!hi_speed_squelch_test())  // If true 5ms sample might be data and not noise

   result=hi_speed_squelch_test();
   if(!result&(Main_Mode.Rx_Mode==sig_check))  // If true 5ms sample might be data and not noise
     {
#if SAVE_SQUELCH_TEST_VALUES==1
   write_eeprom(0x70+saveSquelchIndex,1);
   saveSquelchIndex++;
   write_eeprom(0x70+saveSquelchIndex,0);
   saveSquelchIndex++;
#endif
      return(FALSE);
     } 
   else 
      {
       if((   nOnes_Count < MIN_NUM_ONES_IN_5MS_4xOVERSAMPLED)
	     || (nOnes_Count > MAX_NUM_ONES_IN_5MS_4xOVERSAMPLED)
        || (nZero_Crossing_Count > MAX_ZERO_XINGS_IN_5MS_4xOVERSAMPLED)
          || (nZero_Crossing_Count < MIN_ZERO_XINGS_IN_5MS_4xOVERSAMPLED)) {
#if SAVE_SQUELCH_TEST_VALUES==1	
   write_eeprom(0x70+saveSquelchIndex,2);
   saveSquelchIndex++;
   write_eeprom(0x70+saveSquelchIndex,0);
   saveSquelchIndex++;
#endif


              return(FALSE);
       }
       else
       b5ms_Qualification_Failed = FALSE;
      }
#if SAVE_SQUELCH_TEST_VALUES==1
   write_eeprom(0x70+saveSquelchIndex,3);
   saveSquelchIndex++;
   write_eeprom(0x70+saveSquelchIndex,0);
   saveSquelchIndex++;
#endif

   return(TRUE);	//

} // end of 'initial_RF_activity_search'
//==============================================================================================
void wait_8_32KHz_cycles()
{
	unsigned int nCurrent_Timer1_Lower_Byte, nCount_of_8;

	do
   {
	  nCurrent_Timer1_Lower_Byte = TMR1L;
      hi_speed_data_log();
   }
	while(nCurrent_Timer1_Lower_Byte == nLast_Timer1_Lower_Byte);

	do
	{
		nCurrent_Timer1_Lower_Byte = TMR1L;
		nCount_of_8 = nCurrent_Timer1_Lower_Byte & 0x07;
          hi_speed_data_log();

	}	while(nCount_of_8 != 0);

	nLast_Timer1_Lower_Byte = nCurrent_Timer1_Lower_Byte;
	lView_Period_Countdown -= 8;

} // end of 'wait_8_32KHz_cycles'
//==============================================================================================
#inline void hi_speed_data_log()
{
   BOOLEAN bCurrent_Received_Bit_Sample;


   bCurrent_Received_Bit_Sample = input(RCVR_DATA_IN_PIN);


   lHi_Speed_Sample_Counter++;

   if(bCurrent_Received_Bit_Sample)
      {
      lHi_Speed_Ones_Counter++;
      }
   else

   if(bCurrent_Received_Bit_Sample != bLast_Hi_Speed_Bit_Sample)
      lHi_Speed_0_Xing_Counter++;

   bLast_Hi_Speed_Bit_Sample = bCurrent_Received_Bit_Sample;


} // end of 'hi_speed_data_log()'
//==============================================================================================
void clear_squelch_counters()
{
//   nZero_Crossing_Count = 0;
//   nOnes_Count = 0;
   lHi_Speed_Sample_Counter = 0;
   lHi_Speed_Ones_Counter = 0;
   lHi_Speed_0_Xing_Counter = 0;

} // end of 'clear_hi_speed_counters()'
//==============================================================================================
BOOLEAN sample_and_store()
// Stores every sample as separate bit, so that different #'s frame can be examined
// NB: At end of period Byte 0, Bit 0 of nSample_Buffer holds the last sample taken
{
   BOOLEAN bCurrent_Received_Bit_Sample, bFinished = FALSE;

	//restart_wdt();

#if SAVE_SQUELCH_COUNTERS==1
   write_eeprom(0x80,make8(lHi_Speed_Sample_Counter,0));
   write_eeprom(0x81,make8(lHi_Speed_Sample_Counter,1));
   write_eeprom(0x82,make8(lHi_Speed_Ones_Counter,0));
   write_eeprom(0x83,make8(lHi_Speed_Ones_Counter,1));
   write_eeprom(0x84,make8(lHi_Speed_0_Xing_Counter,0));
   write_eeprom(0x85,make8(lHi_Speed_0_Xing_Counter,1));
#endif


   wait_8_32KHz_cycles();   // On first pass, this may take anywhere between 183-275us, but that's OK
                                                      // (because don't know when crystal edge is coming)
   bCurrent_Received_Bit_Sample = input(RCVR_DATA_IN_PIN);

   if(bCurrent_Received_Bit_Sample)
     {
         bit_set(nSample_Buffer[lSample_Byte_Index],nSample_Bit_Index);
     }
   else
     {
		bit_clear(nSample_Buffer[lSample_Byte_Index],nSample_Bit_Index);
     } 
	//Now update the buffer bit pointer to the next bit DOWN the array
   if(--nSample_Bit_Index < 0)
   {
      nSample_Bit_Index = 7;
      if(--lSample_Byte_Index < 0) bFinished = TRUE;
   }

   if(bCurrent_Received_Bit_Sample != bLast_Received_Bit_Sample)
         nZero_Crossing_Count++;

   if(bCurrent_Received_Bit_Sample)
         nOnes_Count++;

#if SAVE_SQUELCH_TEST_VALUES==1
   if (Main_Mode.Rx_Mode==sig_check) {
      squelchSampleCount++;
   }
#endif

   bLast_Received_Bit_Sample = bCurrent_Received_Bit_Sample;

   lFrame_Sample_Index++;	// initialized to 1, in initial_RF_activity_search
   if(lFrame_Sample_Index >= FRAME_SIZE_SAMPLES)
   {
		lFrame_Sample_Index = 1;
		nFrame_Number++;	// initialized to 1, in initial_RF_activity_search
   }
/*   if(Main_Mode.Rx_Mode==Rx)
     {
    if (lHi_Speed_Sample_Counter = 3000)
        clear_squelch_counters();
    if (lHi_Speed_Sample_Counter > 4000)
       adjust_bias_voltage();
//       I8XSample=0;
//       }
//     else
//     I8XSample++;
     }*/
   // check to see if we're at the end of the multi-frame sampling period

   if(bFinished)
         return(True);  // Yes we are
   else  return(False); // No we ain't!

} // end of store_sample()
//==============================================================================================
BOOLEAN in_message_squelch_test()
// Checks each block of 8 encoded bits (7.8ms @1024bps) for conformance to tach squelch specifications
{
   BOOLEAN bResult;

   if(!hi_speed_squelch_test())
      bResult = FALSE;           // if test fails, return false

   else
   {
      if((   nOnes_Count < MIN_NUM_ONES_IN_8_4xOVERSAMPLED_MESSAGE_BITS)             // if any of these tests FAIL
	     || (nOnes_Count > MAX_NUM_ONES_IN_8_4xOVERSAMPLED_MESSAGE_BITS)
        || (nZero_Crossing_Count > MAX_ZERO_XINGS_IN_8_4xOVERSAMPLED_MESSAGE_BITS)
          || (nZero_Crossing_Count < MIN_ZERO_XINGS_IN_8_4xOVERSAMPLED_MESSAGE_BITS))
            bResult = FALSE;                                // Return FALSE
      else bResult = TRUE;                                  // else Return TRUE
   }

   return(bResult);
} // end of in_message_squelch_test()
//==============================================================================================
BOOLEAN hi_speed_squelch_test()
/* This function discriminates large #'s of zero crossings

   It returns FALSE (i.e. signal is noise only) if it detects the above condition
   It returns TRUE if the # of zero crossings is low

   lHi_Speed_Sample_Counter = K.#bits
                        where K = #samples/bit
   Max # of data crossings  = Dx =(#bits+1)

   Therefore:

   lHi_Speed_Sample_Counter / Max # of data crossings = ~= K i.e. largely independent of # of samples

   lHi_Speed_Sample_Counter / K ~= Max # of data crossings

   Set max # crossings = lHi_Speed_Sample_Counter. y/K

   For fast computation (no division necessary), set the product y/K to a power of 2

   // In this implementation K = 40; let y/K = 1/8 i.e. y = 5

*/
{
#if DONT_USE_HI_SPEED_QUALIFIER==1
   return(TRUE);
#else
//	if(lHi_Speed_0_Xing_Counter > (lHi_Speed_Sample_Counter/8))
	if(lHi_Speed_0_Xing_Counter > (lHi_Speed_Sample_Counter/32))  //increased CPU clock by 4 times
//	if(lHi_Speed_0_Xing_Counter > (lHi_Speed_Sample_Counter/4))  //decreased CPU clock by 1/2 
		return(FALSE);	// failed the test - signal is noise
	else
         return(TRUE);	// passed test - signal may be data
#endif
} // end of 'hi_speed_squelch_test()'
//==============================================================================================

void phase_lock_to_sample_stream()
{
   unsigned long  lZero_Xing_Counts[OVERSAMPLING_RATE] = {0,0,0,0};
   unsigned long  lByte_Index, lStart_Frame_Byte_Index;
   unsigned int   nWindow_Bit_Index, nTemp, nPhase_Lock_Shift_Index, nCount;
   signed int     nBit_Index, nStart_Bit;

   unsigned long  lEnd_Sample_Number, lSample_Number;
   BOOLEAN        bBit0=FALSE, bBit1;

   //restart_wdt();

   //First calculate the initial Sample Buffer offset for the current Start Frame
   lStart_Frame_Byte_Index = (SAMPLE_BUFFER_SIZE_BYTES-1) - ((long)(nStart_Frame_Number - 1) * SAMPLE_BYTES_PER_FRAME);


   for(nStart_Bit = 0; nStart_Bit < OVERSAMPLING_RATE; nStart_Bit++)   //loop through all possible start positions
   {
      lEnd_Sample_Number = (long)nStart_Bit + (FRAME_SIZE_SAMPLES-1) - OVERSAMPLING_RATE; //calculate the relative (w.r.t. single frame) final bit(+1) # for following loop
																// - OVERSAMPLING_RATE term to avoid overshoot when analyzing last frame only

      //lSample_number max range 0 to (FRAME_SIZE_SAMPLES-1)
      for(lSample_Number = nStart_Bit; lSample_Number <= lEnd_Sample_Number; lSample_Number+=OVERSAMPLING_RATE) //loop through successive test windows
      {
		//Calculate the start position in the sample buffer, in TWO steps

		 lByte_Index = lStart_Frame_Byte_Index - (lSample_Number/8);

         nBit_Index =  ((FRAME_SIZE_SAMPLES - 1) - lSample_Number);	//NB result loses top byte of long result but it doesn't matter
         nBit_Index &= 0x07;

         // Calculate # of bit transitions in this test window

         bBit0 = bit_test(nSample_Buffer[lByte_Index], nBit_Index);	// Get the first bit

         for(nWindow_Bit_Index=1; nWindow_Bit_Index < OVERSAMPLING_RATE; nWindow_Bit_Index++)
         {
            if(--nBit_Index < 0)
            {
               lByte_Index--;
               nBit_Index = 7;
            }

            bBit1 = bit_test(nSample_Buffer[lByte_Index], nBit_Index);

            if(bBit1 != bBit0)
               lZero_Xing_Counts[nStart_Bit]++;

            bBit0 = bBit1;	//now set last bit = current bit
         } // end of 'for(nWindow_Bit_Index=0; nWindow_Bit_Index < OVERSAMPLING_RATE; nWindow_Bit_Index++)'
      } // end of 'for(lSample_Bit = nStart_Bit, lSample_Bit < lEnd_Sample_bit, lSample_Bit+=OVERSAMPLING_RATE)'
   } // end of 'for(nStart_Bit = 0, nStart_Bit < OVERSAMPLING_RATE), nStart_Bit++)'

   // NOW DETERMINE PHASE LOCK (BY MINIMUM ZERO CROSSINGS) AND ROTATE AVERAGED SAMPLE BUFFER

   //Determine optimal phase
   nTemp = lZero_Xing_Counts[0];
   nPhase_Lock_Shift_Index = 0;

   for(nBit_Index=1; nBit_Index < OVERSAMPLING_RATE; nBit_Index++)
      if(lZero_Xing_Counts[nBit_Index] < nTemp)
      {
         nTemp = lZero_Xing_Counts[nBit_Index];
         nPhase_Lock_Shift_Index = nBit_Index;
      }

   // rotate Sample Buffer to lock phase

//	HAVE TO REPLACE FOLLOWING ROTATE BECAUSE OF COMPILER ERROR.
//   for(nBit_Index=0; nBit_Index < nPhase_Lock_Shift_Index; nBit_Index++)
//      rotate_left(nSample_Buffer,SAMPLE_BUFFER_SIZE_BYTES);

	// Following routine is a substitute rotation routine.

	for(nCount=0; nCount < nPhase_Lock_Shift_Index; nCount++)
	{
		nBit_Index = 6;	// YES, 6!
		lByte_Index = SAMPLE_BUFFER_SIZE_BYTES - 1;
		bBit0 = 0;

		if(bit_test(nSample_Buffer[SAMPLE_BUFFER_SIZE_BYTES - 1],7)) bBit0 = 1;	//first store most sig. bit in array

		for(lSample_Number = 0; lSample_Number < MAX_SAMPLE_COUNT; lSample_Number++)
		{
			if(bit_test(nSample_Buffer[lByte_Index],nBit_Index))
			{
				if(nBit_Index == 7)
					bit_set(nSample_Buffer[lByte_Index+1],0);
				else bit_set(nSample_Buffer[lByte_Index],(nBit_Index+1));
			}
			else // tested bit is 0
				if(nBit_Index == 7)
					bit_clear(nSample_Buffer[lByte_Index+1],0);
				else bit_clear(nSample_Buffer[lByte_Index],(nBit_Index+1));

			if(--nBit_Index < 0)
			{
				nBit_Index = 7;
				lByte_Index--;
			}
		}

		bit_clear(nSample_Buffer[0],0);	// set least sig bit in array
		if(bBit0)
			bit_set(nSample_Buffer[0],0);

	} // end of 'for(nBit_Index=0; nBit_Index < nPhase_Lock_Shift_Index; nBit_Index++)	'
} // end of 'phase_lock_to_sample_stream()'
//==============================================================================================
void determine_encoded_bit_stream()
// collapses the nAveraged_Sample_Buffer from samples to bit values
// result stored in nAveraged_Sample_Buffer
// NB:  At the end, the first encoded bit is located at bit 7 of the top of the nAveraged_Sample_Buffer array
//      i.e. if the array reads from right to left, with LSB left, the first (oldest) sample taken is on the RIGHT

{
   unsigned int   nBit_Sample_Index, nEncoded_Byte_Index, nEncoded_Bit_Index, nEncoded_Bit;
   unsigned int   nThreshold, nSample_Sum;
   signed	int   nBuffer_Bit_Index;

   unsigned long  lOffset;
   unsigned long  lSample_Buffer_Byte_Index;

   //restart_wdt();

   nThreshold = (nNumber_of_Frames_to_Analyze*OVERSAMPLING_RATE)/2; //set threshold for determination of 1 or 0 from over-samples

	//for each encoded bit
	for(nEncoded_Bit_Index = 0; nEncoded_Bit_Index < ENCODED_MESSAGE_SIZE_BITS; nEncoded_Bit_Index++)
	{
		nEncoded_Byte_Index = (ENCODED_MESSAGE_SIZE_BYTES - 1) - (nEncoded_Bit_Index/8);
		nEncoded_Bit  = 7 - (nEncoded_Bit_Index & 0x07);

		nSample_Sum = 0;

		// for each successive frame
		for(nFrame_Number = nStart_Frame_Number; nFrame_Number <= nNumber_of_Frames_to_Analyze; nFrame_Number++)
		{
	    	//first establish frame's start byte in sample buffer
			lSample_Buffer_Byte_Index = (SAMPLE_BUFFER_SIZE_BYTES-1) - ((long)(nFrame_Number-1)*SAMPLE_BYTES_PER_FRAME);

        	 //then offset further by encoded bit (group of 4) position within required frame....
			lOffset = ((long)nEncoded_Bit_Index)*OVERSAMPLING_RATE;	// calculate the relative offset within the frame...
			lSample_Buffer_Byte_Index -= lOffset/8;					// ...then calculate the absolute index

			nBuffer_Bit_Index = (lOffset-1) & 0x0007;

			//sum consecutive samples of the same bit
			for(nBit_Sample_Index = 0; nBit_Sample_Index < OVERSAMPLING_RATE; nBit_Sample_Index++)
			{
				if(bit_test(nSample_Buffer[lSample_Buffer_Byte_Index],nBuffer_Bit_Index))
				nSample_Sum++;

				if(--nBuffer_Bit_Index < 0)
				{
					nBuffer_Bit_Index = 7;
					lSample_Buffer_Byte_Index--;
				}
			} // end of 'for(nBit_Sample_Index = 0; nBit_Sample_Index < OVERSAMPLING_RATE; nBit_Sample_Index++)'
		}	// end of 'for(nFrame_Number = nStart_Frame_Number; nFrame_Number <= nNumber_of_Frames_to_Analyze; nFrame_Number++)'

		if(nSample_Sum >= nThreshold)
			   bit_set(nAveraged_Sample_Buffer[nEncoded_Byte_Index],nEncoded_Bit);
		else bit_clear(nAveraged_Sample_Buffer[nEncoded_Byte_Index],nEncoded_Bit);
	} // end of 'for(nEncoded_Bit_Index = 0; nEncoded_Bit_Index < ENCODED_MESSAGE_SIZE_BITS; nEncoded_Bit_Index++)'

} // end of 'determine_encoded_bit_stream()'
//==============================================================================================
BOOLEAN locate_start_pattern()
// rotate nAveraged_Sample_Buffer (over # encoded bytes, not full buffer size) so Start Pattern is 'oldest' data

{
   unsigned int  nCount, nStart_Marker_MSByte, nStart_Marker_LSByte;
//   BOOLEAN      bBit_Shifted_Out;

   //restart_wdt();

   if(bStart_Marker == NORMAL)
   {
      nStart_Marker_MSByte = START_FRAME_MARKER_MSByte;
      nStart_Marker_LSByte = START_FRAME_MARKER_LSByte;
   }
   else // bStart_Marker == INVERTED
   {
      nStart_Marker_MSByte = START_FRAME_MARKER_MSByte ^ 0xFF;
      nStart_Marker_LSByte = START_FRAME_MARKER_LSByte ^ 0xFF;
   }

   for(nCount = 0; nCount < ENCODED_MESSAGE_SIZE_BITS; nCount++)
   {
      if((nAveraged_Sample_Buffer[ENCODED_MESSAGE_SIZE_BYTES-1] == nStart_Marker_MSByte)
       && (nAveraged_Sample_Buffer[ENCODED_MESSAGE_SIZE_BYTES-2] == nStart_Marker_LSByte))
	        return(TRUE);
      else
      {
// FOLLOWING COMMENT-OUTs ONLY NEED UN-COMMENTING (AND TESTING!) IF MESSAGE SIZE IS NOT A WHOLE NUMBER OF BYTES
//         if(nLast_Encoded_Bit_BitIndex == 0)
          rotate_left(nAveraged_Sample_Buffer,ENCODED_MESSAGE_SIZE_BYTES);
//         else
//         {
//            bBit_Shifted_Out = shift_left(nAveraged_Sample_Buffer,(ENCODED_MESSAGE_SIZE_BYTES+1),0);
//            if(bBit_Shifted_Out)
//                   bit_set(nAveraged_Sample_Buffer[nLast_Bit_Byte_Index],nLast_Encoded_Bit_BitIndex);
//            else bit_clear(nAveraged_Sample_Buffer[nLast_Bit_Byte_Index],nLast_Encoded_Bit_BitIndex);
//         }
      }
   } // end of 'for(nCount = 0; nCount < ENCODED_MESSAGE_SIZE_BITS; nCount = 0++)'

   return(FALSE); // if code gets here there was no match to the start pattern
} // end of 'locate_start_pattern()'
//==============================================================================================
BOOLEAN decode_data()
// Decoded data is returned in nAveraged_Sample_Buffer
// MSbit in Array is first bit of (post Start Framer Marker) decoded data
{
   unsigned int   nDecoded_Data_Byte_Index, nEncoded_Data_Byte_Index;
   signed int     nDecoded_Data_Bit_Index, nEncoded_Data_Bit_Index;

   unsigned int   nSix_Bit_Encoded_Nibble, nCount;
   signed int     nData_Nibble, nBit_Count;

   //restart_wdt();

   nDecoded_Data_Byte_Index = ENCODED_MESSAGE_SIZE_BYTES - 1;    //result will overwrite (top down) nAveraged_Sample_Buffer..
   nDecoded_Data_Bit_Index = 7;

   //first, move past two-byte start marker
   nEncoded_Data_Byte_Index = (ENCODED_MESSAGE_SIZE_BYTES - 1 - NO_OF_BYTES_IN_START_FRAME);
   nEncoded_Data_Bit_Index = 7;

   for(nCount=0; nCount < NO_OF_6B_IN_MESSAGE; nCount++)
   {
	  nSix_Bit_Encoded_Nibble = 0;

      for(nBit_Count = 5; nBit_Count >= 0; nBit_Count--) // read the 6-bit encoded nibble
      {
         if(bit_test(nAveraged_Sample_Buffer[nEncoded_Data_Byte_Index],nEncoded_Data_Bit_Index))
                bit_set(nSix_Bit_Encoded_Nibble,nBit_Count);
         else bit_clear(nSix_Bit_Encoded_Nibble,nBit_Count);

            if(--nEncoded_Data_Bit_Index < 0)   //decrement the Sample Buffer pointer pair to the next encoded bit
            {
               nEncoded_Data_Bit_Index = 7;
               nEncoded_Data_Byte_Index--;
            }
      }

      nData_Nibble = nDecode6Bto4B[nSix_Bit_Encoded_Nibble];

      if(nData_Nibble < 0)
         return(FALSE);     //the 6-bit value was illegal, abandon this N-frame detection
      else
      {
         for(nBit_Count = 3; nBit_Count >= 0; nBit_Count--)
         {
            if(bit_test(nData_Nibble,nBit_Count))
                   bit_set(nAveraged_Sample_Buffer[nDecoded_Data_Byte_Index],nDecoded_Data_Bit_Index);
            else bit_clear(nAveraged_Sample_Buffer[nDecoded_Data_Byte_Index],nDecoded_Data_Bit_Index);

            if(--nDecoded_Data_Bit_Index < 0)   //decrement the Sample Buffer pointer pair for the next test
            {
               nDecoded_Data_Bit_Index = 7;
               nDecoded_Data_Byte_Index--;
            }
         } // end of 'for(nBit_Count = 3, nBit_Count >= 0, nBit_Count--)'
      } // end of 'else'
   } // end of 'for(nCount=0; nCount < NO_OF_6B_IN_MESSAGE; nCount++)'

   return(TRUE);  // if program gets here all 4b6b decodes were successful
} // end of 'decode_data()'
//==============================================================================================
BOOLEAN verify_customer_ID()
// When this is called the Start Marker has been stripped and the Switch/Receiver starts at the top of
// the nAveraged_Sample_Buffer array
{
   unsigned long lCustomer_ID;
   unsigned int  nCustomer_ID;

   lCustomer_ID = extract_data_from_message(CUSTOMER_ID_OFFSET_NIBBLE_INDEX); // returns a 16-bit result
                                                                              // only upper bits wanted
   nCustomer_ID = swap(make8(lCustomer_ID,1));  // select the upper 8 bits, swap to make nibble order consistent with that received (least sig first)

   if(nCustomer_ID == CUSTOMER_ID)
		return(TRUE);

   else return(FALSE);
} // end of 'verify_customer_ID()'
//==============================================================================================
BOOLEAN verify_receiver_ID()
{
   unsigned long lReceiver_ID;
   unsigned int nMS_Char, nNS_Char, nLS_Char;

   lReceiver_ID = extract_data_from_message(RECEIVER_ID_OFFSET_NIBBLE_INDEX); // returns a 16-bit result

   rotate_right(&lReceiver_ID,2);   //move the result down 4 bits...
   rotate_right(&lReceiver_ID,2);
   rotate_right(&lReceiver_ID,2);
   rotate_right(&lReceiver_ID,2);

   lReceiver_ID &= 0x0FFF;          //... and clear the upper nibble

#ifndef ADDR_MS_FIRST   	// this routine swaps the received uppermost and lowermost nibbles of the 3 nibble address
							// only present if the Address is specified as being least least significant character first
	nLS_Char = make8(lReceiver_ID,1);			// nLS_Char = 0x0* where * is the LEAST significant character
	nMS_Char = (make8(lReceiver_ID,0)) & 0x0F;	// nMS_Char = 0x0* where * is the MOST significant character

	nNS_Char = (lReceiver_ID & 0x00F0);			// clear all except the centre nibble of the 3 nibble address

	lReceiver_ID = make16(nMS_Char, (nNS_Char | nLS_Char));	// recompose lReceiver_ID to ( nibbles order, most sig. first) 0/MOST/NEXT/LEAST

	swap(nNS_Char);		// now swap middle character in nNS_Char from upper to lower byte // (for later routine)

#else
	nLS_Char = (make8(lReceiver_ID,0)) & 0x0F;
	nNS_Char = (make8(lReceiver_ID,0)) & 0xF0;
	swap(nNS_Char);
	nMS_Char = make8(lReceiver_ID,1);			// nLS_Char = 0x0* where * is the LEAST significant character
#endif

#ifdef RCVR_ADDR_IN_DECIMAL // If received address is in Decimal, redefine lReceiver_ID
	lReceiver_ID = nLS_Char;
	lReceiver_ID += ((long)nNS_Char*10);
	lReceiver_ID += ((long)nMS_Char*100);
#endif

#ifdef 999_EQ_ALL_RXS	// NB: 999_EQ_ALL_RXS and FFF_EQ_ALL_RXS defines are mutually exclusive - see AFP_Switch_Config_Options.h
   if(lReceiver_ID == 999) return(TRUE);
#endif

#ifdef FFF_EQ_ALL_RXS	// NB: 999_EQ_ALL_RXS and FFF_EQ_ALL_RXS defines are mutually exclusive - see AFP_Switch_Config_Options.h
	if(lReceiver_ID == 0xFFF) return(TRUE);
#endif

   if(lReceiver_ID == RECEIVER_ID)
		return(TRUE);

   else
		return(FALSE);

} // end of 'verify_receiver_address'
//==============================================================================================
void determine_io()
{
   unsigned long  lData;

   //restart_wdt();

   lData = extract_data_from_message(LOGIC_MODE_CONTROL_NIBBLE_INDEX); // returns a 16-bit result
                                                                       // only upper bits wanted
   nRequired_IO_Direction = make8(lData,1);  // extract the upper byte of the result

} // end of 'determine_io'
//==============================================================================================
void determine_output_control()
{
   unsigned int nCount;
   unsigned long lData1thru4, lData5thru7;

   //restart_wdt();

   lData1thru4 = extract_data_from_message(LINE_1_OUTPUT_CONTROL_NIBBLE_INDEX); // returns a 16-bit result
   lData5thru7 = extract_data_from_message(LINE_1_OUTPUT_CONTROL_NIBBLE_INDEX+4); // returns a 16-bit result

   nRequired_Output_Control[0] = make8(lData1thru4,1); // Line 1 data is upper nibble, Line 2 is lower nibble)
   nRequired_Output_Control[1] = make8(lData1thru4,0); // Line 3 data is upper nibble, Line 4 is lower nibble)
   nRequired_Output_Control[2] = make8(lData5thru7,1); // Line 5 data is upper nibble, Line 6 is lower nibble)
   nRequired_Output_Control[3] = make8(lData5thru7,0); // Line 7 data is upper nibble

   for(nCount=0; nCount < 4; nCount++)
      swap(nRequired_Output_Control[nCount]); // Swap nibbles so lower Line Number is in lower nibble of each byte

   // Now nRequired_Output_Control[0] low  nibble contains required Line 1 Control
   //   & nRequired_Output_Control[3] high nibble contains required Line 7 Control
   //   ...."and all points in between"
}
//==============================================================================================
//==============================================================================================
void determine_actions()
// extracts Action instructions, sets flags, and re-writes the Configuration State in EEPROM if
// it has changed as a result.
   {

   unsigned int nAction_Nibble;

   nAction_Nibble = get_action_nibble();
   if(!bit_test(nAction_Nibble,ACTIVATE_SCHEDULE_FLAG))
      {
      bLong_Message_Flag = FALSE;
      bScheduling = OFF;   // NB bScheduling is only set when valid schedule data has been received

      if(bit_test(nAction_Nibble,ON_OFF_FLAG) || flyronMode==FLYRON_MODE_RECORDING) {
         bOn_Off = ON;

#if AD_SOURCE_SET_BY_MCLR==0
         if (playOrLiveCmdFlag==1) {
            playOrLiveCmdFlag=0; // don;t turn on A/D if this is turning on as a result of a "Play" or "Live" special remote command
  	        Init_extAD(4,5,0);  //initialize the external A/D
         } else {
    	      Init_extAD(4,5,3);  //initialize the external A/D
         }
#else
       adSource=2; // force AD source to be reset
       readMclrSetADSource();
#endif

//         wasOffNowOnFlag==0;
         if (Main_Mode.Tx_Mode==0) {
//            wasOffNowOnFlag==1;
            Main_Mode.Tx_Mode=Tx_OfftoOn_Vtx; //Startup Tx
         }

         if (flyronMode==FLYRON_MODE_RECORDING) { // don't turn off if recording...
            picOnRegsOffMode=1;
            output_low(VTX_PWR_CTL);
            output_low(AD_CHIP_SLCT);
         }

      }  else {
         picOnRegsOffMode=0;
         bOn_Off = OFF;
         if (Main_Mode.Tx_Mode>0) Main_Mode.Tx_Mode=Tx_OntoOff;  //Turn off TX
      }

      if(bit_test(nConfig,STORED_SCHEDULE_STATUS_BIT))   // if Schedule is currently actively, and received command
         {
          bOn_Off = OFF;                               // deactivates it, set outputs to OFF (override previous 'if..'
          if (Main_Mode.Tx_Mode>0) Main_Mode.Tx_Mode=Tx_OntoOff;  //Turn off TX
         }
     }                                                  // (as per Bill M e-mail 6-29-05, 'Finer Points' #3)
   else // ..ACTIVATE_SCHEDULE_FLAG of nAction_Nibble is True
   		bLong_Message_Flag = TRUE;

   set_config_info();

} // end of 'determine_actions()'
//==============================================================================================
BOOLEAN is_long_msg_for_other_unit_imminent()
{
   unsigned int nAction_Nibble;

   nAction_Nibble = get_action_nibble();

   return 	(bit_test(nAction_Nibble,ACTIVATE_SCHEDULE_FLAG));
} // end of 'is_long_msg_for_other_unit_imminent()'
//==============================================================================================
unsigned int get_action_nibble()
{
   unsigned long 	lData;
   unsigned int   nAction_Nibble;

	lData = extract_data_from_message(ACTION_STATUS_NIBBLE_INDEX); // returns a 16-bit result - only need uppermost nibble

	nAction_Nibble = make8(lData,1);	// extract upper byte
	swap(nAction_Nibble);				// swap to place Action nibble in low position..
	nAction_Nibble &= 0x0F;				// ...and clear upper nibble

   return(nAction_Nibble);

} // end of 'get_action_nibble()'
//==============================================================================================
/*
void configure_lines()
{
   unsigned int nCount;

   // Now set i/o in the following order: tristate any require inputs; set control pins for any outputs
   tristate_inputs();            // First, ensure any lines required to be inputs are tristated.

   configure_outputs();          // Secondly, set control pins for any outputs
   set_config_info();            // check for changes in bOn_Off

   if(nActual_IO_Direction != nRequired_IO_Direction)
   {
      nActual_IO_Direction = nRequired_IO_Direction;
      write_eeprom(EEPROM_IO_DIRECTION_BYTE_ADDRESS, nActual_IO_Direction);
   }

   for(nCount=0; nCount < 4; nCount++)
   {
      if(nRequired_Output_Control[nCount] != nActual_Output_Control[nCount])
      {
         nActual_Output_Control[nCount] = nRequired_Output_Control[nCount];
         write_eeprom((EEPROM_OP_CONTROL_BYTE0_ADDRESS+nCount), nActual_Output_Control[nCount]);
      }
   } // end of 'for(nCount=0; nCount < 4; nCount++)'
} // end of 'configure_lines()'
*/
//==============================================================================================

//==============================================================================================

//==============================================================================================
unsigned long extract_data_from_message(unsigned int nNibble_Offset)
{
   unsigned int nByte;
   signed int	nBit, nCount;

   unsigned long lData = 0;

   nByte = (ENCODED_MESSAGE_SIZE_BYTES-1) - (nNibble_Offset/2);

   if((nNibble_Offset - ((nNibble_Offset/2)*2)) == 0)
		nBit = 7;
   else nBit = 3;

	for(nCount = 15; nCount >= 0; nCount--)
	{
		if(bit_test(nAveraged_Sample_Buffer[nByte],nBit))
			bit_set(lData,nCount);

		if(--nBit < 0)
		{
			nBit = 7;
			nByte--;
		}
	}	// end of 'for(nCount = 15; nCount >= 0; nCount--)'
	return(lData);
}  // end of 'extract_data_from_message(unsigned int nNibble_Offset)'
//==============================================================================================
unsigned int determine_output_state(unsigned int nOutput_State)
{
	if(bOn_Off)	nOutput_State &= 0b00000011;	// Controlled device to be in On state; clear the 'OFF' state bits
	else
	{												// Controlled device to bin Off state; clear the 'ON' state bits
		shift_right(&nOutput_State,1,0);
		shift_right(&nOutput_State,1,0);
	}

	return(nOutput_State);
} // end of 'determine_output_state'
//==============================================================================================
unsigned int determine_message_type_to_follow()
{
	unsigned long 	lData;
	unsigned int	nMessage_Type;

	lData = extract_data_from_message(LONG_MESSAGE_TYPE_NIBBLE_INDEX); // returns a 16-bit result - only need uppermost nibble

	nMessage_Type = make8(lData,1);	// extract upper byte
	swap(nMessage_Type);				// swap to place Action nibble in low position..
	nMessage_Type &= 0x0F;				// ...and clear upper nibble

   return(nMessage_Type);
} // end of 'determine_message_type_to_follow()'
//==============================================================================================
unsigned long retrieve_2byte_period(unsigned int nNibble_Offset_Index)
{
	unsigned long 	lData;

   lData = extract_data_from_message(nNibble_Offset_Index);

   return(lData);
} // end of 'retrieve_2byte_period(unsigned int nNibble_Offset_Index)'

//==============================================================================================
unsigned int32 retrieve_3byte_period(unsigned int nNibble_Offset_Index)
{
	unsigned long 	lData[2] = {0,0};
   unsigned int32 dlResult;
   unsigned int nCount;

   lData[1] = extract_data_from_message(nNibble_Offset_Index);
   lData[0] = extract_data_from_message(nNibble_Offset_Index + 4);

   for(nCount = 0; nCount < 8; nCount++)	// Right-shift one byte and ensure upper byte is clear
      shift_right(lData,2,0);

   dlResult = make32(lData[1],lData[0]);

   return(dlResult);
} // end of 'unsigned int32 retrieve_start_time()'

//==============================================================================================
void store_schedule_periods_in_EEPROM()
{
	write_eeprom(EEPROM_OFF_PERIOD_BYTE1of3,make8(dlOff_Period_Minutes,0));
	write_eeprom(EEPROM_OFF_PERIOD_BYTE2of3,make8(dlOff_Period_Minutes,1));
	write_eeprom(EEPROM_OFF_PERIOD_BYTE3of3,make8(dlOff_Period_Minutes,2));

	write_eeprom(EEPROM_ON_PERIOD_BYTE1of2,make8(lOn_Period_Minutes,0));
	write_eeprom(EEPROM_ON_PERIOD_BYTE2of2,make8(lOn_Period_Minutes,1));

} // end of 'store_schedule_periods_in_EEPROM()'
//==============================================================================================
void store_schedule_timer_in_EEPROM()
{
   write_eeprom(EEPROM_MINUTE_TIMER_BYTE1of4,make8(dlSchedule_Minute_Timer,0));
   write_eeprom(EEPROM_MINUTE_TIMER_BYTE2of4,make8(dlSchedule_Minute_Timer,1));
   write_eeprom(EEPROM_MINUTE_TIMER_BYTE3of4,make8(dlSchedule_Minute_Timer,2));
   write_eeprom(EEPROM_MINUTE_TIMER_BYTE4of4,make8(dlSchedule_Minute_Timer,3));

   bStore_Timer_Flag = FALSE;
} // end of 'void store_schedule_timer_in_EEPROM()

//==============================================================================================
void calculate_limits(unsigned long *plLower_Limit, unsigned long *plUpper_Limit)
{
   unsigned int LimSW;
   LimSW=Main_Mode.Rx_Mode;
   if(LimSW==Rx)
      {
      *plUpper_Limit  = lHi_Speed_Sample_Counter/2; //Changed lower to .375 & upper to .5
      *plLower_Limit  = lHi_Speed_Sample_Counter/2;
//      *plLower_Limit -= *plLower_Limit/2;
//      *plLower_Limit -= *plLower_Limit/2;
	  if (bON_OFF)
	   {
        *plLower_Limit -= *plLower_Limit/2;
        *plLower_Limit -= *plLower_Limit/2;
        *plLower_Limit -= *plLower_Limit/2;
//        *plLower_Limit -= *plLower_Limit/2;
        *plUpper_Limit  = *plUpper_Limit + *plUpper_Limit - *plLower_Limit;
 //       *plLower_Limit += *plLower_Limit;
//        *plLower_Limit += *plLower_Limit;
//        *plLower_Limit += *plLower_Limit;
       }   
      }
   else
      {
#if LIMITS_HAVE_DEAD_ZONE==1 // limits are 25% - 75% with this option
      *plLower_Limit  = lHi_Speed_Sample_Counter/2;
      *plLower_Limit -= *plLower_Limit/2;
      *plUpper_Limit  = lHi_Speed_Sample_Counter/2; 
      *plUpper_Limit += *plUpper_Limit/2;
#else
      // Lower Limit = Higher limit = 1/16
      *plLower_Limit  = lHi_Speed_Sample_Counter/2;
      *plLower_Limit -= *plLower_Limit/2;
      *plLower_Limit -= *plLower_Limit/2;
      *plLower_Limit -= *plLower_Limit/2;

      *plUpper_Limit  = lHi_Speed_Sample_Counter/2;
      *plUpper_Limit -= *plUpper_Limit/2;
      *plUpper_Limit -= *plUpper_Limit/2;
      *plUpper_Limit -= *plUpper_Limit/2;
#endif


      }
} // end of 'calculate_limits(plLower_Limit, plUpper_Limit)'
//==============================================================================================
void adjust_bias_voltage()
/* This routine is called each time the receiver fails to detect a valid message, for whatever reason
   adjusts the bias
*/
{
   unsigned long lLower_Limit, lUpper_Limit;
   int incrementFlag=0;
   int decrementFlag=0;

   if(b5ms_Qualification_Failed // if search failed looking for a possible signal..
      || bFirst_Frame_Failed) // ...or got past 5ms qualification but failed in first frame
	{

     calculate_limits(&lLower_Limit, &lUpper_Limit);

#define DOUBLE_THRESHOLD 1

#if USE_FIXED_BIAS==0
      if(lHi_Speed_Ones_Counter < lLower_Limit)
	  {
         incrementFlag=1;
#if DONT_USE_HI_SPEED_DOUBLE_THRESHOLD==0
		if(lHi_Speed_Ones_Counter < DOUBLE_THRESHOLD)
		{								// double increment
            increment_bias_voltage();
            increment_bias_voltage();
		}
		else
		{								// single increment
#endif
            increment_bias_voltage();
#if DONT_USE_HI_SPEED_DOUBLE_THRESHOLD==0
		}
#endif
	  } // end of 'if(lHi_Speed_Ones_Counter < lLower_Limit)'

	  else
		if(lHi_Speed_Ones_Counter > lUpper_Limit)
		{
           decrementFlag=1;
#if DONT_USE_HI_SPEED_DOUBLE_THRESHOLD==0
			if(lHi_Speed_Ones_Counter > (lHi_Speed_Sample_Counter - DOUBLE_THRESHOLD))
			{								// double decrement
				decrement_bias_voltage();
				decrement_bias_voltage();
			}
			else
			{								// single decrement
#endif
				decrement_bias_voltage();
#if DONT_USE_HI_SPEED_DOUBLE_THRESHOLD==0
			}
#endif
		} // end of 'if(lHi_Speed_Ones_Counter > lUpper_Limit)'
/*
   if(!(Main_Mode.Rx_Mode==Sig_check))
    {*/
//   output_high(ENC_CH_B);//Iniialize low at start after reset
#if SAVE_BIAS_ADJUST_VALUES==1
        write_eeprom(0x60+PWMSaveIndex,(int)(lPWM_Duty&0x0ff));
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,(int)((lPWM_Duty>>8)&0x0ff));
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,(int)(lHi_Speed_Ones_Counter&0x0ff));
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,(int)((lHi_Speed_Ones_Counter>>8)&0x0ff));
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,(int)(lLower_Limit&0x0ff));
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,(int)((lLower_Limit>>8)&0x0ff));
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,(int)(lUpper_Limit&0x0ff));
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,(int)((lUpper_Limit>>8)&0x0ff));
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,(int)(lHi_Speed_Sample_Counter&0x0ff));
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,(int)((lHi_Speed_Sample_Counter>>8)&0x0ff));
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,incrementFlag);
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,decrementFlag);
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,0xff);
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,0xff);
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,0xff);
        PWMSaveIndex++;
        write_eeprom(0x60+PWMSaveIndex,0xff);
        PWMSaveIndex++;
        write_eeprom(0x90,PWMSaveIndex);
        if (PWMSaveIndex>47) PWMSaveIndex=0;
#endif

#if DONT_CHANGE_PWM_DURING_RCV==0
#if BIAS_SAME_FOR_ON_OFF==0
	if (bON_OFF)
      set_pwm1_duty(lPWM_ON_Duty);
    else
#endif
      set_pwm1_duty(lPWM_Duty);
#endif
//   output_low(ENC_CH_B);//Iniialize low at start after reset
#endif

	} // end of 'if(b5ms_Qualification_Failed.....'
	// NB: If '1's and '0's are balanced the CMPIN bias voltage is not changed

} // end of 'adjust_bias_voltage()'
//==============================================================================================
void store_bias_voltage()
{
	unsigned long lStored_Bias_Voltage, lStored_ON_Bias_Voltage ;
	unsigned long lStored_Min_Bias_Voltage, lStored_Min_ON_Bias_Voltage ;
	unsigned long lStored_Max_Bias_Voltage, lStored_Max_ON_Bias_Voltage ;

#if BIAS_SAME_FOR_ON_OFF==0
	if (bON_OFF)
	{
	lStored_ON_Bias_Voltage = make16(read_eeprom(EEPROM_PWM_ON_SETTING_HI_BYTE), read_eeprom(EEPROM_PWM_ON_SETTING_LO_BYTE));

	if (lStored_ON_Bias_Voltage > lPWM_ON_Duty)
	 {
	 if((lStored_ON_Bias_Voltage - lPWM_ON_Duty) > 1)
	  {
		write_eeprom(EEPROM_PWM_ON_SETTING_HI_BYTE,make8(lPWM_ON_Duty,1));
		write_eeprom(EEPROM_PWM_ON_SETTING_LO_BYTE,make8(lPWM_ON_Duty,0));
	  }
	 }
	else if (lStored_ON_Bias_Voltage < lPWM_ON_Duty)
	 {
	 if((lPWM_Duty - lStored_Bias_Voltage) > 1)
	  {
		write_eeprom(EEPROM_PWM_ON_SETTING_HI_BYTE,make8(lPWM_ON_Duty,1));
		write_eeprom(EEPROM_PWM_ON_SETTING_LO_BYTE,make8(lPWM_ON_Duty,0));
	  }
	 }
	}
	else
	{
#endif

	lStored_Bias_Voltage = make16(read_eeprom(EEPROM_PWM_SETTING_HI_BYTE), read_eeprom(EEPROM_PWM_SETTING_LO_BYTE));

	if (lStored_Bias_Voltage > lPWM_Duty)
	 {
	 if((lStored_Bias_Voltage - lPWM_Duty) > 1)
	  {
		write_eeprom(EEPROM_PWM_SETTING_HI_BYTE,make8(lPWM_Duty,1));
		write_eeprom(EEPROM_PWM_SETTING_LO_BYTE,make8(lPWM_Duty,0));
	  }
	 }
	else 
     {
     if (lStored_Bias_Voltage < lPWM_Duty)
	   {
	   if((lPWM_Duty - lStored_Bias_Voltage) > 1)
	    {
		 write_eeprom(EEPROM_PWM_SETTING_HI_BYTE,make8(lPWM_Duty,1));
		 write_eeprom(EEPROM_PWM_SETTING_LO_BYTE,make8(lPWM_Duty,0));
	    }
       }   
	 }
#if BIAS_SAME_FOR_ON_OFF==0
	} // end of if ON else OFF 
#endif
/*
// Check for max/min
	if (bON_OFF)
	{
	lStored_Max_ON_Bias_Voltage = make16(read_eeprom(EEPROM_MAX_PWM_ON_HI_BYTE), read_eeprom(EEPROM_MAX_PWM_ON_LO_BYTE));
	if (lPWM_ON_Duty > lStored_Max_ON_Bias_Voltage)
	  {
		write_eeprom(EEPROM_MAX_PWM_ON_HI_BYTE,make8(lPWM_ON_Duty,1));
		write_eeprom(EEPROM_MAX_PWM_ON_LO_BYTE,make8(lPWM_ON_Duty,0));
	  }
	lStored_Min_ON_Bias_Voltage = make16(read_eeprom(EEPROM_MIN_PWM_ON_HI_BYTE), read_eeprom(EEPROM_MIN_PWM_ON_LO_BYTE));
	if (lPWM_ON_Duty < lStored_Min_ON_Bias_Voltage)
	  {
		write_eeprom(EEPROM_MIN_PWM_ON_HI_BYTE,make8(lPWM_ON_Duty,1));
		write_eeprom(EEPROM_MIN_PWM_ON_LO_BYTE,make8(lPWM_ON_Duty,0));
	  }
	}
	else
	{
	lStored_Max_Bias_Voltage = make16(read_eeprom(EEPROM_MAX_PWM_HI_BYTE), read_eeprom(EEPROM_MAX_PWM_LO_BYTE));
	if (lPWM_Duty > lStored_Max_Bias_Voltage)
	  {
		write_eeprom(EEPROM_MAX_PWM_HI_BYTE,make8(lPWM_Duty,1));
		write_eeprom(EEPROM_MAX_PWM_LO_BYTE,make8(lPWM_Duty,0));
	  }
	lStored_Min_Bias_Voltage = make16(read_eeprom(EEPROM_MIN_PWM_HI_BYTE), read_eeprom(EEPROM_MIN_PWM_LO_BYTE));
	if (lPWM_Duty < lStored_Min_Bias_Voltage)
	  {
		write_eeprom(EEPROM_MIN_PWM_HI_BYTE,make8(lPWM_Duty,1));
		write_eeprom(EEPROM_MIN_PWM_LO_BYTE,make8(lPWM_Duty,0));
	  }
	}
*/
}
//==============================================================================================
void enable_RFM()
{
    unsigned long lRequired_Voltage_AD_reading, lAD_reading=0;
	unsigned long lCount=0;
    BOOLEAN ADCStop=FALSE;

  if (!bRFM_Enabled)        //.. Enable the RFM
   {
//   setup_ccp1 (CCP_OFF);
   output_high(RX_ENABLE_PIN);        //.. Enable the RFM & wait for bias to stabilize
//   delay_us(RFM_ENABLE_DELAY_US);//		COMMENTED OUT 10-12-07, REPLACED WITH.....
   rfmJustEnabledFlag=1;

//   set_pwm1_duty(127); //Set to 1.2


#if OSC_DIV_4_ON_RA7==0
	if (bON_OFF)
     output_low(RX_SENSITIVITY_CTL_PIN);   // .. set the Sensitivity to High (when RFM is enabled) note: reversed this from optput_high 11/24/12 MPatten
//     output_float(RX_SENSITIVITY_CTL_PIN);   // .. set the Sensitivity to normal for test
    else
     output_float(RX_SENSITIVITY_CTL_PIN);   // .. set the Sensitivity to normal (when RFM is enabled)
#endif

#if USE_FIXED_BIAS==0

#if BIAS_SAME_FOR_ON_OFF==0
   if (bON_OFF)
     set_pwm1_duty(lPWM_ON_Duty);
   else
#endif
     set_pwm1_duty(lPWM_Duty);
#endif
#if SAVE_CLOCK_REGS_IN_EEPROM==1
   clkRegTemp= CCP1CON;
   if (read_eeprom(0x9c)==0xff) {
      write_eeprom(0x9c,clkRegTemp);
   }

   clkRegTemp= CCPR1L;	
   if (read_eeprom(0x9d)==0xff) {
      write_eeprom(0x9d,clkRegTemp);
   }
#endif

#if TURN_OFF_TX_BIAS_PWM!=1
#if SAVE_BITS_4_5_CCP1CON==1
   clkRegTemp= CCP1CON;
#endif
   setup_ccp1 (CCP_PWM);
#if SAVE_BITS_4_5_CCP1CON==1
   CCP1CON|=(clkRegTemp&0x30);
#endif
#endif
#if SAVE_CLOCK_REGS_IN_EEPROM==1
   clkRegTemp= CCP1CON;
   if (read_eeprom(0x9e)==0xff) {
      write_eeprom(0x9e,clkRegTemp);
   }

   clkRegTemp= CCPR1L;	
   if (read_eeprom(0x9f)==0xff) {
      write_eeprom(0x9f,clkRegTemp);
   }
#endif


#if 1==0
   initialize_AD();

	if (bON_OFF)
	{
	lRequired_Voltage_AD_reading = (unsigned long) ((((int32)lPWM_ON_Duty)*1024)/DUTY_CYCLE_100); 
	}
	else
	{
	lRequired_Voltage_AD_reading = (unsigned long) ((((int32)lPWM_Duty)*1024)/DUTY_CYCLE_100);
	}
#endif
	bRFM_Enabled = TRUE;

delay_ms(delayBeforeQualify);		// Allows RFM to stailize before processing data



   shutdown_AD_and_restore_data_input_pin();

  }

} // end of 'enable_RFM()'
//==============================================================================================
void disable_RFM()
{
   output_low(RX_ENABLE_PIN);        //.. Disable the RFM
#if OSC_DIV_4_ON_RA7==0
   output_float(RX_SENSITIVITY_CTL_PIN);   // .. set the Sensitivity to High (when RFM is enabled)
#endif

   bRFM_Enabled = FALSE;
   //disable the pwm
//   setup_timer_2(T2_DISABLED,0,1);   // Disable Timer 2
#if TURN_OFF_TX_BIAS_PWM!=1
   setup_ccp1(CCP_OFF);
#endif

   output_float(BIAS_SETTING_PIN); 

} // end of 'disable_RFM()'
//==============================================================================================
void initialize_AD()
{
   input(THRESHOLD_SET_MONITOR);   // Needed because set to output_float in disable_RFM
#if USING_PIC18LF26K22==1
   setup_adc_ports(sAN0|sAN1|sAN2 , VSS_VDD);  // AN5 is used to read C12
                                           // A/D range is 0 - Vdd
#else
   setup_adc_ports(AN0_TO_AN2 | VSS_VDD);  // AN5 is used to read C12
                                           // A/D range is 0 - Vdd
#endif
   if (Main_Mode.Tx_Mode==0)
      setup_adc(ADC_CLOCK_DIV_8 | ADC_TAD_MUL_0); // A/D clock source period = 1us;
   else
      setup_adc(ADC_CLOCK_DIV_64 | ADC_TAD_MUL_20); // A/D clock source period = 1us;
                                                // Tacq = 190us [Rsource = 1Mohms]
                                                // Because of 190us acquisition time cannot do automatic need delay after set channel
                                                // Measurement Time = 190us + (11 x 1) = 201us (sampling + conversion)
   set_adc_channel(0);   // Pin AN0 (pin# 27) used to measure RX dc bias
   delay_us(760);
}
// end of 'initialize_AD()'
//==============================================================================================
void shutdown_AD_and_restore_data_input_pin()   // shut down the A/D and restore RA5 to its proper configuration
{
   setup_adc_ports(NO_ANALOGS);
   setup_adc(ADC_OFF);
   output_float(THRESHOLD_SET_MONITOR);
//   set_tris_E(TRIS_E);   // don't think this line is necessary
}
// end of 'shutdown_AD_and_restore_data_input_pin()'

//==============================================================================================
void increment_bias_voltage()
{
#if BIAS_SAME_FOR_ON_OFF==0
  if (bON_OFF)
    { 
	lPWM_ON_Duty++;   // bias voltage is too low; increment it
	if(lPWM_ON_Duty > 511)
	  lPWM_ON_Duty = 400;
	}
  else
   {
#endif
	  lPWM_Duty++;   // bias voltage is too low; increment it
	  if(lPWM_Duty > 511)
		lPWM_Duty = 400;
#if BIAS_SAME_FOR_ON_OFF==0
	}
#endif

#if SAVE_PWM_DUTY_IN_EEPROM==1
    write_eeprom(savEEPROMIndex,make8(lPWM_Duty,0));
    savEEPROMIndex++;
    if (savEEPROMIndex>0x7F) {
       savEEPROMIndex=0x60;
    }
    incrementCount++;
    write_eeprom(0x89,incrementCount);
#endif
 
//   determine_C12_charge_time();
} // end of 'increment_pwm_duty()'
//==============================================================================================
void decrement_bias_voltage()
{
#if BIAS_SAME_FOR_ON_OFF==0
  if (bON_OFF)
   {
   lPWM_ON_Duty--;   // bias voltage is too high; decrement it
   if (lPWM_ON_Duty<1)
	lPWM_ON_Duty = 100;
	}   
  else
   {
#endif
   lPWM_Duty--;   // bias voltage is too high; decrement it
   if (lPWM_Duty<1)
	lPWM_Duty = 100;
#if BIAS_SAME_FOR_ON_OFF==0
   }   
#endif

#if SAVE_PWM_DUTY_IN_EEPROM==1
    write_eeprom(savEEPROMIndex,make8(lPWM_Duty,1));
    savEEPROMIndex++;
    if (savEEPROMIndex>7F) {
       savEEPROMIndex=60;
    }
    decrementCount++;
    write_eeprom(0x8a,decrementCount);
#endif

//   determine_C12_charge_time();
} // end of 'increment_pwm_duty()'///////////////////////////////////////////////////////////////////////////////////////////////
// ============================================================================================
void set_config_info()
{
   unsigned int   nConfig_Local;

   nConfig_Local = nConfig;

   if(bOn_Off == ON)  bit_set(nConfig_Local,OUTPUT_STATE_BIT);
   else             bit_clear(nConfig_Local,OUTPUT_STATE_BIT);

   if(bScheduling == On) bit_set(nConfig_Local,STORED_SCHEDULE_STATUS_BIT);
   else                bit_clear(nConfig_Local,STORED_SCHEDULE_STATUS_BIT);

   if(bSchedule_State == On) bit_set(nConfig_Local,STORED_SCHEDULE_STATE_BIT);
   else                    bit_clear(nConfig_Local,STORED_SCHEDULE_STATE_BIT);

   if(nConfig_Local != nConfig)
   {
      nConfig = nConfig_Local;
      write_eeprom(EEPROM_CONFIG_STATE,nConfig);
   }
}
// end of 'set_config_info()'
/*//==============================================================================================
 #separate float crude_log2(float fValue)
// A second-order estimate:   ln(1+x) = x - (x^2)/2
//                            let y = 1+ x, therefore x = y - 1  ;   here fValue = y
{
   float fResult;

   fValue = fValue - 1;   // now fValue = x

   fResult = fValue * (1 - (fValue/2));

   return(fResult);

} // end of 'crude_log(float fValue)'
*///==============================================================================================
/*
void determine_encoded_msg_lsb_position()
// determine the bit position of the encoded message's lsb in nAveraged_Sample_Buffer
// determines global constants nLast_Encoded_Bit_BitIndex & nLast_Bit_Byte_Index
{
   nLast_Encoded_Bit_BitIndex = ENCODED_MESSAGE_SIZE_BYTES; //copies ENCODED_MESSAGE_SIZE_BYTES with any decimal component removed
   nLast_Encoded_Bit_BitIndex = ENCODED_MESSAGE_SIZE_BITS - (nLast_Encoded_Bit_BitIndex*8); // any non-zero result indicates extra bits in non-whole byte message length

   if(nLast_Encoded_Bit_BitIndex != 0)
   {
      nLast_Encoded_Bit_BitIndex = 8 - nLast_Encoded_Bit_BitIndex;
      nLast_Bit_Byte_Index = (SAMPLE_BYTES_PER_FRAME-1) - (ENCODED_MESSAGE_SIZE_BYTES + 1);
   }
} // end of 'determine_encoded_msg_lsb_position()'
/* // ============================================================================================
// DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....
void pulse_pinE(unsigned int nPin_No, nDly)
{
	if(nPin_No == 0)	output_high(PIN_E0);
	if(nPin_No == 1)	output_high(PIN_E1);
	if(nPin_No == 2)	output_high(PIN_E2);
	delay_msec(nDly);
	if(nPin_No == 0)	output_low(PIN_E0);
	if(nPin_No == 1)	output_low(PIN_E1);
	if(nPin_No == 2)	output_low(PIN_E2);
}
// DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....

// ============================================================================================
// DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....
void pulse_FETGate(nDly)
{
	output_high(Q9G2PIN5);
	delay_usec(nDly);
	output_low(Q9G2PIN5);
}
// DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....DEBUGGING ONLY.....

/*
//DEBUGGING ONLY....DEBUGGING ONLY....DEBUGGING ONLY....DEBUGGING ONLY....DEBUGGING ONLY....DEBUGGING ONLY....DEBUGGING ONLY....

	pulse_FETGate(1);

	for(nDEBUG_Count = 0; nDEBUG_Count < 8; nDEBUG_Count++)
	{
		rotate_left(&nAction_Nibble,1);

		if(bit_test(nAction_Nibble,0)) output_high(Q9G2PIN5);	// DEBUGGING ONLY
		else output_low(Q9G2PIN5);	// DEBUGGING ONLY

		delay_msec(1);
	}

	output_low(Q9G2PIN5);
	delay_usec(100);
	pulse_FETGate(100);

//DEBUGGING ONLY....DEBUGGING ONLY....DEBUGGING ONLY....DEBUGGING ONLY....DEBUGGING ONLY....DEBUGGING ONLY....DEBUGGING ONLY....

//======================================================================================================
void display_long_int_on_Line_6(unsigned long lVariable)
{
	unsigned int nCount;

	output_high(LINE_6_LOGIC_IO);
	delay_msec(1);
	output_low(LINE_6_LOGIC_IO);
	delay_msec(1);

	for(nCount = 0; nCount < 16; nCount++)
	{
		rotate_left(&lVariable,2);

		if(bit_test(lVariable,0))
			output_high(LINE_6_LOGIC_IO);
		else
			output_low(LINE_6_LOGIC_IO);

		delay_msec(1);
	}

	output_low(LINE_6_LOGIC_IO);
	delay_msec(1);
	output_high(LINE_6_LOGIC_IO);
	delay_msec(1);
	output_low(LINE_6_LOGIC_IO);
	output_float(LINE_6_LOGIC_IO);
}
*/
//****************************************************************************************************
/*void TEST_PULSE(long length,N)
{
int m;
	for (m=0;m<N;m++)
{
		OUTPUT_HIGH(TEST_PIN);
		delay_usec(length);
		OUTPUT_LOW(TEST_PIN);
		delay_usec(length);
}
}
*/
//****************************************************************************************************
//****************************************************************************************************
void delay_usec( signed int n)
{
 int m=0;
n = 4*(n/6-20);
if (n<0)
{ n=0;}

while (m<n)
{m++;}
}
//****************************************************************************************************
void delay_msec(int n)
{
long m;
for (m=0; m< n;m++)
{
delay_usec(970);
}
}
//****************************************************************************************************

//****************************************************************************************************
//***DAC Added to parse power setting for SharpTx from the received data string
//****************************************************************************************************

void determine_power_level()

{
   unsigned long f0_field;
   unsigned char f0_c;

   f0_field = extract_data_from_message(LINE_7_OUTPUT_CONTROL_NIBBLE_INDEX);  //returns a 16-bit result with contents of data at nibble index in high byte
   f0_c = make8(f0_field,1);	// extract upper byte
   swap(f0_c);				// swap to place power nibble in low position..
   f0_c &= 0x03;				// ...and clear upper 5 bits for 4 power levels
//   f0_c &= 0x07;				// ...and clear upper nibble for 16 power levels

//   Power_Setting = f0_c<<6; // 4-level Scale to 256 position pot.
//   wasLowPowerNowHighFlag=0;
   if (f0_c>0) {
      if (Power_Setting!=0) {
//         wasLowPowerNowHighFlag=1;
      }
      Power_Setting = 0; // Makes high power with Larry GUI and non-modified for power control ladybugs
   }
   else Power_Setting = 255;
   write_eeprom(INIT_PWR_ADD, Power_Setting);

#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
   updatePSVoltageTarget();
#endif

} //end of determine_power_level

//****************************************************************************************************

void determine_channel()
//************************************************************************************************************
//***ASC*** added to parse channel set commands now processed internally
//***DAC*** modified to ignore off part of message
//************************************************************************************************************
{
   unsigned long f0_field,f1_field,f2_field;
   unsigned char f0_c,f1_c,f2_c;

   remoteChannel_Select = 0;
    f0_field = extract_data_from_message(LINE_4_OUTPUT_CONTROL_NIBBLE_INDEX); // returns a 16-bit result
    f0_c = make8(f0_field,1);	// extract upper byte
	swap(f0_c);				// swap to place Action nibble in low position..
	f0_c &= 0x03;				// ...and clear upper nibble


	switch(f0_c)
      {
	   case(0b00000011):
		 remoteChannel_Select |= 1;
	     break;	

	   default:
	      remoteChannel_Select &= 0XFE ;
  	      break;


       }//end f0_I switch

    f1_field = extract_data_from_message(LINE_5_OUTPUT_CONTROL_NIBBLE_INDEX); // returns a 16-bit result
    f1_c = make8(f1_field,1);	// extract upper byte
	swap(f1_c);				// swap to place Action nibble in low position..
	f1_c &= 0x03;				// ...and clear upper nibble

	switch(f1_c)
       {
	    case(0b00000011):
	       remoteChannel_Select |= 2 ;
	       break;
	
	    default:
		   remoteChannel_Select &= 0xFD ;
	       break;
       }//end f1_I switch

    f2_field = extract_data_from_message(LINE_6_OUTPUT_CONTROL_NIBBLE_INDEX); // returns a 16-bit result
    f2_c = make8(f2_field,1);	// extract upper byte
	swap(f2_c);				// swap to place Action nibble in low position..
	f2_c &= 0x03;				// ...and clear upper nibble

	switch(f2_c)
      {
	   case(0b00000011):
	      remoteChannel_Select |= 0X04 ;
	      break;

	   default:
	      remoteChannel_Select &= 0Xfb ;
  	      break;

       }//end f2_I switch

   if (Main_Mode.Tx_Mode > 2) Main_Mode.Tx_Mode = Tx_On_Change; //Tx already on and initialized do change

} // end of 'determine_channel()'
//**********************************************************************************************


void determine_stereo()
//************************************************************************************************************
//***ASC*** added to parse channel set commands now processed internally
//************************************************************************************************************
{
   unsigned long R_field,L_field;
   unsigned char R_c,L_c;

    L_field = extract_data_from_message(LINE_2_OUTPUT_CONTROL_NIBBLE_INDEX); // returns a 16-bit result
    L_c = make8(L_field,1);	// extract upper byte
	swap(L_c);				// swap to place Action nibble in low position..
	L_c &= 0x0F;				// ...and clear upper nibble

    R_field = extract_data_from_message(LINE_3_OUTPUT_CONTROL_NIBBLE_INDEX); // returns a 16-bit result
    R_c = make8(R_field,1);	// extract upper byte
	swap(R_c);				// swap to place Action nibble in low position..
	R_c &= 0x0F;				// ...and clear upper nibble


	switch(L_c)
      {
	   case(0b00001011):
		  remoteLeft_Set = 1;
          GbAudio_Left=1;
     	  break;	

	   default:
		  remoteLeft_Set = 0;
          GbAudio_Mode=MONO;
	      break;

       }//end L_I switch

	switch(R_c)
      {
	   case(0b00001011):
		  remoteRight_Set = 1;
          GbAudio_Left=0;
	      break;	

	   default:
		  remoteRight_Set = 0;
          GbAudio_Mode=MONO;
	      break;

       }//end R_I switch

   if (remoteLeft_Set&remoteRight_Set) GbAudio_Mode = STEREO;

   write_eeprom(0XF1,GbAudio_Mode);
   write_eeprom(0XF2,remoteLeft_Set);
   if (Main_Mode.Tx_Mode > 2) Main_Mode.Tx_Mode = Tx_On_Change; //Tx already on and initialized do change
}


//****************************************************************************************************

void Init_Synth(int FoLDcode, int pwrDown)
{
   output_high(SYNTH_LE);	//ensure the Synth Latch Enable pin is high before we start

   GnSynth_Msg[2]=FUNCTION1;
//	GnSynth_Msg[2]= -0X00;
   GnSynth_Msg[1]=FUNCTION2;
   GnSynth_Msg[0]=FUNCTION3;
   GnSynth_Msg[0]&=0x8F;
   GnSynth_Msg[0]|=((FoLDcode<<4)&0x70);
   GnSynth_Msg[0]|=((pwrDown<<3)&0x08);

   Send_Synth_Msg();

//   write_eeprom(0x70, 0xBB); // test register to show that synth has initialized (3/26/14 MPatten)


} // end of Function 'Init_Synth'

  //---------------------------------------------------------------------------------------------------

void Send_Synth_Msg()
{
   unsigned int nCount;


   output_low(SYNTH_LE);    //assert low Synth LE (Latch Enable)
   for (nCount=1; nCount<=24; nCount++)
   {
      rotate_left(GnSynth_Msg,3);
         if (bit_test(GnSynth_Msg[0],0)) output_high(SDA);
         else output_low(SDA);

//      delay_usec(SYNTH_PLS_DLY);   //delay 1us (synth min delays typically <100ns)
		delay_us(20);   //delay 1us (synth min delays typically <100ns)
      output_high(SCL);  //toggle Serial Clock
//      delay_usec(SYNTH_PLS_DLY);   //delay 1us (synth min delays typically <100ns)
		delay_us(20);   //delay 1us (synth min delays typically <100ns)
      output_low(SCL);
   }

//   delay_usec(SYNTH_PLS_DLY);   //delay 1us (synth min delays typically <100ns)
		delay_us(20);   //delay 1us (synth min delays typically <100ns)

   output_high(SYNTH_LE);    //assert low Synth LE (Latch Enable)

   output_high(SDA);	// Set the data line high to reduce current (saves ~250uA for a R46 = 10k)
     delay_usec(50);   
//   output_low(SYNTH_LE);    //assert low Synth LE (Latch Enable)
} // end of Function 'Send_Synth_Msg'
  //---------------------------------------------------------------------------------------------------

void Init_extAD(int gainMode, int inputMode, int pwrMode)
{
   unsigned int nAD_Data, nReg_Addr;

   output_low(SCL);				//ensure clock is low to start
   output_high(AD_CHIP_SLCT);	//ensure the A/D CSN (data latch) pin is high before we start

//   nAD_Data=AD_IPSLCTDATA;
   nAD_Data=(inputMode&0x0f);
   nAD_Data|=((gainMode&0x0f)<<4);
   nReg_Addr=REG0;
   Send_AD_Msg(nReg_Addr, nAD_Data);

   nAD_Data=(pwrMode&0x03);
   nReg_Addr=REG1;
   Send_AD_Msg(nReg_Addr, nAD_Data);

   nAD_Data=AD_MODECTL;
   nReg_Addr=REG2;
   Send_AD_Msg(nReg_Addr, nAD_Data);

   nAD_Data=AD_IPGACTL;
   nReg_Addr=REG3;
   Send_AD_Msg(nReg_Addr, nAD_Data);

   output_high(SDA);	// Set the data line high to reduce current (saves ~250uA for a R46 = 10k)
//   output_low(SDA);	// now trying to leave data line low MPatten 12-5-15

}  // end of Function 'Init_extAD'



  //---------------------------------------------------------------------------------------------------

void Send_AD_Msg(int nReg_Addr, nAD_Data)
{
   unsigned int nCount, nAD_Msg[2];

   output_low(AD_CHIP_SLCT);    //assert low Synth LE (Latch Enable)

   nAD_Msg[1] = (0b10100000 | nReg_Addr);
   nAD_Msg[0] = nAD_Data;

   for (nCount=1; nCount<=16; nCount++)
   {
      rotate_left(nAD_Msg,2);
         if (bit_test(nAD_Msg[0],0)) output_high(SDA);
         else output_low(SDA);

      delay_usec(AD_PLS_DLY);   //delay 1us (synth min delays typically <100ns)

      output_high(SCL);  //toggle Serial Clock
      delay_usec(AD_PLS_DLY);   //delay 1us (synth min delays typically <100ns)
      output_low(SCL);
      delay_usec(AD_PLS_DLY);   //delay 1us (synth min delays typically <100ns)
   }
   delay_usec(AD_PLS_DLY);   //delay 1us (synth min delays typically <100ns)
   output_high(AD_CHIP_SLCT);    //assert low Synth LE (Latch Enable)
   delay_usec(AD_PLS_DLY);   //delay 1us (synth min delays typically <100ns)

} // end of Function 'Send_AD_Msg'


  //---------------------------------------------------------------------------------------------------

void Change_Frequency()
{
   unsigned int nEEPROM_read_address, nTemp;


	write_eeprom(0XF0,remoteChannel_select);


   GnSynth_Msg[2] = COMMON_R1;
   GnSynth_Msg[1] = R_BYTE2[Channel];        //read the R2 byte
   GnSynth_Msg[0] = R_BYTE3[Channel];        //read the R3 byte (LS Synth Byte)
   Send_Synth_Msg();


                              //send the N value to the Synth
   GnSynth_Msg[2] = N_BYTE1[Channel];
   GnSynth_Msg[1] = N_BYTE2[Channel];        //read the N2 byte
   GnSynth_Msg[0] = N_BYTE3[Channel];    //read the N3 byte (LS Synth Byte)
   Send_Synth_Msg();


#ifdef COMMON_CHANNEL_BIAS
	nEEPROM_read_address = BIAS_VALUE_BASE_ADDRESS;
#else
      if (Power_Setting==0) {
		 nEEPROM_read_address = BIAS_VALUE_BASE_ADDRESS+Channel+8;
      } else {
   		 nEEPROM_read_address = BIAS_VALUE_BASE_ADDRESS+Channel;
      }
#endif


#ifndef NO_DIGITAL_POTENTIOMETER
   GnCurrent_Ch_Bias_EEPROM = read_eeprom(nEEPROM_read_address);
//   GnCurrent_Ch_Bias 		= GnCurrent_Ch_Bias_EEPROM & Power_Setting;
   GnCurrent_Ch_Bias 		= GnCurrent_Ch_Bias_EEPROM; // & Power_Setting;
   set_pot(GnCurrent_Ch_Bias, POT0);

   if (Power_Setting==0) {
      nEEPROM_read_address = MODINDX_VALUE_BASE_ADDRESS + Channel+8;//read the Channel's Mod Index Pot setting
   } else {
      nEEPROM_read_address = MODINDX_VALUE_BASE_ADDRESS + Channel;//read the Channel's Mod Index Pot setting
   }
   GnCurrent_Ch_ModIndx_EEPROM = read_eeprom(nEEPROM_read_address);
   GnCurrent_Ch_ModIndx 	   = GnCurrent_Ch_ModIndx_EEPROM;

   if(GbAudio_Mode == STEREO)
		set_pot(GnCurrent_Ch_ModIndx_EEPROM, POT1);	//for STEREO MODE

   else                                             //for MONO MODE...
   {	//                         nTemp = shift_right(&GnCurrent_Ch_ModIndx_EEPROM,1,0);
                                   nTemp = 0xff - GnCurrent_Ch_ModIndx_EEPROM;
                                   shift_right(&nTemp,1,0);
                                   nTemp = GnCurrent_Ch_ModIndx_EEPROM + nTemp;

								   set_pot(nTemp, POT1);	//for MONO MODE
   } 
#endif
} //end of Function 'Change_Frequency'
//**********************************************************************************************
BOOLEAN set_pot(unsigned int nValue, nPot_Number)
{

      //11-29-04  NB!! This routine does NOT include   i)the ability to read pot values
      //                                              ii) the ability to write to the General Purpose memory

      // ASSUMES STANDARD_I/O
      // Requires DS1845.h

      unsigned int nAttempt_Index=0, nAttempt_Limit=10;
      unsigned int nTiming_Delay=SCLK_PERIOD; //Longest minimum Timing Delay is 900ns (=taa, see X95820 d'sheet)
      BOOLEAN      bTransmit_Status;

      output_high(WP);              //ensure Digital Pot is disabled
      delay_usec(nTiming_Delay);

      bTransmit_Status = FALSE;

      while (!bTransmit_Status && nAttempt_Index < nAttempt_Limit)
      {
         nAttempt_Index++;

         // START cond'n for Digital Pot = SCL Hi, SDA Hi -> Lo; assume SCL is high, and SDA is low
         output_high(SCL);
         output_high(SDA);
         output_low(WP);   //  enable Digital Pot
                           //  >600ns after START condition before enabling potentiometer
         delay_usec(nTiming_Delay);      //  Min Delay 600ns between SCL rising edge and SDA falling edge, both crossing 70% of Vcc
         output_low(SDA);  //signifies serial comms start

	      bTransmit_Status = Send_Data(ID_BYTE, nPot_Number, nValue, nTiming_Delay);
	        if (bTransmit_Status) bTransmit_Status = Send_Data(ADDRESS_BYTE, nPot_Number, nValue, nTiming_Delay);
            	if (bTransmit_Status) bTransmit_Status = Send_Data(DATA_BYTE, nPot_Number, nValue, nTiming_Delay);

            //STOP cond'n for Digital Pot = SCL Hi, SDA Lo -> Hi; Ensure SCL is high, and SDA then transitions Lo -> Hi
         output_low (SDA); //sets SDA as output, and low
         delay_usec(nTiming_Delay);      //Min Delay 600ns between SCL rising edge crossing 70% of Vcc, to SDA rising edge crossing 30% of Vcc

         output_high(SCL);
         delay_usec(nTiming_Delay);

         output_high (SDA); //set SDA output hi
         delay_usec(nTiming_Delay);      //>600ns after STOP condition before disabling potentiometer

         output_high(WP);  //disable Digital Pot

	      if(!bTransmit_Status) delay_msec(1); // if Dig Pot is not responding it is probably busy writing to its EEPROM
                                            // from a previous set_pot() call; this takes (typ. 5ms).
       } // end of ' while (!bTransmit_Status && nAttempt_Index < nAttempt_Limit)'

   set_data_speed_and_polarity(); // set the data speed and polarity in the FPGA ('cos polarity control line is shared with WP)


   return(bTransmit_Status);

} // END OF FUNCTION 'set_pot'
  //---------------------------------------------------------------------------------------------------

   BOOLEAN Send_Data(unsigned int nWhat_to_Send, nPot_Address, nValue, nTiming_Delay)
   {
      signed int nBit_Index;
      BOOLEAN bBit_To_Send, bTransmit_Status=TRUE;
      BOOLEAN const  A0=1, A1=1, A2=0;

	  nBit_Index=7;

      do
      {
         bBit_To_Send = 0;

         switch (nWhat_to_Send)
            {
            case ID_BYTE:
               Switch (nBit_Index)
               {
                  case 7: bBit_To_Send = 0;
                     break;
                  case 6: bBit_To_Send = 1;
                     break;
                  case 5: bBit_To_Send = 0;
                     break;
                  case 4: bBit_To_Send = 1;
                     break;
                  case 3: bBit_To_Send = A2;
                     break;
                  case 2: bBit_To_Send = A1;
                     break;
                  case 1: bBit_To_Send = A0;
                     break;
                  case 0: bBit_To_Send = 0;  //the Read/Write control bit (Write = Low)
                     break;
               } // end of 'Switch (nBit_Index)'
               break;

            case ADDRESS_BYTE:
				if(bit_test(nPot_Address,nBit_Index))
                     bBit_To_Send = 1;
		        break;

            case DATA_BYTE:
			  bBit_To_Send = bit_test(nValue,nBit_Index);
              break;
            } //end of 'Switch (nWhat_to_Send)'

         output_low(SCL);
         delay_usec(nTiming_Delay);

         if (bBit_To_Send) output_high(SDA);
         else output_low(SDA);

         delay_usec(nTiming_Delay);
         output_high(SCL);

      } while (--nBit_Index >= 0); // end off 'while (--Bit_Index >= 0)'

      // Now wait for Ack pulse
      delay_usec(nTiming_Delay);
      bTransmit_Status=input(SDA);   //ignore result, only to switch SDA pin to an input.
      output_low(SCL);
      delay_usec(nTiming_Delay);
      output_high(SCL); //SDA should be under control of Dig Pot, and stable, at this point.
      delay_usec(nTiming_Delay);

      bTransmit_Status = !input(SDA);

      	output_low(SCL);		// finish Ack pulse portion of function with SCL lo & SDA uC pin as an i/p
      delay_usec(nTiming_Delay);
      return (bTransmit_Status);

   } // end of Function 'Send_Data'
  //---------------------------------------------------------------------------------------------
void set_data_speed_and_polarity()
{
   if (bON_OFF)  //Only control line if transmitter is on otherwise set low to conserve power
   {
//      output_high(DATA_SPEED_SELECT); // set for half speed
//      output_high(DATA_POLARITY_SELECT); // set for cadence on

     if(read_eeprom(EEPROM_DATA_SPEED)!=0) output_low(DATA_SPEED_SELECT);
     else output_high(DATA_SPEED_SELECT);

     if(read_eeprom(EEPROM_CADENCE)==0) output_low(DATA_POLARITY_SELECT);
     else output_high(DATA_POLARITY_SELECT);
   }
   else 
   {
     output_low(DATA_SPEED_SELECT);
     output_low(DATA_POLARITY_SELECT);
   }
} // end of 'set_data_speed_and_polarity()'
  //---------------------------------------------------------------------------------------------------
// disable this unusued routine
/*
void Turn_REMOTE_CTL_ON_OFF()
{

if (bON_OFF!=bON_OFF_State)
   {
	bON_OFF_State=bON_OFF;
	if (bON_OFF) 
	   {

/*		OUTPUT_HIGH(REMOTE_RCVR_PWR_CTL); 
        delay_msec(100);
	    Init_extAD();  //initialize the external A/D
	    Init_Synth();  //initialize the external Synthesize 
        set_data_speed_and_polarity(); // set the data speed and polarity in the FPGA
/
		Change_Frequency();  //set frequency to channel
  	    if ( GbAudio_Mode == STEREO)
	      {
	       output_high(LEFT_PIN); 
   	       output_high(RIGHT_PIN); 
	      }
	    else 
	      {
	       if (GbAudio_Left)
	         {
	         output_high(LEFT_PIN);
	         output_low(RIGHT_PIN);
	         }
  	       else
	         {
	          output_low(LEFT_PIN);
	          output_high(RIGHT_PIN);
	         }
	       }

//  Start the subclk as it stays high indicating the PIC should send Subchannel data
//  but the interrupt for sending the subchannel data is rising edge triggered.  To get the process
//  started a pulse is sent to the FPGA
    	enable_interrupts(INT_EXT); //enable FPGA SubClock interrupt

	  }
	else 
	  {
		disable_interrupts(INT_EXT);
		output_low(SDA); //These outputs requied to be low when TX off else blow LMX
		output_low(AD_CHIP_SLCT);
		output_low(SUBDATA);
		output_low(RIGHT_PIN);
		output_low(SCL);
		output_low(WP);
		output_low(LEFT_PIN);
		output_low(SYNTH_LE);
		OUTPUT_LOW(REMOTE_RCVR_PWR_CTL);
	  } 
	}
}
*/
//****************************************************************************************************

//					FOLD routines
//***************************************************************************************************

void send_R(void)
{
//  output_high(SYNTH_LE);	//ensure the Synth Latch Enable pin is high before we start

   GnSynth_Msg[2]=FUNCTION1;
   GnSynth_Msg[1]=FUNCTION2;
   GnSynth_Msg[0]=FUNCTION3;


   Send_Synth_Msg();

}
//***************************************************************************************************
void send_N(void)
{
//  output_high(SYNTH_LE);	//ensure the Synth Latch Enable pin is high before we start

   GnSynth_Msg[2]=FUNCTION1;
   GnSynth_Msg[1]=FUNCTION2;
   GnSynth_Msg[0]=FUNCTION3 ;
	GnSynth_Msg[0]= N_OUT;

   Send_Synth_Msg();


}
//****************************************************************************************************
void set_FOLD_HIGH(void)
{
//  output_high(SYNTH_LE);	//ensure the Synth Latch Enable pin is high before we start

   GnSynth_Msg[2]=FUNCTION1;
   GnSynth_Msg[1]=FUNCTION2;
   GnSynth_Msg[0]= HIGH_OUT;


   Send_Synth_Msg();



}
//****************************************************************************************************
void set_FOLD_LOW(void)
{
//  output_high(SYNTH_LE);	//ensure the Synth Latch Enable pin is high before we start

   GnSynth_Msg[2]=FUNCTION1;
   GnSynth_Msg[1]=FUNCTION2;
 //  GnSynth_Msg[0]=FUNCTION3 & FOLD_LOW | LOW_OUT;
	GnSynth_Msg[0]= LOW_OUT;

   Send_Synth_Msg();


}
//**************************************************************************************************

