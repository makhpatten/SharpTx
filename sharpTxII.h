/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                             //
//                                              		sharpTxII.h                                            //
//                                      Author: Mark Patten, Senior Engineer,                                  //
//	                           		                                                                           //
//                                      C Firmware code for the SharpTxII                                      //
//                                                                                                             //
//                                        Written for the PICC Compiler                                        //
//             (CCS Information Systems, www.ccsinfo.com, Brookfield, WI 53008, Tel. (262) 522-6500)           //
//                                                                                                             //
//                                                                                                             //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* MODIFICATIONS

7-14-16 Mod's made for Rev C layout - search for '7-14-16' for location in code
This version modified for test; always on ASC 7/28/2018
        See also changes similarly marked in _Rx.cremoter
*/
#define INPUT_SAFETY_MODE // if enabled this, when a Line is configured as an input, it is actually configured as a logic low
// #define ADDR_MS_FIRST		//COMMENT OUT THIS LINE IF RECEIVER ID ADDRESS IS SENT LEAST SIGNIFIFICANT CHARACTER FIRST


// DEBUGGING CODE........DEBUGGING CODE........DEBUGGING CODE........DEBUGGING CODE........DEBUGGING CODE........
//
// #define SLEEP_CURRENT_TEST	// If this line is NOT commented out, THE RECEIVER WILL ONLY SLEEP I.E. WON'T LOOK FOR SIGNALS!!!!!
//#define RF_ALWAYS_ON			// If this line is NOT commented out, the receiver will NOT turn OFF the RFM MODULE 
//***ASC*** MOD 072808
//
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// DEBUGGING CODE........DEBUGGING CODE........DEBUGGING CODE........DEBUGGING CODE........DEBUGGING CODE........DEBUGGING CODE...

#if USING_PIC18LF26K22==1
   #include <18LF26k22.h>
#else
   #include <18F2520.h>
#endif

//DISABLE THIS LINE#device ICD=TRUE
#device high_ints=TRUE
#device adc=10
//#device WRITE_EEPROM=NOINT
#use delay(clock=32000000)
//TEMPORARILY DISABLE THIS LINE IF USING ICD DEBUGGER
///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                            //
//                                     PIC FUSES                                             //
//  11-8-05; MCLR changed to NOMCLR following problem where interface cable shorted pin.8 to gnd, causing device to be held in reset
/////////////////////////////////////////////////////////////////////////////////////////////
#if USING_PIC18LF26K22==1
   #fuses NOPLLEN, NOWDT, INTRC_IO, NOFCMEN, NOBROWNOUT, BORV19, PUT, NOCPD, STVREN, NODEBUG, NOLVP, NOWRT, NOWRTD, NOIESO, NOEBTR, NOEBTRB, MCLR, NOPROTECT, NOCPB, NOWRTB, NOWRT, CCP2B3, XINST 
   // "DEBUGGING CODE ONLY!!!!"                                                                            //
   //DISABLE THIS LINE 
   //#fuses NOPLLEN,NOWDT,INTRC_IO,NOFCMEN,NOBROWNOUT,PUT,NOCPD,NOSTVREN,DEBUG,NOLVP,NOWRT,NOWRTD,NOIESO,NOEBTR,NOEBTRB,MCLR,NOPROTECT,NOCPB,NOWRTB,NOWRTC, CCP2B3
#else
#if OSC_DIV_4_ON_RA7==0
//   #fuses NOWDT, LPT1OSC, INTRC_IO, NOFCMEN, NOBROWNOUT, BORV20, PUT, NOCPD, STVREN, NODEBUG, NOLVP, NOWRT, NOWRTD, NOIESO, NOEBTR, NOEBTRB, NOMCLR, NOPROTECT, NOCPB, NOWRTB, NOWRT, CCP2B3 
   #fuses NOWDT, LPT1OSC, INTRC_IO, NOFCMEN, NOBROWNOUT, BORV20, PUT, NOCPD, STVREN, DEBUG, NOLVP, NOWRT, NOWRTD, NOIESO, NOEBTR, NOEBTRB, NOMCLR, NOPROTECT, NOCPB, NOWRTB, NOWRT, CCP2B3 
#else
// INTRC_IO changed to INTRC
   #fuses NOWDT, LPT1OSC, INTRC, NOFCMEN, NOBROWNOUT, BORV20, PUT, NOCPD, STVREN, NODEBUG, NOLVP, NOWRT, NOWRTD, NOIESO, NOEBTR, NOEBTRB, NOMCLR, NOPROTECT, NOCPB, NOWRTB, NOWRT, CCP2B3 
#endif
   // "DEBUGGING CODE ONLY!!!!"                                                                            //
   //DISABLE THIS LINE 
   //#fuses NOWDT,INTRC_IO,NOFCMEN,NOBROWNOUT,NOPUT,NOCPD,NOSTVREN,DEBUG,NOLVP,NOWRT,NOWRTD,NOIESO,NOEBTR,NOEBTRB,MCLR,NOPROTECT,NOCPB,NOWRTB,NOWRTC, CCP2B3
#endif

// Watchdog minimim (1:1 postscale) = 4ms
//    --->>VALID FUSES<----
// LP		Low power osc < 200 khz
// XT		Crystal osc <= 4mhz
// HS		High speed Osc (> 4mhz)
// EC		External clock with CLKOUT
// EC_IO		External clock
// H4		High speed osc with HW enabled 4X PLL
// RC_IO		Resistor/Capacitor Osc
// INTRC_IO		Internal RC Osc, no CLKOUT
// INTRC		Internal RC Osc
// RC		Resistor/Capacitor Osc with CLKOUT
// FCMEN		Fail-safe clock monitor enabled
// NOFCMEN	Fail-safe clock monitor disabled
// NOBROWNOUT	No brownout reset
// BROWNOUT	Reset when brownout detected
// WDT1		Watch Dog Timer uses 1:1 Postscale
// WDT2		Watch Dog Timer uses 1:2 Postscale
// WDT4		Watch Dog Timer uses 1:4 Postscale
// WDT8		Watch Dog Timer uses 1:8 Postscale
// WDT16		Watch Dog Timer uses 1:16 Postscale
// WDT32		Watch Dog Timer uses 1:32 Postscale
// WDT64		Watch Dog Timer uses 1:64 Postscale
// WDT128		Watch Dog Timer uses 1:128 Postscale
// WDT256		Watch Dog Timer uses 1:256 Postscale
// WDT512		Watch Dog Timer uses 1:512 Postscale
// WDT1024	Watch Dog Timer uses 1:1024 Postscale
// WDT2048	Watch Dog Timer uses 1:2048 Postscale
// WDT4096	Watch Dog Timer uses 1:4096 Postscale
// WDT8192	Watch Dog Timer uses 1:8192 Postscale
// WDT16384	Watch Dog Timer uses 1:16384 Postscale
// WDT32768	Watch Dog Timer uses 1:32768 Postscale
// WDT		Watch Dog Timer
// NOWDT		No Watch Dog Timer
// BORV20		Brownout reset at 2.0V
// BORV27		Brownout reset at 2.7V
// BORV42		Brownout reset at 4.2V
// BORV45		Brownout reset at 4.5V
// PUT		Power Up Timer
// NOPUT		No Power Up Timer
// CPD		Data EEPROM Code Protected
// NOCPD		No EE protection
// NOSTVREN	Stack full/underflow will not cause reset
// STVREN		Stack full/underflow will cause reset
// NODEBUG	No Debug mode for ICD
// DEBUG		Debug mode for use with ICD
// NOLVP		No low voltage prgming, B3(PIC16) or B5(PIC18) used for I/O
// LVP		Low Voltage Programming on B3(PIC16) or B5(PIC18)
// WRT		Program Memory Write Protected
// NOWRT		Program memory not write protected
// WRTD		Data EEPROM write protected
// NOWRTD		Data EEPROM not write protected
// CCP2C1		CCP2 input/output multiplexed with RC1
// CCP2B3		CCP2 input/output multiplexed with RB3
// IESO		Internal External Switch Over mode enabled
// NOIESO		Internal External Switch Over mode disabled
// EBTR		Memory protected from table reads
// NOEBTR		Memory not protected from table reads
// EBTRB		Boot block protected from table reads
// NOEBTRB	Boot block not protected from table reads
// MCLR		Master Clear pin enabled
// NOMCLR		Master Clear pin used for I/O
// PROTECT	Code protected from reads
// NOPROTECT	Code not protected from reading
// CPB		Boot Block Code Protected
// NOCPB		No Boot Block code protection
// WRTB		Boot block write protected
// NOWRTB		Boot block not write protected
// WRTC		configuration registers write protected
// NOWRTC		configuration not registers write protected
// PBADEN		PORTB pins are configured as analog input channels on RESET
// NOPBADEN	PORTB pins are configured as digital I/O on RESET
///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                   STATE STRUCTURE AND DEFINES                                             //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

//Rx Sub-states

#define sleep_mode 0
#define sig_check 1
#define Rx 2
#define process 3
#define action 4
#define schedule 5

//Tx Sub_states

#define Tx_Off 0
#define Tx_OfftoOn_Vtx 1
#define Tx_OfftoOn_InitTx 2
#define Tx_On_change 3
#define Tx_On_chk_adj_PS 4
#define Tx_On_MeasureBatV 5
#define Tx_OntoOff 6
#define Tx_Pot_Adj 7
#define Tx_Bypass 8

// Main mode structure

struct Main_Mode_Type {
   unsigned Rx_Mode: 4;
   unsigned Tx_Mode: 4;
} Main_Mode;

unsigned TxMode_Save;

   
///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                   EEPROM INITIALIZATION VALUES AND RELATIVE ADDRESSES                     //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
#define EEPROM_INITIAL_IO_DIRECTION_BYTE 0xFF   //default IO direction control when PIC first programmed (all inputs)
#define EEPROM_INITIAL_OP_CONTROL_BYTE0  0xFF   // default output control settings when PIC first programmed (all Logic Hi)
#define EEPROM_INITIAL_OP_CONTROL_BYTE1  0xFF   //    "     "     "     "     "     "     "     "     "     "     "
#define EEPROM_INITIAL_OP_CONTROL_BYTE2  0xFF   //    "     "     "     "     "     "     "     "     "     "     "
#define EEPROM_INITIAL_OP_CONTROL_BYTE3  0xFF   //    "     "     "     "     "     "     "     "     "     "     "
#define EEPROM_INITIAL_PWM_SETTING_LO_BYTE 0xF0	// initialize the setting to a impractical value
#define EEPROM_INITIAL_PWM_SETTING_HI_BYTE 0x00
#define EEPROM_INITIAL_PWM_ON_SETTING_LO_BYTE 0xFF	// initialize the setting to a impractical value
#define EEPROM_INITIAL_PWM_ON_SETTING_HI_BYTE 0xFF

//#define EEPROM_IO_DIRECTION_BYTE_ADDRESS 0x00	//this a relative address, relative to the start address below in #rom
//#define EEPROM_OP_CONTROL_BYTE0_ADDRESS  0x01   //Stores Output Control values for Lines 1 thru 2
//#define EEPROM_OP_CONTROL_BYTE1_ADDRESS  0x02   //Stores Output Control values for Lines 3 thru 4
//#define EEPROM_OP_CONTROL_BYTE2_ADDRESS  0x03   //Stores Output Control values for Lines 5 thru 6
//#define EEPROM_OP_CONTROL_BYTE3_ADDRESS  0x04   //Stores Output Control values for Line 7
#define EEPROM_PWM_SETTING_LO_BYTE		  0x05	//Stores the PWM setting that determines the bias voltage
#define EEPROM_PWM_SETTING_HI_BYTE		  0x06	//Stores the PWM setting that determines the bias voltage
#define EEPROM_PWM_ON_SETTING_LO_BYTE		  0x15	//Stores the PWM setting that determines the bias voltage
#define EEPROM_PWM_ON_SETTING_HI_BYTE		  0x16	//Stores the PWM setting that determines the bias voltage

#define EEPROM_CONFIG_STATE              0x07   // EEPROM address of stored configuration
#ROM int8 0XF00007    = {0x00}  // Initialize power state to off
#define EEPROM_INITIAL_CONFIG_STATE      0x04    // Default value for above (when PIC first turned on after programming)
#define OUTPUT_STATE_BIT                 0x02   // bit# in EEPROM_CONFIG_STATE that stores Unit output state

#define STORED_SCHEDULE_STATUS_BIT       0x01   // bit# in EEPROM_CONFIG_STATE that determines whether Scheduling is On or Off
#define STORED_SCHEDULE_STATE_BIT        0x00   // bit# in EEPROM_CONFIG_STATE that determines whether Scheduling State is On or Off

#define EEPROM_MINUTE_TIMER_BYTE1of4     0x08   //Store elapsed minute timer
#define EEPROM_MINUTE_TIMER_BYTE2of4     0x09   //  "    "     "     "    "
#define EEPROM_MINUTE_TIMER_BYTE3of4     0x0A   //  "    "     "     "    "
#define EEPROM_MINUTE_TIMER_BYTE4of4     0x0B   //  "    "     "     "    "

#define EEPROM_MINUTE_TIMER_BYTE_DEFAULT 0xFF

#define EEPROM_OFF_PERIOD_BYTE3of3		0x0C
#define EEPROM_OFF_PERIOD_BYTE2of3		0x0D
#define EEPROM_OFF_PERIOD_BYTE1of3		0x0E

#define EEPROM_ON_PERIOD_BYTE2of2		0x0F
#define EEPROM_ON_PERIOD_BYTE1of2		0x10

#define EEPROM_ON_AND_OFF_PERIOD_DEFAULT 0x00

//EEPROM BASE ADDRESSES FOR DIGITAL POT
#define BIAS_VALUE_BASE_ADDRESS     0x20  //Data EEPROM Address of Bias value (one byte) for Channel 0
#define MODINDX_VALUE_BASE_ADDRESS  0x30  //Data EEPROM Address of Modulation Index vsalue (one byte) for Channel 0

#define VTX_VALUE_BASE_ADDRESS  0x50  //Data EEPROM Address of Modulation Index vsalue (one byte) for Channel 0
#ROM int8 0XF00050    = {0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x6a, 0x6a, 0x6a, 0x6a, 0x6a, 0x6a, 0x6a, 0x6a}  // Power supply threshold low power, power supply threshold high power (corresponds to 2.6 V, 3.7 V)
#ROM int8 0XF00070    = {0xff, 0xff, 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}
#ROM int8 0XF00080    = {0xff, 0xff, 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}
#ROM int8 0XF00090    = {0xff, 0xff, 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}

#define INITIAL_BIAS_VALUE		0xF6
#define INITIAL_MODINDX_VALUE	0xEB

//-------------------------------------------------------------------------------------------------------
// Digital Pot initial values date stored in EEPROM

#ROM int8 0XF00020    = {
/* 0x20 */				INITIAL_BIAS_VALUE,	// the initial value of PA Bias Dig Pot (range 0 - 0x63)
/* 0x21 */				INITIAL_BIAS_VALUE,
/* 0x22 */				INITIAL_BIAS_VALUE,
/* 0x23 */				INITIAL_BIAS_VALUE,
/* 0x24 */				INITIAL_BIAS_VALUE,
/* 0x25 */				INITIAL_BIAS_VALUE,
/* 0x26 */				INITIAL_BIAS_VALUE,
/* 0x27 */				INITIAL_BIAS_VALUE,
/* 0x28 */				INITIAL_BIAS_VALUE,
/* 0x29 */				INITIAL_BIAS_VALUE,
/* 0x2A */				INITIAL_BIAS_VALUE,
/* 0x2B */				INITIAL_BIAS_VALUE,
/* 0x2C */				INITIAL_BIAS_VALUE,
/* 0x2D */				INITIAL_BIAS_VALUE,
/* 0x2E */				INITIAL_BIAS_VALUE,
/* 0x2F */				INITIAL_BIAS_VALUE
						}
#ROM int8 0XF00030    = {
/* 0x30 */				INITIAL_MODINDX_VALUE, 	// the initial value of Modulation cntl Dig Pot(range 0 - 0xFF)
/* 0x31 */				INITIAL_MODINDX_VALUE,
/* 0x32 */				INITIAL_MODINDX_VALUE,
/* 0x33 */				INITIAL_MODINDX_VALUE,
/* 0x34 */				INITIAL_MODINDX_VALUE,
/* 0x35 */				INITIAL_MODINDX_VALUE,
/* 0x36 */				INITIAL_MODINDX_VALUE,
/* 0x37 */				INITIAL_MODINDX_VALUE,
/* 0x38 */				INITIAL_MODINDX_VALUE,
/* 0x39 */				INITIAL_MODINDX_VALUE,
/* 0x3A */				INITIAL_MODINDX_VALUE,
/* 0x3B */				INITIAL_MODINDX_VALUE,
/* 0x3C */				INITIAL_MODINDX_VALUE,
/* 0x3D */				INITIAL_MODINDX_VALUE,
/* 0x3E */				INITIAL_MODINDX_VALUE,
/* 0x3F */				INITIAL_MODINDX_VALUE
						}
#define PWM_ON_OFFSET_ADD               0x40
#define PWM_OFFSET_ADD                  0x41
#ROM int8 0XF00040    = {0x00, 0x00}  // On offset=5  Off offset=5
#define INIT_PWR_ADD					0x42 // Hi Pwr = >0 Lo Pwr = 0 
#define EEPROM_DEAD_ZONE_RADIUS_ADDRESS 0x43
#ROM int8 0XF00043    = {0x05} 
#define EEPROM_DELAY_MS_BEFORE_QUAL_ADDRESS 0x44
#ROM int8 0XF00044    = {0x0a} 
#define EEPROM_VBAT_MIN_VOLTAGE_ADDRESS 0x45
#ROM int8 0XF00045    = {117} // 117= 3.15 Volts
#define EEPROM_VBAT_LOW_VOLTAGE_ADDRESS 0x46
#ROM int8 0XF00046    = {121} // 121= 3.3 Volts
#define EEPROM_VBAT_RESET_VOLTAGE_ADDRESS 0x47
#ROM int8 0XF00047    = {32} // 32 = approx.0.9 Volts


//#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
//#define PS_THRESHOLD_LOWPOWER			0x48
//#define PS_THRESHOLD_HIGHPOWER			0x49
//#ROM int8 0XF00048    = {0x6a, 0x88}  // Power supply threshold low power, power supply threshold high power (corresponds to 2.6 V, 3.7 V)
//#endif

#define EEPROM_MIN_PWM_LO_BYTE			0x90
#define EEPROM_MIN_PWM_HI_BYTE			0x91
#define EEPROM_MAX_PWM_LO_BYTE			0x92
#define EEPROM_MAX_PWM_HI_BYTE			0x93
#define EEPROM_MIN_PWM_ON_LO_BYTE		0x94
#define EEPROM_MIN_PWM_ON_HI_BYTE		0x95
#define EEPROM_MAX_PWM_ON_LO_BYTE		0x96
#define EEPROM_MAX_PWM_ON_HI_BYTE		0x97

#define EEPROM_N_BYTE1_BASE				0XA0
#define EEPROM_COMMON_R1_ADDRESS		0XA8
#define EEPROM_N_BYTE2_BASE				0XB0
#define EEPROM_R_BYTE2_BASE				0XB8
#define EEPROM_N_BYTE3_BASE				0XC0
#define EEPROM_R_BYTE3_BASE				0XC8
#define EEPROM_FREQ_SOURCE				0XD0
#define EEPROM_INITAL_FREQ_SOURCE		0X01  //Initialize eeprom to 360 MHz built-in frq table
// Note: these were used in A2100412 but because Don Tunstal's unit was returned with cadence value overwritten, we decided to make it hardcoded into program
// So therefore these no longer have an effect on the speed and cadence setting
#define EEPROM_DATA_SPEED				0xD1  // 0=half speed, 1=full speed.
#ROM int8 0xF000D1 = {0x01} // initialize to full speed
#define EEPROM_CADENCE					0xD2  // 0=cadence off, 1=cadence on
#ROM int8 0xF000D2 = {0x00} // initialize to cadence off



#rom int8 0xF00005 = {
//*0xF00001*/          EEPROM_INITIAL_IO_DIRECTION_BYTE,	//programs the default configuration
//*0xF00001*/          EEPROM_INITIAL_OP_CONTROL_BYTE0,		// w/out int8, the #rom addresses as Words, NOT Bytes
//*0xF00002*/          EEPROM_INITIAL_OP_CONTROL_BYTE1,
//*0xF00003*/          EEPROM_INITIAL_OP_CONTROL_BYTE2,
//*0xF00004*/          EEPROM_INITIAL_OP_CONTROL_BYTE3,

/*0xF00005*/          EEPROM_INITIAL_PWM_SETTING_LO_BYTE,
/*0xF00006*/          EEPROM_INITIAL_PWM_SETTING_HI_BYTE,

/*0xF00007*/          EEPROM_INITIAL_CONFIG_STATE

//*0xF00008*/          EEPROM_MINUTE_TIMER_BYTE_DEFAULT,
//*0xF00009*/          EEPROM_MINUTE_TIMER_BYTE_DEFAULT,
//*0xF0000A*/          EEPROM_MINUTE_TIMER_BYTE_DEFAULT,
//*0xF0000B*/          EEPROM_MINUTE_TIMER_BYTE_DEFAULT,

//*0xF0000C*/          EEPROM_ON_AND_OFF_PERIOD_DEFAULT,
//*0xF0000D*/          EEPROM_ON_AND_OFF_PERIOD_DEFAULT,
//*0xF0000E*/          EEPROM_ON_AND_OFF_PERIOD_DEFAULT,
//*0xF0000F*/          EEPROM_ON_AND_OFF_PERIOD_DEFAULT,
//*0xF00010*/          EEPROM_ON_AND_OFF_PERIOD_DEFAULT
                }
#rom int8 0xF00015 = {EEPROM_INITIAL_PWM_ON_SETTING_LO_BYTE,
					  EEPROM_INITIAL_PWM_ON_SETTING_HI_BYTE
					 } 

//#rom int8 0xF00090 = {0xE8,0x00,0xE8,0x00,0xE8,0x00,0xE8,0x00}
///////////////////////////////////////////////////////////////////////////////////////////////
//Initial eeprom values for 360 frequency table. This allows frequency table change by changing EEPROM
///////////////////////////////////////////////////////////////////////////////////////////////
#rom int8 0xF000A0 = {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	/*0xF000B0*/	  0x6E,0x6F,0x71,0x72,0x73,0x74,0x76,0x77,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	/*0xF000C0*/	  0xA1,0xE1,0x21,0x61,0xA1,0xE1,0x21,0x61,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,
	/*0xF000D0*/	  EEPROM_INITAL_FREQ_SOURCE}

#define EEPROM_BAUD_RATE				0XDA
#rom int8 0xF000DA = {0x2b,0x5c} // This is the two byte baud rate on the RS-232 output... 0x2b5c = 11100 which seems to give 9800 baud

///////////////////////////////////////////////////////////////////////////////////////////////
//Customer and Receiver IDs
////////////////////////////////////////////////////////////////////////////////////////////////
#define EEPROM_CUSTOMER_ID_BASE  	0XFC
#define EEPROM_RECEIVER_ID_BASE  	0XFE

#rom int8 0xF000FC = {0x39,0x00,0xC9,0x02}
////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef 14V_OPERATION
   #define POWER_SWITCH     LINE_7_LO_ENABLE_PIN // NB Active Low!!!!!
#else
   #define POWER_SWITCH     LINE_1_HI_ENABLE_PIN // NB Active Low!!!!!
#endif

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//      DEFINITIONS FOR FUTURE PROJECTS TRANSMITTER CONTROL                     //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

//#ifdef 14V_OPERATION
//   #define POWER_SWITCH     CH_H_LO_ENABLE_PIN // NB Active Low!!!!!
//#else
//   #define POWER_SWITCH     CH_B_HI_ENABLE_PIN // NB Active Low!!!!!
//#endif

#ifdef ALL_TRANSISTORS
      #define LEFT_MIC_ENABLE_PIN  CH_C_LO_ENABLE_PIN
      #define RIGHT_MIC_ENABLE_PIN CH_D_LO_ENABLE_PIN

      #define FREQ_MSb  CH_G_LO_ENABLE_PIN
      #define FREQ_NSb  CH_F_LO_ENABLE_PIN
      #define FREQ_LSb  CH_E_LO_ENABLE_PIN
#else
      #define LEFT_MIC_ENABLE_PIN  PIN_B4
      #define RIGHT_MIC_ENABLE_PIN PIN_B1

      #define FREQ_MSb  PIN_C5  //Tempory until new board DAC 040309
      #define FREQ_NSb  PIN_B7
      #define FREQ_LSb  PIN_B6
#endif

#define POWER_SWITCH_PORT_PIN 4     //Pin # on Part A for power control

   //////////////////// Command Protocols ///////////////////////////////////////////
//    All references to 'Pins' refer to connector pins, NOT PIC pins
//					                 <-PIN->
//                              |#  a-z|Function	ACTION             COMMENT
//                              |      |
#define TURN_OFF_SWITCH 0x00	// 2   B	 Power		Set RA4 to an input            Sends Pin 2 hi-imp
#define TURN_ON_SWITCH  0x01	// 2	 B	 Power      Set RA4 to an output and =0    Pin 2 = Battery+

#define MONO_LEFT       0x02	// 3	 C	  Mic Left	Set Pin 3 low, Pin 4 hi-imp
#define MONO_RIGHT      0x03	// 4	 D	  Mic Right Set Pin 4 low, Pin 3 hi-imp
// Commands 0x04 & 0x05 are reserved

//#define STEREO          0x06	// 3&4 C&D Mic's on  Set Pins 3 & 4 low
#define STEREO    1
#define MONO      0
#define SPARE         0x07	// 3&4 C&D Mute mic's Set Pins 3 & 4 to hi-imp	- SPARE, AVAILABLE FOR FUTURE USE

#define CHANNEL_1		   0x08  // 5-7 E-G  Sets Freq   Pins 5-7 hi-imp  Pin 7 is MSb NB: These are active low!
#define CHANNEL_2		   0x09  // 5-7 E-G    "     "   Pin 5 low, Pins 6-7 hi-imp
#define CHANNEL_3		   0x0A  // 5-7 E-G    "     "   Pin 6 low, Pins 5,7 hi-imp
#define CHANNEL_4		   0x0B  // 5-7 E-G    "     "   Pins 5,6 low, Pin 7 hi-imp
#define CHANNEL_5		   0x0C  // 5-7 E-G    "     "   Pin 7 low, Pins 5-6 hi-imp
#define CHANNEL_6		   0x0D  // 5-7 E-G    "     "   Pins 5,7 low, Pin 6 hi-imp
#define CHANNEL_7		   0x0E  // 5-7 E-G    "     "   Pins 6,7 low, Pin 5 hi-imp
#define CHANNEL_8		   0x0F  // 5-7 E-G    "     "   Pins 5-7 low


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                                  TIMING DELAYS                                            //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

#define RFM_PWR_UP_DELAY_MS 20    // Wait period for RFM to power up from dead
#define RFM_ENABLE_DELAY_US 2500     // Wait period for RFM to enable and get over any startup glitch COMMENTED OUT 10-12-07, replaced with...
#define TMR1_INIT               0xF333      /* Initialization value of Timer 1 prior to sleep (woken by TMR1 overflow)
                                               0xF333 causes overflow in 100ms (for 32768KHz xtal source)
											   '+1' is necessary to correct a PIC anomaly (DAC anomally different)*/
//#define LISTEN_INTERVAL 30  //LISTEN_INTERVAL is multiples of 100ms
#define LISTEN_INTERVAL 45  //LISTEN_INTERVAL is multiples of 100ms

//#define INITIAL_ACTIVITY_SEARCH_PERIOD 164 // Range 1 - 32767    164 = 5ms,  measured in multiples of 32678 Clock Cycles
#define INITIAL_ACTIVITY_SEARCH_PERIOD 328 // Range 1 - 32767    328 = 10ms,  measured in multiples of 32678 Clock Cycles
//#define INITIAL_ACTIVITY_SEARCH_PERIOD 490 // Range 1 - 32767    490 = 15ms,  measured in multiples of 32678 Clock Cycles

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                        SIGNAL SEARCH PROCESS CONSTANTS                                    //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
//#define MAX_NUMBER_TRIES   10      // range 1-255 - the maximum of attempts for failed messages	COMMENT OUT FOR DEBUGGING ONLY...
#define MAX_NUMBER_TRIES   1      // range 1-255 - the maximum of attempts for failed messages	TEMPORARY REPLACEMENT

// The min # of '1' message bits in 5ms				= 1
// The max # of "	"	"	"	"	"		 		= 4
// The min # of zero crossings in message in 5ms	= 1
// The max # of zero crossings in message in 5ms	= 5
/*
#define MIN_NUM_ONES    2 // The min # of ones             allowed in the initial 5ms for a potential signal (5.1 bits)
#define MIN_ZERO_XINGS  1 // The min # of zero crossings "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "
#define MAX_ZERO_XINGS  11 // The max # of zero crossings "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "
*/
#define MIN_NUM_ONES_IN_5MS_4xOVERSAMPLED    2 // 3The min # of ones             allowed in the initial 5ms for a potential signal (5.1 bits)
#define MAX_NUM_ONES_IN_5MS_4xOVERSAMPLED    32 // 16The max # of ones             allowed in the initial 5ms for a potential signal (5.1 bits)
#if USE_NEW_ZERO_XINGS_VALUES==1
  #define MIN_ZERO_XINGS_IN_5MS_4xOVERSAMPLED  4 // 2The min # of zero crossings "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "
  #define MAX_ZERO_XINGS_IN_5MS_4xOVERSAMPLED  12 //7 The max # of zero crossings "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "
#else
  #define MIN_ZERO_XINGS_IN_5MS_4xOVERSAMPLED  2 // 2The min # of zero crossings "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "
  #define MAX_ZERO_XINGS_IN_5MS_4xOVERSAMPLED  32 //7 The max # of zero crossings "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "
#endif
/*
#define MIN_NUM_ONES_IN_5MS_4xOVERSAMPLED    7 // The min # of ones             allowed in the initial 10ms for a potential signal (5.1 bits)
#define MAX_NUM_ONES_IN_5MS_4xOVERSAMPLED    36 // 18The max # of ones             allowed in the initial 10ms for a potential signal (5.1 bits)
#define MIN_ZERO_XINGS_IN_5MS_4xOVERSAMPLED  4 // The min # of zero crossings "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "
#define MAX_ZERO_XINGS_IN_5MS_4xOVERSAMPLED  30 //15 The max # of zero crossings "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "
*/
// The min # of ones in 8 message bits (7.8ms) 		= 3
// The max # of ones in 8 message bits    " 		= 5
// The min # of zero crossings in 8 message bits  	= 2
// The max # of zero crossings in 8 message bits  	= 8

#define MIN_NUM_ONES_IN_8_4xOVERSAMPLED_MESSAGE_BITS    4 //6 The min # of ones             allowed in 7.8ms (at least three 1's)
#define MAX_NUM_ONES_IN_8_4xOVERSAMPLED_MESSAGE_BITS    48 //24 The max # of ones             allowed in 7.8ms (at least three 1's)
#define MIN_ZERO_XINGS_IN_8_4xOVERSAMPLED_MESSAGE_BITS  2 // 2The min # of zero crossings "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "
#define MAX_ZERO_XINGS_IN_8_4xOVERSAMPLED_MESSAGE_BITS  48 //14 The max # of zero crossings "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "


#define START_FRAME_MARKER_MSByte 0b11001100
#define START_FRAME_MARKER_LSByte 0b00110011
#define NORMAL 0
#define INVERTED 1


#define NO_OF_BYTES_IN_START_FRAME   2
#define OVERSAMPLING_RATE            4
#define NUMBER_FRAMES_TO_SAMPLE      6     // the number of message frames to sample over
#define ENCODED_MESSAGE_SIZE_BYTES   14  //MUST BE EITHER WHOLE OR x.5!!!!!!!!!!!!!!
#define ENCODED_MESSAGE_SIZE_NIBBLES ENCODED_MESSAGE_SIZE_BYTES*2
#define ENCODED_MESSAGE_SIZE_BITS    ENCODED_MESSAGE_SIZE_BYTES*8
#define NO_OF_6B_IN_MESSAGE          DECODED_MESSAGE_SIZE_NIBBLES

#define SAMPLE_BUFFER_SIZE_BYTES    OVERSAMPLING_RATE*NUMBER_FRAMES_TO_SAMPLE*ENCODED_MESSAGE_SIZE_BYTES
// #define SAMPLE_BUFFER_SIZE_BYTES    336
//      SAMPLE_BUFFER_SIZE       //in Bytes: = 4x oversampling x 6 frames x [2 byte Start Frame
                                 //                   + 3.75 bytes (Customer ID & Receiver ID, encoded 4b6b)
                                 //                   + 6.75 Data Byte (encoded 4b,6b]
//#define FRAME_SIZE_SAMPLES       SAMPLE_BYTES_PER_FRAME*8
#define FRAME_SIZE_SAMPLES       448
//      FRAME_SIZE_SAMPLES       // the number of samples in a single frame = 4x oversampling x 8 bits x
                                 //                   x [2 byte Start Frame
                                 //                   + 3 bytes (Customer ID & Receiver ID, encoded 4b6b)
                                 //                   + 1.5 Data Byte (encoded 4b,6b]
#define MAX_SAMPLE_COUNT         FRAME_SIZE_SAMPLES*NUMBER_FRAMES_TO_SAMPLE
#define SAMPLE_BYTES_PER_FRAME   OVERSAMPLING_RATE*ENCODED_MESSAGE_SIZE_BYTES
// #define SAMPLE_BYTES_PER_FRAME   50
///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                                  MESSAGE CONSTRUCTION                                     //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

#define DECODED_MESSAGE_SIZE_BYTES   8  //MUST BE A WHOLE NUMBER!!!!!!!!!!!!!!
#define DECODED_MESSAGE_SIZE_NIBBLES   DECODED_MESSAGE_SIZE_BYTES*2
#define DECODED_MESSAGE_SIZE_BITS   DECODED_MESSAGE_SIZE_BYTES*8

#define NUMBER_OF_CUSTOMER_ID_NIBBLES        2
#define NUMBER_OF_RECEIVER_ID_NIBBLES        3
#define NUMBER_OF_LOGIC_MODE_CONTROL_NIBBLES 2

#define RECEIVER_ID_OFFSET_NIBBLE_INDEX      0
#define LOGIC_MODE_CONTROL_NIBBLE_INDEX      RECEIVER_ID_OFFSET_NIBBLE_INDEX+NUMBER_OF_RECEIVER_ID_NIBBLES
#define LINE_1_OUTPUT_CONTROL_NIBBLE_INDEX   LOGIC_MODE_CONTROL_NIBBLE_INDEX+NUMBER_OF_LOGIC_MODE_CONTROL_NIBBLES
#define LINE_2_OUTPUT_CONTROL_NIBBLE_INDEX   LINE_1_OUTPUT_CONTROL_NIBBLE_INDEX+1
#define LINE_3_OUTPUT_CONTROL_NIBBLE_INDEX   LINE_2_OUTPUT_CONTROL_NIBBLE_INDEX+1
#define LINE_4_OUTPUT_CONTROL_NIBBLE_INDEX   LINE_3_OUTPUT_CONTROL_NIBBLE_INDEX+1
#define LINE_5_OUTPUT_CONTROL_NIBBLE_INDEX   LINE_4_OUTPUT_CONTROL_NIBBLE_INDEX+1
#define LINE_6_OUTPUT_CONTROL_NIBBLE_INDEX   LINE_5_OUTPUT_CONTROL_NIBBLE_INDEX+1
#define LINE_7_OUTPUT_CONTROL_NIBBLE_INDEX   LINE_6_OUTPUT_CONTROL_NIBBLE_INDEX+1
#define LONG_MESSAGE_TYPE_NIBBLE_INDEX       LINE_7_OUTPUT_CONTROL_NIBBLE_INDEX+1
#define CUSTOMER_ID_OFFSET_NIBBLE_INDEX      LONG_MESSAGE_TYPE_NIBBLE_INDEX+1
#define ACTION_STATUS_NIBBLE_INDEX           CUSTOMER_ID_OFFSET_NIBBLE_INDEX+NUMBER_OF_CUSTOMER_ID_NIBBLES

#define NUMBER_OF_START_TIME_NIBBLES                 6 // 3 bytes
#define NUMBER_OF_REPEAT_PERIOD_NIBBLES              6 // 3 bytes
#define NUMBER_OF_ON_PERIOD_NIBBLES                  4 // 2 bytes

#define SCHEDULE_START_TIME_OFFSET_NIBBLE_INDEX    0
#define SCHEDULE_REPEAT_PERIOD_OFFSET_NIBBLE_INDEX SCHEDULE_START_TIME_OFFSET_NIBBLE_INDEX+NUMBER_OF_START_TIME_NIBBLES
#define SCHEDULE_ON_PERIOD_OFFSET_NIBBLE_INDEX     SCHEDULE_REPEAT_PERIOD_OFFSET_NIBBLE_INDEX+NUMBER_OF_REPEAT_PERIOD_NIBBLES

///////////////////////////////////////////////////////////////////////////////////////////////
// 'Action' Byte flag bits
//          Flag                Bit#
#define ON_OFF_FLAG              3
#define ACTIVATE_SCHEDULE_FLAG   1
//////////////////////////////////////////////////////////////////////////////////////////////

//#define FOLLOWING_MESSAGE_TIMEOUT_x100MS 10
#define FOLLOWING_MESSAGE_TIMEOUT_x100MS 50
///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                                  MISCELLANEOUS                                            //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////
#define TOP    1
#define BOTTOM 0

#define ON	   1
#define OFF	   0

#define YES 1
#define NO  0

#define TOO_LOW   0
#define TOO_HIGH  1
#define UNKNOWN   2

//#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
//  #define PR2 80   // Timer 2 reset value for 99.2KHz & 32MHz clock---changed per Bill's request 9-11-13 MPatten
//  #define PR2S 80   // Timer 2 reset value for 99.2KHz & 32MHz clock---changed per Bill's request 9-11-13 MPatten
//#else
  #define PR2 127   // Timer 2 reset value for 62.5KHz & 32MHz clock
  #define PR2S 127   // Timer 2 reset value for 7.8125KHz & 4MHz clock
//#endif

#define DUTY_CYCLE_100	512 // pwm duty value at which duty-cycle = 100%

#define BIAS  1


#define SUBDATA_MSG_LNGTH 4   //the number of bytes in a Sub-Data message
                              //assume battery voltage value byte is lasy byte in Sub-data byte array (i.e. N-1,
                              //                                                    where N = SUBDATA_MSG_LNGTH)
#define BATTVOLT_BYTE     3   //Byte Number in SubData message array [0 to (SUBDATA_MSG_LNGTH-1)]
#define NO_SUBDATA_SYNC_PLSES 3 //The number of sync pulses for FPGA SubData synchronization
#define AD_PLS_DLY      4     //the pulse hold-time (us) for Synth and Clock changes
#define SYNTH_PLS_DLY   1     //the pulse hold-time (us) for Synth and Clock changes
#define ROT_ENC_DBNCE_DLY 1  //the debounce wait time (ms) for the Rotary Encode switch

//EEPROM BASE ADDRESSES

#define SUBDATA_VALUE_BASE_ADDRESS  0x1C  //Data EEPROM Address of Subdata low byte

static BOOLEAN GbAudio_Mode=STEREO; //initialize as Stereo

#define ENCODER_INTERRUPT INT_EXT1
#define SUBDATA_CLOCK_INTERRUPT INT_EXT
//---------- set/initialize LMX2316 serial data ----------
//
//Function Latch register for LMX23x6:
//4 MSbits IGNORED + 18 Function Latch bits + 2 Control Bits
//
// 			                     3-------- F18 1 = synchronous powerDown
//			                     |2------- F17 0 = normal use (test modes)
//			                     ||1------ F16 0 = normal use (test modes)
//			                     |||0----- F15 0 = normal use (test modes)
//	              		         ||||
#define     FUNCTION1	   0b00001000
//
//			                 7----------- F14 1 - count[32] fast lock timeout
//			                 |6---------- F13 1 - count[16] fast lock timeout
//			                 ||5--------- F12 1 - count[8] fast lock timeout
//			                 |||4-------- F11 1 - count[4] fast lock timeout
//			                 ||||3------- F10 1 = enable fast timeout counter
//			                 |||||2------ F9 1 = fastlock mode 4
//			                 ||||||1----- F8 1 = fastlock mode 4
//			                 |||||||0---- F7 0 = charge pump normal (not tri-stated)
//          			     ||||||||
#define     FUNCTION2      0b11111110
//
//			                 7----------- F6 1 = positive vco polarity
//			                 |6---------- F5     Fo/LD bit 0 (001 = Digital Lock Detect)	(see below)
//			                 ||5--------- F4     Fo/LD bit 1 (100 = R divider output) <			"
//			                 |||4-------- F3     Fo/LD bit 2 (011 = Active High)				"
//			                 ||||3------- F2 0 = normal operation (not powered down)
//			                 |||||2------ F1 0 = counter reset off
//			                 ||||||1----- C2 1 = select data to function latch
//			                 |||||||0---- C1 1 = full initialization
//			                 ||||||||
#define     FUNCTION3	   0b10000011		//.. Fo/LD pin is n/c in this fdesign.


#define     R_OUT		   0b01000000
#define 	N_OUT		   0b00100000
#define 	HIGH_OUT	   0b00110000
#define 	LOW_OUT		   0b01110000
#define     FOLD_LOW       0b10001111