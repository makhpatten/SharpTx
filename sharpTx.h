//*************************************************************************************************************
//				SharpTx header modified for 18F2520
//************************************************************************************************************
//				Mark Patten Feb 2018 
//************************************************************************************************************

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
//                                  PORT PIN DEFINITIONS                                     //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

//*********************************************************************************************
//									PORT A
//*********************************************************************************************
//				 	 7			- Output:	Rx Sensitivity Control
//				 	 |6			- Intput:	Serial data input from Receiver
//				 	 ||5		- Output:	Digital Potentiometer Write Protect (active low)/Data Polarity Select(Cadence)
//				 	 |||4		- Output:	AKM5356 A/D chip select pin (active low)/Data Speed Select
//				 	 ||||3		- Output: 	Sub Data serial output pin to FPGA
//				 	 |||||2		- Input:	A/D Battery Supply Voltage
//				 	 ||||||1	- Input:	A/D Wide Input Power Supply output voltage /2
//				 	 |||||||0	- Input:    A/D input for adaptive threshold monitoring
//				 	 ||||||||
#define TRIS_A     0b01000111
#define  TRIS_A_LOW_CURRENT 0b11000111
#define  TRIS_A_HIGH_SENSITIVITY 0b01000111

#define THRESHOLD_SET_MONITOR	PIN_A0  // Input - A/D input for adaptive threshold monitoring
#define PWR_VOLT                PIN_A1   // Input - A/D Wide Input Power Supply output voltage /2
#define BATT_VOLT           	PIN_A2   // i/p Analog - Battery Supply Voltage   
#define SUBDATA   				PIN_A3   // o/p - Sub Data serial output pin to FPGA
#define AD_CHIP_SLCT        	PIN_A4   // o/p - to AKM5356 A/D chip select pin (active low) 
#define DATA_SPEED_SELECT   	PIN_A4
#define WP                 		PIN_A5     // o/p - to Digital Potentiometer Write Protect (active low)
#define DATA_POLARITY_SELECT 	PIN_A5     //same as WP
#define RCVR_DATA_IN_PIN   		PIN_A6  // Serial data input from Receiver
#define RX_SENSITIVITY_CTL_PIN 	PIN_A7  //0 = hi, tristate (define pin as input)= lo

      // Define values for Data Speed
#define DATA_SPEED_1MB_PER_SEC 1
#define DATA_SPEED_2MB_PER_SEC 0
      // Define values for FPGA normal or inverted data
#define DATA_NONINVERTED   0
#define DATA_INVERTED   1


//**************************************************************************************
//							PORT B
//**************************************************************************************
//				 	 7			- Input:	Select Bias (1)/Mod index(0)
//				 	 |6			- Input:	Program the pot select
//				 	 ||5		- Output:	Right mono
//				 	 |||4		- Output:	Left Mono
//				 	 ||||3		- Output:	Tx Power PWM to Wide Input Power Supply
//				 	 |||||2		- Input:	Input from Rotary Encoder Ch.B
//				 	 ||||||1	- Input:	Input from Rotary Encoder Ch.A
//				 	 |||||||0	- Input:	FPGA SubData sync pulse input
//				 	 ||||||||
#if USE_SHAFT_ENCODER_FOR_TEST==0
//   #define TRIS_B     0b11000111 // as below
//   #define TRIS_B     0b01000111 // using b7 (PGD) for clock to synth and pot
//   #define TRIS_B     0b11000111 // no longer using b7 (PGD) for clock to synth and pot 7-12-17 MHPatten
   #define TRIS_B     0b11000011 // using b2 for Vtx control 8-3-17 MHPatten

#else
   #define TRIS_B     0b11000101 // as below
#endif


#define SUBCLK    				PIN_B0     // i/p Digital - FPGA SubData sync pulse input
#define ENC_CH_A                PIN_B1     // i/p Digital - Input from Rotary Encoder Ch.A
//#define ENC_CH_B                PIN_B2     // i/p Digital - Input from Rotary Encoder Ch.B
#define VTX_PWR_CTL             PIN_B2     // i/p Digital - Input from Rotary Encoder Ch.B
#define TX_PWR_PWM_PIN          PIN_B3     // o/p PWM - to Wide Input Power Supply
#define LEFT_PIN				PIN_B4	   // o/p Digital - Left Mono; 
#define RIGHT_PIN   			PIN_B5     // o/p Digital - Right mono; 
#define Program_Mode_Pin		PIN_B6	   // i/p Digital - Low = Program the pot bias and mod index settings
#define F2_FN_SLCT				PIN_B7	   // i/p Digital - Select Bias (1)/Mod index(0)
#define POT_SLCT  BIAS
//#define POT_SLCT  0  //To program mod indx under debugger
#define PORTB_PULLUPS_STATE OFF      //changed to OFF 3-3-15; TRUE causes current drain with some txs when in 'Tx off' state.


//***************************************************************************************
//						PORT C
//***************************************************************************************
//			      7----------- Output - Transmitter Power Control
//			      |6---------- Output - Serial Data Clock
//			      ||5--------- Output - Serial Data Line
//			      |||4-------- Output - Enables RFM receiver chip
//			      ||||3------- Output - Synth Latch Enable
//			      |||||2------ Output -  CMP0 - Bias Setting PWM1
//			      ||||||1----- Input - 32KHz Clock
//			      |||||||0---- input- 32KHz Clock
//                ||||||||

#if TURN_OFF_TX_BIAS_PWM==1
  #define  TRIS_C 0b00000111 // disable the rx bias PWM
#else
  #define  TRIS_C 0b10000011 // set C7 hi-Z since no power control on Flyron version
#endif

#define XTAL32KHz_PIN   		PIN_C0  // 32 KHz Clock
#define BIAS_SETTING_PIN    	PIN_C2	// Output - CMP0 - Bias Setting PWM1
#define SYNTH_LE				PIN_C3	// Output - Synth Latch Enable (Lo->Hi transition loads high, idle high
#define RX_ENABLE_PIN      		PIN_C4  // Output - Sleep - Enables RFM receiver chip
#define SDA                 	PIN_C5   // o/p - Serial Data Line (briefly changes to i/p while communicating with Digital Potentiometer)
//#if CONFIG_UART_IN_HARDWARE==0
//  #define SCL       				PIN_C6     // o/p - Serial Data Clock output to A/D, Synth & Dig. Pot
//#else
  #define SCL       				PIN_B1     // o/p - Serial Data Clock output to A/D, Synth & Dig. Pot
//#endif

//#if USE_C7_FOR_RS232==0
//  #define REMOTE_RCVR_PWR_CTL 			PIN_C7	// Output - Transmitter Power Control  no power control on Flyron version, commented out 7-12-17 MHPatten
//#else
//  #define REMOTE_RCVR_PWR_CTL 			PIN_B1	// Output - Transmitter Power Control
//#endif

//***************************************************************************************
//						PORT D
//***************************************************************************************
//			      7----------- No pin
//			      |6---------- No pin
//			      ||5--------- No pin
//			      |||4-------- No pin
//			      ||||3------- No pin
//			      |||||2------ No pin
//			      ||||||1----- No pin
//			      |||||||0---- No pin
//                ||||||||
//#define  TRIS_D 0b11111111


//*******************************************************************************************
//							PORT E
//*******************************************************************************************
//
//			      7----------- No pin
//			      |6---------- No pin
//			      ||5--------- No pin
//			      |||4-------- No pin
//			      ||||3------- No pin	
//			      |||||2------ No pin
//			      ||||||1----- No pin
//		          |||||||0---- Input: Not used
//          	  ||||||||
#define  TRIS_E 0b11111111


