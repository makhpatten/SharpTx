/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                             //
//                                        sharpTx_Config_Options.h			                                   //
//                                                                                                             //
//                                      Author: Mark Patten, Senior Engineer,                                  //
//                                                                                                             //
//                              C Firmware code for the Remote switch receiver PCB Rev C                      //
//                                                                                                             //
//                                        Written for the PICC Compiler                                        //
//             (CCS Information Systems, www.ccsinfo.com, Brookfield, WI 53008, Tel. (262) 522-6500)           //
//                                                                                                             //
//                                     Program commencement date: Jan 2015                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define RCVR_ADDR_IN_DECIMAL	//COMMENT OUT THIS LINE IF RECEIVER ID ADDRESS IS SENT IN HEX (rather than Decimal)

#ifdef RCVR_ADDR_IN_DECIMAL // // Receiver address is in Decimal
#define 999_EQ_ALL_RXS   //if this line is active, Receiver ID causes all receivers (w/ correct Customer ID) to obey
#else	// Receiver address is in Hex
//#define FFF_EQ_ALL_RXS   //if this line is active, Receiver ID causes all receivers (w/ correct Customer ID) to obey
#endif

#define ALL_TRANSISTORS // this should be active (i.e. not commented out) if the circuit has Q2 thru Q9
                        // it should be commented out if only Q1 is present

#define 14V_OPERATION   // this should be active (i.e. not commented out) if circuit has been configured for >9V operation

//#define LOW_SENSITIVITY_AND_CURRENT // this should be active (i.e. not commented out) if circuit is required to work
									  // in low-current mode (1.8mA vs 3mA, when RFM is enabled). Note the RFM loses
									  // ~5dB sensitivity in this mode.

// #define CONFIG_IO_AS_IP_ON_POWERUP   // if this line is active, on power-up all i/o is set to input
// #define DISABLE_SCHEDULE_ON_POWERUP  // if this line is active, scheduling is lost on loss of power

#define IN_CCT_PRGMING_NO_HARDWARE_CHANGES_REQD

#define PS_VOLTAGE_MAX_SETTING (unsigned long)169 // this is the highest Vtx is allowed to be set in the EEPROM (169 corresponds to about 4.5 volts)
#define PS_VOLTAGE_MAX (unsigned long)192 // corresponds to about 5.5 V--- This is the highest Vtx is allowed to go before shutdown
#define PS_VOLTAGE_HYSTERESIS (unsigned long)2
#define PS_VOLTAGE_HYSTERESIS_2 (unsigned long)5