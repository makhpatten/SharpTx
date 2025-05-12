
//				This file configured for SharpTx;
//				Mark Patten 12/12/2018




//#define DATA_SPEED    DATA_SPEED_1MB_PER_SEC // Stereo dataspeed:
#define DATA_SPEED    DATA_SPEED_2MB_PER_SEC // Stereo dataspeed:
                                             // Choices: DATA_SPEED_2MB_PER_SEC, DATA_SPEED_1MB_PER_SEC
                                             // NB: Mono mode is half whatever speed is chosen

#define DATA_POLARITY DATA_NONINVERTED       // Choices: DATA_NONINVERTED, DATA_INVERTED
//#define DATA_POLARITY DATA_INVERTED       // Choices: DATA_NONINVERTED, DATA_INVERTED

// #define NO_DIGITAL_POTENTIOMETER	// Used if Dig Pot is NOT present. Code will still work if Dig Pot not present and this line is
									// not commented out - it's mainly here for the purposes of tidy code

// #define COMMON_CHANNEL_BIAS		//If commented out, individual channel setting of amplifier bias is enabled

//////////////////////////////////////////////////////////////////////////////////////////
// ---------------------------------- SELECT FREQUENCY RANGE------------------------------
//
//                            Comment out ALL lines EXCEPT that required.

// #define HIFREQ_MHz
#define LOFREQ_MHz

//////////////////////////////////////////////////////////////////////////////////////////
