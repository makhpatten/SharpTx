#define REG0   0
#define REG1   1
#define REG2   2
#define REG3   3

#define D7     7
#define D6     6
#define D5     5
#define D4     4
#define D3     3
#define D2     2
#define D1     1
#define D0     0

//Address 00H
//---------- set/initialize A/D (AKM5356) serial data ----------
//
//A/D Input Select register:
//                 		  7----------- Always 0
//                		  |6---------- PREG 1   00=+13dB, 01=+18dB, 10=+28dB (default), 11=+33dB
//                		  ||5--------- PREG 0   "     "  "  '  "  "  "  "  '     '" "     '" "
//                		  |||4-------- HPF      0= On, 1=Off   default=0 (Off)
//                		  ||||3------- Rin      1= On, 0=Off   default=0 (Off)
//                		  |||||2------ MICR     1= On, 0=Off   default=1 (On)
//                     	  ||||||1----- Lin      1= On, 0=Off   default=0 (Off)
//			                 |||||||0---- MICL     1= On, 0=Off   default=1 (On)
//			                 ||||||||
#define AD_IPSLCTDATA   0b01000101 // chip default = 0b01000101

//Power Management Control register:
//Address 01H
//                 		  7----------- Always 0
//                		  |6---------- Always 0
//                		  ||5--------- Always 0
//                		  |||4-------- Always 0
//                		  ||||3------- Always 0
//                		  |||||2------ Always 0
//                     	  ||||||1----- PMADC - Power Management of IPGA & ADC 0=Off, 1=On (Default)
//			                 |||||||0---- PMMIC - Power Management of Mic Block 0=Off, 1=On (Default)
//			                 ||||||||
#define AD_PWRMNGCTL    0b00000011 // chip default = 0b00000011

//A/D Mode Control register:
//Address 02H
//                 		  7----------- MONO1 - 00=Stereo(default), 01=(L+R)/2, 10=LL, 11=RR
//                		  |6---------- MONO0 -  "  "  "  "  "  "  "  "  "  "  "  "  "  "  "
//                		  ||5--------- ZTM1  - Setting of zero crossing timeout for IPGA
//                		  |||4-------- ZTM0    (cont'd) 00=256/fs, 01=512/fs, 10=1024/fs, 11=2048/fs (default)
//                		  ||||3------- Always 0
//               			  |||||2------ Always 0
//                  	     ||||||1----- DIF   - Digital Interface Format - see datasheet, default = 0
//			                 |||||||0---- Always 0
//			                 ||||||||
#define AD_MODECTL      0b00110000 // chip default = 0b00110000

//A/D input Analog PGA Control register:
//Address 03H
//                 		  7----------- ZEIP  - Zero Crossing Operation, 0=Disable(default, 1=Enable)
//                		  |6---------- IPGA6 - IPGA setting, default =28H = 101000
//                		  ||5--------- IPGA5    "
//                		  |||4-------- IPGA4    "
//                		  ||||3------- IPGA3    "
//                		  |||||2------ IPGA2    "
//                     	  ||||||1----- IPGA1    "
//			                 |||||||0---- IPGA0    "
//			                 ||||||||
//#define AD_IPGACTL      0b00101000 // chip default = 0b00101000
//#define AD_IPGACTL      0b00011010 // IPGA Gain = -7dB (to make equivalent to 1st gen transmitter)
#define AD_IPGACTL      0b00100100 // IPGA Gain = -2dB (to make equivalent to 2nd gen transmitter)

#define C1     1  // Chip address
#define C0     0  // Chip address
#define RW     1  //Write only
