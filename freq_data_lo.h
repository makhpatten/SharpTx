//---------- set/initialize LMX2316 serial data ----------
//

/*	The Fo/LD (pin 14, LX23*6) Output Truth Table
	F5	F4	F3	Fo/LD Output State  (NB BIT ORDER IS REVERSED w.r.t. TABLE 4 in datasheet
	0	0	0	TRI-STATE
	0	0	1	Digital lock Detect
	0	1	0	N Divider Output (fp)
	0	1	1	Active High
	1	0	0	R Divider Output (fr)
	1	0	1	nChannel Open drain lock detect
	1	1	0	Serial Data Output
	1	1	1	Active Low
*/
// R Counter register:
// 3 MSbits IGNORED + LD Prec.=0 + Test Mode=0000
// Next 2 bytes contain the 14-bit R count + control bits
//			                    4----------- R19 0 = easy LD precision (enter w 3 ref cycles)
//			                    |3---------- R18 0 = normal use (test modes)
//			                    ||2--------- R17 0 = normal use (test modes)
//			                    |||1-------- R16 0 = normal use (test modes)
//			                    ||||0------- R15 0 = normal use (test modes)
//			                    |||||
#define     COMMON_360_R1	0b00000000
//
// N Counter register, 1st byte:
// 3MSbits IGNORED + GO bit=1 + 4MSB of B counter
//
//			                    4----------- N19 1 = GO bit high for fast QSY
//			                    |3---------- N18 1 = +4096 (B counter b13)
//			                    ||2--------- N17 1 = +2048 (B counter b12)
//			                    |||1-------- N16 1 = +1024 (B counter b11)
//			                    ||||0------- N15 1 = +512  (B counter b10)
//			                    |||||
#define     COMMON_360_N1	0b00010000 // "B-1 (fixed portion)" on spreadsheet

/////////////////////////////////////////////////////////
//
//------------------ Frequency Data (n2/n3/r2/r3 ------------------
//
//			362.496 - 391.168 MHz band, PLL Reference=2.048 MHz
//			N2	N3	R2	R3	N	Rdiv	Fo
//
//*********************************************************************

//    			   Rcnt        Ncnt
//             2nd  3rd   2nd    3rd
//             byt  byte  byte	 byte
// CH	Fo
// #	MHz
//
// 1	362.496	00   A0	   6E    A1
// 2	366.592	00	  A0	   6F    E1
// 3	370.688	00	  A0	   71    21
// 4	374.784	00	  A0	   72    61
// 5	378.880	00	  A0	   73    A1
// 6	382.976	00	  A0	   74    E1
// 7	387.072	00	  A0	   76    21
// 8	391.168	00	  A0	   77    61
//
//	NOTE: These bytes are sent 'raw' to LMX2316 PLL. They include steering
//	      bits n3:1,0 and r3:1,0 which select proper PLL register to load.

BYTE CONST R_360_BYTE2[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
BYTE CONST R_360_BYTE3[8] = {0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0};
BYTE CONST N_360_BYTE2[8] = {0x6E,0x6F,0x71,0x72,0x73,0x74,0x76,0x77};
BYTE CONST N_360_BYTE3[8] = {0xA1,0xE1,0x21,0x61,0xA1,0xE1,0x21,0x61};

