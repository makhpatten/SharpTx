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
#define     COMMON_900_R1	0b00000000
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
//#define       COMMON_900_N1	   0b00010001 // "B-1 (fixed portion)" on spreadsheet (previous version)
#define       COMMON_900_N1	   0b00010101 // "B-1 (fixed portion)" on spreadsheet (as per Bill 11-12-12

/////////////////////////////////////////////////////////
// This is the original frequency data
//------------------ Frequency Data (n2/n3/r2/r3 ------------------
//
//			900 MHz band, PLL Reference=2.048 MHz
//			N2	N3	R2	R3	N	Rdiv	Fo
//
//*********************************************************************
//    			   Rcnt        Ncnt
//             2nd  3rd   2nd    3rd
//             byt  byte  byte	 byte
// CH	Fo
// #	MHz
//
// 1	899.072	00    A0	   12	   61
// 2	903.168	00	  A0	   13	   A1
// 3	907.264	00	  A0	   14	   E1
// 4	911.36	00	  A0	   16	   21
// 5	915.456	00	  A0	   17	   61
// 6	919.552	00	  A0	   18	   A1
// 7	923.648	00	  A0	   19	   E1
// 8	927.744	00	  A0	   1B	   21
//
//	NOTE: These bytes are sent 'raw' to LMX2316 PLL. They include steering
//	      bits n3:1,0 and r3:1,0 which select proper PLL register to load.

//BYTE CONST R_900_BYTE2[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
//BYTE CONST R_900_BYTE3[8] = {0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0};
//BYTE CONST N_900_BYTE2[8] = {0x12,0x13,0x14,0x16,0x17,0x18,0x19,0x1B};
//BYTE CONST N_900_BYTE3[8] = {0x61,0xA1,0xE1,0x21,0x61,0xA1,0xE1,0x21};

/////////////////////////////////////////////////////////
// This is the new R and N counters as per Bill 11-12-12
//------------------ Frequency Data (n2/n3/r2/r3 ------------------
//
//			900 MHz band, PLL Reference=2.048 MHz
//			N2	N3	R2	R3	N	Rdiv	Fo
//
//*********************************************************************
//    			   Rcnt        Ncnt
//             2nd  3rd   2nd    3rd
//             byt  byte  byte	 byte
// CH	Fo
// #	MHz
//
// 1	899.072	03    7C	   F9	   A5
// 2	903.168	03	  48	   A7	   09
// 3	907.264	03	  20	   68	   61
// 4	911.36	03	  00	   37	   01
// 5	915.456	02	  E0	   05	   21
// 6	919.552	02	  DC	   03	   DD
// 7	923.648	02	  D8	   02	   89
// 8	927.744	02	  D4	   01	   25
//
//	NOTE: These bytes are sent 'raw' to LMX2316 PLL. They include steering
//	      bits n3:1,0 and r3:1,0 which select proper PLL register to load.

BYTE CONST R_900_BYTE2[8] = {0x03,0x03,0x03,0x03,0x02,0x02,0x02,0x02};
BYTE CONST R_900_BYTE3[8] = {0x7C,0x48,0x20,0x00,0xE0,0xDC,0xD8,0xD4};
BYTE CONST N_900_BYTE2[8] = {0xF9,0xA7,0x68,0x37,0x05,0x03,0x02,0x01};
BYTE CONST N_900_BYTE3[8] = {0xA5,0x09,0x61,0x01,0x21,0xDD,0x89,0x25};


