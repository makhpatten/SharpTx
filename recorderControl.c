void outputRecStateOnSubchan();
void processSpecialremoteCmd();
void flushInBuf();
void putcharx(unsigned int x);
unsigned long extract_data_from_message(unsigned int nNibble_Offset);
int waitForComResponse(char responseArray[],int expectedNumBytes);
void Init_extAD(int gainMode,int inputMode, int pwrMode);
void Init_Synth(int FoLDcode, int pwrDown);
static unsigned int GnSubData[SUBDATA_MSG_LNGTH], GnSubData_Bit_Index = 0, GnSubData_Byte_Index = 0;
char popCharFromInBuff();
void readMclrSetADSource();
int picOnRegsOffMode=0;

unsigned int specialremoteMsg[5];
int specialremoteMsgCode;
#define RX_BUFF_SIZE_MAX 32
int8 *rcreg;
char rx_contents;
int8 rxBufSize=0;
int8 rxBufIndex=0;
char inRxBuff[RX_BUFF_SIZE_MAX];
int8 rxBufOverflow=0;
int8 rxBufReadIndex=0;
char comResponseArray[10];
#define FLYRON_MODE_STOPPED 0
#define FLYRON_MODE_RECORDING 1
#define FLYRON_MODE_PLAYING 2
int8 flyronMode=FLYRON_MODE_STOPPED;

#byte SPBRG=0x0FAF
#byte SPBRGH=0x0FB0
#byte RCSTA=0x0FAB
#byte TXSTA=0x0FAC
#byte BAUDCON=0x0FB8
#byte TXREG=0x0FAD


void putcharx(unsigned int x) {
  char oneChar;
#if CONFIG_UART_IN_HARDWARE==1
  oneChar=TXSTA;
  while ((oneChar&0x02)==0) {
     oneChar=TXSTA;
  }
  TXREG=x;
#else
   putchar(x);
#endif
}

int doublecmdlockouttimer;

int determine_special_remote_cmd() {
   unsigned long  lData;
   unsigned int oneByteXX, oneByteYY;
   lData = extract_data_from_message(LOGIC_MODE_CONTROL_NIBBLE_INDEX); // returns a 16-bit result
                                                                       // only upper bits wanted
   oneByteXX = make8(lData,1);  // extract the upper byte of the result   
   oneByteXX&=0xF0;
   if (oneByteXX==0) { // input on bit 3 (i.e. "line 8") of nibble 3 indicates a special remote command
      return 0; // it's just a regular remote command
   } else {

//      putcharx(oneByteXX);
      if (doublecmdlockouttimer>0) {
        return 2; // lockout to prevent double commands
      }
      doublecmdlockouttimer=3;
      oneByteXX>>=4;
      oneByteXX&=0x07;
      switch(oneByteXX) {
         case 0: // generic special remote command
         default:
            specialremoteMsgCode='S';
            break;
         case 1: // generic special remote command
            specialremoteMsgCode='P';
            break;
         case 2: // generic special remote command
            specialremoteMsgCode='L';
            break;
         case 3: // generic special remote command
            specialremoteMsgCode='R';
            break;
         case 4: // generic special remote command
            specialremoteMsgCode='V';
            break;
         case 5: // generic special remote command
            specialremoteMsgCode='N';
            break;
         case 6: // generic special remote command
            specialremoteMsgCode='D';
            break;
         case 7: // generic special remote command
            specialremoteMsgCode='X';
            break;
      }
//      putcharx(specialremoteMsgCode);

      lData = extract_data_from_message(4); // returns a 16-bit result
      oneByteXX=make8(lData,1);
//      putcharx(oneByteXX);
      specialremoteMsg[0]=oneByteXX;

//      specialremoteMsgCode=oneByteXX;

      lData = extract_data_from_message(6); // returns a 16-bit result
      oneByteXX=make8(lData,1);
//      putcharx(oneByteXX);
      specialremoteMsg[1]=oneByteXX;

      lData = extract_data_from_message(8); // returns a 16-bit result
      oneByteXX=make8(lData,1);
//      putcharx(oneByteXX);
      specialremoteMsg[2]=oneByteXX;

      lData = extract_data_from_message(10); // returns a 16-bit result
      oneByteXX=make8(lData,1);
//      putcharx(oneByteXX);
      specialremoteMsg[3]=oneByteXX;

      lData = extract_data_from_message(12); // returns a 16-bit result
      oneByteXX=make8(lData,1);
      oneByteYY=oneByteXX&0xf0;

      lData = extract_data_from_message(14); // returns a 16-bit result
      oneByteXX=make8(lData,1);
      oneByteYY|=oneByteXX&0x0f;

//      putcharx(oneByteYY);
      specialremoteMsg[4]=oneByteYY;

      return 1;
   }
}


extern unsigned int   nElapsed_Second;
//extern unsigned int GnSubData[];
unsigned int   savednElapsed_Second;
int outputRecStateCounter=0;
int outputRecState=0;
void querryRecState();
int secondCounter=0;
extern int1 bOn_Off;

void outputRecStateOnSubchan() {
   char subchan1,subchan2,subchanChk;
   int nValue;

   if (nElapsed_Second!=savednElapsed_Second) {
      savednElapsed_Second = nElapsed_Second;
      secondCounter++;
      if (doublecmdlockouttimer>0) {
         doublecmdlockouttimer--;
      }
      if (secondCounter>1) {
         secondCounter=0;

         if (bOn_Off) {
            if (outputRecState==0) {
               querryRecState();
               outputRecState=1;
            } else {
               outputRecState=0;
               for(nValue=0;nValue<=2;nValue++) {
                  GnSubData[nValue]= read_eeprom((SUBDATA_VALUE_BASE_ADDRESS+nValue));
               }
            }
         }
      }
   }
}

int debugoffset=0;
int adSource=0;

void querryRecState() {
   int comRespBytes;
//   delay_ms(100);
   flushInBuf();
// querry volume
   putcharx(0x7e);
   putcharx(0x03);
   putcharx(0xc1);
   putcharx(0xc4);
   putcharx(0x7e);
   delay_ms(100);
   comRespBytes=waitForComResponse(comResponseArray,2);
//   if (debugoffset>=44) debugoffset=0;
//   write_eeprom(0x70+debugoffset++,comRespBytes);
//   write_eeprom(0x70+debugoffset++,comResponseArray[0]);
//   write_eeprom(0x70+debugoffset++,comResponseArray[1]);
   if (comRespBytes==2) {
     GnSubData[1]=(unsigned int)comResponseArray[1]&0x1f;
     GnSubData[1]|=(unsigned int)0xE0;
   } else {
      GnSubData[1]=0x00;
   }
// querry work status
   flushInBuf();
   putcharx(0x7e);
   putcharx(0x03);
   putcharx(0xc2);
   putcharx(0xc5);
   putcharx(0x7e);
   delay_ms(100);
   comRespBytes=waitForComResponse(comResponseArray,2);

//   if (debugoffset>=44) debugoffset=0;
//   write_eeprom(0x70+debugoffset++,comRespBytes);
//   write_eeprom(0x70+debugoffset++,comResponseArray[0]);
//   write_eeprom(0x70+debugoffset++,comResponseArray[1]);

   if (comRespBytes==2) {
//      GnSubData[0]|=(comResponseArray[1]&0x07)<<5;
      GnSubData[2]=(unsigned int)comResponseArray[1];

#if AD_SOURCE_SET_BY_MCLR==1
      readMclrSetADSource();
#else
      if ((comResponseArray[1]==2 || comResponseArray[1]==3) && adSource!=0) { // if stopped but AD not set to mic input, re-init AD for mic input
         Init_extAD(4,5,3);  //initialize the external A/D to L&R Mic input on (and also as usual L&R Line input off, gain=+28dB, HPF on, ADC & IPGA on, Mic block on)
         adSource=0;
      }
      if (comResponseArray[1]==1 && adSource==0) { // if stopped but AD not set to mic input, re-init AD for mic input
         Init_extAD(4,10,3);  //initialize the external A/D to L&R Line input on (and also as usual L&R Line input off, gain=+28dB, HPF on, ADC & IPGA on, Mic block on)
         adSource=1;
      }
#endif

   } else {
      GnSubData[2]=(unsigned int)0x00;
   }

// querry current connecton status (USB/SD)
   flushInBuf();
   putcharx(0x7e);
   putcharx(0x03);
   putcharx(0xca);
   putcharx(0xcd);
   putcharx(0x7e);
   delay_ms(100);
   comRespBytes=waitForComResponse(comResponseArray,2);
//   if (debugoffset>=44) debugoffset=0;
//   write_eeprom(0x70+debugoffset++,comRespBytes);
//   write_eeprom(0x70+debugoffset++,comResponseArray[0]);
//   write_eeprom(0x70+debugoffset++,comResponseArray[1]);
   if (comRespBytes==2) {
     GnSubData[2]|=((unsigned int)comResponseArray[1]&0x03)<<4;
//   } else {
//      GnSubData[0]=0x00;
   }
//   GnSubData[0]=5; // just a test
//   GnSubData[1]=7; // just a test
   GnSubData[0]=GnSubData[1]^GnSubData[2];
  
}

void processSpecialremoteCmd() {
   int fileNumber;
   char fileNumString[5];
   long checkByteLong;
   char oneByte;
   int val1,val2,val3;

   delay_ms(100);
   flushInBuf();
   switch(specialremoteMsgCode) {
      case 'R': // Record
         flyronMode=FLYRON_MODE_RECORDING;
         if (bOn_Off == OFF) {
            picOnRegsOffMode=1;
            bOn_Off = ON;
            if (Main_Mode.Tx_Mode==0) {
               Main_Mode.Tx_Mode=Tx_OfftoOn_Vtx; //Startup Tx
            }   
            output_low(VTX_PWR_CTL);
            output_low(AD_CHIP_SLCT);
         }
         putcharx(0x7e);  // put audio source command
         putcharx(0x04);
         putcharx(0xd3);
         switch(specialremoteMsg[3]) {
            case 0:
            default:
               putcharx(0x00);
               putcharx(0xd7);
               break;
            case 1:
               putcharx(0x01);
               putcharx(0xd8);
               break;
            case 2:
               putcharx(0x02);
               putcharx(0xd9);
               break;
         }
         putcharx(0x7e);
         delay_ms(100);
 //        waitForComResponse(comResponseArray,1);

         putcharx(0x7e);  // put bit rate command
         putcharx(0x04);
         putcharx(0xd4);
         switch(specialremoteMsg[2]) {
            case 0:
            default:
               putcharx(0x00);
               putcharx(0xd8);
               break;
            case 1:
               putcharx(0x01);
               putcharx(0xd9);
               break;
            case 2:
               putcharx(0x02);
               putcharx(0xda);
               break;
            case 3:
               putcharx(0x03);
               putcharx(0xdb);
               break;
         }
         putcharx(0x7e);
         delay_ms(100);
 //        waitForComResponse(comResponseArray,1);

         putcharx(0x7e);  // put record filemane command
// new way using filename:
         putcharx(0x07);
         putcharx(0xd6);
         putcharx(0x54);
         fileNumber=(unsigned long)specialremoteMsg[0]<<8;
         fileNumber+=(unsigned long)specialremoteMsg[1];

         sprintf(fileNumString,"%03d",fileNumber);

         putcharx(fileNumString[0]);
         putcharx(fileNumString[1]);
         putcharx(fileNumString[2]);
         checkByteLong=((long)fileNumString[0]&0x0FF)+((long)fileNumString[1]&0x0ff)+((long)'T'&0x0ff)+((long)fileNumString[2]&0x0ff)+0x00dd;
         putcharx((char)(checkByteLong&0x0FF));
         putcharx(0x7e);
         delay_ms(100);
  //       waitForComResponse(comResponseArray,1);

						break;
      case 'P':
         flyronMode=FLYRON_MODE_PLAYING;
#if AD_SOURCE_SET_BY_MCLR==1
         adSource=2; // force AD source to be reset
         readMclrSetADSource();
#else
         Init_extAD(4,10,3);  //initialize the external A/D to L&R Line input on (and also as usual L&R Mic input off, gain=+28dB, HPF on, ADC & IPGA on, Mic block on)
         adSource=1; // AD set to line input
#endif

         putcharx(0x7e);  // set volume command
         putcharx(0x04);
         putcharx(0xae);
         putcharx(specialremoteMsg[2]&0x1f);
         checkByteLong=((long)specialremoteMsg[2]&0x01f)+0x00b2;
         putcharx((char)(checkByteLong&0x0FF));
         putcharx(0x7e);
         delay_ms(100);
//         waitForComResponse(comResponseArray,1);

         putcharx(0x7e);  // playback command
         putcharx(0x07);
         putcharx(0xa3);
         putcharx(0x54);
         fileNumber=(unsigned long)specialremoteMsg[0]<<8;
         fileNumber+=(unsigned long)specialremoteMsg[1];

         sprintf(fileNumString,"%03d",fileNumber);

         putcharx(fileNumString[0]);
         putcharx(fileNumString[1]);
         putcharx(fileNumString[2]);
         checkByteLong=((long)fileNumString[0]&0x0FF)+((long)fileNumString[1]&0x0ff)+((long)'T'&0x0ff)+((long)fileNumString[2]&0x0ff)+0x00aa;
         putcharx((char)(checkByteLong&0x0FF));
         putcharx(0x7e);

         delay_ms(100);
//         waitForComResponse(comResponseArray,1);

						break;
      case 'S':
         switch(specialremoteMsg[0]) {
            case 0: // stop
            default:
               flyronMode=FLYRON_MODE_STOPPED;
               if (picOnRegsOffMode==1) {
                  picOnRegsOffMode=0;
                  bOn_Off = OFF;
                  if (Main_Mode.Tx_Mode>0) Main_Mode.Tx_Mode=Tx_OntoOff;  //Turn off TX
               }

               putcharx(0x7e);
               putcharx(0x03);
               putcharx(0xd9);
               putcharx(0xdc);
               putcharx(0x7e);
               delay_ms(100);
//               waitForComResponse(comResponseArray,1);

               break;
            case 1: // querry current volume level
               putcharx(0x7e);
               putcharx(0x03);
               putcharx(0xc1);
               putcharx(0xc4);
               putcharx(0x7e);
               delay_ms(100);
 //              waitForComResponse(comResponseArray,2);
               break;
            case 2: // querry work status
               putcharx(0x7e);
               putcharx(0x03);
               putcharx(0xc2);
               putcharx(0xc5);
               putcharx(0x7e);
               delay_ms(100);
 //              waitForComResponse(comResponseArray,2);
//outputComRespOnSubchannel();

               break;
            case 3: // querry num files in root directory
               putcharx(0x7e);
               putcharx(0x03);
               putcharx(0xc5);
               putcharx(0xc8);
               putcharx(0x7e);
               delay_ms(100);
 //              waitForComResponse(comResponseArray,3);
               break;
            case 4: // querry current sound file being played
               putcharx(0x7e);
               putcharx(0x03);
               putcharx(0xc9);
               putcharx(0xcc);
               putcharx(0x7e);
               delay_ms(100);
//               waitForComResponse(comResponseArray,3);
               break;
            case 5: // querry current connecton status (USB/SD)
               putcharx(0x7e);
               putcharx(0x03);
               putcharx(0xca);
               putcharx(0xcd);
               putcharx(0x7e);
               delay_ms(100);
//               waitForComResponse(comResponseArray,2);
               break;
            case 6: // querry space left on device
               putcharx(0x7e);
               putcharx(0x03);
               putcharx(0xce);
               putcharx(0xd1);
               putcharx(0x7e);
               delay_ms(100);
 //              waitForComResponse(comResponseArray,3);
               break;
            case 7: // toggle fast forward
               putcharx(0x7e);
               putcharx(0x03);
               putcharx(0xd0);
               putcharx(0xd3);
               putcharx(0x7e);
               delay_ms(100);
//               waitForComResponse(comResponseArray,1);
               break;
            case 8: // toggle fast backward
               putcharx(0x7e);
               putcharx(0x03);
               putcharx(0xd1);
               putcharx(0xd4);
               putcharx(0x7e);
               delay_ms(100);
 //              waitForComResponse(comResponseArray,1);
               break;
            case 0xff: // stop playback
#if AD_SOURCE_SET_BY_MCLR==1
               adSource=2; // force AD source to be reset
               readMclrSetADSource();
#else
               Init_extAD(4,5,3);  //initialize the external A/D to L&R Mic input on (and also as usual L&R Line input off, gain=+28dB, HPF on, ADC & IPGA on, Mic block on)
               adSource=0; // AD set to mic input
#endif
               putcharx(0x7e);
               putcharx(0x03);
               putcharx(0xab);
               putcharx(0xae);
               putcharx(0x7e);
               delay_ms(100);
 //              waitForComResponse(comResponseArray,1);
               break;
         }
						break;
/*
      case 'L':
         playOrLiveCmdFlag=1;
         Init_extAD(0);
         bOn_Off = ON;
         if (Main_Mode.Tx_Mode==0) {
            Main_Mode.Tx_Mode=Tx_OfftoOn_Vtx; //Startup Tx
         }
         break;
*/
      case 'X':
         if (specialremoteMsg[0]=='F'&& specialremoteMsg[1]=='o' && specialremoteMsg[2]=='L' && specialremoteMsg[3]=='D') {
            Init_Synth(specialremoteMsg[4]-'0',0);
         }
         if (specialremoteMsg[0]=='A'&& specialremoteMsg[1]=='D') {
            oneByte=specialremoteMsg[2];
            if (oneByte>='0' && oneByte<='9') {
               val1=oneByte-'0';
            } else if (oneByte>='A' && oneByte<='F') {
               val1=oneByte-'A'+10;
            } else {
               val1=15;;
            }
            oneByte=specialremoteMsg[3];
            if (oneByte>='0' && oneByte<='9') {
               val2=oneByte-'0';
            } else if (oneByte>='A' && oneByte<='F') {
               val2=oneByte-'A'+10;
            } else {
               val2=15;;
            }
            oneByte=specialremoteMsg[4];
            if (oneByte>='0' && oneByte<='9') {
               val3=oneByte-'0';
            } else if (oneByte>='A' && oneByte<='F') {
               val3=oneByte-'A'+10;
            } else {
               val3=15;;
            }
            Init_extAD(val1,val2,val3);
         }
         if (specialremoteMsg[0]=='S'&& specialremoteMsg[1]=='Y' && specialremoteMsg[2]=='P' && specialremoteMsg[3]=='D') {
            Init_Synth(0,specialremoteMsg[4]-'0');
         }
         break;
      case 'D': // delete a file
         putcharx(0x7e);  // delete command
         putcharx(0x07);
         putcharx(0xdb);
         putcharx(0x54);
         fileNumber=(unsigned long)specialremoteMsg[0]<<8;
         fileNumber+=(unsigned long)specialremoteMsg[1];

         sprintf(fileNumString,"%03d",fileNumber);

         putcharx(fileNumString[0]);
         putcharx(fileNumString[1]);
         putcharx(fileNumString[2]);
         checkByteLong=((long)fileNumString[0]&0x0FF)+((long)fileNumString[1]&0x0ff)+((long)'T'&0x0ff)+((long)fileNumString[2]&0x0ff)+0x00e2;
         putcharx((char)(checkByteLong&0x0FF));
         putcharx(0x7e);

         delay_ms(100);
         delay_ms(100);
 //        waitForComResponse(comResponseArray,1);

         break;
   }
//   querryRecState();
//   outputRecState=1;
//   secondCounter=0;

}

void flushInBuf() {
   while (rxBufSize>0) {
      popCharFromInBuff();
   }
}


char popCharFromInBuff() {
   char oneChar;

   disable_interrupts(int_rda);
   oneChar=inRxBuff[rxBufReadIndex++];
   if (rxBufReadIndex>=RX_BUFF_SIZE_MAX) {
      rxBufReadIndex=0;
   }
   rxBufSize--;
   enable_interrupts(int_rda);

   return(oneChar);
}

int waitForComResponse(char responseArray[],int expectedNumBytes) {
   unsigned long loopTimeoutCounter;
   int bytesRead;

   bytesRead=0;
   loopTimeoutCounter=0;

   while (bytesRead<expectedNumBytes && loopTimeoutCounter<100) {
      if (rxBufSize>0) {
         responseArray[bytesRead]=popCharFromInBuff();
         bytesRead++;
         loopTimeoutCounter=0;
      } else {
         delay_ms(5);
         loopTimeoutCounter++;
      }
   }
   return(bytesRead);
}

void readMclrSetADSource() {
   if (input(PIN_E3)!=0) {
      if (adSource!=1) {
         Init_extAD(4,10,3);  //initialize the external A/D
         adSource=1;
      }
   } else {
      if (adSource!=0) {
        Init_extAD(4,5,3);  //initialize the external A/D
        adSource=0;
      }
   }
}
