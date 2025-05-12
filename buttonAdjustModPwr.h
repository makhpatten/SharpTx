#ROM int8 0XF00065    = {0x00, 0x00, 0x00}  // Initialize button press counters to 0

int buttonActionTaken=0;
unsigned int button1counter=0;
unsigned int button2counter=0;
int button1pushed=0;
int button2pushed=0;
unsigned int excludeBothTimer;
#define BUTTON_DEBOUNCE_TIME 50   // corresponds to 1 mS
#define BUTTON_EXCLUDE_TIME 2500 // corresponds to 50 mS
#define DELAY_BUTTON_TIME (unsigned long)300
#define BUTTON_REPEAT_TIME 50
int adjustModOrPower=0;

extern char Channel;
extern unsigned int Power_Setting;
extern BOOLEAN set_pot(unsigned int nValue, nPot_Number);
unsigned int biasMod_EEPROM_addr;
unsigned int biasModValue;
void adjustBiasFromButton();
unsigned long delayButtonCounter=0;
unsigned long buttonRepeatCounter=0;
int enableButtons=0;
unsigned int buttonSequence=0;
int upDownUpDown=0;
int toggleHere=0;

void scan_pwr_mod_adjust_buttons() {

   if (!input(PIN_B6)) { // if button 1 pushed increment its counter
      if (buttonActionTaken==0 && button1pushed==0) { // make sure button was released before allowing counter to increment
         button1counter++;
      }

   } else {
      button1counter=0;  // otherwise reset counter
      button1pushed=0;
   }
   if (!input(PIN_B7)) { // if button 2 pushed increment its counter
      if (buttonActionTaken==0 && button2pushed==0) { // make sure button was released before allowing it to be pushed
   	      button2counter++;
      }   
   } else {
      button2counter=0;  // otherwise reset counter
      button2pushed=0;
   }

   if (input(PIN_B6) && input(PIN_B7)) { // if both buttons not pushed
      if (buttonActionTaken==1) {
         buttonActionTaken=0;
         write_eeprom(biasMod_EEPROM_addr,biasModValue);
      }
      excludeBothTimer=0;
      delayButtonCounter=0;
      buttonRepeatCounter=0;
   }

   if (button1counter>BUTTON_DEBOUNCE_TIME) { // if button 1 held down long enough, 
      button1pushed=1; // consider it pushed
   }
   if (button2counter>BUTTON_DEBOUNCE_TIME) { // if button 2 held down long enough, 
      button2pushed=1; // consider it pushed
   }

   if (upDownUpDown>=4) {
      enableButtons=1;

   }

   if ((button1pushed==1 || button2pushed==1)) {
      if (buttonActionTaken==0) { // if haven't taken action yet
         if (button1pushed==1 && button2pushed==1) { // if both buttons pushed at same time
            buttonActionTaken=1;
            if (adjustModOrPower==1) {
               adjustModOrPower=0;
            } else {
               adjustModOrPower=1;
            }
// toggle between adjusting mod width and adjusting power
         } else { // a single button is pushed
            excludeBothTimer++; // otherwise increment exclusion timer
            if (excludeBothTimer>BUTTON_EXCLUDE_TIME) { // button has been held long enought to exclude a double button push
               buttonActionTaken=1; // indicate action taken
               if (enableButtons==1) {
                  adjustBiasFromButton();
//                  write_eeprom(biasMod_EEPROM_addr,biasModValue);
               } else {
                  if (button1pushed==1) {
                    if (upDownUpDown==0 || upDownUpDown==2) {
                       upDownUpDown++;
                    } else {
                       upDownUpDown=0;
                    }
                  }
                  if (button2pushed==1) {
                    if (upDownUpDown==1 || upDownUpDown==3) {
                       upDownUpDown++;
                    } else {
                       upDownUpDown=0;
                    }
                  }
               }
            }  // end if single button held past exclude time
         } // end if a single button pushed
      } else { // action has already been taken
#if 1==0
         if (delayButtonCounter>DELAY_BUTTON_TIME) {
            buttonRepeatCounter++;
            if (buttonRepeatCounter>BUTTON_REPEAT_TIME){
               buttonRepeatCounter=0;
               adjustBiasFromButton();
            }   
         } else {
            delayButtonCounter++;
         }
#endif
      } // end if no action taken yet
          
   } // end if either button pushed
}

void adjustBiasFromButton() {
   unsigned int nTemp;

   if (adjustModOrPower==1) {
      biasMod_EEPROM_addr = BIAS_VALUE_BASE_ADDRESS;
   } else {
      biasMod_EEPROM_addr = MODINDX_VALUE_BASE_ADDRESS;
   }
   biasMod_EEPROM_addr+=Channel;
   if (Power_Setting==0) {
       biasMod_EEPROM_addr+=8;
   }
   biasModValue=read_eeprom(biasMod_EEPROM_addr);
   if (button1pushed==1) {
      biasModValue--;
   } else {
      biasModValue++;
   }
   if (adjustModOrPower==1) {
     set_pot(biasModValue, POT0);
   } else {
      if(GbAudio_Mode == STEREO) {
         set_pot(biasModValue, POT1);	//for STEREO MODE
     } else {                                            //for MONO MODE...
         nTemp = 0xff - biasModValue;
         shift_right(&nTemp,1,0);
         nTemp = biasModValue + nTemp;
         set_pot(nTemp, POT1);	//for MONO MODE
     } 
  }
}