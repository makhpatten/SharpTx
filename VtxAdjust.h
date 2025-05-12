int VtxInitializedFlag=0;
int VtxInitialized=0;

void VtxAdjustInitialize() {
      int Vbatt_tenths_of_volts;
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
       delay_msec(10);
       Vbat_ADC = read_adc(); //read the battery voltage
       GnSubData[BATTVOLT_BYTE] = ((Vbat_ADC>>2)&0xff)+4;        // read the battery voltage (circuit resistor ratio=.1, add 4 for .4v drop across diode)
//       if(Vbat_ADC < 117) { // 117: 3.15 Volts
       if(Vbat_ADC < Vbat_min_voltage) { // 117: 3.15 Volts
//       if(Vbat_ADC < 108) { // 108: 2.9 Volts
          return; // return without setting initialized flag
       }

       Vbatt_tenths_of_volts=GnSubData[BATTVOLT_BYTE];
       if (Vbatt_tenths_of_volts<50) {
         Vtx_PWM_Duty=240;
       } else if (Vbatt_tenths_of_volts<60) {
         Vtx_PWM_Duty=200;
       } else if (Vbatt_tenths_of_volts<70) {
         Vtx_PWM_Duty=137;
       } else if (Vbatt_tenths_of_volts<80) {
         Vtx_PWM_Duty=112;
       } else if (Vbatt_tenths_of_volts<90) {
         Vtx_PWM_Duty=96;
       } else if (Vbatt_tenths_of_volts<100) {
         Vtx_PWM_Duty=76;
       } else {
         Vtx_PWM_Duty=10;
       }

      output_low(VTX_PWR_CTL); //need to force low before power applied

      setup_timer_2(T2_DIV_BY_1,PR2,1);   // For 32MHz clk PR2 = 127, for pwm period of 62.5KHz
      set_pwm2_duty (Vtx_PWM_Duty);
       // Turn on the Tx regulators

      output_high(VTX_PWR_CTL); 
      setup_CCP2 (CCP_PWM);
      return;
}

int toggle=0;
int startRegulating;
int prevOnOff=0;
int toggleHere=0;

void adjustVtxBias() {
	unsigned long PS_Voltage;
	int pot_init_value = 0x00;

//#if USE_SHAFT_ENCODER_A_FOR_TEST==1)
   if (toggleHere==1) {
      output_low(ENC_CH_A);//prove that it got here, measure on pin 3 of nanolatch
      toggleHere=0;
   } else {
      output_high(ENC_CH_A);//prove that it got here, measure on pin 3 of nanolatch
      toggleHere=1;
   }
//#endif
	if (bOn_Off==OFF && bOn_OFF!=prevOnOff ) { // if just turned off...
        Tx_OntoOffF(); // call Dan's "state maching" function to turn off.
        prevOnOff=bOn_Off; // save prev ON/OFF state
		return;
	}   
	if (bOn_Off==ON && bOn_OFF!=prevOnOff ) { // if just turned on...
        VtxAdjustInitialize(); // call Mark's new initialization function
        prevOnOff=bOn_Off; // save prev ON/OFF state
		return;
	}   

    prevOnOff=bOn_Off; // save prev ON/OFF state
	set_adc_channel(2);  //set AD Channel 
	delay_msec(20);
	Vbat_ADC = read_adc(); //read the battery voltage

	if (Vbat_ADC > 119) {  //Check that battery voltage is greater than 3.3 volts (used to be "123" but measured values make me think 121 is closer to 3.3 V
       startRegulating=1;
    }
/*
	if (Vbat_ADC < 116) {  //Check that battery voltage is greater than 3.3 volts (used to be "123" but measured values make me think 121 is closer to 3.3 V
       startRegulating=0;
    }
*/   
    if (startRegulating==1) {
		InitVtx();

#if USING_PIC18LF26K22==1
		setup_adc_ports(sAN0|sAN1|sAN2 , VSS_VDD);  // AN1 is used to read power supply, A/D range is 0 - Vdd
#else
		setup_adc_ports(AN0_TO_AN2 | VSS_VDD);  // AN1 is used to read power supply
#endif
		setup_adc(ADC_CLOCK_DIV_32 | ADC_TAD_MUL_20); // A/D clock source period = 1us;
                                                    // Tacq = 18us [Rsource = 90 Kohms 
                                                    // Acquisition time = 20 x 1us = 20us (before conversion) (11% safety factor = 20/18)
                                                    // Measurement Time = 20 + (11 x 1) = 31us (sampling + conversion)
		set_adc_channel(1);  //set AD Channel 
		delay_msec(10);
		PS_Voltage = read_adc();      // read the voltage   NB Must have a '#device adc=10' to force A/D to 10-bit operation 
	
      //Increase Duty factor if voltage is less  than 2.9v
		if (PS_Voltage < PS_Voltage_Target - Vtx_dead_zone_radius) { // For [6A, 88] and hysteresis=2, resulting voltages=[2.56,3.57}
			if (Vtx_PWM_Duty++ > MAX_VTX_PWM_DUTY) Vtx_PWM_Duty = MAX_VTX_PWM_DUTY;  // VpwmDuty > MAX_VTX_PWM_DUTY (493) is 100% duty factor
			set_pwm2_duty(Vtx_PWM_Duty);
		}

    	//Decrease Duty factor if voltage is greater than 3.1v
		if (PS_Voltage > PS_Voltage_Target + Vtx_dead_zone_radius)  { // For [6A, 88] and hysteresis=2, resulting voltages=[2.70,3.70}
			if (Vtx_PWM_Duty-- < 1) Vtx_PWM_Duty = 0;  //VpwmDuty cannot be less than 0
			set_pwm2_duty(Vtx_PWM_Duty);
		} 

	}


	if (startRegulating==0) { // if battery below 3.1 Volts turn off

		setup_CCP2 (CCP_OFF); //Turn on Tx Power Supply switch since battery voltage is less than 3.3v
		output_high(TX_PWR_PWM_PIN);
		Bat_Lo_Flag = TRUE;
		VtxInitialized=0;
	}

    if (PS_Voltage > PS_VOLTAGE_MAX) { // if Vtx is too high (an error condition since it's not regulating)
	}



#if USE_NEW_VOLTAGE_CTL_WITH_TX_POWER==1
       if (PS_Voltage > PS_VOLTAGE_MAX) {
#endif

	}
}


void InitVtx() {
	if (VtxInitialized!=0) return;
	port_b_pullups(ON);
	delay_msec(1);

	Init_Synth();  //initialize the external Synthesize
	delay_msec(1);
	remoteChannel_Set = remoteChannel_Select;
	Channel = 7-remoteChannel_Select;
	updatePSVoltageTarget();
	Change_Frequency();  //set frequency to channel
	Init_extAD();  //initialize the external A/D
	set_data_speed_and_polarity(); // set the data speed and polarity in the FPGA
	enable_interrupts(INT_EXT); //enable FPGA SubClock interrupt
	enable_interrupts(INT_EXT1);   //enables external encoder

//Reset FPGA
 
	output_high(LEFT_PIN); 
	output_high(RIGHT_PIN); 
	delay_msec(2);
	output_high(LEFT_PIN); 
	output_low(RIGHT_PIN); 
	delay_msec(2);

	if ( GbAudio_Mode == STEREO) {
		output_high(LEFT_PIN); 
		output_high(RIGHT_PIN); 
	} else {
		if (GbAudio_Left) {
			output_high(LEFT_PIN);
			output_low(RIGHT_PIN);
		} else {
			output_low(LEFT_PIN);
			output_high(RIGHT_PIN);
		}
	}

//  Start the subclk as it stays high indicating the PIC should send Subchannel data
//  but the interrupt for sending the subchannel data is rising edge triggered.  To get the process
//  started a pulse is sent to the FPGA

	output_high(SUBDATA);
	output_low(SUBDATA);
	VtxInitialized=1;

}        
