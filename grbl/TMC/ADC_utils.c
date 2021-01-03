/*
 * ADC_utils.c
 *
 * Created: 27/09/2020 23:34:10
 *  Author: Grassbow
 */ 

#include "grbl.h"

#define     FIR_COEFF_TEMPC 					    30		/**< 0-255 defines how quickly FIR converges 255-fastest */
#define     SPINDLE_SPEED_FEEDBACK_N_CONVERGES      10      /* how many times to read and nudge spindle speed signal */
#define     SPINDLE_SIG_MAX_MILLIVOLTS              10300   
#define     SPINDLE_SIG_MIN_MILLIVOLTS              0   
#define     SPINDLE_SIG_CONVERGENCE_DAMPING_FACTOR  0.5
#define		ADC_EXTERNAL_VREF_2048mV					// either ADC_EXTERNAL_VREF_2048mV or ADC_INTERNAL_VREF_1100mV

static uint16_t spindle_load_mV                = 0;            // global variable for latest spindle load value
static uint16_t VDD_5V_Atmega_mV               = 0;            // global variable for latest VDD_5V_Atmega value
static uint16_t VDD_5V_dustshoe_mV             = 0;            // global variable for latest VDD_5V_dustshoe value
static uint16_t VDD_24V_mV                     = 0;            // global variable for latest VDD_24V value
static uint16_t Spindle_speed_Signal_mV        = 0;            // global variable for latest Spindle_speed_Signal_mV value
static uint16_t AC_loss_Signal_mV              = 0;            // global variable for latest AC_loss_Signal_mV value
static uint16_t temperature_TMC_cent_celsius   = 2500;  // global variable for latest temperature value in hundredths of degree celsius
static uint16_t temperature_PCB_cent_celsius   = 2500;  // global variable for latest temperature value in hundredths of degree celsius
static uint16_t temperature_MOT_cent_celsius   = 2500;  // global variable for latest temperature value in hundredths of degree celsius
static uint16_t latest_ADC_measurement			= 0;			// global variable to store
static uint8_t spindle_speed_feedback_update_is_enabled = 0; /* change to 1 to enable spindle feedback auto-adaptation */
static float spindle_sig_gradient; // Precalulated value to speed up rpm to PWM conversions.
static float currentSpindleSpeedRPM, correctedSpindleSpeedRPM;
static int16_t currentSpindleSpeedNreadings = 0; /* counter of number of readings for spindle speed convergence routine */
static uint16_t currentSpindleSpeedSignalTargetmV = 0;

int filter_fir_int16(long in_global_16, long in_16) {
    return (int)( ((FIR_COEFF_TEMPC * in_16) + ( (1<<8) - FIR_COEFF_TEMPC) * in_global_16)>>8 );
}

/* temperature coefficients */ 
long k[] = {TEMP_K0, TEMP_K1, TEMP_K2, TEMP_K3, TEMP_K4, TEMP_K5, TEMP_K6};

/* function to calculate temperature based on ADC value
 * written as a loop to keep calculation within 32 bit integer number
 * based on 6th order interpolation for the datasheet scaling factors
 * of Thermistor_0402_Panasonic_2kOhm_ERT-J0EG202GM, or ERT-J0EG202HM
 * function takes 270us ~ 4320 cycles
*/
uint16_t convert_temperature (uint16_t temperature_ADC_reading){
#ifdef DEBUG_ADC_ENABLED
debug_pin_write(0, DEBUG_2_PIN);
#endif
    
	int i;
	int pow;
	long tmp;    
	int temperature_instantaneous = 0;
	temperature_instantaneous =  k[0];
	for (i = 1 ; i<=6; i++){
		pow = i;
		tmp = k[i];
		while (pow > 0){
			tmp *= temperature_ADC_reading;
			tmp += (1<<9); /* round vs floor*/
			tmp >>= 10;
			pow --;
		}
		temperature_instantaneous += tmp;
	}	
#ifdef DEBUG_ADC_ENABLED
debug_pin_write(1, DEBUG_2_PIN);
#endif  
    return (uint16_t) temperature_instantaneous * 100;
}


void convert_TMC_temperature (uint16_t temperature_ADC_reading){
    uint16_t temperature_instantaneous = convert_temperature (temperature_ADC_reading);
    temperature_TMC_cent_celsius = filter_fir_int16(temperature_TMC_cent_celsius, temperature_instantaneous); /* 7us */
#ifdef DEBUG_ADC_ENABLED
debug_pin_write(0, DEBUG_2_PIN);
debug_pin_write(1, DEBUG_2_PIN);
#endif
    //printInteger( temperature_TMC_cent_celsius );
    //printPgmString(PSTR(","));
}

void convert_PCB_temperature (uint16_t temperature_ADC_reading){
    uint16_t temperature_instantaneous = convert_temperature (temperature_ADC_reading);
    temperature_PCB_cent_celsius = filter_fir_int16(temperature_PCB_cent_celsius, temperature_instantaneous); /* 7us */
}

void convert_MOT_temperature (uint16_t temperature_ADC_reading){
    uint16_t temperature_instantaneous = convert_temperature (temperature_ADC_reading);
    temperature_MOT_cent_celsius = filter_fir_int16(temperature_MOT_cent_celsius, temperature_instantaneous); /* 7us */
}

#ifdef ADC_INTERNAL_VREF_1100mV
uint16_t convert_adc_5V(uint16_t ADC_reading){
    /* on latest HW load sense is connected to the ADC pin through resistive divider of 10/2.4 kOhm,
    * therefore for 5V input the output is 0.968mV. With 1.1V bandgap reference 5 V will be corresponded to ADC code 900
    * to convert the ADC code to voltage: V_out_mV = ADC_code * 1.1*5*(10+2.4)/(5*1023*2.4)*1000 =
    * = ADC*1100*(10+2.4)/(1023*2.4) = ADC * 136400 / 24552 */
    return (uint16_t) ( ( (long)ADC_reading * 136400 ) / 24552 );    
}

uint16_t convert_adc_10V(uint16_t ADC_reading){
    /* on latest HW load sense is connected to the ADC pin through resistive divider of 10/1 kOhm,
    * therefore for 10V input the output is 0.968mV. With 1.1V bandgap reference 10 V will be corresponded to ADC code 900
    * to convert the ADC code to voltage: V_out_mV = ADC_code * 1.1*10*(10+1)/(10*1023*1)*1000 =
    * = ADC*1100*(10+1)/(1023*1) = ADC * 123200 / 12276 */
    return (uint16_t) ( ( (long)ADC_reading * 121000 ) / 10230 );    
}

uint16_t convert_adc_24V(uint16_t ADC_reading){
    /* on latest HW load sense is connected to the ADC pin through resistive divider of 30/1.2 kOhm,
    * therefore for 10V input the output is 0.968mV. With 1.1V bandgap reference 10 V will be corresponded to ADC code 900
    * to convert the ADC code to voltage: V_out_mV = ADC_code * 1.1*10*(10+1.2)/(10*1023*1.2)*1000 =
    * = ADC*1100*(10+1.2)/(1023*1.2) = ADC * 123200 / 12276 */
    return (uint16_t) ( ( (long)ADC_reading * 123200 ) / 12276 );    
}
#endif //#ifdef ADC_INTERNAL_VREF_1100mV

#ifdef ADC_EXTERNAL_VREF_2048mV
uint16_t convert_adc_5V(uint16_t ADC_reading){
    /* HW load sense is connected to the ADC pin through resistive divider of 10/2.4 kOhm,
    * therefore for 5V input the output is 0.968mV. With 2.048V reference 5 V will be corresponded to ADC code 483
    * to convert the ADC code to voltage: V_out_mV = ADC_code * 2.048*5*(10+2.4)/(5*1023*2.4)*1000 =
    * = ADC*1100*(10+2.4)/(1023*2.4) = ADC * 253952 / 24552 */
    return (uint16_t) ( ( (long)ADC_reading * 253952 ) / 24552 );    
}

uint16_t convert_adc_10V(uint16_t ADC_reading){
    /* on latest HW load sense is connected to the ADC pin through resistive divider of 2000/240 kOhm,
    * therefore for 10V input the output is 1071mV. With 2.048V reference 10 V will be corresponded to ADC code 535
    * to convert the ADC code to voltage: V_out_mV = ADC_code * 2048*10*(10+1)/(10*1023*1)*1000 =
    * = ADC*1100*(10+1)/(1023*1) = ADC * 458752 / 24552 */
    return (uint16_t) ( ( (long)ADC_reading * 458752 ) / 24552 );    
}

uint16_t convert_adc_24V(uint16_t ADC_reading){
    /* on latest HW load sense is connected to the ADC pin through resistive divider of 10/0.47 kOhm,
    * therefore for 10V input the output is 1077mV. With 2.048V bandgap reference 24 V will be corresponded to ADC code 538
    * to convert the ADC code to voltage: V_out_mV = ADC_code * 2048*24*(10+1.2)/(10*1023*1.2)*1000 =
    * = ADC*1100*(10+1.2)/(1023*1.2) = ADC * 214426 / 4808 */
    return (uint16_t) ( ( (long)ADC_reading * 214426 ) / 4808 );    
}
#endif //#ifdef ADC_INTERNAL_VREF_2048mV

void convert_spindle_load (uint16_t ADC_reading){
    /* Mafel load output range is 0-5V, convert 10bits ADC output into mV: */
    spindle_load_mV = convert_adc_5V(ADC_reading);
}

void convert_VDD_5V_Atmega_mV (uint16_t ADC_reading){
    /* output range is 0-5V, convert 10bits ADC output into mV: */
    VDD_5V_Atmega_mV = convert_adc_5V(ADC_reading);
}

void convert_VDD_5V_dustshoe_mV (uint16_t ADC_reading){
    /* output range is 0-5V, convert 10bits ADC output into mV: */
    VDD_5V_dustshoe_mV = convert_adc_5V(ADC_reading);
}

void convert_AC_loss_Signal_mV (uint16_t ADC_reading){
    /* output range is 0-5V, convert 10bits ADC output into mV: */
    AC_loss_Signal_mV = convert_adc_5V(ADC_reading);
}


void convert_VDD_24V_mV (uint16_t ADC_reading){
    /* output range is 0-5V, convert 10bits ADC output into mV: */
    VDD_24V_mV = convert_adc_24V(ADC_reading);
}

void convert_Spindle_speed_Signal_mV (uint16_t ADC_reading){
	/* output range is 0-10V, convert 10bits ADC output into mV: */
	Spindle_speed_Signal_mV = convert_adc_10V(ADC_reading);
}

/* three options are possible:
 * 1. No spindle RPM feedback: ignore feedback functions and keep static settings from direct PWM control 
 * 2. Analogue feedback: 10V from the speed control output circuit is fed back to the Atmega ADC and adjustment to the speed is made in SPINDLE_SPEED_FEEDBACK_N_CONVERGES steps 
 * 3. Digital feedback: Digital Mafel spindle is reporting it's speed over UART2 and PWM signal is adjusted accordingly in SPINDLE_SPEED_FEEDBACK_N_CONVERGES steps 

*/
void spindle_speed_feedback_rpm_updated(float rpm){
    if ( ( spindle_speed_feedback_update_is_enabled == 1) || (digitalSpindle.is_present)){
        if (currentSpindleSpeedRPM != rpm){
            currentSpindleSpeedNreadings = SPINDLE_SPEED_FEEDBACK_N_CONVERGES; /* reset convergence state machine */
            /* enable ADC readings of spindle speed signal channel */
            ADCstMachine.max_count[ADC_7_SPINDLE_SPEED  ] = ((SPINDLE_SPEED_ADC_PERIOD_MS   *1000UL)/SPI_READ_OCR_PERIOD_US) ;
        }
        currentSpindleSpeedRPM = rpm;
        correctedSpindleSpeedRPM = rpm;
    
        if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
            // No PWM range possible. Set simple on/off spindle control pin state.
            currentSpindleSpeedSignalTargetmV = SPINDLE_SIG_MAX_MILLIVOLTS;
            } else if (rpm <= settings.rpm_min) {
            currentSpindleSpeedSignalTargetmV = SPINDLE_SIG_MIN_MILLIVOLTS;
            } else {
            // Compute intermediate value with linear spindle speed model.
            // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
            currentSpindleSpeedSignalTargetmV = (uint16_t) ( ((rpm-settings.rpm_min) * spindle_sig_gradient) + SPINDLE_SIG_MIN_MILLIVOLTS ) ;
        } // if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
                   
    } // if ( ( spindle_speed_feedback_update_is_enabled == 1) || (digitalSpindle.is_present)){    
        
}

void converge_Spindle_speed (void){
    
    /* spindle speed convergence is based on feedback mechanism 
       spindle speed feedback is read SPINDLE_SPEED_FEEDBACK_N_CONVERGES times after any speed change request and corresponded nudges are made to ensure exact expected voltage for the given speed is set.
    */
    float rpm_delta_update = 0.0;    
    /* output range is 0-10V, convert 10bits ADC output into mV: */
    uint16_t Spindle_speed_Signal_mV_instantaneous = Spindle_speed_Signal_mV;
	
    //Spindle_speed_Signal_mV = filter_fir_int16(Spindle_speed_Signal_mV, Spindle_speed_Signal_mV_instantaneous); /* 7us */
    
    /* if digital spindle is installed use actual RPM to correct the spindle speed */
    if (digitalSpindle.is_present){
        rpm_delta_update = digitalSpindle.RPM - correctedSpindleSpeedRPM;        
    }
    else
    {
        rpm_delta_update = ((float)Spindle_speed_Signal_mV_instantaneous - (float)currentSpindleSpeedSignalTargetmV)/spindle_sig_gradient;        
    }

    correctedSpindleSpeedRPM -= rpm_delta_update * SPINDLE_SIG_CONVERGENCE_DAMPING_FACTOR;
    //spindle_nudge_pwm(correctedSpindleSpeedRPM);
        
    //printInteger( correctedSpindleSpeedRPM );
    //printPgmString(PSTR("\n"));
    
    currentSpindleSpeedNreadings--;
    if ( currentSpindleSpeedNreadings <= 0 ) {
        /* finish convergence, disable channel set PERIOD_MS to 0xFFFF */ 
        //ADCstMachine.max_count[ADC_7_SPINDLE_SPEED  ] = ((ADC_PERIOD_DISABLE *1000UL)/SPI_READ_OCR_PERIOD_US) ;
    }
}

/* ADC state machine 
 * ADC state machine is required when multiple inputs need to be measured, for example Load sense signal, temperature and input 24V voltage 
 * State machine starts from cold start, set up the ADC to measure first channel, then after the ADC conversion interrupt comes it proceeds to next channel
 * In the end of conversions it stores all values in relevant global variables */

void asmcnc_start_ADC_single(void){

    uint8_t ADCchannel = ADCstMachine.channel[ADCstMachine.adc_state];

	//select ADC channel with safety mask. First check whether the channel is in port K
	if (ADCchannel > 7){
		ADCchannel -= 8;
		ADCSRB |= (1<<MUX5);
	}
    else{
        ADCSRB &=~(1<<MUX5);
    }        
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);

	//start conversion in Single Conversion Mode
	ADCSRA |= (1<<ADSC);
	
}

/* ADC conversion complete interrupt */
ISR(ADC_vect)
{
    /* routine takes 3.26us ~ 50 cycles */
#ifdef DEBUG_ADC_ENABLED
debug_pin_write(0, DEBUG_0_PIN);
#endif

	latest_ADC_measurement = ADC;
	system_set_exec_heartbeat_command_flag(ADC_CONVERGENCE_COMPLETED);

#ifdef DEBUG_ADC_ENABLED
debug_pin_write(1, DEBUG_0_PIN);
#endif
}


/* 
 * ADC state machine is implemented here */
void adc_state_machine(void){

    /* routine takes 3.26us ~ 50 cycles */
    #ifdef DEBUG_ADC_ENABLED
    debug_pin_write(0, DEBUG_1_PIN);
    #endif
    /* store results of last conversion */
    if ( ADCstMachine.adc_state < ADC_TOTAL_CHANNELS ) ADCstMachine.result[ADCstMachine.adc_state] = latest_ADC_measurement;

    /* advance to next channel */
    ADCstMachine.adc_state++;
    while ( ADCstMachine.adc_state < ADC_TOTAL_CHANNELS ){
	    if (ADCstMachine.measure_channel[ADCstMachine.adc_state] == 1)
	    { /* start next conversion */
		    asmcnc_start_ADC_single();
			#ifdef DEBUG_ADC_ENABLED
			debug_pin_write(1, DEBUG_1_PIN);
			#endif
		    return; /* next conversion is started, return from ISR */
	    }
	    ADCstMachine.adc_state++;
    }
    
    /* if the code progressed to this point then all channels are done and it is time to signal to main loop to process all results */
    system_set_exec_heartbeat_command_flag(ADC_PROCESS_ALL_COMMAND);

    #ifdef DEBUG_ADC_ENABLED
    debug_pin_write(1, DEBUG_1_PIN);
    #endif
	
}


/* start ADC state machine from channel 0 */
void asmcnc_start_ADC(void){
	if (ADCstMachine.adc_locked == 0){
		ADCstMachine.adc_locked = 1;
		if (ADCstMachine.adc_state == ADC_TOTAL_CHANNELS){
        
			ADCstMachine.adc_state = ADC_0_SPINDLE_LOAD;
        
			/* check if any of channels are to be measured and fire measurement on first required channel, other remaining channels will be managed by adc_state_machine() */
			while ( ADCstMachine.adc_state < ADC_TOTAL_CHANNELS ){
				if (ADCstMachine.measure_channel[ADCstMachine.adc_state] == 1)
				{ /* start next conversion */ 
					asmcnc_start_ADC_single();
					return; /* next conversion is started, return from function immediately */
				}
				ADCstMachine.adc_state++;
			}
		}
		else{
			/* should not really come here, but if happened, reset state to idle, so next cycle will initialize ADC correctly */
			ADCstMachine.adc_state = ADC_TOTAL_CHANNELS;
		}
		/* if none of the channels is to be measured this time then unlock the adc */        
		ADCstMachine.adc_locked = 0;
		
	} //if (ADCstMachine.adc_locked == 0){
}

/* define ADC channels to be measured and start ADC conversions */
void adc_setup_and_fire(void){
    /* there are 5 channels in total to be measured by ADC:
    1: Spindle load
    2: 5V Atmega
    3: 5V dustshoe
    4: 24V mains
    5: temperature 1
    6: temperature 2
    7: temperature 3
    8: Spindle speed control
    9: AC loss       
    
    each channel has its own poll periodicity that is derived from the main tick time as
    (UPTIME_TICK_PERIOD_MS *1000UL)/SPI_READ_OCR_PERIOD_US
    
    */
#ifdef DEBUG_ADC_ENABLED
debug_pin_write(0, DEBUG_0_PIN);
debug_pin_write(1, DEBUG_0_PIN);
#endif    
    /* loop over each channel in the list, find those that are required to be measured in this round and mark them in parameter "measure_channel"  */
    for (uint8_t adc_ch_idx = 0; adc_ch_idx < ADC_TOTAL_CHANNELS; adc_ch_idx++){
        if (ADCstMachine.max_count[adc_ch_idx] < ((64000000UL)/SPI_READ_OCR_PERIOD_US))  /* if max count is more than 64s then blank out this channel */
        {        
            if (++ADCstMachine.tick_count[adc_ch_idx] % ADCstMachine.max_count[adc_ch_idx] == 0)  { /* fire at predefined interval */
                ADCstMachine.measure_channel[adc_ch_idx] = 1;            
                ADCstMachine.tick_count[adc_ch_idx] = 0; /* reset tick counter for this channel */            
            } //if (++ADCstMachine.tick_count[adc_ch_idx] % ADCstMachine.max_count[adc_ch_idx] == 0)  { /* fire at predefined interval */
                
        } //if (ADCstMachine.max_count[adc_ch_idx] < ((64000000UL)/SPI_READ_OCR_PERIOD_US))  /* if max count is more than 64s then blank out this channle */
        
    } // for (uint8_t adc_ch_idx = 0; adc_ch_idx < ADC_TOTAL_CHANNELS; adc_ch_idx++){
        
    /* start first conversion */
    asmcnc_start_ADC();
    
} 


            
/* Process results of all ADC channels */
void adc_process_all_channels(void){
    /* loop over each channel in the list, find those that are required to be measured in this round and mark them in parameter "measure_channel"  */
    for (uint8_t adc_ch_idx = 0; adc_ch_idx < ADC_TOTAL_CHANNELS; adc_ch_idx++){
        if (ADCstMachine.measure_channel[adc_ch_idx] == 1){ /* channel was measured and need to be processed */
            /* reset measure request flag */
            ADCstMachine.measure_channel[adc_ch_idx] = 0;    
            
            switch (adc_ch_idx){
                case ADC_0_SPINDLE_LOAD:
                    convert_spindle_load(ADCstMachine.result[adc_ch_idx]);
                break;
                
                case ADC_1_VDD_5V_ATMEGA:
                    convert_VDD_5V_Atmega_mV (ADCstMachine.result[adc_ch_idx]);
                break;
                
                case ADC_2_VDD_5V_DUSTSHOE:
                    convert_VDD_5V_dustshoe_mV (ADCstMachine.result[adc_ch_idx]);
                break;
                
                case ADC_3_VDD_24V:
                    convert_VDD_24V_mV (ADCstMachine.result[adc_ch_idx]);                    
                break;
                
                case ADC_4_TEMPERATURE_TMC:
                    convert_TMC_temperature(ADCstMachine.result[adc_ch_idx]); // 2k NTC thermistor on 10k divider gives a voltage from 1.1 to 0.03V for temperature range of 15C to 150C
                break;
                
                case ADC_5_TEMPERATURE_PCB:
                    convert_PCB_temperature(ADCstMachine.result[adc_ch_idx]); // 2k NTC thermistor on 10k divider gives a voltage from 1.1 to 0.03V for temperature range of 15C to 150C
                break;
                
                case ADC_6_TEMPERATURE_MOT:
                    convert_MOT_temperature(ADCstMachine.result[adc_ch_idx]); // 2k NTC thermistor on 10k divider gives a voltage from 1.1 to 0.03V for temperature range of 15C to 150C
                break;
                
                case ADC_7_SPINDLE_SPEED:
                    convert_Spindle_speed_Signal_mV (ADCstMachine.result[adc_ch_idx]);
                break;
                
                case ADC_8_AC_LOSS:
                    convert_AC_loss_Signal_mV (ADCstMachine.result[adc_ch_idx]);
                break;
                
                default:
                break;
                
            }//switch (ADCstMachine.channel[adc_ch_idx]){

        } //if (ADCstMachine.measure_channel[adc_ch_idx] == 1){ /* channel was measured and need to be processed */

    } //for (uint8_t adc_ch_idx = 0; adc_ch_idx < ADC_TOTAL_CHANNELS; adc_ch_idx++){

	/* ADC state machine completed the measurements, unlock it to allow next schedule*/
	ADCstMachine.adc_locked = 0;
}


/* return global variable calculated earlier */
int get_TMC_temperature(void){
    return temperature_TMC_cent_celsius/100;
}

/* return global variable calculated earlier */
int get_PCB_temperature(void){
    return temperature_PCB_cent_celsius/100;
}

/* return global variable calculated earlier */
int get_MOT_temperature(void){
    return temperature_MOT_cent_celsius/100;
}

/* return global variable calculated earlier */
int get_spindle_load_mV(void){    
    return spindle_load_mV;
    //return Spindle_speed_Signal_mV;
}

/* return global variable calculated earlier */
int get_VDD_5V_Atmega_mV(void){
	return VDD_5V_Atmega_mV;
}

/* return global variable calculated earlier */
int get_VDD_5V_dustshoe_mV(void){
	return VDD_5V_dustshoe_mV;
}

/* return global variable calculated earlier */
int get_VDD_24V_mV(void){
	return VDD_24V_mV;
}

/* return global variable calculated earlier */
int get_Spindle_speed_Signal_mV(void){
	return Spindle_speed_Signal_mV;
}


/* Mafell spindles provide very nice feature: it will stop if load on the spindle is too high.
 * But how do we know that it has stopped !? Well there is another nice feature - overload signal (coming
 * through control cable) that informs the state of internal spindle circuitry by the voltage level (from
 * 0V being no-load to 5V being load is critically high). Below code is setting up the Atmega ADC to run
 * continuous measurement of that signal so it could be reported any time in "report_realtime_status" function.
 * The following scheme is implemented for load sense.
 *   Input signal is divided with resistive divider, filtered and fed to the ADC with clean onboard band-gap reference of 1.1V.
 *   This will provide much cleaner and more repeatable way to measure the voltage and it will be independent of onboard 5V supply level and quality.
 *   Option will use the new PCB tracks that will come from HW ver 6 and will be connected to port F on the micro (now empty).
 *   FW will auto-detect the HW by reading the HW key and configure ADC according to option 2 if HW version is higher than 5.
*/
void asmcnc_init_ADC(void)
{

	/* there are 2 ports on mega2560, port F (channels 0-7) and port K (channels 8-15).
	 * On Z-head HW ver < 5 pin 89 (channel 8) is used. For this channel MSB of the MUX (MUX5) need to be used
	 * it is located in register B: ADCSRB. Therefore if channel Number is higher than 7 then ADCSRB need to be written */


    static uint8_t ADC_Spindle_load_channel     = SPINDLE_LOAD_ADC_CHANNEL    ;
    static uint8_t ADC_5V_Atmega_channel        = VDD_5V_ATMEGA_ADC_CHANNEL   ;
    static uint8_t ADC_5V_dustshoe_channel      = VDD_5V_DUSTSHOE_ADC_CHANNEL ;
    static uint8_t ADC_24V_mains_channel        = VDD_24V_ADC_CHANNEL         ;
    static uint8_t ADC_temperature_TMC_channel  = TEMPERATURE_TMC_ADC_CHANNEL ;
    static uint8_t ADC_temperature_PCB_channel  = TEMPERATURE_PCB_ADC_CHANNEL ;
    static uint8_t ADC_temperature_MOT_channel  = TEMPERATURE_MOT_ADC_CHANNEL ;
    static uint8_t ADC_Spindle_speed_channel    = SPINDLE_SPEED_ADC_CHANNEL   ;
    static uint8_t ADC_AC_loss_channel          = AC_LOSS_ADC_CHANNEL         ;

    /* apply correction for HW version */
	if (PIND <= 5){ /* if HW version is 5 and lower*/
    	ADC_Spindle_load_channel          = 8;
	}
	else if (PIND <= 15){ /* if HW version is between 5 and 15 ADC channel is 3*/
    	ADC_Spindle_load_channel          = 3;
	}

	ADCstMachine.adc_state	= ADC_TOTAL_CHANNELS;
	ADCstMachine.adc_locked = 0;
    ADCstMachine.channel[ADC_0_SPINDLE_LOAD   ] = ADC_Spindle_load_channel ;
    ADCstMachine.channel[ADC_1_VDD_5V_ATMEGA  ] = ADC_5V_Atmega_channel    ;
    ADCstMachine.channel[ADC_2_VDD_5V_DUSTSHOE] = ADC_5V_dustshoe_channel  ;
    ADCstMachine.channel[ADC_3_VDD_24V        ] = ADC_24V_mains_channel    ;
    ADCstMachine.channel[ADC_4_TEMPERATURE_TMC] = ADC_temperature_TMC_channel;
    ADCstMachine.channel[ADC_5_TEMPERATURE_PCB] = ADC_temperature_PCB_channel;
    ADCstMachine.channel[ADC_6_TEMPERATURE_MOT] = ADC_temperature_MOT_channel;
    ADCstMachine.channel[ADC_7_SPINDLE_SPEED  ] = ADC_Spindle_speed_channel;
    ADCstMachine.channel[ADC_8_AC_LOSS        ] = ADC_AC_loss_channel      ;

    ADCstMachine.max_count[ADC_0_SPINDLE_LOAD   ] = ((SPINDLE_LOAD_ADC_PERIOD_MS    *1000UL)/SPI_READ_OCR_PERIOD_US) ;
    ADCstMachine.max_count[ADC_1_VDD_5V_ATMEGA  ] = ((VDD_5V_ATMEGA_ADC_PERIOD_MS   *1000UL)/SPI_READ_OCR_PERIOD_US) ;
    ADCstMachine.max_count[ADC_2_VDD_5V_DUSTSHOE] = ((VDD_5V_DUSTSHOE_ADC_PERIOD_MS *1000UL)/SPI_READ_OCR_PERIOD_US) ;
    ADCstMachine.max_count[ADC_3_VDD_24V        ] = ((VDD_24V_ADC_PERIOD_MS         *1000UL)/SPI_READ_OCR_PERIOD_US) ;
    ADCstMachine.max_count[ADC_4_TEMPERATURE_TMC] = ((TEMPERATURE_TMC_ADC_PERIOD_MS *1000UL)/SPI_READ_OCR_PERIOD_US) ;
    ADCstMachine.max_count[ADC_5_TEMPERATURE_PCB] = ((TEMPERATURE_PCB_ADC_PERIOD_MS *1000UL)/SPI_READ_OCR_PERIOD_US) ;
    ADCstMachine.max_count[ADC_6_TEMPERATURE_MOT] = ((TEMPERATURE_MOT_ADC_PERIOD_MS *1000UL)/SPI_READ_OCR_PERIOD_US) ;
    ADCstMachine.max_count[ADC_7_SPINDLE_SPEED  ] = ((SPINDLE_SPEED_ADC_PERIOD_MS   *1000UL)/SPI_READ_OCR_PERIOD_US) ;
    ADCstMachine.max_count[ADC_8_AC_LOSS        ] = ((AC_LOSS_ADC_PERIOD_MS         *1000UL)/SPI_READ_OCR_PERIOD_US) ;

    for (uint8_t adc_ch_idx = 0; adc_ch_idx < ADC_TOTAL_CHANNELS; adc_ch_idx++){
        if (ADCstMachine.max_count[adc_ch_idx] < 1) ADCstMachine.max_count[adc_ch_idx] = 1; /* if max count is less than 1 then requested period is shorter than system tick. Make sure the channel is measured with max speed */
        ADCstMachine.tick_count[adc_ch_idx] = ADCstMachine.max_count[adc_ch_idx] - 2 ; /* -2 to start first conversion soon after boot */
    }

	// reference voltage selection : all commented = external VREF
	//ADMUX |= (1<<REFS1); // Select Internal 1.1V Voltage Reference with external capacitor at AREF pin
	//ADMUX |= (1<<REFS0); //both REFS0 a REFS1 pins means 2.56V Voltage Reference
	//ADMUX |= (1<<REFS0);// Select Vref=AVCC with external capacitor at AREF pin

	//set prescaller to 128 and enable ADC. Pre-scaler 128 with 16M clock corresponds to ~100us long conversion.
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);

	//enable Auto Triggering of the ADC. Free Running mode is default mode of the ADC
	//ADCSRA |= (1<<ADATE);

	//enable ADC interrupt
	ADCSRA |= (1<<ADIE);


    spindle_sig_gradient = (SPINDLE_SIG_MAX_MILLIVOLTS-SPINDLE_SIG_MIN_MILLIVOLTS)/(settings.rpm_max-settings.rpm_min);
    
    digitalSpindle.is_present   = 0;
    digitalSpindle.RPM          = 0;
    digitalSpindle.uptime       = 0;
    digitalSpindle.brush_uptime = 0;
    digitalSpindle.load         = 0;
    digitalSpindle.temperature  = 0;

}

