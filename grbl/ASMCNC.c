/*
 * ASMCNC.c
 *
 *  Created on: 5 Feb 2018
 *  Author: Ian Adkins
 */
#include "grbl.h"

#define 				FIR_COEFF_TEMPC 					30		/**< 0-255 defines how quickly FIR converges 255-fastest */

uint8_t adc_state               = ADC_IDLE; // global ADC state
uint8_t ADC_channel_load        = 0;        // global variable for Load sense pin
uint8_t ADC_channel_temperature = 0;        // global variable for Temperature sense pin
int spindle_load_volts          = 0;        // global variable for latest spindle load value
int temperature_cent_celsius    = 2500;     // global variable for latest temperature value in hundredths of degree celsius
int spindle_load_ADC_reading;               // global variable for latest temperature ADC value
int temperature_ADC_reading;                // global variable for latest Load ADC value

static uint32_t localRunTimeSeconds;

FlashStat flashStatistics;
float totalTravelMillimeters = 0; /* accumulator for accurate tracking of the distance */


/* convert hex char to int */
uint8_t char2int(char input)
{
  if(input >= '0' && input <= '9')
	return input - '0';
  if(input >= 'A' && input <= 'F')
	return input - 'A' + 10;
  if(input >= 'a' && input <= 'f')
	return input - 'a' + 10;
  return 0;
}

/* convert hex char to int and validate result (return 0xFF if character is not hex byte code */
uint8_t char2intValidate(char input)
{
  if(input >= '0' && input <= '9')
	return input - '0';
  if(input >= 'A' && input <= 'F')
	return input - 'A' + 10;
  if(input >= 'a' && input <= 'f')
	return input - 'a' + 10;
  return 17; /* returns error 17 if char is outside of "0123456789ABCDEFabcdef"  */
}

/* This function assumes src to be a zero terminated sanitized string with
* an even number of [0-9a-f] characters, and target to be sufficiently large */
void hex2bin(const char* src, uint8_t* target)
{
	uint32_t byte_count;
	for (byte_count = 0; byte_count<3; byte_count++){
		if (!(*src && src[1])) return; /* null terminated will make this false as left side will be 0 and function will return*/
		*(target++) = char2int(*src)*16 + char2int(src[1]);
		src += 2;
	}
}


uint8_t asmcnc_execute_line(char *line)
{
  switch( line[0] ) {
	case '*':      /* YETI custom realtime commands that does not generate "ok" response */
	  switch( line[1] ) {
		case 'L': {			//RGB HEX Codes, see https://htmlcolorcodes.com/
			uint8_t buffer[3] = {0}; 	/* buffer to hold output int values */
			hex2bin(&line[2], buffer); 	/* convert hex string to numbers , for example: HEX #FFC133 -> RGB 255, 193, 51  */
			asmcnc_RGB_setup(); 		/* Setup pre-scaling = 8 and Waveform Generation Mode: PWM, Phase Correct, 8-bit */
			/* decoded RGB values:
			 * R = buffer [0]
			 * G = buffer [1]
			 * B = buffer [2]
			 * */
			OCR3A=buffer[0]; /* R */
			OCR3B=buffer[1]; /* G */
			OCR3C=buffer[2]; /* B */
			/* debug prints */
//			printPgmString(PSTR("R:"));
//			printInteger(buffer[0]);
//			printPgmString(PSTR("G:"));
//			printInteger(buffer[1]);
//			printPgmString(PSTR("B:"));
//			printInteger(buffer[2]);
			} //case 'L':
			break; //case 'L': 			//RGB HEX Codes, see https://htmlcolorcodes.com/

		default:
			return(ASMCNC_STATUS_INVALID_STATEMENT);
		break;
	  } //switch( line[1] ) {
	  break; //case '*':

	case 'A':      /* YETI custom non-realtime commands, they do generate "ok" response */
	  switch( line[1] ) {
		case 'L': 			//RGD LED PWM values 1=off 255=full on
			asmcnc_RGB_setup(); /* Setup pre-scaling = 8 and Waveform Generation Mode: PWM, Phase Correct, 8-bit */
			if (line[2] == '0') {asmcnc_RGB_off(); break;} //"0" = all off
			if ((line[2] != 'R') && (line[2] != 'G') && (line[2] != 'B')) { return(ASMCNC_STATUS_INVALID_STATEMENT); }
			if ((line[3]<0x30)|(line[3]>0x39)){ return(ASMCNC_STATUS_INVALID_STATEMENT); }
			switch( line[2] ) {
			case 'R':
				switch(line[3]){
					case '0' :OCR3A=0x00; break;
					case '1' :OCR3A=0x0F; break;
					case '2' :OCR3A=0x1F; break;
					case '3' :OCR3A=0x2F; break;
					case '4' :OCR3A=0x3F; break;
					case '5' :OCR3A=0x4F; break;
					case '6' :OCR3A=0x5F; break;
					case '7' :OCR3A=0x6F; break;
					case '8' :OCR3A=0x7F; break;
					case '9' :OCR3A=0xFF; break;
				}
				break; //case 'R':
			case 'G':
				switch(line[3]){
					case '0' :OCR3B=0x00; break;
					case '1' :OCR3B=0x0F; break;
					case '2' :OCR3B=0x1F; break;
					case '3' :OCR3B=0x2F; break;
					case '4' :OCR3B=0x3F; break;
					case '5' :OCR3B=0x4F; break;
					case '6' :OCR3B=0x5F; break;
					case '7' :OCR3B=0x6F; break;
					case '8' :OCR3B=0x7F; break;
					case '9' :OCR3B=0xFF; break;
				}
				break; //case 'G':
			case 'B':
				switch(line[3]){
					case '0' :OCR3C=0x00; break;
					case '1' :OCR3C=0x0F; break;
					case '2' :OCR3C=0x1F; break;
					case '3' :OCR3C=0x2F; break;
					case '4' :OCR3C=0x3F; break;
					case '5' :OCR3C=0x4F; break;
					case '6' :OCR3C=0x5F; break;
					case '7' :OCR3C=0x6F; break;
					case '8' :OCR3C=0x7F; break;
					case '9' :OCR3C=0xFF; break;
				}
				break; //case 'B':
			} //switch( line[2] )
		break; //case 'L':
		case 'E': PORTG |=(1<<AC_EXTRACTOR); break; //Extraction on
		case 'F': PORTG &=~(1<<AC_EXTRACTOR); break; //Extraction off
		//case 'W': PORTG |=(1<<AC_LIGHT); break; //Light on
		//case 'X': PORTG &=~(1<<AC_LIGHT); break; //Light off
#ifdef ENABLE_LASER_POINTER_CONTROL
		case 'Z': PORTE |=(1<<LASER_PIN); break;  //Laser on
		case 'X': PORTE &=~(1<<LASER_PIN); break; //Laser off
#endif
		default:
			return(ASMCNC_STATUS_INVALID_STATEMENT);
		break;
	  } //switch( line[1] ) {
	break; //case 'A'

	default:
		return(ASMCNC_STATUS_INVALID_STATEMENT);
	break;

  } //switch( line[0] )

  return(STATUS_OK); // If 'A' command makes it to here, then everything's ok.
}

void asmcnc_RGB_off(void){

	/* Setup pre-scaling = 8 and Waveform Generation Mode: PWM, Phase Correct, 8-bit */
	asmcnc_RGB_setup();

	/* set min brightness */
	OCR3B=0; OCR3C=0; OCR3A=0;
}

void asmcnc_RGB_white(void){

	/* Setup pre-scaling = 8 and Waveform Generation Mode: PWM, Phase Correct, 8-bit */
	asmcnc_RGB_setup();

	/* set max white brightness */
	OCR3B=0xFF; OCR3C=0xFF; OCR3A=0xFF;
}


void asmcnc_RGB_red_flash(void){		 //Configure PWM for long run time to give visible flash

	/* setup pre-scaler to 256 (ticking @ 8M/256=32768Hz) */
	TCCR3B |=(1<<CS32);
	TCCR3B &=~(1<<CS31); /* unset bit CSn1 (pre-scale 8) set in init function */

	/* setup Waveform Generation Mode: PWM, Phase Correct, based on interrupt ICR3 */
	TCCR3B |=(1<<WGM33);
	TCCR3A |=(1<<WGM31);
	TCCR3A &=~(1<<WGM30);/* unset bit set in init function */

	/* setup interrupt to trigger flash every second */
	ICR3 = 0x8000;					//Timer max count, 1sec period
	OCR3A= 0x4000; OCR3B=0; OCR3C=0; //0.5sec on / off

}

void asmcnc_RGB_setup(void){
	/* Setup pre-scaling = 8 to ensure 256 shades of grey */
	TCCR3B &=~(1<<CS32); /* unset bit CSn2 set in RGB_red_flash function */
	TCCR3B |=(1<<CS31);

	/* Setup Waveform Generation Mode: PWM, Phase Correct, 8-bit */
	TCCR3B &=~(1<<WGM33); /* unset bit set in RGB_red_flash function */
	TCCR3A &=~(1<<WGM31); /* unset bit set in RGB_red_flash function */
	TCCR3A |= (1<<WGM30);
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

	/* there are 2 ports on mega2560, port F (channels 0-7) and power K (channels 8-15).
	 * On Z-head HW ver < 5 pin 89 (channel 8) is used. For this channel MSB of the MUX (MUX5) need to be used
	 * it is located in register B: ADCSRB. Therefore if channel Number is higher than 7 then ADCSRB need to be written */

	ADC_channel_load          = 0;                    //  pin ADC8, channel 8 on HW versions 5 and lower. pin ADC0 on HW ver > 5
	ADC_channel_temperature   = THERMISTOR_MONITOR;

	if (PIND <= 5){ /* if HW version is 5 and lower*/
		ADC_channel_load          = 8;
	}
	else if (PIND <= 15){ /* if HW version is between 5 and 15 ADC channel is 3*/
		ADC_channel_load          = 3;
	}
	else{
		/* for newer than 15 HW ADC load channel is 1 and thermistor is on 3 */
		ADC_channel_load          = SPINDLE_LOAD_MONITOR;
        ADC_channel_temperature   = THERMISTOR_MONITOR;
	}

	// reference voltage selection
	ADMUX |= (1<<REFS1); // Select Internal 1.1V Voltage Reference with external capacitor at AREF pin
	//ADMUX |= (1<<REFS0); //both REFS0 a REFS1 pins means 2.56V Voltage Reference
	//ADMUX |= (1<<REFS0);// Select Vref=AVCC with external capacitor at AREF pin

	//set prescaller to 128 and enable ADC. Pre-scaler 128 with 16M clock corresponds to ~100us long conversion.
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);

	//enable Auto Triggering of the ADC. Free Running mode is default mode of the ADC
	//ADCSRA |= (1<<ADATE);

	//enable ADC interrupt
	ADCSRA |= (1<<ADIE);

  asmcnc_start_ADC();

}

/* function to calculate temperature based on ADC value 
 * written as a loop to keep calculation within 32 bit integer number
 * based on 6th order interpolation for the datasheet scaling factors 
 * of Thermistor_0402_Panasonic_2kOhm_ERT-J0EG202GM
 */

int filter_fir_int16(long in_global_16, long in_16) {
    return (int)(((FIR_COEFF_TEMPC * in_16) + (256 - FIR_COEFF_TEMPC) * in_global_16)/256);
}

/* temperature coefficients */ 
long k[] = { 176   ,
			-1370 ,
			7292  ,
			-21280,
			32853 ,
			-25245,
			  7590};

/* function takes 270us ~ 4320 cycles */
void convert_temperature (void){
#ifdef DEBUG_ADC_ENABLED
debug_pin_write(0, DEBUG_1_PIN);
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
			tmp >>= 10;
			pow --;
		}
		temperature_instantaneous += tmp;
	}	
    temperature_cent_celsius = filter_fir_int16(temperature_cent_celsius, temperature_instantaneous*100);
#ifdef DEBUG_ADC_ENABLED
debug_pin_write(1, DEBUG_1_PIN);
#endif    
}

void convert_spindle_load (void){
    /* Mafel load output range is 0-5V, convert 10bits ADC output into mV: */
    /* on latest HW load sense is connected to the ADC pin through resistive divider of 10/2.4 kOhm,
    * therefore for 5V input the output is 0.968mV. With 1.1V bandgap reference 5 V will be corresponded to ADC code 900
    * to convert the ADC code to voltage: V_out_mV = ADC_code * 1.1*5*(10+2.4)/(5*1023*2.4)*1000 =
    * = ADC*1100*(10+2.4)/(1023*2.4) = ADC * 136400 / 24552 */
    spindle_load_volts = ( (long)spindle_load_ADC_reading * 136400 ) / 24552;

}


/* ADC state machine 
 * ADC state machine is required when multiple inputs need to be measured, for example Load sense signal, temperature and input 24V voltage 
 * State machine starts from cold start, set up the ADC to measure first channel, then after the ADC conversion interrupt comes it proceeds to next channel
 * In the end of conversions it stores all values in relevant global variables */

void asmcnc_start_ADC_single(uint8_t ADCchannel){

	//select ADC channel with safety mask. First check whether the channel is in port K
	if (ADCchannel > 7){
		ADCchannel -= 8;
		ADCSRB |= (1<<MUX5);
	}
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);

	//start conversion in Single Conversion Mode
	ADCSRA |= (1<<ADSC);
	
}

/* ADC conversion complete interrupt 
 * ADC state machine is implemented here */
ISR(ADC_vect)
{
    /* routine takes 3.26us ~ 50 cycles */
#ifdef DEBUG_ADC_ENABLED
debug_pin_write(0, DEBUG_0_PIN);
debug_pin_write(1, DEBUG_0_PIN);
#endif

	switch (adc_state){

		case ADC_CH1:
            spindle_load_ADC_reading = ADC;
            if (PIND <= 15){ /* if HW version is older than 15 ADC channel then dont attempt to measure temperature */
                adc_state = ADC_IDLE; /* back to idle */
            }
            else{
        	    asmcnc_start_ADC_single(ADC_channel_temperature);
        	    adc_state = ADC_CH2; /* advance to next channel */
            }
		break;

		case ADC_CH2:
            temperature_ADC_reading = ADC;
			adc_state = ADC_IDLE;  /* back to idle */
		break;

		default:
		break;
	}
#ifdef DEBUG_ADC_ENABLED
debug_pin_write(0, DEBUG_0_PIN);
debug_pin_write(1, DEBUG_0_PIN);
#endif
}

/* start ADC state machine from channel 1 */
void asmcnc_start_ADC(void){
    if (adc_state == ADC_IDLE){
        asmcnc_start_ADC_single(ADC_channel_load);
        adc_state = ADC_CH1;
    }
    else{
        /* should not really come here, but if happened, reset state to idle, so next cycle will initialise ADC correctly */
        adc_state = ADC_IDLE;
    }
}

/* return global variable calculated earlier */
int get_temperature(void){
    convert_temperature(); // 2k NTC thermistor on 10k divider gives a voltage from 1.1 to 0.03V for temperature range of 15C to 150C
    return temperature_cent_celsius/100;
}

/* return global variable calculated earlier */
int get_spindle_load_volts(void){
    convert_spindle_load();
    return spindle_load_volts;
}

#ifdef ANY_DEBUG_PINS_ENABLED 
/* function including jumps in out takes 5 cycles = 310ns */
void debug_pin_write(uint32_t level, uint32_t pin){
	if (level==0) DEBUG_PORT &=~(1<<pin); /* clear pin */
	else          DEBUG_PORT |= (1<<pin); /* set pin */
}
#endif


/* polynomial is based on crc8.py module from pypl */
static uint8_t const crc8x_table[] PROGMEM = {
            0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
            0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
            0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
            0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
            0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5,
            0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
            0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
            0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
            0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
            0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
            0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2,
            0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
            0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32,
            0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
            0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
            0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
            0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c,
            0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
            0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec,
            0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
            0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
            0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
            0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c,
            0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
            0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b,
            0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
            0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
            0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
            0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb,
            0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
            0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb,
            0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3};


uint8_t crc8x_fast(uint8_t crc, uint8_t *mem, uint16_t len) {
	uint8_t *data = mem;
    if (data == NULL)
        return 0xff;
    crc &= 0xff;
    for(uint16_t idx=0; idx<len; idx++){
        crc = pgm_read_byte_near(crc8x_table + (crc ^ data[idx])); //= crc8x_table[crc ^ data[idx]];
    }
    return crc;
}


void log_resetreason(void)
{
    uint8_t resetReason = MCUSR;
    /* find and log reset reason */    
    printPgmString(PSTR("\r\n\r\nReset cause: "));
    if      ( resetReason & ( 1<<PORF ) )   { flashStatistics.PORF_cnt++;  printPgmString(PSTR("Power On\n")); }
    else if ( resetReason & ( 1<<EXTRF) )   { flashStatistics.EXTRF_cnt++; printPgmString(PSTR("External Reset\n")); }
    else if ( resetReason & ( 1<<BORF ) )   { flashStatistics.BORF_cnt++;  printPgmString(PSTR("Brown out\n")); }
    else if ( resetReason & ( 1<<WDRF ) )   { flashStatistics.WDRF_cnt++;  printPgmString(PSTR("Watch Dog\n")); }
    else if ( resetReason & ( 1<<JTRF ) )   { flashStatistics.JTRF_cnt++;  printPgmString(PSTR("JTAG\n")); }
    else    { printInteger( resetReason );    flashStatistics.WDRF_cnt++;  printPgmString(PSTR(": Watch Dog?\n")); } /* apparently WDR is not showing in the MCUSR, so if anything else then likely it is it. Also bootloader uses WD to reset status, watch out for it later */

    /* clear status register after reading as it is sticky */
    MCUSR = 0;
}


void manage_rst_reasons(void){

    log_resetreason();
    flashStatistics.TOT_cnt++; /* increment total reset count */
    
    printPgmString(PSTR("Total resets: "));   printInteger( flashStatistics.TOT_cnt );
    printPgmString(PSTR(" = "));        printInteger( flashStatistics.PORF_cnt);
    printPgmString(PSTR(" POR + "));     printInteger( flashStatistics.EXTRF_cnt);
    printPgmString(PSTR(" EXT + "));     printInteger( flashStatistics.BORF_cnt);
    printPgmString(PSTR(" BOR + "));     printInteger( flashStatistics.WDRF_cnt);
    printPgmString(PSTR(" WDT + "));     printInteger( flashStatistics.JTRF_cnt);
    printPgmString(PSTR(" JTAG\n"));    

    /* manage last exception storage: if address in different from what was stored in flashStatistics then store it */
    int i;
    uint32_t return_addr;
    return_addr = get_return_addr(); /* return address from the latest stack dump */
    if (flashStatistics.lastReturnAddresses[0] != return_addr){
        /* shift fifo */
        for (i=2; i>=0; i--){
            flashStatistics.lastReturnAddresses[i+1]    = flashStatistics.lastReturnAddresses[i];
        }        
        flashStatistics.lastReturnAddresses[0]          = return_addr;
        
        /* print */
        printPgmString(PSTR(" last WDT addresses: "));
        for (i=0; i<4; i++){
            printInteger( flashStatistics.lastReturnAddresses[i] ); printPgmString(PSTR(", "));
        }
        printPgmString(PSTR("\n"));
        
    } //if (flashConfig.lastReturnAddresses[0] != return_addr){

}

/* implementation of safe flash structure update if new field is added in the end */
void manage_psflash_updates(void){
    if (flashStatistics.flashStatisticsVersion < CURRENT_FLASH_STAT_VER){  /* new trap method to initialise new variables under DFU */
        if (flashStatistics.flashStatisticsVersion < 20090614) {                  /* first time */
            printPgmString(PSTR("trap to update from version "));   printInteger( flashStatistics.flashStatisticsVersion );
            printPgmString(PSTR(" to version "));                   printInteger( CURRENT_FLASH_STAT_VER );
            printPgmString(PSTR("\n"));
            /* trap to update RSSI for new DFU where flash structure changed */
            flashStatistics.TOT_cnt = 0;                           /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
            flashStatistics.EXTRF_cnt = 0;                           /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
            flashStatistics.JTRF_cnt = 0;                           /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
            flashStatistics.totalTravelMillimeters = 0;             /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
            flashStatistics.flashStatisticsVersion = 20090614;     /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
        }
        //if (flashStatistics.flashStatisticsVersion < 20091500) {                  /* second time */
            //printPgmString(PSTR("trap to update from version "));   printInteger( flashStatistics.flashStatisticsVersion );
            //printPgmString(PSTR(" to version "));                   printInteger( CURRENT_FLASH_STAT_VER );
            //printPgmString(PSTR("\n"));
            ///* trap to update RSSI for new DFU where flash structure changed */
            //flashStatistics.flashStatisticsVersion = 20091500;     /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
        //}
        
    } //if (flashStatistics.flashStatisticsVersion < CURRENT_FLASH_STAT_VER){  /* new trap method to initialise new variables under DFU */
}



/* flashStatistics save and restore are done without crc 
 * 1) to avoid wearing flash prematurely (write happens every minute to circular buffer RunTimeMinutesFIFO), and crc location would have been worn fast.
 * 2) statistics is just for info and it is not too critical to keep it checked */

uint8_t flashStatisticsRestore(void){
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(1, DEBUG_1_PIN);
#endif    
    /* load statistics from eeprom */
    memcpy_from_eeprom((char*)&flashStatistics, EEPROM_ADDR_STATISTICS, sizeof(FlashStat));
    if (flashStatistics.flashStatisticsVersion == 0xFFFFFFFF) { 
        /* Reset with default zero vector if load for the first time */
        memset(&flashStatistics, 0, sizeof(FlashStat));
        memset(&flashStatistics.RunTimeMinutesFIFO, 0xFF, UPTIME_FIFO_SIZE_BYTES);
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(0, DEBUG_1_PIN);
#endif        
        return(false);
    }
    
    /* restore local time counter */
    localRunTimeSeconds = flashStatistics.totalRunTimeSeconds;
    /* add seconds remainder rolling bits from array into seconds */
    for (uint8_t byte_index=0; byte_index<UPTIME_FIFO_SIZE_BYTES; byte_index++){
        for (uint8_t bit_position=0; bit_position<8; bit_position++){            
            /* increment localRunTimeSeconds if bit is not 1*/
            if ( !(flashStatistics.RunTimeMinutesFIFO[byte_index] & (1<<bit_position)) ){
                localRunTimeSeconds += 64;
            }
        }        
    }
    
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(0, DEBUG_1_PIN);
#endif    
    return(true);
}

void flashStatisticsSave(void){
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(1, DEBUG_1_PIN);
#endif
/* only access EEPROM in not in motions state */
    if ( !(sys.state & (STATE_CYCLE | STATE_HOMING | STATE_JOG) ) ){
        /* save statistics to eeprom */    
        memcpy_to_eeprom(EEPROM_ADDR_STATISTICS, (char*)&flashStatistics, sizeof(FlashStat));
    }    
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(0, DEBUG_1_PIN);
#endif    
}

void uptime_increment(void){
    
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(1, DEBUG_0_PIN);
#endif
    
    localRunTimeSeconds++;       
    
    /* Flash lifetime is not affected when 0 is written, only depend on erase cycles. 
    array would allow to write 256 zeros and only then erase to loop the counter. 
    If do it every minute eeprom life will be reached in 10 years. */    
    
    /* every minute write bit to the minutes bit array */
    if ( (localRunTimeSeconds % 64) == 0 )
    {
        
        /* update travel distance if long enough (more than 1m) */
        if ( totalTravelMillimeters > 1000.0){
            flashStatistics.totalTravelMillimeters += (uint32_t)totalTravelMillimeters;
            totalTravelMillimeters = 0.0;
        }
        
        uint32_t localRunTimeMinutes = localRunTimeSeconds >> 6; /* not exactly minutes but dies the job be slowing down EEPROM writes 64 times */

        /* unroll the minutes remainder */
        uint16_t minutes_remainder = localRunTimeMinutes % (UPTIME_FIFO_SIZE_BYTES*8);

        /* position "minutes_remainder" is position of the bit in the array that need to be set to "0" */

        /* find which byte it should be set in: */
        uint8_t byte_index = minutes_remainder >> 3;
        uint8_t bit_position = minutes_remainder % 8;

        /* clear relevant bit */
        flashStatistics.RunTimeMinutesFIFO[byte_index] &=~(1<<bit_position) ;

        /* update flashStatistics.totalRunTimeSeconds when bit buffer wraps and erase the buffer RunTimeMinutesFIFO (this is what actually takes time and life from EEPROM */
        if ( minutes_remainder == (UPTIME_FIFO_SIZE_BYTES*8-1) )
        {
            flashStatistics.totalRunTimeSeconds = localRunTimeSeconds+64;
            memset(&flashStatistics.RunTimeMinutesFIFO, 0xFF, UPTIME_FIFO_SIZE_BYTES);
        }


        /* write total statistic to EEPROM every hour */
        flashStatisticsSave();

#ifdef FLASH_DEBUG_ENABLED
        printPgmString(PSTR("total ")); printInteger( flashStatistics.totalRunTimeSeconds );
        printPgmString(PSTR("s, local ")); printInteger( localRunTimeSeconds );
        printPgmString(PSTR("s, minutes_remainder ")); printInteger( minutes_remainder );
        printPgmString(PSTR("\n"));       
#endif
        
        
    }
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(0, DEBUG_0_PIN);
#endif
    
}

void report_statistics(void)
{
    printPgmString(PSTR("^STAT:"));
    printInteger(flashStatistics.TOT_cnt                ); printPgmString(PSTR(", "));
    printInteger(flashStatistics.JTRF_cnt               ); printPgmString(PSTR(", "));
    printInteger(flashStatistics.WDRF_cnt               ); printPgmString(PSTR(", "));
    printInteger(flashStatistics.BORF_cnt               ); printPgmString(PSTR(", "));
    printInteger(flashStatistics.EXTRF_cnt              ); printPgmString(PSTR(", "));
    printInteger(flashStatistics.PORF_cnt               ); printPgmString(PSTR(", "));
    printInteger(localRunTimeSeconds                    ); printPgmString(PSTR(", "));
    printInteger(flashStatistics.totalTravelMillimeters ); printPgmString(PSTR(", "));
    printInteger(flashStatistics.totalStallsDetected    );
    printPgmString(PSTR("v\n"));
}


void asmcnc_init(void)
{
    
    AC_YLIM_XLIM_DDRL 	|=AC_LIM_RED_MASK_XZ;
    AC_ACCS_DDR			|=AC_ACCS_MASK;
    AC_DOOR_DDR			|=AC_DOOR_RED_MASK;
    AC_RGB_DDR 			|=AC_RGB_MASK;
    #if defined(DEBUG_PINS_ENABLED) || defined(DEBUG_ADC_ENABLED) || defined(DEBUG_STEPPER_ENABLED)
    DEBUG_DDR  			|=DEBUG_PORT_MASK;
    #endif
    AC_PROBE_HOLDER_DDR	&=~AC_PROBE_HOLDER_MASK; //Set as input

    PORTL |= AC_LIM_RED_MASK_XZ;
    PORTL |= AC_DOOR_RED_MASK;
    PORTG &=~(1<<AC_EXTRACTOR);
    PORTG &=~(1<<AC_LIGHT);


    /* setup timer 3 for RGB lights */
    TCCR3A = 0;	//Clear timer3 registers
    TCCR3B = 0;
    TCCR3C = 0;
    TCCR3A |= ((1<<COM3C1)|(1<<COM3B1)|(1<<COM3A1)); 	/* Setup non-inverted output for channels A, B and C */
    asmcnc_RGB_setup(); 							/* Setup pre-scaling = 8 and Waveform Generation Mode: PWM, Phase Correct, 8-bit */
    TCNT3=0; 											/* Zero timer 3 */
    OCR3A = 0; OCR3B = 0; OCR3C = 0;					/* Turn off all LED's */

    #if defined(ENABLE_SPINDLE_LOAD_MONITOR) || defined(ENABLE_TEMPERATURE_MONITOR)
    asmcnc_init_ADC();
    #endif

    init_TMC(); /* initialise TMC motor controllers */
    
    enable_watchdog();    
    
    /* report TMC registers */
    system_set_exec_tmc_cal_command_flag(TMC_REGISTERS_REPORT);

#ifdef FLASH_DEBUG_ENABLED
//debug_pin_write(1, DEBUG_0_PIN);
#endif    
    flashStatisticsRestore();
    manage_rst_reasons();
    manage_psflash_updates();
    printPgmString(PSTR("Up time: "));  printInteger( localRunTimeSeconds ); printPgmString(PSTR("seconds\n"));
    printPgmString(PSTR("total distance: "));  printInteger( flashStatistics.totalTravelMillimeters); printPgmString(PSTR("mm\n"));    
    flashStatisticsSave();
#ifdef FLASH_DEBUG_ENABLED
//debug_pin_write(0, DEBUG_0_PIN);
#endif

}

