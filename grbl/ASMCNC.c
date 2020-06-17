/*
 * ASMCNC.c
 *
 *  Created on: 5 Feb 2018
 *  Author: Ian Adkins
 */
#include "grbl.h"
#include "TMC2590.h"

void asmcnc_init(void)
{
	AC_YLIM_XLIM_DDRL 	|=AC_LIM_RED_MASK_XZ;
	AC_ACCS_DDR			|=AC_ACCS_MASK;
	AC_DOOR_DDR			|=AC_DOOR_RED_MASK;
	AC_RGB_DDR 			|=AC_RGB_MASK;
#ifdef DEBUG_PINS_ENABLED
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

#ifdef ENABLE_SPINDLE_LOAD_MONITOR
	asmcnc_init_ADC();
#endif

	init_TMC(); /* initialise TMC motor controllers */



}

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
 * Two options depend on hardware version:
 * 1) quick and dirty. For 130 boards that are in production and need to be modded manually.
 *   To minimise components and simplify work this option will use ADC with dirty VCC 5V reference,
 *   direct connection of the pin to the source with only one decoupling capacitor.
 *   Option will use the unpopulated "Power Loss detection" circuitry on board and will be connected to
 *   port K on the micro (same as used for stop bar, door and probe signals). FW will auto-detect the HW by
 *   reading the HW key and configure ADC according to option 1 if HW version is 4 or 5.
 * 2) Nice and Clean. For the next batch of boards more robust scheme will be implemented.
 *   It will divide the input signal with resistive divider, filter it and feed the ADC with clean onboard band-gap reference of 1.1V.
 *   This will provide much cleaner and more repeatable way to measure the voltage and it will be independent of onboard 5V supply level and quality.
 *   Option will use the new PCB tracks that will come from HW ver 6 and will be connected to port F on the micro (now empty).
 *   FW will auto-detect the HW by reading the HW key and configure ADC according to option 2 if HW version is higher than 5.
*/
void asmcnc_init_ADC(void)
{

	/* there are 2 ports on mega2560, port F (channels 0-7) and power K (channels 8-15).
	 * On Z-head HW ver < 5 pin 89 (channel 8) is used. For this channel MSB of the MUX (MUX5) need to be used
	 * it is located in register B: ADCSRB. Therefore if channel Number is higher than 7 then ADCSRB need to be written */

	uint8_t ADCchannel = 0;// pin ADC8, channel 8 on HW versions 5 and lower. pin ADC0 on HW ver > 5

	if (PIND <= 5){ /* if HW version is 5 and lower*/
		ADCchannel = 8;
	}
	else{
		/* for newer than 5 HW Vref is 1V1 bandgap and ADC channel is 3*/
		ADCchannel = SPINDLE_LOAD_MONITOR;
	}

	// reference voltage selection
    ADMUX |= (1<<REFS1); // Select Internal 1.1V Voltage Reference with external capacitor at AREF pin
    //ADMUX |= (1<<REFS0); //both REFS0 a REFS1 pins means 2.56V Voltage Reference
    //ADMUX |= (1<<REFS0);// Select Vref=AVCC with external capacitor at AREF pin

    //set prescaler to 128 and enable ADC. Pre-scaler 128 with 16M clock corresponds to ~100us long conversion.
    ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);

    //enable Auto Triggering of the ADC. Free Running mode is default mode of the ADC
    ADCSRA |= (1<<ADATE);

    //select ADC channel with safety mask. First check whether the channel is in port K
	if (ADCchannel > 7){
		ADCchannel -= 8;
	    ADCSRB |= (1<<MUX5);
	}
    ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);

    //start conversion in Free Running mode
    ADCSRA |= (1<<ADSC);

}

#ifdef DEBUG_PINS_ENABLED
void debug_pin_write(uint32_t level, uint32_t pin){
	if (level==0) DEBUG_PORT &=~(1<<pin); /* clear pin */
	else          DEBUG_PORT |= (1<<pin); /* set pin */
}
#endif
