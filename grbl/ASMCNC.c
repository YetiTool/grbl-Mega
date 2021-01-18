/*
 * ASMCNC.c
 *
 *  Created on: 5 Feb 2018
 *  Author: Ian Adkins
 */
#include "grbl.h"

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
			asmcnc_RGB_set(buffer[0],buffer[1],buffer[2]);
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
			asmcnc_RGB_init(); /* Setup pre-scaling = 8 and Waveform Generation Mode: PWM, Phase Correct, 8-bit */
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
	asmcnc_RGB_set(0,0,0);
}

void asmcnc_RGB_white(void){
	asmcnc_RGB_set(0xFF, 0xFF, 0xFF);
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

void asmcnc_RGB_init(void){
	/* Setup pre-scaling = 8 to ensure 256 shades of grey */
	TCCR3B &=~(1<<CS32); /* unset bit CSn2 set in RGB_red_flash function */
	TCCR3B |=(1<<CS31);

	/* Setup Waveform Generation Mode: PWM, Phase Correct, 8-bit */
	TCCR3B &=~(1<<WGM33); /* unset bit set in RGB_red_flash function */
	TCCR3A &=~(1<<WGM31); /* unset bit set in RGB_red_flash function */
	TCCR3A |= (1<<WGM30);
}

void asmcnc_RGB_set(uint8_t R, uint8_t G, uint8_t B){
	
	asmcnc_RGB_init(); 		/* Setup pre-scaling = 8 and Waveform Generation Mode: PWM, Phase Correct, 8-bit */

	OCR3A = R;
	OCR3B = G;
	OCR3C = B;
	
}

#ifdef ANY_DEBUG_ENABLED
/* function including jumps in out takes 5 cycles = 310ns */
void debug_pin_write(uint8_t level, uint8_t pin){
	if (level==0) DEBUG_PORT &=~(1<<pin); /* clear pin */
	else          DEBUG_PORT |= (1<<pin); /* set pin */
}
#endif


void asmcnc_init(void)
{
//	AC_YLIM_XLIM_DDRB 	|=AC_LIM_RED_MASK_Y;
    //AC_YLIM_XLIM_DDRL 	|=AC_LIM_RED_MASK_XZ;
	AC_ACCS_DDR			|=AC_ACCS_MASK;
    //AC_DOOR_DDR			|=AC_DOOR_RED_MASK;
	AC_RGB_DDR 			|=AC_RGB_MASK;
    #ifdef ANY_DEBUG_ENABLED
    DEBUG_DDR  			|=DEBUG_PORT_MASK; /* enable output direction of the port */
	DEBUG_PORT			|=DEBUG_PORT_MASK; /* default pin state high*/	
    #endif	
	AC_PROBE_HOLDER_DDR	&=~AC_PROBE_HOLDER_MASK; //Set as input

    //PORTL |= AC_LIM_RED_MASK_XZ;
    //PORTL |= AC_DOOR_RED_MASK;
//	PORTB |= AC_LIM_RED_MASK_Y;
	PORTG &=~(1<<AC_EXTRACTOR);
	PORTG &=~(1<<AC_LIGHT);
//	PORTL &=~(1<<AC_DOOR_RED);


	/* setup timer 3 for RGB lights */
	TCCR3A = 0;	//Clear timer3 registers
	TCCR3B = 0;
	TCCR3C = 0;
	TCCR3A |= ((1<<COM3C1)|(1<<COM3B1)|(1<<COM3A1)); 	/* Setup non-inverted output for channels A, B and C */
	asmcnc_RGB_set(0,0,0); /* Turn off all LED's */
	TCNT3=0; 											/* Zero timer 3 */


	spi_hw_init(); /* enable heartbeat timer 5 */

    #if defined(ENABLE_SPINDLE_LOAD_MONITOR) || defined(ENABLE_TEMPERATURE_MONITOR)
    asmcnc_init_ADC();
    #endif
	
	flashStatisticsInit();

    /* AC live enable pin interrupt. Using sleep module to utilise TIMER5 */
    asmcnc_enable_AC_live_detection();
  
}