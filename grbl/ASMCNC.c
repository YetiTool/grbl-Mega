/*
 * ASMCNC.c
 *
 *  Created on: 5 Feb 2018
 *  Author: Ian Adkins
 */
#include "grbl.h"


void asmcnc_init()
{
//	AC_YLIM_XLIM_DDRB 	|=AC_LIM_RED_MASK_Y;
	AC_YLIM_XLIM_DDRL 	|=AC_LIM_RED_MASK_XZ;
	AC_ACCS_DDR			|=AC_ACCS_MASK;
	AC_DOOR_DDR			|=AC_DOOR_RED_MASK;
	AC_RGB_DDR 			|=AC_RGB_MASK;
	AC_PROBE_HOLDER_DDR	&=~AC_PROBE_HOLDER_MASK; //Set as input

	PORTL |= AC_LIM_RED_MASK_XZ;
	PORTL |= AC_DOOR_RED_MASK;
//	PORTB |= AC_LIM_RED_MASK_Y;
	PORTG &=~(1<<AC_EXTRACTOR);
	PORTG &=~(1<<AC_LIGHT);
//	PORTL &=~(1<<AC_DOOR_RED);

	TCCR3A =0;	//Clear timer3 registers
	TCCR3B =0;
	TCCR3C =0;
	TCCR3A |=((1<<COM3C1)|(1<<COM3B1)|(1<<COM3A1)|(1<<WGM30)|(1<<WGM31)); //Setup PWM
	TCCR3B |=(1<<CS30);

		TCCR3B &=~(1<<CS32);
		TCCR3B &=~(1<<WGM33);
		TCCR3A &=~(1<<WGM31);




	TCNT3=0; //Zero timer 3
	OCR3A = 0; OCR3B = 0; OCR3C = 0;	//Turn off all LED's


}



uint8_t asmcnc_execute_line(char *line)
{
  switch( line[1] ) {
    case 'L': {			//RGD LED PWM values 1=off 255=full on
    	if (line[2] == '0') {asmcnc_RGB_off(); break;} //"0" = all off
    	if ((line[2] != 'R') && (line[2] != 'G') && (line[2] != 'B')) { return(ASMCNC_STATUS_INVALID_STATEMENT); }
    	if ((line[3]<0x30)|(line[3]>0x39)){ return(ASMCNC_STATUS_INVALID_STATEMENT); }
    	switch( line[2] ) {
    	case 'R': {
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
    			} break;
    		}
    	case 'G': {
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
				} break;
    		}
    	case 'B': {
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
				} break;
    		}
    	}break;
    case 'E': PORTG |=(1<<AC_EXTRACTOR); break; //Extraction on
    case 'F': PORTG &=~(1<<AC_EXTRACTOR); break; //Extraction off
    case 'W': PORTG |=(1<<AC_LIGHT); break; //Light on
    case 'X': PORTG &=~(1<<AC_LIGHT); break; //Light off
    }
    default: return(ASMCNC_STATUS_INVALID_STATEMENT); break;
  }
  return(STATUS_OK); // If 'A' command makes it to here, then everything's ok.
}

void asmcnc_RGB_off(){
	TCCR3B &=~(1<<CS32);
	TCCR3B &=~(1<<WGM33);
	TCCR3A &=~(1<<WGM31);
	TCCR3A |= (1<<WGM30);
	OCR3B=0; OCR3C=0; OCR3A=0;}

void asmcnc_RGB_white(){
	TCCR3B &=~(1<<CS32);
	TCCR3B &=~(1<<WGM33);
	TCCR3A &=~(1<<WGM31);
	TCCR3A |= (1<<WGM30);
	OCR3B=0xFF; OCR3C=0xFF; OCR3A=0xFF;}


void asmcnc_RGB_red(){OCR3A=0xFF; OCR3B=0; OCR3C=0;}

void asmcnc_RGB_red_flash(){		 //Configure PWM for long run time to give visable flash
	TCCR3B |=((1<<CS32)|(1<<WGM33)); //Set timer to 1024 pre-scaler
	TCCR3A |=(1<<WGM31);
	TCCR3A &=~(1<<WGM30);
	ICR3 = 0x2FFF;					//Timer max count
	OCR3A=0x20F0; OCR3B=0; OCR3C=0; //MOSFET settings
}

