/*
reduce TMC_REGISTER_COUNT to 16
add handling of incoming messages and parameter change requests
add handling of status and diag flags - 
    - idle detect -> reduce power
    - no idle detect -> increase power.
*/

#include "grbl.h"
#include "spi_to_tmc.h"

/********************************************** below are Atmega2560 specific - timers, SPI, pins etc **********************************************/

void tmc_pin_write(uint32_t level, uint32_t pin){
	if (level==0) TMC_PORT &=~(1<<pin); /* clear pin */
	else          TMC_PORT |= (1<<pin); /* set pin */
}

/* the only remaining unused timer is 8bit timer 2*/
/* initialise timer to periodically poll TMC motor controllers */
void asmcnc_TMC_Timer2_setup(void){

	TCCR2A = 0;	//Clear timer2 registers
	TCCR2B = 0;


	/* Setup Waveform Generation Mode: CTC */
	TCCR2B |= (1<<WGM22); /* set bit */
	TCCR2A |= (1<<WGM21); /* set bit */
	TCCR2A |= (1<<WGM20); /* set bit */

	/* Setup pre-scaling = 1024 to ensure slowest rate of 60.5Hz / 15ms ticks */
	TCCR2B |=(1<<CS20); /* set bit */
	TCCR2B |=(1<<CS21); /* set bit */
	TCCR2B |=(1<<CS22); /* set bit */

	///* Setup pre-scaling = 256 to ensure slowest of 240.5Hz / 4.1ms ticks */
	////TCCR2B |=(1<<CS20); /* set bit */
	//TCCR2B |=(1<<CS21); /* set bit */
	//TCCR2B |=(1<<CS22); /* set bit */

	///* Setup pre-scaling = 128 to ensure slowest of 581Hz / 2.05ms ticks */
	//TCCR2B |=(1<<CS20); /* set bit */
	////TCCR2B |=(1<<CS21); /* set bit */
	//TCCR2B |=(1<<CS22); /* set bit */


	/* setup compare register to achieve wanted SPI polling frequency. Some example values:
	 * 0xFF:
	 * value	F, Hz	T, ms
	 * 0xFF		61.03	16.3
	 * 0xC3		79.71	12.5
	 * 0x9C		99.52	10.0
	 * 0x75		132.4	7.55
	 * 0x4E		197.7	5.05
	 * */

	//OCR2A = 0x9C; /* 2.512ms with prescaler 256*/
    //OCR2A = 0x3E; /* 1.008ms with prescaler 256*/
  	OCR2A = SPI_TIMER_CYCLE_PER_READ;


	/* Zero timer 2 */
	TCNT2=0;

	/* setup port B to see pin OC2A toggling */
	//TMC_DDR	|= TMC_PORT_MASK;
	// Compare Output Mode, non-PWM Mode
	//TCCR2A |= (1<<COM2A0); 	//Toggle OC2A on Compare Match

	/* Enable timer2 Interrupt */
	TIMSK2 |= (1<<OCIE2A); //Timer/Counter2 Output Compare Match A Interrupt Enable

}


void spi_hw_init(void){

	//SPI_MasterInit();
    
	asmcnc_TMC_Timer2_setup(); /* initialise timer to periodically poll TMC motor controllers */

    /* configure CS pins and pull them high */
    //tmc_pin_write(1, SPI_CS_X_PIN);
    //tmc_pin_write(1, SPI_CS_Y_PIN);
    //tmc_pin_write(1, SPI_CS_Z_PIN);
    
}



/********************************************** below is common between platforms (nRF / Atmega) **********************************************/


/*  Function for passing any pending request from the buffer to the SPI hardware.*/
ISR(TIMER2_COMPA_vect)
{
	sei(); // Re-enable interrupts to allow Stepper Interrupt to fire on-time.
    
#ifndef ENABLE_SOFTWARE_DEBOUNCE
	/* feed the dog */
	asm("WDR");
#endif // ENABLE_SOFTWARE_DEBOUNCE

#ifdef DEBUG_SPI_ENABLED
debug_pin_write(1, DEBUG_0_PIN); /* whole ISR routine 18-22us */
#endif
    
	/* slow down polling the drivers, 1 is 16ms , 61 is around 1s */
    //static uint8_t skip_count   = ((UPTIME_TICK_PERIOD_MS *1000UL)/SPI_READ_OCR_PERIOD_US)-2; /* initialize with max-2 to ensure read soon after boot*/
    static uint8_t uptime_count = ((SPI_READ_ALL_PERIOD_MS*1000UL)/SPI_READ_OCR_PERIOD_US)-2; /* initialize with max-2 to ensure read soon after boot*/

    /* notify main loop that ADC state machine tick need to be incremented */    
    system_set_exec_heartbeat_command_flag(ADC_SET_AND_FIRE_COMMAND);    
    
    if (++uptime_count % ((UPTIME_TICK_PERIOD_MS *1000UL)/SPI_READ_OCR_PERIOD_US) == 0)  { /* set uptime interval to 1s */
        system_set_exec_heartbeat_command_flag(UPTIME_INCREMENT_COMMAND);
        uptime_count = 0;
    }
    
    
#ifdef DEBUG_SPI_ENABLED
    debug_pin_write(0, DEBUG_0_PIN);
    debug_pin_write(1, DEBUG_0_PIN); /* second cycle to indicate that this time the "if (++skip_count" came through */
    debug_pin_write(0, DEBUG_0_PIN);
#endif
}



