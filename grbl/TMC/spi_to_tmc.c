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


void SPI_MasterInit(void)
{

    /* Warning: if the SS pin ever becomes a LOW INPUT then SPI automatically switches to Slave, so the data direction of the SS pin MUST be kept as OUTPUT.
     * if the SS pin is not already configured as an output then set it high (to enable the internal pull-up resistor)
     * When the SS pin is set as OUTPUT, it can be used as a general purpose output port (it doesn't influence SPI operations). */
    tmc_pin_write(1, SPI_SS_PIN);

	/* Set MOSI and SCK output, all others input */
	TMC_DDR			|= TMC_PORT_MASK;

    /* enable pull-up resistor on MISO pin */
    //tmc_pin_write(1, SPI_MISO_PIN);

    /* Enable SPI, Master */
    SPCR |= ( (1<<SPE)|(1<<MSTR) );

    /* SPI clocking options: */
    /* option 1: set clock rate fck/4 = 4MHz - defautl config - nothing to be written */
    /* option 2: set clock rate fck/16 = 1MHz*/
    SPCR |= (1<<SPR0);
    /* option 3: set clock rate fck/64 = 0.25MHz*/
    //SPCR |= (1<<SPR1);

	/* Set phase and polarity to mode3 */
	SPCR |= ( (1<<CPOL)|(1<<CPHA) );

	/* enable SPI interrupts  */
	SPCR |= (1<<SPIE);

}

void spi_hw_init(void){

	SPI_MasterInit();
    
	asmcnc_TMC_Timer2_setup(); /* initialise timer to periodically poll TMC motor controllers */

    /* configure CS pins and pull them high */
    tmc_pin_write(1, SPI_CS_X_PIN);
    tmc_pin_write(1, SPI_CS_Y_PIN);
    tmc_pin_write(1, SPI_CS_Z_PIN);
    
}



/********************************************** below is common between platforms (nRF / Atmega) **********************************************/


/* buffer to queue incoming from BLE packets and post them through SPI */
static tx_spi_msg_t	m_spi_tx_buffer[SPI_TX_BUFFER_SIZE];  				/* Transmit buffer for the messages that will be transmitted to the central. */
static uint32_t    	m_spi_tx_insert_index		= 0;        			/* Current index in the transmit buffer where the next message should be inserted. */
static uint32_t    	m_spi_tx_index        		= 0;        			/* Current index in the transmit buffer containing the next message to be transmitted. */
static uint8_t      current_transfer_type       = 0;                    /* 3 for single or 5 for dual */
uint8_t m_spi_rx_data[TX_BUF_SIZE_DUAL]; 						        /* buffer storage for Rx data */


// Used to avoid ISR nesting of the "TMC SPI interrupt". Should never occur though.
static uint8_t spi_busy = false;

static spi_state_type_t SPI_current_state 		= SPI_STATE_IDLE; 		/* flag to distinguish SPI state */


/* below function is to schedule event in the spi send queue */
void spi_schedule_single_tx(TMC2590TypeDef *tmc2590_1, uint8_t *data, uint8_t size, uint8_t rdsel)
{
    m_spi_tx_buffer[m_spi_tx_insert_index].buf_size = size;
    m_spi_tx_buffer[m_spi_tx_insert_index].tmc2590_1 = tmc2590_1;
	//memset(m_spi_tx_buffer[m_spi_tx_insert_index].m_spi_tx_buf, 0, size);
  	memcpy(m_spi_tx_buffer[m_spi_tx_insert_index].m_spi_tx_buf, data, size);		
    m_spi_tx_buffer[m_spi_tx_insert_index].rdsel = rdsel;
	m_spi_tx_insert_index++;
    m_spi_tx_insert_index &= SPI_TX_BUFFER_MASK;    
}

void spi_schedule_dual_tx(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2, uint8_t *data, uint8_t size, uint8_t rdsel)
{
    m_spi_tx_buffer[m_spi_tx_insert_index].buf_size = size;
    m_spi_tx_buffer[m_spi_tx_insert_index].tmc2590_1 = tmc2590_1;
    m_spi_tx_buffer[m_spi_tx_insert_index].tmc2590_2 = tmc2590_2;
	//memset(m_spi_tx_buffer[m_spi_tx_insert_index].m_spi_tx_buf, 0, size);
  	memcpy(m_spi_tx_buffer[m_spi_tx_insert_index].m_spi_tx_buf, data, size);		
    m_spi_tx_buffer[m_spi_tx_insert_index].rdsel = rdsel;
	m_spi_tx_insert_index++;
    m_spi_tx_insert_index &= SPI_TX_BUFFER_MASK;    
}

void spi_process_tx_queue(void){
    if ( spi_busy == false ){
        /* if something is waiting then send it */
        if (m_spi_tx_index != m_spi_tx_insert_index){
            spi_busy = true; /*prevent reentry while transaction is active*/
        
            /* keep log of next element - is it 3 or 5 bytes (single or dual motors) */
            current_transfer_type = m_spi_tx_buffer[m_spi_tx_index].buf_size; //TX_BUF_SIZE_DUAL) { //or TX_BUF_SIZE_SINGLE
        
            /* start state 1 of the transfer: pull SSx low	write byte_x1 to the SPDR */        
            SPI_current_state = SPI_STATE_1;
        
            /* pull CS pin down */
            tmc_pin_write(0, m_spi_tx_buffer[m_spi_tx_index].tmc2590_1->config->channel);

            /* initiate transfer by writing first byte to the data register */
            SPDR = m_spi_tx_buffer[m_spi_tx_index].m_spi_tx_buf[0];
         
            /* next step will be handled by interrupt handler ISR_SPI_STC_vect */
        
        } //if (m_spi_tx_index != m_spi_tx_insert_index){
        else {
            /*nothing else left in a queue, release busy flag */
            //spi_busy = false;
            //SPI_current_state = SPI_STATE_IDLE;
        
            /* indicate to TMC2590 loops that reading is completed (required for homing cycle) */
            tmc_spi_queue_drain_complete();
        
            /* indicate to main loop to process all responses and update the current status of controller's parameters */
            system_set_exec_tmc_command_flag(TMC_SPI_PROCESS_COMMAND);
        }
        
    } //if ( spi_busy == false ){        
    else{
        //printPgmString(PSTR("\n--- SPI process BUSY ---\n"));
    } //else if ( spi_busy == false ){           
}


ISR(SPI_STC_vect)
{
    /* start relevant FSM depend on the single or dual motors */
    
    /* single buffer
    State 1: pull SSz low, write byte_z1 to the SPDR
    state 2: read the RX byte_z1 from SPDR, write byte_z2 to the SPDR
    state 3: read the RX byte_z2 from SPDR, write byte_z3 to the SPDR
    state 4: read the RX byte_z3 from SPDR, pull SSz high
    
    or dual buffer:
    state 1	pull SSx low	write byte_x1 to the SPDR
    state 2	read the RX byte_x1 from SPDR	write byte_x2 to the SPDR
    state 3	read the RX byte_x2 from SPDR	write byte_x3 to the SPDR
    state 4	read the RX byte_x3 from SPDR	write byte_x4 to the SPDR
    state 5	read the RX byte_x4 from SPDR	write byte_x5 to the SPDR
    state 6	read the RX byte_x5 from SPDR	pull SSx high        
    */    

	sei(); // Re-enable interrupts to allow Stepper Interrupt to fire on-time.

#ifdef DEBUG_PINS_ENABLED
	debug_pin_write(1, DEBUG_1_PIN);
#endif
    switch (SPI_current_state)
    {	
        
        case SPI_STATE_1:   /* SPI received byte 1 */				
            SPI_current_state = SPI_STATE_2;						
            //state 2: read the RX byte_z1 from SPDR, write byte_z2 to the SPDR               
            /* copy first received byte to the buffer */
            m_spi_rx_data[0] = SPDR;
            /* initiate transfer by writing next byte to the data register */
            SPDR = m_spi_tx_buffer[m_spi_tx_index].m_spi_tx_buf[1];
        break;
        
        case SPI_STATE_2:   /* SPI received byte 2 */				
            SPI_current_state = SPI_STATE_3;						
            // state 3: read the RX byte_z2 from SPDR, write byte_z3 to the SPDR
            /* copy next received byte to the buffer */
            m_spi_rx_data[1] = SPDR;
            /* initiate transfer by writing next byte to the data register */
            SPDR = m_spi_tx_buffer[m_spi_tx_index].m_spi_tx_buf[2];
        break;
        
        case SPI_STATE_3:   /* SPI received byte 3 */				
            SPI_current_state = SPI_STATE_4;						                
            /* copy next received byte to the buffer */
            m_spi_rx_data[2] = SPDR;
        
            if ( current_transfer_type == TX_BUF_SIZE_DUAL) {
                /* state 4	read the RX byte_x3 from SPDR	write byte_x4 to the SPDR */
                /* initiate transfer by writing next byte to the data register */
                SPDR = m_spi_tx_buffer[m_spi_tx_index].m_spi_tx_buf[3];
            }
            else{
                /* single transfer complete, store result and advance to next queue element */
                // state 4: read the RX byte_z3 from SPDR, pull SSz high                    
                /* deconstruct response */
            	int32_t a;
#ifdef DEBUG_PINS_ENABLED
    debug_pin_write(1, DEBUG_2_PIN);
    debug_pin_write(0, DEBUG_2_PIN);
#endif
    			/* BK profiling: 9us */
                //m_spi_tx_buffer[m_spi_tx_index].tmc2590_1->config->shadowRegister[m_spi_tx_buffer[m_spi_tx_index].tmc2590_1->respIdx] =
                //            TMC2590_VALUE(_8_32(m_spi_rx_data[0], m_spi_rx_data[1], m_spi_rx_data[2], 0) >> 12) ;

    			/* BK profiling: 6.3us */
    			a = TMC2590_VALUE(_8_32(m_spi_rx_data[0], m_spi_rx_data[1], m_spi_rx_data[2], 0) >> 12);
#ifdef DEBUG_PINS_ENABLED
    debug_pin_write(1, DEBUG_2_PIN);
    debug_pin_write(0, DEBUG_2_PIN);
#endif
    			/* BK profiling: 2.7us */
    			m_spi_tx_buffer[m_spi_tx_index].tmc2590_1->config->shadowRegister[m_spi_tx_buffer[m_spi_tx_index].tmc2590_1->respIdx] = a;
#ifdef DEBUG_PINS_ENABLED
    debug_pin_write(1, DEBUG_2_PIN);
    debug_pin_write(0, DEBUG_2_PIN);
#endif
    			/* BK profiling: 2us */
                // set virtual read address for next reply given by RDSEL on given motor, can only change by setting RDSEL in DRVCONF
                //if(m_spi_tx_buffer[m_spi_tx_index].addressIsDrvConf == 1)
                m_spi_tx_buffer[m_spi_tx_index].tmc2590_1->respIdx = m_spi_tx_buffer[m_spi_tx_index].rdsel;
#ifdef DEBUG_PINS_ENABLED
    debug_pin_write(1, DEBUG_2_PIN);
    debug_pin_write(0, DEBUG_2_PIN);
#endif
                
                /* pull CS pin up */
                tmc_pin_write(1, m_spi_tx_buffer[m_spi_tx_index].tmc2590_1->config->channel);

                /* Write SPI returns Success. Increment buffer index and process again in case something is in a queue*/
                m_spi_tx_index++;
                m_spi_tx_index &= SPI_TX_BUFFER_MASK;
                
                spi_busy = false;
                SPI_current_state 		= SPI_STATE_IDLE;                    
                
                /* continue draining the queue: start next SPI transfers*/                    
                spi_process_tx_queue();                                          
            }
            
        break;
        
        case SPI_STATE_4:   /* SPI received byte 4 */				
            SPI_current_state = SPI_STATE_5;						
            // state 5	read the RX byte_x4 from SPDR	write byte_x5 to the SPDR
            /* copy next received byte to the buffer */
            m_spi_rx_data[3] = SPDR;
            /* initiate transfer by writing next byte to the data register */
            SPDR = m_spi_tx_buffer[m_spi_tx_index].m_spi_tx_buf[4];
        break;
        
        case SPI_STATE_5:   /* SPI received byte 5 */				
            SPI_current_state = SPI_STATE_6;						
            // state 6	read the RX byte_x5 from SPDR	pull SSx high        
            /* dual transfer complete, store result and advance to next queue element */
            m_spi_rx_data[4] = SPDR;
        
            /* deconstruct response */
            m_spi_tx_buffer[m_spi_tx_index].tmc2590_1->config->shadowRegister[m_spi_tx_buffer[m_spi_tx_index].tmc2590_1->respIdx] = 
                        _8_32(m_spi_rx_data[2], m_spi_rx_data[3], m_spi_rx_data[4], 0) >> 8 ;
            m_spi_tx_buffer[m_spi_tx_index].tmc2590_2->config->shadowRegister[m_spi_tx_buffer[m_spi_tx_index].tmc2590_2->respIdx] = 
                        TMC2590_VALUE(_8_32(m_spi_rx_data[0], m_spi_rx_data[1], m_spi_rx_data[2], 0) >> 12) ;    
            
            // set virtual read address for next reply given by RDSEL on given motor, can only change by setting RDSEL in DRVCONF
            m_spi_tx_buffer[m_spi_tx_index].tmc2590_1->respIdx = m_spi_tx_buffer[m_spi_tx_index].rdsel;
            m_spi_tx_buffer[m_spi_tx_index].tmc2590_2->respIdx = m_spi_tx_buffer[m_spi_tx_index].rdsel;
            
            /* pull CS pin up */
            tmc_pin_write(1, m_spi_tx_buffer[m_spi_tx_index].tmc2590_1->config->channel);

            /* Write SPI returns Success. Increment buffer index and process again in case something is in a queue*/
            m_spi_tx_index++;
            m_spi_tx_index &= SPI_TX_BUFFER_MASK;
            
            spi_busy = false;
            SPI_current_state 		= SPI_STATE_IDLE;                    
            
            /* continue draining the queue: start next SPI transfers*/                    
            spi_process_tx_queue();

        break;
        
        default:
            break;

    } // switch (SPI_current_state)
#ifdef DEBUG_PINS_ENABLED
    debug_pin_write(0, DEBUG_1_PIN);
#endif
}


/*  Function for passing any pending request from the buffer to the SPI hardware.*/
ISR(TIMER2_COMPA_vect)
{
	sei(); // Re-enable interrupts to allow Stepper Interrupt to fire on-time.
    
#ifdef DEBUG_PINS_ENABLED
debug_pin_write(1, DEBUG_0_PIN);
#endif
    
	/* slow down polling the drivers, 1 is 16ms , 61 is around 1s */
    static uint8_t skip_count = 0 ;
    static uint8_t uptime_count = 0;
    
    if (++uptime_count % ((UPTIME_TICK_PERIOD_MS *1000UL)/SPI_READ_OCR_PERIOD_US) == 0)  { /* set uptime interval to 1s */
        system_set_exec_tmc_command_flag(UPTIME_INCREMENT_COMMAND);
        uptime_count = 0;
    }
    
    if (++skip_count % ((SPI_READ_ALL_PERIOD_MS*1000UL)/SPI_READ_OCR_PERIOD_US) == 0)  { /* set SPI poll interval to 1s */
        #ifdef DEBUG_PINS_ENABLED
        debug_pin_write(0, DEBUG_0_PIN);
        #endif        
        skip_count = 0;

        /* schedule next SPI transfer: indicate to main loop that there is a time to prepare SPI buffer and send it */
        system_set_exec_tmc_command_flag(TMC_SPI_READ_ALL_COMMAND);

        /* if for some reason the SPI was not released (HW glitch or comms loss) wait for 30 timer cycles and reset the busy flag */
        static uint8_t busy_reset_count = 0;
        if ( (spi_busy) && ( busy_reset_count < 30 ) ){
            busy_reset_count++;
            if ( busy_reset_count > 1 ) printPgmString(PSTR("\n!!! SPI BUSY !!!\n")); /* spi is busy for more than 5 us */
            return;
        }

        spi_busy = false;
        SPI_current_state 		= SPI_STATE_IDLE;
        busy_reset_count = 0;
    } //if (++skip_count % ((SPI_READ_ALL_PERIOD_MS*1000UL)/SPI_READ_OCR_PERIOD_US) == 0)  { /* set SPI poll interval to 1s */
    
    
#ifdef DEBUG_PINS_ENABLED
    debug_pin_write(0, DEBUG_0_PIN);
    debug_pin_write(1, DEBUG_0_PIN); /* second cycle to indicate that this time the "if (++skip_count" came through */
    debug_pin_write(0, DEBUG_0_PIN);
#endif
}



