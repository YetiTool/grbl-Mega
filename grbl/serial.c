/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

#define RX_RING_BUFFER (RX_BUFFER_SIZE+1)
#define TX_RING_BUFFER (TX_BUFFER_SIZE+1)

uint8_t serial_rx_buffer[RX_RING_BUFFER];
uint16_t serial_rx_buffer_head = 0;
volatile uint16_t serial_rx_buffer_tail = 0;

uint8_t serial_tx_buffer[TX_RING_BUFFER];
uint16_t serial_tx_buffer_head = 0;
volatile uint16_t serial_tx_buffer_tail = 0;

uint8_t serial_rx_rgb_state = 0; 			/* RGB HEX Rx state machine state */
uint8_t serial_rx_rgb_count = 0; 			/* number of currently received hex characters */
uint8_t serial_rx_rgb_byte_buffer[3] = {0}; /* buffer to hold output int values for RGB codes */
uint8_t serial_rx_rgb_nibble = 0;			/* declaring ISR variable here to minimise declarations in ISR */


// Returns the number of bytes available in the RX serial buffer.
uint8_t serial_get_rx_buffer_available()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (serial_rx_buffer_head-rtail)); }
  return((rtail-serial_rx_buffer_head-1));
}


// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count()
{
  uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_tx_buffer_head >= ttail) { return(serial_tx_buffer_head-ttail); }
  return (TX_RING_BUFFER - (ttail-serial_tx_buffer_head));
}


void serial_init()
{
  // Set baud rate
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
  #endif
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;

  // enable rx, tx, and interrupt on complete reception of a byte
  UCSR0B |= (1<<RXEN0 | 1<<TXEN0 | 1<<RXCIE0);

  // defaults to 8-bit, no parity, 1 stop bit
}


// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data) {
  // Calculate next head
  uint16_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_RING_BUFFER) { next_head = 0; }

  // Wait until there is space in the buffer
  while (next_head == serial_tx_buffer_tail) {
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;

  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  UCSR0B |=  (1 << UDRIE0);
}


// Data Register Empty Interrupt handler
ISR(SERIAL_UDRE)
{
  uint16_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)

  // Send a byte from the buffer
  UDR0 = serial_tx_buffer[tail];

  // Update tail position
  tail++;
  if (tail == TX_RING_BUFFER) { tail = 0; }

  serial_tx_buffer_tail = tail;

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == serial_tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}


// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{
  uint16_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial_rx_buffer[tail];

    tail++;
    if (tail == RX_RING_BUFFER) { tail = 0; }
    serial_rx_buffer_tail = tail;

    return data;
  }
}


ISR(SERIAL_RX)
{
  uint8_t data = UDR0;
  uint16_t next_head;

  /* BK: hack into the ISR routine to intercept RGB hex command and bypass serial_rx_buffer
   * code in below switch statement is optimised for minimum number of CPU cycles, could be optimised further if field tests shows issues*/
  switch (serial_rx_rgb_state) {/* RGB HEX Rx state machine state */
	case RGB_HEX_RTL_IDLE: /*  normal state, usual operation */
		break;
	case RGB_HEX_RTL_RX:   /*  real-time hex code reception ongoing */

		/* Convert hex to byte for speed here, we only have ~80 instructions till next UART character. */
		serial_rx_rgb_nibble = char2intValidate(data); /* returns error 17 if char is outside of "0123456789ABCDEFabcdef"  */

		if ( serial_rx_rgb_nibble < 16 ){ /* correct char received, if char is within hex code then fill the buffer */

			/* convert each pair of hex codes into byte, again for speed */
			if (serial_rx_rgb_count%2 == 0){ /*even */
				serial_rx_rgb_byte_buffer[(serial_rx_rgb_count/2)] = serial_rx_rgb_nibble*16;
			}
			else{ /* odd */
				serial_rx_rgb_byte_buffer[((serial_rx_rgb_count-1)/2)] += serial_rx_rgb_nibble;
			}
			serial_rx_rgb_count++;

			if (serial_rx_rgb_count > 5){ /* hex code reception completed, set RGB and reset FSM and counter */
				serial_rx_rgb_state = RGB_HEX_RTL_IDLE;
				serial_rx_rgb_count = 0;
				asmcnc_RGB_setup(); 		/* Setup pre-scaling = 8 and Waveform Generation Mode: PWM, Phase Correct, 8-bit */
				/* decoded RGB values:
				 * R = buffer [0]
				 * G = buffer [1]
				 * B = buffer [2]
				 * */
				OCR3A=serial_rx_rgb_byte_buffer[0]; /* R */
				OCR3B=serial_rx_rgb_byte_buffer[1]; /* G */
				OCR3C=serial_rx_rgb_byte_buffer[2]; /* B */
			}
		}
		else{
			/* ERROR, abort hex code reception and continue with normal operation */
			//serial_rx_rgb_state = RGB_HEX_RTL_ERR; /* if not hex code then exit with error */
			serial_rx_rgb_state = RGB_HEX_RTL_IDLE;
			serial_rx_rgb_count = 0;
			report_status_message(ASMCNC_STATUS_INVALID_STATEMENT);
			break; /* break the switch and proceed as normal */
		}
		return; /* exit the ISR - this line is where serial bypass actually happens */

	//case RGB_HEX_RTL_ERR:  /* FAULT - other than "0123456789ABCDEF" char received */
	//	break;
	default:
		break;
	}

  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the main buffer, but these set system state flag bits for realtime execution.
  switch (data) {
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
    case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // Set as true
    case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // Set as true
    case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // Set as true
    case CMD_RGB_HEX:
		/* this character defines the first char in the list of 7 chars that need to be
		 * going pass serial_rx_buffer and executed immediately at the end of 7th char
		 * State machine:
		 * RGB_HEX_RTL_IDLE - normal state, usual operation
		 * RGB_HEX_RTL_RX   - hex code reception ongoing
		 * RGB_HEX_RTL_ERR  - FAULT - other than "0123456789ABCDEF" char received
		 *  */
		serial_rx_rgb_state = RGB_HEX_RTL_RX; /* initialise state machine and start bypassing the buffer */
		serial_rx_rgb_count = 0;
		break;
    //ASM Mod to turn off door flashing red LED on sytem cycle start command
    case CMD_RGB_WHITE:	{asmcnc_RGB_off(); asmcnc_RGB_white();} break;
    default :
      if (data > 0x7F) { // Real-time control characters are extended ACSII only.
        switch(data) {
          case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // Set as true
          case CMD_JOG_CANCEL:   
            if (sys.state & STATE_JOG) { // Block all other states from invoking motion cancel.
              system_set_exec_state_flag(EXEC_MOTION_CANCEL); 
            }
            break; 
          #ifdef DEBUG
            case CMD_DEBUG_REPORT: {uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;} break;
          #endif
          case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
          case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
          case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
          case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
          case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
          case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
          case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
          case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
          case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
          case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
          case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
          case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
          case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
          case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
          case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
          case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
        }
        // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
      } else { // Write character to buffer
        next_head = serial_rx_buffer_head + 1;
        if (next_head == RX_RING_BUFFER) { next_head = 0; }

        // Write data to buffer unless it is full.
        if (next_head != serial_rx_buffer_tail) {
          serial_rx_buffer[serial_rx_buffer_head] = data;
          serial_rx_buffer_head = next_head;
        }
      }
  }
}


void serial_reset_read_buffer()
{
  serial_rx_buffer_tail = serial_rx_buffer_head;
}
