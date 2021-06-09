/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2017 Sungeun K. Jeon for Gnea Research LLC
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


static float pwm_gradient; // Precalulated value to speed up rpm to PWM conversions.

static spindle_digital_params spindle_parameters;

void spindle_init()
{    
  // Configure variable spindle PWM and enable pin, if required.
  SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
  SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; // Configure PWM output compare timer
  SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
  SPINDLE_OCRA_REGISTER = SPINDLE_OCRA_TOP_VALUE; // Set the top value for 16-bit fast PWM mode
  SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
  //SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.

  pwm_gradient = SPINDLE_PWM_RANGE/(settings.rpm_max-settings.rpm_min);
  spindle_stop();
  
  memset(&spindle_parameters, 0, sizeof(spindle_digital_params));
  
}


uint8_t spindle_get_state()
{
  #ifdef INVERT_SPINDLE_ENABLE_PIN
    if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT)) && (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT))) {
  #else
    if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT)) && (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT))) {
  #endif
    //if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
    //else { return(SPINDLE_STATE_CW); }
    return(SPINDLE_STATE_CW);
  }
	return(SPINDLE_STATE_DISABLE);
}


// Disables the spindle and sets PWM output to zero when PWM variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
void spindle_stop()
{
    SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.

  #ifdef INVERT_SPINDLE_ENABLE_PIN
    SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
  #else
    SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
  #endif
}


// Sets spindle speed PWM output and enable pin, if configured. Called by spindle_set_state()
// and stepper ISR. Keep routine small and efficient.
void spindle_set_speed(uint16_t pwm_value)
{
#ifdef INVERT_SPINDLE_PWM
  SPINDLE_OCR_REGISTER = SPINDLE_PWM_MAX_VALUE - pwm_value; // ASMCNC in case inverted FET based filter is used
#else
  SPINDLE_OCR_REGISTER = pwm_value; // Set PWM output level.
#endif
  #ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
    if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
      spindle_stop();
    } else {
      SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
      #else
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
      #endif
    }
  #else
#ifdef INVERT_SPINDLE_PWM
    if (pwm_value == SPINDLE_PWM_MAX_VALUE) { // ASMCNC handle PWM off case when inverted FET based filter is used
#else
    if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
#endif
      SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
    } else {
      SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
    }
  #endif
}


#ifdef ENABLE_PIECEWISE_LINEAR_SPINDLE

  // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
  uint16_t spindle_compute_pwm_value(float rpm) // 328p PWM register is 8-bit.
  {
    uint16_t pwm_value;
    rpm *= (0.010*sys.spindle_speed_ovr); // Scale by spindle speed override value.
    // Calculate PWM register value based on rpm max/min settings and programmed rpm.
    if ((settings.rpm_min >= settings.rpm_max) || (rpm >= RPM_MAX)) {
      rpm = RPM_MAX;
      pwm_value = SPINDLE_PWM_MAX_VALUE;
    } else if (rpm <= RPM_MIN) {
      if (rpm == 0.0) { // S0 disables spindle
        pwm_value = SPINDLE_PWM_OFF_VALUE;
      } else {
        rpm = RPM_MIN;
        pwm_value = SPINDLE_PWM_MIN_VALUE;
      }
    } else {
      // Compute intermediate PWM value with linear spindle speed model via piecewise linear fit model.
      #if (N_PIECES > 3)
        if (rpm > RPM_POINT34) {
          pwm_value = floor(RPM_LINE_A4*rpm - RPM_LINE_B4);
        } else 
      #endif
      #if (N_PIECES > 2)
        if (rpm > RPM_POINT23) {
          pwm_value = floor(RPM_LINE_A3*rpm - RPM_LINE_B3);
        } else 
      #endif
      #if (N_PIECES > 1)
        if (rpm > RPM_POINT12) {
          pwm_value = floor(RPM_LINE_A2*rpm - RPM_LINE_B2);
        } else 
      #endif
      {
        pwm_value = floor(RPM_LINE_A1*rpm - RPM_LINE_B1);
      }
    }
    sys.spindle_speed = rpm;
    return(pwm_value);
  }

#else 

  // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
  uint16_t spindle_compute_pwm_value(float rpm) // Mega2560 PWM register is 16-bit.
  {
	uint16_t pwm_value;
	rpm *= (0.010*sys.spindle_speed_ovr); // Scale by spindle speed override value.
	// Calculate PWM register value based on rpm max/min settings and programmed rpm.
	if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
	  // No PWM range possible. Set simple on/off spindle control pin state.
	  sys.spindle_speed = settings.rpm_max;
	  pwm_value = SPINDLE_PWM_MAX_VALUE;
	} else if (rpm <= settings.rpm_min) {
	  if (rpm == 0.0) { // S0 disables spindle
		sys.spindle_speed = 0.0;
		pwm_value = SPINDLE_PWM_OFF_VALUE;
	  } else { // Set minimum PWM output
		sys.spindle_speed = settings.rpm_min;
		pwm_value = SPINDLE_PWM_MIN_VALUE;
	  }
	} else { 
	  // Compute intermediate PWM value with linear spindle speed model.
	  // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
	  sys.spindle_speed = rpm;
	  pwm_value = floor((rpm-settings.rpm_min)*pwm_gradient) + SPINDLE_PWM_MIN_VALUE;
	}
	return(pwm_value);
  }

#endif  

// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
void spindle_set_state(uint8_t state, float rpm)
{
  if (sys.abort) { return; } // Block during abort.
  if (state == SPINDLE_DISABLE) { // Halt or set spindle direction and rpm.
  
    sys.spindle_speed = 0.0;
    spindle_stop();
  
  } else {
  
//    if (state == SPINDLE_ENABLE_CW) {
//      SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
//    } else {
//      SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
//    }

    // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
    if (settings.flags & BITFLAG_LASER_MODE) { 
      if (state == SPINDLE_ENABLE_CCW) { rpm = 0.0; } // TODO: May need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);
    }
    spindle_set_speed(spindle_compute_pwm_value(rpm));
    //spindle_speed_feedback_rpm_updated(rpm);

    #ifndef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
      #else
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
      #endif   
    #endif
  
  }
  
  sys.report_ovr_counter = 0; // Set to report change immediately
}


// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails 
// if an abort or check-mode is active.
void spindle_sync(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
  spindle_set_state(state,rpm);
}

/* spindle signal feedback loop update */
void spindle_nudge_pwm(float correctedSpindleSpeedRPM){
    spindle_set_speed(spindle_compute_pwm_value(correctedSpindleSpeedRPM));
}


uint16_t calcul_crc16(uint8_t *array,uint8_t size)
{
    uint8_t n,nb_octet,t=1;
    uint16_t carry,octet,CRC16,poly;

    octet=(array[t]&0x00FF);  // initialisation with the first byte
    poly=0xA001;    // Polynome 0x8005 (Polynom is mirrored)
    CRC16=0xFFFF;  // handover of initial value

    for(nb_octet=0;nb_octet<size;nb_octet++) // calculation with all bytes
    {
        CRC16=CRC16^octet;  // XOR with the new byte
        for(n=0;n<8;n++)    // 8 times per byte
        {
            carry=(CRC16&0x0001); // carry is value of bit0 before right shift
            CRC16=((CRC16>>1)&(0x7FFF)); // 1 right shift and MSB (bit 15)=0

            if(carry&0x0001)
            CRC16=CRC16^poly; // XOR with the polynome
        }
        octet=(array[++t]&0x00FF); // next byte
    }
    return (CRC16);
}


void spindle_read_digital(void){
    /* check is enough bytes are received in the buffer */
    uint8_t bytes_received;
    bytes_received = serial2_get_rx_buffer_count();
    if ( bytes_received >= DIGITAL_SPINDLE_MESSAGE_SIZE){
        /* find pair of header bytes in the buffer and try to decode CRC */
        uint8_t header_byte1, header_byte2;
        header_byte1 = serial2_read();
        header_byte2 = serial2_read();
        
        if ( (header_byte1 == DIGITAL_SPINDLE_MSG_HEADER_BYTE) && (header_byte2 == DIGITAL_SPINDLE_MSG_HEADER_BYTE) ){
            /* try to decode CRC */
            uint8_t idx;
            uint8_t buffer[DIGITAL_SPINDLE_MESSAGE_SIZE];
            buffer[0] = DIGITAL_SPINDLE_MSG_HEADER_BYTE;
            buffer[1] = DIGITAL_SPINDLE_MSG_HEADER_BYTE;
            
            /* read the rest of the message from UART buffer into local buffer */
            for (idx=2; idx<DIGITAL_SPINDLE_MESSAGE_SIZE; idx++){
                buffer[idx] = serial2_read();
            }
            /* compute CRC16 and compare with received CRC */
            uint16_t CRC16_received, CRC16_calculated;
            CRC16_received = uint16_decode(&buffer[DIGITAL_SPINDLE_CRC_POS]);
            CRC16_calculated = calcul_crc16(buffer, DIGITAL_SPINDLE_CRC_POS);
            
            if ( CRC16_received == CRC16_calculated ){
                /* if header pattern found and CRC matches unpack all parameters from the packet:*/
                spindle_parameters.serial_number            = uint16_decode(&buffer[DIGITAL_SPINDLE_SERIAL_POS]);
                spindle_parameters.production_year          =                buffer[DIGITAL_SPINDLE_YEAR_POS  ] ;
                spindle_parameters.production_week          =                buffer[DIGITAL_SPINDLE_WEEK_POS  ] ;
                spindle_parameters.firmware_version         =                buffer[DIGITAL_SPINDLE_FWVER_POS ] ;
                spindle_parameters.load                     = uint16_decode(&buffer[DIGITAL_SPINDLE_LOAD_POS  ]);
                spindle_parameters.temperature              =       (int8_t) buffer[DIGITAL_SPINDLE_TEMP_POS  ] ;
                spindle_parameters.rpm                      = uint16_decode(&buffer[DIGITAL_SPINDLE_RPM_POS   ]);
                spindle_parameters.remaining_kill_time_s    =                buffer[DIGITAL_SPINDLE_KILL_POS  ] ;
                spindle_parameters.total_run_time_s         = uint32_decode(&buffer[DIGITAL_SPINDLE_RUN_POS   ]);
                spindle_parameters.brush_run_time_s         = uint24_decode(&buffer[DIGITAL_SPINDLE_BRUSH_POS ]);
                                   
                /* call itself again in case more bytes are available from buffer */
                bytes_received = serial2_get_rx_buffer_count();
                if ( bytes_received >= DIGITAL_SPINDLE_MESSAGE_SIZE){                
                    system_set_exec_heartbeat_command_flag(SPINDLE_READ_COMMAND);/* notify main loop that digital Spindle read shall be executed */
                }                    
            } //if ( CRC16_received == CRC16_calculated ){
            else{
                /*if header pattern found but CRC does not match:
                    1) search for another header in the buffer 
                    2) if found - rewind serial buffer tail to that position 
                    3) start over 
                    4) if not found - return*/
                uint8_t another_header_pattern_found = 0;
                
                for (idx=2; idx < (DIGITAL_SPINDLE_MESSAGE_SIZE - 1) ; idx++){
                    if ( (buffer[idx] == DIGITAL_SPINDLE_MSG_HEADER_BYTE) && (buffer[idx+1] == DIGITAL_SPINDLE_MSG_HEADER_BYTE) ){
                        another_header_pattern_found = 1;
                        break; /* for idx loop */
                    } //if ( (buffer[idx] == DIGITAL_SPINDLE_MSG_HEADER_BYTE) && (buffer[idx+1] == DIGITAL_SPINDLE_MSG_HEADER_BYTE) ){
                } //for (idx=2; idx < (DIGITAL_SPINDLE_MESSAGE_SIZE - 1) ; idx++){
                
                if ( another_header_pattern_found == 1){
                    /* rewind serial buffer tail to that position and start over */
                    serial2_rewind( DIGITAL_SPINDLE_MESSAGE_SIZE - idx );
                    system_set_exec_heartbeat_command_flag(SPINDLE_READ_COMMAND);/* notify main loop that digital Spindle read shall be executed */
                } //if ( another_header_pattern_found == 1){
                else{
                    /* header pattern found, but CRC does not match and no other header pattern is found in the buffer. return */
                    return;
                } // else if ( another_header_pattern_found == 1){
                
            } //else if ( CRC16_received == CRC16_calculated ){
            
        } //if ( (header_byte1 == DIGITAL_SPINDLE_MSG_HEADER_BYTE) && (header_byte2 == DIGITAL_SPINDLE_MSG_HEADER_BYTE) ){
        else{ 
            /* call itself recursively until header pattern is found of bytes available is exhausted */                
            system_set_exec_heartbeat_command_flag(SPINDLE_READ_COMMAND);/* notify main loop that digital Spindle read shall be executed */
            //spindle_read_digital();
        } //else{ //if ( (header_byte1 == DIGITAL_SPINDLE_MSG_HEADER_BYTE) && (header_byte2 == DIGITAL_SPINDLE_MSG_HEADER_BYTE) ){
            
    } //if ( bytes_available >= DIGITAL_SPINDLE_MESSAGE_SIZE){
        
}

void spindle_digital_print_info(void){
    printPgmString(PSTR("Spindle serial number: "));printInteger( spindle_parameters.serial_number      ); printPgmString(PSTR("\n"));
    printPgmString(PSTR("Production year: "));      printInteger( spindle_parameters.production_year    ); printPgmString(PSTR("\n"));
    printPgmString(PSTR("Production week: "));      printInteger( spindle_parameters.production_week    ); printPgmString(PSTR("\n"));
    printPgmString(PSTR("Firmware version: "));     printInteger( spindle_parameters.firmware_version   ); printPgmString(PSTR("\n"));
    printPgmString(PSTR("Total run time: "));       printInteger( spindle_parameters.total_run_time_s   ); printPgmString(PSTR("\n"));
    printPgmString(PSTR("Brush run time: "));       printInteger( spindle_parameters.brush_run_time_s   ); printPgmString(PSTR("\n"));
}

void spindle_digital_print_real_time(void){
    printInteger( spindle_parameters.load                   ); printPgmString(PSTR(","));
    printInteger( spindle_parameters.temperature            ); printPgmString(PSTR(","));
    printInteger( spindle_parameters.remaining_kill_time_s  );
}

void spindle_digital_print_rpm(void){
    printInteger( spindle_parameters.rpm                    ); 
}

