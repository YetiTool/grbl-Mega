/*
  spindle_control.h - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
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

#ifndef spindle_control_h
#define spindle_control_h

#define SPINDLE_NO_SYNC false
#define SPINDLE_FORCE_SYNC true

#define SPINDLE_STATE_DISABLE  0  // Must be zero.
#define SPINDLE_STATE_CW       bit(0)
#define SPINDLE_STATE_CCW      bit(1)

#define DIGITAL_SPINDLE_MSG_HEADER_BYTE 0xAA /* two bytes constitute header */
#define DIGITAL_SPINDLE_MSG_HEADER_SIZE 2
#define DIGITAL_SPINDLE_MSG_SERIAL_SIZE 2    /* Serial Number: 000003 */
#define DIGITAL_SPINDLE_MSG_YEAR_SIZE   1    /* Production year */
#define DIGITAL_SPINDLE_MSG_WEEK_SIZE   1    /* Production week */
#define DIGITAL_SPINDLE_MSG_FWVER_SIZE  1    /* Firmware version  */
/* Unique device  ID: 3-4 Serial Number: 000003, 5 Production year, 6 Production week,  7 Firmware version */
#define DIGITAL_SPINDLE_MSG_UID_SIZE    (DIGITAL_SPINDLE_MSG_SERIAL_SIZE + DIGITAL_SPINDLE_MSG_YEAR_SIZE + DIGITAL_SPINDLE_MSG_WEEK_SIZE + DIGITAL_SPINDLE_MSG_FWVER_SIZE)
#define DIGITAL_SPINDLE_MSG_LOAD_SIZE   2    /* Spindle loading	e.g. 11 A --> 12100 qdA */
#define DIGITAL_SPINDLE_MSG_TEMP_SIZE   1    /* Temperature	"physically measured temperature of the cooling air of the milling motor. The resolution of the measurement is 4 Kelvin" */
#define DIGITAL_SPINDLE_MSG_RPM_SIZE    2    /* Actual RPM */
#define DIGITAL_SPINDLE_MSG_KILL_SIZE   1    /* Remaining kill-time	Time period before the motor is switched off.  Since the load on the motor is not absolutely constant over time, the countdown will  decremented with a  non constant step size, e. g.  160, 159, 158, 154,148,147,148,149,153 */
#define DIGITAL_SPINDLE_MSG_RUN_SIZE    4    /* Total unit run time	"For the operating hours every byte are to be evaluated in binary with powers of 256. The low byte has the multiplier 1, the high byte 256, the high-high byte 256*256 " */
#define DIGITAL_SPINDLE_MSG_BRUSH_SIZE  3    /* Brush run time	"resettable throw  an  bit pattern to analog input e.g.  0,5V(100ms)-0V(100ms) - 0,5V(500ms)-0V(500ms)- 0,5V(100ms)-0V(100ms). As a aknowlege the motorcontrol send  immediately  the message B with the updated value" */
#define DIGITAL_SPINDLE_MSG_CRC_SIZE    2    /* CRC 16 */

#define DIGITAL_SPINDLE_MESSAGE_SIZE (DIGITAL_SPINDLE_MSG_HEADER_SIZE + DIGITAL_SPINDLE_MSG_UID_SIZE + DIGITAL_SPINDLE_MSG_LOAD_SIZE + DIGITAL_SPINDLE_MSG_TEMP_SIZE + DIGITAL_SPINDLE_MSG_RPM_SIZE + DIGITAL_SPINDLE_MSG_KILL_SIZE + DIGITAL_SPINDLE_MSG_RUN_SIZE + DIGITAL_SPINDLE_MSG_BRUSH_SIZE + DIGITAL_SPINDLE_MSG_CRC_SIZE)

#define DIGITAL_SPINDLE_SERIAL_POS  ( DIGITAL_SPINDLE_MSG_HEADER_SIZE                              )    /* Serial Number: 000003 */
#define DIGITAL_SPINDLE_YEAR_POS    ( DIGITAL_SPINDLE_SERIAL_POS + DIGITAL_SPINDLE_MSG_SERIAL_SIZE )    /* Production year */
#define DIGITAL_SPINDLE_WEEK_POS    ( DIGITAL_SPINDLE_YEAR_POS   + DIGITAL_SPINDLE_MSG_YEAR_SIZE   )    /* Production week */
#define DIGITAL_SPINDLE_FWVER_POS   ( DIGITAL_SPINDLE_WEEK_POS   + DIGITAL_SPINDLE_MSG_WEEK_SIZE   )    /* Firmware version  */
#define DIGITAL_SPINDLE_LOAD_POS    ( DIGITAL_SPINDLE_FWVER_POS  + DIGITAL_SPINDLE_MSG_FWVER_SIZE  )    /* Spindle loading	e.g. 11 A --> 12100 qdA */
#define DIGITAL_SPINDLE_TEMP_POS    ( DIGITAL_SPINDLE_LOAD_POS   + DIGITAL_SPINDLE_MSG_LOAD_SIZE   )    /* Temperature	"physically measured temperature of the cooling air of the milling motor. The resolution of the measurement is 4 Kelvin" */
#define DIGITAL_SPINDLE_RPM_POS     ( DIGITAL_SPINDLE_TEMP_POS   + DIGITAL_SPINDLE_MSG_TEMP_SIZE   )    /* Actual RPM */
#define DIGITAL_SPINDLE_KILL_POS    ( DIGITAL_SPINDLE_RPM_POS    + DIGITAL_SPINDLE_MSG_RPM_SIZE    )    /* Remaining kill-time	Time period before the motor is switched off.  Since the load on the motor is not absolutely constant over time, the countdown will  decremented with a  non constant step size, e. g.  160, 159, 158, 154,148,147,148,149,153 */
#define DIGITAL_SPINDLE_RUN_POS     ( DIGITAL_SPINDLE_KILL_POS   + DIGITAL_SPINDLE_MSG_KILL_SIZE   )    /* Total unit run time	"For the operating hours every byte are to be evaluated in binary with powers of 256. The low byte has the multiplier 1, the high byte 256, the high-high byte 256*256 " */
#define DIGITAL_SPINDLE_BRUSH_POS   ( DIGITAL_SPINDLE_RUN_POS    + DIGITAL_SPINDLE_MSG_RUN_SIZE    )    /* Brush run time	"resettable throw  an  bit pattern to analog input e.g.  0,5V(100ms)-0V(100ms) - 0,5V(500ms)-0V(500ms)- 0,5V(100ms)-0V(100ms). As a aknowlege the motorcontrol send  immediately  the message B with the updated value" */
#define DIGITAL_SPINDLE_CRC_POS     ( DIGITAL_SPINDLE_BRUSH_POS  + DIGITAL_SPINDLE_MSG_BRUSH_SIZE  )    /* CRC 16 */

#define FIR_COEFF_SPINDLE 20

//#define DIGITAL_SPINDLE_PRINT_RAW //uncomment to print real-time digital spindle data


typedef struct {
    uint16_t    serial_number;              /* Serial Number: 000003 */
    uint8_t     production_year;            /* Production year */
    uint8_t     production_week;            /* Production week */
    uint8_t     firmware_version;           /* Firmware version  */
    uint16_t    load;                       /* Spindle loading	e.g. 11 A --> 12100 qdA */
    int8_t      temperature;                /* Temperature	"physically measured temperature of the cooling air of the milling motor. The resolution of the measurement is 4 Kelvin" */
    uint16_t    rpm;                        /* Actual RPM */
    uint8_t     remaining_kill_time_s;      /* Remaining kill-time	Time period before the motor is switched off.  Since the load on the motor is not absolutely constant over time, the countdown will  decremented with a  non constant step size, e. g.  160, 159, 158, 154,148,147,148,149,153 */
    uint32_t    total_run_time_s;           /* Total unit run time	"For the operating hours every byte are to be evaluated in binary with powers of 256. The low byte has the multiplier 1, the high byte 256, the high-high byte 256*256 " */
    uint32_t    brush_run_time_s;           /* Brush run time	"resettable throw  an  bit pattern to analog input e.g.  0,5V(100ms)-0V(100ms) - 0,5V(500ms)-0V(500ms)- 0,5V(100ms)-0V(100ms). As a aknowlege the motorcontrol send  immediately  the message B with the updated value" */
} spindle_digital_params;



// Initializes spindle pins and hardware PWM, if enabled.
void spindle_init();

// Returns current spindle output state. Overrides may alter it from programmed states.
uint8_t spindle_get_state();

// Called by g-code parser when setting spindle state and requires a buffer sync.
// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by spindle_sync() after sync and parking motion/spindle stop override during restore.

// Called by g-code parser when setting spindle state and requires a buffer sync.
void spindle_sync(uint8_t state, float rpm);

// Sets spindle running state with direction, enable, and spindle PWM.
void spindle_set_state(uint8_t state, float rpm); 

// Sets spindle PWM quickly for stepper ISR. Also called by spindle_set_state().
// NOTE: Mega2560 PWM register is 16-bit.
void spindle_set_speed(uint16_t pwm_value);

// Computes Mega2560-specific PWM register value for the given RPM for quick updating.
uint16_t spindle_compute_pwm_value(float rpm);
  
// Stop and start spindle routines. Called by all spindle routines and stepper ISR.
void spindle_stop();

void spindle_nudge_pwm(float rpm_delta_update); /* spindle signal feedback loop update */

void spindle_read_digital(void); /* attemt to decode serial buffer received from digital Mafell spindle */
void spindle_digital_print_info(void);
void spindle_digital_print_real_time(void);
void spindle_digital_print_rpm(void);
uint8_t get_spindle_AC_state(void);

#endif
