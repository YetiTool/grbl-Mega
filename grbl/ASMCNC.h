/*
 * ASMCNC.h
 *
 *  Created on: 5 Feb 2018
 *      Author: ianda
 */

#ifndef ASMCNC_h
#define ASMCNC_h

#define ASMCNC_VERSION			"1.2.0"
#define ASMCNC_VERSION_BUILD	"20200804"

#define ASMCNC_STATUS_INVALID_STATEMENT	39 //ASM Error code 39 if 'A' is followed by unrecognised command

// Z-head PCB has two options for spindle control:
// 1) FET and resistive divider based filter (non-linear and power hungry)
// 2) OpAmp based filter (linear)
// Default assembly version is option 2 (OpAmp based) uncomment INVERT_SPINDLE_PWM if option 1 is used
//#define INVERT_SPINDLE_PWM

// LIMITS defines
// Port direction pins
#define AC_YLIM_XLIM_DDRB	DDRB
#define AC_YLIM_XLIM_DDRL 	DDRL
#define AC_DOOR_DDR			DDRL
// Port bits
#define AC_YLIM_MIN_RED		5
#define AC_YLIM_MAX_RED		1
#define AC_XLIM_MAX_RED		2
#define AC_XLIM_MIN_RED		4
#define AC_ZLIM_MAX_RED		0
#define AC_DOOR_RED			6
// Used to add to AXIS number for reporting the correct limit switch
#define X_AXIS_MAX		4
#define Y_AXIS_MAX		5
// TODO: Move all LED's to Port L to simplify the code
//#define AC_LIM_RED_MASK_Y	((1<<AC_YLIM_MIN_RED)|(1<<AC_YLIM_MAX_RED))
#define AC_LIM_RED_MASK_XZ	((1<<AC_XLIM_MIN_RED)|(1<<AC_XLIM_MAX_RED)|(1<<AC_ZLIM_MAX_RED)|(1<<AC_YLIM_MIN_RED)|(1<<AC_YLIM_MAX_RED))
#define AC_DOOR_RED_MASK	(1<<AC_DOOR_RED)

// RGB defines
// Port direction pins
#define AC_RGB_DDR			DDRE
// Port bits
#define AC_RGB_R		3 //Red LED
#define AC_RGB_G		4 //Green LED
#define AC_RGB_B		5 //Blue LED
#define LASER_PIN		6 //Laser cross on/off control, pin 8, PE6, line 6 on port E

#define AC_RGB_MASK		((1<<AC_RGB_R)|(1<<AC_RGB_G)|(1<<AC_RGB_B))

//Extractor & Light defines
#define AC_ACCS_DDR			DDRG
#define	AC_EXTRACTOR	0
#define AC_LIGHT		2

#define AC_ACCS_MASK	((1<<AC_EXTRACTOR)|(1<<AC_LIGHT))

//PL3 Probe holder
//Probe holder
#define AC_PROBE_HOLDER_DDR	DDRL
#define AC_PROBE_HOLDER	3
#define AC_PROBE_HOLDER_MASK	(1<<AC_PROBE_HOLDER)
//Enclosure (dust shoe cover)
#define AC_PROBE_ENCLOSURE_DDR	DDRK
#define AC_PROBE_ENCLOSURE	2
#define AC_PROBE_ENCLOSURE_MASK	(1<<AC_PROBE_ENCLOSURE)
//spare control pin
#define AC_PROBE_SPARE1_DDR	DDRK
#define AC_PROBE_SPARE1 4
#define AC_PROBE_SPARE1_MASK	(1<<AC_PROBE_SPARE1)
//Live loss pin
#define AC_LIVE_SENSE 1           /* MEGA2560 Analog Pin 1 on port F, low when live is present, high when live is lost, only for Z-head HW >= Rev H */
#define AC_LIVE_SENSE_MASK		(1<<AC_LIVE_SENSE)
//Spindle load monitor pin
#define SPINDLE_LOAD_MONITOR 1    /* MEGA2560 Analog Pin PF1, spindle load 0-5V signal monitor*/
//Spindle spare pin
#define SPINDLE_SPARE 5           /* MEGA2560 Analog Pin PF5, for future use, for example low when brushes are ok, high when brushes are worn */
#define SPINDLE_SPARE_MASK		(1<<SPINDLE_SPARE)

#define ENABLE_SPINDLE_LOAD_MONITOR // enable spindle load monitoring, apply to Mafell spindles
#define ENABLE_LASER_POINTER_CONTROL // Laser cross unit control

/* RGB HEX Rx state machine state */
enum rgbHexStates{
	RGB_HEX_RTL_IDLE, // normal state, usual operation
	RGB_HEX_RTL_RX,   // hex code reception ongoing
	RGB_HEX_RTL_ERR   // FAULT - other than "0123456789ABCDEF" char received
};


void asmcnc_init(void);
//void asmcnc_TMR3_init();
void asmcnc_RGB_off(void);
void asmcnc_RGB_white(void);
void asmcnc_RGB_red_flash(void);
void asmcnc_RGB_setup(void);
uint8_t asmcnc_execute_line(char *line);
void asmcnc_init_ADC(void); /* initialise ADC for spindle load monitoring */
uint8_t char2intValidate(char); /* convert hex char to int and validate result (return 0xFF if character is not hex byte code */

#endif /* ASMCNC_h */
