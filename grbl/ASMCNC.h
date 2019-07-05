/*
 * ASMCNC.h
 *
 *  Created on: 5 Feb 2018
 *      Author: ianda
 */

#ifndef ASMCNC_h
#define ASMCNC_h


#define ASMCNC_VERSION			"1.0.5"
#define ASMCNC_VERSION_BUILD	"20190614"

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

void asmcnc_init();
//void asmcnc_TMR3_init();
void asmcnc_RGB_off();
void asmcnc_RGB_white();
void asmcnc_RGB_red();
void asmcnc_RGB_red_flash();
uint8_t asmcnc_execute_line(char *line);
#endif /* ASMCNC_h */
