/*
 * ASMCNC.h
 *
 *  Created on: 5 Feb 2018
 *      Author: ianda
 */

#ifndef ASMCNC_h
#define ASMCNC_h

#define ASMCNC_VERSION			"2.0.8"
#define ASMCNC_VERSION_BUILD	"20200912"

/* proprietary error codes */
#define ASMCNC_STATUS_INVALID_STATEMENT	39 //ASM Error code 39 if 'A' is followed by unrecognised command
#define ASMCNC_INVALID_MOTOR_ID	        40 //ASM Error code 40. TMC command received for wrong motor
#define ASMCNC_CRC8_ERROR	            41 //ASM Error code 41. TMC command received but crc8 does not match
#define ASMCNC_INVALID_HEX_CODE         42 //ASM Error code 42. Non "hex code" character received
#define ASMCNC_COMMAND_ERROR            43 //ASM Error code 43. Command supplied to the function is outside wanted range
#define ASMCNC_PARAM_ERROR              44 //ASM Error code 44. Parameter supplied to the function is outside wanted range


// Z-head PCB has two options for spindle control:
// 1) FET and resistive divider based filter (non-linear and power hungry)
// 2) OpAmp based filter (linear)
// Default assembly version is option 2 (OpAmp based) uncomment INVERT_SPINDLE_PWM if option 1 is used
//#define INVERT_SPINDLE_PWM

// LIMITS defines
// Port direction pins
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
#define THERMISTOR_MONITOR   3    /* MEGA2560 Analog Pin PF3, 2k NTC thermistor monitor*/
//Spindle spare pin
#define SPINDLE_SPARE 5           /* MEGA2560 Analog Pin PF5, for future use, for example low when brushes are ok, high when brushes are worn */
#define SPINDLE_SPARE_MASK		(1<<SPINDLE_SPARE)

#define ENABLE_SPINDLE_LOAD_MONITOR // enable spindle load monitoring, apply to Mafell spindles
#define ENABLE_TEMPERATURE_MONITOR  // enable temperatuer monitoring, apply to ZH2 and newer

#define ENABLE_LASER_POINTER_CONTROL // Laser cross unit control
#define ENABLE_TMC_FEEDBACK_MONITOR  // print feedback from TMC motor controllers

/* RGB HEX Rx state machine state */
enum rgbHexStates{
	RTL_IDLE, 			// normal state, usual operation
	RTL_RGB_HEX_RX,   	// hex code reception ongoing
	RTL_RGB_HEX_ERR,   	// FAULT - other than "0123456789ABCDEF" char received
	RTL_TMC_RX,   		// TMC command code reception ongoing
};

#define RTL_TMC_COMMAND_SIZE 7 /* 7 bytes: len, command, value, crc */
#define RTL_RGB_COMMAND_SIZE 6 /* 6 hex bytes: 2xR, 2xG, 2xB */


/* setup TMC port */
#define TMC_DDR			DDRB
#define TMC_PORT		PORTB
// Port bits
#define SPI_SS_PIN			0 //PB0
#define SPI_SCK_PIN			1 //PB1
#define SPI_MOSI_PIN		2 //PB2
#define SPI_MISO_PIN		3 //PB3 //this need to be input pin
#define SPI_CS_X_PIN		4 //PB4
#define SPI_CS_Y_PIN		5 //PB5
#define SPI_CS_Z_PIN		6 //PB6
#define TMC_PORT_MASK	( (1<<SPI_SCK_PIN) | (1<<SPI_MOSI_PIN) | (1<<SPI_CS_X_PIN) | (1<<SPI_CS_Y_PIN) | (1<<SPI_CS_Z_PIN) | (1<<SPI_SS_PIN) );

/* setup debug port. Designed for monitoring real time performance of individual functions to identify potential weaknesses and clashes in the code*/
//#define DEBUG_PINS_ENABLED // comment out to remove debug pins functionality - remove for production version
//#define DEBUG_STEPPER_ENABLED // comment out to remove debug pins functionality - remove for production version
//#define DEBUG_ADC_ENABLED // comment out to remove ADC debug pins functionality - remove for production version
//#define MSTEP_READING_ENABLED // good to have a temporal view of MSTEP for debug purposes
//#define SG_SKIP_DEBUG_ENABLED // enable to debug stall guard masking engine
//#define SG_CAL_DEBUG_ENABLED // enable to debug stall guard calibration engine
//#define FLASH_DEBUG_ENABLED // enable to debug EEPROM storage

#if defined(DEBUG_PINS_ENABLED) || defined(DEBUG_ADC_ENABLED) || defined(DEBUG_STEPPER_ENABLED) || defined(SG_SKIP_DEBUG_ENABLED) || defined(SG_CAL_DEBUG_ENABLED) || defined(FLASH_DEBUG_ENABLED)
#define ANY_DEBUG_PINS_ENABLED // 
#endif

#ifdef ANY_DEBUG_PINS_ENABLED 
#define DEBUG_DDR			DDRK
#define DEBUG_PORT			PORTK
// Port bits
#define DEBUG_0_PIN			0 //PK0 / RXD2
#define DEBUG_1_PIN			1 //PK1
#define DEBUG_2_PIN			5 //PK5
#define DEBUG_PORT_MASK	( (1<<DEBUG_0_PIN) | (1<<DEBUG_1_PIN) | (1<<DEBUG_2_PIN) );
void debug_pin_write(uint32_t level, uint32_t pin);
#endif

/* ADC state machine states */
enum adc_states{
	ADC_IDLE, // normal state, ADC is off
	ADC_CH1,  // ADC is running conversion on Channel 1
	ADC_CH2,  // ADC is running conversion on Channel 2	
};


#define CURRENT_FLASH_STAT_VER			20090614 //0x18100600 
#define UPTIME_FIFO_SIZE_BYTES          32

// Flash statistics
typedef struct {
    /* reset source from MCUSR – MCU Status Register */
    uint32_t TOT_cnt;                                               /* total count of resets */
    uint32_t JTRF_cnt;                                              /* count of resets due to JTAG Reset Flag This bit is set if a reset is being caused by a logic one in the JTAG Reset Register selected by the JTAG instruction AVR_RESET. This bit is reset by a Power-on Reset, or by writing a logic zero to the flag. */
    uint32_t WDRF_cnt;                                              /* count of resets due to Watchdog Reset Flag. This bit is set if a Watchdog Reset occurs. The bit is reset by a Power-on Reset, or by writing a logic zero to the flag. */
    uint32_t BORF_cnt;                                              /* count of resets due to Brown-out Reset Flag. This bit is set if a Brown-out Reset occurs. The bit is reset by a Power-on Reset, or by writing a logic zero to the flag. */
    uint32_t EXTRF_cnt;                                             /* count of resets due to External Reset Flag. This bit is set if an External Reset occurs. The bit is reset by a Power-on Reset, or by writing a logic zero to the flag. */
    uint32_t PORF_cnt;                                              /* count of resets due to Power-on Reset Flag. This bit is set if a Power-on Reset occurs. The bit is reset only by writing a logic zero to the flag. */
    uint32_t flashStatisticsVersion;                                /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
    uint32_t totalRunTimeSeconds;                                   /* total ON time in seconds. (using spi_interrupt) */
    uint8_t  RunTimeMinutesFIFO[UPTIME_FIFO_SIZE_BYTES];            /* Flash lifetime is not affected when 0 is written, only depend on erase cycles. this array would allow to write 256 zeros and only then erase to loop the counter. If do it every minute eeprom life will be reached in 40 years. */
    uint32_t totalTravelMillimeters;                                /* total travelled distance in mm. (using mm_var) */
    uint32_t totalStallsDetected;                                   /* total number of stalls detected */
    /* fifo with last exception addresses causing WD to trigger*/
    uint32_t lastReturnAddresses[4];                                /* return address from the latest stack dump */
    /* fifo with statistic on stall: total distance, feed, */
    uint32_t lastStallsTravel[4];                                   /* fifo buffer for when last stalls were happening */
    uint8_t  lastStallsMotor[4];                                    /* fifo buffer for which motor stalled */
    uint16_t lastStallsSG[4];                                       /* fifo buffer for Stall Guard reading at last stalls */
    uint16_t lastStallsSGdelta[4];                                  /* fifo buffer for SG delta to calibration at last stalls */
    uint16_t lastStallsFeed[4];                                     /* fifo buffer for feed rates at last stalls */
} FlashStat;
extern FlashStat flashStatistics;
extern float totalTravelMillimeters;                                /* accumulator for accurate tracking of the distance */




void asmcnc_init(void);
//void asmcnc_TMR3_init();
void asmcnc_RGB_off(void);
void asmcnc_RGB_white(void);
void asmcnc_RGB_red_flash(void);
void asmcnc_RGB_setup(void);
uint8_t asmcnc_execute_line(char *line);

uint8_t crc8x_fast(uint8_t crc, uint8_t *mem, uint16_t len); /* fast crc8 calculator */

void enable_watchdog(void);
void dumpMemory(void);
uint32_t get_return_addr(void); /* return address from the latest stack dump */

#define UNUSED_VARIABLE(X)  ((void)(X))
#define UNUSED_PARAMETER(X) UNUSED_VARIABLE(X)

void asmcnc_init_ADC(void);         /* initialise ADC for spindle load monitoring */
void asmcnc_start_ADC(void);        /* start ADC state machine from channel 1 */
uint8_t char2intValidate(char);     /* convert hex char to int and validate result (return 0xFF if character is not hex byte code */
int get_temperature (void);
int get_spindle_load_volts(void);

void uptime_increment(void);

void report_statistics(void);

#endif /* ASMCNC_h */
