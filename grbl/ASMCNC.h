/*
 * ASMCNC.h
 *
 *  Created on: 5 Feb 2018
 *      Author: ianda
 */

#ifndef ASMCNC_h
#define ASMCNC_h

#define ASMCNC_VERSION          "2.2.3"
#define ASMCNC_VERSION_BUILD    "20210606"

/* proprietary error codes */
#define ASMCNC_STATUS_INVALID_STATEMENT 39 //ASM Error code 39 if 'A' is followed by unrecognised command
#define ASMCNC_INVALID_MOTOR_ID         40 //ASM Error code 40. TMC command received for wrong motor
#define ASMCNC_CRC8_ERROR               41 //ASM Error code 41. TMC command received but crc8 does not match
#define ASMCNC_INVALID_HEX_CODE         42 //ASM Error code 42. Non "hex code" character received
#define ASMCNC_COMMAND_ERROR            43 //ASM Error code 43. Command supplied to the function is outside wanted range
#define ASMCNC_PARAM_ERROR              44 //ASM Error code 44. Parameter supplied to the function is outside wanted range
#define ASMCNC_RTL_PARSE_ERROR          45 //ASM Error code 45. RTL buffer parser did not find expected packet start modifier.
#define ASMCNC_RTL_LEN_ERROR            46 //ASM Error code 46. RTL buffer parser found length that is higher than maximum.
#define ASMCNC_RTL_SEQ_ERROR            47 //ASM Error code 47. Sequence number does not match the expected one.
#define ASMCNC_RTL_BUFFER_FULL          48 //ASM Error code 48. RTL buffer overflow, slow down sending RTL commands or increase buffer size.

#define BK_INITIATOR "^"
#define BK_TERMINATOR "v\n"

//#define SPI_TIMER_CYCLE_PER_READ 0x26   /* 2.496ms with prescaler 1024*/
//#define SPI_TIMER_CYCLE_PER_READ 0xF    /* 1.024ms with prescaler 1024*/
//#define SPI_TIMER_CYCLE_PER_READ 0x5D   /* 6.016ms with prescaler 1024*/
#define SPI_TIMER_CYCLE_PER_READ 0xFF   /* 16.384ms with prescaler 1024*/
#define SPI_READ_OCR_PERIOD_US ((1+SPI_TIMER_CYCLE_PER_READ)<<6) /* SPI timer period, typically 16384us with prescaler 1024*/
#define SPI_READ_ALL_PERIOD_MS 1500     /* how often SPI engine should read all values from each controller, typically 1s */
#define UPTIME_TICK_PERIOD_MS 1000      /* how often SPI engine should signal to main loop to increment uptime */


// Z-head PCB has two options for spindle control:
// 1) FET and resistive divider based filter (non-linear and power hungry)
// 2) OpAmp based filter (linear)
// Default assembly version is option 2 (OpAmp based) uncomment INVERT_SPINDLE_PWM if option 1 is used
//#define INVERT_SPINDLE_PWM

// LIMITS defines
// Port direction pins
//#define AC_YLIM_XLIM_DDRL     DDRL
//#define AC_DOOR_DDR           DDRL
// Port bits
//#define AC_YLIM_MIN_RED       5
//#define AC_YLIM_MAX_RED       1
//#define AC_XLIM_MAX_RED       2
//#define AC_XLIM_MIN_RED       4
//#define AC_ZLIM_MAX_RED       0
//#define AC_DOOR_RED           6
// Used to add to AXIS number for reporting the correct limit switch
#define X_AXIS_MAX      4
#define Y_AXIS_SG       5
#define X_AXIS_SG       6
#define Z_AXIS_SG       7
// TODO: Move all LED's to Port L to simplify the code
//#define AC_LIM_RED_MASK_Y ((1<<AC_YLIM_MIN_RED)|(1<<AC_YLIM_MAX_RED))
//#define AC_LIM_RED_MASK_XZ    ((1<<AC_XLIM_MIN_RED)|(1<<AC_XLIM_MAX_RED)|(1<<AC_ZLIM_MAX_RED)|(1<<AC_YLIM_MIN_RED)|(1<<AC_YLIM_MAX_RED))
//#define AC_DOOR_RED_MASK  (1<<AC_DOOR_RED)

// RGB defines
// Port direction pins
#define AC_RGB_DDR          DDRE
// Port bits
#define AC_RGB_R        3 //Red LED
#define AC_RGB_G        4 //Green LED
#define AC_RGB_B        5 //Blue LED
#define LASER_PIN       6 //Laser cross on/off control, pin 8, PE6, line 6 on port E

#define AC_RGB_MASK     ((1<<AC_RGB_R)|(1<<AC_RGB_G)|(1<<AC_RGB_B))

//Extractor & Light defines
#define AC_ACCS_DDR         DDRG
#define AC_EXTRACTOR    0
#define AC_LIGHT        2

#define AC_ACCS_MASK    ((1<<AC_EXTRACTOR)|(1<<AC_LIGHT))

//PL3 Probe holder
//Enclosure (dust shoe cover)
#define AC_PROBE_ENCLOSURE_DDR  DDRK
#define AC_PROBE_ENCLOSURE  2
#define AC_PROBE_ENCLOSURE_MASK (1<<AC_PROBE_ENCLOSURE)

//Live loss pin
#define AC_LIVE_SENSE 1           /* MEGA2560 Analog Pin 1 on port F, low when live is present, high when live is lost, only for Z-head HW >= Rev H */
#define AC_LIVE_SENSE_MASK      (1<<AC_LIVE_SENSE)

#define AC_LIVE_DDR       DDRL
#define AC_LIVE_PIN       PINL
#define AC_LIVE_PORT      PORTL
#define AC_LIVE_BIT       1 //pin PL1 (ICP5), pin 36
#define AC_LIVE_INT_vect  TIMER5_CAPT_vect
#define AC_LIVE_TIMSK     TIMSK5 // Pin change interrupt register
#define AC_LIVE_MASK      (1<<AC_LIVE_BIT)


//Spindle load monitor pin
#define SPINDLE_LOAD_MONITOR 1    /* MEGA2560 Analog Pin PF1, spindle load 0-5V signal monitor*/
#define THERMISTOR_MONITOR   3    /* MEGA2560 Analog Pin PF3, 2k NTC thermistor monitor*/

#define ENABLE_SPINDLE_LOAD_MONITOR // enable spindle load monitoring, apply to Mafell spindles
#define ENABLE_TEMPERATURE_MONITOR  // enable temperature monitoring, apply to ZH2 and newer

#define ENABLE_LASER_POINTER_CONTROL // Laser cross unit control

/* RGB HEX Rx state machine state */
enum rgbHexStates{
    RTL_IDLE,           // normal state, usual operation
    RTL_RGB_HEX_RX,     // hex code reception ongoing
    RTL_RGB_HEX_ERR,    // FAULT - other than "0123456789ABCDEF" char received
    RTL_V2_RX,          // TMC command code reception ongoing
};

 /* protocol v2 Commands, refer to gdoc "Yeti-GRBL extended Protocol"*/
#define SET_RGB_LED_STATE                   1            //  Set the dust show light color in RGB 3 bytes format, Green would be “^\x01\x00\xFF\x00“
#define SET_SPINDLE_SPEED                   2            //  Set spindle speed. Speed 0 would also turn off the spindle relay
#define SET_EXTRACTION_STATE                3            //  Enable or disable extraction. 1: enable, 0: disable.
#define SET_LASER_DATUM_STATE               4            //  Enable or disable laser datum. 1: enable, 0: disable.
#define SET_SERIAL_NUMBER                   5            //  Store serial number to persistent memory
#define SET_PRODUCT_VERSION                 6            //  Store product version to persistent memory
#define GET_SERIAL_NUMBER                   7            //  Report serial number stored in persistent memory
#define GET_PRODUCT_VERSION                 8            //  Report product number stored in persistent memory
#define GET_ALARM_REASON                    9            //  Report latest alarm reason (which end switch triggered the alarm)
#define GET_DIGITAL_SPINDLE_INFO            10           //  Report Mafell digital spindle info: serial number, uptime, brush time, etc.
#define RESET_DIGITAL_SPINDLE_BRUSH_TIME    11           //  Reset brush timer in Mafell digital spindle
#define RESET_SEQUENCE_NUMBER               12           //  Reset protocol V2 sequence number to 0. Command would not generate sequence error even if expected sequence number does not match
#define TMC_COMMAND                         50           //  TMC command, see table xx


#define RTL_V2_COMMAND_SIZE_MIN  4  /* 5 bytes: len, seq, command, crc */
#define RTL_V2_COMMAND_SIZE_MAX 20  /* 20 bytes: len, seq, command, data (0-16), crc */
#define RTL_RGB_COMMAND_SIZE     6  /* 6 hex bytes: 2xR, 2xG, 2xB */
#define SERIAL_NUMBER_LEN       12  /* length of serial number field */
#define PRODUCT_VERSION_LEN      8  /* length of product number field */
#define TMC_REG_CMD_LENGTH       4  /* value */
#define TMC_GBL_CMD_LENGTH       1  /* 1 byte command */

/* setup TMC port */
#define TMC_DDR         DDRB
#define TMC_PORT        PORTB
// Port bits
#define TMC_X_ST_ALONE      0 //PB0
#define SPI_SCK_PIN         1 //PB1
#define SPI_MOSI_PIN        2 //PB2
#define SPI_MISO_PIN        3 //PB3 //this need to be input pin
#define SPI_CS_X_PIN        4 //PB4
#define SPI_CS_Y_PIN        5 //PB5
#define SPI_CS_Z_PIN        6 //PB6
#define TMC_Z_ST_ALONE      7 //PB7
#define TMC_PORT_MASK   ( (1<<TMC_X_ST_ALONE) | (1<<SPI_SCK_PIN) | (1<<SPI_MOSI_PIN) | (1<<SPI_CS_X_PIN) | (1<<SPI_CS_Y_PIN) | (1<<SPI_CS_Z_PIN) | (1<<TMC_Z_ST_ALONE) )

/* setup debug port. Designed for monitoring real time performance of individual functions to identify potential weaknesses and clashes in the code*/
//#define DEBUG_SPI_ENABLED // comment out to remove debug pins functionality - remove for production version
//#define DEBUG_STEPPER_ENABLED // comment out to remove debug pins functionality - remove for production version
//#define DEBUG_ADC_ENABLED // comment out to remove ADC debug pins functionality - remove for production version
//#define MSTEP_READING_ENABLED // good to have a temporal view of MSTEP for debug purposes
//#define SG_SKIP_DEBUG_ENABLED // enable to debug stall guard masking engine
//#define SG_CAL_DEBUG_ENABLED // enable to debug stall guard calibration engine
//#define FLASH_DEBUG_ENABLED // enable to debug EEPROM storage
//#define DEBUG_LED_ENABLED // enable to drive indication (second) RGB LED light
//#define DEBUG_CPU_LOAD_ENABLED // enable to drive indication (second) RGB LED light
#define DEBUG_SPINDLE_ENABLED // comment out to remove ADC debug pins functionality - remove for production version

#if defined(DEBUG_SPI_ENABLED) || defined(DEBUG_ADC_ENABLED) || defined(DEBUG_STEPPER_ENABLED) || defined(SG_SKIP_DEBUG_ENABLED) || defined(SG_CAL_DEBUG_ENABLED) || defined(FLASH_DEBUG_ENABLED) || defined(DEBUG_LED_ENABLED) || defined(DEBUG_CPU_LOAD_ENABLED) || defined(DEBUG_SPINDLE_ENABLED)
#define ANY_DEBUG_ENABLED //
#endif

#ifdef ANY_DEBUG_ENABLED
#define DEBUG_DDR           DDRC
#define DEBUG_PORT          PORTC
// Port bits
#define DEBUG_0_PIN         0 //PC0, Blue debug LED
#define DEBUG_1_PIN         1 //PC1, Red debug LED
#define DEBUG_2_PIN         2 //PC2
#define DEBUG_3_PIN         3 //PC3, Green debug LED
#define DEBUG_PORT_MASK ( (1<<DEBUG_0_PIN) | (1<<DEBUG_1_PIN) | (1<<DEBUG_2_PIN) | (1<<DEBUG_3_PIN) )
void debug_pin_write(uint8_t level, uint8_t pin);
#endif



void asmcnc_init(void);
//void asmcnc_TMR3_init();
void asmcnc_RGB_off(void);
void asmcnc_RGB_white(void);
void asmcnc_RGB_red_flash(void);
void asmcnc_RGB_init(void);
void asmcnc_RGB_set(uint8_t R, uint8_t G, uint8_t B);

uint8_t asmcnc_execute_line(char *line);

void enable_watchdog(void);
#define UNUSED_VARIABLE(X)  ((void)(X))
#define UNUSED_PARAMETER(X) UNUSED_VARIABLE(X)

uint8_t char2intValidate(char);     /* convert hex char to int and validate result (return 0xFF if character is not hex byte code */


void asmcnc_enable_AC_live_detection(void);
uint8_t get_AC_lost_state(void);

uint8_t  uint16_encode(uint16_t value, uint8_t * p_encoded_data);
uint16_t uint16_decode(const uint8_t * p_encoded_data);
uint8_t  uint32_encode(uint32_t value, uint8_t * p_encoded_data);
uint32_t uint32_decode(const uint8_t * p_encoded_data);
uint32_t uint24_decode(const uint8_t * p_encoded_data);

/* status string headers definitions for interfacing between GRBL and Console */
#define STATUS_FS_IDENTIFIER    "|FS:"        // feed and speed block
#define STATUS_PN_IDENTIFIER    "|Pn:"        // end switches and other switches state
#define STATUS_BF_IDENTIFIER    "|Bf:"        // UART buffer bytes and blocks status
#define STATUS_LD_IDENTIFIER    "|Ld:"        // spindle load block
#define STATUS_SP_IDENTIFIER    "|Sp:"        // Mafell digital spindle statistics
#define STATUS_TC_IDENTIFIER    "|TC:"        // Temperatures block
#define STATUS_TM_IDENTIFIER    "|TM:"        // full TMC statistics report
#define STATUS_SG_IDENTIFIER    "|SG:"        // Stall guard block
#define STATUS_VOL_IDENTIFIER   "|V:"         // Voltages block
#define STATUS_TCAL_IDENTIFIER  "|TCAL:"      // Calibration coefficients block
#define STATUS_TREG_IDENTIFIER  "|TREG:"      // TMC registers and parameters state: DRVCTRL, CHOPCONF, SMARTEN, SGCSCONF, DRVCONF, activeCurrentScale, standStillCurrentScale, stallGuardAlarmThreshold, step_period_us_to_read_SG, gradient_per_Celsius
#define STATUS_STAT_IDENTIFIER  "|STAT:"      // GRBL runtime statistics
#define STATUS_SGAL_IDENTIFIER  "|SGALARM:"   // Stall guard stop statistics
#define STATUS_ALRM_IDENTIFIER  "|Pa:"        // last Alarm reason

#endif /* ASMCNC_h */

