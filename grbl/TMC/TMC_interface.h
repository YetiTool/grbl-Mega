/*
 * TMC_interface.h
 *
 * Created: 28/08/2020 23:00:55
 *  Author: bk
 */


#ifndef TMC_INTERFACE_H_
#define TMC_INTERFACE_H_

#include "API_Header.h"
#include "TMC2590_Constants.h"
#include "TMC2590_Macros.h"
#include "TMC2590_Register.h"

/* Controller types. */
typedef enum
{
    TMC_X1,
    TMC_X2,
    TMC_Y1,
    TMC_Y2,
    TMC_Z,
    TOTAL_TMCS
} tmc_controller_enum_type_t;

/* Current scale. */
typedef enum
{
    CURRENT_SCALE_STANDSTILL,
    CURRENT_SCALE_ACTIVE
} tmc_current_scale_enum_type_t;


/* TMC controller mode. Defines homing behaviour */
typedef enum
{
    TMC_MODE_IDLE,
    TMC_MODE_HOMING
} tmc_homing_mode_enum_type_t;


//#define RIGGY
#define TMC_SG_PROFILE_POINTS               128
#define SG_READ_STEP_COUNT                  15 // TMC chip reports SG every 16 pulses (1 full step) or every 64 steps (4 full steps) if filtering is enabled. UART reads could halt the readings up to 6ms, so many unfiltered samples could be missed at high speed. So for now read filtered SG twice per change.
#define SG_READING_SKIPS_AFTER_SLOW_FEED    15        /* Slow or 0 feed causes invalid SG reading for several cycles even after the nominal speed was reached. Skip this many readins after feed exceeds nominal (period gets less than max_step_period_us_to_read_SG) for this axis. Actually means 5 reads for dual axis and 10 for single */
#define DEFAULT_TMC_READ_SELECT             1 /* read of the SG is default state of the system */

#define REPORT_TMC_REFRESH_COUNT            10 /* how often print full TMC statistics on UART */

/* max valid periods are limited by acceleration: Z motor with acc=200 starts with 930us pulses and ends with 1600us pulses. X and Y motors with acc=130 starts with 9530us and ends with 12500us */
#ifdef RIGGY
#define SG_MAX_VALID_PERIOD_X_US            8000    /* 2.3rpm (132mm/min feed). for riggy: X motor 17HS15-0404S - 100 rpm */
#define SG_MAX_VALID_PERIOD_Y_US            8000    /* 2.3rpm (132mm/min feed). Slow or 0 feed causes invalid SG reading. This parameter specifies max SG read period that resiult in vaild reading. Anything above it (slower speed) will result in invalid reading. */
#define SG_MAX_VALID_PERIOD_Z_US            4000    /* 12.5rpm (37.5mm/min feed). Z motor 17HS19-2004S1*/
#else
#define SG_MAX_VALID_PERIOD_X_US            2060    /* 9.1rpm (520mm/min feed). Slow or 0 feed causes invalid SG reading. This parameter specifies max SG read period that resiult in vaild reading. Anything above it (slower speed) will result in invalid reading. */
#define SG_MAX_VALID_PERIOD_Y_US            2060    /* 9.1rpm (520mm/min feed). Slow or 0 feed causes invalid SG reading. This parameter specifies max SG read period that resiult in vaild reading. Anything above it (slower speed) will result in invalid reading. */
#define SG_MAX_VALID_PERIOD_Z_US            400     /* 50rpm  (150mm/min feed). Slow or 0 feed causes invalid SG reading. This parameter specifies max SG read period that resiult in vaild reading. Anything above it (slower speed) will result in invalid reading. */
//#define SG_MAX_VALID_PERIOD_X_US            1060    /* 20rpm (1120mm/min feed). Slow or 0 feed causes invalid SG reading. This parameter specifies max SG read period that resiult in vaild reading. Anything above it (slower speed) will result in invalid reading. */
//#define SG_MAX_VALID_PERIOD_Y_US            1060    /* 20rpm (1120mm/min feed). Slow or 0 feed causes invalid SG reading. This parameter specifies max SG read period that resiult in vaild reading. Anything above it (slower speed) will result in invalid reading. */
//#define SG_MAX_VALID_PERIOD_Z_US            800     /* 25rpm  (75mm/min feed). Slow or 0 feed causes invalid SG reading. This parameter specifies max SG read period that resiult in vaild reading. Anything above it (slower speed) will result in invalid reading. */
//#define SG_MAX_VALID_PERIOD_X_US            8000    /* 2.3rpm (132mm/min feed). for riggy: X motor 17HS15-0404S - 100 rpm */
//#define SG_MAX_VALID_PERIOD_Y_US            8000    /* 2.3rpm (132mm/min feed). Slow or 0 feed causes invalid SG reading. This parameter specifies max SG read period that resiult in vaild reading. Anything above it (slower speed) will result in invalid reading. */
//#define SG_MAX_VALID_PERIOD_Z_US            6000     /* 25rpm  (75mm/min feed). Slow or 0 feed causes invalid SG reading. This parameter specifies max SG read period that resiult in vaild reading. Anything above it (slower speed) will result in invalid reading. */
#endif

/* max step period for calibration purposes */
#define SG_MAX_CALIBR_PERIOD_X_US           8000    /* 2.3rpm (132mm/min feed). for riggy: X motor 17HS15-0404S - 100 rpm */
#define SG_MAX_CALIBR_PERIOD_Y_US           8000    /* 2.3rpm (132mm/min feed). Slow or 0 feed causes invalid SG reading. This parameter specifies max SG read period that resiult in vaild reading. Anything above it (slower speed) will result in invalid reading. */
#define SG_MAX_CALIBR_PERIOD_Z_US           6250    /* 3rpm (9 mm/min feed). Z motor 17HS19-2004S1*/


// Stepper ISR data struct. Contains the running data for the main stepper ISR.
typedef struct {
    uint16_t step_period[N_AXIS];           // variables to hold the step period which is direct reflection of shaft rotational speed at the time when SG read was fired.
    uint8_t  step_period_idx[N_AXIS];       // variables to hold the step period index which is direct mapping of step period, used to maximise computational speed.
    uint8_t  step_period_idx_past[N_AXIS];       // variables to hold the step period index which is direct mapping of step period, used to maximise computational speed.
    uint8_t  this_reading_direction[N_AXIS];// variables to hold the direction applied at current SG reading, used to reset SG reading skip counter under direction change
    uint8_t  last_reading_direction[N_AXIS];// variables to hold the direction applied at last SG reading, used to reset SG reading skip counter under direction change
    uint8_t  step_counter[N_AXIS];          // Counter variables for firing SG read. TMC chip reports SG every 16 pulses (1 full step) or every 64 steps (4 full steps) if filtering is enabled
    uint8_t  SG_skips_counter[N_AXIS];      // Counter variables for blocking stall analysis due to preceding slow speed. Slow or 0 feed causes invalid SG reading for several cycles even after the nominal speed was reached. Skip this many readins after feed exceeds nominal (period gets less than max_step_period_us_to_read_SG) for this axis */
    uint8_t  current_scale_state;           // global holding effective current scale
    uint8_t  sg_read_active_axes;           // global variable to hold list of axes that is being read
    uint8_t  stall_alarm_enabled;           // global holding desired stall behaviour: if "true" then stall guard value below the limit will trigger alarm
    uint8_t  calibration_enabled;           // SG calibration ongoing
    uint8_t  calibration_axis;              // SG calibration axis    
} stepper_tmc_t;


typedef struct {
    uint16_t mStepCurrentValue;
    uint8_t coolStepCurrentValue;
    uint8_t stallGuardShortValue;
    uint16_t stallGuardCurrentValue;
    uint16_t StatusBits;
    uint16_t DiagnosticBits;
} TMC2590Response;


// Usage note: use 1 TypeDef per IC
typedef struct {

    /* TMC config parameters */
    /* Parameters stored in EEPROM */
    int32_t shadowRegister[TMC2590_REGISTER_COUNT]; /* latest state of each config register */
    uint16_t stallGuardAlarmThreshold;              /* when current SG reading is lower than calibrated by this value corresponded axis alarm will be triggered */
    uint16_t gradient_per_Celsius;                  /* correction for temperatures other than calibration */
    uint8_t standStillCurrentScale;                 /* standstill current - to reduce energy consumption while job is idle */
    uint8_t activeCurrentScale;                     /* active current 0 - 31 where 31 is max */

    /* running variables */
    uint8_t channel;                                /* pio index defining SPI channel */
    uint8_t thisMotor;                              /* this motor index */
    uint8_t thisAxis;                               /* this motor Axis */
    uint16_t stallGuardAlarmValue;                  /* when current SG reading is lower than this value corresponded axis alarm will be triggered */
    int16_t stallGuardDelta;                        /* difference between current SG reading and calibrated curve */
    int16_t stallGuardDeltaCurrent;                 /* difference between current SG reading and calibrated curve */
    int16_t stallGuardDeltaAxis;                    /* Average delta for axis */
    int16_t stallGuardDeltaPast;                 /* difference between current SG reading and calibrated curve */
    int16_t stallGuardDeltaAxisPast;                    /* Average delta for axis */
    uint8_t respIdx;                                /* current rdsel to know which response is coming next */
    int32_t response[TMC2590_RESPONSE3+1];          /* raw response from controllers */
    TMC2590Response resp;                           /* decoded response from controllers */

    uint8_t SlowDecayDuration;                      /* Off time/MOSFET disable. Duration of slow decay phase. If TOFF is 0, the MOSFETs are shut off. If TOFF is nonzero, slow decay time is a multiple of system clock periods: NCLK= 24 + (32 x TOFF) (Minimum time is 64clocks.), %0000: Driver disable, all bridges off, %0001: 1 (use with TBL of minimum 24 clocks) %0010 ... %1111: 2 ... 15 */

} TMC2590TypeDef;



// Flash configuration settings for Trinamics drivers. When adding new items always add them at the end and formulate
// them such that a value of zero is an appropriate default or backwards compatible. Existing
// modules that are upgraded will have zero in the new fields. This ensures that an upgrade does
// not wipe out the old settings.
typedef struct {
    uint32_t flashTMCconfigVersion;                                 /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
    int32_t  registerState[TOTAL_TMCS][TMC2590_REGISTER_COUNT];     /* TMC registers for each of 5 controllers: DRVCTRL, CHOPCONF, SMARTEN, SGCSCONF, DRVCONF. 160 bytes */
    uint16_t gradient_per_Celsius[TOTAL_TMCS];                      /* coefficient defining thermal offset applied to calibration curve */
    uint16_t stallGuardAlarmThreshold[TOTAL_TMCS];                  /* when current SG reading is lower than calibrated by this value corresponded axis alarm will be triggered */
    uint8_t standStillCurrentScale[TOTAL_TMCS];                     /* standstill current - to reduce energy consumption while job is idle */
    uint8_t activeCurrentScale[TOTAL_TMCS];                         /* active current */
} FlashTMCconfig;


/* host commands definitions */
#define TMC_COMMAND_BIT_SIZE   4
#define MOTOR_OFFSET_MASK      0xF   /* Motor offset mask must be a mask of contiguous zeroes, followed by contiguous sequence of ones: 000...111. */
#define MOTOR_OFFSET           (MOTOR_OFFSET_MASK+1)     /* Motor offset */

// individual commands to be applied to individual motors
#define SET_DRVCTRL            TMC2590_DRVCTRL
#define SET_CHOPCONF           TMC2590_CHOPCONF
#define SET_SMARTEN            TMC2590_SMARTEN
#define SET_SGCSCONF           TMC2590_SGCSCONF
#define SET_DRVCONF            TMC2590_DRVCONF
#define SET_IDLE_CURRENT       5  // set the current scale applied when no pulses are detected on the given axis
#define SET_ACTIVE_CURRENT     6  // set the active current scale
#define SET_MOTOR_ENERGIZED    7  // energize or shut off the motor completely, for example to let user move turret easier
#define SET_SG_ALARM_TRSHLD    8  // SG alarm threshold: when current SG reading is lower than calibrated by this value corresponded axis alarm will be triggered
#define SET_THERMAL_COEFF      9  // coefficient defining thermal offset applied to calibration curve


// common commands to be applied to whole system
#define SET_SG_ALARM           100  // desired stall behaviour: if "true" then stall guard value below the limit will trigger alarm
#define SET_CALIBR_MODE        101  // 1: reset all calibrations and prepare for new one, 2: complete calibration, compute , 3: print calibration coefficients
#define GET_STATISTICS         102
#define GET_TMC_STATUS         103
#define RESTORE_TMC_DEFAULTS   104  // restore all TMC default settings from flash - safety net in case parameters are completely bollocked
#define STORE_TMC_PARAMS       105  // store existing (tuned) paraeters to the flash
#define GET_REGISTERS          106
#define WDT_TMC_TEST           107 // value = 0x10: disable WD feed; other value: report EEPROM dump
#define REPORT_STALLS          108 // report list of last stalls with associated freeze frame
#define UPLOAD_CALIBR_VALUE    109  // 1: upload calibration from host. Must be preceded by TMC_CALIBRATION_INIT_xxx 

#define SET_MRES            1   /* Microstep resolution for STEP/DIR mode. Microsteps per fullstep: %0000: 256; %0001: 128; %0010: 64; %0011: 32; %0100: 16; %0101: 8; %0110: 4; %0111: 2 (halfstep); %1000: 1 (fullstep) */
#define SET_DEDGE           2   /*  */
#define SET_INTERPOL        3   /* Enable STEP interpolation. 0: Disable STEP pulse interpolation. 1: Enable MicroPlyer STEP pulse multiplication by 16 */
#define SET_TOFF            4   /* Off time/MOSFET disable. Duration of slow decay phase. If TOFF is 0, the MOSFETs are shut off. If TOFF is nonzero, slow decay time is a multiple of system clock periods: NCLK= 24 + (32 x TOFF) (Minimum time is 64clocks.), %0000: Driver disable, all bridges off, %0001: 1 (use with TBL of minimum 24 clocks) %0010 ... %1111: 2 ... 15 */
#define SET_HSTRT           5   /*  */
#define SET_HEND            6   /*  */
#define SET_HDEC            7   /*  */
#define SET_RNDTF           8   /*  */
#define SET_CHM             9   /* Chopper mode. This mode bit affects the interpretation of the HDEC, HEND, and HSTRT parameters shown below. 0 Standard mode (SpreadCycle) */
#define SET_TBL             10   /* Blanking time. Blanking time interval, in system clock periods: %00: 16 %01: 24 %10: 36 %11: 54 */
#define SET_SEMIN           11   /* Lower CoolStep threshold/CoolStep disable. If SEMIN is 0, CoolStep is disabled. If SEMIN is nonzero and the StallGuard2 value SG falls below SEMIN x 32, the CoolStep current scaling factor is increased */
#define SET_SEUP            12   /* Current increment size. Number of current increment steps for each time that the StallGuard2 value SG is sampled below the lower threshold: %00: 1; %01: 2; %10: 4; %11: 8 */
#define SET_SEMAX           13   /* Upper CoolStep threshold as an offset from the lower threshold. If the StallGuard2 measurement value SG is sampled equal to or above (SEMIN+SEMAX+1) x 32 enough times, then the coil current scaling factor is decremented. */
#define SET_SEDN            14   /* Current decrement speed. Number of times that the StallGuard2 value must be sampled equal to or above the upper threshold for each decrement of the coil current: %00: 32; %01: 8; %10: 2; %11: 1 */
#define SET_SEIMIN          15   /* Minimum CoolStep current: 0: 1/2 CS current setting; 1: 1/4 CS current setting */
#define SET_CS              16   /* Current scale (scales digital currents A and B). Current scaling for SPI and STEP/DIR operation. 0-31: 1/32, 2/32, 3/32, ... 32/32;  This value is biased by 1 and divided by 32, so the range is 1/32 to 32/32. Example: CS=20 is 21/32 current. */
#define SET_SGT             17   /* StallGuard2 threshold value. The StallGuard2 threshold value controls the optimum measurement range for readout and stall indicator output (SG_TST). A lower value results in a higher sensitivity and less torque is required to indicate a stall. The value is a two’s complement signed integer. Range: -64 to +63 */
#define SET_SFILT           18   /* StallGuard2 filter enable. 0: Standard mode, fastest response time. 1: Filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy. */
#define SET_RDSEL           19   /*  */
#define SET_VSENSE          20   /* Sense resistor voltage-based current scaling. 0: Full-scale sense resistor voltage is 325mV. 1: Full-scale sense resistor voltage is 173mV. (Full-scale refers to a current setting of 31.) */
#define SET_SDOFF           21   /*  */
#define SET_TS2G            22   /*  */
#define SET_DISS2G          23   /*  */
#define SET_SLPL            24   /* Slope control, low side, Gate driver strength 1 to 7. 7 is maximum current for fastest slopes */
#define SET_SLPH            25   /* Slope control, high side. Gate driver strength 1 to 7. 7 is maximum current for fastest slopes */
#define SET_TST             26   /*  */



/* all motors*/
void init_TMC(void);
void tmc2590_schedule_read_all(void); /* schedule periodic read of all values */
void process_status_of_all_controllers(void);

void execute_TMC_command(uint8_t* p_data, uint8_t data_len); /* execute TMC command from rtl serial buffer */

void tmc_standstill_on(void); /* reduce the current through energized motors when idle */
void tmc_standstill_off(void); /* bump the current through energized motors back to working level when cycle starts */

void stall_guard_statistics_reset(void); /* statistics is collected for the whole period between consecutive UART polls so that lost step is not missed between. Reset the statistics on all motors */
void tmc2590_schedule_read_sg(uint8_t axis); /* read Stall Guard on axis. Stepper interrupt signals to main loop that SG_READ_STEP_COUNT steps have happened and it is time to read sg on given motor*/

void tmc_all_current_scale_apply( void );

/*homing engine functions */
void tmc_spi_queue_drain_complete(void);  /* indicate to TMC2590 loops that reading is completed (required for homing cycle) */
void tmc_homing_mode_set(uint8_t mode);  /* set and reset TMC controllers for homing cycle */
void tmc_read_sg_and_trigger_limits(void); /* schedule single read of stall guard, analyse response and set limits limits accordingly */
void tmc_homing_reset_limits(void); /* clear limit switch when pulling off */
void tmc_globals_reset(void); /* reset all tmc global variables to known init state */

/* calibration functions */
void tmc_calibration_init(uint8_t calibration_axis); /* clear calibration matrix and get ready for data collection */
void tmc_compute_and_apply_calibration(void); /* stop calibration and compute coefficients based on accumulated data */
void tmc_report_calibration(void); /* print out calibration data */
void tmc_report_status(void);
void tmc_report_SG_delta(void);


TMC2590TypeDef * get_TMC_controller(uint8_t controller); /* get pointer to required contoller */

//extern uint16_t max_step_period_us_to_read_SG[];
extern uint8_t min_step_period_idx_to_read_SG[];
extern stepper_tmc_t st_tmc; // structure to hold the shaft rotational speed at the time when SG read was fired.
extern const uint16_t SG_step_periods_us[];

void tmc_load_settings(void);
void tmc_store_settings(void);
void restore_TMC_defaults(void);
void apply_TMC_settings_from_flash(void);

void tmc_report_registers(void);

void tmc_store_stall_info(uint8_t  lastStallsMotor, uint16_t lastStallsSG, uint16_t lastStallsSGcalibrated, uint16_t lastStallsStepUs);

#endif /* TMC_INTERFACE_H_ */