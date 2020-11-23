/*
 * ADC_utils.h
 *
 * Created: 27/09/2020 23:34:26
 *  Author: Boris
 */ 


#ifndef ADC_UTILS_H_
#define ADC_UTILS_H_


/* definitions of ADC channels and timeouts. 
 * Max period is 64sec. 
 * To disable channel set PERIOD_MS to 0xFFFF */
#define ADC_PERIOD_DISABLE                  0xFFFF
#define SPINDLE_LOAD_ADC_PERIOD_MS          20                      /* how often ADC should measure Spindle load channel */
#define VDD_5V_ATMEGA_ADC_PERIOD_MS         ADC_PERIOD_DISABLE      /* how often ADC should measure 5V Atmega channel */
#define VDD_5V_DUSTSHOE_ADC_PERIOD_MS       200                     /* how often ADC should measure 5V dustshoe channel */
#define VDD_24V_ADC_PERIOD_MS               ADC_PERIOD_DISABLE      /* how often ADC should measure 24V mains channel */
#define TEMPERATURE_TMC_ADC_PERIOD_MS       100                     /* how often ADC should measure temperature 1 channel */
#define TEMPERATURE_PCB_ADC_PERIOD_MS       ADC_PERIOD_DISABLE      /* how often ADC should measure temperature 2 channel */
#define TEMPERATURE_MOT_ADC_PERIOD_MS       ADC_PERIOD_DISABLE      /* how often ADC should measure temperature 2 channel */
#define SPINDLE_SPEED_ADC_PERIOD_MS         20                     /* how often ADC should measure Spindle speed channel */
#define AC_LOSS_ADC_PERIOD_MS               10                      /* how often ADC should measure AC loss channel */

#define SPINDLE_LOAD_ADC_CHANNEL            SPINDLE_LOAD_MONITOR // 1
#define VDD_5V_ATMEGA_ADC_CHANNEL           9
#define VDD_5V_DUSTSHOE_ADC_CHANNEL         14
#define VDD_24V_ADC_CHANNEL                 12
#define TEMPERATURE_TMC_ADC_CHANNEL         THERMISTOR_MONITOR   // 3
#define TEMPERATURE_PCB_ADC_CHANNEL         13
#define TEMPERATURE_MOT_ADC_CHANNEL         8
#define SPINDLE_SPEED_ADC_CHANNEL           SPINDLE_SPARE        //2
#define AC_LOSS_ADC_CHANNEL                 0



/* ADC state machine states / channels 
   if channels need to be removed or added to the list then also the following need to be updated in asmcnc_init_ADC():
   - ADCstMachine.channel[xxx]
   - ADCstMachine.max_count[xxx]
   - ADCstMachine.tick_count[xxx]
   as well as "definitions of ADC channels and timeouts" in this heades
*/
enum adc_channels{
	ADC_0_SPINDLE_LOAD,       // ADC is running conversion on Channel 1  SPINDLE_LOAD_ADC_CHANNEL
	ADC_1_VDD_5V_ATMEGA,      // ADC is running conversion on Channel 2  VDD_5V_ATMEGA_ADC_CHANNEL
	ADC_2_VDD_5V_DUSTSHOE,    // ADC is running conversion on Channel 3  VDD_5V_DUSTSHOE_ADC_CHANNEL
	ADC_3_VDD_24V,            // ADC is running conversion on Channel 4  VDD_24V_ADC_CHANNEL
	ADC_4_TEMPERATURE_TMC,    // ADC is running conversion on Channel 5  TEMPERATURE_TMC_ADC_CHANNEL
	ADC_5_TEMPERATURE_PCB,    // ADC is running conversion on Channel 6  TEMPERATURE_PCB_ADC_CHANNEL
	ADC_6_TEMPERATURE_MOT,    // ADC is running conversion on Channel 7  TEMPERATURE_MOT_ADC_CHANNEL
	ADC_7_SPINDLE_SPEED,      // ADC is running conversion on Channel 8  SPINDLE_SPEED_ADC_CHANNEL
	ADC_8_AC_LOSS,            // ADC is running conversion on Channel 9  AC_LOSS_ADC_CHANNEL        
	ADC_TOTAL_CHANNELS,       // normal state, ADC is off
};

/* ADC state machine structure */
typedef struct {
    uint8_t adc_state;                            /* global ADC state */
    uint16_t result[ADC_TOTAL_CHANNELS];          /* ADC readings for this round */
    uint8_t channel[ADC_TOTAL_CHANNELS];          /* all ADC channels */
    uint8_t measure_channel[ADC_TOTAL_CHANNELS];  /* ADC channels to be measured in this round */
    uint16_t tick_count[ADC_TOTAL_CHANNELS];      /* counter to define when to fire the conversion on given channel */
    uint16_t max_count[ADC_TOTAL_CHANNELS];       /* max counter value to define when to fire the conversion on given channel */
} ADCstateMachine;

ADCstateMachine ADCstMachine;    

void asmcnc_init_ADC(void);         /* initialise ADC for spindle load monitoring */
void asmcnc_start_ADC(void);        /* start ADC state machine from channel 1 */
int get_TMC_temperature (void);
int get_PCB_temperature (void);
int get_MOT_temperature (void);
int get_spindle_load_mV(void);
void adc_setup_and_fire(void); /* define ADC channels to be measured and start ADC conversions */
void adc_process_all_channels(void);/* Process results of all ADC channels */

void spindle_speed_feedback_rpm_updated(float rpm);


typedef struct
{
    uint8_t is_present;
    uint16_t RPM;
    uint32_t uptime;
    uint32_t brush_uptime;
    uint32_t load;
    uint32_t temperature;
} digitalSpindleStruct;

digitalSpindleStruct digitalSpindle;


#endif /* ADC_UTILS_H_ */