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
#define SPINDLE_LOAD_ADC_PERIOD_MS          80                      /* how often ADC should measure Spindle load channel */
#define VDD_5V_ATMEGA_ADC_PERIOD_MS         10000					/* how often ADC should measure 5V Atmega channel */
#define VDD_5V_DUSTSHOE_ADC_PERIOD_MS       10000                   /* how often ADC should measure 5V dustshoe channel */
#define VDD_24V_ADC_PERIOD_MS               1000				    /* how often ADC should measure 24V mains channel */
#define TEMPERATURE_TMC_ADC_PERIOD_MS       2000                    /* how often ADC should measure temperature 1 channel */
#define TEMPERATURE_PCB_ADC_PERIOD_MS       2000				    /* how often ADC should measure temperature 2 channel */
#define TEMPERATURE_MOT_ADC_PERIOD_MS       ADC_PERIOD_DISABLE      /* how often ADC should measure temperature 2 channel */
#define SPINDLE_SPEED_ADC_PERIOD_MS         1000                    /* how often ADC should measure Spindle speed channel */

#define SPINDLE_LOAD_ADC_CHANNEL            SPINDLE_LOAD_MONITOR // ADC1,  PF1
#define VDD_5V_ATMEGA_ADC_CHANNEL           12					 // ADC12, PK4
#define VDD_5V_DUSTSHOE_ADC_CHANNEL         13					 // ADC13, PK5
#define VDD_24V_ADC_CHANNEL                 14					 // ADC14, PK6
#define TEMPERATURE_TMC_ADC_CHANNEL         THERMISTOR_MONITOR   // ADC3,  PF3
#define TEMPERATURE_PCB_ADC_CHANNEL         8					 // ADC8,  PK0
#define TEMPERATURE_MOT_ADC_CHANNEL         9					 // ADC9,  PK1
#define SPINDLE_SPEED_ADC_CHANNEL           2					 // ADC2,  PF2



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
	ADC_TOTAL_CHANNELS,       // normal state, ADC is off
};

///* temperature coefficients for 2k thermistor, 10k ref, 5V VDD and 2.048V ref source */
//#define TEMP_K6  13176
//#define TEMP_K5 -42365
//#define TEMP_K4  52850
//#define TEMP_K3 -32364
//#define TEMP_K2  10221
//#define TEMP_K1 -1684
//#define TEMP_K0  162

/* temperature coefficients for 10k thermistor, 10k ref, 5V VDD and 2.048V ref source */
#define TEMP_K6_HW17  2968
#define TEMP_K5_HW17 -10456
#define TEMP_K4_HW17  14709
#define TEMP_K3_HW17 -10616
#define TEMP_K2_HW17  4250
#define TEMP_K1_HW17 -1020
#define TEMP_K0_HW17  201

/* temperature coefficients for 10k thermistor, 10k ref, 2.048V VDD and 2.048V ref source */
#define TEMP_K6_HW18  6287
#define TEMP_K5_HW18 -21401
#define TEMP_K4_HW18  28457
#define TEMP_K3_HW18 -18922
#define TEMP_K2_HW18  6690
#define TEMP_K1_HW18 -1337
#define TEMP_K0_HW18  177

/* ADC state machine structure */
typedef struct {
    uint8_t adc_state;                            /* global ADC state */
	uint8_t adc_locked;						      /* flag to indicate that the ADC state machine is in running and no conversions should be scheduled until it finished */
    uint8_t channel[ADC_TOTAL_CHANNELS];          /* all ADC channels */
    uint16_t result[ADC_TOTAL_CHANNELS];          /* ADC readings for this round */
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
int get_MOT_temperature_cent(void);
int get_spindle_load_mV(void);
int get_VDD_5V_Atmega_mV(void);
int get_VDD_5V_dustshoe_mV(void);
int get_VDD_24V_mV(void);
int get_Spindle_speed_Signal_mV(void);
void adc_setup_and_fire(void); /* define ADC channels to be measured and start ADC conversions */
void adc_state_machine(void ); /* store result of measured ADC channels and advance the state machine */
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