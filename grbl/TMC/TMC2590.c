/*
 * TMC2590.c
 
 */

#include "grbl.h"
#include "spi_to_tmc.h"
#include "TMC2590.h"

#include <string.h>

//uint16_t max_step_period_us_to_read_SG[] = { SG_MAX_VALID_PERIOD_X_US, SG_MAX_VALID_PERIOD_Y_US, SG_MAX_VALID_PERIOD_Z_US }; /* for SB2: X motor 23HS22-2804S - 18rpm, Y motor 23HS33-4008S - 18rpm, Z motor 17HS19-2004S1 - 60rpm,   */
uint8_t min_step_period_idx_to_read_SG[] = { 0, 0, 0 }; /* for SB2: X motor 23HS22-2804S - 18rpm, Y motor 23HS33-4008S - 18rpm, Z motor 17HS19-2004S1 - 60rpm,   */

#define HEX_BYTES_LEN 6
char ByteArrayToHexViaLookup[] = "0123456789ABCDEF";


/* calculate index into the LUT based on predefined maximum microstep periods */
void min_step_period_idx_compute(void){
    
    uint16_t step_period_us_to_read_SG[3];
    
    if ( st_tmc.calibration_enabled ) {
        step_period_us_to_read_SG[0] = SG_MAX_CALIBR_PERIOD_X_US;
        step_period_us_to_read_SG[1] = SG_MAX_CALIBR_PERIOD_Y_US;
        step_period_us_to_read_SG[2] = SG_MAX_CALIBR_PERIOD_Z_US;
    }   
    else{
        step_period_us_to_read_SG[0] = SG_MAX_VALID_PERIOD_X_US;
        step_period_us_to_read_SG[1] = SG_MAX_VALID_PERIOD_Y_US;
        step_period_us_to_read_SG[2] = SG_MAX_VALID_PERIOD_Z_US;
    }
                 
    uint16_t step_period_us = 0;
    for (uint8_t thisAxis=0; thisAxis<N_AXIS; thisAxis++) { 
        step_period_us = step_period_us_to_read_SG[thisAxis];
        for (uint8_t idx=0; idx<TMC_SG_PROFILE_POINTS; idx++){
            if ( step_period_us > pgm_read_word_near(SG_step_periods_us + idx) ){ //if storing in PROGMEM then use this: if ( st_tmc.step_period_us[thisAxis] > pgm_read_word_near(SG_step_periods_us + idx) ){
                min_step_period_idx_to_read_SG[thisAxis] = idx;
                break; /* for loop */
            } // if ( prep_segment->step_period_us[thisAxis] > pgm_read_word_near(SG_step_periods_us + idx) ){
        } //for (uint8_t idx=0; idx<TMC_SG_PROFILE_POINTS; idx++){            
    } //for (uint8_t idx=0; idx<N_AXIS; idx++) { 

}

/************************************************ single motor ***********************************************/

void tmc2590_single_writeInt(TMC2590TypeDef *tmc2590_1, uint8_t address)
{
    int32_t controller1_register_value, drvconf_register_value;

    controller1_register_value = tmc2590_1->shadowRegister[TMC_ADDRESS(address)];
    drvconf_register_value     = tmc2590_1->shadowRegister[TMC2590_DRVCONF     ];

    uint8_t rdsel = 0;
    
    uint8_t data[5]; 
    memset(data,0,5);    

    /* construct 5 bytes out of 2x20bit values */
    data[0] =                           NIBBLE(controller1_register_value, 4); //BYTE(controller1_register_value, 2);
    data[1] = (NIBBLE(controller1_register_value, 3)<<4) | NIBBLE(controller1_register_value, 2); //BYTE(controller1_register_value, 1);
    data[2] = (NIBBLE(controller1_register_value, 1)<<4) | NIBBLE(controller1_register_value, 0); //BYTE(controller1_register_value, 0); 
    
    // set virtual read address for next reply given by RDSEL, can only change by setting RDSEL in DRVCONF
    rdsel = TMC2590_GET_RDSEL(drvconf_register_value);
    
    /* schedule write to motor controller */
    spi_schedule_single_tx(tmc2590_1, data, TX_BUF_SIZE_SINGLE, rdsel);

}

uint8_t tmc2590_single_restore(TMC2590TypeDef *tmc2590_1)
{

	tmc2590_single_writeInt(tmc2590_1, TMC2590_DRVCONF);
	tmc2590_single_writeInt(tmc2590_1, TMC2590_DRVCTRL);
	tmc2590_single_writeInt(tmc2590_1, TMC2590_CHOPCONF);
	tmc2590_single_writeInt(tmc2590_1, TMC2590_SMARTEN);
	tmc2590_single_writeInt(tmc2590_1, TMC2590_SGCSCONF);
    
	return 1;
}

void tmc2590_read_single(TMC2590TypeDef *tmc2590_1, uint8_t rdsel){
    
    uint32_t drvconf_register_value;
    
   	drvconf_register_value = tmc2590_1->shadowRegister[TMC2590_DRVCONF];
	   
	drvconf_register_value &= ~TMC2590_SET_RDSEL(-1);      // clear RDSEL bits
	drvconf_register_value |= TMC2590_SET_RDSEL(rdsel);    // set rdsel
    //nrf_delay_us(delay_us);
    
    tmc2590_1->shadowRegister[TMC2590_DRVCONF] = drvconf_register_value;

    tmc2590_single_writeInt(tmc2590_1, TMC2590_DRVCONF);
    
}

void tmc2590_single_read_all(TMC2590TypeDef *tmc2590)
{
    /*read all 4 report values */
    tmc2590_read_single(tmc2590, ( ( DEFAULT_TMC_READ_SELECT + 1 ) % 4 ) ); /* response 1 */
    tmc2590_read_single(tmc2590, ( ( DEFAULT_TMC_READ_SELECT + 2 ) % 4 ) ); /* response 2 */
    tmc2590_read_single(tmc2590, ( ( DEFAULT_TMC_READ_SELECT + 3 ) % 4 ) ); /* response 3 */
    tmc2590_read_single(tmc2590, (   DEFAULT_TMC_READ_SELECT           ) ); /* response 0 */
}


void tmc2590_single_read_sg(TMC2590TypeDef *tmc2590)
{
    /*read stall guard and MSTEP report values */
#ifdef MSTEP_READING_ENABLED
    tmc2590_read_single(tmc2590, ( ( DEFAULT_TMC_READ_SELECT + 3 ) % 4 ) ); /* response 0 */
#endif
    tmc2590_read_single(tmc2590, (   DEFAULT_TMC_READ_SELECT           ) ); /*  response 1 (0 when reading MSTEP) */
}

/************************************************ dual motors ***********************************************/

void tmc2590_dual_writeInt(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2, uint8_t address)
{
    int32_t controller1_register_value, controller2_register_value, drvconf_register_value;

    controller1_register_value = tmc2590_1->shadowRegister[TMC_ADDRESS(address)];
    controller2_register_value = tmc2590_2->shadowRegister[TMC_ADDRESS(address)];
    drvconf_register_value     = tmc2590_1->shadowRegister[TMC2590_DRVCONF     ];
    
    uint8_t rdsel = 0;
    /* read select shall always be taken from DRVCONF register */
    // set virtual read address for next reply given by RDSEL, can only change by setting RDSEL in DRVCONF
    rdsel = TMC2590_GET_RDSEL(drvconf_register_value);

    uint8_t data[5]; 

    /* construct 5 bytes out of 2x20bit values */
    data[0] = (NIBBLE(controller2_register_value, 4)<<4) | NIBBLE(controller2_register_value, 3); //BYTE(controller2_register_value, 2);        
    data[1] = (NIBBLE(controller2_register_value, 2)<<4) | NIBBLE(controller2_register_value, 1); //BYTE(controller2_register_value, 1);        
    data[2] = (NIBBLE(controller2_register_value, 0)<<4) | NIBBLE(controller1_register_value, 4); //BYTE(controller1_register_value, 2);
    data[3] = (NIBBLE(controller1_register_value, 3)<<4) | NIBBLE(controller1_register_value, 2); //BYTE(controller1_register_value, 1);
    data[4] = (NIBBLE(controller1_register_value, 1)<<4) | NIBBLE(controller1_register_value, 0); //BYTE(controller1_register_value, 0); 
    
    
    /* can write to the same channel as both motors are on the same line */
    spi_schedule_dual_tx(tmc2590_1, tmc2590_2, data, TX_BUF_SIZE_DUAL, rdsel);    

}


uint8_t tmc2590_dual_restore(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2)
{

	tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, TMC2590_DRVCONF);
	tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, TMC2590_DRVCTRL);
	tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, TMC2590_CHOPCONF);
	tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, TMC2590_SMARTEN);
	tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, TMC2590_SGCSCONF);
    
	return 1;
}


void tmc2590_dual_read_single(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2, uint8_t rdsel){
    
    //uint32_t delay_us = 50;
    
    uint32_t drvconf1_register_value, drvconf2_register_value;
    
   	drvconf1_register_value = tmc2590_1->shadowRegister[TMC2590_DRVCONF];
   	drvconf2_register_value = tmc2590_2->shadowRegister[TMC2590_DRVCONF];
	            
	drvconf1_register_value &= ~TMC2590_SET_RDSEL(-1);      // clear RDSEL bits
	drvconf1_register_value |= TMC2590_SET_RDSEL(rdsel);    // set rdsel
	drvconf2_register_value &= ~TMC2590_SET_RDSEL(-1);      // clear RDSEL bits
	drvconf2_register_value |= TMC2590_SET_RDSEL(rdsel);    // set rdsel
    
    tmc2590_1->shadowRegister[TMC2590_DRVCONF] = drvconf1_register_value;
    tmc2590_2->shadowRegister[TMC2590_DRVCONF] = drvconf2_register_value;

    tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, TMC2590_DRVCONF);
    
}

void tmc2590_dual_read_all(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2)
{
    /*read all 4 report values */
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, ( ( DEFAULT_TMC_READ_SELECT + 1 ) % 4 ) ); /* response 1 */
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, ( ( DEFAULT_TMC_READ_SELECT + 2 ) % 4 ) ); /* response 2 */
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, ( ( DEFAULT_TMC_READ_SELECT + 3 ) % 4 ) ); /* response 3 */
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, (   DEFAULT_TMC_READ_SELECT     )       ); /* response 0 */
}


void tmc2590_dual_read_sg(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2)
{
    /*read stall guard and MSTEP report values */
#ifdef MSTEP_READING_ENABLED
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, ( ( DEFAULT_TMC_READ_SELECT + 3 ) % 4 ) ); /* response 1 */
#endif
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, (   DEFAULT_TMC_READ_SELECT     )       ); /* response 1 (0 when reading MSTEP) */
}




void tmc_trigger_stall_alarm(uint8_t axis){
    
    if (st_tmc.stall_alarm_enabled){
        
        /* execute alarm by writing 1 to the limit pin, which will trigger the ISR routine (pin has to be configured as output) */
        switch (axis){
            case X_AXIS:
                LIMIT_PORT |= (1<<X_LIMIT_BIT);  /* set pin */           
            break;
            
            case Y_AXIS:
                LIMIT_PORT |= (1<<Y_LIMIT_BIT);  /* set pin */
            break;
            
            case Z_AXIS:
                LIMIT_PORT |= (1<<Z_LIMIT_BIT);  /* set pin */
            break;
            
            default:
                //mc_reset(); // Initiate system kill.
                //system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // Indicate hard limit critical event            
            break;            
            
        } //switch (axis){
        
    } //if (st_tmc.stall_alarm_enabled){
        
}



/************************************************ stall guad calibration ***********************************************/

/* structure to hold live SG profile to track the load and alarm on stall detection*/
uint8_t  SG_calibration_read_cnt[TOTAL_TMCS][TMC_SG_PROFILE_POINTS];  // counter for averaging of SG values at calibration
uint16_t SG_calibration_value[TOTAL_TMCS][TMC_SG_PROFILE_POINTS];     // SG reads for each motor



const uint16_t SG_step_periods_us[] PROGMEM =   /* must be size of TMC_SG_PROFILE_POINTS, can be put in PROGMEM  */
{
    6250, /* entry 1, speed=3.0rpm, feed=169.4mm/min */
    6036, /* entry 2, speed=3.1rpm, feed=175.4mm/min */
    5829, /* entry 3, speed=3.2rpm, feed=181.6mm/min */
    5630, /* entry 4, speed=3.3rpm, feed=188.1mm/min */
    5437, /* entry 5, speed=3.4rpm, feed=194.7mm/min */
    5251, /* entry 6, speed=3.6rpm, feed=201.6mm/min */
    5071, /* entry 7, speed=3.7rpm, feed=208.8mm/min */
    4898, /* entry 8, speed=3.8rpm, feed=216.2mm/min */
    4730, /* entry 9, speed=4.0rpm, feed=223.8mm/min */
    4568, /* entry 10, speed=4.1rpm, feed=231.8mm/min */
    4412, /* entry 11, speed=4.2rpm, feed=240.0mm/min */
    4261, /* entry 12, speed=4.4rpm, feed=248.5mm/min */
    4115, /* entry 13, speed=4.6rpm, feed=257.3mm/min */
    3974, /* entry 14, speed=4.7rpm, feed=266.4mm/min */
    3838, /* entry 15, speed=4.9rpm, feed=275.9mm/min */
    3707, /* entry 16, speed=5.1rpm, feed=285.6mm/min */
    3580, /* entry 17, speed=5.2rpm, feed=295.8mm/min */
    3458, /* entry 18, speed=5.4rpm, feed=306.2mm/min */
    3339, /* entry 19, speed=5.6rpm, feed=317.1mm/min */
    3225, /* entry 20, speed=5.8rpm, feed=328.3mm/min */
    3115, /* entry 21, speed=6.0rpm, feed=340.0mm/min */
    3008, /* entry 22, speed=6.2rpm, feed=352.0mm/min */
    2905, /* entry 23, speed=6.5rpm, feed=364.5mm/min */
    2806, /* entry 24, speed=6.7rpm, feed=377.4mm/min */
    2710, /* entry 25, speed=6.9rpm, feed=390.8mm/min */
    2617, /* entry 26, speed=7.2rpm, feed=404.6mm/min */
    2527, /* entry 27, speed=7.4rpm, feed=419.0mm/min */
    2441, /* entry 28, speed=7.7rpm, feed=433.8mm/min */
    2357, /* entry 29, speed=8.0rpm, feed=449.2mm/min */
    2277, /* entry 30, speed=8.2rpm, feed=465.1mm/min */
    2199, /* entry 31, speed=8.5rpm, feed=481.6mm/min */
    2123, /* entry 32, speed=8.8rpm, feed=498.7mm/min */
    2051, /* entry 33, speed=9.1rpm, feed=516.3mm/min */
    1980, /* entry 34, speed=9.5rpm, feed=534.6mm/min */
    1913, /* entry 35, speed=9.8rpm, feed=553.6mm/min */
    1847, /* entry 36, speed=10.2rpm, feed=573.2mm/min */
    1784, /* entry 37, speed=10.5rpm, feed=593.5mm/min */
    1723, /* entry 38, speed=10.9rpm, feed=614.5mm/min */
    1664, /* entry 39, speed=11.3rpm, feed=636.3mm/min */
    1607, /* entry 40, speed=11.7rpm, feed=658.9mm/min */
    1552, /* entry 41, speed=12.1rpm, feed=682.2mm/min */
    1499, /* entry 42, speed=12.5rpm, feed=706.4mm/min */
    1448, /* entry 43, speed=13.0rpm, feed=731.4mm/min */
    1398, /* entry 44, speed=13.4rpm, feed=757.4mm/min */
    1350, /* entry 45, speed=13.9rpm, feed=784.2mm/min */
    1304, /* entry 46, speed=14.4rpm, feed=812.0mm/min */
    1259, /* entry 47, speed=14.9rpm, feed=840.8mm/min */
    1216, /* entry 48, speed=15.4rpm, feed=870.6mm/min */
    1175, /* entry 49, speed=16.0rpm, feed=901.4mm/min */
    1134, /* entry 50, speed=16.5rpm, feed=933.4mm/min */
    1096, /* entry 51, speed=17.1rpm, feed=966.4mm/min */
    1058, /* entry 52, speed=17.7rpm, feed=1000.7mm/min */
    1022, /* entry 53, speed=18.3rpm, feed=1036.1mm/min */
    987, /* entry 54, speed=19.0rpm, feed=1072.9mm/min */
    953, /* entry 55, speed=19.7rpm, feed=1110.9mm/min */
    921, /* entry 56, speed=20.4rpm, feed=1150.3mm/min */
    889, /* entry 57, speed=21.1rpm, feed=1191.0mm/min */
    859, /* entry 58, speed=21.8rpm, feed=1233.2mm/min */
    829, /* entry 59, speed=22.6rpm, feed=1276.9mm/min */
    801, /* entry 60, speed=23.4rpm, feed=1322.2mm/min */
    773, /* entry 61, speed=24.2rpm, feed=1369.0mm/min */
    747, /* entry 62, speed=25.1rpm, feed=1417.6mm/min */
    721, /* entry 63, speed=26.0rpm, feed=1467.8mm/min */
    697, /* entry 64, speed=26.9rpm, feed=1519.8mm/min */
    673, /* entry 65, speed=27.9rpm, feed=1573.7mm/min */
    650, /* entry 66, speed=28.9rpm, feed=1629.4mm/min */
    628, /* entry 67, speed=29.9rpm, feed=1687.2mm/min */
    606, /* entry 68, speed=30.9rpm, feed=1747.0mm/min */
    585, /* entry 69, speed=32.0rpm, feed=1808.9mm/min */
    565, /* entry 70, speed=33.2rpm, feed=1873.0mm/min */
    546, /* entry 71, speed=34.3rpm, feed=1939.4mm/min */
    527, /* entry 72, speed=35.6rpm, feed=2008.1mm/min */
    509, /* entry 73, speed=36.8rpm, feed=2079.3mm/min */
    492, /* entry 74, speed=38.1rpm, feed=2153.0mm/min */
    475, /* entry 75, speed=39.5rpm, feed=2229.3mm/min */
    459, /* entry 76, speed=40.9rpm, feed=2308.3mm/min */
    443, /* entry 77, speed=42.3rpm, feed=2390.1mm/min */
    428, /* entry 78, speed=43.8rpm, feed=2474.8mm/min */
    413, /* entry 79, speed=45.4rpm, feed=2562.5mm/min */
    399, /* entry 80, speed=47.0rpm, feed=2653.3mm/min */
    385, /* entry 81, speed=48.7rpm, feed=2747.3mm/min */
    372, /* entry 82, speed=50.4rpm, feed=2844.7mm/min */
    359, /* entry 83, speed=52.2rpm, feed=2945.5mm/min */
    347, /* entry 84, speed=54.0rpm, feed=3049.9mm/min */
    335, /* entry 85, speed=55.9rpm, feed=3158.0mm/min */
    324, /* entry 86, speed=57.9rpm, feed=3269.9mm/min */
    313, /* entry 87, speed=60.0rpm, feed=3385.8mm/min */
    302, /* entry 88, speed=62.1rpm, feed=3505.7mm/min */
    292, /* entry 89, speed=64.3rpm, feed=3630.0mm/min */
    282, /* entry 90, speed=66.6rpm, feed=3758.6mm/min */
    272, /* entry 91, speed=68.9rpm, feed=3891.8mm/min */
    263, /* entry 92, speed=71.4rpm, feed=4029.8mm/min */
    254, /* entry 93, speed=73.9rpm, feed=4172.6mm/min */
    245, /* entry 94, speed=76.5rpm, feed=4320.4mm/min */
    237, /* entry 95, speed=79.2rpm, feed=4473.5mm/min */
    229, /* entry 96, speed=82.0rpm, feed=4632.1mm/min */
    221, /* entry 97, speed=84.9rpm, feed=4796.2mm/min */
    213, /* entry 98, speed=87.9rpm, feed=4966.2mm/min */
    206, /* entry 99, speed=91.1rpm, feed=5142.2mm/min */
    199, /* entry 100, speed=94.3rpm, feed=5324.4mm/min */
    192, /* entry 101, speed=97.6rpm, feed=5513.1mm/min */
    185, /* entry 102, speed=101.1rpm, feed=5708.5mm/min */
    179, /* entry 103, speed=104.7rpm, feed=5910.8mm/min */
    173, /* entry 104, speed=108.4rpm, feed=6120.3mm/min */
    167, /* entry 105, speed=112.2rpm, feed=6337.2mm/min */
    161, /* entry 106, speed=116.2rpm, feed=6561.8mm/min */
    156, /* entry 107, speed=120.3rpm, feed=6794.3mm/min */
    151, /* entry 108, speed=124.6rpm, feed=7035.1mm/min */
    145, /* entry 109, speed=129.0rpm, feed=7284.4mm/min */
    140, /* entry 110, speed=133.6rpm, feed=7542.6mm/min */
    136, /* entry 111, speed=138.3rpm, feed=7809.9mm/min */
    131, /* entry 112, speed=143.2rpm, feed=8086.7mm/min */
    126, /* entry 113, speed=148.3rpm, feed=8373.2mm/min */
    122, /* entry 114, speed=153.5rpm, feed=8670.0mm/min */
    118, /* entry 115, speed=159.0rpm, feed=8977.2mm/min */
    114, /* entry 116, speed=164.6rpm, feed=9295.4mm/min */
    110, /* entry 117, speed=170.4rpm, feed=9624.8mm/min */
    106, /* entry 118, speed=176.5rpm, feed=9965.9mm/min */
    103, /* entry 119, speed=182.7rpm, feed=10319.1mm/min */
    99, /* entry 120, speed=189.2rpm, feed=10684.8mm/min */
    96, /* entry 121, speed=195.9rpm, feed=11063.4mm/min */
    92, /* entry 122, speed=202.9rpm, feed=11455.5mm/min */
    89, /* entry 123, speed=210.0rpm, feed=11861.5mm/min */
    86, /* entry 124, speed=217.5rpm, feed=12281.9mm/min */
    83, /* entry 125, speed=225.2rpm, feed=12717.1mm/min */
    80, /* entry 126, speed=233.2rpm, feed=13167.8mm/min */
    78, /* entry 127, speed=241.4rpm, feed=13634.5mm/min */
    75, /* entry 128, speed=250.0rpm, feed=14117.6mm/min */
};           


/* todo rewrite search algo to depend on last value - should be very close and hence search will be faster */
/* profler: RAM: 300us per search;  PROGMEM: 350us per search*/ 
void tmc_store_calibration_point(	uint8_t thisMotor, uint8_t thisAxis, uint16_t stallGuardCurrentValue){      
    
#ifdef SG_CAL_DEBUG_ENABLED
debug_pin_write(1, DEBUG_1_PIN);
#endif    
    /* find entry in calibration table based on step_period_idx and add it */
    uint8_t idx = st_tmc.step_period_idx[thisAxis];
    
    /* option 1: calibration is based on average value */
    //if (SG_calibration_read_cnt[thisMotor][idx] < TMC_SG_MAX_AVERAGE) { /* only accumulate until max average is reached to keep 10bit SG value in 16 bit accumulator */
        //SG_calibration_value[thisMotor][idx] += stallGuardCurrentValue;
        //SG_calibration_read_cnt[thisMotor][idx] ++;
    //} /* end ot option 1 */ 
    
    /* option 2: calibration is based on minimum value - better for harmonic distortions */
    if (SG_calibration_read_cnt[thisMotor][idx]==0){/* first entry */
        SG_calibration_value[thisMotor][idx] = stallGuardCurrentValue;
        SG_calibration_read_cnt[thisMotor][idx] = 1;
    }
    else{ /* keep mimimum */
        if (SG_calibration_value[thisMotor][idx] > stallGuardCurrentValue){
            SG_calibration_value[thisMotor][idx] = stallGuardCurrentValue;    
        }        
    } /* end ot option 2 */ 
    
#ifdef SG_CAL_DEBUG_ENABLED
debug_pin_write(0, DEBUG_1_PIN);
#endif
    
}

/* clear calibration matrix and get ready for data collection */
void tmc_calibration_init(void){
    st_tmc.calibration_enabled = 1;
    st_tmc.stall_alarm_enabled  = false;                  /* global holding desired stall behaviour: if "true" then stall guard value below the limit will trigger alarm      */    

    allow_periodic_TMC_poll(0); /* disable TMC polls for home cycle duration*/

    /* lower the max step period for calibration purposes */ 
    min_step_period_idx_compute();
    
    memset(SG_calibration_read_cnt, 0, sizeof(SG_calibration_read_cnt));
    memset(SG_calibration_value, 0, sizeof(SG_calibration_value));
}

/* stop calibration and compute coefficients based on accumulated data */
void tmc_compute_and_apply_calibration(void){
    
    /* loop over the whole table and calculate average */    
    /* cycle through every motors */
    uint8_t controller_id;
    uint8_t idx;
    uint16_t last_SG_read = 0;
    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
        /* first apply averaging and fill 0 entires with lower values */             
        for (idx=0; idx<TMC_SG_PROFILE_POINTS; idx++){
            if ( SG_calibration_read_cnt[controller_id][idx] > 0 ) { /* catch divide by 0 */
                SG_calibration_value[controller_id][idx] /= SG_calibration_read_cnt[controller_id][idx]; /* find average SG value */
                last_SG_read = SG_calibration_value[controller_id][idx]; /* keep last entry in cache in case next entry is empty */
                SG_calibration_read_cnt[controller_id][idx] = 1; /*reset count to 1 to keep correct values in case that averaging is accidentally requested once again */
            }
            else{ /* if empty entry use the last filled one */
                SG_calibration_value[controller_id][idx] = last_SG_read;
            }            
        } 
        /* finally sweep down and fill 0 entires with upper values */
        for (idx=0; idx<TMC_SG_PROFILE_POINTS; idx++){
            if ( SG_calibration_read_cnt[controller_id][TMC_SG_PROFILE_POINTS-1-idx] == 0 ) { /* catch empty entries */
                SG_calibration_value[controller_id][TMC_SG_PROFILE_POINTS-1-idx] = last_SG_read;
            }
            else{
                last_SG_read = SG_calibration_value[controller_id][TMC_SG_PROFILE_POINTS-1-idx];
            }
        }        
    }             
    
    #ifdef FLASH_DEBUG_ENABLED
    debug_pin_write(1, DEBUG_1_PIN);
    #endif
    /* save calibration to eeprom */
    memcpy_to_eeprom_with_checksum(EEPROM_ADDR_TMC_CALIBRATION, (char*)SG_calibration_value, sizeof(SG_calibration_value));    
    #ifdef FLASH_DEBUG_ENABLED
    debug_pin_write(0, DEBUG_1_PIN);
    #endif
    
    st_tmc.calibration_enabled = 0;    
    
    /*reenable alarm */
    st_tmc.stall_alarm_enabled  = true;                  /* global holding desired stall behaviour: if "true" then stall guard value below the limit will trigger alarm      */
    
    /* restore the max step period after calibration */
    min_step_period_idx_compute();
    
    allow_periodic_TMC_poll(1); /* reenable SPI TMC polls for home cycle duration*/

}

/* print out calibration data */
void tmc_report_calibration(void){
    uint8_t controller_id;
    uint8_t idx;
    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
        printPgmString(PSTR("<|TCAL:M"));
        printInteger( controller_id );        
        printPgmString(PSTR(":"));
        for (idx=0; idx<TMC_SG_PROFILE_POINTS; idx++){
    	    printPgmString(PSTR(","));            
    	    printInteger( SG_calibration_value[controller_id][idx] );
        }
        printPgmString(PSTR(">\n"));
    }

}


/* print TMC stall guard deltas to UART */
void tmc_report_SG_delta(void){
    /* cycle through all motors */        
    uint8_t controller_id;
    TMC2590TypeDef *tmc2590;
    printPgmString(PSTR("|TSG:"));
    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
	    tmc2590 = get_TMC_controller(controller_id);
        printInteger( tmc2590->stallGuardDelta );
        if (controller_id < TOTAL_TMCS-1) {printPgmString(PSTR(","));}
    } //for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
    stall_guard_statistics_reset();          
}

/* print full TMC statistics hex string out to UART */
void tmc_report_status(void){
    
  #ifdef ENABLE_TMC_FEEDBACK_MONITOR
      /* cycle through all motors */
      uint8_t controller_id;
      TMC2590TypeDef *tmc2590;
      uint8_t hex_byte_buffer[HEX_BYTES_LEN];
      for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
	      tmc2590 = get_TMC_controller(controller_id);

	      /* pack values to hex string
	      * motor                       param                   range, bits bytes   hex bytes
	      * X1                          stallGuardCurrentValue   10          2       4
	      * X1                          coolStepCurrentValue     5
	      * X1                          StatusBits              8           1       2
	      * X1                          DiagnosticBits          10          3       6
	      * X1                          MSTEP                   10
	      * */
	      /* split the data into nibbles and convert to hex string through the lookup table */
	      hex_byte_buffer[0]  =  tmc2590->resp.stallGuardCurrentValue       & 0xFF; /* LSB 8 bits of SG */
	      hex_byte_buffer[1]  = (tmc2590->resp.stallGuardCurrentValue >> 8) & 0x03; /* MSB 2 bits of SG */
	      hex_byte_buffer[1] |=  tmc2590->resp.coolStepCurrentValue   << 2;
	      hex_byte_buffer[2]  =  tmc2590->resp.StatusBits;
	      hex_byte_buffer[3]  =  tmc2590->resp.DiagnosticBits              & 0xFF; /* LSB 8 bits of DiagnosticBits */
	      hex_byte_buffer[4]  = (tmc2590->resp.DiagnosticBits        >> 8) & 0x03; /* MSB 2 bits of DiagnosticBits */
	      hex_byte_buffer[4] |= (tmc2590->resp.mStepCurrentValue      << 2) & 0xFC; /* LSB 6 bits of MSTEP */
	      hex_byte_buffer[5]  = (tmc2590->resp.mStepCurrentValue      >> 6) & 0xF;  /* MSB 4 bits of MSTEP */
	      /* convert bytes to hex str  */
	      char hex_str_buffer[HEX_BYTES_LEN*2+1];
	      for (uint8_t i = 0; i < HEX_BYTES_LEN ; i ++){
	          hex_str_buffer[i*2+1] = ByteArrayToHexViaLookup[hex_byte_buffer[i]    & 0xF];        /* LSB 4 bits */
	          hex_str_buffer[i*2  ] = ByteArrayToHexViaLookup[hex_byte_buffer[i]>>4 & 0xF];        /* MSB 4 bits */
	          //printInteger( hex_byte_buffer[i] );
	          //printPgmString(PSTR(","));
	      }
	      hex_str_buffer[HEX_BYTES_LEN*2] = 0; /* terminator */
	      printString(hex_str_buffer);

      } //for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){      

  #endif //#ifdef ENABLE_TMC_FEEDBACK_MONITOR    
    
}


void tmc_load_stall_guard_calibration(void){
    
    if (!(memcpy_from_eeprom_with_checksum((char*)SG_calibration_value, EEPROM_ADDR_TMC_CALIBRATION, sizeof(SG_calibration_value)))) {
        uint8_t controller_id;
        uint8_t idx;
        TMC2590TypeDef *tmc2590;
        /* load TMC calibration from eeprom for each motor */
        for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
                // If no calibration in EEPROM then Reset with default thresholds vector
                /* init default calibration values */
                tmc2590 = get_TMC_controller(controller_id);
                for (idx=0; idx<TMC_SG_PROFILE_POINTS; idx++){
                    SG_calibration_value[controller_id][idx] = tmc2590->stallGuardAlarmValue + tmc2590->stallGuardAlarmThreshold;
                }            
        } //for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
    
    } //if (!(memcpy_from_eeprom_with_checksum((char*)&flashTMCcalibration, EEPROM_ADDR_TMC_CALIBRATION, sizeof(FlashTMCcalibration)))) {
        
    /*reset count to 1 to keep correct values in case that averaging is accidentally requested once again */
    memset(&SG_calibration_read_cnt, 1, sizeof(SG_calibration_read_cnt));

}



/************************************************ all motors ***********************************************/



void process_controller_status(TMC2590TypeDef *tmc2590){

#ifdef SG_SKIP_DEBUG_ENABLED
/* measured 50-60us per function call */
debug_pin_write(1, DEBUG_1_PIN);
#endif
    
    /* TMC2590_RESPONSE0 #define TMC2590_GET_MSTEP(X)  (0x3FF & ((X) >> 10)) */     
    //tmc2590->resp.mStepCurrentValue = TMC2590_GET_MSTEP(tmc2590->response[TMC2590_RESPONSE0]) & 0x1FF; /* bit 9 is polarity bit, ignore it*/
    tmc2590->resp.mStepCurrentValue = TMC2590_GET_MSTEP(tmc2590->response[TMC2590_RESPONSE0]);
    tmc2590->resp.stallGuardCurrentValue = TMC2590_GET_SG(tmc2590->response[TMC2590_RESPONSE1]);

    /* TMC2590_RESPONSE2 #define TMC2590_GET_SGU(X)    (0x1F & ((X) >> 15)) #define TMC2590_GET_SE(X)     (0x1F & ((X) >> 10))    */
    tmc2590->resp.stallGuardShortValue= TMC2590_GET_SGU(tmc2590->response[TMC2590_RESPONSE2]);
    tmc2590->resp.coolStepCurrentValue= TMC2590_GET_SE(tmc2590->response[TMC2590_RESPONSE2]);

    /* TMC2590_RESPONSE3 status and diagnostic */
    tmc2590->resp.StatusBits = tmc2590->response[TMC2590_RESPONSE3] & 0xFF;
    tmc2590->resp.DiagnosticBits = (tmc2590->response[TMC2590_RESPONSE3] & 0xFFC00) >> 10 ;

    if ( ( st_tmc.calibration_enabled ) && ( st_tmc.current_scale_state == CURRENT_SCALE_ACTIVE ) ){
        /* if feed is fast and SG_skips_counter is higher than min then store calibration value */
        if ( st_tmc.SG_skips_counter[tmc2590->thisAxis] >= SG_READING_SKIPS_AFTER_SLOW_FEED ){
            tmc_store_calibration_point(tmc2590->thisMotor, tmc2590->thisAxis, tmc2590->resp.stallGuardCurrentValue);
        }
    }        

    int16_t stallGuardDelta = -999; /* default to invalid SG delta reading */
    
    if ( ( st_tmc.stall_alarm_enabled ) && ( st_tmc.current_scale_state == CURRENT_SCALE_ACTIVE ) ){

        /* check whether direction has changed, if so, reset skip counter */
        uint8_t last_direction = st_tmc.last_reading_direction[tmc2590->thisAxis] & get_direction_pin_mask(tmc2590->thisAxis);
        uint8_t this_direction = st_tmc.this_reading_direction[tmc2590->thisAxis] & get_direction_pin_mask(tmc2590->thisAxis);
        st_tmc.last_reading_direction[tmc2590->thisAxis] = st_tmc.this_reading_direction[tmc2590->thisAxis];
        if (last_direction != this_direction) {
            /* direction changed, reset skip counter */
            //debug_pin_write(1, DEBUG_1_PIN);
            st_tmc.SG_skips_counter[tmc2590->thisAxis] = 0;
            //debug_pin_write(0, DEBUG_1_PIN);
        }                
        
        /* start reading SG if rotational speed is sufficiently high */                       
        /* feed speed validation. If feed was slow then SG_skips_counter gets reset, then decrement till reaches 0, only after that SG analysis for stall detection is allowed */
        if ( st_tmc.step_period_idx[tmc2590->thisAxis] > min_step_period_idx_to_read_SG[tmc2590->thisAxis] ) {  /* check stall only if feed is higher than defined for this motor */
            
            /* feed is fast, increment SG_skips_counter until 0 then analyse SG for stall */
            if ( st_tmc.SG_skips_counter[tmc2590->thisAxis] >= SG_READING_SKIPS_AFTER_SLOW_FEED )
            {
                /* feed rate is high enough and started more than SG_READING_SKIPS_AFTER_SLOW_FEED ago, lets analyse the stall */
                
                /* init stallGuardAlarmValue with entry from max speed */
                uint16_t stallGuardAlarmValue = 0;
                if (SG_calibration_value[tmc2590->thisMotor][TMC_SG_PROFILE_POINTS-1] > tmc2590->stallGuardAlarmThreshold){ /* only do so if there is enough headroom below the caliration value to track the SG, otherwise keep it "0" which is ALARM_DIASBLED*/
                    stallGuardAlarmValue = SG_calibration_value[tmc2590->thisMotor][TMC_SG_PROFILE_POINTS-1] - tmc2590->stallGuardAlarmThreshold ;    
                }
                
                /* find alarm value based on calibration matrix and predefined threshold */
                #ifdef SG_CAL_DEBUG_ENABLED
                debug_pin_write(1, DEBUG_1_PIN);
                #endif    
                /* find entry in calibration table based on step_period_idx and extract SG calibrated level from there */
                uint8_t idx = st_tmc.step_period_idx[tmc2590->thisAxis];
                /* entry found, apply threshold */
                if (SG_calibration_value[tmc2590->thisMotor][idx] > tmc2590->stallGuardAlarmThreshold){ /* only do so if there is enough headroom below the caliration value to track the SG, otherwise keep it "0" which is ALARM_DIASBLED*/
                    stallGuardAlarmValue = SG_calibration_value[tmc2590->thisMotor][idx] - tmc2590->stallGuardAlarmThreshold ;                                
                }
                else{
                    /* silence the alarm: only entries with positive Alarm values will trigger alarm */ 
                    stallGuardAlarmValue = 0;
                }
                #ifdef SG_CAL_DEBUG_ENABLED
                debug_pin_write(0, DEBUG_1_PIN);
                #endif
                
                /* keep track of how far the current SG value is from the calibrated level, this is printed to UART and used to indicate the current load. The bigger the value-> the higher the load.*/
                if (stallGuardAlarmValue > 0){
                    /* SG reading is valid and enough headroom is remaining below calibrated value*/
                    stallGuardDelta = SG_calibration_value[tmc2590->thisMotor][idx] - tmc2590->resp.stallGuardCurrentValue;
                }
                
                /* find maximum stallGuardDelta over reporting period */
                if (tmc2590->stallGuardDelta < stallGuardDelta) {
                    tmc2590->stallGuardDelta  = stallGuardDelta;}
                
                if (tmc2590->resp.stallGuardCurrentValue    < stallGuardAlarmValue) {
                    /* trigger alarm */
                    tmc_trigger_stall_alarm(tmc2590->thisAxis);
                    /* store stall info to flash */
                    tmc_store_stall_info(tmc2590->thisMotor, tmc2590->resp.stallGuardCurrentValue, stallGuardAlarmValue + tmc2590->stallGuardAlarmThreshold, st_tmc.step_period[tmc2590->thisAxis]);
                    /* reset SG period to max as alarm will immediately stop the stepper and period will remain as it was at the point of trigger */
                    st_tmc.step_period_idx[tmc2590->thisAxis] = 0;
                    
                    printPgmString(PSTR(BK_INITIATOR));
                    printPgmString(PSTR("SGALARM:"));
                    printInteger( tmc2590->thisMotor);
                    printPgmString(PSTR(","));
                    printInteger( st_tmc.step_period[tmc2590->thisAxis] ); /* actual step in us */
                    printPgmString(PSTR(","));
                    printInteger( tmc2590->resp.stallGuardCurrentValue);
                    printPgmString(PSTR(","));
                    printInteger( stallGuardAlarmValue );
                    printPgmString(PSTR(BK_TERMINATOR));
                    
                } //if (tmc2590->resp.stallGuardCurrentValue    < tmc2590->stallGuardAlarmValue) {
   
            } //if ( st_tmc.SG_skips_counter[tmc2590->thisAxis] >= SG_READING_SKIPS_AFTER_SLOW_FEED )
                
        } // if ( st_tmc.step_period_idx[tmc2590->thisAxis] > min_step_period_idx_to_read_SG[tmc2590->thisAxis] ) {  /* check stall only if feed is higher than defined for this motor */
            
        /* if above "if" is not entered then feed is slow, and SG_skips_counter is being reset by st_tmc_fire_SG_read() */
            
    } //else if (st_tmc.stall_alarm_enabled){
    else{
        if ((!st_tmc.stall_alarm_enabled) && (st_tmc.current_scale_state == CURRENT_SCALE_ACTIVE)){ /* only report SG delta when active and when period is shorter then calibration*/
            if ((st_tmc.step_period_idx[tmc2590->thisAxis] > min_step_period_idx_to_read_SG[tmc2590->thisAxis])&&(st_tmc.SG_skips_counter[tmc2590->thisAxis] >= SG_READING_SKIPS_AFTER_SLOW_FEED)){ 
                uint8_t idx = st_tmc.step_period_idx[tmc2590->thisAxis];
                
                ///* linear interpolation between calibration points: */
                //int32_t y, y1, y2, x, x1, x2; 
                //// BK TODO: make sure index is within valid range of TMC_SG_PROFILE_POINTS
                //x = st_tmc.step_period[tmc2590->thisAxis]; /* actual step in us */
                //x1 = pgm_read_word_near(SG_step_periods_us+idx-1); /* step in us */
                //x2 = pgm_read_word_near(SG_step_periods_us+idx);   /* step in us */
                //y1 = SG_calibration_value[tmc2590->thisMotor][idx-1]; /* SG calibration reading */
                //y2 = SG_calibration_value[tmc2590->thisMotor][idx];   /* SG calibration reading */                
                ///* linear interpolation */
                //y = y1 + (y2-y1)*(x-x1)/(x2-x1); /* interpolated SG calibration reading */                                                                    
                //stallGuardDelta = y - tmc2590->resp.stallGuardCurrentValue;
                
                stallGuardDelta = SG_calibration_value[tmc2590->thisMotor][idx] - tmc2590->resp.stallGuardCurrentValue;
                /* find maximum stallGuardDelta over reporting period */
                if (tmc2590->stallGuardDelta < stallGuardDelta) {
                    tmc2590->stallGuardDelta  = stallGuardDelta;}                
            }
        }

    }

#ifdef SG_SKIP_DEBUG_ENABLED
debug_pin_write(0, DEBUG_1_PIN);
#endif
    
}


void process_status_of_single_controller(TMC2590TypeDef *tmc2590){
    
    process_controller_status(tmc2590);

    /* check for all motors standstill state and request if not already applied 
     * idle flag is triggered in TMC after 2**20 cycles of inactivity, i.e. 65.5ms with 16MHz XTAL
     * on top of that whatever delay is applied due to regular data poll (currently every second) */    
    if (st_tmc.current_scale_state != CURRENT_SCALE_STANDSTILL){ /* check only if not already standstill */
        if ((sys.state == STATE_IDLE) || (sys.state & STATE_SLEEP)){ /* check only if system is in idle state */
            uint8_t controller_id;
            TMC2590TypeDef *tmc2590_standstill;
            uint8_t all_motors_standstill = 0;
            /* check idle state of each TMC controller and sum them up */
            for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
                tmc2590_standstill = get_TMC_controller(controller_id);
                all_motors_standstill += ( ( tmc2590_standstill->resp.StatusBits >> 7 ) & 1 ); /* Status bit_7 STST - Idle: */
            }            
            /* if all motors are idle apply current reduction */
            if (all_motors_standstill == TOTAL_TMCS)
            {
                system_set_exec_tmc_command_flag(TMC_STANDSTILL_COMMAND);
            }
        } //if (sys.state & (STATE_IDLE | STATE_SLEEP)){
    } // if (st_tmc.current_scale_state != CURRENT_SCALE_STANDSTILL){
}

void process_status_of_dual_controller(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2){
    
    process_controller_status(tmc2590_1);
    process_controller_status(tmc2590_2);
 
}

/* route single motor write to single or dual write command depend on the motor controller type */
void tmc2590_single_write_route(uint8_t controller_id, uint8_t address){
    TMC2590TypeDef *tmc2590_1, *tmc2590_2;
    if ( (controller_id == TMC_X1) || (controller_id == TMC_Y1) ){
        /* choose second pair and execute dual write */
        tmc2590_1 = get_TMC_controller(controller_id);
        tmc2590_2 = get_TMC_controller(controller_id+1);
        tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, address);

    }
    if ( (controller_id == TMC_X2) || (controller_id == TMC_Y2) ){
        /* choose second pair and execute dual write */
        tmc2590_1 = get_TMC_controller(controller_id-1);
        tmc2590_2 = get_TMC_controller(controller_id);
        tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, address);
    }
    if (controller_id == TMC_Z) {
        /* choose second pair and execute single write */
        tmc2590_1 = get_TMC_controller(controller_id);
        tmc2590_single_writeInt(tmc2590_1, address);
    }

}

/* initialise SPI hardware */
void tmc_hw_init(void){
    spi_hw_init();
    min_step_period_idx_compute(); 
}

/* start SPI transfers flushing the queue */
void tmc_kick_spi_processing(void){
    spi_process_tx_queue(); /* flush the SPI queue starting from next SPI transfer */
}

