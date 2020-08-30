/*
 * TMC2590.c
 
 */

#include "grbl.h"
#include "spi_to_tmc.h"
#include "TMC2590.h"

#include <string.h>

const uint16_t max_step_period_us_to_read_SG[] = { SG_MAX_VALID_PERIOD_X_US, SG_MAX_VALID_PERIOD_Y_US, SG_MAX_VALID_PERIOD_Z_US }; /* for SB2: X motor 23HS22-2804S - 18rpm, Y motor 23HS33-4008S - 18rpm, Z motor 17HS19-2004S1 - 60rpm,   */

void tmc2590_init(TMC2590TypeDef *tmc2590, uint8_t channel, ConfigurationTypeDef *tmc2590_config, const int32_t *registerResetState)
{
  	uint32_t value;    

	tmc2590->config               = tmc2590_config;
	tmc2590->config->channel      = channel;

	for(size_t i = 0; i < TMC2590_REGISTER_COUNT; i++)
	{
		//tmc2590->registerAccess[i]      = tmc2590_defaultRegisterAccess[i];
		tmc2590->registerResetState[i]                          = registerResetState[i];
        tmc2590->config->shadowRegister[ i | TMC2590_WRITE_BIT] = registerResetState[i];
	}
    
    /* initialise registers */

    /* TMC2590_DRVCONF */
	value = tmc2590->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT];
    
	value &= ~TMC2590_SET_RDSEL(-1);                                    /* clear RDSEL bits */
	value |= TMC2590_SET_RDSEL(tmc2590->respIdx);                       /* set rdsel  */
    
    value &= ~TMC2590_SET_VSENSE(-1);                                   /* clear bit */           
    value |= TMC2590_SET_VSENSE(tmc2590->vSense);                       /* clear bit, 0: Full-scale sense resistor voltage is 325mV. */           
    
    value &= ~TMC2590_SET_SHRTSENS(-1);                                 /* clear bit */
    value |= TMC2590_SET_SHRTSENS(tmc2590->overcurrentSense);           /* 0: Low sensitivity 1: High sensitivity. The high-side overcurrent detector can be set to a higher sensitivity by setting this flag. This will allow detection of wrong cabling even with higher resistive motors.*/

    value &= ~TMC2590_SET_TS2G(-1);                                     /* clear bits */
    value |= TMC2590_SET_TS2G(tmc2590->shortDetectionDelay);            /* %00: 3.2us, %01: 1.6us, %10: 1.2us, %11: 0.8us, Short detection delay for high-side and low side detectors. The short detection delay shall cover the bridge switching time. %01 will work for most applications. A higher delay makes detection less sensitive to capacitive load.*/

    value &= ~TMC2590_SET_DISS2G(-1);                                   /* clear bit */
    value |= TMC2590_SET_DISS2G(tmc2590->disableShortToVSprotection);   /* Leave detection enabled for normal use (0). Allows to disable short to VS protection. 0/1 Leave detection enabled for normal use (0).*/

    value &= ~TMC2590_SET_ENS2VS(-1);                                   /* clear bit */
    value |= TMC2590_SET_ENS2VS(tmc2590->EnableProtection);             /* 0: Enable detection for normal use (1). Explicitly enable short to VS and overcurrent protection by setting this bit..*/

	value &= ~TMC2590_SET_SLPL(-1);                                     /* clear bit */
	value |= TMC2590_SET_SLPL(tmc2590->slopeCtrlLow);
	/* hadle MSB */
	value &= ~TMC2590_SET_SLP2(-1);                                     /* clear bit */
	value |= TMC2590_SET_SLP2((tmc2590->slopeCtrlLow)>>2);
                
	value &= ~TMC2590_SET_SLPH(-1);                                     /* clear bit */
	value |= TMC2590_SET_SLPH(tmc2590->slopeCtrlHigh);
	/* handle MSB */
	value &= ~TMC2590_SET_SLP2(-1);                                     /* clear bit */
	value |= TMC2590_SET_SLP2((tmc2590->slopeCtrlHigh)>>2);


    tmc2590->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT] = TMC2590_VALUE(value);
    
    
    /* TMC2590_DRVCTRL */
	value = tmc2590->config->shadowRegister[TMC2590_DRVCTRL | TMC2590_WRITE_BIT];
    
	value &= ~TMC2590_SET_INTERPOL(-1);                         // clear bits
    value |= TMC2590_SET_INTERPOL(tmc2590->interpolationEn);    /* enable interpolation */    
    
    /* 16 microsteps */
	value &= ~TMC2590_SET_MRES(-1);                             // clear MRES bits
	value |= TMC2590_SET_MRES(tmc2590->microSteps);             // set MRES  = 16
    
    tmc2590->config->shadowRegister[TMC2590_DRVCTRL | TMC2590_WRITE_BIT] = TMC2590_VALUE(value);


    /* TMC2590_CHOPCONF */
	value = tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT];    
    //default: 0x00091935,
    value &= ~TMC2590_SET_TOFF(-1);                       // clear
    value |= TMC2590_SET_TOFF(tmc2590->SlowDecayDuration);// Off time/MOSFET disable. Duration of slow decay phase. If TOFF is 0, the MOSFETs are shut off. If TOFF is nonzero, slow decay time is a multiple of system clock periods: NCLK= 24 + (32 x TOFF) (Minimum time is 64clocks.), %0000: Driver disable, all bridges off, %0001: 1 (use with TBL of minimum 24 clocks) %0010 ... %1111: 2 ... 15 */

    value &= ~TMC2590_SET_TBL(-1);                       // clear
    value |= TMC2590_SET_TBL(tmc2590->chopperBlankTime); // Blanking time. Blanking time interval, in system clock periods: %00: 16 %01: 24 %10: 36 %11: 54
	
	value &= ~TMC2590_SET_HSTRT(-1);                       // clear
	value |= TMC2590_SET_HSTRT(tmc2590->HystStart);        /* Hysteresis start value, Hysteresis start offset from HEND: %000: 1 %100: 5; %001: 2 %101: 6; %010: 3 %110: 7; %011: 4 %111: 8; Effective: HEND+HSTRT must be ? 15 */
	
	value &= ~TMC2590_SET_HEND(-1);                        // clear
	value |= TMC2590_SET_HEND(tmc2590->HystEnd);           /* Hysteresis end (low) value; %0000 ... %1111: Hysteresis is -3, -2, -1, 0, 1, ..., 12 (1/512 of this setting adds to current setting) This is the hysteresis value which becomes used for the hysteresis chopper. */
	
	value &= ~TMC2590_SET_HDEC(-1);                        // clear
	value |= TMC2590_SET_HDEC(tmc2590->HystDectrement);    /* Hysteresis decrement period setting, in system clock periods: %00: 16; %01: 32; %10: 48; %11: 64 */
	
	value &= ~TMC2590_SET_RNDTF(-1);                       // clear
	value |= TMC2590_SET_RNDTF(tmc2590->SlowDecayRandom);  /* Enable randomizing the slow decay phase duration: 0: Chopper off time is fixed as set by bits tOFF 1: Random mode, tOFF is random modulated by dNCLK= -12 - +3 clocks */

	value &= ~TMC2590_SET_CHM(-1);                         // clear
	value |= TMC2590_SET_CHM(tmc2590->chopperMode);        // Chopper mode. This mode bit affects the interpretation of the HDEC, HEND, and HSTRT parameters shown below. 0 Standard mode (SpreadCycle)

    tmc2590->config->shadowRegister[TMC2590_CHOPCONF  | TMC2590_WRITE_BIT] = TMC2590_VALUE(value);
    

    /* TMC2590_SMARTEN */
	value = tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT];    
    //default: 0x000A0000,  
    
	value &= ~TMC2590_SET_SEIMIN(-1);                       // clear 
	value |= TMC2590_SET_SEIMIN(tmc2590->currentSEmin);// set 1/4 of full scale
    
    value &= ~TMC2590_SET_SEMIN(-1);                        // clear, 
  	value |= TMC2590_SET_SEMIN(tmc2590->coolStepMin);       // set to trigger if SG below 7x32 = 224

    value &= ~TMC2590_SET_SEMAX(-1);                        // clear, 
  	value |= TMC2590_SET_SEMAX(tmc2590->coolStepMax);       // set to trigger if SG above 7x32 = 224

    tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT] = TMC2590_VALUE(value);
    
    
    /* TMC2590_SGCSCONF */
	value = tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT];    
    //default: 0x000D0505,  

    /* 16 scale of 31 for current */
	value &= ~TMC2590_SET_CS(-1);                           // clear Current scale bits
	value |= TMC2590_SET_CS(tmc2590->currentScale);         // set Current scale  = default
    
    value &= ~TMC2590_SET_SFILT(-1);                        // clear, //0: Standard mode, fastest response time.
  	value |= TMC2590_SET_SFILT(tmc2590->stallGuardFilter);  // 1: Filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy.
    
    value &= ~TMC2590_SET_SGT(-1);                          // clear, 
  	value |= TMC2590_SET_SGT(tmc2590->stallGuardThreshold); // set threshold


    tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT] = TMC2590_VALUE(value);

}


/************************************************ single motor ***********************************************/

void tmc2590_single_writeInt(TMC2590TypeDef *tmc2590_1, uint8_t address)
{
    int32_t controller1_register_value, drvconf_register_value;

    controller1_register_value = tmc2590_1->config->shadowRegister[TMC_ADDRESS(address) | TMC2590_WRITE_BIT];
    drvconf_register_value     = tmc2590_1->config->shadowRegister[TMC2590_DRVCONF      | TMC2590_WRITE_BIT];

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
    
   	drvconf_register_value = tmc2590_1->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT];
	   
	drvconf_register_value &= ~TMC2590_SET_RDSEL(-1);      // clear RDSEL bits
	drvconf_register_value |= TMC2590_SET_RDSEL(rdsel);    // set rdsel
    //nrf_delay_us(delay_us);
    
    tmc2590_1->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT] = drvconf_register_value;

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

    controller1_register_value = tmc2590_1->config->shadowRegister[TMC_ADDRESS(address) | TMC2590_WRITE_BIT];
    controller2_register_value = tmc2590_2->config->shadowRegister[TMC_ADDRESS(address) | TMC2590_WRITE_BIT];
    drvconf_register_value     = tmc2590_1->config->shadowRegister[TMC2590_DRVCONF      | TMC2590_WRITE_BIT];
    
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
    
   	drvconf1_register_value = tmc2590_1->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT];
   	drvconf2_register_value = tmc2590_2->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT];
	            
	drvconf1_register_value &= ~TMC2590_SET_RDSEL(-1);      // clear RDSEL bits
	drvconf1_register_value |= TMC2590_SET_RDSEL(rdsel);    // set rdsel
	drvconf2_register_value &= ~TMC2590_SET_RDSEL(-1);      // clear RDSEL bits
	drvconf2_register_value |= TMC2590_SET_RDSEL(rdsel);    // set rdsel
    
    tmc2590_1->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT] = drvconf1_register_value;
    tmc2590_2->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT] = drvconf2_register_value;

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

stall_guard_tmc_matrix_t SG_calibration_table;


const uint16_t SG_step_periods_us[] PROGMEM =   /* must be size of TMC_SG_PROFILE_POINTS, can be put in PROGMEM  */
{
    3750, /* entry 1, speed=5.0rpm, feed=282.4mm/min */
    3646, /* entry 2, speed=5.1rpm, feed=290.4mm/min */
    3544, /* entry 3, speed=5.3rpm, feed=298.7mm/min */
    3446, /* entry 4, speed=5.4rpm, feed=307.3mm/min */
    3350, /* entry 5, speed=5.6rpm, feed=316.1mm/min */
    3257, /* entry 6, speed=5.8rpm, feed=325.1mm/min */
    3166, /* entry 7, speed=5.9rpm, feed=334.4mm/min */
    3078, /* entry 8, speed=6.1rpm, feed=344.0mm/min */
    2992, /* entry 9, speed=6.3rpm, feed=353.9mm/min */
    2909, /* entry 10, speed=6.4rpm, feed=364.0mm/min */
    2828, /* entry 11, speed=6.6rpm, feed=374.4mm/min */
    2749, /* entry 12, speed=6.8rpm, feed=385.1mm/min */
    2673, /* entry 13, speed=7.0rpm, feed=396.1mm/min */
    2599, /* entry 14, speed=7.2rpm, feed=407.5mm/min */
    2526, /* entry 15, speed=7.4rpm, feed=419.1mm/min */
    2456, /* entry 16, speed=7.6rpm, feed=431.1mm/min */
    2388, /* entry 17, speed=7.9rpm, feed=443.5mm/min */
    2321, /* entry 18, speed=8.1rpm, feed=456.2mm/min */
    2257, /* entry 19, speed=8.3rpm, feed=469.2mm/min */
    2194, /* entry 20, speed=8.5rpm, feed=482.6mm/min */
    2133, /* entry 21, speed=8.8rpm, feed=496.5mm/min */
    2073, /* entry 22, speed=9.0rpm, feed=510.7mm/min */
    2016, /* entry 23, speed=9.3rpm, feed=525.3mm/min */
    1960, /* entry 24, speed=9.6rpm, feed=540.3mm/min */
    1905, /* entry 25, speed=9.8rpm, feed=555.8mm/min */
    1852, /* entry 26, speed=10.1rpm, feed=571.7mm/min */
    1801, /* entry 27, speed=10.4rpm, feed=588.0mm/min */
    1751, /* entry 28, speed=10.7rpm, feed=604.9mm/min */
    1702, /* entry 29, speed=11.0rpm, feed=622.2mm/min */
    1654, /* entry 30, speed=11.3rpm, feed=640.0mm/min */
    1608, /* entry 31, speed=11.7rpm, feed=658.3mm/min */
    1564, /* entry 32, speed=12.0rpm, feed=677.1mm/min */
    1520, /* entry 33, speed=12.3rpm, feed=696.5mm/min */
    1478, /* entry 34, speed=12.7rpm, feed=716.4mm/min */
    1437, /* entry 35, speed=13.1rpm, feed=737.0mm/min */
    1397, /* entry 36, speed=13.4rpm, feed=758.0mm/min */
    1358, /* entry 37, speed=13.8rpm, feed=779.7mm/min */
    1320, /* entry 38, speed=14.2rpm, feed=802.1mm/min */
    1283, /* entry 39, speed=14.6rpm, feed=825.0mm/min */
    1248, /* entry 40, speed=15.0rpm, feed=848.6mm/min */
    1213, /* entry 41, speed=15.5rpm, feed=872.9mm/min */
    1179, /* entry 42, speed=15.9rpm, feed=897.9mm/min */
    1146, /* entry 43, speed=16.4rpm, feed=923.6mm/min */
    1115, /* entry 44, speed=16.8rpm, feed=950.0mm/min */
    1084, /* entry 45, speed=17.3rpm, feed=977.2mm/min */
    1053, /* entry 46, speed=17.8rpm, feed=1005.2mm/min */
    1024, /* entry 47, speed=18.3rpm, feed=1033.9mm/min */
    996, /* entry 48, speed=18.8rpm, feed=1063.5mm/min */
    968, /* entry 49, speed=19.4rpm, feed=1094.0mm/min */
    941, /* entry 50, speed=19.9rpm, feed=1125.3mm/min */
    915, /* entry 51, speed=20.5rpm, feed=1157.5mm/min */
    889, /* entry 52, speed=21.1rpm, feed=1190.6mm/min */
    865, /* entry 53, speed=21.7rpm, feed=1224.7mm/min */
    841, /* entry 54, speed=22.3rpm, feed=1259.7mm/min */
    817, /* entry 55, speed=22.9rpm, feed=1295.8mm/min */
    794, /* entry 56, speed=23.6rpm, feed=1332.9mm/min */
    772, /* entry 57, speed=24.3rpm, feed=1371.0mm/min */
    751, /* entry 58, speed=25.0rpm, feed=1410.2mm/min */
    730, /* entry 59, speed=25.7rpm, feed=1450.6mm/min */
    710, /* entry 60, speed=26.4rpm, feed=1492.1mm/min */
    690, /* entry 61, speed=27.2rpm, feed=1534.8mm/min */
    671, /* entry 62, speed=28.0rpm, feed=1578.7mm/min */
    652, /* entry 63, speed=28.8rpm, feed=1623.9mm/min */
    634, /* entry 64, speed=29.6rpm, feed=1670.4mm/min */
    616, /* entry 65, speed=30.4rpm, feed=1718.2mm/min */
    599, /* entry 66, speed=31.3rpm, feed=1767.4mm/min */
    582, /* entry 67, speed=32.2rpm, feed=1817.9mm/min */
    566, /* entry 68, speed=33.1rpm, feed=1870.0mm/min */
    550, /* entry 69, speed=34.1rpm, feed=1923.5mm/min */
    535, /* entry 70, speed=35.0rpm, feed=1978.5mm/min */
    520, /* entry 71, speed=36.0rpm, feed=2035.2mm/min */
    506, /* entry 72, speed=37.1rpm, feed=2093.4mm/min */
    492, /* entry 73, speed=38.1rpm, feed=2153.3mm/min */
    478, /* entry 74, speed=39.2rpm, feed=2214.9mm/min */
    465, /* entry 75, speed=40.3rpm, feed=2278.3mm/min */
    452, /* entry 76, speed=41.5rpm, feed=2343.5mm/min */
    439, /* entry 77, speed=42.7rpm, feed=2410.6mm/min */
    427, /* entry 78, speed=43.9rpm, feed=2479.6mm/min */
    415, /* entry 79, speed=45.2rpm, feed=2550.5mm/min */
    404, /* entry 80, speed=46.5rpm, feed=2623.5mm/min */
    392, /* entry 81, speed=47.8rpm, feed=2698.6mm/min */
    381, /* entry 82, speed=49.2rpm, feed=2775.8mm/min */
    371, /* entry 83, speed=50.6rpm, feed=2855.3mm/min */
    361, /* entry 84, speed=52.0rpm, feed=2937.0mm/min */
    350, /* entry 85, speed=53.5rpm, feed=3021.0mm/min */
    341, /* entry 86, speed=55.0rpm, feed=3107.5mm/min */
    331, /* entry 87, speed=56.6rpm, feed=3196.4mm/min */
    322, /* entry 88, speed=58.2rpm, feed=3287.9mm/min */
    313, /* entry 89, speed=59.9rpm, feed=3382.0mm/min */
    304, /* entry 90, speed=61.6rpm, feed=3478.8mm/min */
    296, /* entry 91, speed=63.4rpm, feed=3578.4mm/min */
    288, /* entry 92, speed=65.2rpm, feed=3680.8mm/min */
    280, /* entry 93, speed=67.0rpm, feed=3786.1mm/min */
    272, /* entry 94, speed=69.0rpm, feed=3894.5mm/min */
    264, /* entry 95, speed=70.9rpm, feed=4005.9mm/min */
    257, /* entry 96, speed=73.0rpm, feed=4120.6mm/min */
    250, /* entry 97, speed=75.1rpm, feed=4238.5mm/min */
    243, /* entry 98, speed=77.2rpm, feed=4359.8mm/min */
    236, /* entry 99, speed=79.4rpm, feed=4484.5mm/min */
    230, /* entry 100, speed=81.7rpm, feed=4612.9mm/min */
    223, /* entry 101, speed=84.0rpm, feed=4744.9mm/min */
    217, /* entry 102, speed=86.4rpm, feed=4880.7mm/min */
    211, /* entry 103, speed=88.9rpm, feed=5020.4mm/min */
    205, /* entry 104, speed=91.4rpm, feed=5164.0mm/min */
    199, /* entry 105, speed=94.1rpm, feed=5311.8mm/min */
    194, /* entry 106, speed=96.8rpm, feed=5463.9mm/min */
    188, /* entry 107, speed=99.5rpm, feed=5620.2mm/min */
    183, /* entry 108, speed=102.4rpm, feed=5781.1mm/min */
    178, /* entry 109, speed=105.3rpm, feed=5946.5mm/min */
    173, /* entry 110, speed=108.3rpm, feed=6116.7mm/min */
    168, /* entry 111, speed=111.4rpm, feed=6291.7mm/min */
    164, /* entry 112, speed=114.6rpm, feed=6471.8mm/min */
    159, /* entry 113, speed=117.9rpm, feed=6657.0mm/min */
    155, /* entry 114, speed=121.3rpm, feed=6847.5mm/min */
    150, /* entry 115, speed=124.7rpm, feed=7043.5mm/min */
    146, /* entry 116, speed=128.3rpm, feed=7245.1mm/min */
    142, /* entry 117, speed=132.0rpm, feed=7452.4mm/min */
    138, /* entry 118, speed=135.7rpm, feed=7665.7mm/min */
    134, /* entry 119, speed=139.6rpm, feed=7885.1mm/min */
    131, /* entry 120, speed=143.6rpm, feed=8110.7mm/min */
    127, /* entry 121, speed=147.7rpm, feed=8342.8mm/min */
    123, /* entry 122, speed=152.0rpm, feed=8581.6mm/min */
    120, /* entry 123, speed=156.3rpm, feed=8827.2mm/min */
    117, /* entry 124, speed=160.8rpm, feed=9079.8mm/min */
    113, /* entry 125, speed=165.4rpm, feed=9339.7mm/min */
    110, /* entry 126, speed=170.1rpm, feed=9607.0mm/min */
    107, /* entry 127, speed=175.0rpm, feed=9881.9mm/min */
    104, /* entry 128, speed=180.0rpm, feed=10164.7mm/min */
};           


/* todo rewrite search algo to depend on last value - should be very close and hence search will be faster */
/* profler: RAM: 300us per search;  PROGMEM: 350us per search*/ 
void tmc_store_calibration_point(	uint8_t thisMotor, uint8_t thisAxis, uint16_t stallGuardCurrentValue){      
    
#ifdef SG_CAL_DEBUG_ENABLED
debug_pin_write(1, DEBUG_1_PIN);
#endif    
    /* find entry in calibration table based on step_period_us and add it */
    uint8_t idx;
    for (idx=0; idx<TMC_SG_PROFILE_POINTS; idx++){
        //if ( st_tmc.step_period_us[thisAxis] > SG_step_periods_us[idx] ){ 
        if ( st_tmc.step_period_us[thisAxis] > pgm_read_word_near(SG_step_periods_us + idx) ){ //if storing in PROGMEM then use this: if ( st_tmc.step_period_us[thisAxis] > pgm_read_word_near(SG_step_periods_us + idx) ){
            if (SG_calibration_table.SG_read_cnt[thisMotor][idx] < TMC_SG_MAX_AVERAGE) {
                SG_calibration_table.SG_read[thisMotor][idx] += stallGuardCurrentValue;
                SG_calibration_table.SG_read_cnt[thisMotor][idx] ++;
            }                
            break; /* for loop */            
        }
    } //for (idx=0; idx<TMC_SG_PROFILE_POINTS; idx++){
#ifdef SG_CAL_DEBUG_ENABLED
debug_pin_write(0, DEBUG_1_PIN);
#endif
    
}

/* clear calibration matrix and get ready for data collection */
void tmc_calibration_init(void){
    st_tmc.calibration_enabled = 1;
    st_tmc.stall_alarm_enabled  = false;                  /* global holding desired stall behaviour: if "true" then stall guard value below the limit will trigger alarm      */    
    memset(&SG_calibration_table, 0, sizeof(stall_guard_tmc_matrix_t));
}

/* stop calibration and compute coefficients based on accumulated data */
void tmc_compute_and_apply_calibration(void){
    
    st_tmc.calibration_enabled = 0;
    
    /* loop over the whole table and calculate average */    
    /* cycle through every motors */
    uint8_t controller_id;
    uint8_t idx;
    uint16_t last_SG_read = 0;
    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
        /* first apply averaging and fill 0 entires with lower values */             
        for (idx=0; idx<TMC_SG_PROFILE_POINTS; idx++){
            if ( SG_calibration_table.SG_read_cnt[controller_id][idx] > 0 ) { /* catch divide by 0 */
                SG_calibration_table.SG_read[controller_id][idx] /= SG_calibration_table.SG_read_cnt[controller_id][idx]; /* find averge SG value */
                last_SG_read = SG_calibration_table.SG_read[controller_id][idx]; /* keep last entry in cache in case next entry is empty */
            }
            else{ /* if empty entry use the last filled one */
                SG_calibration_table.SG_read[controller_id][idx] = last_SG_read;
            }            
        } 
        /* finally sweep down and fill 0 entires with upper values */
        for (idx=0; idx<TMC_SG_PROFILE_POINTS; idx++){
            if ( SG_calibration_table.SG_read_cnt[controller_id][TMC_SG_PROFILE_POINTS-1-idx] == 0 ) { /* catch empty entries */
                SG_calibration_table.SG_read[controller_id][TMC_SG_PROFILE_POINTS-1-idx] = last_SG_read;
            }
            else{
                last_SG_read = SG_calibration_table.SG_read[controller_id][TMC_SG_PROFILE_POINTS-1-idx];
            }
        }        
    }             
    /*reenable alarm */
    st_tmc.stall_alarm_enabled  = true;                  /* global holding desired stall behaviour: if "true" then stall guard value below the limit will trigger alarm      */
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
    	    printInteger( SG_calibration_table.SG_read[controller_id][idx] );
        }
        printPgmString(PSTR(">\n"));
    }

}

void stall_guard_calibration_load(void){
    
    /* init default calibration values */
    uint8_t controller_id;
    uint8_t idx;
    TMC2590TypeDef *tmc2590;
    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
        tmc2590 = get_TMC_controller(controller_id);
        for (idx=0; idx<TMC_SG_PROFILE_POINTS; idx++){
            SG_calibration_table.SG_read[controller_id][idx] = tmc2590->stallGuardAlarmValue + tmc2590->stallGuardAlarmThreshold;
        }
    }    
}



/************************************************ all motors ***********************************************/



void process_controller_status(TMC2590TypeDef *tmc2590){

#ifdef SG_SKIP_DEBUG_ENABLED
debug_pin_write(1, DEBUG_1_PIN);
#endif
    
    /* TMC2590_RESPONSE0 #define TMC2590_GET_MSTEP(X)  (0x3FF & ((X) >> 10)) */     
    tmc2590->resp.mStepCurrentValue = TMC2590_GET_MSTEP(tmc2590->config->shadowRegister[TMC2590_RESPONSE0]) & 0x1FF; /* bit 9 is polarity bit, ignore it*/
    tmc2590->resp.stallGuardCurrentValue = TMC2590_GET_SG(tmc2590->config->shadowRegister[TMC2590_RESPONSE1]);

    /* TMC2590_RESPONSE2 #define TMC2590_GET_SGU(X)    (0x1F & ((X) >> 15)) #define TMC2590_GET_SE(X)     (0x1F & ((X) >> 10))    */
    tmc2590->resp.stallGuardShortValue= TMC2590_GET_SGU(tmc2590->config->shadowRegister[TMC2590_RESPONSE2]);
    tmc2590->resp.coolStepCurrentValue= TMC2590_GET_SE(tmc2590->config->shadowRegister[TMC2590_RESPONSE2]);

    /* TMC2590_RESPONSE3 status and diagnostic */
    tmc2590->resp.StatusBits = tmc2590->config->shadowRegister[TMC2590_RESPONSE_LATEST] & 0xFF;
    tmc2590->resp.DiagnosticBits = (tmc2590->config->shadowRegister[TMC2590_RESPONSE_LATEST] & 0xFFC00) >> 10 ;

    if ( ( st_tmc.calibration_enabled ) && ( st_tmc.current_scale_state == CURRENT_SCALE_ACTIVE ) ){
        if ( st_tmc.SG_skips_counter[tmc2590->thisAxis] >= SG_READING_SKIPS_AFTER_SLOW_FEED ){
            /* if feed is fast and SG_skips_counter is higher than min then store calibration value */
            if ( st_tmc.SG_skips_counter[tmc2590->thisAxis] >= SG_READING_SKIPS_AFTER_SLOW_FEED ){
                tmc_store_calibration_point(tmc2590->thisMotor, tmc2590->thisAxis, tmc2590->resp.stallGuardCurrentValue);
            }
        }                   
    }        

    if ( ( st_tmc.stall_alarm_enabled ) && ( st_tmc.current_scale_state == CURRENT_SCALE_ACTIVE ) ){
        
        /* start reading SG if rotational speed is sufficiently high */                       
        /* feed speed validation. If feed was slow then SG_skips_counter gets reset, then decrement till reaches 0, only after that SG analysis for stall detection is allowed */
        if ( st_tmc.step_period_us[tmc2590->thisAxis] < max_step_period_us_to_read_SG[tmc2590->thisAxis] ) {  /* check stall only if feed is higher than defined for this motor */
                    
            /* feed is fast, increment SG_skips_counter until 0 then analyse SG for stall */
            if ( st_tmc.SG_skips_counter[tmc2590->thisAxis] >= SG_READING_SKIPS_AFTER_SLOW_FEED )
            {
                /* feed rate is high enough and started more than SG_READING_SKIPS_AFTER_SLOW_FEED ago, lets analyse the stall */
                
                /* init stallGuardAlarmValue with entry from max speed */
                uint16_t stallGuardAlarmValue = SG_calibration_table.SG_read[tmc2590->thisMotor][TMC_SG_PROFILE_POINTS-1] - tmc2590->stallGuardAlarmThreshold;
                
                /* find alarm value based on calibration matrix and predefined threshold */
                #ifdef SG_CAL_DEBUG_ENABLED
                debug_pin_write(1, DEBUG_1_PIN);
                #endif    
                    /* find entry in calibration table based on step_period_us and extract SG calibrated level from there */
                    uint8_t idx;
                    for (idx=0; idx<TMC_SG_PROFILE_POINTS; idx++){
                        if ( st_tmc.step_period_us[tmc2590->thisAxis] > pgm_read_word_near(SG_step_periods_us + idx) ){ //if storing in PROGMEM then use this: if ( st_tmc.step_period_us[thisAxis] > pgm_read_word_near(SG_step_periods_us + idx) ){
                            /* entry found, apply threshold */
                            if (SG_calibration_table.SG_read[tmc2590->thisMotor][idx] > tmc2590->stallGuardAlarmThreshold){
                                stallGuardAlarmValue = SG_calibration_table.SG_read[tmc2590->thisMotor][idx] - tmc2590->stallGuardAlarmThreshold ;                                
                            }                                    
                        }                
                        break; /* for loop */                                    
                    } //for (idx=0; idx<TMC_SG_PROFILE_POINTS; idx++){
                #ifdef SG_CAL_DEBUG_ENABLED
                debug_pin_write(0, DEBUG_1_PIN);
                #endif

                
                
                if (tmc2590->resp.stallGuardCurrentValue < tmc2590->resp.stallGuardMinValue) {
                tmc2590->resp.stallGuardMinValue    = tmc2590->resp.stallGuardCurrentValue;}
                if (tmc2590->resp.stallGuardCurrentValue    < stallGuardAlarmValue) {
                    /* trigger alarm */
                    tmc_trigger_stall_alarm(tmc2590->thisAxis);
                    /* reset SG period to max as alarm will immediately stop the stepper and period will remain as it was at the point of trigger */
                    st_tmc.step_period_us[tmc2590->thisAxis] = 0xFFFF;
                    printPgmString(PSTR("\nSG ALARM, motor "));
                    printInteger( tmc2590->thisMotor);
                    printPgmString(PSTR(", SG: "));
                    printInteger( tmc2590->resp.stallGuardCurrentValue);
                    printPgmString(PSTR("\n"));
                } //if (tmc2590->resp.stallGuardCurrentValue    < tmc2590->stallGuardAlarmValue) {
   
            } //if ( st_tmc.SG_skips_counter[tmc2590->thisAxis] >= SG_READING_SKIPS_AFTER_SLOW_FEED )
                
        } // if ( st_tmc.step_period_us[tmc2590->thisAxis] < max_step_period_us_to_read_SG[tmc2590->thisAxis] ) {  /* check stall only if feed is higher than defined for this motor */
        else {
                /* feed is slow, reset SG_skips_counter */
                st_tmc.SG_skips_counter[tmc2590->thisAxis] = 0;
        }
            
    } //else if (st_tmc.stall_alarm_enabled){
    else{
        if (tmc2590->resp.stallGuardCurrentValue < tmc2590->resp.stallGuardMinValue) {
        tmc2590->resp.stallGuardMinValue    = tmc2590->resp.stallGuardCurrentValue;}
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
}

/* start SPI transfers flushing the queue */
void tmc_kick_spi_processing(void){
    spi_process_tx_queue(); /* flush the SPI queue starting from next SPI transfer */
}

