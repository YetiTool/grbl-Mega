/*
 * TMC2590.c
 
 */

#include "grbl.h"
#include "spi_to_tmc.h"

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

    if ( ( st_tmc.stall_alarm_enabled ) && ( st_tmc.current_scale_state == CURRENT_SCALE_ACTIVE ) ){
        
        /* start reading SG if rotational speed is sufficiently high */                       
        /* feed speed validation. If feed was slow then SG_skips_counter gets reset, then decrement till reaches 0, only after that SG analysis for stall detection is allowed */
        if ( st_tmc.step_period_us[tmc2590->thisAxis] < max_step_period_us_to_read_SG[tmc2590->thisAxis] ) {  /* check stall only if feed is higher than defined for this motor */
                    
            /* feed is fast, increment SG_skips_counter until 0 then analyse SG for stall */
            if ( st_tmc.SG_skips_counter[tmc2590->thisAxis] >= SG_READING_SKIPS_AFTER_SLOW_FEED )
            {
                /* feed rate is high enough and started more than SG_READING_SKIPS_AFTER_SLOW_FEED ago, lets analyse the stall */
                if (tmc2590->resp.stallGuardCurrentValue < tmc2590->resp.stallGuardMinValue) {
                tmc2590->resp.stallGuardMinValue    = tmc2590->resp.stallGuardCurrentValue;}
                if (tmc2590->resp.stallGuardCurrentValue    < tmc2590->stallGuardAlarmValue) {
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

