/*
 * TMC2590.c
 
 */

#include "grbl.h"
#include "TMC2590.h"
#include "spi_to_tmc.h"
#include <string.h>

static void readWrite(TMC2590TypeDef *tmc2590, uint32_t value);

uint8_t current_scale_state = CURRENT_SCALE_ACTIVE; /* global holding effective current scale */

/* declare structures for all 5 motors */
TMC2590TypeDef tmc2590_X1, tmc2590_X2, tmc2590_Y1, tmc2590_Y2, tmc2590_Z;
ConfigurationTypeDef tmc2590_config_X1, tmc2590_config_X2, tmc2590_config_Y1, tmc2590_config_Y2, tmc2590_config_Z;

static void readWrite(TMC2590TypeDef *tmc2590, uint32_t value)
{	// sending data (value) via spi to TMC262, coping written and received data to shadow register
	static uint8_t rdsel = 0; // number of expected read response

	uint8_t data[] = { BYTE(value, 2), BYTE(value, 1), BYTE(value, 0) };

	//tmc2590_readWriteArray(tmc2590->config->channel, &data[0], 3);

	tmc2590->config->shadowRegister[rdsel] = _8_32(data[0], data[1], data[2], 0) >> 12;
	tmc2590->config->shadowRegister[TMC2590_RESPONSE_LATEST] = tmc2590->config->shadowRegister[rdsel];

// set virtual read address for next reply given by RDSEL, can only change by setting RDSEL in DRVCONF
	if(TMC2590_GET_ADDRESS(value) == TMC2590_DRVCONF)
		rdsel = TMC2590_GET_RDSEL(value);

// write store written value to shadow register
	tmc2590->config->shadowRegister[TMC2590_GET_ADDRESS(value)] = value;
}


void tmc2590_writeInt(TMC2590TypeDef *tmc2590, uint8_t address, int32_t value)
{
	value = TMC2590_VALUE(value);
	tmc2590->config->shadowRegister[TMC_ADDRESS(address) | TMC2590_WRITE_BIT] = value;
	readWrite(tmc2590, value);
}

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
    
    value &= ~TMC2590_SET_VSENSE(-1);                       /* clear bit */           
    value |= TMC2590_SET_VSENSE(tmc2590->vSense);          /* clear bit, 0: Full-scale sense resistor voltage is 325mV. */           
    
    value &= ~TMC2590_SET_SHRTSENS(-1);                       /* clear bit */
    value |= TMC2590_SET_SHRTSENS(tmc2590->overcurrentSense);          /* 0: Low sensitivity 1: High sensitivity. The high-side overcurrent detector can be set to a higher sensitivity by setting this flag. This will allow detection of wrong cabling even with higher resistive motors.*/

    value &= ~TMC2590_SET_TS2G(-1);                       /* clear bits */
    value |= TMC2590_SET_TS2G(tmc2590->shortDetectionDelay);          /* %00: 3.2us, %01: 1.6us, %10: 1.2us, %11: 0.8us, Short detection delay for high-side and low side detectors. The short detection delay shall cover the bridge switching time. %01 will work for most applications. A higher delay makes detection less sensitive to capacitive load.*/

    value &= ~TMC2590_SET_DISS2G(-1);                       /* clear bit */
    value |= TMC2590_SET_DISS2G(tmc2590->disableShortToVSprotection);          /* Leave detection enabled for normal use (0). Allows to disable short to VS protection. 0/1 Leave detection enabled for normal use (0).*/

    value &= ~TMC2590_SET_ENS2VS(-1);                       /* clear bit */
    value |= TMC2590_SET_ENS2VS(tmc2590->EnableProtection);          /* 0: Enable detection for normal use (1). Explicitly enable short to VS and overcurrent protection by setting this bit..*/

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
    value |= TMC2590_SET_TOFF(tmc2590->SlowDecayDuration);// Off time/MOSFET disable. Duration of slow decay phase. If TOFF is 0, the MOSFETs are shut off. If TOFF is nonzero, slow decay time is a multiple of system clock periods: NCLK= 24 + (32 x TOFF) (Minimum time is 64clocks.), %0000: Driver disable, all bridges off, %0001: 1 (use with TBL of minimum 24 clocks) %0010 … %1111: 2 … 15 */

    value &= ~TMC2590_SET_TBL(-1);                       // clear
    value |= TMC2590_SET_TBL(tmc2590->chopperBlankTime); // Blanking time. Blanking time interval, in system clock periods: %00: 16 %01: 24 %10: 36 %11: 54


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

uint8_t tmc2590_reset(TMC2590TypeDef *tmc2590)
{
	tmc2590_writeInt(tmc2590, TMC2590_DRVCONF,  tmc2590->registerResetState[TMC2590_DRVCONF]);
	tmc2590_writeInt(tmc2590, TMC2590_DRVCTRL,  tmc2590->registerResetState[TMC2590_DRVCTRL]);
	tmc2590_writeInt(tmc2590, TMC2590_CHOPCONF, tmc2590->registerResetState[TMC2590_CHOPCONF]);
	tmc2590_writeInt(tmc2590, TMC2590_SMARTEN,  tmc2590->registerResetState[TMC2590_SMARTEN]);
	tmc2590_writeInt(tmc2590, TMC2590_SGCSCONF, tmc2590->registerResetState[TMC2590_SGCSCONF]);

	return 1;
}

uint8_t tmc2590_restore(TMC2590TypeDef *tmc2590)
{
	tmc2590_writeInt(tmc2590, TMC2590_DRVCONF,  tmc2590->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT]);
	tmc2590_writeInt(tmc2590, TMC2590_DRVCTRL,  tmc2590->config->shadowRegister[TMC2590_DRVCTRL | TMC2590_WRITE_BIT]);
	tmc2590_writeInt(tmc2590, TMC2590_CHOPCONF, tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT]);
	tmc2590_writeInt(tmc2590, TMC2590_SMARTEN,  tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT]);
	tmc2590_writeInt(tmc2590, TMC2590_SGCSCONF, tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT]);

	return 1;
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
    tmc2590_read_single(tmc2590, 0); /* ignore this response*/
    tmc2590_read_single(tmc2590, 1); /* response 0 */
    tmc2590_read_single(tmc2590, 2); /* response 1 */
    tmc2590_read_single(tmc2590, 3); /* response 2 */
    tmc2590_read_single(tmc2590, 0); /* response 3 */    
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
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, 0); /* ignore this response*/
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, 1); /* response 0 */
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, 2); /* response 1 */
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, 3); /* response 2 */
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, 0); /* response 3 */

}


/************************************************ all motors ***********************************************/

/* schedule periodic read of all values */
void tmc2590_schedule_read_all(void){
    tmc2590_dual_read_all(&tmc2590_X1, &tmc2590_X2);
    tmc2590_dual_read_all(&tmc2590_Y1, &tmc2590_Y2);
    tmc2590_single_read_all(&tmc2590_Z);
}

void process_status_of_single_controller(TMC2590TypeDef *tmc2590){
    /* TMC2590_RESPONSE0 #define TMC2590_GET_MSTEP(X)  (0x3FF & ((X) >> 10)) */     
    tmc2590->resp.mStepCurrenValue = TMC2590_GET_MSTEP(tmc2590->config->shadowRegister[TMC2590_RESPONSE0]) & 0x1FF; /* bit 9 is polarity bit, ignore it*/
    
    /* TMC2590_RESPONSE1 #define TMC2590_GET_SG(X)     (0x3FF & ((X) >> 10)) */
    tmc2590->resp.stallGuardCurrenValue = TMC2590_GET_SG(tmc2590->config->shadowRegister[TMC2590_RESPONSE1]);  
    if (tmc2590->resp.stallGuardCurrenValue < tmc2590->resp.stallGuardMinValue) {
        tmc2590->resp.stallGuardMinValue    = tmc2590->resp.stallGuardCurrenValue;}
    if (tmc2590->resp.stallGuardMinValue    < tmc2590->stallGuardAlarmValue) {
        /* trigger alarm */
        //printPgmString(PSTR("\n!!! SG ALARM !!!\n"));
        //printPgmString(PSTR("--\n"));
    }

    /* TMC2590_RESPONSE2 #define TMC2590_GET_SGU(X)    (0x1F & ((X) >> 15)) #define TMC2590_GET_SE(X)     (0x1F & ((X) >> 10))    */
    tmc2590->resp.stallGuardShortValue= TMC2590_GET_SGU(tmc2590->config->shadowRegister[TMC2590_RESPONSE2]);  
    tmc2590->resp.coolStepCurrenValue= TMC2590_GET_SE(tmc2590->config->shadowRegister[TMC2590_RESPONSE2]);      

    /* TMC2590_RESPONSE3 status and diagnostic */
    tmc2590->resp.StatusBits = tmc2590->config->shadowRegister[TMC2590_RESPONSE_LATEST] & 0xFF;   
    tmc2590->resp.DiagnosticBits = (tmc2590->config->shadowRegister[TMC2590_RESPONSE_LATEST] & 0xFFC00) >> 10 ;      
}

void process_status_of_dual_controller(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2){
    /* TMC2590_RESPONSE0 #define TMC2590_GET_MSTEP(X)  (0x3FF & ((X) >> 10)) */     
    tmc2590_1->resp.mStepCurrenValue = TMC2590_GET_MSTEP(tmc2590_1->config->shadowRegister[TMC2590_RESPONSE0]) & 0x1FF; /* bit 9 is polarity bit, ignore it*/
    tmc2590_2->resp.mStepCurrenValue = TMC2590_GET_MSTEP(tmc2590_2->config->shadowRegister[TMC2590_RESPONSE0]) & 0x1FF; /* bit 9 is polarity bit, ignore it*/
    
    /* TMC2590_RESPONSE1 #define TMC2590_GET_SG(X)     (0x3FF & ((X) >> 10)) */
    tmc2590_1->resp.stallGuardCurrenValue = TMC2590_GET_SG(tmc2590_1->config->shadowRegister[TMC2590_RESPONSE1]);  
    tmc2590_2->resp.stallGuardCurrenValue = TMC2590_GET_SG(tmc2590_2->config->shadowRegister[TMC2590_RESPONSE1]);  
    
    /*if motor is active, update statistics and trigger alarm if needed */
    if (current_scale_state == CURRENT_SCALE_ACTIVE){

        if (tmc2590_1->resp.stallGuardCurrenValue < tmc2590_1->resp.stallGuardMinValue) {
            tmc2590_1->resp.stallGuardMinValue    = tmc2590_1->resp.stallGuardCurrenValue;}
        if (tmc2590_1->resp.stallGuardMinValue    < tmc2590_1->stallGuardAlarmValue) {
            /* trigger alarm */
            //printPgmString(PSTR("\n!!! SG ALARM !!!\n"));
            printInteger( tmc2590_1->thisMotor);
            printPgmString(PSTR("--\n"));
        }        

        if (tmc2590_2->resp.stallGuardCurrenValue  < tmc2590_2->resp.stallGuardMinValue) {
            tmc2590_2->resp.stallGuardMinValue     = tmc2590_2->resp.stallGuardCurrenValue;}
        if (tmc2590_2->resp.stallGuardMinValue     < tmc2590_2->stallGuardAlarmValue) {
            //printPgmString(PSTR("\n!!! SG ALARM !!!\n"));
            printInteger( tmc2590_1->thisMotor);
            printPgmString(PSTR("--\n"));
            /* execute alarm by writing 1 to the limit pin, which will trigger the ISR routine */
            //mc_reset(); // Initiate system kill.
            //system_set_exec_alarm(EXEC_ALARM_HARD_LIMIT); // Indicate hard limit critical event
        }        
        
    } //if (current_scale_state == CURRENT_SCALE_ACTIVE){
    

    /* TMC2590_RESPONSE2 #define TMC2590_GET_SGU(X)    (0x1F & ((X) >> 15)) #define TMC2590_GET_SE(X)     (0x1F & ((X) >> 10))    */
    tmc2590_1->resp.stallGuardShortValue= TMC2590_GET_SGU(tmc2590_1->config->shadowRegister[TMC2590_RESPONSE2]);  
    tmc2590_1->resp.coolStepCurrenValue= TMC2590_GET_SE(tmc2590_1->config->shadowRegister[TMC2590_RESPONSE2]);      
    tmc2590_2->resp.stallGuardShortValue= TMC2590_GET_SGU(tmc2590_2->config->shadowRegister[TMC2590_RESPONSE2]);  
    tmc2590_2->resp.coolStepCurrenValue= TMC2590_GET_SE(tmc2590_2->config->shadowRegister[TMC2590_RESPONSE2]);      

    /* TMC2590_RESPONSE3 status and diagnostic */
    tmc2590_1->resp.StatusBits = tmc2590_1->config->shadowRegister[TMC2590_RESPONSE_LATEST] & 0xFF;   
    tmc2590_1->resp.DiagnosticBits = (tmc2590_1->config->shadowRegister[TMC2590_RESPONSE_LATEST] & 0xFFC00) >> 10 ;  
    tmc2590_2->resp.StatusBits = tmc2590_2->config->shadowRegister[TMC2590_RESPONSE_LATEST] & 0xFF;   
    tmc2590_2->resp.DiagnosticBits = (tmc2590_2->config->shadowRegister[TMC2590_RESPONSE_LATEST] & 0xFFC00) >> 10 ;
}

/* statistics is collected for the whole period between consecutive UART polls so that lost step is not missed between */
/* reset the statistics on all motors */
void stall_guard_statistics_reset(void ){
    uint8_t controller_id;
    TMC2590TypeDef *tmc2590;
    /* reset SG min value in each TMC controller */
    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
        tmc2590 = get_TMC_controller(controller_id);
        tmc2590->resp.stallGuardMinValue = 1023;        
    }
}





void process_status_of_all_controllers(void){
    /* process all responses and update the current status of controller's parameters */
    process_status_of_dual_controller(&tmc2590_X1, &tmc2590_X2);
    process_status_of_dual_controller(&tmc2590_Y1, &tmc2590_Y2);
    process_status_of_single_controller(&tmc2590_Z);    
}


/* get pointer to required contoller */
TMC2590TypeDef * get_TMC_controller(uint8_t controller){
    switch (controller){
        
        case TMC_X1:
            return &tmc2590_X1;            
        case TMC_X2:
            return &tmc2590_X2;            
        case TMC_Y1:
            return &tmc2590_Y1;            
        case TMC_Y2:
            return &tmc2590_Y2;            
        case TMC_Z:
            return &tmc2590_Z;
        
        default:
            break;                             
    } //switch (controller){
    return &tmc2590_X1; /* return first controller in case of a wrong parameter supplied */
}


void init_TMC(void){

	/* init TMC */
    uint8_t channel_X = SPI_CS_X_PIN;
    uint8_t channel_Y = SPI_CS_Y_PIN;
    uint8_t channel_Z = SPI_CS_Z_PIN;

	tmc2590_X1.interpolationEn              = 1;
	tmc2590_X1.microSteps                   = 4; /* 4 : set MRES  = 16*/
	tmc2590_X1.currentScale                 = 31; /* 0 - 31 where 31 is max */
	tmc2590_X1.stallGuardFilter             = 1; // 1: Filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy.
	tmc2590_X1.stallGuardThreshold          = 6; 
	tmc2590_X1.stallGuardAlarmValue         = 100; /* when current SG reading is lower than this value corresponded axis alarm will be triggered */
	tmc2590_X1.vSense                       = 0; /* 0: Full-scale sense resistor voltage is 325mV. */
	tmc2590_X1.currentSEmin                 = 1; // 1: set 1/4 of full scale when CoolStep is active
	tmc2590_X1.coolStepMin                  = 0; // default CoolStep = 0 (disable); if want to enable then set for example to trigger if SG below 7x32 = 224
	tmc2590_X1.coolStepMax                  = 1; // set to trigger if SG above (7+1)8x32 = 256
    tmc2590_X1.respIdx                      = 0; // very first resp index would be 0
    tmc2590_X1.SlowDecayDuration            = 5; // Off time/MOSFET disable. Duration of slow decay phase. If TOFF is 0, the MOSFETs are shut off. If TOFF is nonzero, slow decay time is a multiple of system clock periods: NCLK= 24 + (32 x TOFF) (Minimum time is 64clocks.), %0000: Driver disable, all bridges off, %0001: 1 (use with TBL of minimum 24 clocks) %0010 … %1111: 2 … 15 */
    tmc2590_X1.chopperBlankTime             = 2; // Blanking time. Blanking time interval, in system clock periods: %00: 16 %01: 24 %10: 36 %11: 54
	tmc2590_X1.standStillCurrentScale       = 15; // 15: set 1/2 of full scale, 1/4th of power

    /* control protection */
    tmc2590_X1.overcurrentSense             = 0; //0/1 0: Low sensitivity 1: High sensitivity. The high-side overcurrent detector can be set to a higher sensitivity by setting this flag. This will allow detection of wrong cabling even with higher resistive motors.
    tmc2590_X1.shortDetectionDelay          = 0; //0/1 %00: 3.2us, %01: 1.6us, %10: 1.2us, %11: 0.8us, Short detection delay for high-side and low side detectors. The short detection delay shall cover the bridge switching time. %01 will work for most applications. A higher delay makes detection less sensitive to capacitive load.
    tmc2590_X1.disableShortToVSprotection   = 0; //0/1 Leave detection enabled for normal use (0). Allows to disable short to VS protection. 0/1 Leave detection enabled for normal use (0).
    tmc2590_X1.EnableProtection             = 1; //0/1 Enable detection for normal use (1). Explicitly enable short to VS and overcurrent protection by setting this bit.

	memcpy(&tmc2590_X2, &tmc2590_X1, sizeof(tmc2590_X1));
	memcpy(&tmc2590_Y1, &tmc2590_X1, sizeof(tmc2590_X1));
	memcpy(&tmc2590_Y2, &tmc2590_X1, sizeof(tmc2590_X1));
	memcpy(&tmc2590_Z,  &tmc2590_X1, sizeof(tmc2590_X1));
    
    /* individual motor settings */
    tmc2590_X1.thisMotor                    = TMC_X1;
    tmc2590_X2.thisMotor                    = TMC_X2;
    tmc2590_Y1.thisMotor                    = TMC_Y1;
    tmc2590_Y2.thisMotor                    = TMC_Y2;
    tmc2590_Z.thisMotor                     = TMC_Z ;
    
	tmc2590_Y1.standStillCurrentScale       = 30; // 30: set 30/31 of full scale, 90% of power; this is required for Y motor to prevent operator from accidentally knock the X beam off the position
	tmc2590_Y2.standStillCurrentScale       = 30; // 30: set 30/31 of full scale, 90% of power; this is required for Y motor to prevent operator from accidentally knock the X beam off the position
    
	tmc2590_X1.stallGuardThreshold          = 7;
	tmc2590_X1.stallGuardAlarmValue         = 300; 
	tmc2590_X2.stallGuardThreshold          = 7;
	tmc2590_X2.stallGuardAlarmValue         = 300; 
    tmc2590_Y1.stallGuardThreshold          = 6;
	tmc2590_Y1.stallGuardAlarmValue         = 300; 
    tmc2590_Y2.stallGuardThreshold          = 6;
	tmc2590_Y2.stallGuardAlarmValue         = 300; 
    tmc2590_Z.stallGuardThreshold           = 6;
	tmc2590_Z.stallGuardAlarmValue          = 200; 

    
    stall_guard_statistics_reset();    
    
    spi_hw_init();

	/* initialise wanted variables */
	tmc2590_init(&tmc2590_X1, channel_X, &tmc2590_config_X1, tmc2590_defaultRegisterResetState);
	tmc2590_init(&tmc2590_X2, channel_X, &tmc2590_config_X2, tmc2590_defaultRegisterResetState);
	tmc2590_init(&tmc2590_Y1, channel_Y, &tmc2590_config_Y1, tmc2590_defaultRegisterResetState);
	tmc2590_init(&tmc2590_Y2, channel_Y, &tmc2590_config_Y2, tmc2590_defaultRegisterResetState);
	tmc2590_init(&tmc2590_Z,  channel_Z, &tmc2590_config_Z, tmc2590_defaultRegisterResetState);

	/* initialise motors with wanted parameters */
    tmc2590_dual_restore(&tmc2590_X1, &tmc2590_X2);
    tmc2590_dual_restore(&tmc2590_Y1, &tmc2590_Y2);
	tmc2590_single_restore(&tmc2590_Z);
    

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


/* fetch TMC host command from rtl serial buffer and execute */
void execute_TMC_command(){
    /* fetch 3 bytes from the buffer, calculate CRC and execute*/
    uint8_t tmc_command[RTL_TMC_COMMAND_SIZE], idx;

	/* check if data is available from rtl_serial bufer */
	uint8_t rtl_data_available = serial_rtl_data_available();
	
	if (rtl_data_available != SERIAL_NO_DATA) {

		for (idx = 0; idx < RTL_TMC_COMMAND_SIZE; idx++){
			tmc_command[idx] = serial_read_rtl();
		}
	
		/* check if more data is available from rtl_serial buffer */
		rtl_data_available = serial_rtl_data_available();
		if (rtl_data_available != SERIAL_NO_DATA) {
			/* schedule next TMC execute: indicate to main loop that there is a TMC command to process */
			system_set_exec_tmc_command_flag(RTL_TMC_COMMAND);		
		};


		/* calculate CRC 8 on the command and value and compare with checksum */
		uint8_t crc_in;
		crc_in = crc8x_fast(0, tmc_command, 2);
		if (crc_in == tmc_command[RTL_TMC_COMMAND_SIZE-1]){
			TMC2590TypeDef *tmc2590;
			uint32_t register_value;
			uint8_t controller_id = (tmc_command[0] & ~MOTOR_OFFSET_MASK) >> TMC_COMMAND_BIT_SIZE;
			uint8_t command = tmc_command[0] & MOTOR_OFFSET_MASK;
			uint8_t value = tmc_command[1];
			if (controller_id < TOTAL_TMCS){
				tmc2590 = get_TMC_controller(controller_id);
			}
			else{
				report_status_message(ASMCNC_STATUS_INVALID_STATEMENT);
				return;
			}

			switch (command){

			/* Microstep resolution for STEP/DIR mode. Microsteps per fullstep: %0000: 256; %0001: 128; %0010: 64; %0011: 32; %0100: 16; %0101: 8; %0110: 4; %0111: 2 (halfstep); %1000: 1 (fullstep) */
			case SET_MRES:
				/* TMC2590_DRVCTRL */
				register_value = tmc2590->config->shadowRegister[TMC2590_DRVCTRL | TMC2590_WRITE_BIT];
				tmc2590->microSteps = value;
				register_value &= ~TMC2590_SET_MRES(-1);                        // clear
				register_value |= TMC2590_SET_MRES(tmc2590->microSteps);  // Microstep resolution for STEP/DIR mode. Microsteps per fullstep: %0000: 256; %0001: 128; %0010: 64; %0011: 32; %0100: 16; %0101: 8; %0110: 4; %0111: 2 (halfstep); %1000: 1 (fullstep)
				tmc2590->config->shadowRegister[TMC2590_DRVCTRL | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_DRVCTRL);
				break;

			/* Enable STEP interpolation. 0: Disable STEP pulse interpolation. 1: Enable MicroPlyer STEP pulse multiplication by 16 */
			case SET_INTERPOL:
				/* TMC2590_DRVCTRL */
				register_value = tmc2590->config->shadowRegister[TMC2590_DRVCTRL | TMC2590_WRITE_BIT];
				tmc2590->interpolationEn = value;
				register_value &= ~TMC2590_SET_INTERPOL(-1);                        // clear
				register_value |= TMC2590_SET_INTERPOL(tmc2590->interpolationEn);  // Enable STEP interpolation. 0: Disable STEP pulse interpolation. 1: Enable MicroPlyer STEP pulse multiplication by 16
				tmc2590->config->shadowRegister[TMC2590_DRVCTRL | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_DRVCTRL);
				break;

			/* Chopper mode. This mode bit affects the interpretation of the HDEC, HEND, and HSTRT parameters shown below. 0 Standard mode (SpreadCycle) */
			case SET_CHM:
				/* TMC2590_CHOPCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT];
				tmc2590->chopperMode = value;
				register_value &= ~TMC2590_SET_CHM(-1);                        // clear
				register_value |= TMC2590_SET_CHM(tmc2590->chopperMode);  // Chopper mode. This mode bit affects the interpretation of the HDEC, HEND, and HSTRT parameters shown below. 0 Standard mode (SpreadCycle)
				tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_CHOPCONF);
				break;

			/* Blanking time. Blanking time interval, in system clock periods: %00: 16 %01: 24 %10: 36 %11: 54 */
			case SET_TBL:
				/* TMC2590_CHOPCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT];
				tmc2590->chopperBlankTime = value;
				register_value &= ~TMC2590_SET_TBL(-1);                        // clear
				register_value |= TMC2590_SET_TBL(tmc2590->chopperBlankTime);
				tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_CHOPCONF);
				break;

			/* Off time/MOSFET disable. Duration of slow decay phase. If TOFF is 0, the MOSFETs are shut off. If TOFF is nonzero, slow decay time is a multiple of system clock periods: NCLK= 24 + (32 x TOFF) (Minimum time is 64clocks.), %0000: Driver disable, all bridges off, %0001: 1 (use with TBL of minimum 24 clocks) %0010 … %1111: 2 … 15 */
			case SET_TOFF:
				/* TMC2590_CHOPCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT];
				tmc2590->SlowDecayDuration = value;
				register_value &= ~TMC2590_SET_TOFF(-1);                        // clear
				register_value |= TMC2590_SET_TOFF(tmc2590->SlowDecayDuration);
				tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_CHOPCONF);
				break;

			/* Hysteresis start value, Hysteresis start offset from HEND: %000: 1 %100: 5; %001: 2 %101: 6; %010: 3 %110: 7; %011: 4 %111: 8; Effective: HEND+HSTRT must be ? 15 */
			case SET_HSTRT:
				/* TMC2590_CHOPCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT];
				tmc2590->SlowDecayDuration = value;
				register_value &= ~TMC2590_SET_HSTRT(-1);                        // clear
				register_value |= TMC2590_SET_HSTRT(tmc2590->SlowDecayDuration);
				tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_CHOPCONF);
				break;

			/* Hysteresis end (low) value; %0000 … %1111: Hysteresis is -3, -2, -1, 0, 1, …, 12 (1/512 of this setting adds to current setting) This is the hysteresis value which becomes used for the hysteresis chopper. */
			case SET_HEND:
				/* TMC2590_CHOPCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT];
				tmc2590->SlowDecayDuration = value;
				register_value &= ~TMC2590_SET_HEND(-1);                        // clear
				register_value |= TMC2590_SET_HEND(tmc2590->SlowDecayDuration);
				tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_CHOPCONF);
				break;

			/* Hysteresis decrement period setting, in system clock periods: %00: 16; %01: 32; %10: 48; %11: 64 */
			case SET_HDEC:
				/* TMC2590_CHOPCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT];
				tmc2590->SlowDecayDuration = value;
				register_value &= ~TMC2590_SET_HDEC(-1);                        // clear
				register_value |= TMC2590_SET_HDEC(tmc2590->SlowDecayDuration);
				tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_CHOPCONF);
				break;

			/* Enable randomizing the slow decay phase duration: 0: Chopper off time is fixed as set by bits tOFF 1: Random mode, tOFF is random modulated by dNCLK= -12 - +3 clocks */
			case SET_RNDTF:
				/* TMC2590_CHOPCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT];
				tmc2590->SlowDecayDuration = value;
				register_value &= ~TMC2590_SET_RNDTF(-1);                        // clear
				register_value |= TMC2590_SET_RNDTF(tmc2590->SlowDecayDuration);
				tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_CHOPCONF);
				break;

			/* Lower CoolStep threshold/CoolStep disable. If SEMIN is 0, CoolStep is disabled. If SEMIN is nonzero and the StallGuard2 value SG falls below SEMIN x 32, the CoolStep current scaling factor is increased */
			case SET_SEMIN:
				/* TMC2590_SMARTEN */
				register_value = tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT];
				tmc2590->coolStepMin = value;
				register_value &= ~TMC2590_SET_SEMIN(-1);                        // clear
				register_value |= TMC2590_SET_SEMIN(tmc2590->coolStepMin);
				tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_SMARTEN);
				break;

			/* Current increment size. Number of current increment steps for each time that the StallGuard2 value SG is sampled below the lower threshold: %00: 1; %01: 2; %10: 4; %11: 8 */
			case SET_SEUP:
				/* TMC2590_SMARTEN */
				register_value = tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT];
				tmc2590->coolStepUp = value;
				register_value &= ~TMC2590_SET_SEUP(-1);                        // clear
				register_value |= TMC2590_SET_SEUP(tmc2590->coolStepUp);
				tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_SMARTEN);
				break;

			/* Upper CoolStep threshold as an offset from the lower threshold. If the StallGuard2 measurement value SG is sampled equal to or above (SEMIN+SEMAX+1) x 32 enough times, then the coil current scaling factor is decremented. */
			case SET_SEMAX:
				/* TMC2590_SMARTEN */
				register_value = tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT];
				tmc2590->coolStepMax = value;
				register_value &= ~TMC2590_SET_SEMAX(-1);                        // clear
				register_value |= TMC2590_SET_SEMAX(tmc2590->coolStepMax);
				tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_SMARTEN);
				break;

			/* Current decrement speed. Number of times that the StallGuard2 value must be sampled equal to or above the upper threshold for each decrement of the coil current: %00: 32; %01: 8; %10: 2; %11: 1 */
			case SET_SEDN:
				/* TMC2590_SMARTEN */
				register_value = tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT];
				tmc2590->coolStepDown = value;
				register_value &= ~TMC2590_SET_SEDN(-1);                        // clear
				register_value |= TMC2590_SET_SEDN(tmc2590->coolStepDown);
				tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_SMARTEN);
				break;

			/* Minimum CoolStep current: 0: 1/2 CS current setting; 1: 1/4 CS current setting */
			case SET_SEIMIN:
				/* TMC2590_SMARTEN */
				register_value = tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT];
				tmc2590->coolStepCurrentMin = value;
				register_value &= ~TMC2590_SET_SEIMIN(-1);                        // clear
				register_value |= TMC2590_SET_SEIMIN(tmc2590->coolStepCurrentMin);
				tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_SMARTEN);
				break;

			/* Current scale (scales digital currents A and B). Current scaling for SPI and STEP/DIR operation. 0-31: 1/32, 2/32, 3/32, ... 32/32;  This value is biased by 1 and divided by 32, so the range is 1/32 to 32/32. Example: CS=20 is 21/32 current. */
			case SET_CS:
				/* TMC2590_SGCSCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT];
				//printPgmString(PSTR("before:")); printInteger( register_value ); printPgmString(PSTR(","));
				tmc2590->currentScale = value;
				register_value &= ~TMC2590_SET_CS(-1);                           // clear Current scale bits
				register_value |= TMC2590_SET_CS(tmc2590->currentScale);         // set Current scale  = 16
				tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_SGCSCONF);
				//printPgmString(PSTR("after:"));printInteger( register_value ); printPgmString(PSTR(",")); printPgmString(PSTR("SET_CS"));
				break;

			/* StallGuard2 threshold value. The StallGuard2 threshold value controls the optimum measurement range for readout and stall indicator output (SG_TST). A lower value results in a higher sensitivity and less torque is required to indicate a stall. The value is a two’s complement signed integer. Range: -64 to +63 */
			case SET_SGT:
				/* TMC2590_SGCSCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT];
				tmc2590->stallGuardThreshold = value;
				register_value &= ~TMC2590_SET_SGT(-1);                          // clear,
				register_value |= TMC2590_SET_SGT(tmc2590->stallGuardThreshold); // set threshold
				tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_SGCSCONF);
				break;

			/* StallGuard2 filter enable. 0: Standard mode, fastest response time. 1: Filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy. */
			case SET_SFILT:
				/* TMC2590_SGCSCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT];
				tmc2590->stallGuardFilter = value;
				register_value &= ~TMC2590_SET_SFILT(-1);                        // clear
				register_value |= TMC2590_SET_SFILT(tmc2590->stallGuardFilter);  // 1: Filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy.
				tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_SGCSCONF);
				break;

			/* Slope control, low side, Gate driver strength 1 to 7. 7 is maximum current for fastest slopes */
			case SET_SLPL:
				/* TMC2590_DRVCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT];
				tmc2590->slopeCtrlLow = value;
				register_value &= ~TMC2590_SET_SLPL(-1);                        // clear
				register_value |= TMC2590_SET_SLPL(tmc2590->slopeCtrlLow);
				/* hadle MSB */
				register_value &= ~TMC2590_SET_SLP2(-1);                        // clear
				register_value |= TMC2590_SET_SLP2((tmc2590->slopeCtrlLow)>>2);
				tmc2590->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_DRVCONF);
				break;

			/* Slope control, high side. Gate driver strength 1 to 7. 7 is maximum current for fastest slopes */
			case SET_SLPH:
				/* TMC2590_DRVCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT];
				tmc2590->slopeCtrlHigh = value;
				register_value &= ~TMC2590_SET_SLPH(-1);                        // clear
				register_value |= TMC2590_SET_SLPH(tmc2590->slopeCtrlHigh);
				/* handle MSB */
				register_value &= ~TMC2590_SET_SLP2(-1);                        // clear
				register_value |= TMC2590_SET_SLP2((tmc2590->slopeCtrlHigh)>>2);
				tmc2590->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_DRVCONF);
				break;

				/* Sense resistor voltage-based current scaling. 0: Full-scale sense resistor voltage is 325mV. 1: Full-scale sense resistor voltage is 173mV. (Full-scale refers to a current setting of 31.) */
			case SET_VSENSE:
				/* TMC2590_DRVCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT];
				tmc2590->senseVoltage = value;
				register_value &= ~TMC2590_SET_VSENSE(-1);                        // clear
				register_value |= TMC2590_SET_VSENSE(tmc2590->senseVoltage);
				tmc2590->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_DRVCONF);
				break;

			/* set the current scale applied when no pulses are detected on the given axis */
			case SET_IDLE_CURRENT:
                tmc2590->standStillCurrentScale = value;
				break;

			/* shut off the motor completely, for example to let user move turret easier */
			case SET_MOTOR_FREEWHEEL:
			break;

			/* energize the motor  */
			case SET_MOTOR_ENERGIZED:
			break;

			default:
				break;
			} //switch (command){

		} // if (crc_in == tmc_command[RTL_TMC_COMMAND_SIZE-1]){
            
		else{ /* crc error */
			report_status_message(ASMCNC_STATUS_INVALID_STATEMENT);
		} //else{ /* crc error */

	} // if rtl_data_available != SERIAL_NO_DATA {

} //void execute_TMC_command(){


/* reduce the current through energized motors when idle */
void tmc_all_current_scale_apply( void ){
    
    uint8_t controller_id;    
	TMC2590TypeDef *tmc2590;
	uint32_t register_value;
    
    /* reduce current in each TMC controller */
    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){

        tmc2590 = get_TMC_controller(controller_id);        
		/* TMC2590_SGCSCONF */
		register_value = tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT];
		register_value &= ~TMC2590_SET_CS(-1);                           // clear Current scale bits
        if (current_scale_state == CURRENT_SCALE_STANDSTILL) register_value |= TMC2590_SET_CS(tmc2590->standStillCurrentScale);  // set standstill Current scale
        else                                                            register_value |= TMC2590_SET_CS(tmc2590->currentScale);            // set full operational Current scale
		tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT] = register_value;
        
        /* below if statement is to ensure both dual controllers are bing written in one transaction to speed up the execution */
        if ( (controller_id == TMC_X1) || (controller_id == TMC_Y1) ){
            /* choose second pair and prepare for dual write */
            controller_id++;
            tmc2590 = get_TMC_controller(controller_id);
            /* TMC2590_SGCSCONF */
            register_value = tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT];
            register_value &= ~TMC2590_SET_CS(-1);                           // clear Current scale bits
            if (current_scale_state == CURRENT_SCALE_STANDSTILL) register_value |= TMC2590_SET_CS(tmc2590->standStillCurrentScale);  // set standstill Current scale
            else                                                            register_value |= TMC2590_SET_CS(tmc2590->currentScale);            // set full operational Current scale
            tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT] = register_value;            
        } //if ( (controller_id == TMC_X1) || (controller_id == TMC_Y1) ){
        
        tmc2590_single_write_route(controller_id, TMC2590_SGCSCONF);            
    } //for (controller_id = TMC_X1; controller_id < TOTAL_TMCS, controller_id++){
    
    /* kick SPI process buffer immediately to apply new current settings before the motors start */
    spi_process_tx_queue();

    
} //void tmc_standstill_apply(uint8_t current_scale_standstill_state){

/* reduce the current through energized motors when idle */
void tmc_standstill_on(void){
    /* only apply new current scale if state is different. Prevents muptiple writes of the same command */
    if (current_scale_state != CURRENT_SCALE_STANDSTILL){
        current_scale_state = CURRENT_SCALE_STANDSTILL;    
        tmc_all_current_scale_apply(); /* set standstill Current scale on all motors */
    }      
}

/* bump the current through energized motors back to working level when cycle starts */
void tmc_standstill_off(void){     
    /* only apply new current scale if state is different. Prevents muptiple writes of the same command */
    if (current_scale_state != CURRENT_SCALE_ACTIVE){
        current_scale_state = CURRENT_SCALE_ACTIVE;
        tmc_all_current_scale_apply(); /* set standstill Current scale on all motors */
    }
}


