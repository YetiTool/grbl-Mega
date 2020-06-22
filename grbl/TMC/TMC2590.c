/*
 * TMC2590.c
 
 */

#include "grbl.h"
#include "TMC2590.h"
#include "spi_to_tmc.h"
#include <string.h>

static void continousSync(TMC2590TypeDef *tmc2590);
static void readWrite(TMC2590TypeDef *tmc2590, uint32_t value);
static void readImmediately(TMC2590TypeDef *tmc2590, uint8_t rdsel);

/* declare structures for all 5 motors */
TMC2590TypeDef tmc2590_X1, tmc2590_X2, tmc2590_Y1, tmc2590_Y2, tmc2590_Z;
ConfigurationTypeDef tmc2590_config_X1, tmc2590_config_X2, tmc2590_config_Y1, tmc2590_config_Y2, tmc2590_config_Z;


static void standStillCurrentLimitation(TMC2590TypeDef *tmc2590, uint32_t tick)
{
	// Check if the motor is in standstill
	if (!TMC2590_GET_STST(tmc2590_readInt(tmc2590, TMC2590_RESPONSE_LATEST)))
	{
		// The standStillTick variable holds the tick counter where a standstill
		// started.
		// Not standing still -> standstill tick equals tick -> Time since
		// standstill == 0
		tmc2590->standStillTick = tick;
	}

	// Check if standstill timeout has been reached
	if (tick - tmc2590->standStillTick > tmc2590->standStillTimeout)
	{
		tmc2590->isStandStillCurrent = 1;
		// Change to standstill current
		TMC2590_FIELD_UPDATE(tmc2590, TMC2590_SGCSCONF, TMC2590_CS_MASK, TMC2590_CS_SHIFT, tmc2590->standStillCurrentScale);
	}
	else
	{
		tmc2590->isStandStillCurrent = 0;
		// Change to run current
		TMC2590_FIELD_UPDATE(tmc2590, TMC2590_SGCSCONF, TMC2590_CS_MASK, TMC2590_CS_SHIFT, tmc2590->runCurrentScale);
	}
}

static void continousSync(TMC2590TypeDef *tmc2590)
{ // refreshes settings to prevent chip from loosing settings on brownout
	static uint8_t write  = 0;
	static uint8_t read   = 0;
	static uint8_t rdsel  = 0;

	// rotational reading all replys to keep values up to date
	uint32_t value, drvConf;

	// additional reading to keep all replies up to date
	value = drvConf = tmc2590_readInt(0, TMC2590_WRITE_BIT | TMC2590_DRVCONF);  // buffer value amd  drvConf to write back later
	value &= ~TMC2590_SET_RDSEL(-1);                                        // clear RDSEL bits
	value |= TMC2590_SET_RDSEL(rdsel % 3);                                  // clear set rdsel
	readWrite(tmc2590, value);
	readWrite(tmc2590, drvConf);

	// determine next read address
	read = (read + 1) % 3;

	// Write settings from shadow register to chip.
	readWrite(tmc2590, tmc2590->config->shadowRegister[TMC2590_WRITE_BIT | write]);

	// Determine next write address while skipping unused addresses
	if (write == TMC2590_DRVCTRL)
	{
		// Skip over the unused addresses between DRVCTRL and CHOPCONF
		write = TMC2590_CHOPCONF;
	}
	else
	{
		// Increase the address
		write = (write + 1) & TMC2590_REGISTER_COUNT;
	}
}

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

static void readImmediately(TMC2590TypeDef *tmc2590, uint8_t rdsel)
{ // sets desired reply in DRVCONF register, resets it to previous settings whilst reading desired reply
	uint32_t value, drvConf;

// additional reading to keep all replies up to date

	value = tmc2590_readInt(tmc2590, TMC2590_WRITE_BIT | TMC2590_DRVCONF);  // buffer value amd  drvConf to write back later
	drvConf = value;
	value &= ~TMC2590_SET_RDSEL(-1);                              // clear RDSEL bits
	value |= TMC2590_SET_RDSEL(rdsel%3);                          // set rdsel
	readWrite(tmc2590, value);                                    // write to chip and readout reply
	readWrite(tmc2590, drvConf);
}

void tmc2590_writeInt(TMC2590TypeDef *tmc2590, uint8_t address, int32_t value)
{
	value = TMC2590_VALUE(value);
	tmc2590->config->shadowRegister[TMC_ADDRESS(address) | TMC2590_WRITE_BIT] = value;
	if(!tmc2590->continuousModeEnable)
		readWrite(tmc2590, value);
}

uint32_t tmc2590_readInt(TMC2590TypeDef *tmc2590, uint8_t address)
{
	if(!tmc2590->continuousModeEnable && !(address & TMC2590_WRITE_BIT))
		readImmediately(tmc2590, address);

	return tmc2590->config->shadowRegister[TMC_ADDRESS(address)];
}

void tmc2590_init(TMC2590TypeDef *tmc2590, uint8_t channel, ConfigurationTypeDef *tmc2590_config, const int32_t *registerResetState)
{
  	uint32_t value;    

	tmc2590->config               = tmc2590_config;
	//tmc2590->config->callback     = NULL;
	tmc2590->config->channel      = channel;
	//tmc2590->config->configIndex  = 0;
	//tmc2590->config->state        = CONFIG_READY;

	tmc2590->continuousModeEnable      = 0;

	tmc2590->coolStepActiveValue       = 0;
	tmc2590->coolStepInactiveValue     = 0;
	tmc2590->coolStepThreshold         = 0;

	tmc2590->isStandStillCurrent       = 0;
	tmc2590->runCurrentScale           = 7;
	tmc2590->standStillCurrentScale    = 3;
	tmc2590->standStillTimeout         = 0;

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

    tmc2590->config->shadowRegister[TMC2590_CHOPCONF  | TMC2590_WRITE_BIT] = TMC2590_VALUE(value);
    

    /* TMC2590_SMARTEN */
	value = tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT];    
    //default: 0x000A0000,  
    
	value &= ~TMC2590_SET_SEIMIN(-1);                       // clear 
	value |= TMC2590_SET_SEIMIN(tmc2590->currentStandStill);// set 1/4 of full scale
    
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
	value |= TMC2590_SET_CS(tmc2590->currentScale);         // set Current scale  = 16
    
    value &= ~TMC2590_SET_SFILT(-1);                        // clear, //0: Standard mode, fastest response time.
  	value |= TMC2590_SET_SFILT(tmc2590->stallGuardFilter);  // 1: Filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy.
    
    value &= ~TMC2590_SET_SGT(-1);                          // clear, 
  	value |= TMC2590_SET_SGT(tmc2590->stallGuardThreshold); // set threshold


    tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT] = TMC2590_VALUE(value);

}

void tmc2590_periodicJob(TMC2590TypeDef *tmc2590, uint32_t tick)
{
	standStillCurrentLimitation(tmc2590, tick);

	if(tmc2590->continuousModeEnable)
	{ // continuously write settings to chip and rotate through all reply types to keep data up to date
		continousSync(tmc2590);
	}
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
    int32_t value_1;

    value_1 = tmc2590_1->config->shadowRegister[TMC_ADDRESS(address) | TMC2590_WRITE_BIT];

    uint8_t addressIsDrvConf = 0;
    uint8_t rdsel = 0;
    
    uint8_t data[5]; 
    memset(data,0,5);    

    /* construct 5 bytes out of 2x20bit values */
    data[0] =                           NIBBLE(value_1, 4); //BYTE(value_1, 2);
    data[1] = (NIBBLE(value_1, 3)<<4) | NIBBLE(value_1, 2); //BYTE(value_1, 1);
    data[2] = (NIBBLE(value_1, 1)<<4) | NIBBLE(value_1, 0); //BYTE(value_1, 0); 
    
    // set virtual read address for next reply given by RDSEL, can only change by setting RDSEL in DRVCONF
    if(TMC2590_GET_ADDRESS(value_1) == TMC2590_DRVCONF){
        addressIsDrvConf = 1;
        rdsel = TMC2590_GET_RDSEL(value_1);
    }
    
    /* schedule write to motor controller */
    spi_schedule_single_tx(tmc2590_1, data, TX_BUF_SIZE_SINGLE, addressIsDrvConf, rdsel);

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
    
    uint32_t value_1;
    
   	value_1 = tmc2590_1->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT];
	   
	value_1 &= ~TMC2590_SET_RDSEL(-1);      // clear RDSEL bits
	value_1 |= TMC2590_SET_RDSEL(rdsel);    // set rdsel
    //nrf_delay_us(delay_us);
    
    tmc2590_1->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT] = value_1;

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
    int32_t value_1, value_2;

    value_1 = tmc2590_1->config->shadowRegister[TMC_ADDRESS(address) | TMC2590_WRITE_BIT];
    value_2 = tmc2590_2->config->shadowRegister[TMC_ADDRESS(address) | TMC2590_WRITE_BIT];
    
    uint8_t addressIsDrvConf = 0;
    uint8_t rdsel = 0;

    uint8_t data[5]; 

    /* construct 5 bytes out of 2x20bit values */
    data[0] = (NIBBLE(value_2, 4)<<4) | NIBBLE(value_2, 3); //BYTE(value_2, 2);        
    data[1] = (NIBBLE(value_2, 2)<<4) | NIBBLE(value_2, 1); //BYTE(value_2, 1);        
    data[2] = (NIBBLE(value_2, 0)<<4) | NIBBLE(value_1, 4); //BYTE(value_1, 2);
    data[3] = (NIBBLE(value_1, 3)<<4) | NIBBLE(value_1, 2); //BYTE(value_1, 1);
    data[4] = (NIBBLE(value_1, 1)<<4) | NIBBLE(value_1, 0); //BYTE(value_1, 0); 
    
    // set virtual read address for next reply given by RDSEL, can only change by setting RDSEL in DRVCONF
    if(TMC2590_GET_ADDRESS(value_1) == TMC2590_DRVCONF){
        addressIsDrvConf = 1;
        rdsel = TMC2590_GET_RDSEL(value_1);
    }
    
    /* can write to the same channel as both motors are on the same line */
    spi_schedule_dual_tx(tmc2590_1, tmc2590_2, data, TX_BUF_SIZE_DUAL, addressIsDrvConf, rdsel);    

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
    
    uint32_t value_1, value_2;
    
   	value_1 = tmc2590_1->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT];
   	value_2 = tmc2590_2->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT];
	   
	value_1 &= ~TMC2590_SET_RDSEL(-1);      // clear RDSEL bits
	value_1 |= TMC2590_SET_RDSEL(rdsel);    // set rdsel
	value_2 &= ~TMC2590_SET_RDSEL(-1);      // clear RDSEL bits
	value_2 |= TMC2590_SET_RDSEL(rdsel);    // set rdsel
    
    tmc2590_1->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT] = value_1;
    tmc2590_2->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT] = value_2;

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
    //tmc2590_dual_read_all(&tmc2590_X1, &tmc2590_X2);
    tmc2590_single_read_all(&tmc2590_X1);
    tmc2590_single_read_all(&tmc2590_X2);
    tmc2590_dual_read_all(&tmc2590_Y1, &tmc2590_Y2);
    tmc2590_single_read_all(&tmc2590_Z);
}

void process_status_of_single_controller(TMC2590TypeDef *tmc2590){
    /* TMC2590_RESPONSE0 #define TMC2590_GET_MSTEP(X)  (0x3FF & ((X) >> 10)) */     
    tmc2590->resp.mStepCurrenValue = TMC2590_GET_MSTEP(tmc2590->config->shadowRegister[TMC2590_RESPONSE0]) & 0x1FF; /* bit 9 is polarity bit, ignore it*/
    
    /* TMC2590_RESPONSE1 #define TMC2590_GET_SG(X)     (0x3FF & ((X) >> 10)) */
    tmc2590->resp.stallGuardCurrenValue = TMC2590_GET_SG(tmc2590->config->shadowRegister[TMC2590_RESPONSE1]);  

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


void process_status_of_all_controllers(void){
    /* process all responses and update the current status of controller's parameters */
    //process_status_of_dual_controller(&tmc2590_X1, &tmc2590_X2);
    process_status_of_single_controller(&tmc2590_X1);
    process_status_of_single_controller(&tmc2590_X2);
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
    return &tmc2590_X1;
}


void init_TMC(void){

	/* init TMC */
    uint8_t channel_X = SPI_CS_X_PIN;
    uint8_t channel_Y = SPI_CS_Y_PIN;
    uint8_t channel_X2 = SPI_CS_X2_PIN;
    uint8_t channel_Y2 = SPI_CS_Y2_PIN;
    uint8_t channel_Z = SPI_CS_Z_PIN;

	tmc2590_X1.interpolationEn      = 1;
	tmc2590_X1.microSteps           = 4; /* 4 : set MRES  = 16*/
	tmc2590_X1.currentScale         = 4; /* 0 - 31 where 31 is max */
	tmc2590_X1.stallGuardFilter     = 1; // 1: Filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy.
	tmc2590_X1.stallGuardThreshold  = 0;
	tmc2590_X1.vSense               = 0; /* 0: Full-scale sense resistor voltage is 325mV. */
	tmc2590_X1.currentStandStill    = 1; // 1: set 1/4 of full scale
	tmc2590_X1.coolStepMin          = 7; // set to trigger if SG below 7x32 = 224
	tmc2590_X1.coolStepMax          = 1; // set to trigger if SG above (7+1)8x32 = 256
    tmc2590_X1.respIdx              = 0; // very first resp index would be 0
	memcpy(&tmc2590_X2, &tmc2590_X1, sizeof(tmc2590_X1));
	memcpy(&tmc2590_Y1, &tmc2590_X1, sizeof(tmc2590_X1));
	memcpy(&tmc2590_Y2, &tmc2590_X1, sizeof(tmc2590_X1));
	memcpy(&tmc2590_Z,  &tmc2590_X1, sizeof(tmc2590_X1));
    
    spi_hw_init();

	/* initialise wanted variables */
	tmc2590_init(&tmc2590_X1, channel_X, &tmc2590_config_X1, tmc2590_defaultRegisterResetState);
	tmc2590_init(&tmc2590_X2, channel_X2, &tmc2590_config_X2, tmc2590_defaultRegisterResetState);
	tmc2590_init(&tmc2590_Y1, channel_Y, &tmc2590_config_Y1, tmc2590_defaultRegisterResetState);
	tmc2590_init(&tmc2590_Y2, channel_Y2, &tmc2590_config_Y2, tmc2590_defaultRegisterResetState);
	tmc2590_init(&tmc2590_Z,  channel_Z, &tmc2590_config_Z, tmc2590_defaultRegisterResetState);

	/* initialise motors with wanted parameters */
    //tmc2590_dual_restore(&tmc2590_X1, &tmc2590_X2);
    tmc2590_single_restore(&tmc2590_X1);
    tmc2590_single_restore(&tmc2590_X2);

    tmc2590_dual_restore(&tmc2590_Y1, &tmc2590_Y2);
    tmc2590_single_restore(&tmc2590_Z);

}

/* route single motor write to single or dual write command depend on the motor controller type */
void tmc2590_single_write_route(uint8_t controller_id, uint8_t address){
    TMC2590TypeDef *tmc2590_1, *tmc2590_2;
    if ( controller_id == TMC_Y1) {
        /* choose second pair and execute dual write */
        tmc2590_1 = get_TMC_controller(controller_id);
        tmc2590_2 = get_TMC_controller(controller_id+1);
        tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, address);

    }
    if ( controller_id == TMC_Y2 ){
        /* choose second pair and execute dual write */
        tmc2590_1 = get_TMC_controller(controller_id-1);
        tmc2590_2 = get_TMC_controller(controller_id);
        tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, address);
    }
    if ( (controller_id == TMC_X1) || (controller_id == TMC_X2) || (controller_id == TMC_Z) ) {
        /* choose second pair and execute single write */
        tmc2590_1 = get_TMC_controller(controller_id);
        tmc2590_single_writeInt(tmc2590_1, address);
    }

}


/* fetch TMC host command from rtl serial buffer and execute */
void execute_TMC_command(){
    /* fetch 3 bytes from the buffer, calculate CRC and execute*/
    uint8_t tmc_command[RTL_TMC_COMMAND_SIZE], idx;
    for (idx = 0; idx < RTL_TMC_COMMAND_SIZE; idx++){
        tmc_command[idx] = serial_read_rtl();
    }
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
            break;

        /* Enable STEP interpolation. 0: Disable STEP pulse interpolation. 1: Enable MicroPlyer STEP pulse multiplication by 16 */
        case SET_INTERPOL:
            break;

        /* Lower CoolStep threshold/CoolStep disable. If SEMIN is 0, CoolStep is disabled. If SEMIN is nonzero and the StallGuard2 value SG falls below SEMIN x 32, the CoolStep current scaling factor is increased */
        case SET_SEMIN:
            break;

        /* Current increment size. Number of current increment steps for each time that the StallGuard2 value SG is sampled below the lower threshold: %00: 1; %01: 2; %10: 4; %11: 8 */
        case SET_SEUP:
            break;

        /* Upper CoolStep threshold as an offset from the lower threshold. If the StallGuard2 measurement value SG is sampled equal to or above (SEMIN+SEMAX+1) x 32 enough times, then the coil current scaling factor is decremented. */
        case SET_SEMAX:
            break;

        /* Current decrement speed. Number of times that the StallGuard2 value must be sampled equal to or above the upper threshold for each decrement of the coil current: %00: 32; %01: 8; %10: 2; %11: 1 */
        case SET_SEDN:
            break;

        /* Minimum CoolStep current: 0: 1/2 CS current setting; 1: 1/4 CS current setting */
        case SET_SEIMIN:
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
            tmc2590_writeInt(tmc2590, TMC2590_SGCSCONF, tmc2590->registerResetState[TMC2590_SGCSCONF]);
            break;

        /* StallGuard2 filter enable. 0: Standard mode, fastest response time. 1: Filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy. */
        case SET_SFILT:
            /* TMC2590_SGCSCONF */
            register_value = tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT];
            tmc2590->stallGuardFilter = value;
            register_value &= ~TMC2590_SET_SFILT(-1);                        // clear, //0: Standard mode, fastest response time.
            register_value |= TMC2590_SET_SFILT(tmc2590->stallGuardFilter);  // 1: Filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy.
            tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT] = register_value;
            tmc2590_writeInt(tmc2590, TMC2590_SGCSCONF, tmc2590->registerResetState[TMC2590_SGCSCONF]);
            break;

        /* set the current scale applied when no pulses are detected on the given axis */
        case SET_IDLE_CURRENT:
            break;

        /* shut off the motor completely, for example to let user move turret easier */
        case SET_SHUT_OFF:
            break;

        default:
            break;
        }


    }
    else{ /* crc error */
        report_status_message(ASMCNC_STATUS_INVALID_STATEMENT);
    }


}

