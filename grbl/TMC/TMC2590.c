/*
 * TMC2590.c
 *
 *  Created on: 09.01.2019
 *      Author: LK
 */

#include "TMC2590.h"
//#include "nrf_delay.h"

// => SPI wrapper
extern void tmc2590_readWriteArray(uint8_t channel, uint8_t *data, size_t length);
// <= SPI wrapper

static void continousSync(TMC2590TypeDef *tmc2590);
static void readWrite(TMC2590TypeDef *tmc2590, uint32_t value);
static void readImmediately(TMC2590TypeDef *tmc2590, uint8_t rdsel);


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

	tmc2590_readWriteArray(tmc2590->config->channel, &data[0], 3);

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
	tmc2590->config->callback     = NULL;
	tmc2590->config->channel      = channel;
	tmc2590->config->configIndex  = 0;
	tmc2590->config->state        = CONFIG_READY;

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
    
    tmc2590->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT] = value;
    
    
    /* TMC2590_DRVCTRL */
	value = tmc2590->config->shadowRegister[TMC2590_DRVCTRL | TMC2590_WRITE_BIT];
    
	value &= ~TMC2590_SET_INTERPOL(-1);                         // clear bits
    value |= TMC2590_SET_INTERPOL(tmc2590->interpolationEn);    /* enable interpolation */    
    
    /* 16 microsteps */
	value &= ~TMC2590_SET_MRES(-1);                             // clear MRES bits
	value |= TMC2590_SET_MRES(tmc2590->microSteps);             // set MRES  = 16
    
    tmc2590->config->shadowRegister[TMC2590_DRVCTRL | TMC2590_WRITE_BIT] = value;


    /* TMC2590_CHOPCONF */
	value = tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT];    
    //default: 0x00091935,  

    tmc2590->config->shadowRegister[TMC2590_CHOPCONF  | TMC2590_WRITE_BIT] = value;
    

    /* TMC2590_SMARTEN */
	value = tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT];    
    //default: 0x000A0000,  
    
	value &= ~TMC2590_SET_SEIMIN(-1);                       // clear 
	value |= TMC2590_SET_SEIMIN(tmc2590->currentStandStill);// set 1/4 of full scale
    
    value &= ~TMC2590_SET_SEMIN(-1);                        // clear, 
  	value |= TMC2590_SET_SEMIN(tmc2590->coolStepMin);       // set to trigger if SG below 7x32 = 224

    value &= ~TMC2590_SET_SEMAX(-1);                        // clear, 
  	value |= TMC2590_SET_SEMAX(tmc2590->coolStepMax);       // set to trigger if SG above 7x32 = 224

    tmc2590->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT] = value;
    
    
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

    tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT] = value;  

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

void tmc2590_single_writeInt(TMC2590TypeDef *tmc2590_1, uint8_t address, uint8_t respIdx, int32_t value_1)
{
	value_1 = TMC2590_VALUE(value_1);
	tmc2590_1->config->shadowRegister[TMC_ADDRESS(address) | TMC2590_WRITE_BIT] = value_1;

    uint8_t data[3]; 

    /* construct 5 bytes out of 2x20bit values */
    data[0] =                           NIBBLE(value_1, 4); //BYTE(value_1, 2);
    data[1] = (NIBBLE(value_1, 3)<<4) | NIBBLE(value_1, 2); //BYTE(value_1, 1);
    data[2] = (NIBBLE(value_1, 1)<<4) | NIBBLE(value_1, 0); //BYTE(value_1, 0); 

    /* can write to the same channel as both motors are on the same line */
	tmc2590_readWriteArray(tmc2590_1->config->channel, &data[0], 3);     

    /* deconstruct response */
    tmc2590_1->config->shadowRegister[respIdx] = TMC2590_VALUE(_8_32(data[0], data[1], data[2], 0) >> 12) ;    
}

uint8_t tmc2590_single_restore(TMC2590TypeDef *tmc2590_1)
{

	tmc2590_single_writeInt(tmc2590_1, TMC2590_DRVCONF,  0, tmc2590_1->config->shadowRegister[TMC2590_DRVCONF  | TMC2590_WRITE_BIT]);
	tmc2590_single_writeInt(tmc2590_1, TMC2590_DRVCTRL,  0, tmc2590_1->config->shadowRegister[TMC2590_DRVCTRL  | TMC2590_WRITE_BIT]);
	tmc2590_single_writeInt(tmc2590_1, TMC2590_CHOPCONF, 0, tmc2590_1->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT]);
	tmc2590_single_writeInt(tmc2590_1, TMC2590_SMARTEN,  0, tmc2590_1->config->shadowRegister[TMC2590_SMARTEN  | TMC2590_WRITE_BIT]);
	tmc2590_single_writeInt(tmc2590_1, TMC2590_SGCSCONF, 0, tmc2590_1->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT]);
    
	return 1;
}

void tmc2590_read_single(TMC2590TypeDef *tmc2590_1, uint8_t respIdx, uint8_t rdsel){
    
    //uint32_t delay_us = 50;
    
    uint32_t value_1;
    
   	value_1 = tmc2590_1->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT];
	   
	value_1 &= ~TMC2590_SET_RDSEL(-1);      // clear RDSEL bits
	value_1 |= TMC2590_SET_RDSEL(rdsel);    // set rdsel
    //nrf_delay_us(delay_us);
    
    tmc2590_single_writeInt(tmc2590_1, TMC2590_DRVCONF, respIdx, value_1); 
    
}

void tmc2590_read_all(TMC2590TypeDef *tmc2590)
{    
    /*read all 4 report values */   
    tmc2590_read_single(tmc2590, TMC2590_RESPONSE0,       0); /* ignore this response*/
    tmc2590_read_single(tmc2590, TMC2590_RESPONSE0,       1); /* response 0 */
    tmc2590_read_single(tmc2590, TMC2590_RESPONSE1,       2); /* response 1 */
    tmc2590_read_single(tmc2590, TMC2590_RESPONSE2,       3); /* response 2 */
    tmc2590_read_single(tmc2590, TMC2590_RESPONSE_LATEST, 0); /* response 3 */    
    
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

/************************************************ dual motors ***********************************************/

void tmc2590_dual_writeInt(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2, uint8_t address, uint8_t respIdx, int32_t value_1, int32_t value_2)
{
	value_1 = TMC2590_VALUE(value_1);
	value_2 = TMC2590_VALUE(value_2);
	tmc2590_1->config->shadowRegister[TMC_ADDRESS(address) | TMC2590_WRITE_BIT] = value_1;
	tmc2590_2->config->shadowRegister[TMC_ADDRESS(address) | TMC2590_WRITE_BIT] = value_2;

    uint8_t data[5]; 

    /* construct 5 bytes out of 2x20bit values */
    data[0] = (NIBBLE(value_2, 4)<<4) | NIBBLE(value_2, 3); //BYTE(value_2, 2);        
    data[1] = (NIBBLE(value_2, 2)<<4) | NIBBLE(value_2, 1); //BYTE(value_2, 1);        
    data[2] = (NIBBLE(value_2, 0)<<4) | NIBBLE(value_1, 4); //BYTE(value_1, 2);
    data[3] = (NIBBLE(value_1, 3)<<4) | NIBBLE(value_1, 2); //BYTE(value_1, 1);
    data[4] = (NIBBLE(value_1, 1)<<4) | NIBBLE(value_1, 0); //BYTE(value_1, 0); 

    /* can write to the same channel as both motors are on the same line */
	tmc2590_readWriteArray(tmc2590_1->config->channel, &data[0], 5);     

    /* deconstruct response */
    tmc2590_2->config->shadowRegister[respIdx] =               _8_32(data[0], data[1], data[2], 0) >> 12;    
    tmc2590_1->config->shadowRegister[respIdx] = TMC2590_VALUE(_8_32(data[2], data[3], data[4], 0) >> 8) ;    
}


uint8_t tmc2590_dual_restore(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2)
{

	tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, TMC2590_DRVCONF, 0,  
                            tmc2590_1->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT] , 
                            tmc2590_2->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT] );
	tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, TMC2590_DRVCTRL, 0,  
                            tmc2590_1->config->shadowRegister[TMC2590_DRVCTRL | TMC2590_WRITE_BIT] , 
                            tmc2590_2->config->shadowRegister[TMC2590_DRVCTRL | TMC2590_WRITE_BIT] );
	tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, TMC2590_CHOPCONF, 0, 
                            tmc2590_1->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT], 
                            tmc2590_2->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT]);
	tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, TMC2590_SMARTEN, 0,  
                            tmc2590_1->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT] , 
                            tmc2590_2->config->shadowRegister[TMC2590_SMARTEN | TMC2590_WRITE_BIT] );
	tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, TMC2590_SGCSCONF, 0, 
                            tmc2590_1->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT], 
                            tmc2590_2->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT]);
    
	return 1;
}


void tmc2590_dual_read_single(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2, uint8_t respIdx, uint8_t rdsel){
    
    //uint32_t delay_us = 50;
    
    uint32_t value_1, value_2;
    
   	value_1 = tmc2590_1->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT];
   	value_2 = tmc2590_2->config->shadowRegister[TMC2590_DRVCONF | TMC2590_WRITE_BIT];
	   
	value_1 &= ~TMC2590_SET_RDSEL(-1);      // clear RDSEL bits
	value_1 |= TMC2590_SET_RDSEL(rdsel);    // set rdsel
	value_2 &= ~TMC2590_SET_RDSEL(-1);      // clear RDSEL bits
	value_2 |= TMC2590_SET_RDSEL(rdsel);    // set rdsel
    //nrf_delay_us(delay_us);
    
    tmc2590_dual_writeInt(tmc2590_1, tmc2590_2, TMC2590_DRVCONF, respIdx, value_1, value_2); 
    
}

void tmc2590_dual_read_all(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2)
{
	
    /*read all 4 report values */   
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, TMC2590_RESPONSE0,       0); /* ignore this response*/
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, TMC2590_RESPONSE0,       1); /* response 0 */
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, TMC2590_RESPONSE1,       2); /* response 1 */
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, TMC2590_RESPONSE2,       3); /* response 2 */
    tmc2590_dual_read_single(tmc2590_1, tmc2590_2, TMC2590_RESPONSE_LATEST, 0); /* response 3 */
    
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

      
