/*
 * TMC2590.h
 *
 *  Created on: 09.01.2019
 *      Author: LK
 */

#ifndef TMC_IC_TMC2590_H_
#define TMC_IC_TMC2590_H_

#include "API_Header.h"
#include "TMC2590_Constants.h"
#include "TMC2590_Fields.h"
#include "TMC2590_Macros.h"
#include "TMC2590_Register.h"

#define TMC2590_FIELD_READ(tdef, address, mask, shift) \
	FIELD_GET(tmc2590_readInt(tdef, address), mask, shift)
#define TMC2590_FIELD_UPDATE(tdef, address, mask, shift, value) \
	(tmc2590_writeInt(tdef, address, FIELD_SET(tmc2590_readInt(tdef, address), mask, shift, value)))


typedef struct {
	uint16_t mStepCurrenValue;
	uint8_t coolStepCurrenValue;
	uint8_t stallGuardShortValue;
	uint16_t stallGuardCurrenValue;
	uint16_t StatusBits;
	uint16_t DiagnosticBits;
} TMC2590Response;


// Usage note: use 1 TypeDef per IC
typedef struct {
	ConfigurationTypeDef *config;

	uint8_t continuousModeEnable;

	uint8_t coolStepInactiveValue;
	uint8_t coolStepActiveValue;
	uint32_t coolStepThreshold;

	uint8_t isStandStillCurrent;
	uint8_t runCurrentScale;
	uint8_t standStillCurrentScale;
	uint32_t standStillTimeout;
	uint32_t standStillTick;
    

	uint8_t interpolationEn;
    uint8_t microSteps; /* 4 : set MRES  = 16*/
	uint8_t currentScale; /* 0 - 31 where 31 is max */
	uint8_t stallGuardFilter; // 1: Filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy.
	uint8_t stallGuardThreshold;
	uint8_t vSense; /* 0: Full-scale sense resistor voltage is 325mV. */   
	uint8_t currentStandStill; //set 1/4 of full scale
	uint8_t coolStepMin; // set to trigger if SG below 7x32 = 224
	uint8_t coolStepMax; // set to trigger if SG below 7x32 = 224
    
    TMC2590Response resp;

	uint8_t registerAccess[TMC2590_REGISTER_COUNT];
	int32_t registerResetState[TMC2590_REGISTER_COUNT];
    
} TMC2590TypeDef;


static const uint8_t tmc2590_defaultRegisterAccess[TMC2590_REGISTER_COUNT] =
{
	0x02,  // 0: DRVCTRL
	0x00,  // 1: UNUSED
	0x00,  // 2: UNUSED
	0x00,  // 3: UNUSED
	0x02,  // 4: CHOPCONF
	0x02,  // 5: SMARTEN
	0x02,  // 6: SGCSCONF
	0x02   // 7: DRVCONF
};

static const int32_t tmc2590_defaultRegisterResetState[TMC2590_REGISTER_COUNT] =
{
	0x00000204,  // 0: DRVCTRL
	0x00000000,  // 1: UNUSED
	0x00000000,  // 2: UNUSED
	0x00000000,  // 3: UNUSED
	0x00091935,  // 4: CHOPCONF
	0x000A8000,  // 5: SMARTEN
	0x000C0507,  // 6: SGCSCONF
	0x000EF000   // 7: DRVCONF
};

/*
      
static const int32_t tmc2590_defaultRegisterResetState[TMC2590_REGISTER_COUNT] =
{
	0x10000000,  // 0: DRVCTRL
	0x00000000,  // 1: UNUSED
	0x00000000,  // 2: UNUSED
	0x00000000,  // 3: UNUSED
	0x00091935,  // 4: CHOPCONF
	0x000A0000,  // 5: SMARTEN
	0x000D0505,  // 6: SGCSCONF
	0x000EF040   // 7: DRVCONF
};

//====================================================================================================//
// ACTUAL SETTINGS FOR TMC2590 (created: 2020/06/08 09:24:42)                                        //
//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv//

	TMC2590_SPIWriteInt(0x08, 	0x00000204); 		// writing value 0x00000204 = 516 = 0.0 to address 0 = 0x08(DRVCTRL)
	TMC2590_SPIWriteInt(0x0C, 	0x00091935); 		// writing value 0x00091935 = 596277 = 0.0 to address 1 = 0x0C(CHOPCONF)
	TMC2590_SPIWriteInt(0x0D, 	0x000A8000); 		// writing value 0x000A8000 = 688128 = 0.0 to address 2 = 0x0D(SMARTEN)
	TMC2590_SPIWriteInt(0x0E, 	0x000C0507); 		// writing value 0x000C0507 = 787719 = 0.0 to address 3 = 0x0E(SGCSCONF)
	TMC2590_SPIWriteInt(0x0F, 	0x000EF000); 		// writing value 0x000EF000 = 978944 = 0.0 to address 4 = 0x0F(DRVCONF)

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//
*/


/*single motor*/

void tmc2590_init(TMC2590TypeDef *tmc2590, uint8_t channel, ConfigurationTypeDef *tmc2590_config, const int32_t *registerResetState);
void tmc2590_periodicJob(TMC2590TypeDef *tmc2590, uint32_t tick);
void tmc2590_writeInt(TMC2590TypeDef *tmc2590, uint8_t address, int32_t value);
uint32_t tmc2590_readInt(TMC2590TypeDef *tmc2590, uint8_t address);
uint8_t tmc2590_reset(TMC2590TypeDef *tmc2590);
uint8_t tmc2590_restore(TMC2590TypeDef *tmc2590);
uint8_t tmc2590_set_init_drvctrl(TMC2590TypeDef *tmc2590);
uint8_t tmc2590_set_init_SGCSCONF(TMC2590TypeDef *tmc2590);
uint8_t tmc2590_set_init_DRVCONF(TMC2590TypeDef *tmc2590);
uint8_t tmc2590_set_init_SMARTEN(TMC2590TypeDef *tmc2590);
void tmc2590_read_all(TMC2590TypeDef *tmc2590);

/*single motor*/
uint8_t tmc2590_single_restore(TMC2590TypeDef *tmc2590_1);

/*dual motors*/
void tmc2590_dual_writeInt(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2, uint8_t address, uint8_t rdsel, int32_t value_1, int32_t value_2);
uint8_t tmc2590_dual_restore(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2);
void tmc2590_dual_read_all(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2);

void init_TMC(void);

#endif /* TMC_IC_TMC2590_H_ */
