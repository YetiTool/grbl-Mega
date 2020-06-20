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
    uint8_t microSteps;         /* 4 : set MRES  = 16*/
	uint8_t currentScale;       /* 0 - 31 where 31 is max */
	uint8_t stallGuardFilter;   // 1: Filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy.
	uint8_t stallGuardThreshold;
	uint8_t vSense;             /* 0: Full-scale sense resistor voltage is 325mV. */   
	uint8_t currentStandStill;  //set 1/4 of full scale
	uint8_t coolStepMin;        // set to trigger if SG below 7x32 = 224
	uint8_t coolStepMax;        // set to trigger if SG below 7x32 = 224
    
    uint8_t respIdx;            /* current rdsel to know which response is coming next */
    
    TMC2590Response resp;

	//uint8_t registerAccess[TMC2590_REGISTER_COUNT];
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


/* host commands definitions */
#define TMC_COMMAND_BIT_SIZE 5
#define MOTOR_OFFSET_MASK  0x1F   /* Motor offset mask must be a mask of contiguous zeroes, followed by contiguous sequence of ones: 000...111. */
#define MOTOR_OFFSET (MOTOR_OFFSET_MASK+1)     /* Motor offset */
#define SET_MRES    1   /* Microstep resolution for STEP/DIR mode. Microsteps per fullstep: %0000: 256; %0001: 128; %0010: 64; %0011: 32; %0100: 16; %0101: 8; %0110: 4; %0111: 2 (halfstep); %1000: 1 (fullstep) */
#define SET_DEDGE       2   /*  */
#define SET_INTERPOL    3   /* Enable STEP interpolation. 0: Disable STEP pulse interpolation. 1: Enable MicroPlyer STEP pulse multiplication by 16 */
#define SET_TOFF        4   /*  */
#define SET_HSTRT       5   /*  */
#define SET_HEND        6   /*  */
#define SET_HDEC        7   /*  */
#define SET_RNDTF       8   /*  */
#define SET_CHM         9   /*  */
#define SET_TBL         10   /*  */
#define SET_SEMIN       11   /* Lower CoolStep threshold/CoolStep disable. If SEMIN is 0, CoolStep is disabled. If SEMIN is nonzero and the StallGuard2 value SG falls below SEMIN x 32, the CoolStep current scaling factor is increased */
#define SET_SEUP        12   /* Current increment size. Number of current increment steps for each time that the StallGuard2 value SG is sampled below the lower threshold: %00: 1; %01: 2; %10: 4; %11: 8 */
#define SET_SEMAX       13   /* Upper CoolStep threshold as an offset from the lower threshold. If the StallGuard2 measurement value SG is sampled equal to or above (SEMIN+SEMAX+1) x 32 enough times, then the coil current scaling factor is decremented. */
#define SET_SEDN        14   /* Current decrement speed. Number of times that the StallGuard2 value must be sampled equal to or above the upper threshold for each decrement of the coil current: %00: 32; %01: 8; %10: 2; %11: 1 */
#define SET_SEIMIN      15   /* Minimum CoolStep current: 0: 1/2 CS current setting; 1: 1/4 CS current setting */
#define SET_CS          16   /* Current scale (scales digital currents A and B). Current scaling for SPI and STEP/DIR operation. 0-31: 1/32, 2/32, 3/32, ... 32/32;  This value is biased by 1 and divided by 32, so the range is 1/32 to 32/32. Example: CS=20 is 21/32 current. */
#define SET_SGT         17   /* StallGuard2 threshold value. The StallGuard2 threshold value controls the optimum measurement range for readout and stall indicator output (SG_TST). A lower value results in a higher sensitivity and less torque is required to indicate a stall. The value is a two’s complement signed integer. Range: -64 to +63 */
#define SET_SFILT       18   /* StallGuard2 filter enable. 0: Standard mode, fastest response time. 1: Filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy. */
#define SET_RDSEL       19   /*  */
#define SET_VSENSE      20   /*  */
#define SET_SDOFF       21   /*  */
#define SET_TS2G        22   /*  */
#define SET_DISS2G      23   /*  */
#define SET_SLPL        24   /*  */
#define SET_SLPH        25   /*  */
#define SET_TST         26   /*  */
#define SET_IDLE_CURRENT    27   /* set the current scale applied when no pulses are detected on the given axis */
#define SET_SHUT_OFF    28   /* shut off the motor completely, for example to let user move turret easier */


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
void tmc2590_single_read_all(TMC2590TypeDef *tmc2590);

/*single motor*/
uint8_t tmc2590_single_restore(TMC2590TypeDef *tmc2590_1);
void tmc2590_single_writeInt(TMC2590TypeDef *tmc2590_1, uint8_t address);

/*dual motors*/
void tmc2590_dual_writeInt(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2, uint8_t address);
uint8_t tmc2590_dual_restore(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2);
void tmc2590_dual_read_all(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2);

/* all motors*/
void init_TMC(void);
void tmc2590_schedule_read_all(void); /* schedule periodic read of all values */
void process_status_of_all_controllers(void);
TMC2590TypeDef * get_TMC_controller(uint8_t controller); /* get pointer to required contoller */

void execute_TMC_command(void); /* fetch TMC command from rtl serial buffer and execute */

#endif /* TMC_IC_TMC2590_H_ */
