/*
 * TMC2590.h
 *
 *  Created on: 09.01.2019
 *      Author: LK
 */

#ifndef TMC_IC_TMC2590_H_
#define TMC_IC_TMC2590_H_

#include "TMC_interface.h"

/* thermal test to match rack cutter: X:34C, Y:40C; Z:30C */
static const uint8_t tmc2590_defaultStandStillCurrentScale[TOTAL_TMCS] =
{
    23,   /* X1 motor */
    23,   /* X2 motor */
    19,   /* Y1 motor */
    19,   /* Y2 motor */
    21    /* Z motor  */
};

static const uint8_t tmc2590_defaultActiveCurrentScale[TOTAL_TMCS] =
{
    23,   /* X1 motor */
    23,   /* X2 motor */
    19,   /* Y1 motor */
    19,   /* Y2 motor */
    21     /* Z motor  */
};

/* Stall guard temperature compensation coefficients. Those are suitable for stepper motors used in SB1 */
/* full power: 3530x 1999y 5114z*/
/* medium power: 2500x 2500y 2000z*/
/* medium power practical: 5000x 2000y 2000z*/

static const uint16_t tmc2590_defaultTemperatureCoefficient[TOTAL_TMCS] = {
    4000, // X1 23HS22-2804S
    4000, // X2
    1000, // Y1 23HS33-4008S
    1000, // Y2
    1000  // Z  17HS19-2004S1
};

static const uint16_t tmc2590_defaultStallGuardAlarmThreshold[TOTAL_TMCS] =
{
    200,   /* X1 motor */
    200,   /* X2 motor */
    200,   /* Y1 motor */
    200,   /* Y2 motor */
    150    /* Z motor  */
};

static const int32_t tmc2590_defaultRegisterResetState[TOTAL_TMCS][TMC2590_REGISTER_COUNT] =
{
    {
        0x00000204,  // 0: X1 DRVCTRL
        0x000932d4,  // 4: X1 CHOPCONF
        0x000a8100,  // 5: X1 SMARTEN
        0x000d0617,  // 6: X1 SGCSCONF
        0x000ef011   // 7: X1 DRVCONF
    },
    {
        0x00000204,  // 0: X2 DRVCTRL
        0x000932d4,  // 4: X2 CHOPCONF
        0x000a8100,  // 5: X2 SMARTEN
        0x000d0617,  // 6: X2 SGCSCONF
        0x000ef011   // 7: X2 DRVCONF
    },
    {
        0x00000204,  // 0: Y1 DRVCTRL
        0x000932d4,  // 4: Y1 CHOPCONF
        0x000a8100,  // 5: Y1 SMARTEN
        0x000d0413,  // 6: Y1 SGCSCONF
        0x000ef011   // 7: Y1 DRVCONF
    },
    {
        0x00000204,  // 0: Y2 DRVCTRL
        0x000932d4,  // 4: Y2 CHOPCONF
        0x000a8100,  // 5: Y2 SMARTEN
        0x000d0413,  // 6: Y2 SGCSCONF
        0x000ef011   // 7: Y2 DRVCONF
    },
    {
        0x00000204,  // 0: Z  DRVCTRL
        0x000932d4,  // 4: Z  CHOPCONF
        0x000a8100,  // 5: Z  SMARTEN
        0x000d0515,  // 6: Z  SGCSCONF
        0x000ef011   // 7: Z  DRVCONF
    }
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

#define TMC_SG_MAX_AVERAGE    64    /* max number of SG reads into one matrix cell. SG is 10 bit, matrix is 16 bits, so only 2^(16-10) = 64 values could fit safely */

/*single motor*/
void tmc2590_single_read_all(TMC2590TypeDef *tmc2590);
uint8_t tmc2590_single_restore(TMC2590TypeDef *tmc2590_1);

/*dual motors*/
uint8_t tmc2590_dual_restore(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2);
void tmc2590_dual_read_all(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2);

void tmc2590_dual_read_sg(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2);
void tmc2590_single_read_sg(TMC2590TypeDef *tmc2590);
void process_status_of_dual_controller(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2);
void process_status_of_single_controller(TMC2590TypeDef *tmc2590);
void tmc2590_single_write_route(uint8_t controller_id, uint8_t address);
void tmc_hw_init(void);
void tmc_kick_spi_processing(void); /* flush the SPI queue starting from next SPI transfer */
void tmc_load_stall_guard_calibration(void);
void allow_periodic_TMC_poll(uint8_t allowed); /* set global variable allowing or blocking periodic polls */
void tmc_store_calibration_point_from_host(	uint8_t thisMotor, uint8_t idx, uint16_t stallGuardLoadedValue);

#endif /* TMC_IC_TMC2590_H_ */
