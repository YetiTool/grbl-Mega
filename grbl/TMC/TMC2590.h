/*
 * TMC2590.h
 *
 *  Created on: 09.01.2019
 *      Author: LK
 */

#ifndef TMC_IC_TMC2590_H_
#define TMC_IC_TMC2590_H_

#include "TMC_interface.h"

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

#define TMC_SG_MAX_AVERAGE    64    /* max number of SG reads into one matrix cell. SG is 10 bit, matrix is 16 bits, so only 2^(16-10) = 64 values could fit safely */

void tmc2590_init(TMC2590TypeDef *tmc2590, uint8_t channel, ConfigurationTypeDef *tmc2590_config, const int32_t *registerResetState);

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
void stall_guard_calibration_load(void);


#endif /* TMC_IC_TMC2590_H_ */
