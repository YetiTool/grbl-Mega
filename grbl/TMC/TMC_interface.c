/*
 * TMC_interface.c
 *
 * Created: 28/08/2020 23:00:42
 *  Author: bk
 */ 

#include "grbl.h"

#include "TMC2590.h"

stepper_tmc_t st_tmc;                               /* global structure holding stall guard counters and current speed */

/* declare structures for all 5 motors */
TMC2590TypeDef tmc2590_X1, tmc2590_X2, tmc2590_Y1, tmc2590_Y2, tmc2590_Z;
ConfigurationTypeDef tmc2590_config_X1, tmc2590_config_X2, tmc2590_config_Y1, tmc2590_config_Y2, tmc2590_config_Z;

uint8_t homing_sg_read_ongoing = false;             /* global flag indicating homing is ongoing */

/************************************************ all motors ***********************************************/


/* schedule periodic read of all values */
void tmc2590_schedule_read_all(void){    
    tmc2590_dual_read_all(&tmc2590_X1, &tmc2590_X2);
    tmc2590_dual_read_all(&tmc2590_Y1, &tmc2590_Y2);
    tmc2590_single_read_all(&tmc2590_Z);
    /* update st_tmc.sg_read_active_axes with all axes to be processed */
    st_tmc.sg_read_active_axes |= ( ( 1 << X_AXIS ) | ( 1 << Y_AXIS ) | ( 1 << Z_AXIS ) );
    /* start SPI transfers flushing the queue */
    tmc_kick_spi_processing();
}

/* schedule read of SG value on axis passed as parameter */
void tmc2590_schedule_read_sg(uint8_t axis){
    if ( st_tmc.SG_skips_counter[axis] < SG_READING_SKIPS_AFTER_SLOW_FEED )
    {
        /* increment skip counter used to skip readings at slow feed - if slow feed was too recent SG might be invalid */
        st_tmc.SG_skips_counter[axis]++;
        #ifdef SG_SKIP_DEBUG_ENABLED
        debug_pin_write(1, DEBUG_2_PIN);
        debug_pin_write(0, DEBUG_2_PIN);
        #endif
    }
    /* update st_tmc.sg_read_active_axes with axis to be processed */
    st_tmc.sg_read_active_axes |= ( 1 << axis );
    
    switch (axis){
        case X_AXIS:
        tmc2590_dual_read_sg(&tmc2590_X1, &tmc2590_X2);
        break;
            
        case Y_AXIS:
        tmc2590_dual_read_sg(&tmc2590_Y1, &tmc2590_Y2);
        break;
            
        case Z_AXIS:
        tmc2590_single_read_sg(&tmc2590_Z);
        break;
            
        default:
        break;
            
        } //switch (axis){
            
    /* start SPI transfers flushing the queue */
    tmc_kick_spi_processing();            
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
    
    tmc_globals_reset();

	tmc2590_X1.interpolationEn              = 1;
	tmc2590_X1.microSteps                   = 4; /* 4 : set MRES  = 16*/
	tmc2590_X1.currentScale                 = 31; /* 0 - 31 where 31 is max */
	tmc2590_X1.stallGuardFilter             = 1; // 1: Filtered mode, updated once for each four fullsteps to compensate for variation in motor construction, highest accuracy.
	tmc2590_X1.stallGuardThreshold          = 6; 
	tmc2590_X1.stallGuardAlarmValue         = 100; /* when current SG reading is lower than this value corresponded axis alarm will be triggered */    
	tmc2590_X1.stallGuardAlarmThreshold     = 200; /* when current SG reading is lower than calibrated by this value corresponded axis alarm will be triggered */
	tmc2590_X1.vSense                       = 0; /* 0: Full-scale sense resistor voltage is 325mV. 1: Full-scale sense resistor voltage is 173mV. */
	tmc2590_X1.currentSEmin                 = 1; // 1: set 1/4 of full scale when CoolStep is active
	tmc2590_X1.coolStepMin                  = 0; // default CoolStep = 0 (disable); if want to enable then set for example to trigger if SG below 7x32 = 224
	tmc2590_X1.coolStepMax                  = 1; // set to trigger if SG above (7+1)8x32 = 256
    tmc2590_X1.respIdx                      = DEFAULT_TMC_READ_SELECT; // very first resp index would be DEFAULT_TMC_READ_SELECT
    tmc2590_X1.SlowDecayDuration            = 5; // Off time/MOSFET disable. Duration of slow decay phase. If TOFF is 0, the MOSFETs are shut off. If TOFF is nonzero, slow decay time is a multiple of system clock periods: NCLK= 24 + (32 x TOFF) (Minimum time is 64clocks.), %0000: Driver disable, all bridges off, %0001: 1 (use with TBL of minimum 24 clocks) %0010 ... %1111: 2 ... 15 */
    tmc2590_X1.HystStart                    = 2; /* Hysteresis start value, Hysteresis start offset from HEND: %000: 1 %100: 5; %001: 2 %101: 6; %010: 3 %110: 7; %011: 4 %111: 8; Effective: HEND+HSTRT must be 15 */
    tmc2590_X1.HystEnd                      = 2; /* Hysteresis end (low) value; %0000 ... %1111: Hysteresis is -3, -2, -1, 0, 1, ..., 12 (1/512 of this setting adds to current setting) This is the hysteresis value which becomes used for the hysteresis chopper. */
    tmc2590_X1.HystDectrement               = 2; /* Hysteresis decrement period setting, in system clock periods: %00: 16; %01: 32; %10: 48; %11: 64 */
    tmc2590_X1.SlowDecayRandom              = 1; /* Enable randomizing the slow decay phase duration: 0: Chopper off time is fixed as set by bits tOFF 1: Random mode, tOFF is random modulated by dNCLK= -12 - +3 clocks */
    tmc2590_X1.chopperMode                  = 0; // Chopper mode. This mode bit affects the interpretation of the HDEC, HEND, and HSTRT parameters shown below. 0 Standard mode (SpreadCycle)    
    tmc2590_X1.chopperBlankTime             = 2; // Blanking time. Blanking time interval, in system clock periods: %00: 16 %01: 24 %10: 36 %11: 54
	tmc2590_X1.standStillCurrentScale       = 15; // 15: set 1/2 of full scale, 1/4th of power
    tmc2590_X1.slopeCtrlHigh                = 3;  // Slope control, high side. Gate driver strength 1 to 7. 7 is maximum current for fastest slopes
    tmc2590_X1.slopeCtrlLow                 = 3;  // Slope control, low side, Gate driver strength 1 to 7. 7 is maximum current for fastest slopes



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
    
    tmc2590_X1.thisAxis                     = X_AXIS;
    tmc2590_X2.thisAxis                     = X_AXIS;
    tmc2590_Y1.thisAxis                     = Y_AXIS;
    tmc2590_Y2.thisAxis                     = Y_AXIS;
    tmc2590_Z.thisAxis                      = Z_AXIS;


    
	tmc2590_Y1.standStillCurrentScale       = 30; // 30: set 30/31 of full scale, 90% of power; this is required for Y motor to prevent operator from accidentally knock the X beam off the position
	tmc2590_Y2.standStillCurrentScale       = 30; // 30: set 30/31 of full scale, 90% of power; this is required for Y motor to prevent operator from accidentally knock the X beam off the position
    
    
    
#ifdef RIGGY
    /* no motor */
    tmc2590_X1.stallGuardThreshold          = 6;
    tmc2590_X1.stallGuardAlarmValue         = 300;
    tmc2590_X1.stallGuardAlarmThreshold     = 200;
    tmc2590_X1.currentScale                 = 31; /* 0 - 31 where 31 is max */
	tmc2590_X1.standStillCurrentScale       = 0; // 30: set 30/31 of full scale, 90% of power; this is required for Y motor to prevent operator from accidentally knock the X beam off the position
    /* ZH motor (medium 23HS22) in riggy conditions (177steps/mm)*/
    //tmc2590_X1.stallGuardThreshold          = 7;
    //tmc2590_X1.stallGuardAlarmValue         = 200;
    //tmc2590_X1.stallGuardAlarmThreshold         = 200;
    //tmc2590_X1.currentScale                 = 31; /* 0 - 31 where 31 is max */
    
    /* riggy motor (smallest 17HS15-0404S) idle SG ~500, loaded ~400  at 3000mm/min on X with 177steps/mm*/
    tmc2590_X2.stallGuardThreshold          = 5;
    tmc2590_X2.stallGuardAlarmValue         = 400;
    tmc2590_X2.stallGuardAlarmThreshold     = 200;
    tmc2590_X2.currentScale                 = 1; /* 0 - 31 where 31 is max, 0.25A */
    tmc2590_X2.standStillCurrentScale       = 0; //  2: set 1/2 of full scale, 1/4th of power
    tmc2590_X2.vSense                       = 1; /* 0: Full-scale sense resistor voltage is 325mV. 1: Full-scale sense resistor voltage is 173mV.*/
    
    tmc2590_Y1.stallGuardThreshold          = 3;
    tmc2590_Y1.stallGuardAlarmValue         = 400;
    tmc2590_Y1.stallGuardAlarmThreshold     = 200;
    tmc2590_Y1.currentScale                 = 31; /* 0 - 31 where 31 is max */
    tmc2590_Y1.SlowDecayDuration            = 4;
    tmc2590_Y1.HystStart                    = 5; /* Hysteresis start value, Hysteresis start offset from HEND: %000: 1 %100: 5; %001: 2 %101: 6; %010: 3 %110: 7; %011: 4 %111: 8; Effective: HEND+HSTRT must be 15 */
    tmc2590_Y1.HystEnd                      = 5; /* Hysteresis end (low) value; %0000 ... %1111: Hysteresis is -3, -2, -1, 0, 1, ..., 12 (1/512 of this setting adds to current setting) This is the hysteresis value which becomes used for the hysteresis chopper. */
    tmc2590_Y1.HystDectrement               = 2; /* Hysteresis decrement period setting, in system clock periods: %00: 16; %01: 32; %10: 48; %11: 64 */
    tmc2590_Y1.slopeCtrlLow                 = 3;  // Slope control, low side, Gate driver strength 1 to 7. 7 is maximum current for fastest slopes
    tmc2590_Y1.slopeCtrlHigh                = 3;  // Slope control, high side. Gate driver strength 1 to 7. 7 is maximum current for fastest slopes
    
    tmc2590_Y2.stallGuardThreshold          = 3;
    tmc2590_Y2.stallGuardAlarmValue         = 400;
    tmc2590_Y2.stallGuardAlarmThreshold     = 200;
    tmc2590_Y2.currentScale                 = 31; /* 0 - 31 where 31 is max */
    tmc2590_Y2.SlowDecayDuration            = 4;
    tmc2590_Y2.HystStart                    = 5; /* Hysteresis start value, Hysteresis start offset from HEND: %000: 1 %100: 5; %001: 2 %101: 6; %010: 3 %110: 7; %011: 4 %111: 8; Effective: HEND+HSTRT must be 15 */
    tmc2590_Y2.HystEnd                      = 5; /* Hysteresis end (low) value; %0000 ... %1111: Hysteresis is -3, -2, -1, 0, 1, ..., 12 (1/512 of this setting adds to current setting) This is the hysteresis value which becomes used for the hysteresis chopper. */
    tmc2590_Y2.HystDectrement               = 2; /* Hysteresis decrement period setting, in system clock periods: %00: 16; %01: 32; %10: 48; %11: 64 */
    tmc2590_Y2.slopeCtrlLow                 = 3;  // Slope control, low side, Gate driver strength 1 to 7. 7 is maximum current for fastest slopes
    tmc2590_Y2.slopeCtrlHigh                = 3;  // Slope control, high side. Gate driver strength 1 to 7. 7 is maximum current for fastest slopes
    
    /* ZH motor */
    /* riggy motor (smallest 17HS15-0404S) idle SG ~500, loaded ~400 at 2000mm/min on Z with 267steps/mm*/
    tmc2590_Z.HystEnd                       = 0;   /* Hysteresis end (low) value; %0000 ... %1111: Hysteresis is -3, -2, -1, 0, 1, ..., 12 (1/512 of this setting adds to current setting) This is the hysteresis value which becomes used for the hysteresis chopper. */
    tmc2590_Z.stallGuardThreshold           = 5;
    tmc2590_Z.stallGuardAlarmValue          = 200;
    tmc2590_Z.stallGuardAlarmThreshold      = 200;
    tmc2590_Z.currentScale                  = 1; /* 0 - 31 where 31 is max, 0.25A */
    tmc2590_Z.standStillCurrentScale        = 0; //  2: set 1/2 of full scale, 1/4th of power
    tmc2590_X2.vSense                       = 1; /* 0: Full-scale sense resistor voltage is 325mV. 1: Full-scale sense resistor voltage is 173mV.*/

#else
    /* ZH motor (medium 23HS22) in normal conditions (56steps/mm)*/
    tmc2590_X1.stallGuardThreshold          = 6;
    tmc2590_X1.stallGuardAlarmValue         = 400;
    tmc2590_X1.stallGuardAlarmThreshold     = 200;
    tmc2590_X1.currentScale                 = 31; /* 0 - 31 where 31 is max */
    
    /* ZH motor (medium 23HS22) in normal conditions (56steps/mm)*/
    tmc2590_X2.stallGuardThreshold          = 6;
    tmc2590_X2.stallGuardAlarmValue         = 400;
    tmc2590_X2.stallGuardAlarmThreshold     = 200;
    tmc2590_X2.currentScale                 = 31; /* 0 - 31 where 31 is max */
    
    tmc2590_Y1.stallGuardThreshold          = 3;
    tmc2590_Y2.stallGuardAlarmValue         = 400;
    tmc2590_Y1.stallGuardAlarmThreshold     = 200;
    tmc2590_Y1.currentScale                 = 31; /* 0 - 31 where 31 is max */
    tmc2590_Y1.SlowDecayDuration            = 4;
    tmc2590_Y1.HystStart                    = 5; /* Hysteresis start value, Hysteresis start offset from HEND: %000: 1 %100: 5; %001: 2 %101: 6; %010: 3 %110: 7; %011: 4 %111: 8; Effective: HEND+HSTRT must be 15 */
    tmc2590_Y1.HystEnd                      = 5; /* Hysteresis end (low) value; %0000 ... %1111: Hysteresis is -3, -2, -1, 0, 1, ..., 12 (1/512 of this setting adds to current setting) This is the hysteresis value which becomes used for the hysteresis chopper. */
    tmc2590_Y1.HystDectrement               = 2; /* Hysteresis decrement period setting, in system clock periods: %00: 16; %01: 32; %10: 48; %11: 64 */
    tmc2590_Y1.slopeCtrlLow                 = 3;  // Slope control, low side, Gate driver strength 1 to 7. 7 is maximum current for fastest slopes
    tmc2590_Y1.slopeCtrlHigh                = 3;  // Slope control, high side. Gate driver strength 1 to 7. 7 is maximum current for fastest slopes
    
    tmc2590_Y2.stallGuardThreshold          = 3;
    tmc2590_Y2.stallGuardAlarmValue         = 400;
    tmc2590_Y2.stallGuardAlarmThreshold     = 200;
    tmc2590_Y2.currentScale                 = 31; /* 0 - 31 where 31 is max */
    tmc2590_Y2.SlowDecayDuration            = 4;
    tmc2590_Y2.HystStart                    = 5; /* Hysteresis start value, Hysteresis start offset from HEND: %000: 1 %100: 5; %001: 2 %101: 6; %010: 3 %110: 7; %011: 4 %111: 8; Effective: HEND+HSTRT must be 15 */
    tmc2590_Y2.HystEnd                      = 5; /* Hysteresis end (low) value; %0000 ... %1111: Hysteresis is -3, -2, -1, 0, 1, ..., 12 (1/512 of this setting adds to current setting) This is the hysteresis value which becomes used for the hysteresis chopper. */
    tmc2590_Y2.HystDectrement               = 2; /* Hysteresis decrement period setting, in system clock periods: %00: 16; %01: 32; %10: 48; %11: 64 */
    tmc2590_Y2.slopeCtrlLow                 = 3;  // Slope control, low side, Gate driver strength 1 to 7. 7 is maximum current for fastest slopes
    tmc2590_Y2.slopeCtrlHigh                = 3;  // Slope control, high side. Gate driver strength 1 to 7. 7 is maximum current for fastest slopes
    
    
    /* ZH motor */
    tmc2590_Z.stallGuardThreshold           = 6;
    tmc2590_Z.stallGuardAlarmValue          = 200;
    tmc2590_Z.stallGuardAlarmThreshold      = 200;
    tmc2590_Z.currentScale                  = 31; /* 0 - 31 where 31 is max */

#endif    
    
    stall_guard_calibration_load();

    
    stall_guard_statistics_reset();    
    
    tmc_hw_init();

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



void process_status_of_all_controllers(void){
    /* process all responses and update the current status of controller's parameters */
    //#ifdef SG_SKIP_DEBUG_ENABLED
    //debug_pin_write(1, DEBUG_0_PIN);
    //debug_pin_write(0, DEBUG_0_PIN);
    //#endif

    /* only process those axes that has been scheduled through spi read queue */
    uint8_t axis;
    for (axis=0; axis<N_AXIS; axis++) {
        if (bit_istrue(st_tmc.sg_read_active_axes,bit(axis))) {
            switch (axis){
                case X_AXIS:
                process_status_of_dual_controller(&tmc2590_X1, &tmc2590_X2);
                break;
                
                case Y_AXIS:
                process_status_of_dual_controller(&tmc2590_Y1, &tmc2590_Y2);
                break;
                
                case Z_AXIS:
                process_status_of_single_controller(&tmc2590_Z);
                break;
                
                default:
                break;
            } //switch (axis){
                
            st_tmc.sg_read_active_axes &= ~( 1 << axis ); /* mark this axis as read */

        }//if (bit_istrue(st_tmc.sg_read_active_axes,bit(axis))) {
    } //for (idx=0; idx<N_AXIS; idx++) {
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
            /* check if all 3 bytes are received yet */
            rtl_data_available = serial_rtl_data_available_length();
            if (rtl_data_available >= RTL_TMC_COMMAND_SIZE){
			    /* schedule next TMC execute: indicate to main loop that there is a TMC command to process */
			    system_set_exec_rtl_command_flag(RTL_TMC_COMMAND);		                
            }            
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
				report_status_message(ASMCNC_INVALID_MOTOR_ID);                
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

			/* Off time/MOSFET disable. Duration of slow decay phase. If TOFF is 0, the MOSFETs are shut off. If TOFF is nonzero, slow decay time is a multiple of system clock periods: NCLK= 24 + (32 x TOFF) (Minimum time is 64clocks.), %0000: Driver disable, all bridges off, %0001: 1 (use with TBL of minimum 24 clocks) %0010 ... %1111: 2 ... 15 */
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
				tmc2590->HystStart = value;
				register_value &= ~TMC2590_SET_HSTRT(-1);                        // clear
				register_value |= TMC2590_SET_HSTRT(tmc2590->HystStart);
				tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_CHOPCONF);
				break;

			/* Hysteresis end (low) value; %0000 ... %1111: Hysteresis is -3, -2, -1, 0, 1, ..., 12 (1/512 of this setting adds to current setting) This is the hysteresis value which becomes used for the hysteresis chopper. */
			case SET_HEND:
				/* TMC2590_CHOPCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT];
				tmc2590->HystEnd = value;
				register_value &= ~TMC2590_SET_HEND(-1);                        // clear
				register_value |= TMC2590_SET_HEND(tmc2590->HystEnd);
				tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_CHOPCONF);
				break;

			/* Hysteresis decrement period setting, in system clock periods: %00: 16; %01: 32; %10: 48; %11: 64 */
			case SET_HDEC:
				/* TMC2590_CHOPCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT];
				tmc2590->HystDectrement = value;
				register_value &= ~TMC2590_SET_HDEC(-1);                        // clear
				register_value |= TMC2590_SET_HDEC(tmc2590->HystDectrement);
				tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_CHOPCONF);
				break;

			/* Enable randomizing the slow decay phase duration: 0: Chopper off time is fixed as set by bits tOFF 1: Random mode, tOFF is random modulated by dNCLK= -12 - +3 clocks */
			case SET_RNDTF:
				/* TMC2590_CHOPCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT];
				tmc2590->SlowDecayRandom = value;
				register_value &= ~TMC2590_SET_RNDTF(-1);                        // clear
				register_value |= TMC2590_SET_RNDTF(tmc2590->SlowDecayRandom);
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

			/* energize or shut off the motor completely, for example to let user move turret easier */
			case SET_MOTOR_ENERGIZED:
            {
                uint8_t SlowDecayDuration = tmc2590->SlowDecayDuration;
                if (value == 0){
                    SlowDecayDuration = 0;
                }
				/* TMC2590_CHOPCONF */
				register_value = tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT];				
				register_value &= ~TMC2590_SET_TOFF(-1);                        // clear
				register_value |= TMC2590_SET_TOFF(SlowDecayDuration);
				tmc2590->config->shadowRegister[TMC2590_CHOPCONF | TMC2590_WRITE_BIT] = register_value;
				tmc2590_single_write_route(controller_id, TMC2590_CHOPCONF);
            }			            
			break;

			/* desired stall behaviour: if "true" then stall guard value below the limit will trigger alarm */
			case SET_SG_ALARM:
                st_tmc.stall_alarm_enabled = value;
			break;

			/* 1: reset all calibrations and prepare for new one, 2: complete calibration, compute cal tables and apply correction, 4: print calibration coefficients */
			case SET_CALIBR_MODE:
                if ( (value == TMC_CALIBRATION_INIT) || (value == TMC_CALIBRATION_COMPUTE) || (value == TMC_CALIBRATION_REPORT) )
                    {
                        system_set_exec_tmc_cal_command_flag(value);                
                    }
                    else{
                        report_status_message(ASMCNC_PARAM_ERROR);
                    }                                         
			break;

			default:
				break;
			} //switch (command){

        /* start SPI transfers flushing the queue */
        tmc_kick_spi_processing();

		} // if (crc_in == tmc_command[RTL_TMC_COMMAND_SIZE-1]){
            
		else{ /* crc error */
			report_status_message(ASMCNC_CRC8_ERROR);
		} //else{ /* crc error */

	} // if rtl_data_available != SERIAL_NO_DATA {

} //void execute_TMC_command(){


/* apply working current to motors:
 - when idle: reduce the current through energized motors 
 - when active: apply full scale current  */
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
        if (st_tmc.current_scale_state == CURRENT_SCALE_STANDSTILL) register_value |= TMC2590_SET_CS(tmc2590->standStillCurrentScale);  // set standstill Current scale
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
            if (st_tmc.current_scale_state == CURRENT_SCALE_STANDSTILL) register_value |= TMC2590_SET_CS(tmc2590->standStillCurrentScale);  // set standstill Current scale
            else                                                            register_value |= TMC2590_SET_CS(tmc2590->currentScale);            // set full operational Current scale
            tmc2590->config->shadowRegister[TMC2590_SGCSCONF | TMC2590_WRITE_BIT] = register_value;            
        } //if ( (controller_id == TMC_X1) || (controller_id == TMC_Y1) ){
        
        tmc2590_single_write_route(controller_id, TMC2590_SGCSCONF);            
    } //for (controller_id = TMC_X1; controller_id < TOTAL_TMCS, controller_id++){
    
    /* kick SPI process buffer immediately to apply new current settings before the motors start */
    tmc_kick_spi_processing();

    
} //void tmc_standstill_apply(uint8_t current_scale_standstill_state){

/* reduce the current through energized motors when idle */
void tmc_standstill_on(void){
    /* only apply new current scale if state is different. Prevents muptiple writes of the same command */
    if (st_tmc.current_scale_state != CURRENT_SCALE_STANDSTILL){
        st_tmc.current_scale_state = CURRENT_SCALE_STANDSTILL;    
        tmc_all_current_scale_apply(); /* set standstill Current scale on all motors */
    }      
}

/* bump the current through energized motors back to working level when cycle starts */
void tmc_standstill_off(void){     
    /* only apply new current scale if state is different. Prevents muptiple writes of the same command */
    if (st_tmc.current_scale_state != CURRENT_SCALE_ACTIVE){
        st_tmc.current_scale_state = CURRENT_SCALE_ACTIVE;
        tmc_all_current_scale_apply(); /* set standstill Current scale on all motors */
    }
}



/* ------------------------------ homing engine functions -----------------------------------*/

/* clear limit switch after homing cycle found the end stop*/
void tmc_homing_reset_limits(void){
    
    st_tmc.stall_alarm_enabled = true; /* enable the alarm in case it was disabled */
    
    /* clear limit switch */
    LIMIT_PORT &= ~(LIMIT_MASK); // Normal low operation. Set pin high to trigger ISR
    
}


/* prepare for homing 
- disable SPI regular interrupts
- set current scale to working level
*/
void tmc_homing_mode_set(uint8_t mode){
    if (mode == TMC_MODE_HOMING){
        /* disable timer2 Interrupt for home cycle duration*/
        TIMSK2 &=~(1<<OCIE2A); //Timer/Counter2 Output Compare Match A Interrupt   
        /* apply operational current to motor */
        st_tmc.current_scale_state = CURRENT_SCALE_ACTIVE;
        tmc_all_current_scale_apply(); /* set operational Current scale on all motors */
    }  
    else if (mode == TMC_MODE_IDLE){
        st_tmc.current_scale_state = CURRENT_SCALE_STANDSTILL;
        /* reenable SPI engine timer */
        tmc_all_current_scale_apply(); /* set standstill Current scale on all motors */              
        /* reenable SPI engine timer */
        /* Enable timer2 Interrupt */
        TIMSK2 |= (1<<OCIE2A); //Timer/Counter2 Output Compare Match A Interrupt Enable
    }
  
}  

void tmc_spi_queue_drain_complete(void){
    /* in homing mode homing_sg_read_ongoing flag shall lead to processing the SG values and releasing the homing while loop in the end of tmc_read_sg_and_trigger_limits()*/
    if ( homing_sg_read_ongoing ) {
        process_status_of_all_controllers();
        homing_sg_read_ongoing = false;
    }
}

/* BK: function to replace limits read with SPI actions for homing
 * new homing principle - use StallGuard readings to detect the end stop by increasing the load on the motor: 
 * wait for stepper function st_tmc_fire_SG_read() to raise flag in system_set_exec_tmc_command_flag(command);
 * when sys_rt_exec_tmc_command raised - immediately read SG.
 * homing is done in plain loop outside of main core loop - so usual main signals and indicators cannot be used here, 
 * therefore dumb while loop is used here to wait for completion of SPI read and SG analysis
 * main loop in limits.c "do { //} while (STEP_MASK & axislock);" handles the periodic reads of SG through tmc_read_sg_and_trigger_limits()
 * tmc_read_sg_and_trigger_limits() schedules the read and initiates it while setting "homing_sg_read_ongoing" variable to "True"
 * then while loop stays forever in this function until tmc_spi_queue_drain_complete() is called from last SPI interrupt and SG data is processed
 * tmc_spi_queue_drain_complete() releases the flag "homing_sg_read_ongoing" and homing continues
 * function process_status_of_single_controller() looks for SG trigger point and sets the limit port to corresponding state it alarm need to be triggered
 
*/            
void tmc_read_sg_and_trigger_limits(void){
    
  uint8_t rt_exec; // Temp variable to avoid calling volatile multiple times.
  
  rt_exec = sys_rt_exec_tmc_command;
  
  if (rt_exec) {
      system_clear_exec_tmc_flags(); // Clear all accessory flags. Shall be done after last command in the buffer is processed
   
      /* schedule next SPI read Stall Guard from X motor controllers */
      if (rt_exec & TMC_SPI_READ_SG_X_COMMAND) {
          tmc2590_schedule_read_sg(X_AXIS);
          homing_sg_read_ongoing = true;
      }
      
      /* schedule next SPI read Stall Guard from Y motor controllers */
      if (rt_exec & TMC_SPI_READ_SG_Y_COMMAND) {
          tmc2590_schedule_read_sg(Y_AXIS);
          homing_sg_read_ongoing = true;          
      }
      
      /* schedule next SPI read Stall Guard from Z motor controllers */
      if (rt_exec & TMC_SPI_READ_SG_Z_COMMAND) {
          tmc2590_schedule_read_sg(Z_AXIS);
          homing_sg_read_ongoing = true;
      }

  }//if (rt_exec) {

    /* wait for current stall guard read complete. Delay in a loop is required to let other (ISR) threads to continue */
    while (homing_sg_read_ongoing) delay_us(1); 
}


/* reset all global variables to known init state */
void tmc_globals_reset(void)
{

    /* initialise stepper TMC interface structure */
    /* initialise SG periods to max values (slowest feed) */
    st_tmc.step_period_us[X_AXIS] = 0xFFFF;
    st_tmc.step_period_us[Y_AXIS] = 0xFFFF;
    st_tmc.step_period_us[Z_AXIS] = 0xFFFF;
    /* initialise SG skip counters used to analyse stall based on SG readings */
    st_tmc.SG_skips_counter[X_AXIS] = 0;
    st_tmc.SG_skips_counter[Y_AXIS] = 0;
    st_tmc.SG_skips_counter[Z_AXIS] = 0;
    /* initialise step counters used to fire SG readings */
    st_tmc.step_counter[X_AXIS] = 0;
    st_tmc.step_counter[Y_AXIS] = 0;
    st_tmc.step_counter[Z_AXIS] = 0;

    st_tmc.current_scale_state  = CURRENT_SCALE_ACTIVE;  /* global holding effective current scale */
    homing_sg_read_ongoing      = false;                 /* global flag indicating stall guard read process is ongoing */
    st_tmc.sg_read_active_axes  = 0;                     /* global variable to hold current axis that is being homed */
    st_tmc.stall_alarm_enabled  = true;                  /* global holding desired stall behaviour: if "true" then stall guard value below the limit will trigger alarm      */
    st_tmc.calibration_enabled  = false;
    
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


