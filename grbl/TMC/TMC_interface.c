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
TMC2590TypeDef tmc[TOTAL_TMCS];
FlashTMCconfig flashTMCconfig;


uint8_t homing_sg_read_ongoing = false;             /* global flag indicating homing is ongoing */

/************************************************ all motors ***********************************************/

  

/* schedule periodic read of all values */
void tmc2590_schedule_read_all(void){
#if defined(TMC_5_CONTROLLERS)
    tmc2590_dual_read_all(&tmc[TMC_X1], &tmc[TMC_X2]);
    tmc2590_dual_read_all(&tmc[TMC_Y1], &tmc[TMC_Y2]);
#elif defined(TMC_3_CONTROLLERS)
    tmc2590_single_read_all(&tmc[TMC_X1]);
    tmc2590_single_read_all(&tmc[TMC_Y1]);
#elif defined(TMC_2_CONTROLLERS)
    tmc2590_single_read_all(&tmc[TMC_X1]);
#endif
#if !defined (TMC_ALL_STANDALONE)
    tmc2590_single_read_all(&tmc[TMC_Z]);
    /* update st_tmc.sg_read_active_axes with all axes to be processed */
    st_tmc.sg_read_active_axes |= ( ( 1 << X_AXIS ) | ( 1 << Y_AXIS ) | ( 1 << Z_AXIS ) );
    /* start SPI transfers flushing the queue */
    tmc_kick_spi_processing();
#endif
}


void read_SG_standalone(uint8_t motor, uint8_t limit_bit){
    /* read SG pin and apply SG value accordingly */
    uint32_t SG_value = 1023; /* Stall guard value to report in case of no SG pin detection: pin is low, TMC chip reports all OK */
    uint8_t lim_pin_state   = limits_get_state();

    if (bit_istrue(lim_pin_state,bit(limit_bit)))  { /* limit pin is high, TMC chip reports stall */
        SG_value = 11;
    }
    tmc[motor].response[TMC2590_RESPONSE1] = (SG_value << 10);
            
    /* indicate to main loop to process all responses and update the current status of controller's parameters */
    system_set_exec_tmc_command_flag(TMC_SPI_PROCESS_COMMAND);
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
        case X_AXIS:{
#if defined(TMC_5_CONTROLLERS)
            tmc2590_dual_read_sg(&tmc[TMC_X1], &tmc[TMC_X2]);
#elif defined(TMC_3_CONTROLLERS) || defined(TMC_2_CONTROLLERS)
            tmc2590_single_read_sg(&tmc[TMC_X1]);
#elif defined(TMC_ALL_STANDALONE)
            read_SG_standalone(TMC_X1, X_AXIS_SG);
#endif
        }
        break;

        case Y_AXIS:{
            
#if defined(TMC_5_CONTROLLERS)
            tmc2590_dual_read_sg(&tmc[TMC_Y1], &tmc[TMC_Y2]);
#elif defined(TMC_3_CONTROLLERS)
            tmc2590_single_read_sg(&tmc[TMC_Y1]);
#elif defined(TMC_2_CONTROLLERS) || defined(TMC_ALL_STANDALONE)
            read_SG_standalone(TMC_Y1, Y_AXIS_SG);
#endif
        }
        break;

        case Z_AXIS:{        
#if defined(TMC_5_CONTROLLERS) || defined(TMC_3_CONTROLLERS) || defined(TMC_2_CONTROLLERS)
            tmc2590_single_read_sg(&tmc[TMC_Z]);
#elif defined(TMC_ALL_STANDALONE)
            read_SG_standalone(TMC_Z, Z_AXIS_SG);
#endif
        }
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
            return &tmc[TMC_X1];
        case TMC_X2:
            return &tmc[TMC_X2];
        case TMC_Y1:
            return &tmc[TMC_Y1];
        case TMC_Y2:
            return &tmc[TMC_Y2];
        case TMC_Z:
            return &tmc[TMC_Z];

        default:
            break;
    } //switch (controller){
    return &tmc[TMC_X1]; /* return first controller in case of a wrong parameter supplied */
}

/* initialise motors with stored parameters */
void tmc_restore_all(void){
#if defined(TMC_5_CONTROLLERS)
    tmc2590_dual_restore(&tmc[TMC_X1], &tmc[TMC_X2]);
    tmc2590_dual_restore(&tmc[TMC_Y1], &tmc[TMC_Y2]);
#elif defined(TMC_3_CONTROLLERS) 
    tmc2590_single_restore(&tmc[TMC_X1]);
    tmc2590_single_restore(&tmc[TMC_Y1]);    
#elif defined(TMC_2_CONTROLLERS)
    tmc2590_single_restore(&tmc[TMC_X1]);
#endif
#if !defined (TMC_ALL_STANDALONE)
    tmc2590_single_restore(&tmc[TMC_Z]);
#endif
}

void init_TMC(void){

    /* init TMC */
    uint8_t channel_X = SPI_CS_X_PIN;
    uint8_t channel_Y = SPI_CS_Y_PIN;
    uint8_t channel_Z = SPI_CS_Z_PIN;

    tmc_globals_reset();

    tmc[TMC_X1].stallGuardAlarmValue         = 100; /* when current SG reading is lower than this value corresponded axis alarm will be triggered */

    /* individual motor settings */

    tmc[TMC_X1].thisAxis                     = X_AXIS;
    tmc[TMC_X2].thisAxis                     = X_AXIS;
    tmc[TMC_Y1].thisAxis                     = Y_AXIS;
    tmc[TMC_Y2].thisAxis                     = Y_AXIS;
    tmc[TMC_Z].thisAxis                      = Z_AXIS;

#ifdef RIGGY
    /* no motor */
    tmc[TMC_X1].stallGuardAlarmValue         = 300;
    /* riggy motor (smallest 17HS15-0404S) idle SG ~500, loaded ~400  at 3000mm/min on X with 177steps/mm*/
    tmc[TMC_X2].stallGuardAlarmValue         = 400;
    tmc[TMC_Y1].stallGuardAlarmValue         = 400;
    tmc[TMC_Y2].stallGuardAlarmValue         = 400;
    tmc[TMC_Z].stallGuardAlarmValue          = 200;
#else
    tmc[TMC_X1].stallGuardAlarmValue         = 300;
    ///* ZH motor (medium 23HS22) in normal conditions (56steps/mm)*/
    tmc[TMC_X2].stallGuardAlarmValue         = 300;
    tmc[TMC_Y1].stallGuardAlarmValue         = 400;
    tmc[TMC_Y2].stallGuardAlarmValue         = 400;
    tmc[TMC_Z].stallGuardAlarmValue          = 200;
#endif


    /* initialise wanted variables */
    tmc[TMC_X1].channel      = channel_X;
    tmc[TMC_X2].channel      = channel_X;
    tmc[TMC_Y1].channel      = channel_Y;
    tmc[TMC_Y2].channel      = channel_Y;
    tmc[TMC_Z].channel       = channel_Z;

    tmc_load_settings();

    uint8_t controller_id;
    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
        tmc[controller_id].respIdx              = DEFAULT_TMC_READ_SELECT; // very first resp index would be DEFAULT_TMC_READ_SELECT
        tmc[controller_id].SlowDecayDuration    = TMC2590_GET_TOFF(tmc[controller_id].shadowRegister[TMC2590_CHOPCONF]);
        tmc[controller_id].thisMotor            = controller_id;
    } //for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){

    tmc_load_stall_guard_calibration();

    stall_guard_statistics_reset();
    
    tmc_restore_all();

    tmc_hw_init();

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
#if defined(TMC_5_CONTROLLERS)
                process_status_of_dual_controller(&tmc[TMC_X1], &tmc[TMC_X2]);
#elif defined(TMC_3_CONTROLLERS) || defined(TMC_2_CONTROLLERS) || defined(TMC_ALL_STANDALONE)
                process_status_of_single_controller(&tmc[TMC_X1]);
#endif    
                break;

                case Y_AXIS:
#if defined(TMC_5_CONTROLLERS)
                process_status_of_dual_controller(&tmc[TMC_Y1], &tmc[TMC_Y2]);
#elif defined(TMC_3_CONTROLLERS) || defined(TMC_2_CONTROLLERS) || defined(TMC_ALL_STANDALONE)
                process_status_of_single_controller(&tmc[TMC_Y1]);
#endif    
                break;

                case Z_AXIS:
                process_status_of_single_controller(&tmc[TMC_Z]);
                break;

                default:
                break;
            } //switch (axis){

            st_tmc.sg_read_active_axes &= ~( 1 << axis ); /* mark this axis as read */

        }//if (bit_istrue(st_tmc.sg_read_active_axes,bit(axis))) {
    } //for (idx=0; idx<N_AXIS; idx++) {
}


void process_individual_command(uint8_t controller_id, uint8_t command, uint32_t value){

    TMC2590TypeDef *tmc2590;
    uint32_t register_value;

    tmc2590 = get_TMC_controller(controller_id);

        switch (command){

        case SET_DRVCTRL:
            /* TMC2590_DRVCTRL */
            tmc2590->shadowRegister[TMC2590_DRVCTRL] = value;
            tmc2590_single_write_route(controller_id, TMC2590_DRVCTRL);
            break;

        case SET_CHOPCONF:
            /* TMC2590_CHOPCONF */
            tmc2590->shadowRegister[TMC2590_CHOPCONF] = value;
            tmc2590_single_write_route(controller_id, TMC2590_CHOPCONF);
            break;

        case SET_SMARTEN:
            /* TMC2590_SMARTEN */
            tmc2590->shadowRegister[TMC2590_SMARTEN] = value;
            tmc2590_single_write_route(controller_id, TMC2590_SMARTEN);
            break;

        case SET_SGCSCONF:
            /* TMC2590_SGCSCONF */
            tmc2590->shadowRegister[TMC2590_SGCSCONF] = value;
            tmc2590_single_write_route(controller_id, TMC2590_SGCSCONF);
            break;

        case SET_DRVCONF:
            /* TMC2590_DRVCONF */
            /* keep original rdsel to avoid interference with internal TMC reading state logic */
            {
                uint8_t rdsel;
                rdsel = TMC2590_GET_RDSEL(tmc2590->shadowRegister[TMC2590_DRVCONF]);/* copy rdsel */
                value &= ~TMC2590_SET_RDSEL(-1);                                    /* clear RDSEL bits */
                value |= TMC2590_SET_RDSEL(rdsel);                                  /* set rdsel  */
                tmc2590->shadowRegister[TMC2590_DRVCONF] = value;
            }
            tmc2590_single_write_route(controller_id, TMC2590_DRVCONF);
            break;

        /* set the current scale applied when no pulses are detected on the given axis */
        case SET_IDLE_CURRENT:
            value = value & 0xFF;
            tmc2590->standStillCurrentScale = value;
            tmc_all_current_scale_apply();
            break;


        /* set the current scale applied when no pulses are detected on the given axis */
        case SET_ACTIVE_CURRENT:
            value = value & 0xFF;
            tmc2590->activeCurrentScale = value;
            tmc_all_current_scale_apply();
            break;

        /* energize or shut off the motor completely, for example to let user move turret easier */
        case SET_MOTOR_ENERGIZED:
        {
            value = value & 0xFF;
            uint8_t SlowDecayDuration = tmc2590->SlowDecayDuration;
            if (value == 0){
                SlowDecayDuration = 0;
            }
            /* TMC2590_CHOPCONF */
            register_value = tmc2590->shadowRegister[TMC2590_CHOPCONF];
            register_value &= ~TMC2590_SET_TOFF(-1);                        // clear
            register_value |= TMC2590_SET_TOFF(SlowDecayDuration);
            tmc2590->shadowRegister[TMC2590_CHOPCONF] = register_value;
            tmc2590_single_write_route(controller_id, TMC2590_CHOPCONF);
        }
            break;

        /* set the stallGuardAlarmThreshold: when current SG reading is lower than calibrated by this value corresponded axis alarm will be triggered */
        case SET_SG_ALARM_TRSHLD:
            tmc2590->stallGuardAlarmThreshold = value;
            break;

        /* set the correction for temperatures other than calibration */
        case SET_THERMAL_COEFF:
            tmc2590->gradient_per_Celsius = value;
            break;

        /* set the correction for temperatures other than calibration */
        case SET_MAX_SG_STEP_US:
            tmc2590->max_step_period_us_to_read_SG = value;
            min_step_period_idx_compute();
            break;

        default:
            report_status_message(ASMCNC_COMMAND_ERROR);
        break;
    } //switch (command){
}

void process_global_command(uint8_t command, uint32_t value){

        switch (command){

        /* desired stall behaviour: if "true" then stall guard value below the limit will trigger alarm */
        case SET_SG_ALARM:
            value = value & 0xFF;
            st_tmc.stall_alarm_enabled = value;
        break;

        /* 1: reset all calibrations and prepare for new one, 2: complete calibration, compute cal tables and apply correction, 4: print calibration coefficients */
        case SET_CALIBR_MODE:
            value = value & 0xFF;
            if ( (value == TMC_CALIBRATION_INIT) || (value == TMC_CALIBRATION_COMPUTE) || (value == TMC_CALIBRATION_REPORT) || (value == TMC_CALIBRATION_INIT_X) || (value == TMC_CALIBRATION_INIT_Y) || (value == TMC_CALIBRATION_INIT_Z) )
                {
                    system_set_exec_tmc_cal_command_flag(value);
                }
                else{
                    report_status_message(ASMCNC_PARAM_ERROR);
                }
        break;
        
        case UPLOAD_CALIBR_VALUE:    // 1: upload calibration from host. Must be preceded by TMC_CALIBRATION_INIT_xxx             
        {
            uint8_t motor;
            uint8_t idx;
            uint16_t stallGuardLoadedValue;
            motor                   = (uint8_t) ( (uint32_t)(value & 0xFF000000)>>24 );
            idx                     = (uint8_t) ( (uint32_t)(value & 0x00FF0000)>>16 );
            stallGuardLoadedValue   = (uint16_t)( (uint32_t)(value & 0x0000FFFF)     );
            if ( ( motor < TOTAL_TMCS ) && ( idx < TMC_SG_PROFILE_POINTS + 1) && ( stallGuardLoadedValue < 10000) )
            {
                tmc_store_calibration_point_from_host(	motor, idx, stallGuardLoadedValue);
            }
            else{
                report_status_message(ASMCNC_PARAM_ERROR);
            }
        }            
        break;
            
        /* print out 2560 statistics */
        case GET_STATISTICS:
            system_set_exec_tmc_cal_command_flag(TMC_STATISTICS_REPORT);
        break;

        /* print out register state for all motors */
        case GET_REGISTERS:
            system_set_exec_tmc_cal_command_flag(TMC_REGISTERS_REPORT);
        break;

        /* restore all TMC default settings from flash */
        case RESTORE_TMC_DEFAULTS:
        {
            printPgmString(PSTR("TMC settings Restore\n"));
            restore_TMC_defaults();
            /* apply loaded settings to each controller */
            apply_TMC_settings_from_flash();
            /* store the default settings */
            tmc_store_settings();
            /* initialise motors with default parameters */
            tmc_restore_all();
            /* start SPI transfers flushing the queue */
            tmc_kick_spi_processing();
            //tmc_standstill_off();
            st_tmc.current_scale_state = CURRENT_SCALE_ACTIVE;

        }
        break;

        /* store existing (tuned) paraeters to the flash */
        case STORE_TMC_PARAMS:
            tmc_store_settings();
        break;

        /*  value = 0x10: disable WD feed; other value: report EEPROM dump */
        case WDT_TMC_TEST:
            value = value & 0xFF;
            if (value == 0x10){
                TIMSK2 &= ~(1<<OCIE2A); // disable timer that feeds the dog
                //stay here forever emulating stuck in a loop
                while(1) {;}
            }
            else{
                printPgmString(PSTR("Dump:\n"));
                report_last_wdt_addresses();
                dumpMemory();
            }

        break;

        /* report list of last stalls with associated freeze frame */
        case REPORT_STALLS:
            report_stall_info();
        break;

        default:
            report_status_message(ASMCNC_COMMAND_ERROR);
        break;
    } //switch (command){


}



void execute_TMC_command(uint8_t* p_data, uint8_t data_len){


    uint8_t tmc_command[TMC_REG_CMD_LENGTH+1];
    memset(tmc_command, 0, TMC_REG_CMD_LENGTH+1);
    memcpy(tmc_command, p_data, data_len);

    uint8_t controller_id = (tmc_command[0] & ~MOTOR_OFFSET_MASK) >> TMC_COMMAND_BIT_SIZE;
    uint8_t command = tmc_command[0] & MOTOR_OFFSET_MASK;
    /* value comes as big endian array to accommodate for variable length */
    uint32_t value = _8_32(tmc_command[4], tmc_command[3], tmc_command[2], tmc_command[1]);

    if (controller_id < TOTAL_TMCS){
        process_individual_command(controller_id, command, value);
    }
    else{
        command = tmc_command[0];
        process_global_command(command, value);
        //report_status_message(ASMCNC_INVALID_MOTOR_ID);
        return;
    }

    /* start SPI transfers flushing the queue */
    tmc_kick_spi_processing();

}


///* fetch TMC host command from rtl serial buffer and execute */
//void execute_TMC_command(){
    ///* fetch 3 bytes from the buffer, calculate CRC and execute*/
    //uint8_t tmc_command[RTL_TMC_COMMAND_SIZE], idx, len;
    //
    //memset(tmc_command, 0, RTL_TMC_COMMAND_SIZE);
//
    ///* check if data is available from rtl_serial bufer */
    //uint8_t rtl_data_available = serial_rtl_data_available();
    //
    //if ( (rtl_data_available != SERIAL_NO_DATA) && (rtl_data_available != SERIAL_DATA_INCOMPLETE) ) {
        ///* if enough bytes received yet process the command */
        //len = rtl_data_available;
        ///* if the function serial_rtl_data_available() returnced other than SERIAL_NO_DATA it should be a length of the next command */
        //for (idx = 0; idx < len; idx++){
            //tmc_command[idx] = serial_read_rtl();
        //}
    //
        ///* check if more data is available from rtl_serial buffer */
        //rtl_data_available = serial_rtl_data_available();
        //if ( (rtl_data_available != SERIAL_NO_DATA) && (rtl_data_available != SERIAL_DATA_INCOMPLETE) ) {
            ///* schedule next TMC execute: indicate to main loop that there is a TMC command to process */
            //system_set_exec_rtl_command_flag(RTL_TMC_COMMAND);
        //};
//
//
        ///* calculate CRC 8 on the command and value and compare with checksum */
        //uint8_t crc_in;
        //crc_in = crc8x_fast(0, tmc_command, len-1);
        //if (crc_in == tmc_command[len-1]){
            //uint8_t controller_id = (tmc_command[1] & ~MOTOR_OFFSET_MASK) >> TMC_COMMAND_BIT_SIZE;
            //uint8_t command = tmc_command[1] & MOTOR_OFFSET_MASK;
            ///* value comes as big endian array to accommodate for variable length */
            //uint32_t value = _8_32(tmc_command[5], tmc_command[4], tmc_command[3], tmc_command[2]);
//
            //if (controller_id < TOTAL_TMCS){
                //process_individual_command(controller_id, command, value);
            //}
            //else{
                //command = tmc_command[1];
                //process_global_command(command, value);
                ////report_status_message(ASMCNC_INVALID_MOTOR_ID);
                //return;
            //}
//
        ///* start SPI transfers flushing the queue */
        //tmc_kick_spi_processing();
//
        //} // if (crc_in == tmc_command[RTL_TMC_COMMAND_SIZE-1]){
            //
        //else{ /* crc error */
            //report_status_message(ASMCNC_CRC8_ERROR);
        //} //else{ /* crc error */
//
    //} // if rtl_data_available != SERIAL_NO_DATA {
//
//} //void execute_TMC_command(){


/* apply working current to motors:
 - when idle: reduce the current through energized motors
 - when active: apply full scale current  */
void tmc_all_current_scale_apply( void ){

    uint8_t controller_id;
    TMC2590TypeDef *tmc2590;
    uint32_t register_value;

    /* reduce current in each TMC controller */
#if defined(TMC_5_CONTROLLERS)
    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
#elif defined(TMC_3_CONTROLLERS)
    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id+=2){
#elif defined(TMC_2_CONTROLLERS)
    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id+=4){
#elif defined(TMC_ALL_STANDALONE)
    return;
    {
#endif

        tmc2590 = get_TMC_controller(controller_id);
        /* TMC2590_SGCSCONF */
        register_value = tmc2590->shadowRegister[TMC2590_SGCSCONF];
        register_value &= ~TMC2590_SET_CS(-1);                           // clear Current scale bits
        if (st_tmc.current_scale_state == CURRENT_SCALE_STANDSTILL) register_value |= TMC2590_SET_CS(tmc2590->standStillCurrentScale);  // set standstill Current scale
        else                                                            register_value |= TMC2590_SET_CS(tmc2590->activeCurrentScale);            // set full operational Current scale
        tmc2590->shadowRegister[TMC2590_SGCSCONF] = register_value;

#if defined(TMC_5_CONTROLLERS)
        /* below if statement is to ensure both dual controllers are being written in one transaction to speed up the execution */
        if ( (controller_id == TMC_X1) || (controller_id == TMC_Y1) ){
            /* choose second pair and prepare for dual write */
            controller_id++;
            tmc2590 = get_TMC_controller(controller_id);
            /* TMC2590_SGCSCONF */
            register_value = tmc2590->shadowRegister[TMC2590_SGCSCONF];
            register_value &= ~TMC2590_SET_CS(-1);                           // clear Current scale bits
            if (st_tmc.current_scale_state == CURRENT_SCALE_STANDSTILL) register_value |= TMC2590_SET_CS(tmc2590->standStillCurrentScale);  // set standstill Current scale
            else                                                            register_value |= TMC2590_SET_CS(tmc2590->activeCurrentScale);            // set full operational Current scale
            tmc2590->shadowRegister[TMC2590_SGCSCONF] = register_value;
            } //if ( (controller_id == TMC_X1) || (controller_id == TMC_Y1) ){
#endif

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
#ifdef TMC_SG_BASED_HOMING
    st_tmc.stall_alarm_enabled = true; /* enable the alarm in case it was disabled */

    /* clear limit switch */
    LIMIT_PORT &= ~(LIMIT_MASK_OUTPUT); // Normal low operation. Set pin high to trigger ISR
#endif //#ifdef TMC_SG_BASED_HOMING
}


/* prepare for homing
- disable SPI regular interrupts
- set current scale to working level
*/
void tmc_homing_mode_set(uint8_t mode){
    if (mode == TMC_MODE_HOMING){
        allow_periodic_TMC_poll(0); /* disable TMC polls for home cycle duration*/
        /* apply operational current to motor */
        st_tmc.current_scale_state = CURRENT_SCALE_ACTIVE;
        tmc_all_current_scale_apply(); /* set operational Current scale on all motors */
    }
    else if (mode == TMC_MODE_IDLE){
        st_tmc.current_scale_state = CURRENT_SCALE_STANDSTILL;
        /* reenable SPI engine timer */
        tmc_all_current_scale_apply(); /* set standstill Current scale on all motors */
        allow_periodic_TMC_poll(1); /* reenable SPI TMC polls for home cycle duration*/
    }

}

void tmc_spi_queue_drain_complete(void){
#ifdef TMC_SG_BASED_HOMING
    /* in homing mode homing_sg_read_ongoing flag shall lead to processing the SG values and releasing the homing while loop in the end of tmc_read_sg_and_trigger_limits()*/
    if ( homing_sg_read_ongoing ) {
        process_status_of_all_controllers();
        homing_sg_read_ongoing = false;
    }
#endif //#ifdef TMC_SG_BASED_HOMING
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
#ifdef TMC_SG_BASED_HOMING
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
#endif //#ifdef TMC_SG_BASED_HOMING
}


/* reset all global variables to known init state */
void tmc_globals_reset(void)
{

    /* initialise stepper TMC interface structure */
    /* initialise SG periods to max values (slowest feed) */
    st_tmc.step_period_idx[X_AXIS] = 0;
    st_tmc.step_period_idx[Y_AXIS] = 0;
    st_tmc.step_period_idx[Z_AXIS] = 0;
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
        tmc2590->stallGuardDelta            = -999;
        tmc2590->stallGuardDeltaAxis        = -999;
#ifdef SG_SAMPLE_FILTERING_ENABLED
        tmc2590->stallGuardDeltaPast        = -999;
        tmc2590->stallGuardDeltaAxisPast    = -999;
#endif  
#ifdef SG_AVG_OVER_REPORT_ENABLED
        tmc2590->stallGuardDeltaSum         = 0;
        tmc2590->stallGuardDeltaCount       = 0;
#endif       
        
    }
}

void restore_TMC_defaults(void){

    uint8_t controller_id;
    uint8_t reg_idx;

    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
        // Reset with default values
        /* init default calibration values */
        for(reg_idx = 0; reg_idx < TMC2590_REGISTER_COUNT; reg_idx++)
        {
            flashTMCconfig.registerState[controller_id][reg_idx] = tmc2590_defaultRegisterResetState[controller_id][reg_idx];
        }
        flashTMCconfig.stallGuardAlarmThreshold     [controller_id] = tmc2590_defaultStallGuardAlarmThreshold   [controller_id];
        flashTMCconfig.gradient_per_Celsius         [controller_id] = tmc2590_defaultTemperatureCoefficient     [controller_id];
        flashTMCconfig.max_step_period_us_to_read_SG[controller_id] = default_max_step_period_us_to_read_SG     [controller_id];
        flashTMCconfig.standStillCurrentScale       [controller_id] = tmc2590_defaultStandStillCurrentScale     [controller_id];
        flashTMCconfig.activeCurrentScale           [controller_id] = tmc2590_defaultActiveCurrentScale         [controller_id];
        

    } //for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
}

void apply_TMC_settings_from_flash(void){

    uint8_t controller_id;
    uint8_t reg_idx;

    /* apply loaded settings to each controller */

    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
        for(reg_idx = 0; reg_idx < TMC2590_REGISTER_COUNT; reg_idx++)
        {
            tmc[controller_id].shadowRegister[reg_idx] = flashTMCconfig.registerState[controller_id][reg_idx];
        }
        tmc[controller_id].stallGuardAlarmThreshold         = flashTMCconfig.stallGuardAlarmThreshold       [controller_id];
        tmc[controller_id].gradient_per_Celsius             = flashTMCconfig.gradient_per_Celsius           [controller_id];
        tmc[controller_id].max_step_period_us_to_read_SG    = flashTMCconfig.max_step_period_us_to_read_SG  [controller_id];
        tmc[controller_id].standStillCurrentScale           = flashTMCconfig.standStillCurrentScale         [controller_id];
        tmc[controller_id].activeCurrentScale               = flashTMCconfig.activeCurrentScale             [controller_id];

    } //for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){

}

void tmc_load_settings(void){

    uint8_t load_successful = true;

    if (!(memcpy_from_eeprom_with_checksum((char*)&flashTMCconfig, EEPROM_ADDR_TMC_SETTINS, sizeof(flashTMCconfig)))) {
        load_successful = false;
        /* if CRC is wrong then load default config */
        restore_TMC_defaults();
    } //if (!(memcpy_from_eeprom_with_checksum((char*)&flashTMCcalibration, EEPROM_ADDR_TMC_CALIBRATION, sizeof(FlashTMCcalibration)))) {

    /* apply loaded settings to each controller */
    apply_TMC_settings_from_flash();

    /* if load did not find correct settings store the defualt settings */
    if (!load_successful) {
        tmc_store_settings();
    }

}


void tmc_store_settings(void){

    /* update flashTMCconfig with latest states */
    uint8_t controller_id;
    uint8_t reg_idx;
    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
            for(reg_idx = 0; reg_idx < TMC2590_REGISTER_COUNT; reg_idx++)
            {
                flashTMCconfig.registerState[controller_id][reg_idx] = tmc[controller_id].shadowRegister[reg_idx];
            }
            flashTMCconfig.stallGuardAlarmThreshold     [controller_id] = tmc[controller_id].stallGuardAlarmThreshold;
            flashTMCconfig.gradient_per_Celsius         [controller_id] = tmc[controller_id].gradient_per_Celsius;
            flashTMCconfig.max_step_period_us_to_read_SG[controller_id] = tmc[controller_id].max_step_period_us_to_read_SG;
            flashTMCconfig.standStillCurrentScale       [controller_id] = tmc[controller_id].standStillCurrentScale;
            flashTMCconfig.activeCurrentScale           [controller_id] = tmc[controller_id].activeCurrentScale;
    } //for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){

    /* save flashTMCconfig  to eeprom */
    memcpy_to_eeprom_with_checksum(EEPROM_ADDR_TMC_SETTINS, (char*)&flashTMCconfig, sizeof(FlashTMCconfig));

}

void tmc_report_registers(void)
{
    uint8_t controller_id;
    uint8_t reg_idx;
    for (controller_id = TMC_X1; controller_id < TOTAL_TMCS; controller_id++){
        /* print out register state for this motor */
        printPgmString(PSTR(BK_INITIATOR));
        printPgmString(PSTR("TREG:"));
        printInteger( tmc[controller_id].thisMotor );
        for (reg_idx=TMC2590_DRVCTRL; reg_idx <= TMC2590_DRVCONF; reg_idx++){
            printPgmString(PSTR(","));
            printInteger( tmc[controller_id].shadowRegister[reg_idx] );
        }
        printPgmString(PSTR(","));
        printInteger( tmc[controller_id].activeCurrentScale );
        printPgmString(PSTR(","));
        printInteger( tmc[controller_id].standStillCurrentScale );
        printPgmString(PSTR(","));
        printInteger( tmc[controller_id].stallGuardAlarmThreshold );
        printPgmString(PSTR(","));
        printInteger( get_step_period_us_to_read_SG(controller_id) );
        printPgmString(PSTR(","));
        printInteger( tmc[controller_id].gradient_per_Celsius );
        printPgmString(PSTR(BK_TERMINATOR));
        
    }
}


void tmc_store_stall_info(uint8_t  lastStallsMotor, uint16_t lastStallsSG, uint16_t lastStallsSGcalibrated,  uint16_t lastStallsStepUs){
    if ( !homing_sg_read_ongoing ) {
        /* store stall only if it is not due to homing, where stall is used to detect the end stop */
        store_stall_info(lastStallsMotor, lastStallsSG, lastStallsSGcalibrated,  lastStallsStepUs);

        printPgmString(PSTR(BK_INITIATOR));
        printPgmString(PSTR("SGALARM:"));
        printInteger( lastStallsMotor);
        printPgmString(PSTR(","));
        printInteger( lastStallsStepUs ); /* actual step in us */
        printPgmString(PSTR(","));
        printInteger( lastStallsSG);
        printPgmString(PSTR(","));
        printInteger( lastStallsSGcalibrated );
        printPgmString(PSTR(BK_TERMINATOR));
        
    }//if ( !homing_sg_read_ongoing ) {
}

