// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* \li File:               eeprom.c
* \li Compiler:           IAR EWAAVR 3.10c
* \li Support mail:       avr@atmel.com
*
* \li Supported devices:  All devices with split EEPROM erase/write
*                         capabilities can be used.
*                         The example is written for ATmega48.
*
* \li AppNote:            AVR103 - Using the EEPROM Programming Modes.
*
* \li Description:        Example on how to use the split EEPROM erase/write
*                         capabilities in e.g. ATmega48. All EEPROM
*                         programming modes are tested, i.e. Erase+Write,
*                         Erase-only and Write-only.
*
*                         $Revision: 1.6 $
*                         $Date: Friday, February 11, 2005 07:16:44 UTC $
****************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include "grbl.h"

/* These EEPROM bits have different names on different devices. */
#ifndef EEPE
		#define EEPE  EEWE  //!< EEPROM program/write enable.
		#define EEMPE EEMWE //!< EEPROM master program/write enable.
#endif

/* Define to reduce code size. */
#define EEPROM_IGNORE_SELFPROG //!< Remove SPM flag polling.

static uint32_t localRunTimeSeconds;

FlashStat flashStatistics;
float totalTravelMillimeters = 0; /* accumulator for accurate tracking of the distance */


/*! \brief  Read byte from EEPROM.
 *
 *  This function reads one byte from a given EEPROM address.
 *
 *  \note  The CPU is halted for 4 clock cycles during EEPROM read.
 *
 *  \param  addr  EEPROM address to read from.
 *  \return  The byte read from the EEPROM address.
 */
unsigned char eeprom_get_char( unsigned int addr )
{
	do {} while( EECR & (1<<EEPE) ); // Wait for completion of previous write.
	EEAR = addr; // Set EEPROM address register.
	EECR = (1<<EERE); // Start EEPROM read operation.
	return EEDR; // Return the byte read from EEPROM.
}

/*! \brief  Write byte to EEPROM.
 *
 *  This function writes one byte to a given EEPROM address.
 *  The differences between the existing byte and the new value is used
 *  to select the most efficient EEPROM programming mode.
 *
 *  \note  The CPU is halted for 2 clock cycles during EEPROM programming.
 *
 *  \note  When this function returns, the new EEPROM value is not available
 *         until the EEPROM programming time has passed. The EEPE bit in EECR
 *         should be polled to check whether the programming is finished.
 *
 *  \note  The EEPROM_GetChar() function checks the EEPE bit automatically.
 *
 *  \param  addr  EEPROM address to write to.
 *  \param  new_value  New EEPROM value.
 */
void eeprom_put_char( unsigned int addr, unsigned char new_value )
{
    
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(1, DEBUG_2_PIN);
#endif
    
	char old_value; // Old EEPROM value.
	char diff_mask; // Difference mask, i.e. old value XOR new value.

	do {} while( EECR & (1<<EEPE) ); // Wait for completion of previous write.
	#ifndef EEPROM_IGNORE_SELFPROG
	do {} while( SPMCSR & (1<<SELFPRGEN) ); // Wait for completion of SPM.
	#endif
    

	cli(); // Ensure atomic operation for the write operation.
	
	EEAR = addr; // Set EEPROM address register.
	EECR = (1<<EERE); // Start EEPROM read operation.
	old_value = EEDR; // Get old EEPROM value.
	diff_mask = old_value ^ new_value; // Get bit differences.
	
	// Check if any bits are changed to '1' in the new value.
	if( diff_mask & new_value ) {
		// Now we know that _some_ bits need to be erased to '1'.
		
		// Check if any bits in the new value are '0'.
		if( new_value != 0xff ) {
			// Now we know that some bits need to be programmed to '0' also.
			
			EEDR = new_value; // Set EEPROM data register.
			EECR = (1<<EEMPE) | // Set Master Write Enable bit...
			       (0<<EEPM1) | (0<<EEPM0); // ...and Erase+Write mode.
			EECR |= (1<<EEPE);  // Start Erase+Write operation.
		} else {
			// Now we know that all bits should be erased.

			EECR = (1<<EEMPE) | // Set Master Write Enable bit...
			       (1<<EEPM0);  // ...and Erase-only mode.
			EECR |= (1<<EEPE);  // Start Erase-only operation.
		}
	} else {
		// Now we know that _no_ bits need to be erased to '1'.
		
		// Check if any bits are changed from '1' in the old value.
		if( diff_mask ) {
			// Now we know that _some_ bits need to the programmed to '0'.
			
			EEDR = new_value;   // Set EEPROM data register.
			EECR = (1<<EEMPE) | // Set Master Write Enable bit...
			       (1<<EEPM1);  // ...and Write-only mode.
			EECR |= (1<<EEPE);  // Start Write-only operation.
		}
	}
	
	sei(); // Restore interrupt flag state.
    
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(0, DEBUG_2_PIN);
#endif
    
}

// Extensions added as part of Grbl 


void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size) {

  unsigned char checksum = crc8x_fast(0, (uint8_t *) source, size);

  for(; size > 0; size--) { 
    eeprom_put_char(destination++, *(source++)); 
  }
  eeprom_put_char(destination, checksum);
}

int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size) {

  char * destination_copy = destination;
  unsigned int size_copy = size;
  unsigned char data, checksum = 0;
  
  for(; size > 0; size--) { 
    data = eeprom_get_char(source++);
    *(destination++) = data; 
  }
  
  checksum = crc8x_fast(0, (uint8_t *)destination_copy, size_copy);
  
  return(checksum == eeprom_get_char(source));
}

void memcpy_to_eeprom(unsigned int destination, char *source, unsigned int size) {
    for(; size > 0; size--) {
        eeprom_put_char(destination++, *(source++));
    }    
}

void memcpy_from_eeprom(char *destination, unsigned int source, unsigned int size) {
    unsigned char data;    
    for(; size > 0; size--) {
        data = eeprom_get_char(source++);
        *(destination++) = data;
    }
}



/* polynomial is based on crc8.py module from pypl */
static uint8_t const crc8x_table[] PROGMEM = {
            0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
            0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
            0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
            0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
            0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5,
            0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
            0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
            0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
            0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
            0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
            0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2,
            0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
            0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32,
            0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
            0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
            0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
            0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c,
            0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
            0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec,
            0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
            0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
            0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
            0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c,
            0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
            0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b,
            0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
            0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
            0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
            0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb,
            0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
            0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb,
            0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3};


uint8_t crc8x_fast(uint8_t crc, uint8_t *mem, uint16_t len) {
	uint8_t *data = mem;
    if (data == NULL)
        return 0xff;
    crc &= 0xff;
    for(uint16_t idx=0; idx<len; idx++){
        crc = pgm_read_byte_near(crc8x_table + (crc ^ data[idx])); //= crc8x_table[crc ^ data[idx]];
    }
    return crc;
}


void log_resetreason(void)
{
    uint8_t resetReason = MCUSR;
    /* find and log reset reason */    
#ifdef PRINT_DETAILED_RESET_INFO	
    printPgmString(PSTR("\r\n\r\nReset cause: "));
    if      ( resetReason & ( 1<<PORF ) )   { printPgmString(PSTR("Power On\r\n")); }
    else if ( resetReason & ( 1<<EXTRF) )   { printPgmString(PSTR("External Reset\r\n")); }
    else if ( resetReason & ( 1<<BORF ) )   { printPgmString(PSTR("Brown out\r\n")); }
    else if ( resetReason & ( 1<<WDRF ) )   { printPgmString(PSTR("Watch Dog\r\n")); }
    else if ( resetReason & ( 1<<JTRF ) )   { printPgmString(PSTR("JTAG\r\n")); }
    else    { printInteger( resetReason );    printPgmString(PSTR(": Watch Dog?\r\n")); } /* apparently WDR is not showing in the MCUSR, so if anything else then likely it is it. Also bootloader uses WD to reset status, watch out for it later */
#endif //PRINT_DETAILED_RESET_INFO

    if      ( resetReason & ( 1<<PORF ) )   { flashStatistics.PORF_cnt++;  }
    else if ( resetReason & ( 1<<EXTRF) )   { flashStatistics.EXTRF_cnt++; }
    else if ( resetReason & ( 1<<BORF ) )   { flashStatistics.BORF_cnt++;  }
    else if ( resetReason & ( 1<<WDRF ) )   { flashStatistics.WDRF_cnt++;  }
    else if ( resetReason & ( 1<<JTRF ) )   { flashStatistics.JTRF_cnt++;  }
    else    {								  flashStatistics.WDRF_cnt++;  } /* apparently WDR is not showing in the MCUSR, so if anything else then likely it is it. Also bootloader uses WD to reset status, watch out for it later */

    /* clear status register after reading as it is sticky */
    MCUSR = 0;
}


void report_last_wdt_addresses(void){
    /* print */
    uint8_t i;
    printPgmString(PSTR("Last WDT addresses: "));
    for (i=0; i<FLASH_STAT_FIFO_SIZE; i++){
        printInteger( flashStatistics.lastReturnAddresses[i] ); printPgmString(PSTR(", "));
    }
    printPgmString(PSTR("\n"));    
}

void manage_rst_reasons(void){

    log_resetreason();
    flashStatistics.TOT_cnt++; /* increment total reset count */
    
    printPgmString(PSTR("Total resets: "));   printInteger( flashStatistics.TOT_cnt );
#ifdef PRINT_DETAILED_RESET_INFO	
    printPgmString(PSTR(" = "));        printInteger( flashStatistics.PORF_cnt);
    printPgmString(PSTR(" POR + "));     printInteger( flashStatistics.EXTRF_cnt);
    printPgmString(PSTR(" EXT + "));     printInteger( flashStatistics.BORF_cnt);
    printPgmString(PSTR(" BOR + "));     printInteger( flashStatistics.WDRF_cnt);
    printPgmString(PSTR(" WDT + "));     printInteger( flashStatistics.JTRF_cnt);
    printPgmString(PSTR(" JTAG"));    
#endif //PRINT_DETAILED_RESET_INFO
    printPgmString(PSTR("\r\n"));

    /* manage last exception storage: if address in different from what was stored in flashStatistics then store it */
    int8_t i;
    uint32_t return_addr;
    return_addr = get_return_addr(); /* return address from the latest stack dump */
    if (flashStatistics.lastReturnAddresses[0] != return_addr){
        /* shift fifo */
        for (i=FLASH_STAT_FIFO_SIZE-2; i>=0; i--){
            flashStatistics.lastReturnAddresses[i+1]    = flashStatistics.lastReturnAddresses[i];
        }        
        flashStatistics.lastReturnAddresses[0]          = return_addr;
        
        report_last_wdt_addresses();
        
    } //if (flashConfig.lastReturnAddresses[0] != return_addr){

}

/* implementation of safe flash structure update if new field is added in the end */
void manage_psflash_updates(void){
    if (flashStatistics.flashStatisticsVersion < CURRENT_FLASH_STAT_VER){  /* new trap method to initialise new variables under DFU */
        if (flashStatistics.flashStatisticsVersion < 20090614) {                  /* first time */
            printPgmString(PSTR("trap to update from version "));   printInteger( flashStatistics.flashStatisticsVersion );
            printPgmString(PSTR(" to version "));                   printInteger( CURRENT_FLASH_STAT_VER );
            printPgmString(PSTR("\n"));
            /* trap to update RSSI for new DFU where flash structure changed */
            flashStatistics.TOT_cnt = 0;                           /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
            flashStatistics.EXTRF_cnt = 0;                           /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
            flashStatistics.JTRF_cnt = 0;                           /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
            flashStatistics.totalTravelMillimeters = 0;             /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
            flashStatistics.flashStatisticsVersion = 20090614;     /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
        }
        //if (flashStatistics.flashStatisticsVersion < 20091500) {                  /* second time */
            //printPgmString(PSTR("trap to update from version "));   printInteger( flashStatistics.flashStatisticsVersion );
            //printPgmString(PSTR(" to version "));                   printInteger( CURRENT_FLASH_STAT_VER );
            //printPgmString(PSTR("\n"));
            ///* trap to update RSSI for new DFU where flash structure changed */
            //flashStatistics.flashStatisticsVersion = 20091500;     /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
        //}
        
    } //if (flashStatistics.flashStatisticsVersion < CURRENT_FLASH_STAT_VER){  /* new trap method to initialise new variables under DFU */
}



/* flashStatistics save and restore are done without crc 
 * 1) to avoid wearing flash prematurely (write happens every minute to circular buffer RunTimeMinutesFIFO), and crc location would have been worn fast.
 * 2) statistics is just for info and it is not too critical to keep it checked */

uint8_t flashStatisticsRestore(void){
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(1, DEBUG_1_PIN);
#endif    
    /* load statistics from eeprom */
    memcpy_from_eeprom((char*)&flashStatistics, EEPROM_ADDR_STATISTICS, sizeof(FlashStat));
    if (flashStatistics.flashStatisticsVersion == 0xFFFFFFFF) { 
        /* Reset with default zero vector if load for the first time */
        memset(&flashStatistics, 0, sizeof(FlashStat));
        memset(&flashStatistics.RunTimeMinutesFIFO, 0xFF, UPTIME_FIFO_SIZE_BYTES);
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(0, DEBUG_1_PIN);
#endif        
        return(false);
    }
    
    /* restore local time counter */
    localRunTimeSeconds = flashStatistics.totalRunTimeSeconds;
    /* add seconds remainder rolling bits from array into seconds */
    for (uint8_t byte_index=0; byte_index<UPTIME_FIFO_SIZE_BYTES; byte_index++){
        for (uint8_t bit_position=0; bit_position<8; bit_position++){            
            /* increment localRunTimeSeconds if bit is not 1*/
            if ( !(flashStatistics.RunTimeMinutesFIFO[byte_index] & (1<<bit_position)) ){
                localRunTimeSeconds += 64;
            }
        }        
    }
    
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(0, DEBUG_1_PIN);
#endif    
    return(true);
}

void flashStatisticsSave(void){
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(1, DEBUG_1_PIN);
#endif
/* only access EEPROM in not in motions state */
    if ( !(sys.state & (STATE_CYCLE | STATE_HOMING | STATE_JOG) ) ){
        /* save statistics to eeprom */    
        memcpy_to_eeprom(EEPROM_ADDR_STATISTICS, (char*)&flashStatistics, sizeof(FlashStat));
    }    
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(0, DEBUG_1_PIN);
#endif    
}

void uptime_increment(void){
    
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(1, DEBUG_0_PIN);
#endif
    
    localRunTimeSeconds++;       
    
    /* Flash lifetime is not affected when 0 is written, only depend on erase cycles. 
    array would allow to write 256 zeros and only then erase to loop the counter. 
    If do it every minute eeprom life will be reached in 10 years. */    
    
    /* every minute write bit to the minutes bit array */
    if ( (localRunTimeSeconds % 64) == 0 )
    {
        
        /* update travel distance if long enough (more than 1m) */
        if ( totalTravelMillimeters > 1000.0){
            flashStatistics.totalTravelMillimeters += (uint32_t)totalTravelMillimeters;
            totalTravelMillimeters = 0.0;
        }
        
        uint32_t localRunTimeMinutes = localRunTimeSeconds >> 6; /* not exactly minutes but dies the job be slowing down EEPROM writes 64 times */

        /* unroll the minutes remainder */
        uint16_t minutes_remainder = localRunTimeMinutes % (UPTIME_FIFO_SIZE_BYTES*8);

        /* position "minutes_remainder" is position of the bit in the array that need to be set to "0" */

        /* find which byte it should be set in: */
        uint8_t byte_index = minutes_remainder >> 3;
        uint8_t bit_position = minutes_remainder % 8;

        /* clear relevant bit */
        flashStatistics.RunTimeMinutesFIFO[byte_index] &=~(1<<bit_position) ;

        /* update flashStatistics.totalRunTimeSeconds when bit buffer wraps and erase the buffer RunTimeMinutesFIFO (this is what actually takes time and life from EEPROM */
        if ( minutes_remainder == (UPTIME_FIFO_SIZE_BYTES*8-1) )
        {
            flashStatistics.totalRunTimeSeconds = localRunTimeSeconds+64;
            memset(&flashStatistics.RunTimeMinutesFIFO, 0xFF, UPTIME_FIFO_SIZE_BYTES);
        }


        /* write total statistic to EEPROM every hour */
        flashStatisticsSave();

#ifdef FLASH_DEBUG_ENABLED
        printPgmString(PSTR("total ")); printInteger( flashStatistics.totalRunTimeSeconds );
        printPgmString(PSTR("s, local ")); printInteger( localRunTimeSeconds );
        printPgmString(PSTR("s, minutes_remainder ")); printInteger( minutes_remainder );
        printPgmString(PSTR("\n"));       
#endif
        
        
    }
#ifdef FLASH_DEBUG_ENABLED
debug_pin_write(0, DEBUG_0_PIN);
#endif
    
}
void report_statistics(void)
{
    printPgmString(PSTR(BK_INITIATOR));
    printPgmString(PSTR("STAT:"));
    printInteger(flashStatistics.TOT_cnt                ); printPgmString(PSTR(", "));
    printInteger(flashStatistics.JTRF_cnt               ); printPgmString(PSTR(", "));
    printInteger(flashStatistics.WDRF_cnt               ); printPgmString(PSTR(", "));
    printInteger(flashStatistics.BORF_cnt               ); printPgmString(PSTR(", "));
    printInteger(flashStatistics.EXTRF_cnt              ); printPgmString(PSTR(", "));
    printInteger(flashStatistics.PORF_cnt               ); printPgmString(PSTR(", "));
    printInteger(localRunTimeSeconds                    ); printPgmString(PSTR(", "));
    printInteger(flashStatistics.totalTravelMillimeters ); printPgmString(PSTR(", "));
    printInteger(flashStatistics.totalStallsDetected    );
    printPgmString(PSTR(BK_TERMINATOR));
}


void store_stall_info(uint8_t  lastStallsMotor, uint16_t lastStallsSG, uint16_t lastStallsSGcalibrated,  uint16_t lastStallsStepUs){
    /* manage fifo storage*/
    int8_t i;
    for (i=FLASH_STAT_FIFO_SIZE-2; i>=0; i--){
        flashStatistics.lastStallsMotor[i+1]        = flashStatistics.lastStallsMotor[i];
        flashStatistics.lastStallsSG[i+1]           = flashStatistics.lastStallsSG[i];
        flashStatistics.lastStallsSGcalibrated[i+1] = flashStatistics.lastStallsSGcalibrated[i];
        flashStatistics.lastStallsStepUs[i+1]       = flashStatistics.lastStallsStepUs[i];
        flashStatistics.lastStallsTravel[i+1]       = flashStatistics.lastStallsTravel[i];
    }
    flashStatistics.lastStallsMotor[0]          =lastStallsMotor;
    flashStatistics.lastStallsSG[0]             =lastStallsSG;
    flashStatistics.lastStallsSGcalibrated[0]   =lastStallsSGcalibrated;
    flashStatistics.lastStallsStepUs[0]         =lastStallsStepUs;
    flashStatistics.lastStallsTravel[0]         = flashStatistics.totalTravelMillimeters + totalTravelMillimeters;
    flashStatistics.totalStallsDetected++; /* increment total reset count */
}

void report_stall_info(void){
    printPgmString(PSTR("Stalls statistics:\n"));
    uint8_t i;
    for (i=0; i<FLASH_STAT_FIFO_SIZE; i++){
        printPgmString(PSTR("M: "));        printInteger(flashStatistics.lastStallsMotor[i]);           printPgmString(PSTR(", "));
        printPgmString(PSTR("SG: "));       printInteger(flashStatistics.lastStallsSG[i]);              printPgmString(PSTR(", "));
        printPgmString(PSTR("calibr: "));   printInteger(flashStatistics.lastStallsSGcalibrated[i]);    printPgmString(PSTR(", "));
        printPgmString(PSTR("step: "));     printInteger(flashStatistics.lastStallsStepUs[i]);          printPgmString(PSTR("us, "));
        printPgmString(PSTR("travel: "));   printInteger(flashStatistics.lastStallsTravel[i]);          printPgmString(PSTR("mm, "));
        printPgmString(PSTR("\n"));
    }
}

uint32_t getLocalRunTimeSeconds(void){
	return localRunTimeSeconds;
}

void flashStatisticsInit(void){

#ifdef FLASH_DEBUG_ENABLED
    debug_pin_write(1, DEBUG_0_PIN);
#endif    

    flashStatisticsRestore();
    manage_rst_reasons();
    manage_psflash_updates();
    printPgmString(PSTR("Up time: "));  printInteger( getLocalRunTimeSeconds() ); printPgmString(PSTR("seconds\r\n"));
    printPgmString(PSTR("Total distance: "));  printInteger( flashStatistics.totalTravelMillimeters); printPgmString(PSTR("mm\r\n"));    
    flashStatisticsSave();
	
#ifdef FLASH_DEBUG_ENABLED
    debug_pin_write(0, DEBUG_0_PIN);
#endif
	
}


// end of file
