/*
  eeprom.h - EEPROM methods
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef eeprom_h
#define eeprom_h

#define CURRENT_FLASH_STAT_VER			21010315 //0x18100600
#define UPTIME_FIFO_SIZE_BYTES          32
//#define PRINT_DETAILED_RESET_INFO     //uncomment to print out reset reasons statistics at boot

// Flash statistics
#define FLASH_STAT_FIFO_SIZE            8

typedef struct {
    /* reset source from MCUSR - MCU Status Register */
    uint32_t TOT_cnt;                                               /* total count of resets */
    uint32_t JTRF_cnt;                                              /* count of resets due to JTAG Reset Flag This bit is set if a reset is being caused by a logic one in the JTAG Reset Register selected by the JTAG instruction AVR_RESET. This bit is reset by a Power-on Reset, or by writing a logic zero to the flag. */
    uint32_t WDRF_cnt;                                              /* count of resets due to Watchdog Reset Flag. This bit is set if a Watchdog Reset occurs. The bit is reset by a Power-on Reset, or by writing a logic zero to the flag. */
    uint32_t BORF_cnt;                                              /* count of resets due to Brown-out Reset Flag. This bit is set if a Brown-out Reset occurs. The bit is reset by a Power-on Reset, or by writing a logic zero to the flag. */
    uint32_t EXTRF_cnt;                                             /* count of resets due to External Reset Flag. This bit is set if an External Reset occurs. The bit is reset by a Power-on Reset, or by writing a logic zero to the flag. */
    uint32_t PORF_cnt;                                              /* count of resets due to Power-on Reset Flag. This bit is set if a Power-on Reset occurs. The bit is reset only by writing a logic zero to the flag. */
    uint32_t flashStatisticsVersion;                                /* Current flashConfigVersion, required to decide which fields to be updated under DFU, CURRENT_FLASHCONFIG_VER*/
    uint32_t totalRunTimeSeconds;                                   /* total ON time in seconds. (using spi_interrupt) */
    uint8_t  RunTimeMinutesFIFO[UPTIME_FIFO_SIZE_BYTES];            /* Flash lifetime is not affected when 0 is written, only depend on erase cycles. this array would allow to write 256 zeros and only then erase to loop the counter. If do it every minute eeprom life will be reached in 40 years. */
    uint32_t totalTravelMillimeters;                                /* total travelled distance in mm. (using mm_var) */
    uint32_t totalStallsDetected;                                   /* total number of stalls detected */
    /* fifo with last exception addresses causing WD to trigger*/
    uint32_t lastReturnAddresses[FLASH_STAT_FIFO_SIZE];             /* return address from the latest stack dump */
    /* fifo with statistic on stall: total distance, feed, */
    uint32_t lastStallsTravel[FLASH_STAT_FIFO_SIZE];                /* fifo buffer for when last stalls were happening */
    uint8_t  lastStallsMotor[FLASH_STAT_FIFO_SIZE];                 /* fifo buffer for which motor stalled */
    uint16_t lastStallsSG[FLASH_STAT_FIFO_SIZE];                    /* fifo buffer for Stall Guard reading at last stalls */
    uint16_t lastStallsSGcalibrated[FLASH_STAT_FIFO_SIZE];          /* fifo buffer for SG delta to calibration at last stalls */
    uint16_t lastStallsStepUs[FLASH_STAT_FIFO_SIZE];                /* fifo buffer for feed rates at last stalls */
} FlashStat;
extern FlashStat flashStatistics;
extern float totalTravelMillimeters;                                /* accumulator for accurate tracking of the distance */

unsigned char eeprom_get_char(unsigned int addr);
void eeprom_put_char(unsigned int addr, unsigned char new_value);
void memcpy_to_eeprom_with_checksum(unsigned int destination, char *source, unsigned int size);
int memcpy_from_eeprom_with_checksum(char *destination, unsigned int source, unsigned int size);
void memcpy_to_eeprom(unsigned int destination, char *source, unsigned int size);
void memcpy_from_eeprom(char *destination, unsigned int source, unsigned int size);

uint8_t crc8x_fast(uint8_t crc, uint8_t *mem, uint16_t len); /* fast crc8 calculator */
void dumpMemory(void);
uint32_t get_return_addr(void); /* return address from the latest stack dump */
void uptime_increment(void);
void report_statistics(void);
void store_stall_info(uint8_t  lastStallsMotor, uint16_t lastStallsSG, uint16_t lastStallsSGcalibrated,  uint16_t lastStallsStepUs);
void report_stall_info(void);
void report_last_stall_info(void);
void report_last_wdt_addresses(void);

uint8_t flashStatisticsRestore(void);
void flashStatisticsSave(void);
void manage_psflash_updates(void);
void manage_rst_reasons(void);
uint32_t getLocalRunTimeSeconds(void);
void flashStatisticsInit(void);

void flash_serial_store(uint8_t* serial_number);
void flash_product_store(uint8_t* product_version);
void flash_serial_read(void);
void flash_product_read(void);

#endif
