/*
 * watchdog.c
 *
 * Created: 12/09/2020 22:53:00
 *  Author: Boris
 */ 

#include "grbl.h"


void enable_watchdog(void){

	/* 	1. In the same operation, write a logic one to the Watchdog change enable bit (WDCE) and WDE. A logic one
		must be written to WDE regardless of the previous value of the WDE bit.
		2. Within the next four clock cycles, write the WDE and Watchdog prescaler bits (WDP) as desired, but with
		the WDCE bit cleared. This must be done in one operation. */

	/* clear WDT event, if any */
    MCUSR &= ~(1<<WDRF);

	/* enable watchdog macro*/
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    //WDTCSR = (1<<WDP0); // Set time-out at ~32msec.
    WDTCSR = (1<<WDP2) | (1<<WDP0) | (1<<WDE);  //Set time-out at ~500msec and reset on watchdog expire.
    //WDTCSR = (1<<WDP2) | (1<<WDP1) | (1<<WDP0);  //Set time-out at ~2000msec.


    //WDTCSR = (1<<WDP2) | (1<<WDP0);  //Set time-out at ~500msec

    WDTCSR |= (1<<WDIE); //enable watchdog interrupts

}



char ByteArrayToHexViaLookup2[] = "0123456789ABCDEF";

#if defined(__AVR__)
#include <util/atomic.h>
//#include <avr/wdt.h>
#endif

void serial_print_hex8(uint8_t byte){

	/* convert bytes to hex str  */
	char hex_str_buffer1[3];
	hex_str_buffer1[1] = ByteArrayToHexViaLookup2[byte    & 0xF];        /* LSB 4 bits */
	hex_str_buffer1[0] = ByteArrayToHexViaLookup2[byte>>4 & 0xF];        /* MSB 4 bits */
	hex_str_buffer1[2] = 0; /* terminator */
	printString(hex_str_buffer1);

}

void serial_print_hex16(uint32_t byte){

	/* convert bytes to hex str  */
	char hex_str_buffer2[5];
	hex_str_buffer2[3] = ByteArrayToHexViaLookup2[byte     & 0xF];        /* LSB 4 bits */
	hex_str_buffer2[2] = ByteArrayToHexViaLookup2[byte>>4  & 0xF];        /* MSB 4 bits */
	hex_str_buffer2[1] = ByteArrayToHexViaLookup2[byte>>8  & 0xF];        /* MSB 4 bits */
	hex_str_buffer2[0] = ByteArrayToHexViaLookup2[byte>>12 & 0xF];        /* MSB 4 bits */
	hex_str_buffer2[4] = 0; /* terminator */
	printString(hex_str_buffer2);

}

void serial_print_hex32(uint32_t byte){

	/* convert bytes to hex str  */
	char hex_str_buffer4[9];
	hex_str_buffer4[7] = ByteArrayToHexViaLookup2[byte     & 0xF];        /* LSB 4 bits */
	hex_str_buffer4[6] = ByteArrayToHexViaLookup2[byte>>4  & 0xF];        /* MSB 4 bits */
	hex_str_buffer4[5] = ByteArrayToHexViaLookup2[byte>>8  & 0xF];        /* MSB 4 bits */
	hex_str_buffer4[4] = ByteArrayToHexViaLookup2[byte>>12 & 0xF];        /* MSB 4 bits */
	hex_str_buffer4[3] = ByteArrayToHexViaLookup2[byte>>16 & 0xF];        /* LSB 4 bits */
	hex_str_buffer4[2] = ByteArrayToHexViaLookup2[byte>>20 & 0xF];        /* MSB 4 bits */
	hex_str_buffer4[1] = ByteArrayToHexViaLookup2[byte>>24 & 0xF];        /* MSB 4 bits */
	hex_str_buffer4[0] = ByteArrayToHexViaLookup2[byte>>28 & 0xF];        /* MSB 4 bits */
	hex_str_buffer4[8] = 0; /* terminator */
	printString(hex_str_buffer4);

}

void dumpExtAddrLineIhex(uint16_t upper_addr) {
  // Since normal records only have 16 address bit, this special record
  // sets the upper 16 address bits to be a prepended to all subsequent
  // records.
  printPgmString(PSTR(":02000004"));
  serial_print_hex16(upper_addr);

  uint8_t sum = 0 - 2 - 4 - (upper_addr >> 8) - (upper_addr & 0xff);
  serial_print_hex8(sum);
  printPgmString(PSTR("\n"));
}

void dumpLineIhex(uint16_t addr, uint16_t end, uint16_t sp) {
  uint8_t sum = 0;

  //uintptr_t intaddr = (uintptr_t)addr;
  //if ((intaddr >> 16) != ((uintptr_t)prev_addr >> 16))
    //dumpExtAddrLineIhex(intaddr >> 16);

  serial_write(':');
  uint8_t len = end - addr;
  // Line length
  serial_print_hex8(len);
  sum -= len;

  // Addr
  //uint16_t addr16 = (uint16_t)intaddr;
  uint16_t addr16 = sp+1;
  serial_print_hex16(addr16);
  sum -= addr16 >> 8;
  sum -= addr16 & 0xff;

  // Record type
  serial_print_hex8((uint8_t)0);
  sum -= 0;

  while (addr < end) {
    uint8_t byte = eeprom_get_char(addr++);
    sum -= byte;
    serial_print_hex8(byte);
  }
  serial_print_hex8(sum);
  printPgmString(PSTR("\n"));
}

#define LINE_LENGTH 32

void dumpMemoryIhex(uint16_t addr, uint16_t end, uint16_t sp) {

  while (addr < end) {
	uint16_t next = addr + LINE_LENGTH;
    if (next > end)
      next = end;
    dumpLineIhex(addr, next, sp);
    sp += LINE_LENGTH;
    addr = next;
  }
  //EOF marker
  printPgmString(PSTR(":00000001FF\n"));
}


  // On AVR 3-byte PC processors, the builtin doesn't work, so we'll
  // improvise
  // This returns a word address, not a byte address
  inline uint32_t get_return_address() __attribute__((__always_inline__));
  inline uint32_t get_return_address() {
    // Frame layout:
    // [RA0]
    // [RA1]
    // [RA2]
    // ... Variables ...
    // [empty] <-- SP

    // Find out how big the stack usage of the function
    // (into which we are inlined) is. It seems gcc won't
    // tell us, but we can trick the assembler into telling
    // us at runtime.
    uint8_t stack_usage;
    __asm__ __volatile__("ldi %0, .L__stack_usage" : "=r"(stack_usage));

    // Using the stack usage, we can find the top of the
    // frame (the byte below the return address)
    uint8_t *frame_top = (uint8_t*)SP + stack_usage;

    // And then read the return address
    return (uint32_t)frame_top[1] << 16 | (uint16_t)frame_top[2] << 8 | frame_top[3];
  }

  // We also need a retaddr variable that is bigger than uintptr_t
  //using retaddr_t = uint32_t;


//inline void dumpMemory() __attribute__((__always_inline__));

void dumpMemory(void) {
  //uint16_t sp;
  //ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//    sp = SP;
  //}
  //printPgmString(PSTR("SP = 0x"));
  //serial_print_hex16(sp);
//  printPgmString(PSTR("\n"));
//  printPgmString(PSTR("Return = 0x"));
//  serial_print_hex32(get_return_address() * 2);
//  printPgmString(PSTR(" (byte address)\n"));

  // Dump I/O memory
  //printPgmString(PSTR("IO registers:"));
  //uint8_t * nullptr = NULL;
  //dumpMemoryIhex(nullptr, (uint8_t*)RAMSTART);
  //printPgmString(PSTR("\n"));

	  //serial_print_hex32(NULL);
	  //printPgmString(PSTR("-"));
	  //serial_print_hex32(RAMSTART);


// recover stack length
	uint16_t stack_size;
	uint16_t sp;
	uint32_t ret_addr;
	uint32_t addr = EEPROM_ADDR_STACK_DUMP;

	stack_size  = eeprom_get_char(addr  );
	stack_size <<= 8;
	stack_size |= eeprom_get_char(addr+1);
	printPgmString(PSTR("stack_size: "));
	printInteger(stack_size);
	printPgmString(PSTR("\n"));

	//stack pointer
	sp  = eeprom_get_char(addr+2);
	sp <<= 8;
	sp |= eeprom_get_char(addr+3);
	printPgmString(PSTR("SP: 0x"));
	serial_print_hex32(sp);
	printPgmString(PSTR("\n"));

	//ret addres
	ret_addr  = eeprom_get_char(addr+4);
	ret_addr <<= 8;
	ret_addr |= eeprom_get_char(addr+5);
	ret_addr <<= 8;
	ret_addr |= eeprom_get_char(addr+6);
	printPgmString(PSTR("ret_addr: 0x"));
	serial_print_hex32(ret_addr);
	printPgmString(PSTR("\n"));


  printPgmString(PSTR("Stack:\n"));
  // Dump stack (SP is the next unused value, so SP + 1 is the first valid
  // stack value)
  //dumpMemoryIhex((uint8_t*)sp + 1, (uint8_t*)RAMEND+1);

  dumpMemoryIhex(EEPROM_ADDR_STACK_DUMP+EEPROM_STACK_HDR_SIZE, EEPROM_ADDR_STACK_DUMP+EEPROM_STACK_HDR_SIZE+stack_size, sp);
  printPgmString(PSTR("Dump complete\n"));

  // Ensure the output is transmitted before returning from the ISR
  //Serial.flush();
}

inline void dumpMemoryToEEPROM() __attribute__((__always_inline__));

inline void dumpMemoryToEEPROM() {

  uint32_t addr = EEPROM_ADDR_STACK_DUMP;
  uint16_t stack_size;
  uint16_t sp;
  uint32_t ret_addr;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    sp = SP;
  }
  //printPgmString(PSTR("SP = 0x"));
  //serial_print_hex16(sp);
  //printPgmString(PSTR("\n"));
  //printPgmString(PSTR("Return = 0x"));
  //serial_print_hex32(get_return_address() * 2);
  //printPgmString(PSTR(" (byte address)\n"));

  // Dump I/O memory
  //printPgmString(PSTR("IO registers:"));
  //uint8_t * nullptr = NULL;
  //dumpMemoryIhex(nullptr, (uint8_t*)RAMSTART);
  //printPgmString(PSTR("\n"));


  stack_size = RAMEND - sp;
  //memset(line, 0, 1024);
  //memcpy(line, (uint8_t*)sp + 1, stack_size);
  //memcpy_to_eeprom_with_checksum(addr,(char*)line, stack_size);


  //store the stack to EEPROM
  memcpy_to_eeprom_with_checksum(addr+EEPROM_STACK_HDR_SIZE,(char*)sp + 1, stack_size);

  //store length, stack pointer and return addr to eeprom
  //char header[7];
  //length
  eeprom_put_char(addr+1, (stack_size   )&0xFF);
  eeprom_put_char(addr  , (stack_size>>8)&0xFF);

  //stack pointer
  eeprom_put_char(addr+3, (sp   )&0xFF);
  eeprom_put_char(addr+2, (sp>>8)&0xFF);

  //ret addres
  ret_addr = get_return_address() * 2;
  eeprom_put_char(addr+6, (ret_addr      )&0xFF);
  eeprom_put_char(addr+5, (ret_addr >>  8)&0xFF);
  eeprom_put_char(addr+4, (ret_addr >> 16)&0xFF);


	  //serial_print_hex32(NULL);
	//printPgmString(PSTR("RAMSTART:"));
	//serial_print_hex32(RAMSTART);
	//printPgmString(PSTR("\n"));
	//printPgmString(PSTR("sp:"));
	//serial_print_hex32(sp);
	//printPgmString(PSTR("\n"));
	printPgmString(PSTR("stack_size: "));
	printInteger(stack_size);
	printPgmString(PSTR("\n"));
	//dumpMemoryIhex((uint8_t*)sp + 1, (uint8_t*)RAMEND+1);

  //printPgmString(PSTR("Stack:\n"));
  // Dump stack (SP is the next unused value, so SP + 1 is the first valid
  // stack value)
  //dumpMemoryIhex((uint8_t*)sp + 1, (uint8_t*)RAMEND+1);
  //printPgmString(PSTR("Dump complete\n"));

  // Ensure the output is transmitted before returning from the ISR
  //Serial.flush();
}




ISR(WDT_vect) // Watchdog timer ISR
{
  cli(); // clear interrupts to block any other functions from interfering with EEPROM storage
  dumpMemoryToEEPROM();
}


