/* 
 *
 */

#ifndef SPI_TO_TMC_H__
#define SPI_TO_TMC_H__

#include <string.h>
#include "TMC_interface.h"


/* SPI_TX_BUFFER holds the outgoing SPI commands, worst case - required for single motors - in that case number initial SPI transactions
 * could be 5x5+5x5 = 50. In dual motor case it is 5x3 + 5x3 = max 30 transactions*/
#define SPI_TX_BUFFER_MASK 						0x3F                 	/**< TX Buffer mask, must be a mask of contiguous zeroes, followed by contiguous sequence of ones: 000...111. */
#define SPI_TX_BUFFER_SIZE 						(SPI_TX_BUFFER_MASK + 1)/**< Size of the send buffer, which is 1 higher than the mask. */
#define TX_BUF_SIZE_SINGLE                      3                       /* maximum number of bytes transmitted to the TMC chip */
#define TX_BUF_SIZE_DUAL                        5                       /* maximum number of bytes transmitted to two TMC chips */


/* Structure for holding the data that will be transmitted to the TMC chip. Used in the queue
 */
typedef struct
{
    uint8_t buf_size;                                                   /* 3 for single or 5 for dual */
	uint8_t m_spi_tx_buf[TX_BUF_SIZE_DUAL]; 						    /* individual buffer storage for each element of the Tx queue*/
    TMC2590TypeDef *tmc2590_1;                                          /* pointer to current motor controller 1 */
    TMC2590TypeDef *tmc2590_2;                                          /* pointer to current motor controller 2 */
    uint8_t rdsel;    
} tx_spi_msg_t;								

/* SPI state types. */
typedef enum
{
	SPI_STATE_IDLE,   													/* normal state no activity*/
	SPI_STATE_1,    												    /* state 1	pull SSx low	write byte_x1 to the SPDR                  */
	SPI_STATE_2,    												    /* state 2	read the RX byte_x1 from SPDR	write byte_x2 to the SPDR  */
	SPI_STATE_3, 												        /* state 3	read the RX byte_x2 from SPDR	write byte_x3 to the SPDR  */
	SPI_STATE_4,    											        /* state 4	read the RX byte_x3 from SPDR	write byte_x4 to the SPDR  */
	SPI_STATE_5,         											    /* state 5	read the RX byte_x4 from SPDR	write byte_x5 to the SPDR  */
	SPI_STATE_6 													    /* state 6	read the RX byte_x5 from SPDR	pull SSx high              */
} spi_state_type_t;



void spi_hw_init(void);
void spi_schedule_single_tx(TMC2590TypeDef *tmc2590_1, uint8_t *data, uint8_t size, uint8_t rdsel);
#if defined(TMC_5_CONTROLLERS)
void spi_schedule_dual_tx(TMC2590TypeDef *tmc2590_1, TMC2590TypeDef *tmc2590_2, uint8_t *data, uint8_t size, uint8_t rdsel);
#endif
void tmc_pin_write(uint32_t level, uint32_t pin);

void spi_process_tx_queue(void); /* flush the SPI queue starting from next SPI transfer */

#endif // SPI_TO_WIFI_H__
