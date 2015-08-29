#ifndef NOR_FLASH_H
#define NOR_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "includes.h"

#define USE_DMA1_FOR_SPI
#define NOR_FLASH_SAFE_CHECK	/* Use for check nor flash write enable */

/* program return value */
#define NOR_FLASH_EXEC_OK		0x0
#define NOR_FLASH_BUSY			0x01
#define NOR_FLASH_TIMEOUT		0x02
#define NOR_FLASH_WR_DIS		0x03
#define NOR_FLASH_RD_ERR		0x04
#define NOR_FLASH_WR_ERR		0x05
#define NOR_FLASH_NOT_FOUND	0x06
#define NOR_FLASH_DMA_ERR		0x07

/* NOR FLASH Status reg */
#define NOR_STA_WIP					0x01
#define NOR_STA_WEL					0x02
#define NOR_STA_BP					0x7c
#define NOR_STA_SRP0				0x80
#define NOR_STA_SRP1				0x01
#define NOR_STA_QE					0x02
#define NOR_STA_LB					0x04
#define NOR_STA_CMP					0x40
#define NOR_STA_SUS					0x80

/* CONFIG PARAM */
#define INIT_RETRY_TIMES		3	/* init progress the nor flash retry times */
#define INIT_RETRY_DELAY		1	/* init progress the nor flash retry delay */
#define DMA_DELAY_TIME_MIN	5	/* use for the mini dma delay time (ms) */

/* param use for debug spi norflash */
//#define TEST_SPI_NORFLASH

	uint32_t nor_flash_init(void);
	uint32_t nor_flash_get_id(void);
	uint32_t nor_flash_wip_op(uint8_t op);
	uint32_t nor_flash_status_get(void);
	uint32_t nor_flash_status_w(uint8_t * status);
	uint32_t nor_flash_busy(void);
	uint32_t read_data(uint32_t address, uint8_t * dat, uint32_t len);
	uint32_t page_program(uint32_t address, uint8_t * dat, uint32_t len);
	uint32_t sector_earse(uint32_t address);
	uint32_t nor_flash_chip_erase(void);
	void test_spi_nor(void);

#ifdef __cplusplus
}
#endif
#endif
