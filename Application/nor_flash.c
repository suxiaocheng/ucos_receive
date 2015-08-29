#include "nor_flash.h"

#ifdef NOR_FLASH_OPERATION_ENABLE
uint8_t nor_device_id;
uint8_t nor_jedec_id[3];
uint8_t nor_manufacturer_id[2];
uint8_t uni_id;
uint8_t nor_flash_status[2];
static uint8_t spi_dummy;

const uint8_t cmd_jedec_id[] = { 0x9f };
const uint8_t cmd_manufacturer_id[] = { 0x90, 0x0, 0x0, 0x0 };

const uint8_t cmd_write_en[] = { 0x06 };
const uint8_t cmd_write_dis[] = { 0x04 };
const uint8_t cmd_read_status1[] = { 0x05 };
const uint8_t cmd_read_status2[] = { 0x35 };
const uint8_t cmd_write_status[] = { 0x01 };
const uint8_t cmd_read_data[] = { 0x03 };
const uint8_t cmd_page_program[] = { 0x02 };
const uint8_t cmd_sector_erase[] = { 0x20 };
const uint8_t cmd_chip_erase[] = { 0xc7 };

#if TEST_SPI_NORFLASH
uint8_t spi_ram_buf[512];
uint8_t spi_ram_cache[512];
#endif

__inline void spi_cs(uint8_t status)
{
	if (status == DISABLE) {
		GPIOA->BSRR = GPIO_Pin_4;
	} else {
		GPIOA->BRR = GPIO_Pin_4;
	}
}

#ifdef USE_DMA1_FOR_SPI
uint8_t config_dma_for_tx(const uint8_t * buf, uint16_t count)
{
	uint8_t dummy;
	uint32_t timeout = (uint32_t) count << 1;
	uint32_t i;
	uint8_t ret = NOR_FLASH_EXEC_OK;

	if (count == 0) {
		return ret;
	}

	if (timeout < DMA_DELAY_TIME_MIN) {
		timeout = DMA_DELAY_TIME_MIN;
	}

	/* SPI1_RX->DMA1->Channel2 */
	/* SPI1_TX->DMA1->Channel3 */
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* Disable the DMA transfer */
	DMA1_Channel2->CCR &= ~DMA_CCR1_EN;
	DMA1_Channel3->CCR &= ~DMA_CCR1_EN;

	/* Clear the interrupt flag */
	DMA1->IFCR =
	    DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 |
	    DMA_IFCR_CTEIF3;
	DMA1->IFCR =
	    DMA_IFCR_CGIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 |
	    DMA_IFCR_CTEIF2;

	/* config the transfer register */
	DMA1_Channel3->CPAR = (uint32_t) & (SPI1->DR);
	DMA1_Channel3->CMAR = (uint32_t) buf;
	DMA1_Channel3->CNDTR = count;
	DMA1_Channel3->CCR = DMA_CCR1_DIR | DMA_CCR1_MINC;

	/* config the receive register */
	DMA1_Channel2->CPAR = (uint32_t) & (SPI1->DR);
	DMA1_Channel2->CMAR = (uint32_t) & dummy;
	DMA1_Channel2->CNDTR = count;
	DMA1_Channel2->CCR = 0x0;

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

	/* Enable the DMA Channel */
	DMA1_Channel3->CCR |= DMA_CCR1_EN;
	DMA1_Channel2->CCR |= DMA_CCR1_EN;

	/* wait transfer complete */
	for (i = 0; i < timeout; i++) {
		if ((DMA1->ISR & (DMA_IFCR_CTCIF3 | DMA_IFCR_CTCIF2)) ==
		    (DMA_IFCR_CTCIF3 | DMA_IFCR_CTCIF2)) {
			break;
		} else if (DMA1->ISR & (DMA_IFCR_CTEIF3 | DMA_IFCR_CTEIF2)) {
			ret = NOR_FLASH_DMA_ERR;
			break;
		}
		OSTimeDlyHMSM(0, 0, 0, 2);
	}

	if (i == timeout) {
		ret = NOR_FLASH_TIMEOUT;
	}

	DMA1_Channel2->CCR &= ~DMA_CCR1_EN;
	DMA1_Channel3->CCR &= ~DMA_CCR1_EN;

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);

	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);

	return ret;
}

uint8_t config_dma_for_rx(uint8_t * buf, uint16_t count)
{
	uint8_t dummy = 0xff;
	uint32_t timeout = (uint32_t) count << 1;
	uint32_t i;
	uint8_t ret = NOR_FLASH_EXEC_OK;

	if (count == 0) {
		return ret;
	}

	if (timeout < DMA_DELAY_TIME_MIN) {
		timeout = DMA_DELAY_TIME_MIN;
	}

	/* SPI1_RX->DMA1->Channel2 */
	/* SPI1_TX->DMA1->Channel3 */
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* Disable the DMA transfer */
	DMA1_Channel2->CCR &= ~DMA_CCR1_EN;
	DMA1_Channel3->CCR &= ~DMA_CCR1_EN;

	/* Clear the interrupt flag */
	DMA1->IFCR =
	    DMA_IFCR_CGIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 |
	    DMA_IFCR_CTEIF3;
	DMA1->IFCR =
	    DMA_IFCR_CGIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 |
	    DMA_IFCR_CTEIF2;

	/* config the transfer register */
	DMA1_Channel3->CPAR = (uint32_t) & (SPI1->DR);
	DMA1_Channel3->CMAR = (uint32_t) & dummy;
	DMA1_Channel3->CNDTR = count;
	DMA1_Channel3->CCR = DMA_CCR1_DIR;

	/* config the receive register */
	DMA1_Channel2->CPAR = (uint32_t) & (SPI1->DR);
	DMA1_Channel2->CMAR = (uint32_t) buf;
	DMA1_Channel2->CNDTR = count;
	DMA1_Channel2->CCR = DMA_CCR1_MINC;

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);

	/* Enable the DMA Channel */
	DMA1_Channel3->CCR |= DMA_CCR1_EN;
	DMA1_Channel2->CCR |= DMA_CCR1_EN;

	/* wait transfer complete */
	for (i = 0; i < timeout; i++) {
		if ((DMA1->ISR & (DMA_IFCR_CTCIF3 | DMA_IFCR_CTCIF2)) ==
		    (DMA_IFCR_CTCIF3 | DMA_IFCR_CTCIF2)) {
			break;
		} else if (DMA1->ISR & (DMA_IFCR_CTEIF3 | DMA_IFCR_CTEIF2)) {
			ret = NOR_FLASH_DMA_ERR;
			break;
		}
		OSTimeDlyHMSM(0, 0, 0, 2);
	}

	if (i == timeout) {
		ret = NOR_FLASH_TIMEOUT;
	}

	DMA1_Channel2->CCR &= ~DMA_CCR1_EN;
	DMA1_Channel3->CCR &= ~DMA_CCR1_EN;

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);

	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);

	return ret;
}
#else
uint32_t get_spi_data(uint8_t * dat)
{
	if (((SPI1->SR & SPI_SR_BSY) == 0) && (SPI1->SR & SPI_SR_RXNE)) {
		*dat = *(uint8_t *) & (SPI1->DR);
		return 1;
	}
	return 0;
}

uint32_t send_spi_data(uint8_t mo_dat, uint8_t * mi_dat)
{
	uint32_t timeout = 30;
	while ((SPI1->SR & SPI_SR_TXE) == 0) {
		timeout--;
		if (timeout == 0) {
			system_dump("wait tx buffer timeout\n");
			//stm_printf("wait tx buffer timeout\n");
			return 1;
		}
		OSTimeDlyHMSM(0, 0, 0, 2);
	}
	*(uint8_t *) & (SPI1->DR) = mo_dat;
	//SPI1->DR = mo_dat;

	timeout = 30;
	while (get_spi_data(mi_dat) == 0) {
		timeout--;
		if (timeout == 0) {
			system_dump("wait rx data timeout\n");
			//stm_printf("wait rx data timeout\n");
			return 1;
		}
		OSTimeDlyHMSM(0, 0, 0, 2);
	}

	return 0;
}
#endif

uint32_t spi_read(const uint8_t * cmd, uint8_t cmd_len, uint8_t * dat,
		  uint16_t dat_len)
{
#ifndef USE_DMA1_FOR_SPI
	uint16_t i;
#endif
	volatile uint8_t dummy_data;
	uint32_t ret;
	uint32_t timeout;

	SPI_Cmd(SPI1, ENABLE);
	spi_cs(ENABLE);
#ifdef USE_DMA1_FOR_SPI
	ret = config_dma_for_tx(cmd, cmd_len);
	if (ret) {
		goto spi_read_end;
	}
	ret = config_dma_for_rx(dat, dat_len);
#else
	for (i = 0; i < cmd_len; i++) {
		ret = send_spi_data(*cmd++, &dummy_data);
		if (ret) {
			goto spi_read_end;
		}
	}

	for (i = 0; i < dat_len; i++) {
		ret = send_spi_data(0xff, dat++);
		if (ret) {
			goto spi_read_end;
		}
	}
#endif
      spi_read_end:
	timeout = 30;
	while ((SPI1->SR & SPI_SR_TXE) == 0) {
		timeout--;
		if (timeout == 0) {
			ret = 1;
			break;
		}
		OSTimeDlyHMSM(0, 0, 0, 2);
	}
	/* wait until spi is not busy */
	timeout = 30;
	while ((SPI1->SR & SPI_SR_BSY) != 0) {
		timeout--;
		if (timeout == 0) {
			ret = 1;
			break;
		}
		OSTimeDlyHMSM(0, 0, 0, 2);
	}

	/* disable spi communcation */
	spi_cs(DISABLE);
	SPI_Cmd(SPI1, DISABLE);

	/* read out spi fifo data */
	timeout = 30;
	while ((SPI1->SR & SPI_SR_RXNE) != 0) {
		dummy_data = *(uint8_t *) & (SPI1->DR);
		if (--timeout == 0) {
			ret = 1;
			break;
		}
	}

	return ret;
}

uint32_t spi_write(const uint8_t * cmd, uint8_t cmd_len, uint8_t * dat,
		   uint16_t dat_len)
{
#ifndef USE_DMA1_FOR_SPI
	uint16_t i;
#endif
	volatile uint8_t dummy_data;
	uint32_t ret;
	uint32_t timeout;

	SPI_Cmd(SPI1, ENABLE);
	spi_cs(ENABLE);

#ifdef USE_DMA1_FOR_SPI
	ret = config_dma_for_tx(cmd, cmd_len);
	if (ret) {
		goto spi_write_end;
	}
	ret = config_dma_for_tx(dat, dat_len);
#else
	for (i = 0; i < cmd_len; i++) {
		ret = send_spi_data(*cmd++, &dummy_data);
		if (ret) {
			goto spi_write_end;
		}
	}

	for (i = 0; i < dat_len; i++) {
		ret = send_spi_data(*dat++, &dummy_data);
		if (ret) {
			goto spi_write_end;
		}
	}
#endif
      spi_write_end:
	timeout = 30;
	while ((SPI1->SR & SPI_SR_TXE) == 0) {
		timeout--;
		if (timeout == 0) {
			if (!ret) {
				ret = NOR_FLASH_TIMEOUT;
			}
			break;
		}
		OSTimeDlyHMSM(0, 0, 0, 2);
	}
	/* wait until spi is not busy */
	timeout = 30;
	while ((SPI1->SR & SPI_SR_BSY) != 0) {
		timeout--;
		if (timeout == 0) {
			if (!ret) {
				ret = NOR_FLASH_TIMEOUT;
			}
			break;
		}
		OSTimeDlyHMSM(0, 0, 0, 2);
	}

	/* disable spi communcation */
	spi_cs(DISABLE);
	SPI_Cmd(SPI1, DISABLE);

	/* read out spi fifo data */
	timeout = 30;
	while ((SPI1->SR & SPI_SR_RXNE) != 0) {
		dummy_data = *(uint8_t *) & (SPI1->DR);
		if (--timeout == 0) {
			if (!ret) {
				ret = NOR_FLASH_TIMEOUT;
			}
			break;
		}
	}

	return ret;
}
/*******************************************************************************
* Function Name  :  nor_flash_init
* Description       :  init the nor flash 
* Input               :  void
* Output             : 
* Return             : the state of the nor flash. NOR_FLASH_EXEC_OK on Sucess.
*			       Other value indicate of fail of program of the nor flash.
* ReMark		    : The supply current must exceed 200mA, because I'm test it program fail,
*				But the awful thing happened that nothing was impile.
*******************************************************************************/
uint32_t nor_flash_init(void)
{
	SPI_InitTypeDef spi_init_def;
	GPIO_InitTypeDef GPIO_InitStructure;
#ifdef USE_DMA1_FOR_SPI
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	SPI_I2S_DeInit(SPI1);

	/* MOSI SCLK */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* MISO */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* NSS */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	SPI_SSOutputCmd(SPI1, ENABLE);

	SPI_StructInit(&spi_init_def);
	spi_init_def.SPI_CPHA = SPI_CPHA_2Edge;
	spi_init_def.SPI_CPOL = SPI_CPOL_High;
	spi_init_def.SPI_DataSize = SPI_DataSize_8b;
	spi_init_def.SPI_FirstBit = SPI_FirstBit_MSB;
	spi_init_def.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi_init_def.SPI_Mode = SPI_Mode_Master;
	spi_init_def.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	spi_init_def.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI1, &spi_init_def);

	SPI_Cmd(SPI1, DISABLE);

	spi_cs(ENABLE);
	OSTimeDlyHMSM(0, 0, 0, 2);
	spi_cs(DISABLE);

	return nor_flash_get_id();
}

uint32_t nor_flash_get_id(void)
{
	uint32_t ret;
	ret = spi_read(cmd_jedec_id, sizeof(cmd_jedec_id), nor_jedec_id, 3);

	stm_printf("jedec->%x\t%x\t%x\n", nor_jedec_id[0], nor_jedec_id[1],
		   nor_jedec_id[2]);

	/* init the nor flash fail */
	if ((ret == 0)
	    && ((nor_jedec_id[0] == 0xff) || (nor_jedec_id[0] == 0x0))) {
		ret = NOR_FLASH_NOT_FOUND;
	} else if (ret) {
		ret = NOR_FLASH_RD_ERR;
	}

	return ret;
}

/* op->1:enable write
	 op->0: disable write
*/
uint32_t nor_flash_wip_op(uint8_t op)
{
	uint32_t ret;
	uint32_t timeout;
	uint32_t status = (op == 1) ? NOR_STA_WEL : 0;

	ret = nor_flash_status_get();
	if (ret) {
		return NOR_FLASH_RD_ERR;
	}

	if ((nor_flash_status[0] & NOR_STA_WEL) == status)
		return NOR_FLASH_EXEC_OK;

	if (op == 1) {
		ret =
		    spi_read(cmd_write_en, sizeof(cmd_write_en), &spi_dummy, 0);
	} else {
		ret =
		    spi_read(cmd_write_dis, sizeof(cmd_write_dis), &spi_dummy,
			     0);
	}

	if (ret == 0) {
		timeout = 20;
		while (--timeout) {
			if (nor_flash_busy() == 0) {
				break;
			}
			OSTimeDlyHMSM(0, 0, 0, 2);
		}
		if (timeout == 0) {
			ret = NOR_FLASH_TIMEOUT;
		}
	} else {
		ret = NOR_FLASH_RD_ERR;
	}

#ifdef NOR_FLASH_SAFE_CHECK
	if (!ret) {
		ret = nor_flash_status_get();
		if (ret) {
			return NOR_FLASH_RD_ERR;
		}

		if ((nor_flash_status[0] & NOR_STA_WEL) == status) {
			ret = NOR_FLASH_EXEC_OK;
		} else {
			ret = NOR_FLASH_WR_DIS;
		}
	}
#endif

	return ret;
}

uint32_t nor_flash_status_get(void)
{
	uint32_t ret;
	ret =
	    spi_read(cmd_read_status1, sizeof(cmd_read_status1),
		     nor_flash_status, 1);
	ret |=
	    spi_read(cmd_read_status2, sizeof(cmd_read_status2),
		     &nor_flash_status[1], 1);
	if (ret) {
		return NOR_FLASH_RD_ERR;
	} else {
		return NOR_FLASH_EXEC_OK;
	}
}

uint32_t nor_flash_status_w(uint8_t * status)
{
	uint32_t ret;
	uint32_t timeout;

	ret = spi_write(cmd_write_status, sizeof(cmd_write_status), status, 2);

	if (ret == 0) {
		timeout = 20;
		while (--timeout) {
			if (nor_flash_busy() == 0) {
				break;
			}
			OSTimeDlyHMSM(0, 0, 0, 2);
		}
		if (timeout == 0) {
			ret = NOR_FLASH_TIMEOUT;
		}
	} else {
		ret = NOR_FLASH_WR_ERR;
	}

	return ret;
}

uint32_t nor_flash_busy(void)
{
	nor_flash_status_get();
	return (nor_flash_status[0] & NOR_STA_WIP) ? 1 : 0;
}

/*******************************************************************************
* Function Name  : read_data.
* Description       :  read nor flash 
* Input               : address: Nor flash address, in byte unit
*			       dat: reading data buffer
*			       len: reading length
* Output             : 
* Return             : the reading state of the nor flash. NOR_FLASH_EXEC_OK on Sucess.
*			       Other value indicate of fail of reading of the nor flash.
*******************************************************************************/
uint32_t read_data(uint32_t address, uint8_t * dat, uint32_t len)
{
	uint8_t read_cmd[4];
	uint32_t ret;

	if (nor_flash_busy()) {
		return NOR_FLASH_BUSY;
	}

	read_cmd[0] = cmd_read_data[0];
	read_cmd[1] = (address >> 16) & 0xff;
	read_cmd[2] = (address >> 8) & 0xff;
	read_cmd[3] = address & 0xff;
	ret = spi_read(read_cmd, sizeof(read_cmd), dat, len);

	if (ret) {
		ret = NOR_FLASH_RD_ERR;
	}

	return ret;
}

/*******************************************************************************
* Function Name  : page_program.
* Description       :  program nor flash 
* Input               : address: Nor flash address, in byte unit
*			       dat: Writing data buffer
*			       len: Writing length
* Output             : 
* Return             : the program state of the nor flash. NOR_FLASH_EXEC_OK on Sucess.
*			       Other value indicate of fail of program of the nor flash.
*******************************************************************************/
uint32_t page_program(uint32_t address, uint8_t * dat, uint32_t len)
{
	uint32_t unalign_max_len = 0;
	uint8_t page_write_cmd[4];
	uint32_t ret;
	uint32_t timeout;
	uint16_t write_len;

	if (nor_flash_busy()) {
		return NOR_FLASH_BUSY;
	}
	ret = nor_flash_wip_op(1);
	if (ret) {
		return ret;
	}

	page_write_cmd[0] = cmd_page_program[0];
	page_write_cmd[1] = (address >> 16) & 0xff;
	page_write_cmd[2] = (address >> 8) & 0xff;
	page_write_cmd[3] = address & 0xff;

	if (address & 0xff) {
		unalign_max_len = 0xff - (address & 0xff);
	}

	if (unalign_max_len != 0) {
		if ((len + (address & 0xff)) > 0xff) {
			ret =
			    spi_write(page_write_cmd, sizeof(page_write_cmd),
				      dat, unalign_max_len);
			len -= unalign_max_len;
			address += unalign_max_len;
			dat += unalign_max_len;
		} else {
			ret =
			    spi_write(page_write_cmd, sizeof(page_write_cmd),
				      dat, len);
			len = 0;
		}
		if (ret) {
			return NOR_FLASH_WR_ERR;
		}
		timeout = 6;
		while (--timeout) {
			if (nor_flash_busy() == 0) {
				break;
			}
			OSTimeDlyHMSM(0, 0, 0, 2);
		}
		if (timeout == 0) {
			return NOR_FLASH_TIMEOUT;
		}
	}

	while (len) {
		//page_write_cmd[0] = cmd_page_program[0];
		page_write_cmd[1] = (address >> 16) & 0xff;
		page_write_cmd[2] = (address >> 8) & 0xff;
		page_write_cmd[3] = 0x0;

		ret = nor_flash_wip_op(1);
		if (ret) {
			return ret;
		}

		if (len >= 0x100) {
			write_len = 0x100;
		} else {
			write_len = len;
		}
		ret =
		    spi_write(page_write_cmd, sizeof(page_write_cmd), dat,
			      write_len);
		if (ret) {
			return NOR_FLASH_WR_ERR;
		}
		dat += write_len;
		address += write_len;
		len -= write_len;

		timeout = 6;
		while (--timeout) {
			if (nor_flash_busy() == 0) {
				break;
			}
			OSTimeDlyHMSM(0, 0, 0, 2);
		}
		if (timeout == 0) {
			return NOR_FLASH_TIMEOUT;
		}
	}

	return NOR_FLASH_EXEC_OK;
}

/*******************************************************************************
* Function Name  : sector_earse.
* Description       :  Erase one sector
* Input               : address: Nor flash address, in byte unit*			       
* Output             : 
* Return             : the erase state of the nor flash. NOR_FLASH_EXEC_OK on Sucess.
*			       Other value indicate of fail of erase of the nor flash.
*******************************************************************************/
uint32_t sector_earse(uint32_t address)
{
	uint8_t page_earse_cmd[4];
	uint32_t ret;
	uint32_t timeout;

	if (nor_flash_busy()) {
		return NOR_FLASH_BUSY;
	}
	ret = nor_flash_wip_op(1);
	if (ret) {
		return ret;
	}

	page_earse_cmd[0] = cmd_sector_erase[0];
	page_earse_cmd[1] = (address >> 16) & 0xff;
	page_earse_cmd[2] = (address >> 8) & 0xf0;
	page_earse_cmd[3] = 0x0;

	ret = spi_write(page_earse_cmd, sizeof(page_earse_cmd), 0x0, 0x0);
	if (ret) {
		return NOR_FLASH_WR_ERR;
	}
	timeout = 600;
	while (--timeout) {
		if (nor_flash_busy() == 0) {
			break;
		}
		OSTimeDlyHMSM(0, 0, 0, 2);
	}
	if (timeout == 0) {
		ret = NOR_FLASH_TIMEOUT;
	}

	return ret;
}

uint32_t nor_flash_chip_erase(void)
{
	uint32_t ret;
	uint32_t timeout;

	if (nor_flash_busy()) {
		return NOR_FLASH_BUSY;
	}
	ret = nor_flash_wip_op(1);
	if (ret) {
		return ret;
	}
	ret = spi_write(cmd_chip_erase, sizeof(cmd_chip_erase), &spi_dummy, 0);

	if (ret) {
		return NOR_FLASH_WR_ERR;
	}
	timeout = 25000;
	while (--timeout) {
		if (nor_flash_busy() == 0) {
			break;
		}
		OSTimeDlyHMSM(0, 0, 0, 2);
	}
	if (timeout == 0) {
		ret = NOR_FLASH_TIMEOUT;
	}

	return ret;
}

#ifdef TEST_SPI_NORFLASH
void test_spi_nor(void)
{
	uint32_t count;
	uint32_t err = 0;
	for (count = 0; count < INIT_RETRY_TIMES; count++) {
		if (nor_flash_init() == 0) {
			break;
		}
		OSTimeDlyHMSM(0, 0, 0, INIT_RETRY_DELAY);
	}
	if (count < INIT_RETRY_TIMES) {
		stm_printf("init sucessfully\n");

		err = nor_flash_chip_erase();

		if (err != NOR_FLASH_EXEC_OK) {
			return;
		}

		for (count = 0; count < (1024 * 1024 / 512); count++) {
			MemSet(spi_ram_buf, count, sizeof(spi_ram_buf));
			spi_ram_buf[0] = count & 0xff;
			spi_ram_buf[1] = (count >> 8) & 0xff;
			//page_program((uint32_t)(count<<9), spi_ram_buf, sizeof(spi_ram_buf));
			read_data((uint32_t) (count << 9), spi_ram_cache,
				  sizeof(spi_ram_cache));
			if (MemCmp
			    ((char *)spi_ram_cache, (char *)spi_ram_buf,
			     sizeof(spi_ram_cache))) {
				err++;
			}
			//stm_printf("operate %d sucessfully, err:%d\n", count, err);
		}
	}
}
#endif
#endif

