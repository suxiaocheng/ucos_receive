#include "nrf24l01.h"

/* IO assign:
 * GPIOB12->MISO
 * GPIOB14->SCK
 * GPIOA8->CE
 * GPIOB13->MOSI
 * GPIOB15->CSN
 */
#ifdef NRF24L01_OPERATION_ENABLE

const uint8_t nrf24l01_addr[]={0x34,0x43,0x10,0x10,0x01};

__inline static uint8_t nrf24l01_miso(void)
{
	return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);
}

__inline static void nrf24l01_sck(uint8_t status)
{
	if (status == TRUE) {
		GPIOB->BSRR = GPIO_Pin_14;
	} else {
		GPIOB->BRR = GPIO_Pin_14;
	}
}

__inline static void nrf24l01_ce(uint8_t status)
{
	if (status == TRUE) {
		GPIOA->BSRR = GPIO_Pin_8;
	} else {
		GPIOA->BRR = GPIO_Pin_8;
	}
}

__inline static void nrf24l01_mosi(uint8_t status)
{
	if (status == TRUE) {
		GPIOB->BSRR = GPIO_Pin_13;
	} else {
		GPIOB->BRR = GPIO_Pin_13;
	}
}

__inline static void nrf24l01_csn(uint8_t status)
{
	if (status == TRUE) {
		GPIOB->BSRR = GPIO_Pin_15;
	} else {
		GPIOB->BRR = GPIO_Pin_15;
	}
}

uint32_t nrf24l01_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Init the gpio */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	nrf24l01_ce(FALSE);
	nrf24l01_csn(TRUE);
	nrf24l01_sck(TRUE);
	nrf24l01_mosi(TRUE);

	/* read to test if the devices is working ok */
	while(1){
		uint8_t buf[5];
		nrf24l01_write_buf(W_REGISTER+TX_ADDR,nrf24l01_addr, sizeof(nrf24l01_addr));
		nrf24l01_read_buf(R_REGISTER+TX_ADDR,buf, sizeof(buf));
		stm_printf("nrf24l01 config addr: %x-%x-%x-%x-%x\n", \
			buf[0], buf[1], buf[2], buf[3], buf[4]);
		if(MemCmp((const char *)nrf24l01_addr, (const char *)buf, sizeof(nrf24l01_addr)) == 0){			
			break;
		}
		OSTimeDlyHMSM(0, 0, 0, 1);
	}

	return TRUE;
}

uint8_t nrf24l01_spi(uint8_t dat)
{
	uint32_t i;
	for(i=0;i<8;i++)
	{
		nrf24l01_mosi((dat&0x80)?TRUE:FALSE);
		dat<<=1;
		nrf24l01_sck(TRUE);
		if(nrf24l01_miso())
			dat|=0x01;
		nrf24l01_sck(FALSE);
	}
	return dat;
}

/*****************SPI读寄存器一字节函数*********************************/
uint8_t nrf24l01_read_reg(uint8_t RegAddr)
{
	uint8_t ret;
	nrf24l01_csn(FALSE);//启动时序
	nrf24l01_spi(RegAddr);//写寄存器地址
	ret=nrf24l01_spi(0xff);//写入读寄存器指令  
	nrf24l01_csn(TRUE);
	return ret;
}

/*****************SPI写寄存器一字节函数*********************************/
uint8_t nrf24l01_write_reg(uint8_t RegAddr,uint8_t date)
{
	uint8_t ret;
	nrf24l01_csn(FALSE);//启动时序
	ret=nrf24l01_spi(RegAddr);//写入地址
	nrf24l01_spi(date);//写入值
	nrf24l01_csn(TRUE);
	return ret;
}

/*****************SPI读取RXFIFO寄存器的值********************************/
uint8_t nrf24l01_read_buf(uint8_t RegAddr,uint8_t *RxDate,uint8_t DateLen)
{  //寄存器地址//读取数据存放变量//读取数据长度//用于接收
	uint8_t ret, i;
	nrf24l01_csn(FALSE);//启动时序
	ret=nrf24l01_spi(RegAddr);//写入要读取的寄存器地址
	for(i=0;i<DateLen;i++) //读取数据
	{
		RxDate[i]=nrf24l01_spi(0xff);
	} 
	nrf24l01_csn(TRUE);
	return ret; 
}

/*****************SPI写入TXFIFO寄存器的值**********************************/
uint8_t nrf24l01_write_buf(uint8_t RegAddr,const uint8_t *TxDate,uint8_t DateLen)
{ //寄存器地址//写入数据存放变量//读取数据长度//用于发送
	uint8_t ret,i;
	nrf24l01_csn(FALSE);
	ret=nrf24l01_spi(RegAddr);//写入要写入寄存器的地址
	for(i=0;i<DateLen;i++)//写入数据
	{
		nrf24l01_spi(*TxDate++);
	}   
	nrf24l01_csn(TRUE);
	return ret;
}

/*****************NRF设置为发送模式并发送数据******************************/
void nrf24l01_set_tx_mode(uint8_t *TxDate)
{
	nrf24l01_ce(FALSE); 
	nrf24l01_write_buf(W_REGISTER+TX_ADDR,nrf24l01_addr,TX_ADDR_WITDH);//写寄存器指令+接收地址使能指令+接收地址+地址宽度
	nrf24l01_write_buf(W_REGISTER+RX_ADDR_P0,nrf24l01_addr,TX_ADDR_WITDH);//为了应答接收设备，接收通道0地址和发送地址相同
	nrf24l01_write_buf(W_TX_PAYLOAD,TxDate,TX_DATA_WITDH);//写入数据 
	/******下面有关寄存器配置**************/
	nrf24l01_write_reg(W_REGISTER+EN_AA,0x01);       // 使能接收通道0自动应答
	nrf24l01_write_reg(W_REGISTER+EN_RXADDR,0x01);   // 使能接收通道0
	nrf24l01_write_reg(W_REGISTER+SETUP_RETR,0x0a);  // 自动重发延时等待250us+86us，自动重发10次
	nrf24l01_write_reg(W_REGISTER+RF_CH,0x40);         // 选择射频通道0x40
	nrf24l01_write_reg(W_REGISTER+RF_SETUP,0x07);    // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益
	nrf24l01_write_reg(W_REGISTER+CONFIG,0x0e);      // CRC使能，16位CRC校验，上电  
	nrf24l01_ce(TRUE);
}

/*****************NRF设置为接收模式并接收数据******************************/
//主要接收模式
void nrf24l01_set_rx_mode(void)
{
	nrf24l01_ce(FALSE);  
	nrf24l01_write_buf(W_REGISTER+RX_ADDR_P0,nrf24l01_addr,TX_ADDR_WITDH);  // 接收设备接收通道0使用和发送设备相同的发送地址
	nrf24l01_write_reg(W_REGISTER+EN_AA,0x01);               // 使能接收通道0自动应答
	nrf24l01_write_reg(W_REGISTER+EN_RXADDR,0x01);           // 使能接收通道0
	nrf24l01_write_reg(W_REGISTER+RF_CH,0x40);                 // 选择射频通道0x40
	nrf24l01_write_reg(W_REGISTER+RX_PW_P0,TX_DATA_WITDH);  // 接收通道0选择和发送通道相同有效数据宽度
	nrf24l01_write_reg(W_REGISTER+RF_SETUP,0x07);            // 数据传输率1Mbps，发射功率0dBm，低噪声放大器增益*/
	nrf24l01_write_reg(W_REGISTER+CONFIG,0x0f);              // CRC使能，16位CRC校验，上电，接收模式
	nrf24l01_ce(TRUE);
}

/****************************检测应答信号******************************/
uint32_t nrf24l01_check_ack(void)
{
	uint8_t ret;
	ret = nrf24l01_read_reg(R_REGISTER+STATUS);                    // 返回状态寄存器
	if(ret & STATUS_TX_DS){
		nrf24l01_write_reg(W_REGISTER+STATUS, STATUS_TX_DS);  // 清除TX_DS或MAX_RT中断标志
		return NRF24L01_TRANSFER_OK;
	}else if(ret & STATUS_MAX_RT){
		nrf24l01_write_reg(W_REGISTER+STATUS, STATUS_MAX_RT);  // 清除TX_DS或MAX_RT中断标志
		nrf24l01_csn(FALSE);
		nrf24l01_spi(FLUSH_TX);
		nrf24l01_csn(TRUE); 
		return NRF24L01_TRANSFER_MAX_RETRY;
	}
	return NRF24L01_TRANSFER_PROGRESS;
}

/******************判断是否接收收到数据，接到就从RX取出*********************/
//用于接收模式
uint32_t nrf24l01_receive_data(uint8_t *RevDate)
{
	uint8_t RevFlags=0;
	uint32_t ret;
	ret = nrf24l01_read_reg(R_REGISTER+STATUS);//发送数据后读取状态寄存器
	//stm_printf("status: %x\n", ret);
	if(ret & STATUS_RX_DR)				// 判断是否接收到数据
	{
		nrf24l01_write_reg(W_REGISTER+STATUS,0xff); //接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清楚中断标
		nrf24l01_read_buf(R_RX_PAYLOAD,RevDate,RX_DATA_WITDH);// 从RXFIFO读取数据
		RevFlags=1;	   //读取数据完成标志		
	}
	/* Check if fifo is not empty */
	if(!RevFlags)
	{
		if((ret & STATUS_RX_P_NO) != STATUS_RX_P_NO)
		{
			nrf24l01_read_buf(R_RX_PAYLOAD,RevDate,RX_DATA_WITDH);// 从RXFIFO读取数据
			RevFlags=1;	   //读取数据完成标志
		}
	}	
	return RevFlags;
}
#endif

