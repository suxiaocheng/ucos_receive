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

/*****************SPI���Ĵ���һ�ֽں���*********************************/
uint8_t nrf24l01_read_reg(uint8_t RegAddr)
{
	uint8_t ret;
	nrf24l01_csn(FALSE);//����ʱ��
	nrf24l01_spi(RegAddr);//д�Ĵ�����ַ
	ret=nrf24l01_spi(0xff);//д����Ĵ���ָ��  
	nrf24l01_csn(TRUE);
	return ret;
}

/*****************SPIд�Ĵ���һ�ֽں���*********************************/
uint8_t nrf24l01_write_reg(uint8_t RegAddr,uint8_t date)
{
	uint8_t ret;
	nrf24l01_csn(FALSE);//����ʱ��
	ret=nrf24l01_spi(RegAddr);//д���ַ
	nrf24l01_spi(date);//д��ֵ
	nrf24l01_csn(TRUE);
	return ret;
}

/*****************SPI��ȡRXFIFO�Ĵ�����ֵ********************************/
uint8_t nrf24l01_read_buf(uint8_t RegAddr,uint8_t *RxDate,uint8_t DateLen)
{  //�Ĵ�����ַ//��ȡ���ݴ�ű���//��ȡ���ݳ���//���ڽ���
	uint8_t ret, i;
	nrf24l01_csn(FALSE);//����ʱ��
	ret=nrf24l01_spi(RegAddr);//д��Ҫ��ȡ�ļĴ�����ַ
	for(i=0;i<DateLen;i++) //��ȡ����
	{
		RxDate[i]=nrf24l01_spi(0xff);
	} 
	nrf24l01_csn(TRUE);
	return ret; 
}

/*****************SPIд��TXFIFO�Ĵ�����ֵ**********************************/
uint8_t nrf24l01_write_buf(uint8_t RegAddr,const uint8_t *TxDate,uint8_t DateLen)
{ //�Ĵ�����ַ//д�����ݴ�ű���//��ȡ���ݳ���//���ڷ���
	uint8_t ret,i;
	nrf24l01_csn(FALSE);
	ret=nrf24l01_spi(RegAddr);//д��Ҫд��Ĵ����ĵ�ַ
	for(i=0;i<DateLen;i++)//д������
	{
		nrf24l01_spi(*TxDate++);
	}   
	nrf24l01_csn(TRUE);
	return ret;
}

/*****************NRF����Ϊ����ģʽ����������******************************/
void nrf24l01_set_tx_mode(uint8_t *TxDate)
{
	nrf24l01_ce(FALSE); 
	nrf24l01_write_buf(W_REGISTER+TX_ADDR,nrf24l01_addr,TX_ADDR_WITDH);//д�Ĵ���ָ��+���յ�ַʹ��ָ��+���յ�ַ+��ַ���
	nrf24l01_write_buf(W_REGISTER+RX_ADDR_P0,nrf24l01_addr,TX_ADDR_WITDH);//Ϊ��Ӧ������豸������ͨ��0��ַ�ͷ��͵�ַ��ͬ
	nrf24l01_write_buf(W_TX_PAYLOAD,TxDate,TX_DATA_WITDH);//д������ 
	/******�����йؼĴ�������**************/
	nrf24l01_write_reg(W_REGISTER+EN_AA,0x01);       // ʹ�ܽ���ͨ��0�Զ�Ӧ��
	nrf24l01_write_reg(W_REGISTER+EN_RXADDR,0x01);   // ʹ�ܽ���ͨ��0
	nrf24l01_write_reg(W_REGISTER+SETUP_RETR,0x0a);  // �Զ��ط���ʱ�ȴ�250us+86us���Զ��ط�10��
	nrf24l01_write_reg(W_REGISTER+RF_CH,0x40);         // ѡ����Ƶͨ��0x40
	nrf24l01_write_reg(W_REGISTER+RF_SETUP,0x07);    // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������
	nrf24l01_write_reg(W_REGISTER+CONFIG,0x0e);      // CRCʹ�ܣ�16λCRCУ�飬�ϵ�  
	nrf24l01_ce(TRUE);
}

/*****************NRF����Ϊ����ģʽ����������******************************/
//��Ҫ����ģʽ
void nrf24l01_set_rx_mode(void)
{
	nrf24l01_ce(FALSE);  
	nrf24l01_write_buf(W_REGISTER+RX_ADDR_P0,nrf24l01_addr,TX_ADDR_WITDH);  // �����豸����ͨ��0ʹ�úͷ����豸��ͬ�ķ��͵�ַ
	nrf24l01_write_reg(W_REGISTER+EN_AA,0x01);               // ʹ�ܽ���ͨ��0�Զ�Ӧ��
	nrf24l01_write_reg(W_REGISTER+EN_RXADDR,0x01);           // ʹ�ܽ���ͨ��0
	nrf24l01_write_reg(W_REGISTER+RF_CH,0x40);                 // ѡ����Ƶͨ��0x40
	nrf24l01_write_reg(W_REGISTER+RX_PW_P0,TX_DATA_WITDH);  // ����ͨ��0ѡ��ͷ���ͨ����ͬ��Ч���ݿ��
	nrf24l01_write_reg(W_REGISTER+RF_SETUP,0x07);            // ���ݴ�����1Mbps�����书��0dBm���������Ŵ�������*/
	nrf24l01_write_reg(W_REGISTER+CONFIG,0x0f);              // CRCʹ�ܣ�16λCRCУ�飬�ϵ磬����ģʽ
	nrf24l01_ce(TRUE);
}

/****************************���Ӧ���ź�******************************/
uint32_t nrf24l01_check_ack(void)
{
	uint8_t ret;
	ret = nrf24l01_read_reg(R_REGISTER+STATUS);                    // ����״̬�Ĵ���
	if(ret & STATUS_TX_DS){
		nrf24l01_write_reg(W_REGISTER+STATUS, STATUS_TX_DS);  // ���TX_DS��MAX_RT�жϱ�־
		return NRF24L01_TRANSFER_OK;
	}else if(ret & STATUS_MAX_RT){
		nrf24l01_write_reg(W_REGISTER+STATUS, STATUS_MAX_RT);  // ���TX_DS��MAX_RT�жϱ�־
		nrf24l01_csn(FALSE);
		nrf24l01_spi(FLUSH_TX);
		nrf24l01_csn(TRUE); 
		return NRF24L01_TRANSFER_MAX_RETRY;
	}
	return NRF24L01_TRANSFER_PROGRESS;
}

/******************�ж��Ƿ�����յ����ݣ��ӵ��ʹ�RXȡ��*********************/
//���ڽ���ģʽ
uint32_t nrf24l01_receive_data(uint8_t *RevDate)
{
	uint8_t RevFlags=0;
	uint32_t ret;
	ret = nrf24l01_read_reg(R_REGISTER+STATUS);//�������ݺ��ȡ״̬�Ĵ���
	//stm_printf("status: %x\n", ret);
	if(ret & STATUS_RX_DR)				// �ж��Ƿ���յ�����
	{
		nrf24l01_write_reg(W_REGISTER+STATUS,0xff); //���յ����ݺ�RX_DR,TX_DS,MAX_PT���ø�Ϊ1��ͨ��д1������жϱ�
		nrf24l01_read_buf(R_RX_PAYLOAD,RevDate,RX_DATA_WITDH);// ��RXFIFO��ȡ����
		RevFlags=1;	   //��ȡ������ɱ�־		
	}
	/* Check if fifo is not empty */
	if(!RevFlags)
	{
		if((ret & STATUS_RX_P_NO) != STATUS_RX_P_NO)
		{
			nrf24l01_read_buf(R_RX_PAYLOAD,RevDate,RX_DATA_WITDH);// ��RXFIFO��ȡ����
			RevFlags=1;	   //��ȡ������ɱ�־
		}
	}	
	return RevFlags;
}
#endif

