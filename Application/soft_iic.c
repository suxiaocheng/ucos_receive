#include "soft_iic.h" 	  

/* PB10->SCL */
/* PB11->SDA */

__inline static void iic_scl(uint8_t status)
{
	uint32_t count;
	if (status == TRUE) {
		GPIOB->BSRR = GPIO_Pin_10;
	} else {
		GPIOB->BRR = GPIO_Pin_10;
	}
	/* delay */
	for(count=0; count<200; count++){
		__nop();
	}
}

__inline static void iic_sda(uint8_t status)
{
	if (status == TRUE) {
		GPIOB->BSRR = GPIO_Pin_11;
	} else {
		GPIOB->BRR = GPIO_Pin_11;
	}
}

__inline static uint8_t iic_sda_get_dat(void)
{
	return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
}

/*******************************************************************************
* Function Name  : iic_sda_ctrl.
* Description    : Set the in or out state of the sda
* Input          : in_out: 0x0-> IN, 0x1-> OUT
* Output         : void
* Remark         : 
*******************************************************************************/
__inline static void iic_sda_ctrl(uint8_t in_out)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	if (in_out) {
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	} else {
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;
	}
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name  : iic_init.
* Description    : init the iic io port
* Input          : void
* Output         : void
* Remark         : 
*******************************************************************************/
void iic_init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	iic_scl(TRUE);
	iic_sda(TRUE);
}

/*******************************************************************************
* Function Name  : iic_start.
* Description    : Generate the iic start bit
* Input          : void
* Output         : void
* Remark         : 
*******************************************************************************/
void iic_start(void)
{
	iic_sda_ctrl(IIC_SDA_CTRL_OUT);
	iic_sda(TRUE);
	iic_scl(TRUE);
	iic_sda(FALSE);
	iic_scl(FALSE);
}

/*******************************************************************************
* Function Name  : iic_stop.
* Description    : Generate the iic stop bit
* Input          : void
* Output         : void
* Remark         : 
*******************************************************************************/
void iic_stop(void)
{
	iic_scl(FALSE);
	iic_sda(FALSE);
	iic_scl(TRUE);
	iic_sda(TRUE);	
}

/*******************************************************************************
* Function Name  : iic_write_byte.
* Description    : Write one byte to the iic interface.
* Input          : byte: data to write
* Output         : TRUE when receive ack, otherwise FALSE;
* Remark         : when execute this function, the scl is low, after execute
*                  this function, scl is still low
*******************************************************************************/
uint32_t iic_write_byte(uint8_t byte)
{
	uint32_t count;
	for(count=0; count<8; count++){
		iic_sda((byte&0x80)?TRUE:FALSE);
		byte <<= 1;
		iic_scl(TRUE);
		iic_scl(FALSE);
	}
	
	/* Get the iic ack signal */
	iic_sda(TRUE);
	iic_sda_ctrl(IIC_SDA_CTRL_IN);	
	iic_scl(TRUE);
	
	count = 0;
	while(iic_sda_get_dat()){
		count++;
		if(count > 250){
			iic_scl(FALSE);
			iic_sda_ctrl(IIC_SDA_CTRL_OUT);	
			iic_sda(TRUE);
			iic_stop();
			return FALSE;
		}
	}
	iic_scl(FALSE);
	iic_sda_ctrl(IIC_SDA_CTRL_OUT);	
	iic_sda(TRUE);
	
	return TRUE;
}

/*******************************************************************************
* Function Name  : iic_read_byte.
* Description    : Read one byte from the iic interface.
* Input          : ack: Send ack if it is needed.
* Output         : One byte of data read out from the iic interface
* Remark         : when execute this function, the scl is low, after execute
*                  this function, scl is still low
*******************************************************************************/
uint8_t iic_read_byte(unsigned char ack)
{
	uint32_t count;
	uint8_t byte = 0;
	
	iic_sda_ctrl(IIC_SDA_CTRL_IN);
	for(count=0; count<8; count++){		
		iic_scl(TRUE);
		byte <<= 1;
		if(iic_sda_get_dat()){
			byte |= 0x1;
		}
		iic_scl(FALSE);
	}
	
	iic_sda_ctrl(IIC_SDA_CTRL_OUT);
	/* Is there any ack, when ack!=0, pull down the sda line */
	iic_sda(ack?FALSE:TRUE);
	iic_scl(TRUE);
	iic_scl(FALSE);

	return byte;
}

/*******************************************************************************
* Function Name  : iic_write_bytes.
* Description    : Write multi bytes to the iic interface.
* Input          : addr: iic address
*                  reg: iic device register 
*                  dat: data write to the iic device register
*                  len: the length of dat
* Output         : TRUE when sucessfully transfer data, otherwise FALSE;
* Remark         : 
*******************************************************************************/
uint32_t iic_write_bytes(uint8_t addr, uint8_t reg, uint8_t *dat, uint32_t len)
{
	uint32_t count;
	uint32_t ret;
	
	iic_start();
	
	ret = iic_write_byte(addr);
	if(ret == FALSE){
		goto WRITE_OVER;
	}
	
	ret = iic_write_byte(reg);
	if(ret == FALSE){
		goto WRITE_OVER;
	}
	
	for(count=0; count<len; count++){
		ret = iic_write_byte(*dat++);
		if(ret == FALSE){
			goto WRITE_OVER;
		}
	}
	
WRITE_OVER:
	iic_stop();
	return ret;
}

/*******************************************************************************
* Function Name  : iic_read_bytes.
* Description    : Read multi bytes from the iic interface.
* Input          : addr: iic address
*                  reg: iic device register 
*                  dat: data buffer used receive data
*                  len: the length of data to read
* Output         : TRUE when operation sucessfully.
* Remark         : 
*******************************************************************************/
uint32_t iic_read_bytes(uint8_t addr, uint8_t reg, uint8_t *dat, uint32_t len)
{
	uint32_t count;
	uint32_t ret;
	
	iic_start();
	
	ret = iic_write_byte(addr);
	if(ret == FALSE){
		goto READ_OVER;
	}
	
	ret = iic_write_byte(reg);
	if(ret == FALSE){
		goto READ_OVER;
	}
	
	iic_start();
	
	ret = iic_write_byte(addr|0x01);
	if(ret == FALSE){
		goto READ_OVER;
	}
	
	for(count=0; count<len; count++){
		*dat++ = iic_read_byte((count==(len-1))?FALSE:TRUE);
	}
	
READ_OVER:
	iic_stop();
	return ret;
}

