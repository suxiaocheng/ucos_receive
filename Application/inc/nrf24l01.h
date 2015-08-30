#ifndef NRF_24L01_H
#define NRF_24L01_H

#ifdef __cplusplus
extern "C" {
#endif

#include "includes.h"

/*******************************************************/
#define TX_ADDR_WITDH 5	//发送地址宽度设置为5个字节
#define RX_ADDR_WITDH 5	//接收地址宽度设置为5个字节
#define TX_DATA_WITDH 4//发送数据宽度1个字节
#define RX_DATA_WITDH 4//接收数据宽度1个字节
/*******************命令寄存器***************************/
#define  R_REGISTER      0x00//读取配置寄存器
#define  W_REGISTER      0x20//写配置寄存器
#define  R_RX_PAYLOAD 	 0x61//读取RX有效数据
#define  W_TX_PAYLOAD	 0xa0//写TX有效数据
#define  FLUSH_TX		 0xe1//清除TXFIFO寄存器
#define  FLUSH_RX		 0xe2//清除RXFIFO寄存器
#define  REUSE_TX_PL     0xe3//重新使用上一包有效数据
#define  NOP             0xff//空操作
/******************寄存器地址****************************/
#define  CONFIG          0x00//配置寄存器
#define  EN_AA			 0x01//使能自动应答
#define  EN_RXADDR       0x02//接收通道使能0-5个通道
#define  SETUP_AW        0x03//设置数据通道地址宽度3-5
#define  SETUP_RETR      0x04//建立自动重发
#define  RF_CH           0x05//射频通道设置
#define  RF_SETUP        0x06//射频寄存器
#define  STATUS          0x07//状态寄存器
#define  OBSERVE_TX      0x08//发送检测寄存器
#define  CD              0x09//载波
#define  RX_ADDR_P0      0x0a//数据通道0接收地址
#define  RX_ADDR_P1      0x0b//数据通道1接收地址
#define  RX_ADDR_P2      0x0c//数据通道2接收地址
#define  RX_ADDR_P3      0x0d//数据通道3接收地址
#define  RX_ADDR_P4      0x0e//数据通道4接收地址
#define  RX_ADDR_P5      0x0f//数据通道5接收地址
#define  TX_ADDR         0x10//发送地址
#define  RX_PW_P0        0x11//P0通道数据宽度设置
#define  RX_PW_P1        0x12//P1通道数据宽度设置
#define  RX_PW_P2        0x13//P2通道数据宽度设置
#define  RX_PW_P3        0x14//P3通道数据宽度设置
#define  RX_PW_P4        0x15//P4通道数据宽度设置
#define  RX_PW_P5        0x16//P5通道数据宽度设置
#define  FIFO_STATUS     0x17//FIFO状态寄存器

/* STATUS BIT Define */
#define STATUS_RX_DR		(0x1<<6)
#define STATUS_TX_DS		(0x1<<5)
#define STATUS_MAX_RT		(0x1<<4)
#define STATUS_RX_P_NO		(0x7<<1)
#define STATUS_TX_FULL		(0x1<<0)

/* FIFO_STATUS BIT Define */
#define FIFO_STATUS_TX_REUSE			(0x1<<6)
#define FIFO_STATUS_TX_FULL			(0x1<<5)
#define FIFO_STATUS_TX_EMPTY			(0x1<<4)
#define FIFO_STATUS_RX_FULL			(0x1<<1)
#define FIFO_STATUS_RX_EMPTY			(0x1<<0)

/* Transfer status define */
#define NRF24L01_TRANSFER_OK				0x00
#define NRF24L01_TRANSFER_MAX_RETRY		0x01
#define NRF24L01_TRANSFER_PROGRESS		0x02


uint32_t nrf24l01_init(void);
uint8_t nrf24l01_read_reg(uint8_t RegAddr);
uint8_t nrf24l01_write_reg(uint8_t RegAddr,uint8_t date);
uint8_t nrf24l01_read_buf(uint8_t RegAddr,uint8_t *RxDate,uint8_t DateLen);
uint8_t nrf24l01_write_buf(uint8_t RegAddr,const uint8_t *TxDate,uint8_t DateLen);
void nrf24l01_set_tx_mode(uint8_t *TxDate);
void nrf24l01_set_rx_mode(void);
uint32_t nrf24l01_check_ack(void);
uint32_t nrf24l01_receive_data(uint8_t *RevDate);

#ifdef __cplusplus
}
#endif

#endif
