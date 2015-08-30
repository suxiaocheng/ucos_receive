#ifndef NRF_24L01_H
#define NRF_24L01_H

#ifdef __cplusplus
extern "C" {
#endif

#include "includes.h"

/*******************************************************/
#define TX_ADDR_WITDH 5	//���͵�ַ�������Ϊ5���ֽ�
#define RX_ADDR_WITDH 5	//���յ�ַ�������Ϊ5���ֽ�
#define TX_DATA_WITDH 4//�������ݿ��1���ֽ�
#define RX_DATA_WITDH 4//�������ݿ��1���ֽ�
/*******************����Ĵ���***************************/
#define  R_REGISTER      0x00//��ȡ���üĴ���
#define  W_REGISTER      0x20//д���üĴ���
#define  R_RX_PAYLOAD 	 0x61//��ȡRX��Ч����
#define  W_TX_PAYLOAD	 0xa0//дTX��Ч����
#define  FLUSH_TX		 0xe1//���TXFIFO�Ĵ���
#define  FLUSH_RX		 0xe2//���RXFIFO�Ĵ���
#define  REUSE_TX_PL     0xe3//����ʹ����һ����Ч����
#define  NOP             0xff//�ղ���
/******************�Ĵ�����ַ****************************/
#define  CONFIG          0x00//���üĴ���
#define  EN_AA			 0x01//ʹ���Զ�Ӧ��
#define  EN_RXADDR       0x02//����ͨ��ʹ��0-5��ͨ��
#define  SETUP_AW        0x03//��������ͨ����ַ���3-5
#define  SETUP_RETR      0x04//�����Զ��ط�
#define  RF_CH           0x05//��Ƶͨ������
#define  RF_SETUP        0x06//��Ƶ�Ĵ���
#define  STATUS          0x07//״̬�Ĵ���
#define  OBSERVE_TX      0x08//���ͼ��Ĵ���
#define  CD              0x09//�ز�
#define  RX_ADDR_P0      0x0a//����ͨ��0���յ�ַ
#define  RX_ADDR_P1      0x0b//����ͨ��1���յ�ַ
#define  RX_ADDR_P2      0x0c//����ͨ��2���յ�ַ
#define  RX_ADDR_P3      0x0d//����ͨ��3���յ�ַ
#define  RX_ADDR_P4      0x0e//����ͨ��4���յ�ַ
#define  RX_ADDR_P5      0x0f//����ͨ��5���յ�ַ
#define  TX_ADDR         0x10//���͵�ַ
#define  RX_PW_P0        0x11//P0ͨ�����ݿ������
#define  RX_PW_P1        0x12//P1ͨ�����ݿ������
#define  RX_PW_P2        0x13//P2ͨ�����ݿ������
#define  RX_PW_P3        0x14//P3ͨ�����ݿ������
#define  RX_PW_P4        0x15//P4ͨ�����ݿ������
#define  RX_PW_P5        0x16//P5ͨ�����ݿ������
#define  FIFO_STATUS     0x17//FIFO״̬�Ĵ���

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
