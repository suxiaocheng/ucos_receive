#include "debug.h"
#include "stdarg.h"

#define BAUDRATE        115200

uint8_t init_debug_fun(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_ClocksTypeDef RCC_ClocksStatus;
	uint16_t br_interger, br_fractionaldivider;

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	USART1->CR1 &= ~(USART_CR1_RE | USART_CR1_TE);	/* Disable Receive and transmit */

	USART1->CR1 |= USART_CR1_UE;	/* Enable usart */
	USART1->CR1 &= ~USART_CR1_M;	/* Word length 8 bits */
	USART1->CR2 = (USART1->CR1 & ~USART_CR2_STOP);	/* 1 stop bit */

	RCC_GetClocksFreq(&RCC_ClocksStatus);
	br_interger = RCC_ClocksStatus.PCLK2_Frequency / (16 * BAUDRATE);
	br_fractionaldivider =
	    RCC_ClocksStatus.PCLK2_Frequency / BAUDRATE - br_interger * 16;
	USART1->BRR = (br_interger << 4) | br_fractionaldivider;

	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE;	/* Enable Receive and transmit */

	/* Enable receive && transfer interrupt */
	USART1->CR1 |= USART_CR1_RXNEIE;

	return 0;
}

void USART1_IRQHandler(void)
{
	uint8_t uart_status_reg;
	volatile uint8_t tmp;
	uart_status_reg = USART1->SR;
	if (uart_status_reg & USART_SR_RXNE) {
		tmp = USART1->DR;
	}
}

void serial_put_binary(uint8_t * dat_bin, uint32_t len)
{
	while (len--) {
		USART1->DR = *dat_bin++;
		/* TC is clear after reading the SR reg */
		while ((USART1->SR & USART_SR_TXE) == 0) {
		}
	}
}

uint8_t stm_putc(uint8_t str)
{
	USART1->DR = str;
	while ((USART1->SR & USART_SR_TXE) == 0) {
	}

	return 0;
}

char stm_puts(char *str)
{
#if OS_CRITICAL_METHOD == 3u	/* Allocate storage for CPU status register               */
	OS_CPU_SR cpu_sr = 0u;
#endif
	char *p = str;

	OS_ENTER_CRITICAL();

	while (*p) {
		if (*p == '\n') {
			stm_putc('\r');
		}
		stm_putc(*p++);
	}

	OS_EXIT_CRITICAL();

	return *p;
}

#ifdef DEBUG
int stm_printf(const char *fmt, ...)
{
#if OS_CRITICAL_METHOD == 3u	/* Allocate storage for CPU status register               */
	OS_CPU_SR cpu_sr = 0u;
#endif
	va_list args;
	const char *str = fmt;

	OS_ENTER_CRITICAL();

	va_start(args, fmt);

	while (*str != '\0') {
		if (*str != '%') {
			if (*str == '\n') {
				stm_putc('\r');
			}
			stm_putc(*str);
			++str;
			continue;
		} else {
			++str;
			switch (*str) {
			case 'd':
				{
					int val = va_arg(args, int);
					char res[16] = { 0 };
					DECToASCII(val, res);
					stm_puts(res);
				}
				break;
			case 'x':
				{
					int val = va_arg(args, int);
					char res[16] = { 0 };
					HexToASCII(val, res);
					stm_puts(res);
				}
				break;
			case 'c':
				{
					char ch = va_arg(args, int);
					stm_putc(ch);
				}
				break;
			case 's':
				{
					char *res = va_arg(args, char *);
					stm_puts(res);
				}
				break;
			}
			++str;
		}
	}
	va_end(args);

	OS_EXIT_CRITICAL();

	return 0;
}
#endif

void system_dump(char *str)
{
	while (1) {
		stm_printf(str);
		OSTimeDlyHMSM(0, 0, 0, 1000);
	}
}
