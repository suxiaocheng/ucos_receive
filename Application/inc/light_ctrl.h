#ifndef LIGHT_CTRL_H
#define LIGHT_CTRL_H

#include "includes.h"

/* BACKLIGHT->PB5 */
__inline void lcd_backlight(uint8_t status)
{
	if (status == HIGH) {
		GPIOB->BSRR = GPIO_Pin_5;
	} else {
		GPIOB->BRR = GPIO_Pin_5;
	}
}

void light_init(void);

#endif
