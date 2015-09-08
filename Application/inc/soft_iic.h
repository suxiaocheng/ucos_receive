#ifndef SOFT_IIC_H
#define SOFT_IIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "includes.h"
	
#define IIC_SDA_CTRL_IN			0x0
#define IIC_SDA_CTRL_OUT		0x1

void iic_init(void);
uint32_t iic_write_bytes(uint8_t addr, uint8_t reg, uint8_t *dat, uint32_t len);
uint32_t iic_read_bytes(uint8_t addr, uint8_t reg, uint8_t *dat, uint32_t len);
	
#ifdef __cplusplus
}
#endif

#endif
