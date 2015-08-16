/********************************************************
* lm75.h
* aethylic@gmail.com
* date
********************************************************/

#ifndef LM75_H_INCLUDED
#define LM75_H_INCLUDED

#include "inc/i2csoft.h"
#include <stdint.h>

#define LM75_I2C_ADDR_MASK(x) (0x90 | (((x)<<1)&0x0E))
#define LM75_I2C_READ         (0x1)

#define LM75_POINTER_MASK(x) (0x3 & (x))
#define LM75_TEMP_REG     0x0
#define LM75_CONF_REG     0x1
#define LM75_THYS_REG     0x2
#define LM75_TOS_REG      0x3

#define LM75_CONF_MASK(x) (0x1F & (x))
#define LM75_FT1_BIT (1<<4)
#define LM75_FT0_BIT (1<<3)
#define LM75_POL_BIT (1<<2)
#define LM75_M_BIT   (1<<1)
#define LM75_SD_BIT  (1<<0)


void LM75_set_pointer(uint8_t addr, uint8_t ptr);
void LM75_set_conf(uint8_t addr, uint8_t data); //use after LM75_set_pointer!
int16_t LM75_TempRead(uint8_t addr);



#endif // LM75_H_INCLUDED
