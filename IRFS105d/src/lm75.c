/********************************************************
* lm75.c
* aethylic@gmail.com
* date
********************************************************/

#include "inc/lm75.h"

void LM75_set_pointer(uint8_t addr, uint8_t ptr) {
  SoftI2CStart();
  while(1) {
    if(!SoftI2CWriteByte(LM75_I2C_ADDR_MASK(addr))) break; //break if LM75 not ACK
    SoftI2CWriteByte(LM75_POINTER_MASK(ptr)); //set pointer
    break;
  }
  SoftI2CStop();
}

void LM75_set_conf(uint8_t addr, uint8_t data) {
  SoftI2CStart();
  while(1) {
    if(!SoftI2CWriteByte(LM75_I2C_ADDR_MASK(addr))) break; //break if LM75 not ACK
    SoftI2CWriteByte(LM75_CONF_MASK(data));
  }
  SoftI2CStop();
}

int16_t LM75_TempRead(uint8_t addr) {
  int16_t temp=0;

  SoftI2CStart();
  while(1) {
    if(!SoftI2CWriteByte(LM75_I2C_ADDR_MASK(addr) | LM75_I2C_READ)) break; //break if LM75 not ACK
    temp = (SoftI2CReadByte(1)<<8); //Read high byte
    temp |= (SoftI2CReadByte(0) & 0xE0); // Read low byte. Mask? maybe 0x80
    break;
  }
  SoftI2CStop();
  return temp;
}

