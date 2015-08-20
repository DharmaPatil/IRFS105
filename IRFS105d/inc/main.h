/********************************************************
* @file cc2500.c
* @author aethylic@gmail.com
* @date  09.08.15
********************************************************/

#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#define TIMER2_PRELOAD 156

#include <stdint.h>

//#include <inc/cc2500.h>
#include "inc/ct_assert.h"

/* Exported types ------------------------------------------------------------*/
typedef struct __Timers_t {
  uint8_t temp;
  uint8_t door;
  uint8_t intrusion;
  uint8_t vbat;
  uint8_t reed;
  uint8_t alive;
}Timers_t;

#define CC2500_REPLACED_REG 5
/*
cc2500_rw_settings[5][2] =
{
  {CC2500_ADDR,         0x00},
  {CC2500_CHANNR,       0x00},
  {CC2500_FREQ2,        0x5B},
  {CC2500_FREQ1,        0x3E},
  {CC2500_FREQ0,        0x0D}
};
*/

typedef struct __Settings_t{
  Timers_t timer;
  uint8_t door_threshold;
  uint8_t temp_threshold;
  uint8_t cc2500_rw_settings[CC2500_REPLACED_REG][2];
  uint32_t wdt_cnt;
  uint32_t write_cnt;
  uint16_t crc;
}Settings_t;

void cc_conf_replace(uint8_t* cc_init_conf[2] , Settings_t* patch);

STATIC_ASSERT(sizeof(Settings_t)<512); //512 áàéò â ÅÅÏÐÎÌ

#endif // MAIN_H_INCLUDED
