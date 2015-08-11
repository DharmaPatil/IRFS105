/********************************************************
* @file cc2500.c
* @author aethylic@gmail.com
* @date  09.08.15
********************************************************/

#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#define TIMER2_PRELOAD 156

#include <stdint.h>

#include "inc/ct_assert.h"

/* Exported types ------------------------------------------------------------*/
typedef struct __Settings_t{
  uint8_t a;
}Settings_t;

STATIC_ASSERT(sizeof(Settings_t)<512); //512 áàéò â ÅÅÏÐÎÌ

#endif // MAIN_H_INCLUDED
