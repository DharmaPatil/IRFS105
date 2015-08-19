/********************************************************
* @file cc2500.c
* @author aethylic@gmail.com
* @date  09.08.15
********************************************************/

#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#define TIMER2_PRELOAD 156
#define RS485_BAUDRATE 19200
#define RS485_DE_HIGH  (PORTD |= _BV(PD2))
#define RS485_DE_LOW   (PORTD &= ~(_BV(PD2)))

#define BUTTON_SEND_STATE    (!!(PIND & _BV(PD3)))
#define BUTTON_BIND_STATE    (!!(PIND & _BV(PD3)))

#define OUTPUT1_HIGH      (PORTC |= _BV(PC2))
#define OUTPUT1_LOW       (PORTC &= ~_BV(PC2))
#define OUTPUT2_HIGH      (PORTC |= _BV(PC3))
#define OUTPUT2_LOW       (PORTC &= ~_BV(PC3))
#define OUTPUT3_HIGH      (PORTC |= _BV(PC4))
#define OUTPUT3_LOW       (PORTC &= ~_BV(PC4))
#define OUTPUT4_HIGH      (PORTC |= _BV(PC5))
#define OUTPUT4_LOW       (PORTC &= ~_BV(PC5))


#include <stdint.h>

#include "inc/ct_assert.h"

/* Exported types ------------------------------------------------------------*/
typedef struct __Settings_t{
  uint8_t a;
}Settings_t;

STATIC_ASSERT(sizeof(Settings_t)<512); //512 áàéò â ÅÅÏÐÎÌ

#endif // MAIN_H_INCLUDED
