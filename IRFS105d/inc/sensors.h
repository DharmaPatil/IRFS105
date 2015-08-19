/********************************************************
* @file sensors.h
* @author aethylic@gmail.com
* @date  18.08.15
* @brief
********************************************************/

#ifndef SENSORS_H_INCLUDED
#define SENSORS_H_INCLUDED

#include <avr/io.h>
#include <util/delay.h>
#include "inc/adc.h"
#include "inc/lm75.h"
#include "inc/ct_assert.h"

#define LM75_ADDR              0x00
#define PHOTODIODE_DELAY       5 //ms
#define DOOR_BASELINE_READINGS 8
#define DOOR_SUN_LVL           0xFA // 0xFA 0x14

#define IRLED_ON              (PORTD |= _BV(PD0))
#define IRLED_OFF             (PORTD &= ~_BV(PD0))
#define SENSOR_DOOR_VDD_ON    (PORTC |= _BV(PC5))
#define SENSOR_DOOR_VDD_OFF   (PORTC &= ~_BV(PC5))
#define SENSOR_DOOR_CHAN      0x04
#define VBAT_DIVIDER_ON       (PORTC &= ~_BV(PC1)) //low level is active
#define VBAT_DIVIDER_OFF      (PORTC |= _BV(PC1))  //high level is powersave
#define VBAT_CHAN             0x07
#define CASE_SENS_VDD_ON      (PORTB |= _BV(PB4))
#define CASE_SENS_VDD_OFF     (PORTB &= ~_BV(PB4))
#define CASE_SENS_STATE       (PINB & _BV(PB5))
#define LM75_VDD_ON           (PORTB |= _BV(PB7))
#define LM75_VDD_OFF          (PORTB &= ~_BV(PB7))

/* exported types*/
typedef enum {open, closed, undefined} door_state_t;
typedef enum {case_closed, case_open} case_state_t;

/* function prototypes*/
door_state_t ir_sensor(uint8_t threshold);
case_state_t check_intrusion(void);
uint8_t get_vbat(void);
int16_t get_temp(void);


#endif // SENSORS_H_INCLUDED

