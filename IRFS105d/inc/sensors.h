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

#define PHOTODIODE_DELAY      5 //ms
#define DOOR_BASELINE_READINGS 8
#define DOOR_SUN_LVL      0xFA

#define IRLED_ON              (PORTD |= _BV(PD0))
#define IRLED_OFF             (PORTD &= ~_BV(PD0))
#define SENSOR_DOOR_VDD_ON    (PORTC |= _BV(PC5))
#define SENSOR_DOOR_VDD_OFF   (PORTC &= ~_BV(PC5))
#define SENSOR_DOOR_ADC       0x04
#define VBAT_DIVIDER_ON       (PORTC |= _BV(PC1))
#define VBAT_DIVIDER_OFF      (PORTC &= ~_BV(PC1))
#define VBAT_CHAN ADC7        0x07
#define CASE_SENS_VDD_ON      (PORTD |= _BV(PD0))
#define CASE_SENS_VDD_OFF     (PORTD &= ~_BV(PD0))
#define CASE_SENS_INP


/* exported types*/
typedef enum {open, closed, undefined} door_state_t;
typedef enum {case_closed, case_open} case_state_t;

/* function prototypes*/
door_state_t ir_sensor(uint8_t threshold);
case_state_t check_intrusion(void);
uint8_t get_vbat(void);


#endif // SENSORS_H_INCLUDED

