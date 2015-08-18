/********************************************************
* @file cc2500.h
* @author Tejas Dhagawkar,
*         aethylic@gmail.com
* @date  09.08.15
* @brief functions for work with IR LED sensors
********************************************************/

#include "inc/sensors.h"

door_state_t ir_sensor(uint8_t threshold) {
  door_state_t state = undefined;
  uint8_t baseline=0, active_level=0;

  SENSOR_DOOR_VDD_ON;
  _delay_ms(PHOTODIODE_DELAY);
  //for (uint8_t i=0; i < DOOR_BASELINE_READINGS; i++) {
  baseline = read_ADC(SENSOR_DOOR_ADC);

  for(uint8_t i = baseline; i>0; i--) {
    PORTB |= _BV(PB3);
    _delay_ms(1);
    PORTB &= ~_BV(PB3);
    _delay_ms(1);
  }
  //}

  if(baseline > DOOR_SUN_LVL) {
    SENSOR_DOOR_VDD_OFF;
    state = open;
  }
  else {
    IRLED_ON;
    _delay_ms(10);
    active_level = read_ADC(SENSOR_DOOR_ADC);
    IRLED_OFF;
    SENSOR_DOOR_VDD_OFF;

    for(uint8_t i = active_level; i>0; i--) {
      PORTB |= _BV(PB3);
      _delay_ms(1);
      PORTB &= ~_BV(PB3);
      _delay_ms(1);
    }

    if(active_level > (baseline + threshold)) {
      state = closed;
    }
    else {
      state = open;
    }//end active level check
  }// end baseline check

  return state;
}


case_state_t check_intrusion(void) {
  return closed;
}

uint8_t get_vbat(void) {

  return 0;
}

