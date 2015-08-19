/********************************************************
* @file cc2500.h
* @author Tejas Dhagawkar,
*         aethylic@gmail.com
* @date  09.08.15
* @brief functions for work with IR LED sensors
********************************************************/

#include "inc/sensors.h"
void led_count(uint8_t cnt);

door_state_t ir_sensor(uint8_t threshold) {
  door_state_t state = undefined;
  uint8_t baseline=0, active_level=0;

  SENSOR_DOOR_VDD_ON;
  _delay_ms(PHOTODIODE_DELAY);
  baseline = read_ADC(SENSOR_DOOR_CHAN);

  if(baseline > DOOR_SUN_LVL) {
    SENSOR_DOOR_VDD_OFF;
    state = open;
  }
  else {
    IRLED_ON;
    active_level = read_ADC(SENSOR_DOOR_CHAN);
    IRLED_OFF;
    SENSOR_DOOR_VDD_OFF;

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
  case_state_t state = closed;

  CASE_SENS_VDD_ON;
  if(CASE_SENS_STATE) {
    state = open;
  }
  CASE_SENS_VDD_OFF;

  return state;
}

uint8_t get_vbat(void) {
  uint8_t voltage;

  VBAT_DIVIDER_ON;
  voltage = read_ADC(VBAT_CHAN);
  VBAT_DIVIDER_OFF;
  // 20,078 mV/bit
  return voltage;
}

int16_t get_temp(void) {
  int16_t temp=0xFE; //init temperature +254C
  LM75_VDD_ON;
  temp = LM75_TempRead(LM75_ADDR);
  LM75_VDD_OFF;

  return temp;
}

#ifndef NDEBUG
void led_count(uint8_t cnt) {
  for(uint8_t i = cnt; i>0; i--) {
    PORTB |= _BV(PB3);
    _delay_ms(1);
    PORTB &= ~_BV(PB3);
    _delay_ms(1);
  }
}
#endif // NDEBUG


