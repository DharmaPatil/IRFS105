/*
Copyright 2013 Duong Quang Ha
This file is part of Simple AVR Library
Simple AVR Library is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
Simple AVR Library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with Simple AVR Library. If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>
#include <util/delay.h>
#include "inc/adc.h"

void InitADC () {
  ADMUX |= (_BV(REFS1) | _BV(REFS0) | _BV(ADLAR)); //internal REF, left adjust result
  ADCSRA = (_BV(ADEN) | _BV(ADPS0) | _BV(ADPS1)); //division 8, freq = 125kHz
}


uint8_t read_ADC(uint8_t ch) {
  ADMUX = ((0x0F & ch) | (ADMUX & 0xF0)); //set channel
  //ADMUX &= 0xF0; //clear channel
  //ADMUX |= _BV(ch);  //set channel
  _delay_us(10);
  ADCSRA |= (_BV(ADSC) | _BV(ADEN) ); //enable ADC(if disabled) and start single conversion
  //while ( bit_is_set(ADCSRA, ADSC) ); //wait end of conversion in blocking mode
  while(bit_is_clear(ADCSRA, ADIF)); //wait end of conversion in blocking mode
  ADCSRA |= _BV(ADIF);
  //ADCSRA |= (1 << ADIF);
  // перед уходом в спящий режим выключить АЦП, тут или в конечном автомате
  return (ADCH);
}

