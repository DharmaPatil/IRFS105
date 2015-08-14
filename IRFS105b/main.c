/**
*  @file main.c
*  @author aethylic@gmail.com
*  @date  09.08.15
*  @brief project IRFS104d
 */

// F_CPU defined in default build options
//#define F_CPU 1000000UL //1.2 mA current consumption


#include <avr/io.h>
#include <stdint.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/crc16.h>

#include "inc/main.h"
#include "inc/soft_spi.h"
#include "inc/i2csoft.h"
#include "inc/cc2500.h"
#include "inc/adc.h"
#include "inc/ct_assert.h"
#include "inc/fsm_timers.h"
#include "inc/fsm_messages.h"

/* TODO:
UART rx 30, DE PD2, tx 31
���������� �������� 0dBm � �������� PATABLE
�������� 150 msek - max ����� ��������� STLM75 �� ��������
���� ���������� �� 1,0�C ��� �����, �� bizmen=1
����� ������������ ������������� ������� �� EEPROM
����� ������������ ����������� �������� ������� �� EEPROM
����� ������ ����������� ������� �� EEPROM
����� ������ ����������� �� EEPROM
����� ������ ���������� ������� �� EEPROM
����� �������� ������ � ������ ���������� ������ ������� ��� �������� �� EEPROM

*/

/*��������� ��������� ��������*/
typedef enum { CC_IDLE=0, CC_TX, CC_RX, CC_CAL } CC_State_t;

/* Global variables **************************************************************/
volatile uint8_t sys_timer = 0;
/* END Global variables **********************************************************/

/*Function prototypes ************************************************************/
inline void InitSystemTimer(void);
inline void InitGPIO(void);
//inline void InitEXTI(void);
//��������� ������� ��������� �������� ���������
void ccIdle(void) __attribute__ ((used));
void ccTx(void) __attribute__ ((used));
void ccRx(void) __attribute__ ((used));
void ccCal(void) __attribute__ ((used));
/* END Func Prototupes ************************************************************/

/* array function pointers*/
void (*cc_table_state[])(void) = {ccIdle, ccTx, ccRx, ccCal};

CC_State_t CC_state = CC_IDLE;

int main(void)
{
    //STATIC_ASSERT(sizeof(uint16_t) == 2);
    InitGPIO();
    InitSPI_soft();
    InitI2C_soft();
    InitADC();
    InitSystemTimer();
    InitTimers();
    InitMessages();
    InitCC2500();
    //_delay_ms(5000);
    //InitEXTI();
    //MCUCR |= (_BV(ISC11) | _BV(ISC01));

    /*check Watchdog reset flag*/
    if(bit_is_set(MCUCSR, WDRF)) {
      //increase wathcdog reset counter and save in eeprom
    }

    set_sleep_mode(SLEEP_MODE_IDLE); //�������� SLEEP_MODE_PWR_SAVE SLEEP_MODE_IDLE SLEEP_MODE_ADC
    //wdt_enable(WDTO_2S);
    sei(); //enable interrupts

    for (uint8_t i = 0; i<0x05; i++) {
          //_spi_start();
          //spi_TxRx(0x9D);
          //_spi_stop();
          _delay_ms(100);
          PORTC |= _BV(PC2); //blink for test
          _delay_ms(100);
          PORTC &= ~_BV(PC2);
          _delay_ms(100);
        }

    while(1) {
      cc_table_state[CC_state]();


      ProcessTimers(&sys_timer);
      ProcessMessages();
      wdt_reset();

      /*enter in sleep mode until interrupts occured*/
//      cli(); //disable interrupts
//      if (some_condition)
//      {
//        sleep_enable();
//        sei();
//        sleep_cpu();
//        sleep_disable();
//      }
//      sei();
    }

    return 0;
}

/* INTERRUPT HANDLERS *************************************************************/
ISR(TIMER2_OVF_vect) {
	sys_timer++;
	TCNT2 = TIMER2_PRELOAD;
}

ISR(INT0_vect) { //�� ������� ������ �� ����� INT0 (������ Send)

}

ISR(INT1_vect) { //�� ������� ������ �� ����� INT1 (������ ����������)

}
/* ********************************************************************************/

/* FUNCTIONS **********************************************************************/
inline void InitSystemTimer(void) {
  // Init Timer0
	TCCR2 = _BV(CS22);	// Set prescaler 64 -> F = 1MHz/64/156 = 100Hz (10ms) system timer. 32768Hz Xtal not used
	TCNT2 = TIMER2_PRELOAD;				// Preload TCNT0 value to get 125 division coefficient
	TIMSK |= _BV(TOIE2); 					// Enable Timer2 overflow interrupt
}

inline void InitGPIO(void) {
  //DDRC &= ~_BV(PC2);   // PC2 ������ Input
  DDRC |= ( _BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5) );    // PC2-5 digital Out
  //PORTC |= _BV(PC2);   // PC2 ������ Pull Up
                       // PC1 power???

  DDRB &= ~(_BV(PB5));   // PB5 emitter phototransistor input
  DDRB |= (_BV(PB3) | _BV(PB7) | _BV(PB4));     // PB3 ��������� ��������� Out, PB7 STLM75 VDD, PB4 collector
  PORTB |= _BV(PB4);

  DDRD &= ~(_BV(PD3) | _BV(PD4));    // PD3 send to therminal, PD4 ��������
  DDRD |= _BV(PD0);                  // PD0 IR LED
  PORTD |= (_BV(PD3) | _BV(PD4));    // pullup

  DDRD &= ~(_BV(PD1) | _BV(PD2) | _BV(PD3)); //PD1 INT calibrate, PD2 INT send, PD3 STLM75 INT input
  PORTD |= _BV(PD1) | _BV(PD2);    //interrups pin pull up
}

/*inline void InitEXTI(void) {
  MCUCR |= (_BV(ISC11) | _BV(ISC01)); // enable extarnal interrupts on falling edge
}*/

/* FSM functions *******************************************************************/
void ccIdle(void) {
  //_delay_ms(1000);
  PORTC |= _BV(PC2); //blink for test
  _delay_ms(100);
  CC_state=CC_RX;
}

void ccTx(void) {
  send();
  CC_state=CC_IDLE;
}

void ccRx(void) {
  PORTC &= ~_BV(PC2);
  _delay_ms(100);
  receive();
  CC_state=CC_IDLE;
}

void ccCal(void) {
  ;
}

/* END FUNCTIONS*********************************************************************/



