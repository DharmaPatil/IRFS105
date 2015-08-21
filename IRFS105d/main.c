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
//#include "inc/soft_spi.h"
//#include "inc/i2csoft.h"
#include "inc/cc2500.h"
//#include "inc/lm75.h"
//#include "inc/adc.h"
#include "inc/sensors.h"
#include "inc/ct_assert.h"
#include "inc/fsm_timers.h"
#include "inc/fsm_messages.h"

/* TODO:
Установить мощность 0dBm в регистре PATABLE
задержка 150 msek - max время конверсии STLM75 по паспорту
Если изменилось на 1,0°C или более, то bizmen=1
время сканирования инфракрасного датчика из EEPROM
время сканирования фотодатчика вскрытия корпуса из EEPROM
время опроса герконового датчика из EEPROM
время опроса температуры из EEPROM
время опроса напряжения питания из EEPROM
время передачи пакета в случае отсутствия других пакетов для передачи из EEPROM

*/

/*состояния конечного автомата*/
typedef enum { CC_IDLE=0, CC_TX, CC_RX, CC_CAL } CC_State_t;

/* Global variables **************************************************************/
volatile uint8_t sys_timer = 0;
uint8_t smphr_sleep = 0; //sleep semaphore
extern uint8_t ram_cc_Settings[][2]; //можно считывать с флешки или еепром

Settings_t EEMEM saved_settings =
{
  {5,5,5,20,5,250},  //timers
  3,1,            //thresholds
  {
    {CC2500_ADDR,         0x00},
    {CC2500_CHANNR,       0x00},
    {CC2500_FREQ2,        0x5B},
    {CC2500_FREQ1,        0x3E},
    {CC2500_FREQ0,        0x0D}
  },
  0,    //wdt cnt
  1,    //write cnt
  0x0   //crc
};



uint8_t data[7] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfa};
/* END Global variables **********************************************************/

/*Function prototypes ************************************************************/
inline void InitSystemTimer(void);
inline void InitGPIO(void);
inline void InitEXTI(void);
//прототипы функций состояний конечных автоматов
void ccIdle(void);
void ccTx(void);
void ccRx(void);
void ccCal(void);
/* END Func Prototupes ************************************************************/

/* array function pointers*/
void (*cc_table_state[])(void) = {ccIdle, ccTx, ccRx, ccCal};

CC_State_t CC_state = CC_IDLE;

int main(void)
{
    Settings_t global_settings;

    //STATIC_ASSERT(sizeof(uint16_t) == 2);
    //copy stored settings to RAM
    eeprom_read_block((void *)&global_settings, (const void *)&saved_settings, sizeof(Settings_t) );
    cc_conf_replace((uint8_t **)ram_cc_Settings, &global_settings);

    InitGPIO();
    InitSPI_soft();
    InitI2C_soft();
    InitADC();
    _delay_ms(2000);//wait before enable async timer2
    InitSystemTimer();
    InitTimers();
    InitMessages();

    InitCC2500(ram_cc_Settings); //(const uint8_t **)conf(+6bytes of code), preferredSettings
    InitEXTI();
    //MCUCR |= (_BV(ISC11) | _BV(ISC01));

    /*check Watchdog reset flag*/
    if(bit_is_set(MCUCSR, WDRF)) {
      //increase wathcdog reset counter and save in eeprom
    }

    set_sleep_mode(SLEEP_MODE_PWR_SAVE); //варианты SLEEP_MODE_PWR_SAVE SLEEP_MODE_IDLE SLEEP_MODE_ADC
    //wdt_enable(WDTO_2S);
    sei(); //enable interrupts
    smphr_sleep++;

    while(1) {
      #ifndef NDEBUG
      //PORTB |= _BV(PB3);
      _delay_ms(50);
      #endif // NDEBUG
      cc_table_state[CC_state]();

      ProcessTimers(&sys_timer);
      ProcessMessages();
      wdt_reset();

      /*enter in sleep mode if sleep semaphore is null, until interrupts occured*/
      cli(); //disable interrupts
      if (!smphr_sleep)
      {
        #ifndef NDEBUG
        PORTB &= ~_BV(PB3);
        #endif // NDEBUG
        sleep_enable();
        sei();
        sleep_cpu();
        sleep_disable();
      }
      sei();
    }

    return 0;
}

/* INTERRUPT HANDLERS *************************************************************/
ISR(TIMER2_OVF_vect) {

	sys_timer++;
	#ifndef NDEBUG
  PORTB |= _BV(PB3);
  _delay_ms(20);
  PORTB &= ~_BV(PB3);
  _delay_ms(20);
  #endif // NDEBUG

#ifndef ASYNC_TIMER
	TCNT2 = TIMER2_PRELOAD;
#endif // ASYNC_TIMER
}

ISR(INT0_vect) { //По низкому уровню на ножке INT0 (кнопка Send)

}

ISR(INT1_vect) { //По низкому уровню на ножке INT1 (кнопка Калибровка)

}
/* ********************************************************************************/

/* FUNCTIONS **********************************************************************/
inline void InitSystemTimer(void) {
  // Init Timer2
#ifdef ASYNC_TIMER
  //Disable timer2 interrupts
  TIMSK  = 0;
  //Enable asynchronous mode
  ASSR  = _BV(AS2);
  //set initial counter value
  //TCNT2=0;
  //set prescaller 128
  TCCR2 |= (_BV(CS22) | _BV(CS20) );
  //wait for registers update
  while (ASSR & _BV(TCR2UB)) { //while (ASSR & (_BV(TCN2UB) | _BV(TCR2UB)));
      	#ifndef NDEBUG
        PORTB |= _BV(PB3);//blink
        _NOP();
        PORTB &= ~_BV(PB3);
        _NOP();
        _NOP();
        //_NOP();
        //_NOP();
        #endif // NDEBUG
  }
  //clear interrupt flags
  TIFR  = _BV(TOV2);
  //enable TOV2 interrupt
  TIMSK  = _BV(TOIE2);
#else
	TCCR2 = _BV(CS22);	// Set prescaler 64 -> F = 1MHz/64/156 = 100Hz (10ms) system timer. 32768Hz Xtal not used
	TCNT2 = TIMER2_PRELOAD;	// Preload TCNT0 value to get 125 division coefficient
	TIMSK |= _BV(TOIE2); 		// Enable Timer2 overflow interrupt
#endif // ASYNC_TIMER
}

inline void InitGPIO(void) {
  DDRC &= ~(_BV(PC2));   // PC2 Геркон Input
  DDRC |= (_BV(PC5) | _BV(PC1));    // PC5 IR Photodiode Out PC1 power divider
  PORTC &= ~(_BV(PC1));
  PORTC |= (_BV(PC2));   // PC2 Геркон Pull Up


  DDRB &= ~(_BV(PB5)); // PB5 emitter phototransistor
  DDRB |= (_BV(PB3) | _BV(PB7) | _BV(PB4));     // PB3 светодиод индикатор Out, PB7 STLM75 VDD, PB4 Case VDD
  PORTB &= ~(_BV(PB4) | _BV(PB5) | _BV(PB7)); // Case VDD OFF, pullup OFF, STLM75 VDD OFF

        // PB4 collector phototransistor

  DDRD |= (_BV(PD0));                  // PD0 IR LED

  DDRD &= ~(_BV(PD1) | _BV(PD2) | _BV(PD3)); //PD1 INT calibrate, PD2 INT send, PD3 STLM75 INT input
  PORTD |= (_BV(PD1) | _BV(PD2));    //interrups pin pull up
}

inline void InitEXTI(void) {
  MCUCR |= (_BV(ISC11) | _BV(ISC01)); // enable extarnal interrupts on falling edge
}

/*search and replace register of cc2500 conf*/
void cc_conf_replace(uint8_t* cc_init_conf[2] , Settings_t* patch) {
  for(uint8_t i = 0; i<=CC2500_REPLACED_REG; i++) {
    for(uint8_t j = 0; j<=CC_N_REG; j++) {
      if( cc_init_conf[j][0] == patch->cc2500_rw_settings[i][0]) {
        cc_init_conf[j][1] = patch->cc2500_rw_settings[i][0];
        break;
      }
    }
  }
}

/* FSM functions *******************************************************************/
void ccIdle(void) {
  _delay_ms(700);
  //PORTB |= _BV(PB3);
  _delay_ms(100);
  //PORTB &= ~_BV(PB3);
  CC_state=CC_TX;
}

void ccTx(void) {
  command(SIDLE);    //turn CC2500 into idle mode
  command(SFTX);      //flush tx FIFO

  cc2500_fifo_write(data, 7);
  command(STX);  //command to send data in tx FIFO wirelessly
  //ждать пока не закончится передача пакета и трансивер не вернется в состояние IDLE
  //(нужна соответствующая настройка регистра MCSM0)
  while( (MASK_MARCSTATE(cc2500_get_status(CC2500_MARCSTATE)) != MARCSTATE_IDLE_STATE) ) {
    ;
  }
  command(SFTX);      //flush tx FIFO
  //send();

  CC_state=CC_IDLE;
}

void ccRx(void) {
  ;
}

void ccCal(void) {
//  if(ir_sensor(3) == open) {
//    PORTB |= _BV(PB3);
//  }
//  else {
//    PORTB &= ~_BV(PB3);
//  }

  //data[1] = (uint8_t)get_temp();

  CC_state = CC_TX;
}

/* END FUNCTIONS*********************************************************************/



