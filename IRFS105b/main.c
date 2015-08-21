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
#include "inc/uart.h"
#include "inc/soft_spi.h"
#include "inc/i2csoft.h"
#include "inc/cc2500.h"
#include "inc/adc.h"
#include "inc/ct_assert.h"
#include "inc/fsm_timers.h"
#include "inc/fsm_messages.h"

/* TODO:
UART rx 30, DE PD2, tx 31
	out	UBRRH,	h00	;\
	ldi	Temp1,	11	;|Установить скорость по UART 19200 бод (11 при кварце 3.6864 МГц)
	out	UBRRL,	Temp1	;/
	ldi	Temp1,	0x98	;\ Разрешить прием по UART, передачу по UART, прерывание по приему байта по UART
	out	UCSRB,	Temp1	;/
	ldi	Temp1,	0x86	;\ Установить протокол по интерфейсу как "8-битные посылки с одним стоп битом без бита паритета"
	out	UCSRC,	Temp1	;/
Re2:	sbis	UCSRA,	RXC	;\
	rjmp	Re3		;| Сбросить RXC (вроде как не обязательно)
	in	Temp1,	UDR	;|
	rjmp	Re2		;/
Re3:	sbi	UCSRA,	TXC	; Обнулить TXC (на всякий случай)

*/

/*состояния конечного автомата*/
typedef enum { CC_IDLE=0, CC_TX, CC_RX, CC_CAL } CC_State_t;

/* Global variables **************************************************************/
volatile uint8_t sys_timer = 0;
extern const uint8_t preferredSettings[][2]; //можно считывать с флешки или еепром
static uint16_t freq = 0x4B18;
static uint16_t rssi_acc;
/* END Global variables **********************************************************/

/*Function prototypes ************************************************************/
inline void InitSystemTimer(void);
inline void InitGPIO(void);
//inline void InitEXTI(void);
//прототипы функций состояний конечных автоматов
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
    InitCC2500(preferredSettings); //(const uint8_t **)conf(+6bytes of code), preferredSettings
    uart0_init( UART_BAUD_SELECT(RS485_BAUDRATE, F_CPU) );
    //_delay_ms(5000);
    //InitEXTI();
    //MCUCR |= (_BV(ISC11) | _BV(ISC01));

    /*check Watchdog reset flag*/
    if(bit_is_set(MCUCSR, WDRF)) {
      //increase wathcdog reset counter and save in eeprom
    }

    set_sleep_mode(SLEEP_MODE_IDLE); //варианты SLEEP_MODE_PWR_SAVE SLEEP_MODE_IDLE SLEEP_MODE_ADC
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

          RS485_DE_HIGH;
          uart0_putc(0xba);
          _delay_ms(2);
          RS485_DE_LOW;
        }
        RS485_DE_LOW;

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

ISR(INT0_vect) { //По низкому уровню на ножке INT0 (кнопка Send)

}

ISR(INT1_vect) { //По низкому уровню на ножке INT1 (кнопка Калибровка)

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
  //DDRC &= ~_BV(PC2);   // PC2 Геркон Input
  DDRC |= ( _BV(PC2) | _BV(PC3) | _BV(PC4) | _BV(PC5) );    // PC2-5 digital Out
  //PORTC |= _BV(PC2);   // PC2 Геркон Pull Up
                       // PC1 power???

  DDRB &= ~(_BV(PB5));   // PB5 emitter phototransistor input
  DDRB |= (_BV(PB3) | _BV(PB7) | _BV(PB4));     // PB3 светодиод индикатор Out, PB7 STLM75 VDD, PB4 collector
  PORTB |= _BV(PB4);

  DDRD &= ~(_BV(PD0) | _BV(PD3) | _BV(PD4));    // PD0 rxd, PD3 send to therminal, PD4 привязка
  DDRD |= ( _BV(PD1) | _BV(PD2));    // PD1 txd, PD2 rs485 DE
  PORTD &= ~(_BV(PD2));               //default state rs485 DE is LOW
  PORTD |= (_BV(PD3) | _BV(PD4));    // buttons pullup

  DDRD &= ~(_BV(PD1) | _BV(PD3)); //PD1 INT calibrate, PD2 INT send, PD3 STLM75 INT input
  PORTD |= _BV(PD1) | _BV(PD2);    //interrups pin pull up
}

/*inline void InitEXTI(void) {
  MCUCR |= (_BV(ISC11) | _BV(ISC01)); // enable extarnal interrupts on falling edge
}*/

/* FSM functions *******************************************************************/
void ccIdle(void) {
  //_delay_ms(1000);
  PORTC |= _BV(PC2); //blink for test
  _delay_ms(30);

  static uint8_t old_rssi;
  static uint8_t cnt=0;

  if(cnt++ < 8){
    rssi_acc += cc2500_get_status(CC2500_RSSI);
  }
  else {
    cnt = 0;
    if(rssi_acc>(old_rssi + 8)) {
      freq+=5;
      cc2500_change_freq(freq);

    }
    else if(rssi_acc < (old_rssi - 8)) {//check underflow!
      freq-=5;
      cc2500_change_freq(freq);

    }
    old_rssi = rssi_acc;

  }




  CC_state=CC_RX;
}

void ccTx(void) {
  //send();
  CC_state=CC_IDLE;
}

void ccRx(void) {
  PORTC &= ~_BV(PC2);
  _delay_ms(20);

  uint8_t receiver_buf[7];

  command(SFRX); // command to flush RX FIFO
  _delay_ms(1);
  command(SRX);  // command to receive data wirelessly
  while( (MASK_MARCSTATE(cc2500_get_status(CC2500_MARCSTATE)) != MARCSTATE_IDLE_STATE) ) { //ждать пока не закончится прием пакета
    cc2500_get_status(CC2500_PKTSTATUS);
  }
  if ( cc2500_get_status(CC2500_RXBYTES) == 0 ) { //exit if rx fifo empty(autoflush)
      PORTC |= _BV(PC3) | _BV(PC5);
      _delay_ms(200);
      PORTC &= ~(_BV(PC3) | _BV(PC5));
      return;
  }
  cc2500_fifo_read(receiver_buf, 7);
  command(SFRX); // flush receiver FIFO if owerflow state
  command(SIDLE); // turn CC2500 into idle mode
  command(SFRX); // flush receiver FIFO in IDDLE mode
  receiver_buf[2] = *(uint8_t *)&freq;
  receiver_buf[3] = *(((uint8_t *)&freq)+1);
  receiver_buf[4] = cc2500_get_status(CC2500_RSSI);
  receiver_buf[5] = cc2500_get_rssi();
  RS485_DE_HIGH;
  for(uint8_t i=0; i<7; i++) {
    uart0_putc(receiver_buf[i]);
  }
  //uart0_puts((const char *)receiver_buf);
  _delay_ms(10);
  RS485_DE_LOW;
    //test LED ON if packet received and data correct
//  if (receiver_buf[0] == 0x6A) {
//    PORTC |= _BV(PC3);
//    _delay_ms(200);
//  }
//  if (receiver_buf[1] == 0x6A) {
//    PORTC |= _BV(PC4);
//    _delay_ms(200);
//  }
//  if (receiver_buf[2] == 0x6A) {
//    PORTC |= _BV(PC5);
//    _delay_ms(200);
//  }
//  PORTC &= ~( _BV(PC3) | _BV(PC4) | _BV(PC5) );//LED OFF
//  _delay_ms(100);

  //receive();
  //send();
  CC_state=CC_IDLE;
}

void ccCal(void) {
  ;
}

/* END FUNCTIONS*********************************************************************/



