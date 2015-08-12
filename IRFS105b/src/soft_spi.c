/**
 *  @file soft_spi.c
 *  @date 09.08.2015
 *  @author aethylic@gmail.com
 *  @brief
 */

#include "inc/soft_spi.h"


/*
А чем не угодила битовая математика? Ну и оно было бы короче и читабельнее в виде SPI_SDI = (d & 0x80)? 1: 0;

    if (SPI_SDO == 1) { spiReadData |= 0x01; }

Аналогично, чем не угодила битовая математика? spiReadData |= SPI_SDO.
Ну и поскольку SPI — шина дуплексная, то обычно делают одну функцию — обмен данными.
 Она одновременно передает и принимает байт. А дальше для при передаче выкидывается принятый байт,
  при приеме — передается пустой байт. Кроме того, бывают случаи, когда нужны оба одновременно.*/

// инициализация SPI
void InitSPI_soft(void) {
   // первоначаотное положение линий SPI
   DDRD  |= _BV(PD5);  //выход - Chip Select
   PORTD |= _BV(PD5);  //1

   DDRB  |= _BV(PB1);  //выход - SCLK
   PORTB &= ~_BV(PB1); //0

   DDRB  |= _BV(PB2);  //выход - MOSI
   PORTB &= ~_BV(PB2); //0

   DDRB  &= ~_BV(PB0); //вход  - MISO
   PORTB |= _BV(PB0);  //1
}

#if defined(CPOL0_CPHA0)
//// инициализация SPI
//inline void InitSPI_soft(void) {
//   // первоначаотное положение линий SPI
//   DDRD  |= _BV(PD5);  //выход - Chip Select
//   PORTD |= _BV(PD5);  //1
//
//   DDRB  |= _BV(PB1);  //выход - SCLK
//   PORTB &= ~_BV(PB1); //0
//
//   DDRB  |= _BV(PB2);  //выход - MOSI
//   PORTB &= ~_BV(PB2); //0
//
//   DDRB  &= ~_BV(PB0); //вход  - MISO
//   PORTB |= _BV(PB0);  //1
//}

// Заменить на макросы или инлайны
// СТАРТ последовательность
void _spi_start(void) {
  //SCK_PORT &= (~(1<<SCK_PIN));
   SCK_LOW; // SPI_MODE = 0
   CS_LOW; // Chip Select - Enable
}
// СТОП последовательность
void _spi_stop(void) {
   CS_HIGH; // Chip Select - Disable
   SCK_LOW; // SPI_MODE = 0
}

/*void spi_send_byte(uint8_t data) {
   // отправить 1 байт
   for(uint8_t i=0; i<8; i++)
   {
      //CLRWDT();

      // проверить старший бит = 1
      if (data & 0x80)
      {
         MOSI_HIGH; // передать 1
      }
      else
      {
         MOSI_LOW; // передать 0
      }

      SCK_HIGH; // синхроимпульс
      data <<= 1;     // сдвиг для передачи след бита
      //NOP();
      //NOP();
      SCK_LOW; // синхроимпульс
   }
}*/
/*
uint8_t spi_read_byte(void) {
    uint8_t spiReadData=0;

   for(uint8_t i=0; i<8; i++) {
      //CLRWDT();

      spiReadData <<= 1; // сдвиг для передачи след бита

      SCK_HIGH; // синхроимпульс
      NOP();
      if (MISO_STATE == 1) { spiReadData |= 0x01; } // читаем бит
      //NOP();
      //NOP();
      SCK_LOW; // синхроимпульс
   }

   return spiReadData;

}*/

/* read and write one byte through SPI*/
uint8_t spi_TxRx(uint8_t data) {
    uint8_t spiReadData = 0;

    for(uint8_t i = 0; i<8; i++) {
      spiReadData <<= 1; // сдвиг для передачи след бита

      //(data & 0x80) ? MOSI_HIGH : MOSI_LOW; //проверить не ругается ли компилятор на неиспользование резултата выражения

      if (data & 0x80) {
        MOSI_HIGH; // передать 1
      }
      else {
        MOSI_LOW; // передать 0
      }
      _NOP();
      _NOP();
      _NOP();
      _NOP();
      SCK_HIGH; // синхроимпульс
      data <<= 1;     // сдвиг для передачи след бита
      //if (MISO_STATE == 1) { spiReadData |= 0x01; } // читаем бит
      spiReadData |= MISO_STATE; // читаем бит
      //_NOP();
      SCK_LOW; // синхроимпульс
    }
    MOSI_LOW;
    return spiReadData;
}

#elif defined(CPOL0_CPHA1)

#elif defined(CPOL1_CPHA0)

#elif defined(CPOL1_CPHA1)
//Нужно переписать более переносимо
void spi_send_byte(uint8_t data) {
  // ????????? 1 ????
  for(uint32_t i=0; i<8; i++) {

    //CLRWDT();
		SCK_LOW;
		//__NOP();
    // ????????? ????? ??? = 1
    if (data & 0x80) {
      MOSI_HIGH; // ???????? 1
    }
    else {
      MOSI_LOW; // ???????? 0
    }

     // ?????????????
    data <<= 1;     // ????? ??? ???????? ???? ????
		__NOP();
		SCK_HIGH; // ?????????????
    __NOP();
    __NOP();
		__NOP();
    __NOP();
   }

}

uint8_t spi_read_byte(void) {
	//
	uint8_t spiReadData=0;
	//__NOP();
	//__NOP();
	//__NOP();

   for(uint32_t i=0; i<8; i++) {
     //CLRWDT();
     SCK_LOW; // ?????????????
     spiReadData <<= 1; // ????? ??? ???????? ???? ????
     __NOP();
		 __NOP();
		 __NOP();
		 __NOP();
		 __NOP();
		 __NOP();
		 //__NOP();

		 SCK_HIGH; // ?????????????
     if (MISO_STATE != (uint32_t)GPIO_PIN_RESET) { spiReadData |= 0x01; } // ?????? ???
		 //__NOP();
		 //__NOP();
		 //__NOP();
   }

   return spiReadData;

}

#else
 #error "define SPI mode polarity and phase"
#endif // CPOL0_CPHA0


