/**
 *  @file soft_spi.c
 *  @date 09.08.2015
 *  @author aethylic@gmail.com
 *  @brief
 */

#include <avr/interrupt.h>
#include "inc/soft_spi.h"

// инициализация SPI
void InitSPI_soft(void) {
   // первоначаотное положение линий SPI
   CS_DDR  |= _BV(CS_PIN);  //выход - Chip Select
   CS_PORT |= _BV(CS_PIN);  //1

   SCK_DDR  |= _BV(SCK_PIN);  //выход - SCLK
   SCK_PORT &= ~_BV(SCK_PIN); //0

   MOSI_DDR  |= _BV(MOSI_PIN);  //выход - MOSI
   MOSI_PORT &= ~_BV(MOSI_PIN); //0

   MISO_DDR  &= ~_BV(MISO_PIN); //вход  - MISO
   MISO_PORT |= _BV(MISO_PIN);  //1
}

#if defined(CPOL0_CPHA0)


// Заменить на макросы или инлайны?
// СТАРТ последовательность
void _spi_start(void) {
   SCK_LOW; // SPI_MODE = 0
   CS_LOW; // Chip Select - Enable
}
// СТОП последовательность
void _spi_stop(void) {
   CS_HIGH; // Chip Select - Disable
   SCK_LOW; // SPI_MODE = 0
}


/* read and write one byte through SPI*/
uint8_t spi_TxRx(uint8_t data) {
    uint8_t spiReadData = 0;

    for(uint8_t i = 0; i<8; i++) {
      spiReadData <<= 1; // сдвиг для передачи след бита

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
      spiReadData |= MISO_STATE; // читаем бит, !! для перевода состояния бита в булевый тип
      _NOP();
      SCK_LOW; // синхроимпульс
    } //End For

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


