/*
* soft_spi.h
* aethylic@gmail.com
* date
*/

#ifndef __SOFT_SPI_H
#define __SOFT_SPI_H

#include <avr/io.h>

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)

#define CPOL0_CPHA0
//#define CPOL0_CPHA1
//#define CPOL1_CPHA0
//#define CPOL1_CPHA1

/* Includes *******************************************************************/
//#include "gpio.h"
#include <stdint.h>

/*GPIO defines ****************************************************************/
// назначение выводов порта
#define SCK_PORT   PORTB  // выход - SCLK
#define SCK_PIN    PB1
#define MOSI_PORT  PORTB  // выход - MOSI
#define MOSI_PIN   PB2
#define MISO_PORT  PINB   // вход  - MISO
#define MISO_PIN   PB0
#define CS_PORT    PORTD  // выход - Chip Select
#define CS_PIN     PD5

#define SCK_LOW (SCK_PORT &= (~(1<<SCK_PIN))) //не использую _BV(), не помню глубину вложенности макросов
#define SCK_HIGH (SCK_PORT |= (1<<SCK_PIN))

#define MOSI_LOW (MOSI_PORT &= (~(1<<MOSI_PIN)))
#define MOSI_HIGH (MOSI_PORT |= (1<<MOSI_PIN))

#define CS_LOW (CS_PORT &= (~(1<<CS_PIN)))
#define CS_HIGH (CS_PORT |= (1<<CS_PIN))

//#define MISO_STATE (bit_is_set(MISO_PORT, MISO_PIN))
#define MISO_STATE (MISO_PORT & _BV(MISO_PIN))

/*#define MOSI_HIGH (GPIOB->BSRR = (uint32_t)GPIO_PIN_15)
#define MOSI_LOW (GPIOB->BSRR = (uint32_t)GPIO_PIN_15 << 16)

#define SCK_HIGH (GPIOB->BSRR = (uint32_t)GPIO_PIN_13)
#define SCK_LOW (GPIOB->BSRR = (uint32_t)GPIO_PIN_13 << 16)

#define MISO_STATE (GPIOB->IDR & GPIO_PIN_14)*/


void InitSPI_soft(void);
void _spi_start(void);
void _spi_stop(void);
//void spi_send_byte(uint8_t data);
//uint8_t spi_read_byte(void);
uint8_t spi_TxRx(uint8_t data);

#endif /* __SOFT_SPI_H */

