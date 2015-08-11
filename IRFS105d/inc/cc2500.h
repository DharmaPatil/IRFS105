/********************************************************
* @file cc2500.h
* @author Tejas Dhagawkar,
*         aethylic@gmail.com
* @date  09.08.15
* @brief
********************************************************/

/*
For register addresses in the range 0x30-0x3D, the burst bit is used to select between
status registers, burst bit is one, and command
strobes, burst bit is zero
*/
#pragma once
#ifndef CC2500_H_INCLUDED
#define CC2500_H_INCLUDED

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "inc/soft_spi.h"

#define ADDR(mask, addr) ((mask)|(addr))
#define CC_N_REG   0x2F //размер массива конфигурации
#define RW_F       0x80 //READ! flag
#define BRST_F     0x40 //Burst access to regs


// CC2500 STROBE, CONTROL AND STATUS REGISTER
#define CC2500_IOCFG2       0x00        // GDO2 output pin configuration
#define CC2500_IOCFG1       0x01        // GDO1 output pin configuration
#define CC2500_IOCFG0       0x02        // GDO0 output pin configuration
#define CC2500_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CC2500_SYNC1        0x04        // Sync word, high byte
#define CC2500_SYNC0        0x05        // Sync word, low byte
#define CC2500_PKTLEN       0x06        // Packet length
#define CC2500_PKTCTRL1     0x07        // Packet automation control
#define CC2500_PKTCTRL0     0x08        // Packet automation control
#define CC2500_ADDR         0x09        // Device address
#define CC2500_CHANNR       0x0A        // Channel number
#define CC2500_FSCTRL1      0x0B        // Frequency synthesizer control
#define CC2500_FSCTRL0      0x0C        // Frequency synthesizer control
#define CC2500_FREQ2        0x0D        // Frequency control word, high byte
#define CC2500_FREQ1        0x0E        // Frequency control word, middle byte
#define CC2500_FREQ0        0x0F        // Frequency control word, low byte
#define CC2500_MDMCFG4      0x10        // Modem configuration
#define CC2500_MDMCFG3      0x11        // Modem configuration
#define CC2500_MDMCFG2      0x12        // Modem configuration
#define CC2500_MDMCFG1      0x13        // Modem configuration
#define CC2500_MDMCFG0      0x14        // Modem configuration
#define CC2500_DEVIATN      0x15        // Modem deviation setting
#define CC2500_MCSM2        0x16        // Main Radio Control State Machine configuration
#define CC2500_MCSM1        0x17        // Main Radio Control State Machine configuration
#define CC2500_MCSM0        0x18        // Main Radio Control State Machine configuration
#define CC2500_FOCCFG       0x19        // Frequency Offset Compensation configuration
#define CC2500_BSCFG        0x1A        // Bit Synchronization configuration
#define CC2500_AGCCTRL2     0x1B        // AGC control
#define CC2500_AGCCTRL1     0x1C        // AGC control
#define CC2500_AGCCTRL0     0x1D        // AGC control
#define CC2500_WOREVT1      0x1E        // High byte Event 0 timeout
#define CC2500_WOREVT0      0x1F        // Low byte Event 0 timeout
#define CC2500_WORCTRL      0x20        // Wake On Radio control
#define CC2500_FREND1       0x21        // Front end RX configuration
#define CC2500_FREND0       0x22        // Front end TX configuration
#define CC2500_FSCAL3       0x23        // Frequency synthesizer calibration
#define CC2500_FSCAL2       0x24        // Frequency synthesizer calibration
#define CC2500_FSCAL1       0x25        // Frequency synthesizer calibration
#define CC2500_FSCAL0       0x26        // Frequency synthesizer calibration
#define CC2500_RCCTRL1      0x27        // RC oscillator configuration
#define CC2500_RCCTRL0      0x28        // RC oscillator configuration
#define CC2500_FSTEST       0x29        // Frequency synthesizer calibration control
#define CC2500_PTEST        0x2A        // Production test
#define CC2500_AGCTEST      0x2B        // AGC test
#define CC2500_TEST2        0x2C        // Various test settings
#define CC2500_TEST1        0x2D        // Various test settings
#define CC2500_TEST0        0x2E        // Various test settings


//Status registers, use Burst bit to access
#define CC2500_PARTNUM      0x30
#define CC2500_VERSION      0x31
#define CC2500_FREQEST      0x32 //Frequency offset estimate
#define CC2500_LQI          0x33 //Demodulator estimate for Link Quality
#define CC2500_RSSI         0x34
#define CC2500_MARCSTATE    0x35 //Control state machine state
#define CC2500_WORTIME1     0x36 //High byte of WOR timer
#define CC2500_WORTIME0     0x37 //Low byte of WOR timer
#define CC2500_PKTSTATUS    0x38 //Current GDOx status and packet status
#define CC2500_VCO_VC_DAC   0x39 //Current setting from PLL calibration module
#define CC2500_TXBYTES      0x3A //Underflow and number of bytes in the TX FIFO
#define CC2500_RXBYTES      0x3B //Overflow and number of bytes in the RX FIFO


#define CC2500_PATABLE      0x3E
#define CC2500_TXFIFO       0x3F
#define CC2500_RXFIFO       0x3F

// Strobe commands
#define SRES         0x30        // Reset chip.
#define SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                 // If in RX/TX: Go to a wait state where only the synthesizer is
                                 // running (for quick RX / TX turnaround).
#define SXOFF        0x32        // Turn off crystal oscillator.
#define SCAL         0x33        // Calibrate frequency synthesizer and turn it off
                                 // (enables quick start).
#define SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                 // MCSM0.FS_AUTOCAL=1.
#define STX          0x35        // In IDLE state: Enable TX. Perform calibration first if
                                 // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                 // Only go to TX if channel is clear.
#define SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                 // Wake-On-Radio mode if applicable.
#define SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define SPWD         0x39        // Enter power down mode when CSn goes high.
#define SFRX         0x3A        // Flush the RX FIFO buffer.
#define SFTX         0x3B        // Flush the TX FIFO buffer.
#define SWORRST      0x3C        // Reset real time clock.
#define SNOP         0x3D        // No operation. May be used to pad strobe commands to two
                                 // bytes for simpler software.


//int main()
//{
//unsigned int j;
//uint8_t i,b;
//_delay_ms(5);
//SPI_init();
//PORTC|=(1<<CSn);
//for(i=0x00;i<0x2F;i++)   // configure registers of CC2500
//{
//PORTC=(0<<CSn);
//while(bit_is_set(PINB,PB4));
//SPI_TX(i); //address byte
//SPI_TX(CC_rfSettings[i]);// data byte
//PORTC|=(1<<CSn);
//}
//while(1)
//{
////use send()in transmitter and receive() in receiver
//send();
////receive();
//}
//}

void cc2500_reset(void);
void InitCC2500(void);
void command(uint8_t a); // give commands to CC
void send();
void receive();


#endif // CC2500_H_INCLUDED
