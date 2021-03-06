// CRC autoflush = false
// Manchester enable = false
// Modulation format = MSK
// RX filter BW = 337.497401
// Sync word qualifier mode = 30/32 sync word bits detected
// Data rate = 99.9062
// TX power = 0
// Whitening = false
// Preamble count = 4
// Carrier frequency = 2463.499906
// Packet length = 7
// Phase transition time = 0
// Channel number = 0
// Modulated = true
// Address config = No address check
// Base frequency = 2463.499906
// Packet length mode = Fixed packet length mode. Length configured in PKTLEN register
// Channel spacing = 199.812304
// Data format = Normal mode
// CRC enable = false
// Device address = 0
// PA table


#include <inc/cc2500.h>
#include "inc/ct_assert.h"

/*    freq2 freq1 freq0
b4712       CF    B0
b4714  5B   3�    85  //CF    89
b4715       CF    71
B4700       D0    A7

D4713       D0    B8
D4702       D0    7D
D4714       CF    A2
D4715  5B   3E    0D
D4716  5B   3D    CF

*/

const uint8_t preferredSettings[][2]=
{
  {CC2500_FSCTRL1,      0x06},
  {CC2500_FSCTRL0,      0x00},
  {CC2500_FREQ2,        0x5B}, //
  {CC2500_FREQ1,        0x4F}, //b4714
  {CC2500_FREQ0,        0x7C}, //7C
  {CC2500_MDMCFG4,      0x0B},
  {CC2500_MDMCFG3,      0xE5},
  {CC2500_MDMCFG2,      0x73},
  {CC2500_MDMCFG1,      0xA2},
  {CC2500_MDMCFG0,      0xE5},
  {CC2500_CHANNR,       0x00},
  {CC2500_DEVIATN,      0x00},
  {CC2500_FREND1,       0x56},
  {CC2500_FREND0,       0x10},
  {CC2500_MCSM2,        0x07},
  {CC2500_MCSM1,        0x30},
  {CC2500_MCSM0,        0x18},
  {CC2500_FOCCFG,       0x16},
  {CC2500_BSCFG,        0x6C},
  {CC2500_AGCCTRL2,     0x43},
  {CC2500_AGCCTRL1,     0x40},
  {CC2500_AGCCTRL0,     0x91},
  {CC2500_FSCAL3,       0xA9},
  {CC2500_FSCAL2,       0x0A},
  {CC2500_FSCAL1,       0x00},
  {CC2500_FSCAL0,       0x11},
  {CC2500_TEST2,        0x88},
  {CC2500_TEST1,        0x31},
  {CC2500_TEST0,        0x0B},
  {CC2500_IOCFG2,       0x2E},
  {CC2500_IOCFG1,       0x06},
  {CC2500_IOCFG0,       0x30},
  {CC2500_PKTCTRL1,     0x80},
  {CC2500_PKTCTRL0,     0x08},
  {CC2500_ADDR,         0x00},
  {CC2500_PKTLEN,       0x07},
  {CC2500_FIFOTHR,      0x15},
};

STATIC_ASSERT( sizeof(preferredSettings)/2 == 37 ); // define CC_N_REG

