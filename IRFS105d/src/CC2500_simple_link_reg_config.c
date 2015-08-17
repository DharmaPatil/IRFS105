// CRC autoflush = false
// Manchester enable = false
// Modulation format = GFSK
// RX filter BW = 843.743503
// Sync word qualifier mode = 30/32 sync word bits detected
// Data rate = 49.9531
// TX power = 0
// Whitening = false
// Preamble count = 4
// Carrier frequency = 2425.748976
// Deviation = 1.647937
// Packet length = 3
// Channel number = 3
// Modulated = true
// Address config = No address check
// Base frequency = 2424.999989
// Packet length mode = Variable packet length mode. Packet length configured by the first byte after sync word
// Channel spacing = 249.662384
// Data format = Normal mode
// CRC enable = false
// Device address = 0
// PA table


#include <inc/cc2500.h>
#include "inc/ct_assert.h"

/*    freq1 freq0
b4712 CF    B0
b4714 CF    89
b4715 CF    71
B4700 D0    A7

D4713 D0    B8
D4702 D0    7D
D4714 CF    A2
D4716 D0    C5

*/

const uint8_t preferredSettings[][2]=
{
  {CC2500_IOCFG0,      0x06},
  {CC2500_PKTLEN,      0x03},
  {CC2500_PKTCTRL0,    0x01},
  {CC2500_CHANNR,      0x03},
  {CC2500_FSCTRL1,     0x0A},
  {CC2500_FREQ2,       0x59},
  {CC2500_FREQ1,       0xD0},
  {CC2500_FREQ0,       0xC5},
  {CC2500_MDMCFG4,     0x0A},
  {CC2500_MDMCFG3,     0xE5},
  {CC2500_MDMCFG2,     0x13},
  {CC2500_MDMCFG1,     0x23},
  {CC2500_MDMCFG0,     0x2F},
  {CC2500_DEVIATN,     0x00},
  {CC2500_MCSM0,       0x18},
  {CC2500_FOCCFG,      0x1D},
  {CC2500_BSCFG,       0x1C},
  {CC2500_AGCCTRL2,    0xC7},
  {CC2500_AGCCTRL1,    0x00},
  {CC2500_AGCCTRL0,    0xB0},
  {CC2500_FREND1,      0xB6},
  {CC2500_FSCAL1,      0x00},
  {CC2500_FSCAL0,      0x11},
};

STATIC_ASSERT( sizeof(preferredSettings)/2 == 23 ); // define CC_N_REG

