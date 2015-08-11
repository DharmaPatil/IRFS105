// CRC autoflush = true
// Manchester enable = false
// Modulation format = GFSK
// RX filter BW = 562.500000
// Sync word qualifier mode = 30/32 sync word bits detected
// Data rate = 249.664
// TX power = 0
// Whitening = false
// Preamble count = 4
// Carrier frequency = 2425.748703
// Deviation = 1.647949
// Packet length = 3
// Channel number = 3
// Modulated = true
// Address config = No address check
// Base frequency = 2424.999710
// Packet length mode = Fixed packet length mode. Length configured in PKTLEN register
// Channel spacing = 249.664307
// Data format = Normal mode
// CRC enable = true
// Device address = 0
// PA table


#include <inc/cc2500.h>
#include "inc/ct_assert.h"


const uint8_t preferredSettings[][2]=
{
  {CC2500_IOCFG0,      0x06},
  {CC2500_PKTLEN,      0x03},
  {CC2500_PKTCTRL1,    0x0C},
  {CC2500_PKTCTRL0,    0x04},
  {CC2500_CHANNR,      0x03},
  {CC2500_FSCTRL1,     0x0A},
  {CC2500_FREQ2,       0x59},
  {CC2500_FREQ1,       0xD0},
  {CC2500_FREQ0,       0x97},
  {CC2500_MDMCFG4,     0x2D},
  {CC2500_MDMCFG3,     0x2F},
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
  {CC2500_FSCAL3,      0xEA},
  {CC2500_FSCAL1,      0x00},
  {CC2500_FSCAL0,      0x11},
};

STATIC_ASSERT( sizeof(preferredSettings)/2 == 25 ); // define CC_N_REG

