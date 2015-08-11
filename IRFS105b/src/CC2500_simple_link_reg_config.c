// Packet length = 255
// Base frequency = 2424.999710
// Modulation format = GFSK
// Device address = 0
// Whitening = false
// TX power = 0
// Data rate = 24.9767
// RX filter BW = 562.500000
// Carrier frequency = 2425.748703
// CRC enable = true
// Preamble count = 4
// CRC autoflush = false
// Data format = Normal mode
// Deviation = 1.647949
// Modulated = true
// Channel number = 3
// Address config = No address check
// Manchester enable = false
// Channel spacing = 249.664307
// Sync word qualifier mode = 30/32 sync word bits detected
// Packet length mode = Variable packet length mode. Packet length configured by the first byte after sync word

// PA table
#include <inc/cc2500.h>

const uint8_t preferredSettings[22][2]=
{
  {CC2500_IOCFG0,      0x06},
  {CC2500_PKTCTRL0,    0x05},
  {CC2500_CHANNR,      0x03},
  {CC2500_FSCTRL1,     0x0A},
  {CC2500_FREQ2,       0x59},
  {CC2500_FREQ1,       0xD0},
  {CC2500_FREQ0,       0x97},
  {CC2500_MDMCFG4,     0x29},
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
