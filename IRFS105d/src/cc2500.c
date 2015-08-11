/********************************************************
* @file cc2500.c
* @author Tejas Dhagawkar,
*         aethylic@gmail.com
* @date  09.08.15
* @brief
********************************************************/

#include "inc/cc2500.h"
//#include <avr/cpufunc.h>
//#include "inc/cc2500_config.h"
#define PA_TABLE {0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,}

extern const uint8_t preferredSettings[22][2];
static const uint8_t pa_table[8] = PA_TABLE;


//creating array for assigned register values
 /*const uint8_t CC_rfSettings[CC_N_REG] = {
CC_IOCFG2_value,
CC_IOCFG1_value,
CC_IOCFG0D_value,
CC_FIFOTHR_value,
CC_SYNC1_value,
CC_SYNC0_value,
CC_PKTLEN_value,
CC_PKTCTRL1_value,
CC_PKTCTRL0_value,
CC_ADDR_value,
CC_CHANNR_value,
CC_FSCTRL1_value,
CC_FSCTRL0_value,
CC_FREQ2_value,
CC_FREQ1_value,
CC_FREQ0_value,
CC_MDMCFG4_value,
CC_MDMCFG3_value,
CC_MDMCFG2_value,
CC_MDMCFG1_value,
CC_MDMCFG0_value,
CC_DEVIATN_value,
CC_MCSM2_value,
CC_MCSM1_value,
CC_MCSM0_value,
CC_FOCCFG_value,
CC_BSCFG_value,
CC_AGCCTRL2_value,
CC_AGCCTRL1_value,
CC_AGCCTRL0_value,
CC_WOREVT1_value,
CC_WOREVT0_value,
CC_WORCTRL_value,
CC_FREND1_value,
CC_FREND0_value,
CC_FSCAL3_value,
CC_FSCAL2_value,
CC_FSCAL1_value,
CC_FSCAL0_value,
CC_RCCTRL1_value,
CC_RCCTRL0_value,
CC_FSTEST_value,
CC_PTEST_value,
CC_AGCTEST_value,
CC_TEST2_value,
CC_TEST1_value,
CC_TEST0_value
};*/

uint8_t p,q,r,t; //deleete!


void InitCC2500(void) {
  //ћожет потом добавить возврат статуса?
  cc2500_reset();
  _spi_stop();
  _delay_ms(5);
  for(uint8_t i=0; i<CC_N_REG; i++) {
    //
    _spi_start();
    while(MISO_STATE); //wait
    spi_TxRx(preferredSettings[i][0]);  //address byte, Write, BRST=0
    spi_TxRx(preferredSettings[i][1]);  //data byte
    //spi_TxRx(i);                //address byte  акой пор€док записи регистров?
    //spi_TxRx(CC_rfSettings[i]); // data byte
    _spi_start();
  }
  _delay_ms(5);

  spi_TxRx(ADDR(BRST_F, CC2500_PATABLE)); // PA_TABLE init
  for (uint8_t i=0; i<8; i++) {
    spi_TxRx(pa_table[i]);
  }
  _delay_ms(5);

  command(SIDLE);
  command(SFRX);
  command(SFTX);

} //void cc2500_init(void)

/* sotware reset from datasheet */
void cc2500_reset(void) {
  SCK_HIGH;
  MOSI_LOW;
  CS_LOW;
  _NOP();
  CS_HIGH;
  _delay_us(40);
  CS_LOW;
  while(MISO_STATE);
  command(SRES);
}

void command(uint8_t d) { // give commands to CC
  _spi_start();
  while(MISO_STATE); //wait for stability
  spi_TxRx(d);
  _spi_stop();
}

//receive data wirelessly with CC
void receive() {
command(SRX); // command to receive data wirelessly
command(SRX);

//while(bit_is_clear(PINC,PC1)); // check GD0pin of CC2500 нужно читать регистр PKTSTATUS

_spi_start();
while(MISO_STATE);
spi_TxRx(0xFF); // rx FIFO address burst mode
p=spi_TxRx(0x00); // data byte1
q=spi_TxRx(0x00);// data byte2
r=spi_TxRx(0x00);// data byte3
_spi_stop();
command(SFRX); // flush receiver FIFO
command(SIDLE); // turn CC2500 into idle mode
command(SCAL);
PORTD=r;
}

void send() {  // send data in CC wirelessly
command(SIDLE);    //turn CC2500 into idle mode
command(SFTX);      //flush tx FIFO

//command(SCAL);
_spi_start();
while(MISO_STATE); //MISO
spi_TxRx(0x7F);    // tx FIFO address in burst mode
spi_TxRx(0x55); // data byte1
spi_TxRx(0xAA); // data byte2
spi_TxRx(0xBB); //data byte3
//_spi_stop();
command(STX);  //command to send data in tx FIFO wirelessly
while (!MARXSTATE_TX_END_STATE) {;} //wait until TX END
//_delay_us(10);
}
