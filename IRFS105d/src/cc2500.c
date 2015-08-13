/********************************************************
* @file cc2500.c
* @author Tejas Dhagawkar,
*         aethylic@gmail.com
* @date  09.08.15
* @brief
********************************************************/

#include "inc/cc2500.h"
#include "inc/ct_assert.h"

#define PA_TABLE {0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,}

extern const uint8_t preferredSettings[][2]; //можно считывать с флешки или еепром
static const uint8_t pa_table[8] = PA_TABLE;

uint8_t p,q,r,t; //deleete!


void InitCC2500(void) {
  //ћожет потом добавить возврат статуса?
  cc2500_reset();
  _spi_stop();
  _delay_ms(5);

/*for (uint16_t i = 0; i<100; i++) { //test
  cc2500_get_status(CC2500_PARTNUM);
  cc2500_get_status(CC2500_VERSION);
  _delay_ms(10);
}*/

  if (cc2500_get_status(CC2500_PARTNUM) == 0x80) {
    //write config
    for(uint8_t i=0; i<CC_N_REG; i++) {
      //
      _delay_ms(5);
      _spi_start();
      while(MISO_STATE); //wait
      spi_TxRx(preferredSettings[i][0]);  //address byte, Write, BRST=0
      spi_TxRx(preferredSettings[i][1]);  //data byte
      _spi_stop();
    }
    _delay_ms(5);


    _spi_start();
    while(MISO_STATE);
    spi_TxRx(ADDR(BRST_F, CC2500_PATABLE)); // PA_TABLE init
    for (uint8_t i=0; i<8; i++) {
      spi_TxRx(pa_table[i]);
    }
    _delay_ms(1);
    _spi_stop();

    command(SIDLE);
    command(SFRX);
    command(SFTX);
    _spi_stop();
  } //check PARTNUM register
} // END void cc2500_init(void)

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
  command(SFRX); // command to flush RX FIFO
  //_delay_ms(1);
  //command(SCAL);
  _delay_ms(1);
  command(SRX);  // command to receive data wirelessly

  //while(bit_is_clear(PINC,PC1)); // check GD0pin of CC2500 нужно читать регистр PKTSTATUS

  _spi_start();
  while(MISO_STATE){;}

  //block until Carrier Sense detected
  while ( !(cc2500_get_status(CC2500_PKTSTATUS) & PKTSTATUS_CS) ) {
      ;
  }
  //while ( !PKTSTATUS_SFD ) {;} //delay until sync word found
  //block until RX end
  while ( !(cc2500_get_status(CC2500_MARCSTATE) & MARXSTATE_RX_END_STATE) ) {;}

  //exit if RXFIFO empty(with CRC AUTO FLUSH)
  if ( cc2500_get_status(CC2500_RXBYTES) == 0 ) {
    return;
  }

  _spi_start();
  spi_TxRx(0xFF); // rx FIFO address burst mode
  p=spi_TxRx(0xFF); // data byte1
  q=spi_TxRx(0xFF);// data byte2
  r=spi_TxRx(0xFF);// data byte3. –екомендуетс€ считать фифо 2 раза, непон€тно почему
  _spi_stop();
  command(SFRX); // flush receiver FIFO if owerflow state
  command(SIDLE); // turn CC2500 into idle mode
  command(SFRX); // flush receiver FIFO in IDDLE mode
  //command(SCAL);

  //test LED ON if packet received and data correct
  if (p == 0x55) {
    PORTC |= _BV(PC3);
    _delay_ms(200);
  }
  if (q == 0xAA) {
    PORTC |= _BV(PC4);
    _delay_ms(200);
  }
  if (r == 0xBB) {
    PORTC |= _BV(PC5);
    _delay_ms(200);
  }
  PORTC &= ~(_BV(PC3) | _BV(PC4) | _BV(PC5) );//LED OFF

}

void send() {  // send data in CC wirelessly
  command(SIDLE);    //turn CC2500 into idle mode
  command(SFTX);      //flush tx FIFO
  //command(SCAL);

  _spi_start();
  while(MISO_STATE); //MISO
  spi_TxRx(ADDR(BRST_F, CC2500_TXFIFO));    // tx FIFO address in burst mode
  spi_TxRx(0x55); // data byte1
  spi_TxRx(0xAA); // data byte2
  spi_TxRx(0xBB); // data byte3
  _spi_stop();

  command(STX);  //command to send data in tx FIFO wirelessly
  while (!MARXSTATE_TX_END_STATE) {;} //wait until TX END
  //_delay_us(10);
  command(SFTX);
  command(SIDLE);    //turn CC2500 into idle mode
  command(SFTX);      //flush tx FIFO
}

uint8_t cc2500_get_status(uint8_t address) {
  _spi_start();
  while(MISO_STATE);
  spi_TxRx(ADDR(BRST_F|RW_F, address));
  uint8_t status = spi_TxRx(0xFF);
  _spi_stop();
  return status;
}

