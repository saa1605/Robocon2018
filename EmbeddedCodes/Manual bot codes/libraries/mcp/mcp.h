
#ifndef MCP_h
#define MCP_h

/* I N C L U D E S */
#include <sra128.h>
#include <avr/io.h>
#include <arduino.h>
#include <stdlib.h>
#include <compat/deprecated.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <io128.h>
#include <avr/interrupt.h>
#include <inttypes.h>

/* E N D S */

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
int SlaveSelected = 8;
int SPI_lockup=0;
// SPI_timeOutDelay
uint16_t SPI_timeOutDelay = 3;
unsigned char spi_transceiver(unsigned char data)
{
  // Load data into the buffer
  //Initially drive SS pin high
  SPDR = data;

  //SerialWriteString("while started");
  //Wait until transmission complete
  unsigned long SPI_startingTime = millis();
  
  while(!(SPSR & (1<<SPIF) ))
  {
    if((millis() - SPI_startingTime) >= SPI_timeOutDelay)
    {
      SPI_lockup = 1;
      break;
      // lockUp();
      // LED0 = 0;
      // LED1 = 1;
      // _delay_ms(1000);
      // return(1);
    }
  }
  //SerialWriteString("while over");
  // Return received data

  return(SPDR);
}

int spi_dataMCP3208(int csPin, int channel)
{
  //spi_init(csPin);
  //sbi(PORTD,csPin); // SS pin set to ensure a clean high to low transition to SSpin
  // if(csPin != SlaveSelected)
  //     spi_init(csPin);
  unsigned char byte2Rx, byte3Rx;
  unsigned char byte1Tx, byte2Tx, byte3Tx;
  unsigned char D2 = 0, D1 = 0, D0 = 0;
  D2 = channel & (1<<2);
  D1 = channel & (1<<1);
  D0 = channel & (1<<0);

  //For Byte 1 Tx
  //’0' ’0' ’0' ’0' ’0' ’1' ‘S’ ‘D2'
  //S = 1 for differential
  byte1Tx = 0b00000110;
  byte1Tx |= (D2>>2);

  //For Byte 2 Tx
  //‘D1' ‘D0' ‘X’ ‘X’ ‘X’ ‘X’ ‘X ‘X’
  byte2Tx = 0b00000000;
  byte2Tx |= D1<<6;
  byte2Tx |= D0<<6;

  //For Byte 3 Tx
  byte3Tx = 0;

  cbi(PORTF,csPin); //Pull SS line LOW;
  SPI_lockup = 0;
  byte2Rx = spi_transceiver(byte1Tx);
  byte2Rx = spi_transceiver(byte2Tx);
  byte3Rx = spi_transceiver(byte3Tx);
  sbi(PORTF, csPin); //Pull SS Line Left HIGH
  //For byte2 Rx
  //‘?’ ‘?’ ‘?’ ‘N’ ‘B11' ‘B10' ‘B9' ‘B8'
  //For byte3 Rx
  //B7 to B0 of conversion.
  if(SPI_lockup == 1)
    return 0;
  else
    return((int)(byte3Rx + (byte2Rx & 0x0F)*256));
}


void spi_init(int csPin)
{
  //sei();
  //SREG  = 1<<7;	//ENABLING THE I-BIT IN THE SREG
  /* //For 128
   	DDRB &= 0b11110111; 		//Set MISO as INPUT
   	DDRB |= 0b00000111;	//DDRB |= ((1<<MOSI) | (1<<SS) | (1<< SCK));
   	PORTB |= (1<<0); //PORTB |= (1<<SS); //4 for 16 // 0 for 128
   	*/
  //For 16
  
    // SlaveSelected = csPin;
    // sbi(PORTB, 0); //Pull SS pin on atmega high. Might not be used as CS(chip select) for any slave, but should be pulled high!!!

    // cbi(DDRB, 3); //Set MISO as INPUT
    // sbi(DDRB, 2); //MOSI OUTPUT
    // sbi(DDRB, 1); //SCK OUTPUT
    if(csPin != SlaveSelected)
    {
      sbi(DDRF, csPin); //CS as OUTPUT
      sbi(PORTF, csPin); //Drive CS HIGH
      SlaveSelected = csPin;
    }
    /*
  		  7	  6	  5		4	3	  2	  1		0
     		SPIE SPE DORD MSTR CPOL CPHA SPR1 SPR0 - SPCR
     		 0	  1	  0	  	1 	0     0	  0		1
     		SPIE - spi interrupt enable
     		SPE - spi enable
     		DORD - data order 1 - LSB first,  0 - MSB first
     		MSTR - master/slave select 1 - master mode, 0 - slave mode
     			// If SS is configured as an input and is driven low while MSTR is set, MSTR will be cleared,and SPIF in SPSR will become set.
     			The user will then have to set MSTR to re-enable SPI Master mode.
     		CPOL - clock polarity 1 - idle high, 0 - idle low
     		CPHA -  clock phase 1 - sample on trailing edge, 0 - sample on leading edge
     		SPR1, SPR0: SPI Clock Rate Select 1 and 0 - with SPI2X bit 
     		SPI2X SPR1 SPR0 SCK Frequency
     			0 0 0 f osc / 4
     			0 0 1 f osc / 16
     			0 1 0 f osc / 64
     			0 1 1 f osc / 128
     			1 0 0 f osc / 2
     			1 0 1 f osc / 8
     			1 1 0 f osc / 32
     			1 1 1 f osc / 64
     	*/
    // SPCR = 0b01010001;
    /*
  		  7	  6 		  5 4 3 2 1		  0
     		SPIF WCOL      – – – – – 		SPI2X  -  SPSR
     		  x x							  0
     	*/
    // SPSR &= 0b11111110;
}

void spi_communication_init()
{
//sei();
//SREG  = 1<<7; //ENABLING THE I-BIT IN THE SREG
/* //For 128
  DDRB &= 0b11110111;     //Set MISO as INPUT
  DDRB |= 0b00000111; //DDRB |= ((1<<MOSI) | (1<<SS) | (1<< SCK));
  PORTB |= (1<<0); //PORTB |= (1<<SS); //4 for 16 // 0 for 128
  */
//For 16

  // SlaveSelected = csPin;
  sbi(PORTB, 0); //Pull SS pin on atmega high. Might not be used as CS(chip select) for any slave, but should be pulled high!!!

  cbi(DDRB, 3); //Set MISO as INPUT
  sbi(DDRB, 2); //MOSI OUTPUT
  sbi(DDRB, 1); //SCK OUTPUT
  // if(csPin != SlaveSelected)
  // {
  //   sbi(DDRF, csPin); //CS as OUTPUT
  //   sbi(PORTF, csPin); //Drive CS HIGH
  //   SlaveSelected = csPin;
  // }
  /*
      7   6   5   4 3   2   1   0
      SPIE SPE DORD MSTR CPOL CPHA SPR1 SPR0 - SPCR
       0    1   0     1   0     0   0   1
      SPIE - spi interrupt enable
      SPE - spi enable
      DORD - data order 1 - LSB first,  0 - MSB first
      MSTR - master/slave select 1 - master mode, 0 - slave mode
        // If SS is configured as an input and is driven low while MSTR is set, MSTR will be cleared,and SPIF in SPSR will become set.
        The user will then have to set MSTR to re-enable SPI Master mode.
      CPOL - clock polarity 1 - idle high, 0 - idle low
      CPHA -  clock phase 1 - sample on trailing edge, 0 - sample on leading edge
      SPR1, SPR0: SPI Clock Rate Select 1 and 0 - with SPI2X bit 
      SPI2X SPR1 SPR0 SCK Frequency
        0 0 0 f osc / 4
        0 0 1 f osc / 16
        0 1 0 f osc / 64
        0 1 1 f osc / 128
        1 0 0 f osc / 2
        1 0 1 f osc / 8
        1 1 0 f osc / 32
        1 1 1 f osc / 64
    */
  SPCR = 0b01010001;
  /*
      7   6       5 4 3 2 1     0
      SPIF WCOL      – – – – –    SPI2X  -  SPSR
        x x               0
    */
  SPSR &= 0b11111110;
}
#endif