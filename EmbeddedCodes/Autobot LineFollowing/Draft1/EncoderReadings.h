#include <io128.h>
#include <sra128.h>

#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>

#ifndef
#define MOSI PB2
#define MISO PB3 
#define SCLK PB1 
#define SS PB0 
#endif

void convertEncoderBit(void)
{
  encoderData = MSBData<<8 | LSBData;
}

int16_t getEncoderReadings()
{
  uint8_t x = 0xFF;
  
  uint8_t LSBData;
  uint8_t MSBData;
  int16_t encoderData;

//  spiMasterInit();    TO BE CALLED IN SETUP
  PORTD &= ~ (1<<PD7);
  MSBData=spiTransfer(x);
  LSBData=spiTransfer(x);
  PORTD |= (1<<PD3);
  convertEncoderBit();
  return encoderData
}


