#include <io128.h>
#include <sra128.h>

#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>

//#define MOSI PB2
//#define MISO PB3 
//#define SCLK PB1 
//#define SS PB0

//int16_t convertEncoderBit()
//{
//  return MSBData<<8 | LSBData;
//}

int16_t getEncoderReadings1()
{
  uint8_t x = 0xFF;
  uint8_t LSBData;
  uint8_t MSBData;

  PORTD &= ~ (1<<PD6);
  MSBData = spiTransfer(x);
  LSBData = spiTransfer(x);
  PORTD |= (1<<PD6);
//  return convertEncoderBit();
  return (MSBData << 8 | LSBData);
}

int16_t getEncoderReadings2()
{
  uint8_t x = 0xFF;
  uint8_t LSBData;
  uint8_t MSBData;

  PORTD &= ~ (1<<PD7);
  MSBData = spiTransfer(x);
  LSBData = spiTransfer(x);
  PORTD |= (1<<PD7);
//  return convertEncoderBit();
  return (MSBData << 8 | LSBData);
}

int getEncoderPosition()
{
  int16_t encoderReading1 = getEncoderReadings1();
  int16_t encoderReading2 = getEncoderReadings2();
  return encoderReading1 - encoderReading2;
}

