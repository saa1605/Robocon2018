#include <io128.h>
#include <sra128.h>

#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>

#define MOSI PB2
#define MISO PB3 
#define SCLK PB1 
#define SS PB0 

uint8_t x = 0xFF;
uint8_t LSBData;
uint8_t MSBData;
int16_t adcValue;
int sensorReadings[8];

float weightages[8];

bool isSmooth = false;

int sum,weightedSum;

void spiMasterInit(void)
{
  DDRB |= (1 << MOSI) | (1 << SCLK) | (1 << SS);
  PORTB |= (1 << SS);
  DDRF |= (1 << PF2); //MCP CS
  SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0)| (1 << SPR1);
}

unsigned char spiTransfer(unsigned char data)
{
  SPDR = data;
 
  while(!(SPSR & (1 << SPIF)));
  return(SPDR); 
}

void convertBit(void)
{
  MSBData = MSBData & 0b00001111;
  adcValue = (MSBData << 8) | LSBData;
}

void readAdc(int channel)
{
  byte channelbit = channel<<6;
  if(channel < 4)
  {
    spiTransfer(0b00000110);
  }
  else
  {
   spiTransfer(0b00000111);
  }
  MSBData = spiTransfer(channelbit);
  LSBData = spiTransfer(x);
  convertBit();  
}

void getSensorReadings()
{ 
  for(int i = 0; i < 8; i++)
  {
    PORTF &= ~(1 << PF2);
    readAdc(i);
    sensorReadings[i] = adcValue;
    PORTF |= (1 << PF2);
  }
}

void assignWeightages(float w0, float w1, float w2, float w3, float w4, float w5, float w6, float w7)
{
  weightages[0] = w0;
  weightages[1] = w1;
  weightages[2] = w2;
  weightages[3] = w3;
  weightages[4] = w4;
  weightages[5] = w5;
  weightages[6] = w6;
  weightages[7] = w7;
}

void decideWeightages()
{
  if(!isSmooth)
  {
    assignWeightages(-7, -5, -3, -1, 1, 3, 5, 7);  
  }  
  else
  {
    assignWeightages(-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5);
  }
}

void multiplyWeightagesToReadings()
{
  for(int i = 0; i < 8; i++)
  {
    sensorReadings[i] *= weightages[i];
  } 
}

float getLinePosition()
{ 
  getSensorReadings();
  for(int i = 0; i < 8; i++)
  {
    sum += sensorReadings[i];     
  }
  multiplyWeightagesToReadings();
  for(int i = 0; i < 8; i++)
  {
    weightedSum += sensorReadings[i];     
  }

  return weightedSum / sum;    
}
