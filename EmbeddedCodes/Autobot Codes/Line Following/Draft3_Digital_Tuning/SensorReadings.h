#include <io128.h>
#include <sra128.h>

#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>

#define MOSI PB2
#define MISO PB3 
#define SCLK PB1 
#define SS PB0 

#define frontSlaveSelect PF2
#define backSlaveSelect PF3

#define sideSensorThreshold 500
#define mainSensorThreshold 1950   //1600 is max of red
									                 //2300 is max of white

int leftFirst = 0, leftSecond = 0, leftThird = 0, leftFourth = 0;
int rightFirst = 0, rightSecond = 0, rightThird = 0, rightFourth = 0;

// uint8_t x = 0xFF;
// uint8_t LSBData;
// uint8_t MSBData;
// int16_t adcValue;
int sensorReadingsFront[8];
int sensorReadingsBack[8];

int weightedSumFront = 0, weightedSumBack = 0; ///this must be global to be used in Draft2.ino

float weightages[8];

bool isSmooth = false; //this is global since used in main program

// int sumFront, sumBack;

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

// void convertBit(void)
// {
//   MSBData = MSBData & 0b00001111;
//   adcValue = (MSBData << 8) | LSBData;
// }

uint16_t readAdc(int channel)
{
  uint8_t x = 0xFF;
  uint8_t LSBData;
  uint8_t MSBData;

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
  // convertBit();  
  MSBData = MSBData & 0b00001111;
  return (MSBData << 8 | LSBData);
}

void getSensorReadings()
{ 
  for(int i = 0; i < 8; i++)
  {
    PORTF &= ~(1 << frontSlaveSelect);
    sensorReadingsFront[i] = readAdc(i);
    PORTF |= (1 << frontSlaveSelect);
  }
  
  for(int i = 0; i < 8; i++)
  {
    PORTF &= ~(1 << backSlaveSelect);
    sensorReadingsBack[i] = readAdc(i);
    PORTF |= (1 << backSlaveSelect);
  }

  leftFirst = adc_start(0);
  leftSecond = adc_start(1);
  leftThird = adc_start(2);
  leftFourth = adc_start(3);
  rightFirst = adc_start(4);
  rightSecond = adc_start(5);
  rightThird = adc_start(6);
  rightFourth = adc_start(7);
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
//  leftFirst *= 1;
//  leftSecond *= -1;
//  rightFirst *= 1;
//  rightSecond *= -1;
}

void selectWeightages()
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

void digitaliseReadings()
{
	leftFirst = 0;
	leftSecond = 0;
	leftThird = 0;
	leftFourth = 0;
	rightFirst = 0;
	rightSecond = 0;
	rightThird = 0;
	rightFourth = 0;

	for(int i =0; i < 8; i++)
	{
		if(sensorReadingsFront[i] < mainSensorThreshold)
		{
			sensorReadingsFront[i] = 0;
		}
		else
		{
			sensorReadingsFront[i] = 1;
		}

		if(sensorReadingsBack[i] < mainSensorThreshold)
		{
			sensorReadingsBack[i] = 0;
		}
		else
		{
			sensorReadingsBack[i] = 1;
		}

		if(adc_start(i) > sideSensorThreshold)
		{
			switch(i)
			{
				case 0: leftFirst = 1; break;
				case 1: leftSecond = 1; break;
				case 2: leftThird = 1; break;
				case 3: leftFourth = 1; break;
				case 4: rightFirst = 1; break;
				case 5: rightSecond = 1; break;
				case 6: rightThird = 1; break;
				case 7: rightFourth = 1; break;
				default:break;
			}
		}
	}
}

void multiplyWeightagesToReadings()
{
  selectWeightages();
  for(int i = 0; i < 8; i++)
  {
    sensorReadingsFront[i] *= weightages[i];
    sensorReadingsBack[i] *= weightages[i];
  } 
}

float getLinePosition()
{ 
  int sumFront, sumBack;
  getSensorReadings();
  digitaliseReadings();
  for(int i = 0; i < 8; i++)
  {
    sumFront += sensorReadingsFront[i];
    sumBack += sensorReadingsBack[i];
  }
  multiplyWeightagesToReadings();
  for(int i = 0; i < 8; i++)
  {
    weightedSumFront += sensorReadingsFront[i];
    weightedSumBack += sensorReadingsBack[i];
  }

  return (weightedSumFront / sumFront)*0.5 + (weightedSumBack / sumBack)*(-0.5);  //Sensors symmetric to center and opposite errors
}
