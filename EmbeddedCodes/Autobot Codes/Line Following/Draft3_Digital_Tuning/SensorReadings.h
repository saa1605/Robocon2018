//#define frontSlaveSelect PF2
//#define backSlaveSelect PF3

#define sideSensorThreshold 500
#define mainSensorThreshold 1400   

int leftFirst = 0, leftSecond = 0, leftThird = 0, leftFourth = 0;
int rightFirst = 0, rightSecond = 0, rightThird = 0, rightFourth = 0;

int sensorReadingsFront[8];
int sensorReadingsBack[8];

int frontError = 0, backError = 0;
// int sumFront, sumBack;
//int weightedSumFront = 0, weightedSumBack = 0;

int lineRotationalPos = 0, lineTranslationalPos = 0;

float weightages[8];

bool isSmooth = false;		//this is global since used in main program

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
    PORTF &= ~(1 << frontSlaveSelect);
    readAdc(i);
    sensorReadingsFront[i] = adcValue;
    PORTF |= (1 << frontSlaveSelect);
  }
  
  for(int i = 0; i < 8; i++)
  {
    PORTF &= ~(1 << backSlaveSelect);
    readAdc(i);
    sensorReadingsBack[i] = adcValue;
    PORTF |= (1 << backSlaveSelect);
  }

//  leftFirst = adc_start(0);
//  leftSecond = adc_start(1);
//  leftThird = adc_start(2);
//  leftFourth = adc_start(3);
//  rightFirst = adc_start(4);
//  rightSecond = adc_start(5);
//  rightThird = adc_start(6);
//  rightFourth = adc_start(7);
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
//	leftFirst = 0;
//	leftSecond = 0;
//	leftThird = 0;
//	leftFourth = 0;
//	rightFirst = 0;
//	rightSecond = 0;
//	rightThird = 0;
//	rightFourth = 0;

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

//		if(adc_start(i) > sideSensorThreshold)
//		{
//			switch(i)
//			{
//				case 0: leftFirst = 1; break;
//				case 1: leftSecond = 1; break;
//				case 2: leftThird = 1; break;
//				case 3: leftFourth = 1; break;
//				case 4: rightFirst = 1; break;
//				case 5: rightSecond = 1; break;
//				case 6: rightThird = 1; break;
//				case 7: rightFourth = 1; break;
//				default:break;
//			}
//		}   
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

void getLinePosition()
{
  int sumFront = 0, sumBack = 0;
  int weightedSumFront = 0, weightedSumBack = 0;
  
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

  if(sumFront == 0 && sumBack == 0);

  else if(sumFront == 0)
  {
    backError = (weightedSumBack / sumBack) * 0.5;
  }
  else if(sumBack == 0)
  {
    frontError = (weightedSumFront / sumFront) * 0.5;
  }
  else 
  {
    frontError = (weightedSumFront / sumFront) * 0.5;
    backError = (weightedSumBack / sumBack) * 0.5;
  }
  
  lineRotationalPos = frontError + backError;
  lineTranslationalPos = frontError - backError;
}
