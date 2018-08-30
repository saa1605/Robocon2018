#include <io128.h>
#include <sra128.h>
#include <pinDefsAutoNew.h>
#include "mcp.h"

#define numOfSensors 8
//
//int sensorMinLeft[numOfSensors]  = {4000,  4000,  4000,  4000, 4000,  4000,  4000,  4000};
//int sensorMaxLeft[numOfSensors]  = {0, 0, 0, 0, 0, 0, 0, 0};
//int sensorMinRight[numOfSensors] = {4000,  4000,  4000,  4000,  4000,  4000,  4000, 4000};
//int sensorMaxRight[numOfSensors] = {0, 0, 0, 0, 0, 0, 0, 0};
//
//int sensorMinFront[numOfSensors]  = {4000,  4000,  4000,  4000,  4000,  4000,  4000,  4000};
//int sensorMaxFront[numOfSensors]  = {0, 0, 0, 0, 0, 0, 0, 0};                        
//int sensorMinBack[numOfSensors] = {4000,  4000,  4000,  4000,  4000,  4000,  4000,  4000};     
//int sensorMaxBack[numOfSensors] = {0, 0, 0, 0, 0, 0, 0, 0};

int sensorMinFront[numOfSensors] = {1898, 1509, 1579, 1794, 857, 1845, 1326, 1385};
int sensorMaxFront[numOfSensors] = {3013, 3012, 3012, 3012, 2839, 3014, 3013, 3006};
int sensorMinLeft[numOfSensors] = {2119, 1969, 1513, 1384, 1204, 1667, 1830, 2340};
int sensorMaxLeft[numOfSensors] = {3004, 3005, 3003, 3004, 2928, 3005, 3005, 3005};
int sensorMinRight[numOfSensors] = {1698, 1783, 1944, 1835, 2222, 1784, 2199, 2122};
int sensorMaxRight[numOfSensors] = {3021, 3020, 3021, 3021, 3027, 3028, 3028, 3028};
//int sensorMinFront[numOfSensors] = {1590, 1385, 1662, 1961, 707, 1955, 1074, 1231};
//int sensorMaxFront[numOfSensors] = {3022, 3021, 3020, 3020, 2853, 3023, 3022, 3023};
int sensorMinBack[numOfSensors] = {497, 2273, 2124, 1642, 1385, 1563, 1457, 2535};
int sensorMaxBack[numOfSensors] = {97, 3062, 3038, 2914, 2956, 3018, 2960, 3030};
//back ka 0th 0 kiya hai
//calibrated values of 14th Feb
//int sensorMinLeft[numOfSensors] = {2038, 1778, 1425, 1409, 1018, 1549, 1693, 2197};
//int sensorMaxLeft[numOfSensors] = {2992, 2992, 2991, 2992, 2943, 2991, 2991, 2991};
//int sensorMinRight[numOfSensors] = {1115, 1226, 1432, 1242, 1572, 1150, 1599, 1562};
//int sensorMaxRight[numOfSensors] = {2988, 3003, 3004, 3004, 3008, 3009, 3009, 3009};
//int sensorMinFront[numOfSensors] = {1898, 1509, 1579, 1794, 857, 1845, 1326, 1385};
//int sensorMaxFront[numOfSensors] = {3013, 3012, 3012, 3012, 2839, 3014, 3013, 3006};
//int sensorMinBack[numOfSensors] = {2599, 1580, 1482, 964, 840, 1093, 956, 1478};
//int sensorMaxBack[numOfSensors] = {3003, 3004, 3005, 2973, 2893, 3011, 2950, 3020};

int sensorRawRight[numOfSensors], sensorValRight[numOfSensors], sensorRawLeft[numOfSensors], sensorValLeft[numOfSensors];
int sensorRawFront[numOfSensors], sensorValFront[numOfSensors], sensorRawBack[numOfSensors], sensorValBack[numOfSensors];

int toggle = 0;   //0 - for calibrating white and 1 - for calibrating black

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup Start");
  
  spiMasterInit();
  
  DDRA &= ~((1 << PA4)|(1 << PA5)|(1 << PA6)|(1 << PA7));   //for buttons A4567
  PORTA |= ((1 << PA4)|(1 << PA5)|(1 << PA6)|(1 << PA7));   //for buttons A4567
  
  Serial.println("Setup End");
}

void loop()
{
  readSensorFront();
  readSensorBack();
  readSensorLeft();
  readSensorRight();
  
  calcSensorValFront();
  calcSensorValBack();
  calcSensorValLeft();
  calcSensorValRight();

  SerialPrint();

  if(bit_is_clear(PINA,7))
  {
    _delay_ms(1000);
    toggle = 0;       //calibrate white
    calibrateLeft();
    calibrateRight();
    calibrateFront();
    calibrateBack();
  
    while(bit_is_set(PINA,7));
    _delay_ms(1000);
    toggle = 1;      //calibrate black
    calibrateLeft();
    calibrateRight();
    calibrateFront();
    calibrateBack();
    while(bit_is_set(PINA,7));
  }
}

void SerialPrint()
{
//  for(int i = 0; i < numOfSensors; i++)
//  {
//   Serial.print(sensorRawFront[i]);Serial.print(' ');
//  }
//  Serial.print("\t"); 
//  for(int i = 0; i < numOfSensors; i++)
//  {
//   Serial.print(sensorRawBack[i]);Serial.print(' ');
//  }
//  Serial.print("\t");  
//
//  for(int i = 0; i < numOfSensors; i++)
//  {
//   Serial.print(sensorRawLeft[i]);Serial.print(' ');
//  }
//  Serial.print("\t");
//  for(int i = 0; i < numOfSensors; i++)
//  {
//   Serial.print(sensorRawRight[i]);Serial.print(' ');
//  }
//  Serial.print("\t");
//
  for(int i = 0; i < numOfSensors; i++)
  {
   Serial.print(sensorValFront[i]);Serial.print(' ');
  }
  Serial.print("\t"); 
  for(int i = 0; i < numOfSensors; i++)
  {
   Serial.print(sensorValBack[i]);Serial.print(' ');
  }
  Serial.print("\t");  

  for(int i = 0; i < numOfSensors; i++)
  {
   Serial.print(sensorValLeft[i]);Serial.print(' ');
  }
  Serial.print("\t");
  for(int i = 0; i < numOfSensors; i++)
  {
   Serial.print(sensorValRight[i]);Serial.print(' ');
  }
  Serial.print("\t");

  
  Serial.println("");
}

void readSensorFront() //front dekha toh left wala 0
{
  sensorRawFront[0] = getSensorReading(frontSensor, 4);
  sensorRawFront[1] = getSensorReading(frontSensor, 5);
  sensorRawFront[2] = getSensorReading(frontSensor, 6);
  sensorRawFront[3] = getSensorReading(frontSensor, 7);
  sensorRawFront[4] = getSensorReading(frontSensor, 0);
  sensorRawFront[5] = getSensorReading(frontSensor, 1);
  sensorRawFront[6] = getSensorReading(frontSensor, 2);
  sensorRawFront[7] = getSensorReading(frontSensor, 3);
}

void readSensorBack() //front dekha toh left wala 0
{
  sensorRawBack[0] = getSensorReading(backSensor, 3);
  sensorRawBack[1] = getSensorReading(backSensor, 2);
  sensorRawBack[2] = getSensorReading(backSensor, 1);
  sensorRawBack[3] = getSensorReading(backSensor, 0);
  sensorRawBack[4] = getSensorReading(backSensor, 7);
  sensorRawBack[5] = getSensorReading(backSensor, 6);
  sensorRawBack[6] = getSensorReading(backSensor, 5);
  sensorRawBack[7] = getSensorReading(backSensor, 4);
}

void readSensorRight()  //left dekha toh left wala 0
{
  sensorRawRight[0] = getSensorReading(rightSensor, 7);
  sensorRawRight[1] = getSensorReading(rightSensor, 6);
  sensorRawRight[2] = getSensorReading(rightSensor, 5);
  sensorRawRight[3] = getSensorReading(rightSensor, 4);
  sensorRawRight[4] = getSensorReading(rightSensor, 3);
  sensorRawRight[5] = getSensorReading(rightSensor, 2);
  sensorRawRight[6] = getSensorReading(rightSensor, 1);
  sensorRawRight[7] = getSensorReading(rightSensor, 0);
}

void readSensorLeft()  //left dekha toh left wala 0
{
  sensorRawLeft[0] = getSensorReading(leftSensor, 0);
  sensorRawLeft[1] = getSensorReading(leftSensor, 1);
  sensorRawLeft[2] = getSensorReading(leftSensor, 2);
  sensorRawLeft[3] = getSensorReading(leftSensor, 3);
  sensorRawLeft[4] = getSensorReading(leftSensor, 4);
  sensorRawLeft[5] = getSensorReading(leftSensor, 5);
  sensorRawLeft[6] = getSensorReading(leftSensor, 6);
  sensorRawLeft[7] = getSensorReading(leftSensor, 7);
}

void calcSensorValFront()
{
  for (int j = 0; j < numOfSensors; j++)
  {
    sensorValFront[j] = map(sensorRawFront[j], sensorMinFront[j], sensorMaxFront[j], 1000, 0);
    if (sensorValFront[j] < 0)
      sensorValFront[j] = 0;
    else if (sensorValFront[j] > 1000)
      sensorValFront[j] = 1000;
  }
}

void calcSensorValBack()
{
  for (int j = 0; j < numOfSensors; j++)
  {
    sensorValBack[j] = map(sensorRawBack[j], sensorMinBack[j], sensorMaxBack[j], 1000, 0);
    if (sensorValBack[j] < 0)
      sensorValBack[j] = 0;
    else if (sensorValBack[j] > 1000)
      sensorValBack[j] = 1000;
  }
  sensorValBack[0] = 0;    /////////////////////////----------------------------------------------------------------------------------
}

void calcSensorValRight()
{
  for (int j = 0; j < numOfSensors; j++)
  {
    sensorValRight[j] = map(sensorRawRight[j], sensorMinRight[j], sensorMaxRight[j], 1000, 0);
    if (sensorValRight[j] < 0)
      sensorValRight[j] = 0;
    else if (sensorValRight[j] > 1000)
      sensorValRight[j] = 1000;
  }
}

void calcSensorValLeft()
{
  for (int j = 0; j < numOfSensors; j++)
  {
    sensorValLeft[j] = map(sensorRawLeft[j], sensorMinLeft[j], sensorMaxLeft[j], 1000, 0);
    if (sensorValLeft[j] < 0)
      sensorValLeft[j] = 0;
    else if (sensorValLeft[j] > 1000)
      sensorValLeft[j] = 1000;
  }
}

void calibrateFront()
{
  unsigned int time = 10;
  if(toggle == 0)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorFront();
        for(int j = 0; j < numOfSensors; j++)
        {
          sensorMinFront[j] += sensorRawFront[j];
        }
     }
     for(int j = 0; j < numOfSensors; j++)
     {
        sensorMinFront[j] = sensorMinFront[j]/10;
     }
  }
  else if(toggle == 1)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorFront();
        for(int j = 0; j < numOfSensors; j++)
        {
          sensorMaxFront[j] += sensorRawFront[j];
        }
     }
     for(int j = 0; j < numOfSensors; j++)
     {
        sensorMaxFront[j] = sensorMaxFront[j]/10;
     }
  }
  
  Serial.print("int sensorMinFront[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMinFront[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
  
  Serial.print("int sensorMaxFront[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMaxFront[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
}

void calibrateBack()
{
  unsigned int time = 10;
  if(toggle == 0)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorBack();
        for(int j = 0; j < numOfSensors; j++)
        {
          sensorMinBack[j] += sensorRawBack[j];
        }
     }
     for(int j = 0; j < numOfSensors; j++)
     {
        sensorMinBack[j] = sensorMinBack[j]/10;
     }
      
  }
  else if(toggle == 1)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorBack();
        for(int j = 0; j < numOfSensors; j++)
        {
          sensorMaxBack[j] += sensorRawBack[j];
        }
     }
     for(int j = 0; j < numOfSensors; j++)
     {
        sensorMaxBack[j] = sensorMaxBack[j]/10;
     }
  }

  Serial.print("int sensorMinBack[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMinBack[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
  
  Serial.print("int sensorMaxBack[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMaxBack[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
  _delay_ms(2000);
}

void calibrateRight()
{
  unsigned int time = 10;

  if(toggle == 0)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorRight();
      for(int j = 0; j < numOfSensors; j++)
      {
        sensorMinRight[j] += sensorRawRight[j];
      }
     }
     for(int j = 0; j < numOfSensors; j++)
     {
        sensorMinRight[j] = sensorMinRight[j]/10;
     }
      
  }
  else if(toggle == 1)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorRight();
        for(int j = 0; j < numOfSensors; j++)
        {
          sensorMaxRight[j] += sensorRawRight[j];
        }
     }
     for(int j = 0; j < numOfSensors; j++)
     {
        sensorMaxRight[j] = sensorMaxRight[j]/10;
     }
  }

  Serial.print("int sensorMinRight[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMinRight[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
  
  Serial.print("int sensorMaxRight[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMaxRight[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
}

void calibrateLeft()
{
  unsigned int time = 10;

  if(toggle == 0)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorLeft();
        for(int j = 0; j < numOfSensors; j++)
        {
          sensorMinLeft[j] += sensorRawLeft[j];
        }
     }
     for(int j = 0; j  < numOfSensors; j++)
     {
        sensorMinLeft[j] = sensorMinLeft[j]/10;
     }
      
  }
  else if(toggle == 1)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorLeft();
        for(int j = 0; j < numOfSensors; j++)
        {
          sensorMaxLeft[j] += sensorRawLeft[j];
        }
     }
     for(int j = 0; j < numOfSensors; j++)
     {
        sensorMaxLeft[j] = sensorMaxLeft[j]/10;
     }
  }

  Serial.print("int sensorMinLeft[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMinLeft[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
  
  Serial.print("int sensorMaxLeft[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMaxLeft[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
}

