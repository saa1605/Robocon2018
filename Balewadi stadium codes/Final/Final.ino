#include <io128.h>
#include <sra128.h>
#include "avr/io.h"
#include "avr/interrupt.h"
#include <pinDefsAuto.h>
#include "mcp.h"
#include <math.h>

volatile long int ticks = 0;    //power the bot keeping the stencil always
volatile int pidTicks = 0;

int armError = 0, prevArmError = 0, integralError = 0;
float armKp = 0.05, armKi = 0.01, armKd = 0.1;

int armPID = 0, armPWM = 0;
int pidCount = 0;

int throwingPWM = 0, throwCount = 0;

int pidMode = 1, toggle = 0;  //for pid tuning

#define numOfSensors 8

float alignKPError = 0.20, alignKDError = 0, alignKPRotation = 0.32, alignKDRotation = 0;
int alignOpt = 1500;

float fastKPError = 0.42, fastKDError = 0, fastKPRotation = 0.25, fastKDRotation = 0;
int fastOpt = 2500;

int frontOnline = 0, backOnline = 0, leftOnline = 0, rightOnline = 0;

//int sensorMinLeft[numOfSensors]  = {4000,  4000,  4000,  4000, 4000,  4000,  4000,  4000};
//int sensorMaxLeft[numOfSensors]  = {0, 0, 0, 0, 0, 0, 0, 0};
//int sensorMinRight[numOfSensors] = {4000,  4000,  4000,  4000,  4000,  4000,  4000, 4000};
//int sensorMaxRight[numOfSensors] = {0, 0, 0, 0, 0, 0, 0, 0};
//
//int sensorMinFront[numOfSensors]  = {4000,  4000,  4000,  4000,  4000,  4000,  4000,  4000};
//int sensorMaxFront[numOfSensors]  = {0, 0, 0, 0, 0, 0, 0, 0};                        
//int sensorMinBack[numOfSensors] = {4000,  4000,  4000,  4000,  4000,  4000,  4000,  4000};     
//int sensorMaxBack[numOfSensors] = {0, 0, 0, 0, 0, 0, 0, 0};                             

int sensorMinLeft[numOfSensors]  = {570,  570,  545,  561,  581,  571,  598,  578};
int sensorMaxLeft[numOfSensors]  = {2173, 2121, 1175, 1882, 2283, 2098, 2464, 2332};
int sensorMinRight[numOfSensors] = {745,  570,  582,  564,  1021, 1045, 1411, 1759};
int sensorMaxRight[numOfSensors] = {2561, 1517, 1998, 1386, 2872, 2841, 2924, 2953};

int sensorMinFront[numOfSensors]  = {948,  631,  592,  610,  571,  588,  594,  624};
int sensorMaxFront[numOfSensors]  = {2702, 2391, 1761, 2220, 2102, 2376, 2411, 2563};                        
int sensorMinBack[numOfSensors] = {583,  578,  559,  560,  563,  682,  580,  588};     
int sensorMaxBack[numOfSensors] = {2344, 2431, 2027, 1901, 2039, 2193, 2346, 2307};                             

int maxPWM = 400, alignMaxPWM = 400, fastMaxPWM = 500;                  

int errorRight = 0, errorLeft = 0, errorFront = 0, errorBack = 0, error = 0, prevError = 0;          
int rotation = 0, prevRotation = 0;

int sensorRawRight[numOfSensors], sensorValRight[numOfSensors], sensorRawLeft[numOfSensors], sensorValLeft[numOfSensors];
int sensorRawFront[numOfSensors], sensorValFront[numOfSensors], sensorRawBack[numOfSensors], sensorValBack[numOfSensors];

int turnFlag = -1;  //0-Left  1-Right  2-Forward  3-Back
int firstTurn = -1;   //0-after 1st turn and 1-before                                  

float kPError = fastKPError, kDError = fastKDError, kPRotation = fastKPRotation, kDRotation = fastKDRotation;
int opt = fastOpt;

int juncCount = 0;
bool juncFlag = true;

int decimal = 0;   //for aruco correction

int sLeft, sRight, sFront, sBack;

// Main loop, read and display data
//-----------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  //Serial.println("Setup Started");
  encoderInterruptInitialize();
  baseMotorsInit();
  throwingMotorInit();
  clampPistonInit();
  arucoInterfaceInit();
  spiMasterInit();
//  openClamp();
  
  DDRD &= ~(1 << PD3);  //just for practise
  PORTD |= (1 << PD3);  //just for practise
  pidAlign(0);  //fast line following in the begining
  PORTD &= ~(1 << PD6); //Throwing Motor brake
  //Serial.println("Setup Over");
}

void loop()
{
  if(bit_is_set(PINF,0))
  {
    PORTC |= 0xFF;
    delay(1000);
    PORTC = 0x00;
    
    //Serial.print("here");
		turnFlag = 0;
		firstTurn = 1;
  	while(1)
  	{
//      alignArm();
//      while(bit_is_set(PIND,3));
       
	  	lineFollow();
//      SerialPrint();
      
//      toggle = 0;
//      calibrateLeft();
//      calibrateRight();
//      calibrateFront();
//      calibrateBack();
//      while(bit_is_set(PIND,3));
//      toggle = 1;
//      calibrateLeft();
//      calibrateRight();
//      calibrateFront();
//      calibrateBack();
//      while(bit_is_set(PIND,3));
	  }
	}
	else
	{
    //Serial.print("not here");
	  botKill();
	  //Serial.println(" Show Aruco");
	}
}

void SerialPrint()
{
//  //Serial.print(sFront);//Serial.print(' ');
//  //Serial.print(sBack);//Serial.print(' ');
//  //Serial.print(sLeft);//Serial.print(' ');
//  //Serial.print(sRight);//Serial.print(' ');
//  //Serial.print('\t');
   
   //Serial.print(ticks%600);//Serial.print('\t');
   //Serial.print("  leftOnline = ");//Serial.print(leftOnline);
   //Serial.print("  rightOnline = ");//Serial.print(rightOnline);
   //Serial.print("  frontOnline = ");//Serial.print(frontOnline);
   //Serial.print("  backOnline = ");//Serial.print(backOnline);
   
//  for(int i = 0; i < numOfSensors; i++)
//  {
//    //Serial.print(sensorValFront[i]);//Serial.print(' ');
//  }
//  //Serial.print("\t"); 
//  for(int i = 0; i < numOfSensors; i++)
//  {
//    //Serial.print(sensorValBack[i]);//Serial.print(' ');
//  }
//  //Serial.print("\t");  
//  
//  for(int i = 0; i < numOfSensors; i++)
//  {
//    //Serial.print(sensorValLeft[i]);//Serial.print(' ');
//  }
//  //Serial.print("\t");
//  for(int i = 0; i < numOfSensors; i++)
//  {
//    //Serial.print(sensorValRight[i]);//Serial.print(' ');
//  }
//  //Serial.print("\t");

//   //Serial.print("  errLeft = ");
//   //Serial.print(errorLeft);
//   //Serial.print(" errRight ");
//   //Serial.print(errorRight);
//   //Serial.print("  Err = ");
//   //Serial.print(error);
//   //Serial.print("  Rot = ");
//   //Serial.print(rotation);
//   //Serial.print("  firstTurn = ");
//   //Serial.print(firstTurn);
   //Serial.print("  juncCount = ");
   //Serial.print(juncCount);
//   //Serial.print("  turnFlag = ");
//   //Serial.print(turnFlag);
  
   //Serial.println("");
}

void encoderInterruptInitialize()
{
  DDRD &= ~(1 << PD4);
  DDRD &= ~(1 << PD1);
  PORTD |= (1 << PD4)|(1 << PD1);
  EICRA |= (1 << ISC11);
  EIMSK |= (1 << INT1);
}

void baseMotorsInit()
{
  DDRC |= 0xFF;
  PORTC |= 0xFF;
  pwm0_init();
  pwm1_init();
  DDRD |= (1 << PD7);  //For Relay braking of Base
  PORTD |= (1 << PD7); //Active Low
}

void throwingMotorInit()
{
  DDRD |= (1 << PD0)|(1 << PD2); //Throwing Motor
  DDRD |= (1 << PD6);  //For Relay braking of Throwing
  PORTD |= (1 << PD6); //Active Low
}

void clampPistonInit()
{
  DDRE |= (1 << PE6)|(1 << PE7);  //Clamp Piston
}

void arucoInterfaceInit()
{
  DDRF &= ~((1 << PF0)|(1 << PF1)|(1 << PF2)|(1 << PF3));   //PF0 - detection, PF 1,2,3 - Decimal
  PORTF &= ~((1 << PF0)|(1 << PF1)|(1 << PF2)|(1 << PF3));  //PF0 - detection, PF 1,2,3 - Decimal
//  DDRD &= ~(1 << PD3);    //for sign of decimal
//  PORTD &= ~(1 << PD3);   //set = -ve and clear = +ve
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
  //Serial.println("Calibrating Front");
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
  //Serial.println("Front Calibrated");
  for(int i = 0; i < numOfSensors; i++)
  {
    //Serial.print("Min : ");
    //Serial.print(sensorMinFront[i]);
    //Serial.print('\t');
  }
  for(int i = 0; i < numOfSensors; i++)
  {
    //Serial.print("Max : ");
    //Serial.print(sensorMaxFront[i]);
    //Serial.print('\t');
  }
  //Serial.print('\n');
}

void calibrateBack()
{
  //Serial.println("Calibrating Back");
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
  //Serial.println("Back Calibrated");
  for(int i = 0; i < numOfSensors; i++)
  {
    //Serial.print("Min : ");
    //Serial.print(sensorMinBack[i]);
    //Serial.print('\t');
  }
  for(int i = 0; i < numOfSensors; i++)
  {
    //Serial.print("Max : ");
    //Serial.print(sensorMaxBack[i]);
    //Serial.print('\t');
  }
  //Serial.print('\n');
  delay(2000);
}

void calibrateRight()
{
  //Serial.println("Calibrating Right");
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
  //Serial.println("Right Calibrated");
  for(int i = 0; i < numOfSensors; i++)
  {
    //Serial.print("Min : ");
    //Serial.print(sensorMinRight[i]);
    //Serial.print('\t');
  }
  for(int i = 0; i < numOfSensors; i++)
  {
    //Serial.print("Max : ");
    //Serial.print(sensorMaxRight[i]);
    //Serial.print('\t');
  }
  //Serial.print('\n');
}

void calibrateLeft()
{
  //Serial.println("Calibrating Left");
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
  //Serial.println("Left Calibrated");
  for(int i = 0; i < numOfSensors; i++)
  {
    //Serial.print("Min : ");
    //Serial.print(sensorMinLeft[i]);
    //Serial.print('\t');
  }
  for(int i = 0; i < numOfSensors; i++)
  {
    //Serial.print("Max : ");
    //Serial.print(sensorMaxLeft[i]);
    //Serial.print('\t');
  }
  //Serial.print('\n');
}

void calcErrorFront() //extreme right 3500 and left -3500
{
  frontOnline = 0;
  unsigned long avg = 0, sum = 0, pos = (numOfSensors - 1) * 1000.0;
  for (int j = 0; j < numOfSensors; j++)
  {
    if (sensorValFront[j] > 600)
      frontOnline++;
    if (sensorValFront[j] > 250)
    {
      avg += (long)(sensorValFront[j]) * ((j) * 1000);
      sum += sensorValFront[j];
    }
  }
  if (sum != 0)
  {
    pos = avg / sum;
  }

  if(frontOnline == 0)
  { 
    if(errorFront > 0)
      pos = (numOfSensors - 1) * 1000.0;                    // If it last read to the right of center, return the max.
    else
      pos = 0;                        // If it last read to the left of center, return 0.
  }
  errorFront = pos - (numOfSensors - 1) * 500.0;
}

void calcErrorBack()  //extreme right 3500 and left -3500
{
  backOnline = 0;
  unsigned long avg = 0, sum = 0, pos = (numOfSensors - 1) * 1000.0;
  for (int j = 0; j < numOfSensors; j++)
  {
    if (sensorValBack[j] > 600)
      backOnline++;
    if (sensorValBack[j] > 300)
    {
      avg += (long)(sensorValBack[j]) * ((j) * 1000);
      sum += sensorValBack[j];
    }
  }
  if (sum != 0)
  {
    pos = avg / sum;
  }

  if(backOnline == 0)
  { 
    if(errorBack > 0)
      pos = (numOfSensors - 1) * 1000.0;                    // If it last read to the right of center, return the max.
    else
      pos = 0;                        // If it last read to the left of center, return 0.
  }
  errorBack = pos - (numOfSensors - 1) * 500.0;
  if(firstTurn == 2)
  {
    errorBack = 0;
    if(backOnline >= 1)
    {
      firstTurn = 0;
      pidAlign(0);
    }
  }
}

void calcErrorLeft()
{
  leftOnline = 0;
  unsigned long avg = 0, sum = 0, pos = (numOfSensors - 1) * 1000.0;
  for (int j = 0; j < numOfSensors; j++)
  {
    if (sensorValLeft[j] > 600)
      leftOnline++;
    if (sensorValLeft[j] > 300)
    {
      avg += (long)(sensorValLeft[j]) * ((j) * 1000);
      sum += sensorValLeft[j];
    }
  }
  if (sum != 0)
  {
    pos = avg / sum;
  }

  if(leftOnline == 0)
  {
    if(errorLeft > 0)
      pos = (numOfSensors - 1) * 1000.0;      // If it last read to the right of center, return the max.
    else
      pos = 0;      // If it last read to the left of center, return 0.
  }
  errorLeft = pos - (numOfSensors - 1) * 500.0;
  if(turnFlag == 0 && firstTurn == 1 && (leftOnline == 0 || leftOnline >= 3))  //leftOnline >= 3
  {
    errorLeft = 0;
  }       
}

void calcErrorRight()
{
  rightOnline = 0;
  unsigned long avg = 0, sum = 0, pos = (numOfSensors - 1) * 1000.0;
  for (int j = 0; j < numOfSensors; j++)
  {
    if (sensorValRight[j] > 600)
      rightOnline++;
    if (sensorValRight[j] > 300)
    {
      avg += (long)(sensorValRight[j]) * ((j) * 1000);
      sum += sensorValRight[j];
    }
  }
  if (sum != 0)
  {
    pos = avg / sum;
  }
  if(rightOnline == 0)
  {
    if(errorRight > 0)
      pos = (numOfSensors - 1) * 1000.0;      // If it last read to the right of center, return the max.
    else
      pos = 0;        // If it last read to the left of center, return 0.
  }
  errorRight = pos - (numOfSensors - 1) * 500.0;
}

void calcErrorAndRotation()  //error is negative when bot is behind the line  //rot is CCW +ve
{
  if(turnFlag == 0 || turnFlag == 1)
  {
    error = (errorRight + errorLeft)*(10000.0/7000.0);
    rotation = (errorRight - errorLeft);
  }
  else if(turnFlag == 2 || turnFlag == 3)
  {
    error = (errorFront + errorBack)*(10000.0/7000.0);
    rotation = (errorFront - errorBack);
  }
}

void pidAlign(int pidAlign)   //pidALign(0) - Fast and pidAlign(1) - Slow
{
  if(pidAlign)
  {
    kPError = alignKPError; kDError = alignKDError; kPRotation = alignKPRotation; kDRotation = alignKDRotation;
    opt = alignOpt;
    maxPWM = alignMaxPWM;
  }
  else
  {
    kPError = fastKPError; kDError = fastKDError; kPRotation = fastKPRotation; kDRotation = fastKDRotation;
    opt = fastOpt;
    maxPWM = fastMaxPWM;
  }
}

void calcSpeedToMotors()
{
  switch(turnFlag)
  {
    case 0:
    ///to left
    sFront = 0 - (opt + kPRotation * rotation - kDRotation * (rotation - prevRotation));
    sLeft  =     (kPError * error - kDError * (error - prevError));
    sBack  = 0 - (opt - kPRotation * rotation + kDRotation * (rotation - prevRotation));
    sRight =     (kPError*error - kDError * (error - prevError));

    if(sFront > 0)
      sFront = 0;

    if(sBack > 0)
      sBack = 0;
    break;

    case 1:
    ///to right
    sFront =     (opt - kPRotation * rotation + kDRotation * (rotation - prevRotation));
    sLeft  =     (kPError * error - kDError * (error - prevError));  
    sBack  =     (opt + kPRotation * rotation - kDRotation * (rotation - prevRotation));
    sRight =     (kPError * error - kDError * (error - prevError));

    if(sFront < 0)
      sFront = 0;

    if(sBack < 0)
      sBack = 0;
    break;

    case 2:
    ///to front
    sFront  =     (kPError * error - kDError * (error - prevError));  
    sLeft   =     (opt + kPRotation * rotation - kDRotation * (rotation - prevRotation));
    sBack   =     (kPError * error - kDError * (error - prevError));
    sRight  =     (opt - kPRotation * rotation + kDRotation * (rotation - prevRotation));

    if(sLeft < 0)
      sLeft = 0;

    if(sRight < 0)
      sRight = 0;
    break;
    
    case 3:
    //to back
    sFront  =     (kPError * error - kDError * (error - prevError));  
    sLeft   = 0 - (opt - kPRotation * rotation + kDRotation * (rotation - prevRotation));
    sBack   =     (kPError * error - kDError * (error - prevError));
    sRight  = 0 - (opt + kPRotation * rotation - kDRotation * (rotation - prevRotation));

    if(sLeft > 0)
      sLeft = 0;

    if(sRight > 0)
      sRight = 0;
    break;

    default:break;
  }
  prevError = error;
  prevRotation = rotation;
}

void uploadSpeedLineFollow(int maxPWMLine)
{
  PORTD |= (1 << PD7);  //for unbraking base motors
  
  if(turnFlag == 0 || turnFlag == 1)
  {
    sFront = map(sFront, 0 - (opt + kPRotation * 7000), (opt + kPRotation * 7000), 0 - maxPWM, maxPWM);
    sLeft  = map(sLeft,  0 - 10000*kPError, 10000*kPError, 0 - maxPWMLine*0.5, maxPWMLine*0.5);
    sBack  = map(sBack,  0 - (opt + kPRotation * 7000), (opt + kPRotation * 7000), 0 - maxPWM, maxPWM);
    sRight = map(sRight, 0 - 10000*kPError, 10000*kPError, 0 - maxPWMLine*0.5, maxPWMLine*0.5);  
  }
  else if (turnFlag == 2 || turnFlag == 3)
  {
    sFront = map(sFront, 0 - (10000*kPError+1), (10000*kPError + 1), 0 - maxPWMLine*0.5, maxPWMLine*0.5);//+100-100
    sLeft = map(sLeft,   0 - (opt + kPRotation * 7000), (opt + kPRotation * 7000), 0 - maxPWM, maxPWM);//+350-350
    sBack = map(sBack,   0 - (10000*kPError + 1), (10000*kPError + 1), 0 - maxPWMLine*0.5, maxPWMLine*0.5);//+100-100
    sRight = map(sRight, 0 - (opt + kPRotation * 7000), (opt + kPRotation * 7000), 0 - (maxPWM-20 ), maxPWM-20);//+390-390
  }
  
  if(sFront < -30)
  {
    sFront *= (-1);
    MOTORFF = 0;
    MOTORFB = 1;
    if(sFront > maxPWMLine)
      sFront = maxPWMLine;
    sFront = map(sFront, 0, 666, 0, 255);
    PWMF = sFront;
  }
  else if(sFront > 30)
  {
    MOTORFF = 1;
    MOTORFB = 0;
    if(sFront > maxPWMLine)
      sFront = maxPWMLine;
    sFront = map(sFront, 0, 666, 0, 255);
    PWMF = sFront;
  }
  else
  {
    MOTORFF = 1;
    MOTORFB = 1;
    PWMF = 666;
  }

  if(sBack < -30)
  {
    sBack *= (-1);
    MOTORBF = 0;
    MOTORBB = 1;
    if(sBack > maxPWMLine)
      sBack = maxPWMLine;
    PWMB = sBack;
  }
  else if(sBack > 30)
  {
    MOTORBF = 1;
    MOTORBB = 0;
    if(sBack > maxPWMLine)
      sBack = maxPWMLine;
    PWMB = sBack;
  }
  else
  {
    MOTORBF = 1;
    MOTORBB = 1;
    if(sBack > maxPWMLine)
      sBack = maxPWMLine;
    PWMB = 666;
  }

  if(sLeft < -30)
  {
    sLeft *= (-1);
    MOTORLF = 0;
    MOTORLB = 1;
    if(sLeft > maxPWMLine)
      sLeft = maxPWMLine;
    PWML = sLeft;
  }
  else if(sLeft > 30)
  {
    MOTORLF = 1;
    MOTORLB = 0;
    if(sLeft > maxPWMLine)
      sLeft = maxPWMLine;
    PWML = sLeft;
  }
  else
  {
    MOTORLF = 1;
    MOTORLB = 1;
    PWML = 666;
  }

  if(sRight < -30)
  {
    sRight *= (-1);
    MOTORRF = 0;
    MOTORRB = 1;
    if(sRight > maxPWMLine)
      sRight = maxPWMLine;
    PWMR = sRight;
  }
  else if(sRight > 30)
  {
    MOTORRF = 1;
    MOTORRB = 0;
    if(sRight > maxPWMLine)
      sRight = maxPWMLine;
    PWMR = sRight;
  }
  else
  {
    MOTORRF = 1;
    MOTORRB = 1;
    PWMR = 666;
  }
}

void lineFollow()
{
  readSensorFront();
  readSensorBack();
  calcSensorValFront();
  calcSensorValBack();
  calcErrorFront();
  calcErrorBack();
 
  readSensorLeft();
  readSensorRight();
  calcSensorValLeft();
  calcSensorValRight();
  calcErrorLeft();
  calcErrorRight();
  
  if(leftOnline <= 1 && rightOnline >= 2 && frontOnline >= 2 && backOnline == 0 && firstTurn == 1)  //the 1st turn
  {
    turnFlag = 2;
    firstTurn = 2;
  }

  if((((frontOnline >= 4 && turnFlag == 2) || (leftOnline >= 6 && turnFlag == 0) || (rightOnline >= 4 && turnFlag == 1)) && firstTurn == 0) || (leftOnline == 0 && firstTurn == 1))
  {  
    pidAlign(1);
  }

  if(frontOnline >= 1 && backOnline >= 1  &&  leftOnline >= 1 && rightOnline >= 1 && firstTurn==0)   //junction counter
  {
    if(juncFlag)
    {
      if(turnFlag == 1)    //Rightwards linefollow
        juncCount--;
      else
        juncCount++;
       
      juncFlag = false;
    }
  }
  else if((backOnline == 0 && frontOnline == 0) || (leftOnline == 0 && rightOnline == 0))
    juncFlag = true;

  if(juncCount == 1 && (turnFlag == 2 || turnFlag == 1)) //tranfer junction for TZ1
  {
    botKill();
    delay(1000);
    pidAlign(1);
    juncFlag = false;
    if(turnFlag == 2)
    	turnFlag = 3;
    else if(turnFlag == 1)
    	turnFlag = 0;
    while(!((sensorValLeft[3]>400 || sensorValLeft[4]>400) && (sensorValRight[3]>400 || sensorValRight[4]>400)))
    {
      //Serial.print("#@$&^$$#");
      readSensorFront();
      readSensorBack();
      calcSensorValFront();
      calcSensorValBack();
      calcErrorFront();
      calcErrorBack();
     
      readSensorLeft();
      readSensorRight();
      calcSensorValLeft();
      calcSensorValRight();
      calcErrorLeft();
      calcErrorRight();
      
      calcErrorAndRotation();
      calcSpeedToMotors();
      uploadSpeedLineFollow(maxPWM);
    }
    botKill();
    alignArm();
    openClamp();
    while(bit_is_clear(PINF,0))
      Serial.println(ticks%600);
    
    PORTC |= 0xFF;
    delay(1000);
    PORTC = 0x00;
    closeClamp();
    int arcount = 0;
    while(arcount < 10000)             //////5000
    {
    	if(bit_is_clear(PINF,0))
    		arcount++;
    	else
    		arcount = 0;
  	}
    turnFlag = 0;
    pidAlign(0);    
  }
  else if(juncCount == 2 && turnFlag == 0) //throwing junction of TZ1
  {
    botKill();
    juncFlag = false;
    delay(1000);
    throwAtTZ1();
    turnFlag = 1;
    pidAlign(0);
  }

  calcErrorAndRotation();
  calcSpeedToMotors();
  uploadSpeedLineFollow(maxPWM);
}

void closeClamp()
{
  MOTOROC = 0;
  MOTORCC = 0;
}
void openClamp()
{
  MOTOROC = 1;
  MOTORCC = 0;
}

void forwardRotation()
{
  MOTORFR = 1;
  MOTORBR = 0;
}

void backwardRotation()
{
  MOTORFR = 0;
  MOTORBR = 1;
}

void stopRotation()
{
  MOTORFR = 0;
  MOTORBR = 0;
}

void botKill()
{
  MOTORFF = 0;
  MOTORFB = 0;
  PWMF = 0;

  MOTORBF = 0;
  MOTORBB = 0;
  PWMB = 0;

  MOTORLF = 0;
  MOTORLB = 0;
  PWML = 0;

  MOTORRF = 0;
  MOTORRB = 0;
  PWMR = 0;

  PORTD &= ~(1 << PD7);   //for base motor braking
}

int gpioToDecimal()
{
  int decimal = 0;

  for(int i = 1; i < 4; i++)
    decimal += (bit_is_set(PINF,i)) * pow(2,i-1);

  if(bit_is_set(PIND,3))
    decimal *= -1;

  return decimal;
}

void throwAtTZ1()
{
  throwCount = 0;
  //Serial.println("Entered throw ");
  forwardRotation();  //to give forward direction to the motor
  throwingPWM = 100;
  PORTD |= (1 << PD6);  //unbrake the throwing motor
  while(bit_is_set(PIND,6))
  {
     //Serial.print(", ticks : "); //Serial.print(ticks%600);
     //Serial.print(", throwCount : "); //Serial.print(throwCount);
     //Serial.print(", throwingPWM : "); //Serial.print(throwingPWM);
     //Serial.println("");
    if((ticks%600 <525) && (ticks%600 >515) && throwCount == 5)   //change the "ticks%600 == 250" for the tick count at which it is required to throw
    {
      throwCount = 0;
      openClamp();
      stopRotation();
      throwingPWM = 0;
      PORTD &= ~(1 << PD6);   //brake the throwing motor
    }
    OCR0 = throwingPWM;
  }
  // //Serial.print("ThrowingSuccessfull");
}

int adjust()
{
  if(ticks > 0)
  {
    if(ticks%600 > 300)
      pidTicks = (ticks%600 - 600);  //%600;
    else
      pidTicks = ticks%600;
  }
  else
  {
    if(ticks%600 < -300)
      pidTicks = (ticks%600 + 600);  //%(-600); //agar sign ka error aaya toh directly *(-1) karde
    else
      pidTicks = ticks%(-600);   //agar sign ka error aaya toh directly *(-1) karde
  }
  return pidTicks;
}

void alignArm()
{
  int pidCount = 10000;
  while(pidCount != 0)
  {
    armError = adjust();
    while(armError < -3 || 3 < armError)
    {
      PORTD |= (1 << PD6); //for unbraking the throwing motor
      armError = adjust();
  
      integralError += armError;
      integralError = constrain(integralError, 0, 300);
      armPID = abs(armKp * armError + armKi * integralError);
  
      armPWM = map(armPID * 100, 55, 1500, 20, 70);
      armPWM = constrain(armPWM, 20, 70);
      
      if(armError > 0)
        backwardRotation();
      else if(armError < 0)
        forwardRotation();
  
      OCR0 = armPWM;
      prevArmError = armError;
//      Serial.println(ticks%600);
      //Serial.print(" armError = ");//Serial.print(armError);
      //Serial.print(" armPWM = ");//Serial.print(armPWM);
      //Serial.println();
    }
    OCR0 = 0;
    stopRotation();
    PORTD &= ~(1 << PD6); //for braking the throwing motor
    //Serial.print(" pidCount = ");//Serial.println(pidCount);
    pidCount--;
  }
  //Serial.println("alignArm Ended");
}

ISR(INT1_vect)
{
  if(bit_is_set(PIND,4))
    ticks--;
    
  else if(bit_is_clear(PIND,4))
    ticks++;
  
  if(ticks%600 == 0)
    throwCount++;
}

