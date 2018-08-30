/**********************
Changes after 16th Feb
1. 
**********************/

#include <io128.h>
#include <sra128.h>
#include "avr/io.h"
#include "avr/interrupt.h"
#include <pinDefsAutoNew.h>
#include "mcp.h"
#include <math.h>
#include <Wire.h>
#include <VL53L0X.h>

#define debug   //comment for NO Serial.prints

VL53L0X sensor;

bool tz1 = false;   //false - TZ1 and true - TZ2
bool rongBay = false;   //false - TZ2 and true - TZ3

volatile long int ticks = 0;
volatile int pidTicks = 0, throwCount = 0;

bool firstTimeTZ2 = true, transferFlag = true;

bool firstTimeAlign = true, aligned = false, stopAlign = false, alignTemp = true;
int armError = 0, prevArmError = 0, integralError = 0, diffError = 0;
int armKp = 5, armKi = 0, armKd = 10;

int armPID = 0, armPWM = 0;
int pidCount = 0;

int throwingPWM = 0;

int pidMode = 1, toggle = 0;  //for pid tuning

#define numOfSensors 8

float alignKPError = 0.20, alignKDError = 0, alignKPRotation = 0.30, alignKDRotation = 0, sensitiveRot = 0.45;         ////alignRot:0.32
int alignOpt = 1500;

float fastKPError = 0.42, fastKDError = 0, fastKPRotation = 0.25, fastKDRotation = 0;
int fastOpt = 3000;

int frontOnline = 0, backOnline = 0, leftOnline = 0, rightOnline = 0;

//back ka 0th 0 kiya hai
int sensorMinFront[numOfSensors] = {1898, 1509, 1579, 1794, 857, 1845, 1326, 1385};
int sensorMaxFront[numOfSensors] = {3013, 3012, 3012, 3012, 2839, 3014, 3013, 3006};
int sensorMinLeft[numOfSensors] = {2119, 1969, 1513, 1384, 1204, 1667, 1830, 2340};
int sensorMaxLeft[numOfSensors] = {3004, 3005, 3003, 3004, 2928, 3005, 3005, 3005};
int sensorMinRight[numOfSensors] = {1698, 1783, 1944, 1835, 2222, 1784, 2199, 2122};
int sensorMaxRight[numOfSensors] = {3021, 3020, 3021, 3021, 3027, 3028, 3028, 3028};
int sensorMinBack[numOfSensors] = {497, 2273, 2124, 1642, 1385, 1563, 1457, 2535};
int sensorMaxBack[numOfSensors] = {97, 3062, 3038, 2914, 2956, 3018, 2960, 3030};

int maxPWM = 400, alignMaxPWM = 575, fastMaxPWM = 650, slowAlignPWM = 400, mediumAlignPWM = 600;

int errorRight = 0, errorLeft = 0, errorFront = 0, errorBack = 0, error = 0, prevError = 0;          
int rotation = 0, prevRotation = 0;

int sensorRawRight[numOfSensors], sensorValRight[numOfSensors], sensorRawLeft[numOfSensors], sensorValLeft[numOfSensors];
int sensorRawFront[numOfSensors], sensorValFront[numOfSensors], sensorRawBack[numOfSensors], sensorValBack[numOfSensors];

int turnFlag = -1;  //0-Left  1-Right  2-Forward  3-Back
int firstTurn = -1;   //0-after 1st turn and 1-before                                  

float kPError = fastKPError, kDError = fastKDError, kPRotation = fastKPRotation, kDRotation = fastKDRotation;
int opt = fastOpt;

int juncCount = 0, shortPlus = 0;
bool juncFlag = true, shortPlusFlag = true, frontPlusFlag = true;
int frontPlus = 0; 

int sLeft, sRight, sFront, sBack;
int32_t distance = 0, minDistanceThreshold = 220, maxDistanceThreshold = 350;   //for VL53L0X (TOF)

int throws = 0;   //to count the no of throws

int TZ1Ticks = -229, TZ2Ticks = -223, TZ3Ticks = -212;    //-228 -225 -215
int TZ1ThrowCount = 3, TZ2ThrowCount = 4, TZ3ThrowCount = 5;    //2 4 5
int throwPWM1 = 255, throwPWM2 = 215, throwPWM3 = 212;    //255 215 180

// Main loop, read and display data
//-----------------------------------------------------------------------

void setup()
{ 
  #ifdef debug
  Serial.begin(115200);
  #endif
  baseMotorsInit();
  encoderInterruptInit();
  throwingMotorInit();
  clampPistonInit();
  piInterfaceInit();
  spiMasterInit();
 
  pidAlign(0);  //fast line following in the begining
  throwBrake();   //for braking the throwing motor

  DDRA &= ~((1 << PA4)|(1 << PA5)|(1 << PA6)|(1 << PA7));   //for buttons A4567
  PORTA |= ((1 << PA4)|(1 << PA5)|(1 << PA6)|(1 << PA7));   //for buttons A4567

  DDRA |= (1 << PA0)|(1 << PA1);   //for LED Flashing and led strip
  PORTA &= ~((1 << PA0)|(1 << PA1));   //Active High
  
  initTOF();
}

void loop()
{
  if(bit_is_clear(PINA,7))    //Full Run
  {
    PORTC = 0xFF;
    _delay_ms(20);
    PORTC = 0x00;
    turnFlag = 0;
    firstTurn = 1;
    while(1)
    {
      alignArm();
      lineFollow();
      SerialPrint();

      if(throws >= 20)
      {
        botKill();
        while(1)
        {
          PORTC = 0xFF;
          _delay_ms(100);
          PORTC = 0x00;
          _delay_ms(100);
        }
      }
    }
  }
  
  else if(bit_is_clear(PINA,4))
  {
    while(1)
    {
      while(bit_is_set(PINA,4));
      openClamp();
      alignAfterFailure();
      
      while(bit_is_set(PINA,4));
      closeClamp();
      _delay_ms(500);
      while(bit_is_clear(PIND,4));
      
      throwUnbrake();   //unbraking the throwing motor
      backwardRotation();   //to give backwards direction to the motor
      PWMT = map(20, 0, 255, 0, 666);
      while(ticks > -80)                  //////////////***************************************
      {
        PORTC = 0x0F;
      }
      PORTC = 0x00;
      stopRotation();
      PWMT = 0;
//      throwBrake();   //for braking the throwing motor
    }
  }
  
  else if(bit_is_clear(PINA,5))
  {
    PORTC = 0xFF;
    transferFlag = false;
    firstTimeTZ2 = true;

    PORTA |= (1 << PA1); //LED STRIP ON
    int colorCount = 0;
    while(colorCount < 500)
    {
      if(!TZ2)
        colorCount++;
      else if(!TZ1)
        colorCount++;
      else if(!TZ3)
        colorCount++;
      else
        colorCount = 0;
    }
    PORTA&=~(1 << PA1); //LED STRIP OFF
    
    if(!TZ1)
      tz1 = false;    //TZ1
    else if(!TZ2)
      tz1 = true;    //TZ2
    else if(!TZ3)
    {
      rongBay = true;   //TZ3
      tz1 = true;
    }
    aligned = true;
    PORTC = 0x00;
  }
  
  else if(bit_is_clear(PINA,6))
  {
    PORTC |= 0xFF;
    tz1 = true;
    firstTimeTZ2 = false;
   _delay_ms(20);
   PORTC = 0x00;
  }
  else
  {
    botKill();
  }
}

void SerialPrint()
{
  #ifdef debug
//  Serial.print("  firstTurn = ");Serial.print(firstTurn);
//  Serial.print("  turnFlag = ");Serial.print(turnFlag);
//  Serial.print("  juncCount = ");Serial.print(juncCount);
//  Serial.print("  tz1 = ");Serial.print(tz1);
//  Serial.print("  rongBay = ");Serial.print(rongBay);
//  Serial.print("  frontPlus = ");Serial.print(frontPlus);
//  Serial.print("  frontPlusFlag = ");Serial.print(frontPlusFlag);

  Serial.print("  frontOnline = ");Serial.print(frontOnline);
  Serial.print("  backOnline = ");Serial.print(backOnline);
  Serial.print("  leftOnline = ");Serial.print(leftOnline);
  Serial.print("  rightOnline = ");Serial.print(rightOnline);

  Serial.print("  errorFront = ");Serial.print(errorFront);
  Serial.print("  errorBack = ");Serial.print(errorBack);
  Serial.print("  errorRight = ");Serial.print(errorRight);
  Serial.print("  errorLeft = ");Serial.print(errorLeft);
  Serial.print("\t");
  
  Serial.println("");
  #endif
}

void encoderInterruptInit()
{
  DDRE &= ~(1 << PE5);    //Channel Z
  EICRB |= (1 << ISC51) | (1 << ISC50);    //rising egde
  EIMSK |= (1 << INT5);

  DDRE &= ~(1 << PE6);    //Channel A
  EICRB |= (1 << ISC61) | (1 << ISC60);   //rising egde
  EIMSK |= (1 << INT6);
  DDRE &= ~(1 << PE7);    //Channel B
  sei();
}

void baseMotorsInit()
{
  DDRC |= 0xFF;
  PORTC |= 0xFF;
  pwm1_init();
  pwm3_init();
  BASEBRAKE_DIR = 1;   //For Relay braking of Base
  baseUnbrake();    //Active Low
}

void baseBrake()
{
  BASEBRAKE = 0;    //braking the base motor
  _delay_ms(5);
}

void baseUnbrake()
{
  BASEBRAKE = 1;    //unbraking the base motor
  _delay_ms(5);
}

void throwingMotorInit()
{
  pwm1_init();
  MOTORFR_DIR = 1;
  MOTORBR_DIR = 1;    //Throwing Motor
  THROWBRAKE_DIR = 1;    //For Relay braking of Throwing
  throwUnbrake();   //unbraking the throwing motor
}

void throwBrake()
{
  THROWBRAKE = 0;    //braking the throwing motor
  _delay_ms(5);
}

void throwUnbrake()
{ 
  THROWBRAKE = 1;    //unbraking the throwing motor
  _delay_ms(5);
}

void clampPistonInit()
{
  OPENCLAMP_DIR = 1;   //Clamp Piston
  OPENCLAMP = 0;
}

void piInterfaceInit()
{
  TZ1_DIR = 0;
  TZ2_DIR = 0;
  TZ3_DIR = 0;
  
  TZ1_PORT = 1;
  TZ2_PORT = 1;
  TZ3_PORT = 1;
}

void readSensorFront()    //front dekha toh left wala 0
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

void readSensorBack()   //front dekha toh left wala 0
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

void readSensorRight()   //left dekha toh left wala 0
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

void readSensorLeft()   //left dekha toh left wala 0
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

void calcErrorFront()
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

void calcErrorBack()
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

void calcErrorAndRotation()
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
  if(pidAlign == 1)
  {
    kPError = alignKPError; kDError = alignKDError; kPRotation = alignKPRotation; kDRotation = alignKDRotation;
    opt = alignOpt;
    maxPWM = alignMaxPWM;
  }
  else if(pidAlign == 0)
  {
    kPError = fastKPError; kDError = fastKDError; kPRotation = fastKPRotation; kDRotation = fastKDRotation;
    opt = fastOpt;
    maxPWM = fastMaxPWM;
  }
  else if(pidAlign == 2)
  {
    kPError = alignKPError; kDError = alignKDError; kPRotation = sensitiveRot; kDRotation = alignKDRotation;
    opt = alignOpt;
    maxPWM = slowAlignPWM;
  }
  else if(pidAlign == 3)
  {
    kPError = fastKPError; kDError = fastKDError; kPRotation = fastKPRotation; kDRotation = fastKDRotation;
    opt = alignOpt;
    maxPWM = mediumAlignPWM;
  }
}

void calcSpeedToMotors()
{
  switch(turnFlag)
  {
    case 0:
    //to left
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
    //to right
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
    //to front
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
  baseUnbrake();   //for unbraking the base motor
  
  if(turnFlag == 0 || turnFlag == 1)
  {
    sFront = map(sFront, 0 - (opt + kPRotation * 7000), (opt + kPRotation * 7000), 0 - maxPWMLine, maxPWMLine);
    sLeft  = map(sLeft,  0 - 10000*kPError, 10000*kPError, 0 - maxPWMLine*0.5, maxPWMLine*0.5);
    sBack  = map(sBack,  0 - (opt + kPRotation * 7000), (opt + kPRotation * 7000), 0 - maxPWMLine, maxPWMLine);
    sRight = map(sRight, 0 - 10000*kPError, 10000*kPError, 0 - maxPWMLine*0.5, maxPWMLine*0.5);  
  }
  else if (turnFlag == 2 || turnFlag == 3)
  {
    sFront = map(sFront, 0 - (10000*kPError+1), (10000*kPError + 1), 0 - maxPWMLine*0.5, maxPWMLine*0.5);
    sLeft = map(sLeft,   0 - (opt + kPRotation * 7000), (opt + kPRotation * 7000), 0 - maxPWMLine, maxPWMLine);   //line*
    sBack = map(sBack,   0 - (10000*kPError + 1), (10000*kPError + 1), 0 - maxPWMLine*0.5, maxPWMLine*0.5);
    sRight = map(sRight, 0 - (opt + kPRotation * 7000), (opt + kPRotation * 7000), 0 - maxPWMLine, maxPWMLine);   //(line*-20) line*-20
  }
  
  if(sFront < -30)
  {
    sFront *= (-1);
    MOTORFF = 0;
    MOTORFB = 1;
    if(sFront > maxPWMLine)
      sFront = maxPWMLine;
    PWMF = sFront;
  }
  else if(sFront > 30)
  {
    MOTORFF = 1;
    MOTORFB = 0;
    if(sFront > maxPWMLine)
      sFront = maxPWMLine;
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

void botAlign(int currentDir)
{
  int alignCount = 0;
  botKill();
  int rcount = 0;
  pidAlign(2);
  if(currentDir != 1)
  {
    while(rcount <= 30)
    {
      rcount++;
      calcAllError();
    }
  }
  switch(currentDir)
  {
    case 0:
    alignCount = 0;
    while(((errorBack + errorFront) > 450) || ((errorBack + errorFront) < -450) || (frontOnline == 0 || backOnline == 0))
    {
      alignCount++ ;
      if((errorBack + errorFront) > 450 || (frontOnline == 0 || backOnline == 0))
      {
        turnFlag = 1;
        while((errorBack + errorFront) > 450)
        {
          calcAllError();
          calcErrorAndRotation();
          calcSpeedToMotors();
          uploadSpeedLineFollow(maxPWM);
        }
      }
      else if(((errorBack + errorFront) < -450) && (frontOnline != 0 && backOnline != 0))
      {
        turnFlag = 0;
        while(((errorBack + errorFront) < -450))
        {
          calcAllError();
          calcErrorAndRotation();
          calcSpeedToMotors();
          uploadSpeedLineFollow(maxPWM);
        }
      }
      botKill();
      _delay_ms(20);
      calcAllError();
      if(alignCount >= 4)
      {
        PORTC |= 0xff;
        _delay_ms(100);
        PORTC = 0x00;
        break;
      }
    }
    break;

    case 1:
    alignCount = 0;
    while(((errorBack+errorFront) > 450) || ((errorBack+errorFront) < -450) || (frontOnline == 0 || backOnline == 0))
    {
      alignCount++ ;
      if((errorBack+errorFront) > 450 && (frontOnline != 0 && backOnline != 0))
      {
        turnFlag = 1;
        while((errorBack+errorFront) > 450)
        {
          calcAllError();
          calcErrorAndRotation();
          calcSpeedToMotors();
          uploadSpeedLineFollow(maxPWM);
        }
      }
      else if(((errorBack+errorFront) < -450) || (frontOnline == 0 || backOnline == 0))
      {
        turnFlag = 0;
        while(((errorBack+errorFront) < -450))
        {
          calcAllError();
          calcErrorAndRotation();
          calcSpeedToMotors();
          uploadSpeedLineFollow(maxPWM);
        }
      }
      botKill();
      _delay_ms(20);
      calcAllError();
      if(alignCount >= 4)
      {
        PORTC |= 0xff;
        _delay_ms(100);
        PORTC = 0x00;
        break;
      }
    }
    break;

    case 2:
    alignCount = 0;
    while(((errorRight+errorLeft) > 450) || ((errorRight+errorLeft) < -450) || (leftOnline == 0 || rightOnline == 0))
    {
      alignCount++ ;
      if((errorRight+errorLeft) > 450 && (leftOnline != 0 && rightOnline != 0))
      {
        turnFlag = 2;
        while((errorRight+errorLeft) > 450)
        {
          calcAllError();
          calcErrorAndRotation();
          calcSpeedToMotors();
          uploadSpeedLineFollow(maxPWM);
        }
      }
      else if(((errorRight+errorLeft) < -450) || (leftOnline == 0 || rightOnline == 0))
      {
        turnFlag = 3;
        while(((errorRight+errorLeft) < -450) || (leftOnline == 0 || rightOnline == 0))
        {
          calcAllError();
          calcErrorAndRotation();
          calcSpeedToMotors();
          uploadSpeedLineFollow(maxPWM);
        }
      }
      botKill();
      _delay_ms(20);
      calcAllError();
      if(alignCount >= 4)
      {
        PORTC |= 0xff;
        _delay_ms(100);
        PORTC = 0x00;
        break;
      }
    }
    break;

    default:break;
  }
  pidAlign(0);
  botKill();
}

void calcAllError()
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
}
void lineFollow()
{
  calcAllError();
  
  if(leftOnline == 0 && rightOnline >= 1 && frontOnline >= 1 && backOnline == 0 && firstTurn == 1)  //the 1st turn
  {
    turnFlag = 2;
    firstTurn = 2;
  }

  if((leftOnline == 0 && firstTurn == 1))                      //slow for first turn
  {
    pidAlign(1);
  }

  if(((leftOnline >= 6 && turnFlag == 0) || (rightOnline >= 6 && turnFlag == 1)) && firstTurn == 0)    ///short junc counter
  {
    if(shortPlusFlag)
    {
      if(turnFlag == 1)    //Rightwards linefollow
        shortPlus--;
      else
        shortPlus++;
      shortPlusFlag = false;
    }
  }
  else
  {
    shortPlusFlag = true;
  }

  if(frontOnline >= 1 && backOnline >= 1  &&  leftOnline >= 1 && rightOnline >= 1 && firstTurn==0)   //throw junction counter
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

  shortJuncCheck();       //shortPlus
  juncCheck();              //Junction
  calcErrorAndRotation();
  calcSpeedToMotors();
  uploadSpeedLineFollow(maxPWM);
}

void shortJuncCheck()
{
  if(turnFlag == 0)
  {
    if((rongBay == 0 && shortPlus == 1) || (rongBay == 1 && shortPlus == 3))    //1 4
    {
      pidAlign(3);
    }
    
    else if((rongBay == 0 && shortPlus == 2) || (rongBay == 1 && shortPlus == 5))   //2 5
    {
      pidAlign(1);
    }
  }
  else if(turnFlag == 1)
  { 
    if(shortPlus == 0)
    {
      pidAlign(1);
    }
//    else if(shortPlus == 1)
//    {
//      pidAlign(3);
//    }
  }
  else if(turnFlag == 2)
  {
    if(frontOnline >= 5)
    {
      if(frontPlusFlag)
      {
        frontPlusFlag = false;
        frontPlus++;
        if(frontPlus == 1 && tz1 == false)
        {
          pidAlign(1);
        }
        else if(frontPlus == 2)
        {
          pidAlign(1);
        }
      }
    }
    else 
    {
      frontPlusFlag = true;
    }
  }
}

void juncCheck()
{ 
  if((juncCount == 1 && turnFlag == 2 && (!tz1)) || (juncCount == 1 && turnFlag == 1))    //transferFlag junction for TZ1
  {
    shortPlus = 0;
    botKill();
    pidAlign(1);
    juncFlag = false;
    botAlign(turnFlag);
    botKill();
    
    if(transferFlag)
    {
      startTransferAfterFailureTZ1:
      openClamp();
      stopAlign = true;   
      alignArm();
      startTransferTZ1:    //label for goto
      
      initTOF();
      
      int sureCount = 0;
      while(sureCount < 15)
      {
        distance = readTOF();
        if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
          sureCount++;
        else
          sureCount = 0;
      }

      closeClamp();

      int unsureCount = 0;
      sureCount = 0;
      while(unsureCount < 25)
      {
        distance = readTOF();
        if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
          sureCount++;
        else
          sureCount = 0;
        unsureCount++;
      }
      if(sureCount >= 25)
      {
        openClamp();
        goto startTransferTZ1;
      }
      
      throwUnbrake();   //unbraking the throwing motor
      backwardRotation();   //to give backwards direction to the motor
      PWMT = map(20, 0, 255, 0, 666);
      while(ticks > -80)               //////////////*******************************************************************************
      {
        PORTC = 0x0F;
      }
      PORTC = 0x00;
      stopRotation();
      PWMT = 0;
      
      /*sureCount = 0;
      while(sureCount < 30)
      {
        distance = readTOF();
        if(distance > maxDistanceThreshold + 50)
          sureCount++;
        else
          sureCount = 0;
      }*/
     
      PORTA |= (1 << PA1);    //LED STRIP OFF
      int colorCount = 0;
      while(colorCount < 250)
      {
        if(!TZ2)
          colorCount++;
        else if(!TZ1)
          colorCount++;
        else if(!TZ3)
          colorCount++;
        else{
          distance = readTOF();
          if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
          {
            openClamp();
            alignAfterFailure();
            goto startTransferTZ1;
          }
          colorCount = 0;
          
        }
      }
      PORTA &= ~(1 << PA1);   //LED STRIP OFF
      
      throwBrake();   //for braking the throwing motor
      
      if(!TZ1)
      {
        tz1 = false;    //TZ1
        turnFlag = 0;
      }
      else if(!TZ2)
      {
        tz1 = true;    //TZ2
        turnFlag = 2;
      }
      else if(!TZ3)
      {
        rongBay = true;   //TZ3
        tz1 = true;
        turnFlag = 2;
      }
    }
    else
    {
      turnFlag = 0;
    }
    pidAlign(0);
  }
  else if(juncCount == 2 && turnFlag == 0 && (!tz1))    //throwing junction of TZ1
  {
    botKill();
    juncFlag = false;
    botAlign(turnFlag);
    botKill();
    
    throwAtTZ(1);
    transferFlag = true;
    turnFlag = 1;
    pidAlign(0);
  }
  else if((juncCount == 2 && turnFlag == 2) || (juncCount == 3 && turnFlag == 1))        //transfer zone for TZ2
  {
    juncCount = 3;
    shortPlus = 0;
    botKill();
    botAlign(turnFlag);
    juncFlag = false;
    pidAlign(1);
    if(!firstTimeTZ2)
    {
      
      botKill();
      
      pidAlign(1);
      juncFlag = false;
      startTransferAfterFailureTZ2:                                     
      openClamp();
      stopAlign = true;   
      alignArm();
 
      startTransferTZ2:    //label for goto

      initTOF();
      
      int sureCount = 0;
      while(sureCount < 15)
      {
        distance = readTOF();
        if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
          sureCount++;
        else
          sureCount = 0;
      }
  
      closeClamp();
  
      int unsureCount = 0;
      sureCount = 0;
      
      while(unsureCount < 25)
      {
        distance = readTOF();
        if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
          sureCount++;
        else
          sureCount = 0;
        unsureCount++;
      }
      PORTA &= ~(1 << PA0);

      if(sureCount >= 25)
      {
        openClamp();
        goto startTransferTZ2;
      }

      throwUnbrake();   //unbraking the throwing motor
      backwardRotation();   //to give backwards direction to the motor
      PWMT = map(20, 0, 255, 0, 666);
      while(ticks > -80)                  //////////////*********************************************************************************************
      {
        PORTC = 0x0F;
      }
      PORTC = 0x00;
      stopRotation();
      PWMT = 0;
      
      /*sureCount = 0;
      while(sureCount < 30)
      {
        distance = readTOF();
        if(distance > maxDistanceThreshold + 50)
          sureCount++;
        else
          sureCount = 0;
      }*/

      PORTA |= (1 << PA1);    //LED STRIP OFF
      int colorCount = 0;
      while(colorCount < 250)
      {
        if(!TZ2)
          colorCount++;
        else if(!TZ1)
          colorCount++;
        else if(!TZ3)
          colorCount++;
        else{
          distance = readTOF();
          if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
          {
            openClamp();
            alignAfterFailure();
            goto startTransferTZ2;
          }
          colorCount = 0;
        }
      }
      PORTA &= ~(1 << PA1);   //LED STRIP OFF

      throwBrake();   //for braking the throwing motor
      
      if(!TZ2 || !TZ1)
        rongBay = false;    //TZ2
      else if(!TZ3)
        rongBay = true;   //TZ3
    }
    firstTimeTZ2 = false;
    turnFlag = 0;
    pidAlign(0);
  }
  else if(juncCount == 4 && turnFlag == 0)                                            //throwing zone for TZ2
  {
    if(!rongBay)
    {
      botKill();
      juncFlag = false;
      botAlign(turnFlag);
      
      throwAtTZ(2);
      transferFlag = true;
      turnFlag = 1;
      pidAlign(0);
    }
  }
  else if(juncCount == 5 && turnFlag == 0)                                              //throwing zone for TZ3
  {
    botKill();
    juncFlag = false;
    botAlign(turnFlag);
    
    throwAtTZ(3);
    transferFlag = true;
    turnFlag = 1;
    pidAlign(0);  
  }
}

int adjust()
{
  if(ticks > 0)
  {
    if(ticks > 180)
      pidTicks = (ticks - 360);
    else
      pidTicks = ticks;
  }
  else
  {
    if(ticks < -180)
      pidTicks = (ticks + 360);
    else
      pidTicks = ticks;
  }
  return pidTicks;
}

void alignAfterFailure()
{
  throwUnbrake();   //for unbraking the throwing motor

  armError = adjust();
  if(armError > 0)
  {
    backwardRotation();
  }
  else if(armError <= 0) 
  {
    forwardRotation();
  }
  PWMT = map(60, 0, 255, 0, 666);

  throwCount = 0;
  while(throwCount == 0)
  {
    #ifdef debug
    Serial.print("ticks: ");Serial.print(ticks);
    Serial.print(" throwCount: ");Serial.print(throwCount);
    Serial.println("");
    #endif
  }
  PWMT = 0;
  stopRotation(); 
  
  int pidCount = 100;
  while(pidCount != 0)
  {
    armError = adjust();
    while(armError < -4 || 1 < armError)
    {
      pidCount = 100;
      #ifdef debug
      Serial.println(ticks);
      #endif
      throwUnbrake();   //for unbraking the throwing motor
      armError = adjust();
      
      diffError = armError - prevArmError;
      armPID = abs(armKp * armError + armKd * diffError);
      
      armPWM = map(armPID, 8, 400, 30, 80);
      armPWM = constrain(armPWM, 30, 80);
      
      if(armError > 0)
      {
        backwardRotation();
      }
      else if(armError < 0) 
      {
        forwardRotation();
      }

      PWMT = map(armPWM, 0, 255, 0, 666);
      prevArmError = armError;
    }
    PWMT = 0;
    stopRotation();
    throwBrake();   //for braking the throwing motor
    #ifdef debug
    Serial.print(" pidCount = ");Serial.println(pidCount);
    #endif
    pidCount--;
  }
}

void alignArm()
{
  if(!aligned)    //only when arm is not aligned
  {
    if(firstTimeAlign)    //only till Z is not triggered 
    {
      if(alignTemp)   //only to give directions once
      {
        alignTemp = false;
        throwCount = 0;
        throwUnbrake();   //for unbraking the throwing motor

        armError = adjust();
        if(armError > 0)
        {
          backwardRotation();
        }
        else if(armError <= 0)
        {
          forwardRotation();
        }
        PWMT = map(60, 0, 255, 0, 666);
      }
      
      if(throwCount > 0)   //rotate till Z
      {
        PWMT = 0;
        stopRotation();
        alignTemp = true;
        firstTimeAlign = false;
        pidCount = 100;
        #ifdef debug
//        Serial.print(" throwCount: ");Serial.print(throwCount);
//        Serial.println("");
        #endif
      }
    }
    
    else
    {
      if(!stopAlign)
      {
        armError = adjust();
        if(armError < -4 || 1 < armError)
        {
          throwUnbrake();   //for unbraking the throwing motor
          armError = adjust();
          
          diffError = armError - prevArmError;
          armPID = abs(armKp * armError + armKd * diffError);
          
          armPWM = map(armPID, 8, 400, 30, 80);
          armPWM = constrain(armPWM, 30, 80);
          
          if(armError > 0)
          {
            backwardRotation();
          }
          else if(armError < 0) 
          {
            forwardRotation();
          }
          
          PWMT = map(armPWM, 0, 255, 0, 666);
          prevArmError = armError;
        }
        
        else
        {
          PWMT = 0;
          stopRotation();
          throwBrake();   //for braking the throwing motor
        }
      }
      else
      {
        pidCount = 50;
        while(pidCount != 0)
        {
          armError = adjust();
          while(armError < -4 || 1 < armError)
          {
            pidCount = 50;
            throwUnbrake();   //for unbraking the throwing motor
            armError = adjust();
            
            diffError = armError - prevArmError;
            armPID = abs(armKp * armError + armKd * diffError);
            
            armPWM = map(armPID, 8, 400, 30, 80);
            armPWM = constrain(armPWM, 30, 80);
            
            if(armError > 0)
            {
              backwardRotation();
            }
            else if(armError < 0) 
            {
              forwardRotation();
            }
      
            PWMT = map(armPWM, 0, 255, 0, 666);
            prevArmError = armError;
          }
          PWMT = 0;
          stopRotation();
          throwBrake();   //for braking the throwing motor
          pidCount--;
        }
        
        stopAlign = false;
        firstTimeAlign = true;
        aligned = true;     //to Stop Aligning
      }
    }
  }
}

void throwAtTZ(int zone)
{
  throwCount = 0;
  throwUnbrake();   //unbraking the throwing motor
  
  if(zone == 1)
  {
    backwardRotation();   //to give backwards direction to the motor
    PWMT = map(throwPWM1, 0, 255, 0, 666);
    
    while(1)
    {
      #ifdef debug
      Serial.print("ticks:\t");
      Serial.print(ticks);  
      Serial.println(" ");
      #endif

      if(throwCount == 3)
        PORTA |= (1 << PA0);    //LED ON
      else
        PORTA &= ~(1 << PA0);    //LED OFF
      
      if(ticks <= TZ1Ticks && throwCount >= TZ1ThrowCount)
      {
        #ifdef debug
        Serial.print("finalticks : ");
        Serial.println(ticks);
        #endif

        openClamp();
        break;
      }
    }
  }
  
  else if(zone == 2)
  {
    backwardRotation();   //to give backwards direction to the motor
    PWMT = map(throwPWM2, 0, 255, 0, 666);
    
    while(1)
    {
      #ifdef debug
      Serial.print("ticks:\t");
      Serial.print(ticks);  
      Serial.println(" ");
      #endif
      
      if(throwCount == 3)
        PORTA |= (1 << PA0);    //LED ON
      else
        PORTA &= ~(1 << PA0);    //LED OFF
      
      if(ticks <= TZ2Ticks && throwCount >= TZ2ThrowCount)
      {
        #ifdef debug
        Serial.print("finalticks : ");
        Serial.println(ticks);
        #endif
        
        openClamp();
        break;
      }
    }
  }

  else if(zone == 3)
  {
    _delay_ms(2000);
    backwardRotation();   //to give backwards direction to the motor
    PWMT = map(throwPWM3, 0, 255, 0, 666);
    
    while(1)
    {
      #ifdef debug
      Serial.print("ticks:\t");
      Serial.print(ticks);  
      Serial.println(" ");
      #endif

      if(throwCount == 3)
        PORTA |= (1 << PA0);    //LED ON
      else
        PORTA &= ~(1 << PA0);    //LED OFF
      
      if(ticks <= TZ3Ticks && throwCount >= TZ3ThrowCount)
      {
        #ifdef debug
        Serial.print("finalticks : ");
        Serial.println(ticks);
        #endif

        openClamp();
        break;
      }
    }
  }
  throwCount = 0;
  PORTA &= ~(1 << PA0);   //LED OFF
  _delay_ms(100);
  stopRotation();
  PWMT = 0;
  throwBrake();   //for braking the throwing motor
  throws++;
  aligned = false;
}

void closeClamp()
{
  OPENCLAMP = 0;
}

void openClamp()
{
  OPENCLAMP = 1;
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

  baseBrake();   //braking the base motor
}

void initTOF()
{
  int32_t temp = 0;
  
  Wire.begin();
  
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();

  while(1)
  {
    temp = sensor.readRangeContinuousMillimeters();
    if(temp == 65535)
    {
      PORTC = 0xF0;
      _delay_ms(200);
      PORTC = 0x0F;
      _delay_ms(200);

      Wire.begin();
  
      sensor.init();
      sensor.setTimeout(500);
      sensor.startContinuous();
    }
    
    else
    {
      PORTC = 0x00;
      break;
    }
  }
}

int32_t readTOF()
{
  return sensor.readRangeContinuousMillimeters(); 
}

ISR(INT6_vect)
{
  if(bit_is_set(PINE,7))
  {
    ticks--;
  }
  else if(bit_is_clear(PINE,7))
  {
    ticks++;
  }
}

ISR(INT5_vect)
{
  ticks = 0;
  throwCount++;
}


