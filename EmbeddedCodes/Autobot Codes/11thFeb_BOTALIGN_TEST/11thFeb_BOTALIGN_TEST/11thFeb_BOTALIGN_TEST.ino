/******************
changes on 10th feb
1. botAlign New function added
2. pidAlign Special case added
3. calcAllError calculates error for all sensor
4. bot slows down at previous shortplus
************/
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

bool firstTimeTZ2 = true;

bool firstTimeAlign = true;
int armError = 0, prevArmError = 0, integralError = 0, diffError = 0;
int armKp = 3, armKi = 0, armKd = 8;

int armPID = 0, armPWM = 0;
int pidCount = 0;

int throwingPWM = 0;

int pidMode = 1, toggle = 0;  //for pid tuning

#define numOfSensors 8

float alignKPError = 0.20, alignKDError = 0, alignKPRotation = 0.32, alignKDRotation = 0;
int alignOpt = 1500;

float fastKPError = 0.42, fastKDError = 0, fastKPRotation = 0.25, fastKDRotation = 0;
int fastOpt = 2500;

int frontOnline = 0, backOnline = 0, leftOnline = 0, rightOnline = 0;

//calibrated values of 10th Feb
int sensorMinLeft[numOfSensors] = {1272, 790, 972, 1084, 1363, 627, 714, 1480};
int sensorMaxLeft[numOfSensors] = {2959, 2780, 2976, 3018, 3040, 1581, 2804, 2946};
int sensorMinRight[numOfSensors] = {638, 665, 904, 679, 934, 635, 1147, 1097};
int sensorMaxRight[numOfSensors] = {2792, 2903, 2996, 2841, 2953, 2784, 2975, 2925};
int sensorMinFront[numOfSensors] = {1532, 1081, 640, 702, 1011, 713, 1209, 1827};
int sensorMaxFront[numOfSensors] = {3040, 2974, 2805, 2858, 2940, 2836, 3031, 3038};
int sensorMinBack[numOfSensors] = {1109, 569, 596, 560, 593, 576, 602, 1339};
int sensorMaxBack[numOfSensors] = {3052, 2385, 2763, 2203, 2763, 2500, 2751, 3033};

int maxPWM = 400, alignMaxPWM = 575, fastMaxPWM = 650;  //600 - 650 

int errorRight = 0, errorLeft = 0, errorFront = 0, errorBack = 0, error = 0, prevError = 0;          
int rotation = 0, prevRotation = 0;

int sensorRawRight[numOfSensors], sensorValRight[numOfSensors], sensorRawLeft[numOfSensors], sensorValLeft[numOfSensors];
int sensorRawFront[numOfSensors], sensorValFront[numOfSensors], sensorRawBack[numOfSensors], sensorValBack[numOfSensors];

int turnFlag = -1;  //0-Left  1-Right  2-Forward  3-Back
int firstTurn = -1;   //0-after 1st turn and 1-before                                  

float kPError = fastKPError, kDError = fastKDError, kPRotation = fastKPRotation, kDRotation = fastKDRotation;
int opt = fastOpt;

int juncCount = 0, shortPlus = 0;
bool juncFlag = true, shortPlusFlag = true;

int sLeft, sRight, sFront, sBack;
int slowAlignPWM = 400;
int32_t distance = 0, minDistanceThreshold = 220, maxDistanceThreshold = 370;   //for VL53L0X (TOF)

int throws = 0;   //to count the no of throws

int TZ1Ticks = -221, TZ2Ticks = -219, TZ3Ticks = -208;

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

  DDRA |= (1 << PA0);   //for LED Flashing
  PORTA &= ~(1 << PA0);   //Active High

  initTOF();

  PORTC = 0xFF;
  _delay_ms(1000);
  PORTC = 0x00;
}

void loop()
{
  if(bit_is_clear(PINA,7))    //Full Run
  {
    PORTC = 0xFF;
    _delay_ms(1000);
    PORTC = 0x00;
    turnFlag = 0;
    firstTurn = 1;
    while(1)
    {
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
      alignArm();
      _delay_ms(500);
      openClamp();
      while(bit_is_clear(PINA,4));
      closeClamp();
      _delay_ms(500);
      while(bit_is_clear(PIND,4));
      throwUnbrake();   //unbraking the throwing motor
      backwardRotation();   //to give backwards direction to the motor
      PWMT = map(50, 0, 255, 0, 666);
      PORTC = 0xFF;
      while(ticks > -100);    //_delay_ms(200);
      PORTC = 0x00;
      stopRotation();
      PWMT = 0;
      throwBrake();   //for braking the throwing motor
    }
  }
  
  else
  {
    botKill();
  }
}

void SerialPrint()
{
  #ifdef debug
  Serial.print("  shortPlus = ");Serial.print(shortPlus);
  Serial.print("  shortPlusFlag = ");Serial.print(shortPlusFlag);
  Serial.print("  rongBay = ");Serial.print(rongBay);
  
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
    kPError = alignKPError; kDError = alignKDError; kPRotation = alignKPRotation; kDRotation = alignKDRotation;
    opt = alignOpt;
    maxPWM = slowAlignPWM;
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
  while(rcount <= 50)
  {
    rcount++; 
    calcAllError();
  }
  switch(currentDir)
  {
    case 0:
    alignCount = 0;
    while(((errorBack+errorFront) > 500) || ((errorBack+errorFront) < -500) || (frontOnline == 0 || backOnline == 0))
    {
      alignCount++ ;
      if((errorBack+errorFront) > 500 || (frontOnline == 0 || backOnline == 0))
      {
        turnFlag = 1;
        while((errorBack+errorFront) > 500)
        {
          calcAllError();
          calcErrorAndRotation();
          calcSpeedToMotors();
          uploadSpeedLineFollow(maxPWM);
        }
      }
      else if(((errorBack+errorFront) < -500) && (frontOnline != 0 && backOnline != 0))
      {
        turnFlag = 0;
        while(((errorBack+errorFront) < -500))
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
      if(alignCount >= 3)
      {
        PORTC |= 0xff;
        _delay_ms(500);
        PORTC = 0x00;
        break;
      }
    }
    break;

    case 2:
    alignCount = 0;
    while(((errorRight+errorLeft) > 500) || ((errorRight+errorLeft) < -500) || (leftOnline == 0 || rightOnline == 0))
    {
      alignCount++ ;
      if((errorRight+errorLeft) > 500 && (leftOnline != 0 && rightOnline != 0))
      {
        turnFlag = 2;
        while((errorRight+errorLeft) > 500)
        {
          calcAllError();
          calcErrorAndRotation();
          calcSpeedToMotors();
          uploadSpeedLineFollow(maxPWM);
        }
      }
      else if(((errorRight+errorLeft) < -500) || (leftOnline == 0 || rightOnline == 0))
      {
        turnFlag = 3;
        while(((errorRight+errorLeft) < -500) || (leftOnline == 0 || rightOnline == 0))
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
      if(alignCount >= 3)
      {
        PORTC |= 0xff;
        _delay_ms(500);
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
  
  if(leftOnline == 0 && rightOnline >= 1 && frontOnline >= 1 && backOnline == 0 && firstTurn == 1)  //the 1st turn
  {
    turnFlag = 2;
    firstTurn = 2;
  }

  if((leftOnline == 0 && firstTurn == 1) || (frontOnline >= 5 && turnFlag == 2))                      //slow for first turn
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

void shortJuncCheck()
{
  if(turnFlag == 0)
  {
    if((rongBay == 0 && shortPlus == 1) || (rongBay == 1 && shortPlus == 4))
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
  }
}

void juncCheck()
{
  if((juncCount == 1 && turnFlag == 2 && (!tz1)) || (juncCount == 1 && turnFlag == 1))    //tranfer junction for TZ1
  {
    shortPlus = 0;
    botKill();
    _delay_ms(1000);    //till botAlign is not used
    pidAlign(1);
    juncFlag = false;
    botAlign(turnFlag);
    botKill();
    openClamp();
    alignArm();
    
    startTransferTZ1:    //label for goto
    int sureCount = 0;
    while(sureCount < 15)
    {
      distance = readTOF();
      if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
        sureCount++;
      else
        sureCount = 0;
    }

    PORTC = 0xFF;
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
    while(ticks > -70)                  //////////////*******************************
    {
      PORTC = 0x0F;
    }
    PORTC = 0x00;
    stopRotation();
    PWMT = 0;
    throwBrake();   //for braking the throwing motor

    /*sureCount = 0;
    while(sureCount < 30)
    {
      distance = readTOF();
      if(distance > maxDistanceThreshold + 50)
        sureCount++;
      else
        sureCount = 0;
    }*/
  
    int colorCount = 0;
    while(colorCount < 500)
    {
      if(!TZ1)
        colorCount++;
      else if(!TZ2)
        colorCount++;
      else
        colorCount = 0;
    }
    if(!TZ1)
    {
      tz1 = false;    //TZ1
      turnFlag = 0;   //TZ1
    }
      
    else if(!TZ2)
    {
      tz1 = true;    //TZ2
      turnFlag = 2;   //TZ2
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
    
    turnFlag = 1;
    pidAlign(0);
  }
  else if((juncCount == 2 && turnFlag == 2) || (juncCount == 3 && turnFlag == 1))        //transfer zone for TZ2
  {
    juncCount = 3;
    shortPlus = 0;
    botKill();
    if(turnFlag == 2)
    {
    botAlign(turnFlag);
    }
    juncFlag = false;
    pidAlign(1);
      
    if(!firstTimeTZ2)
    {
      botKill();
      
      pidAlign(1);
      juncFlag = false;

      openClamp();
      alignArm();
 
      startTransferTZ2:    //label for goto
      
      int sureCount = 0;
      while(sureCount < 15)
      {
        distance = readTOF();
        if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
          sureCount++;
        else
          sureCount = 0;
      }
  
      PORTC = 0xFF;
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
        goto startTransferTZ2;
      }

      throwUnbrake();   //unbraking the throwing motor
      backwardRotation();   //to give backwards direction to the motor
      PWMT = map(20, 0, 255, 0, 666);
      while(ticks > -70)                  //////////////*******************************
      {
        PORTC = 0x0F;
      }
      PORTC = 0x00;
      stopRotation();
      PWMT = 0;
      throwBrake();   //for braking the throwing motor
      
      /*sureCount = 0;
      while(sureCount < 30)
      {
        distance = readTOF();
        if(distance > maxDistanceThreshold + 50)
          sureCount++;
        else
          sureCount = 0;
      }*/
      
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
      
      /*int rcount = 0;
      while(rcount <= 50)
      {
        rcount++; 
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
      botAlign();
      */
      
      throwAtTZ(2);
      
      turnFlag = 1;
      pidAlign(0);
    }
  }
  else if(juncCount == 5 && turnFlag == 0)                                              //throwing zone for TZ3
  {
    botKill();
    juncFlag = false;
    botAlign(turnFlag);
    /*int rcount = 0;
    while(rcount <= 50)
    {
      rcount++; 
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
    botAlign();
    */
    PORTC = 0xFF;
    _delay_ms(50);
    PORTC = 0x00;
    _delay_ms(50);

    PORTC = 0xFF;
    _delay_ms(50);
    PORTC = 0x00;
    _delay_ms(50);

    PORTC = 0xFF;
    _delay_ms(50);
    PORTC = 0x00;
    _delay_ms(50);
    
    throwAtTZ(3);
    
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

void alignArm()
{
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
  PWMT = map(50, 0, 255, 0, 666);
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
    while(armError <= -2 || 2 <= armError)
    {
      pidCount = 100;
      #ifdef debug
      Serial.println(ticks);
      #endif
      throwUnbrake();   //for unbraking the throwing motor
      armError = adjust();
      
      diffError = armError - prevArmError;
      armPID = abs(armKp * armError + armKd * diffError);
      
      armPWM = map(armPID, 8, 400, 28, 80);
      armPWM = constrain(armPWM, 25, 80);
      
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

void throwAtTZ(int zone)
{
  throwCount = 0;
  throwUnbrake();   //unbraking the throwing motor
  
  if(zone == 1)
  {
    backwardRotation();   //to give backwards direction to the motor
    PWMT = map(215, 0, 255, 0, 666);
    
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
      
      if(ticks <= TZ1Ticks && throwCount >= 4)
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
    PWMT = map(215, 0, 255, 0, 666);
    
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
      
      if(ticks <= TZ2Ticks && throwCount >= 4)
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
    backwardRotation();   //to give backwards direction to the motor
    PWMT = map(195, 0, 255, 0, 666);
    
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
      
      if(ticks <= TZ3Ticks && throwCount >= 4)   //-218 4
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
}

void initTOF()
{ 
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();
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

