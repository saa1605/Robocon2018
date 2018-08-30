#include <io128.h>
#include <sra128.h>
#include "avr/io.h"
#include "avr/interrupt.h"
#include <pinDefsAuto.h>
#include "mcp.h"
#include <math.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

bool tz1 = false;   //false - TZ1 and true - TZ2
bool rongBay = false;   //false - TZ2 and true - TZ3

volatile long int ticks = 0;
volatile int pidTicks = 0, throwCount = 0;

bool firstTimeTZ2 = true;
int armError = 0, prevArmError = 0, integralError = 0, diffError = 0;
int armKp = 2, armKi = 0, armKd = 8;

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

//calibrated values of 6th Feb Special
int sensorMinLeft[numOfSensors] = {2887, 2170, 1822, 1499, 1224, 977, 1694, 2133};
int sensorMaxLeft[numOfSensors] = {3072, 3072, 3074, 3073, 3073, 2982, 3137, 3137};
int sensorMinRight[numOfSensors] = {2413, 1371, 1516, 638, 621, 869, 623, 1461};
int sensorMaxRight[numOfSensors] = {3072, 2970, 3070, 2770, 2752, 2903, 1881, 3056};
int sensorMinFront[numOfSensors] = {1946, 1273, 868, 892, 599, 600, 613, 1717};
int sensorMaxFront[numOfSensors] = {3202, 3117, 2985, 3000, 1664, 2785, 2756, 3113};
int sensorMinBack[numOfSensors] = {1829, 613, 883, 973, 623, 569, 1394, 1200};
int sensorMaxBack[numOfSensors] = {3075, 2646, 2873, 2873, 2403, 2262, 3072, 2925};

int maxPWM = 400, alignMaxPWM = 575, fastMaxPWM = 650;  //525 - 650 

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

int decimal = 0;   //for aruco correction

int sLeft, sRight, sFront, sBack;

int32_t distance = 0, minDistanceThreshold = 220, maxDistanceThreshold = 370;   //for VL53L0X

int throws = 0;   //to count the no of throws

// Main loop, read and display data
//-----------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  encoderInterruptInit();
  baseMotorsInit();
  throwingMotorInit();
  clampPistonInit();
  piInterfaceInit();
  spiMasterInit();
 
  DDRD &= ~(1 << PD3);  //just for practise
  PORTD |= (1 << PD3);  //just for practise
  pidAlign(0);  //fast line following in the begining
  PORTD &= ~(1 << PD6); //Throwing Motor brake

  initVL53L0X();

  PORTC = 0xFF;
  _delay_ms(1000);
  PORTC = 0x00;
}

void loop()
{
  if(bit_is_clear(PIND,3))
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
  else
  {
    botKill();
  }
}

void SerialPrint()
{ 
// Serial.print("  leftOnline = ");Serial.print(leftOnline);
// Serial.print("  rightOnline = ");Serial.print(rightOnline);
// Serial.print("  frontOnline = ");Serial.print(frontOnline);
// Serial.print("  backOnline = ");Serial.print(backOnline);
  
  Serial.print("  shortPlus = ");Serial.print(shortPlus);
  Serial.print("  shortPlusFlag = ");Serial.print(shortPlusFlag);
  Serial.print("  rongBay = ");Serial.print(rongBay);
  
   Serial.println("");
}

void encoderInterruptInit()
{
  DDRD &= ~(1 << PD2);    //Channel Z
  EICRA |= (1 << ISC21) | (1 << ISC20);    //rising egde
  EIMSK |= (1 << INT2);

  DDRE &= ~(1 << PE6);    //Channel A
  EICRB |= (1 << ISC61) | (1 << ISC60);   //rising egde
  EIMSK |= (1 << INT6);
  DDRA &= ~(1 << PA2);    //Channel B
  sei();
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
  DDRA |= (1 << PA0)|(1 << PA1); //Throwing Motor
  DDRD |= (1 << PD6);  //For Relay braking of Throwing
  PORTD |= (1 << PD6); //Active Low
}

void clampPistonInit()
{
  DDRE |= (1 << PE7);  //Clamp Piston
}

void piInterfaceInit()
{
  DDRF &= ~((1 << PF0) | (1 << PF1) | (1 << PF2));    //0 : TZ1, 1 : TZ2, 2 : TZ3
  PORTF |= (1 << PF0)|(1 << PF1)|(1 << PF2);
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
  sensorValRight[0] = 0;    //fuck this later **********************************************************************************************************************
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
  sensorValLeft[0] = 0;    //fuck this later **********************************************************************************************************************
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

void botAlign()
{
  opt = 0;
  kPRotation = 1.25;
  kPError = 1.25;
  bool breakOut = false;
  while(abs(errorFront)>250 || abs(errorBack)>250 || abs(errorLeft)>250 || abs(errorRight)>250)
  {
    int rc = 0, fc = 0;
    while(abs(errorLeft) >250 || abs(errorRight) > 250)
    {
      rc+=1;
      kPRotation = 1.25;
      kPError = 1.25;
      turnFlag = 0;
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
      Serial.print(sFront);Serial.print(' ');
      Serial.print(sBack);Serial.print(' ');
      Serial.print(sLeft);Serial.print(' ');
      Serial.print(sRight);Serial.print(' ');
      Serial.print('\t');
      kPRotation = 1;
      kPError = 1;
      sFront = constrain(sFront, -4500, 4500);
      sBack = constrain(sBack, -4500, 4500);
      sLeft = constrain(sLeft, -4500, 4500);
      sRight = constrain(sRight, -4500, 4500);
      
      uploadSpeedLineFollow(maxPWM/0.5);
      SerialPrint();
      if(rc>=50)
      {
        breakOut = true;
        break;
      }
    }
    while(abs(errorFront) > 250 || abs(errorBack) > 250)
    {
      fc+=1;
      kPRotation = 1.25;
      kPError = 1.25;
      turnFlag = 2;
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
      Serial.print(sFront);Serial.print(' ');
      Serial.print(sBack);Serial.print(' ');
      Serial.print(sLeft);Serial.print(' ');
      Serial.print(sRight);Serial.print(' ');
      Serial.print('\t');
      kPRotation = 1;
      kPError = 1;
      sLeft = constrain(sLeft, -4500, 4500);
      sRight = constrain(sRight, -4500, 4500);
      if(sFront >0)
        sFront = constrain(sFront, 1000, 4500);
      else
        sFront = constrain(sFront, -4500, 1000);
      if(sBack >0)
        sBack = constrain(sBack, 1000, 4500);
      else
        sBack = constrain(sBack, -4500, 1000);
        
      uploadSpeedLineFollow(maxPWM/0.5);
      SerialPrint();
      if(fc>=50)
      {
        breakOut = true;
        break;
      }
    }
    botKill();
    _delay_ms(50);
    if(breakOut)
    {
      break;
    }
  }
  
  botKill();
  pidAlign(0);
  
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
  Serial.println("close");
  MOTOROC = 0;
}
void openClamp()
{
  Serial.println("open");
  MOTOROC = 1;
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

void shortJuncCheck()
{
  if(turnFlag == 0)
  {
    if((rongBay == 0 && shortPlus == 2) || (rongBay == 1 && shortPlus == 5))
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
    _delay_ms(1000);
    pidAlign(1);
    juncFlag = false;
    
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
    botAlign();*/
    if(turnFlag == 2)
    {
      turnFlag = 3;
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
    }
    botKill();
    openClamp();
    alignArm();
    
    startTransferTZ1:    //label for goto
    int sureCount = 0;
    while(sureCount < 15)
    {
      distance = readVL53L0X();
      if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
        sureCount++;
      else
        sureCount = 0;
    }

    PORTC = 0xFF;
    closeClamp();

    int unsureCount = 0;
    sureCount = 0;
    while(unsureCount < 30)
    {
      distance = readVL53L0X();
      if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
        sureCount++;
      else
        sureCount = 0;
      unsureCount++;
    }
    if(sureCount >= 30)
    {
      openClamp();
      goto startTransferTZ1;
    }

    PORTD |= (1 << PD6);    //unbraking the throwing motor
    backwardRotation();   //to give backwards direction to the motor
    OCR0 = 30;
    _delay_ms(200);
    stopRotation();
    OCR0 = 0;
    
    /*sureCount = 0;
    while(sureCount < 30)
    {
      distance = readVL53L0X();
      if(distance > maxDistanceThreshold + 50)
        sureCount++;
      else
        sureCount = 0;
    }*/
  
    int colorCount = 0;
    while(colorCount < 500)
    {
      if(bit_is_clear(PINF,0))
        colorCount++;
      else if(bit_is_clear(PINF,1))
        colorCount++;
      else
        colorCount = 0;
    }
    if(bit_is_clear(PINF,0))
      tz1 = false;    //TZ1
    else if(bit_is_clear(PINF,1))
      tz1 = true;    //TZ2
    
    if(!(tz1))
      turnFlag = 0;   //TZ1
    else
      turnFlag = 2;   //TZ2
    
    pidAlign(0);
  }
  else if(juncCount == 2 && turnFlag == 0 && (!tz1))    //throwing junction of TZ1
  {
    botKill();
    juncFlag = false;
    
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
    _delay_ms(100);
    PORTC = 0x00;
    _delay_ms(100);

    PORTC = 0xFF;
    _delay_ms(100);
    PORTC = 0x00;
    _delay_ms(100);

    PORTC = 0xFF;
    _delay_ms(100);
    PORTC = 0x00;
    _delay_ms(100);
    
    throwAtTZ(1);
    _delay_ms(500);
    turnFlag = 1;
    pidAlign(0);
  }
  else if((juncCount == 2 && turnFlag == 2) || (juncCount == 3 && turnFlag == 1))        //transfer zone for TZ2
  {
    juncCount = 3;
    shortPlus = 0;
    botKill();
    _delay_ms(1000);
    pidAlign(1);
    juncFlag = false;
    
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
    botAlign();*/
    if(turnFlag == 2)
    {
      turnFlag = 3;
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
    }
    botKill();
    if(!firstTimeTZ2)
    {
      openClamp();
      alignArm();
 
      startTransferTZ2:    //label for goto
      int sureCount = 0;
      while(sureCount < 15)
      {
        distance = readVL53L0X();
        if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
          sureCount++;
        else
          sureCount = 0;
      }
  
      PORTC = 0xFF;
      closeClamp();
  
      int unsureCount = 0;
      sureCount = 0;
      while(unsureCount < 30)
      {
        distance = readVL53L0X();
        if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
          sureCount++;
        else
          sureCount = 0;
        unsureCount++;
      }
      if(sureCount >= 30)
      {
        openClamp();
        goto startTransferTZ2;
      }

      PORTD |= (1 << PD6);    //unbraking the throwing motor
      backwardRotation();   //to give backwards direction to the motor
      OCR0 = 30;
      _delay_ms(200);
      stopRotation();
      OCR0 = 0;
      
      /*sureCount = 0;
      while(sureCount < 30)
      {
        distance = readVL53L0X();
        if(distance > maxDistanceThreshold + 50)
          sureCount++;
        else
          sureCount = 0;
      }*/
      
      int colorCount = 0;
      while(colorCount < 500)
      {
        if(bit_is_clear(PINF,1))
          colorCount++;
        else if(bit_is_clear(PINF,2))
          colorCount++;
        else
          colorCount = 0;
      }
      
      if(bit_is_clear(PINF,1))
        rongBay = false;    //TZ2
      else if(bit_is_clear(PINF,2))
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
      _delay_ms(100);
      PORTC = 0x00;
      _delay_ms(100);
  
      PORTC = 0xFF;
      _delay_ms(100);
      PORTC = 0x00;
      _delay_ms(100);
  
      PORTC = 0xFF;
      _delay_ms(100);
      PORTC = 0x00;
      _delay_ms(100);
      
      throwAtTZ(2);
      _delay_ms(500);
      turnFlag = 1;
      pidAlign(0);
    }
  }
  else if(juncCount == 5 && turnFlag == 0)                                              //throwing zone for TZ3
  {
    botKill();
    juncFlag = false;
    
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
    _delay_ms(100);
    PORTC = 0x00;
    _delay_ms(100);

    PORTC = 0xFF;
    _delay_ms(100);
    PORTC = 0x00;
    _delay_ms(100);

    PORTC = 0xFF;
    _delay_ms(100);
    PORTC = 0x00;
    _delay_ms(100);
    
    throwAtTZ(3);
    _delay_ms(500);
    turnFlag = 1;
    pidAlign(0);  
  }
}

void alignArm()
{
  throwCount = 0;
  PORTD |= (1 << PD6);    //for unbraking the throwing motor
  forwardRotation();
  OCR0 = 40;
  while(throwCount == 0)
  {
    Serial.print("ticks: ");Serial.print(ticks);
    Serial.print(" throwCount: ");Serial.print(throwCount);
    Serial.println("");
  }
  OCR0 = 0;
  stopRotation();
  int pidCount = 1000;//500
  while(pidCount != 0)
  {
    armError = ticks;
    while(armError < 0 || 2 < armError)
    {
      Serial.println(ticks);
      PORTD |= (1 << PD6); //for unbraking the throwing motor
      armError = ticks;
  
      integralError += armError;
      diffError = armError - prevArmError;
      integralError = constrain(integralError, 0, 300);
      armPID = abs(armKp * armError + armKi * integralError + armKd * diffError);
  
      armPWM = map(armPID, 8, 400, 25, 50);
      armPWM = constrain(armPWM, 25, 50);
      
      if(armError > 0)
      {
        backwardRotation();
      }
      else if(armError < 0) 
      {
        forwardRotation();
      }
  
      OCR0 = armPWM;
      prevArmError = armError;
    }
    OCR0 = 0;
    stopRotation();
    PORTD &= ~(1 << PD6); //for braking the throwing motor
    Serial.print(" pidCount = ");Serial.println(pidCount);
    pidCount--;
  }
}

void throwAtTZ(int zone)
{
  throwCount = 0;
  PORTD |= (1 << PD6);    //unbrake the throwing motor
  
  if(zone == 1)
  {
    forwardRotation();    //to give forward direction to the motor
    throwingPWM = 220;
    OCR0 = throwingPWM;
    
    while(bit_is_set(PIND,6))
    {
      Serial.print("ticks:\t");
      Serial.print(ticks);  
      Serial.println(" ");
      
      if(ticks >= 282 && throwCount >= 3)   //==158 3 
      {
        Serial.print("finalticks : ");
        Serial.println(ticks);

        throwCount = 0;
        openClamp();
        _delay_ms(100);
        PORTC |= 0xFF;
        stopRotation();
        throwingPWM = 0;
        PORTD &= ~(1 << PD6);   //brake the throwing motor
      }
    }
  }

  else if(zone == 2)
  {
    forwardRotation();    //to give forward direction to the motor
    throwingPWM = 220;
    OCR0 = throwingPWM;
    
    while(bit_is_set(PIND,6))
    {
      Serial.print("ticks:\t");
      Serial.print(ticks);  
      Serial.println(" ");
      
      if((ticks >= 278) && throwCount >= 3)   //161 3
      {
        Serial.print("finalticks : ");
        Serial.println(ticks);
        
        throwCount = 0;
        openClamp();
        _delay_ms(100);
        PORTC |= 0xFF;
        stopRotation();
        throwingPWM = 0;
        PORTD &= ~(1 << PD6);   //brake the throwing motor
      }
    }
  }

  else if(zone == 3)
  {
    backwardRotation();   //to give backwards direction to the motor
    throwingPWM = 215;
    OCR0 = throwingPWM;
    
    while(bit_is_set(PIND,6))
    {
      Serial.print("ticks:\t");
      Serial.print(ticks);  
      Serial.println(" ");
      
      if(ticks <= -218 && throwCount >= 4)   //
      {
        Serial.print("finalticks : ");
        Serial.println(ticks);

        throwCount = 0;
        openClamp();
        _delay_ms(100);
        PORTC |= 0xFF;
        stopRotation();
        throwingPWM = 0;
        PORTD &= ~(1 << PD6);   //brake the throwing motor
      }
    }
  }
  throws++;
}

void initVL53L0X()
{ 
  Wire.begin();

  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();
}

int32_t readVL53L0X()
{
  return sensor.readRangeContinuousMillimeters();
}

ISR(INT6_vect)
{
  if(bit_is_set(PINA,2))
  {
    ticks--;
  }
  else if(bit_is_clear(PINA,2))
  {
    ticks++;
  }
}

ISR(INT2_vect)
{
  ticks = 0;
  throwCount++;
}
