#include <io128.h>
#include <sra128.h>
#include <psx.h>
#include <psx2.h>
#include <pinDefsAuto.h>
#include "mcp.h"

int pidMode = 1, toggle = 0;

//float currentJunction = 5;///////
//float desiredJunction = currentJunction;
//int junctionOnLeft = 0, junctionOnRight = 0;////////
//unsigned long MPUIters = 0;**********
//int shootingMotorLevelSpeed[4] = {0, 135, 260, 550};
//int shootingMotorLevel = 0;
//float shootingMotorRPM = 0;
//int shootingMotorReadTime = 0, shootingMotorReadTimePrev = 0;
//volatile long shootingMotorTicks = 0;*******
int initialErrorCorrectionPWM = 200;

#define numOfSensors 8
//#define numOfJunctionSensors 2/////
//#define numOfExtraSwitches 4///////

float normalKPError = 0, normalKDError = 0, normalKPRotation = 0, normalKDRotation = 0;
int normalOpt = 600;

float fastKPError = 0, fastKDError = 0, fastKPRotation = 0, fastKDRotation = 0;
int fastOpt = 5000;
int frontOnline = 0, backOnline = 0, leftOnline = 0, rightOnline = 0;    /////////////////////////////
//float normalKPError = 0.24, normalKDError = 0.04, normalKPRotation = 0.096, normalKDRotation = 0;
//int normalOpt = 600;
//
//float fastKPError = 0.26, fastKDError = 0.05, fastKPRotation = 0.4, fastKDRotation = 0;
//int fastOpt = 9500;

//int sensorMinLeft[numOfSensors]  = {890, 850, 970, 760, 1080, 950, 860, 650};
//int sensorMaxLeft[numOfSensors]  = {1920, 2590, 2530, 2350, 2870, 2870, 2500, 2280};
//int sensorMinRight[numOfSensors] = {630, 890, 630, 800, 1000, 1000, 920, 920};
//int sensorMaxRight[numOfSensors] = {2750, 2600, 2840, 2520, 2830, 2830, 2800, 2000};
int sensorMinLeft[numOfSensors]  = {4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000};
int sensorMaxLeft[numOfSensors]  = {0, 0, 0, 0, 0, 0, 0, 0};
int sensorMinRight[numOfSensors] = {4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000};
int sensorMaxRight[numOfSensors] = {0, 0, 0, 0, 0, 0, 0, 0};
//front and back
int sensorMinFront[numOfSensors]  = {4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000};     /////////////////
int sensorMaxFront[numOfSensors]  = {0, 0, 0, 0, 0, 0, 0, 0};                        //////////////////
int sensorMinBack[numOfSensors] = {4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000};      ////////////////
int sensorMaxBack[numOfSensors] = {0, 0, 0, 0, 0, 0, 0, 0};                             /////////////
//int sensorRawJunction[numOfJunctionSensors];/////
//int sensorValJunction[numOfJunctionSensors];
//int sensorThresholdJunction[numOfJunctionSensors] = {450, 900};
//int extraSwitches[numOfExtraSwitches];///////
//int sensorMinLeft[numOfSensors]  = {220, 210, 190, 200, 200, 190, 210, 210};
//int sensorMaxLeft[numOfSensors]  = {2760, 2720, 2760, 2760, 2030, 2170, 2550, 2420};
//int sensorMinRight[numOfSensors] = {220, 210, 250, 250, 210, 210, 250, 200};
//int sensorMaxRight[numOfSensors] = {2650, 2360, 2660, 2530, 2340, 2390, 2660, 2230};

#define maxPWM 666
//float kPError = 0.0069, kDError = 0.0, kPRotation = 0.096, kDRotation = 0;

//unsigned long stTime = 0;

#define joystickBuffer 25
#define maxJoystick (128 - joystickBuffer)
#define diagonalJoystickBuffer 35

int errorRight = 0, errorLeft = 0, errorFront = 0, errorBack = 0, error = 0, prevError = 0;          ////////////////////////
int rotation = 0, prevRotation = 0;
//int sensorMinJunction[numOfJunctionSensors] = {220, 210, 250, 250};
//int sensorMaxJunction[numOfJunctionSensors] = {2650, 2360, 2660, 2530};
int sensorRawRight[numOfSensors], sensorValRight[numOfSensors], sensorRawLeft[numOfSensors], sensorValLeft[numOfSensors];
int sensorRawFront[numOfSensors], sensorValFront[numOfSensors], sensorRawBack[numOfSensors], sensorValBack[numOfSensors];
int turnFlag = 2;
bool firstTurn = 1;                                   //////////////////////////////////////////////
//float kPError = normalKPError, kDError = normalKDError, kPRotation = normalKPRotation, kDRotation = normalKDRotation;
//int opt = normalOpt;
float kPError = fastKPError, kDError = fastKDError, kPRotation = fastKPRotation, kDRotation = fastKDRotation;
int opt = fastOpt;
int setRot = 0, setErr = 0;

//int discToBeLoaded = 0, discDetected = 0, discShot = 0;******
//int shootingMotorVelocity = 0;********

int sLeft, sRight, sFront, sBack;
//int shootingMotorState = 0;*******
//int shootingMotorPWM = 0;
//int alphaUpPWM = 666, alphaPWM = alphaUpPWM, alphaDownPWM = 600;
//int loadingMotorPWM = 500;
//int servoLeftAngle = 45, servoCenterAngle = 90, servoRightAngle = 135;
//int servoLoadSide = 0; // 0 - left Side to be loaded next // 1 - right Side to be loaded next********** 
int pwm = 666, spotPWM = 200;

int LX = 0, LY = 0, RX = 0, RY = 0;
// Main loop, read and display data
//-----------------------------------------------------------------------

int junc_count = 0, df = 0;
void setup()
{
  Serial.begin(115200);
  Serial.println("Setup Started");
  psx_init(&PORTG,0,&PORTG,3,&PORTG,1,&PORTG,2);
  baseMotorsInitialize();
  spiMasterInit();
  Serial.println("Setup Over");
}

void loop()
{
  if(ReadController())
  {
    if(!lineFollow())
      baseMotorsMotion();

    serialPrint();
  }
  else
  {
    botKill();
    Serial.println("Controller Disconnected");
  }
}

int ReadController()
{
  psx_read_gamepad();
  LX = (psx_stick(PSS_LX)) - 127;
  LY = 127 - (psx_stick(PSS_LY));
  RX = (psx_stick(PSS_RX)) - 127;
  RY = 127 - (psx_stick(PSS_RY));
  if(LX == 128 && LY == (-128) && RX == 128 && RY == (-128))
  {
    botKill();
    return (0);
  }
  return (1);
}

void serialPrint()
{
  Serial.print(toggle);Serial.print(' ');
  Serial.print(pidMode);Serial.print(' ');
  Serial.print(kPRotation);Serial.print(' ');
  Serial.print(kDRotation);Serial.print('\t');
  Serial.print(kPError);Serial.print(' ');
  Serial.print(kDError);Serial.print(' ');
  
  Serial.print('\t');
  Serial.print(sFront);Serial.print(' ');
  Serial.print(sBack);Serial.print(' ');
  Serial.print(sLeft);Serial.print(' ');
  Serial.print(sRight);Serial.print(' ');
  
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorRawFront[i]);Serial.print(' ');
  }
  Serial.print("\t"); 
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorRawBack[i]);Serial.print(' ');
  }
  Serial.print("\t");
//  for(int i = 0; i < numOfSensors; i++)
//  {
//    Serial.print(sensorValFront[i]);Serial.print(" ");
//  }
//  Serial.print("\t");
//  for(int i = 0; i < numOfSensors; i++)
//  {
//    Serial.print(sensorValBack[i]);Serial.print(" ");
//  }
//  Serial.print("\t");
  
  Serial.print("  errLeft = ");
  Serial.print(errorLeft);
  Serial.print(" errRight ");
  Serial.print(errorRight);
  Serial.print("  Err = ");
  Serial.print(error);
  Serial.print("  Rot = ");
  Serial.print(rotation);
//  Serial.print("  crntJnctn = ");
//  Serial.print(currentJunction);
//  Serial.print("  dsrdJnctn = ");
//  Serial.print(desiredJunction);
//  Serial.print("  jnctnOnLt = ");
//  Serial.print(junctionOnLeft);
//  Serial.print("  jnctnOnRt = ");
//  Serial.print(junctionOnRight);
  
  Serial.println("");
}

void baseMotorsInitialize()
{
  DDRC |= 0xFF;
  PORTC |= 0xFF;
  pwm0_init();
  pwm1_init();
}

void baseMotorsMotion()
{
  sFront = 0;
  sLeft = 0;
  sBack = 0;
  sRight = 0;
  
  if (LX > joystickBuffer || LX < 0 - joystickBuffer)
  {
    if (LX > 0)
    {
      sFront = map(LX, joystickBuffer, 127, 0, pwm);
      sBack = map(LX, joystickBuffer, 127, 0, pwm);
    }
    else if (LX < 0)
    {
      sFront = 0 - map(LX, -127, 0 - joystickBuffer, pwm, 0);
      sBack = 0 - map(LX, -127, 0 - joystickBuffer, pwm, 0);
    }
  }

  else if (RY > joystickBuffer || RY < 0 - joystickBuffer)
  {
    if(RY > 0)
    {
      sLeft = map(RY, joystickBuffer, 127, 0, pwm);
      sRight = map(RY, joystickBuffer, 127, 0, pwm);
    }
    else if(RY < 0)
    {
      sLeft = 0 - map(RY, -127, 0 - joystickBuffer, pwm, 0);
      sRight = 0 - map(RY, -127, 0 - joystickBuffer, pwm, 0);
    }
  }

  if(psx_button_click(PSB_R1,flag_R1))
  {
    if(pidMode == 1)
    {
      if(!toggle)
        kPRotation = 0;
      else
        kPError  = 0;
    }
    else if(pidMode == 2)
    {
      if(!toggle)
        kDRotation = 0;
      else
        kDError  = 0;
    }
  }
  
  if(psx_button_click(PSB_L1,flag_L1))
  {
    if(pidMode == 1)
    {
      if(!toggle)
        kPRotation += 0.02;
      else
        kPError  += 0.02;
    }
    else if(pidMode == 2)
    {
      if(!toggle)
        kDRotation += 0.01;
      else
        kDError  += 0.01;
    }
  }

  if(psx_button_click(PSB_L2,flag_L2))
  {
    if(pidMode == 1)
    {
      if(!toggle)
        kPRotation -= 0.02;
      else
        kPError  -= 0.02;
    }
    else if(pidMode == 2)
    {
      if(!toggle)
        kDRotation -= 0.01;
      else
        kDError  -= 0.01;
    }
  }
    
  if(psx_button_click(PSB_PAD_UP,flag_PAD_UP))
    toggle = 0;

  if(psx_button_click(PSB_PAD_DOWN,flag_PAD_DOWN))
    toggle = 1;

  if(psx_button_click(PSB_PAD_LEFT,flag_PAD_LEFT))
    pidMode = 1;

  if(psx_button_click(PSB_PAD_RIGHT,flag_PAD_RIGHT))
    pidMode = 2;

  if(psx_button_click(PSB_R2,flag_R2))
  {
    //calibrateRight();
    //calibrateLeft();
  }

  if(sFront < -30)
  {
    sFront *= (-1);
    MOTORFF = 0;
    MOTORFB = 1;
    if(sFront > 666)
      sFront = 666;
    PWMF = map(sFront, 0, 666, 0, 255);
//    PWMF = sFront;
  }
  else if(sFront > 30)
  {
    MOTORFF = 1;
    MOTORFB = 0;
    if(sFront > 666)
      sFront = 666;
    PWMF = map(sFront, 0, 666, 0, 255);
//    PWMF = sFront;
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
    if(sBack > 666)
      sBack = 666;
    PWMB = sBack;
  }
  else if(sBack > 30)
  {
    MOTORBF = 1;
    MOTORBB = 0;
    if(sBack > 666)
      sBack = 666;
    PWMB = sBack;
  }
  else
  {
    MOTORBF = 1;
    MOTORBB = 1;
    if(sBack > 666)
      sBack = 666;
    PWMB = 666;
  }

  if(sLeft < -30)
  {
    sLeft *= (-1);
    MOTORLF = 0;
    MOTORLB = 1;
    if(sLeft > 666)
      sLeft = 666;
    PWML = sLeft;
  }
  else if(sLeft > 30)
  {
    MOTORLF = 1;
    MOTORLB = 0;
    if(sLeft > 666)
      sLeft = 666;
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
    if(sRight > 666)
      sRight = 666;
    PWMR = sRight;
  }
  else if(sRight > 30)
  {
    MOTORRF = 1;
    MOTORRB = 0;
    if(sRight > 666)
      sRight = 666;
    PWMR = sRight;
  }
  else
  {
    MOTORRF = 1;
    MOTORRB = 1;
    PWMR = 666;
  }
}
////Pin Defs for for front and back sensor
void readSensorFront()
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
void readSensorBack()
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
//void readSensorRight()
//{
//  sensorRawRight[0] = getSensorReading(rightSensor, 7);
//  sensorRawRight[1] = getSensorReading(rightSensor, 6);
//  sensorRawRight[2] = getSensorReading(rightSensor, 5);
//  sensorRawRight[3] = getSensorReading(rightSensor, 4);
//  sensorRawRight[4] = getSensorReading(rightSensor, 3);
//  sensorRawRight[5] = getSensorReading(rightSensor, 2);
//  sensorRawRight[6] = getSensorReading(rightSensor, 1);
//  sensorRawRight[7] = getSensorReading(rightSensor, 0);
//}
//void readSensorLeft()
//{
//  sensorRawLeft[0] = getSensorReading(leftSensor, 0);
//  sensorRawLeft[1] = getSensorReading(leftSensor, 1);
//  sensorRawLeft[2] = getSensorReading(leftSensor, 2);
//  sensorRawLeft[3] = getSensorReading(leftSensor, 3);
//  sensorRawLeft[4] = getSensorReading(leftSensor, 4);
//  sensorRawLeft[5] = getSensorReading(leftSensor, 5);
//  sensorRawLeft[6] = getSensorReading(leftSensor, 6);
//  sensorRawLeft[7] = getSensorReading(leftSensor, 7);
//} 

//void readJunctionSensorsAndSwitches()//////////
//{
//  spi_init(SENSORSANDSWITCHESCS);
//  extraSwitches[3]     = spi_dataMCP3208(SENSORSANDSWITCHESCS, 7);
//  extraSwitches[2]     = spi_dataMCP3208(SENSORSANDSWITCHESCS, 6);
//  extraSwitches[1]     = spi_dataMCP3208(SENSORSANDSWITCHESCS, 5);
//  extraSwitches[0]     = spi_dataMCP3208(SENSORSANDSWITCHESCS, 4);
//  sensorRawJunction[3] = spi_dataMCP3208(SENSORSANDSWITCHESCS, 3);
//  sensorRawJunction[2] = spi_dataMCP3208(SENSORSANDSWITCHESCS, 2);
//  sensorRawJunction[1] = spi_dataMCP3208(SENSORSANDSWITCHESCS, 1);
//  sensorRawJunction[0] = spi_dataMCP3208(SENSORSANDSWITCHESCS, 0);
//}////////////

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


//void calcSensorValRight()
//{
//  for (int j = 0; j < numOfSensors; j++)
//  {
//    sensorValRight[j] = map(sensorRawRight[j], sensorMinRight[j], sensorMaxRight[j], 0, 1000);
//    if (sensorValRight[j] < 0)
//      sensorValRight[j] = 0;
//    else if (sensorValRight[j] > 1000)
//      sensorValRight[j] = 1000;
//  }
//}
//
//void calcSensorValLeft()
//{
//  for (int j = 0; j < numOfSensors; j++)
//  {
//    sensorValLeft[j] = map(sensorRawLeft[j], sensorMinLeft[j], sensorMaxLeft[j], 0, 1000);
//    if (sensorValLeft[j] < 0)
//      sensorValLeft[j] = 0;
//    else if (sensorValLeft[j] > 1000)
//      sensorValLeft[j] = 1000;
//  }
//  sensorValLeft[7] = 0;
//}
void calibrateFront()
{
  Serial.println("Calibrating Front");
  unsigned int time = 2000;
  for(int i = 0; i < time; i++)
  {
    readSensorFront();
    for(int j = 0; j < numOfSensors; j++)
    {
      if(sensorMinFront[j] > sensorRawFront[j] && sensorRawFront[i] != 0)
        sensorMinFront[j] = sensorRawFront[j];
      else if(sensorMaxFront[j] < sensorRawFront[j])
        sensorMaxFront[j] = sensorRawFront[j];
    }
  }
  for(int j = 0; j < numOfSensors; j++)
  {
    sensorMinFront[j] += 1;
    sensorMaxFront[j] -= 1;
  }
  Serial.println("Front Calibrated");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(i);
    Serial.print(". ");
    Serial.print(sensorMinFront[i]);
    Serial.print('\t');
    Serial.print(sensorMaxFront[i]);
    Serial.print('\t');
  }
  Serial.print('\n');
  _delay_ms(500);
}
void calibrateBack()
{
  Serial.println("Calibrating Back");
  unsigned int time = 2000;
  for(int i = 0; i < time; i++)
  {
    readSensorBack();
    for(int j = 0; j < numOfSensors; j++)
    {
      if(sensorMinBack[j] > sensorRawBack[j] && sensorRawBack[i] != 0)
        sensorMinBack[j] = sensorRawBack[j];
      else if(sensorMaxBack[j] < sensorRawBack[j])
        sensorMaxBack[j] = sensorRawBack[j];
    }
  }
  for(int j = 0; j < numOfSensors; j++)
  {
    sensorMinBack[j] += 1;
    sensorMaxBack[j] -= 1;
  }
  Serial.println("Back Calibrated");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(i);
    Serial.print(". ");
    Serial.print(sensorMinBack[i]);
    Serial.print('\t');
    Serial.print(sensorMaxBack[i]);
    Serial.print('\t');
  }
  Serial.print('\n');
  _delay_ms(500);
}

// void calibrateRight()
// {
//   Serial.println("Calibrating Right");
//   unsigned int time = 2000;
//   for(int i = 0; i < time; i++)
//   {
//     readSensorRight();
//     for(int j = 0; j < numOfSensors; j++)
//     {
//       if(sensorMinRight[j] > sensorRawRight[j] && sensorRawRight[i] != 0)
//         sensorMinRight[j] = sensorRawRight[j];
//       else if(sensorMaxRight[j] < sensorRawRight[j])
//         sensorMaxRight[j] = sensorRawRight[j];
//     }
//   }
//   for(int j = 0; j < numOfSensors; j++)
//   {
//     sensorMinRight[j] += 1;
//     sensorMaxRight[j] -= 1;
//   }
//   Serial.println("Right Calibrated");
//   for(int i = 0; i < numOfSensors; i++)
//   {
//     Serial.print(i);
//     Serial.print(". ");
//     Serial.print(sensorMinRight[i]);
//     Serial.print('\t');
//     Serial.print(sensorMaxRight[i]);
//     Serial.print('\t');
//   }
//   Serial.print('\n');
//   _delay_ms(500);
// }

// void calibrateLeft()
// {
//   Serial.println("Calibrating Left");
//   unsigned int time = 2000;
//   for(int i = 0; i < time; i++)
//   {
//     readSensorLeft();
//     for(int j = 0; j < numOfSensors; j++)
//     {
//       if(sensorMinLeft[j] > sensorRawLeft[j] && sensorRawLeft[i] != 0)
//         sensorMinLeft[j] = sensorRawLeft[j];
//       else if(sensorMaxLeft[j] < sensorRawLeft[j])
//         sensorMaxLeft[j] = sensorRawLeft[j];
//     }
//   }
//   for(int j = 0; j < numOfSensors; j++)
//   {
//     sensorMinLeft[j] += 1;
//     sensorMaxLeft[j] -= 1;
//   }
//   Serial.println("Left Calibrated");
//   for(int i = 0; i < numOfSensors; i++)
//   {
//     Serial.print(i);
//     Serial.print(". ");
//     Serial.print(sensorMinLeft[i]);
//     Serial.print('\t');
//     Serial.print(sensorMaxLeft[i]);
//     Serial.print('\t');
//   }
//   Serial.print('\n');
//   _delay_ms(500);
// }

void calcErrorFront()
{
  int onLine = 0;
  unsigned long avg = 0, sum = 0, pos = (numOfSensors - 1) * 1000.0;
  for (int j = 0; j < numOfSensors; j++)
  {
    if (sensorValFront[j] > 400)
      frontOnline++;
    if (sensorValFront[j] > 150)
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
      pos = 0;												// If it last read to the left of center, return 0.
  }
  errorFront = pos - (numOfSensors - 1) * 500.0;
}
void calcErrorBack()
{
  int onLine = 0;
  unsigned long avg = 0, sum = 0, pos = (numOfSensors - 1) * 1000.0;
  for (int j = 0; j < numOfSensors; j++)
  {
    if (sensorValBack[j] > 400)
      backOnline++;
    if (sensorValBack[j] > 150)
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
      pos = 0;												// If it last read to the left of center, return 0.
  }
  errorBack = pos - (numOfSensors - 1) * 500.0;
}

// void calcErrorRight()
// {
//   int onLine = 0;
//   unsigned long avg = 0, sum = 0, pos = (numOfSensors - 1) * 1000.0;
//   for (int j = 0; j < numOfSensors; j++)
//   {
//     if (sensorValRight[j] > 400)
//       rightOnline++;
//     if (sensorValRight[j] > 150)
//     {
//       avg += (long)(sensorValRight[j]) * ((j) * 1000);
//       sum += sensorValRight[j];
//     }
//   }
//   if (sum != 0)
//   {
//     pos = avg / sum;
//   }

//   if(rightOnline == 0)
//   { 
//     if(errorRight > 0)
//       pos = (numOfSensors - 1) * 1000.0;                    // If it last read to the right of center, return the max.
//     else
//       pos = 0;												// If it last read to the left of center, return 0.
//   }
//   errorRight = pos - (numOfSensors - 1) * 500.0;
// }
// void calcErrorLeft()
// {
//   int onLine = 0;
//   unsigned long avg = 0, sum = 0, pos = (numOfSensors - 1) * 1000.0;
//   for (int j = 0; j < numOfSensors; j++)
//   {
//     if (sensorValLeft[j] > 400)
//       leftOnline++;
//     if (sensorValLeft[j] > 150)
//     {
//       avg += (long)(sensorValLeft[j]) * ((j) * 1000);
//       sum += sensorValLeft[j];
//     }
//   }
//   if (sum != 0)
//   {
//     pos = avg / sum;
//   }

//   if(leftOnline == 0)
//   {
//     if(errorLeft > 0)
//       pos = (numOfSensors - 1) * 1000.0;                     // If it last read to the right of center, return the max.
//     else
//       pos = 0;               								// If it last read to the left of center, return 0.
//   }
//   errorLeft = pos - (numOfSensors - 1) * 500.0;
// }

void calcErrorAndRotation()  //error is negative when bot is behind the line  //rot is CCW +ve
{
	if(turnFlag == 0 || turnFlag == 1)
	{
	  error = (errorRight + errorLeft)*(10000.0/7000.0) - setErr;
	  rotation = (errorRight - errorLeft) - setRot;
	}
	else if(turnFlag == 2 || turnFlag == 3)
	{
	  error = (errorFront + errorBack)*(10000.0/7000.0) - setErr;
	  rotation = (errorFront - errorBack) - setRot;
	}
}

void calcSpeedToMotors()
{
	switch(turnFlag)
	{
		case 0:
		///to left
		sFront = 0 - (opt - kPRotation * rotation + kDRotation * (rotation - prevRotation));
	  sLeft  =     (kPError * error - kDError * (error - prevError));
	  sBack  = 0 - (opt + kPRotation * rotation - kDRotation * (rotation - prevRotation));
	  sRight =     (kPError*error - kDError * (error - prevError));

	  if(sFront > 0)
	    sFront = 0;

	  if(sBack > 0)
	    sBack = 0;

	  prevError = error;
	  prevRotation = rotation;
		break;

		case 1:
		///to right
	  sFront =     (opt - kPRotation * rotation - kDRotation * (rotation - prevRotation));
	  sLeft  = 0 - (kPError * error - kDError * (error - prevError));  
	  sBack  =     (opt + kPRotation * rotation + kDRotation * (rotation - prevRotation));
	  sRight = 0 - (kPError * error - kDError * (error - prevError));

	  if(sFront < 0)
	    sFront = 0;

	  if(sBack < 0)
	    sBack = 0;

	  prevError = error;
	  prevRotation = rotation;
		break;

		case 2:
		///to front
	  sFront  = (kPError * error - kDError * (error - prevError));  
		sLeft   = 0 - (opt - kPRotation * rotation - kDRotation * (rotation - prevRotation));
	  sBack   = (kPError * error - kDError * (error - prevError));
	  sRight  = 0 - (opt + kPRotation * rotation + kDRotation * (rotation - prevRotation));

		case 3:
	  sFront  = 0 - (kPError * error - kDError * (error - prevError));  
		sLeft   = (opt - kPRotation * rotation - kDRotation * (rotation - prevRotation));
	  sBack   = 0 - (kPError * error - kDError * (error - prevError));
	  sRight  = (opt + kPRotation * rotation + kDRotation * (rotation - prevRotation));
		////to back

		break;

	}
	

  
}

void calcSpeedToRight()
{
}

void uploadSpeedLineFollow(int maxPWMLine)
{
  sFront = map(sFront, 0 - (opt + kPRotation * 7000), (opt + kPRotation * 7000), 0 - maxPWM + 350, maxPWM - 350);
  sLeft  = map(sLeft,  0 - 10000*kPError, 10000*kPError, 0 - maxPWMLine*0.5 + 100, maxPWMLine*0.5 - 100);
  sBack  = map(sBack,  0 - (opt + kPRotation * 7000), (opt + kPRotation * 7000), 0 - maxPWM + 350, maxPWM - 350);
  sRight = map(sRight, 0 - 10000*kPError, 10000*kPError, 0 - maxPWMLine*0.5 + 100, maxPWMLine*0.5 - 100);  

  if(sFront < -30)
  {
    sFront *= (-1);
    MOTORFF = 0;
    MOTORFB = 1;
    if(sFront > maxPWMLine)
      sFront = maxPWMLine;
    PWMF = map(sFront, 0, 666, 0, 255);
//    PWMF = sFront;
  }
  else if(sFront > 30)
  {
    MOTORFF = 1;
    MOTORFB = 0;
    if(sFront > maxPWMLine)
      sFront = maxPWMLine;
    PWMF = map(sFront, 0, 666, 0, 255);
//    PWMF = sFront;
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

int lineFollow()
{
  //readSensorLeft();
  //readSensorRight();
  readSensorFront();
  readSensorBack();

//  readJunctionSensorsAndSwitches();
  //calcSensorValLeft();
  //calcSensorValRight();
  calcSensorValFront();
  calcSensorValBack();

  //calcErrorLeft();
  //calcErrorRight();
  calcErrorFront();
  calcErrorBack();
  // if(leftOnline == 0 && rightOnline >= 2 && frontOnline >= 2 && backOnline == 0 && firstTurn == 1)
  // {
  // 	turnFlag = 2;
  // 	firstTurn = 0;
  // }
  // if(leftOnline >= 2 && rightOnline >= 2 && frontOnline >= 2 && backOnline >= 2 && firstTurn == 0)
  // {
  // 	if(df==0)
  // 	{
  // 		junc_count++;
  // 		df=1;
  // 	}
  // 	////first junction ... transfer
  // }
  // else
  // {
  // 	df = 0;     ///for debouncing
  // }

  // if(junc_count == 1)
  // {
  // 	while(bit_is_clear(PINF,6));
  // 	//closeClamp();
  // 	while(bit_is_set(PINF,6));
  // 	turnFlag == 0;

  // }

  calcErrorAndRotation();

//  calcJunction();////////

  if(psx_button_press(PSB_TRIANGLE))
  {
    calcSpeedToMotors();
    uploadSpeedLineFollow(maxPWM);

    return 1;
  }
  // else if(psx_button_press(PSB_CROSS))
  // {
  //   calcSpeedToRight();
  //   uploadSpeedLineFollow(maxPWM);
  //   return 1;
  // }
  else
    return 0;
}

//void calcJunction()///////////
//{
//  int  errorTemp = error;
//  int  errorRot = rotation;
////  if(1)
//  if(readJunctionSensors())
//  {
//    if(junctionOnLeft == 2)
//    {
//      junctionOnLeft = 0;
//      junctionOnRight = 0;
//      currentJunction--;
//    }
//    else if(junctionOnRight == 2)
//    {
//      junctionOnLeft = 0;
//      junctionOnRight = 0;
//      currentJunction++;
//    }
//    if(junctionOnLeft == 1)
//    {
//      junctionOnLeft = 0;
//      junctionOnRight = 0;
//    }
//    else if(junctionOnRight == 1)
//    {
//      junctionOnLeft = 0;
//      junctionOnRight = 0;
//    }
//  }
//  
//  error = errorTemp;
//  rotation = errorRot;
//}///////////

//int readJunctionSensors()//////////////
//{
//  spi_init(SENSORSANDSWITCHESCS);
//  for(int i = 0; i < numOfJunctionSensors; i++)
//  {
//    sensorRawJunction[i] = spi_dataMCP3208(SENSORSANDSWITCHESCS, i);
//    sensorValJunction[i] = ((sensorRawJunction[i] < sensorThresholdJunction[i])? 0: 1);
//  }
////  for(int i = 0; i < numOfExtraSwitches; i++)
////  {
////    extraSwitches[i+4] = spi_dataMCP3208(SENSORSANDSWITCHESCS, i+4);
////    extraSwitches[i]   = ((extraSwitches[i+4] < 1500)? 1 : 0);
////  }
//  for(int i = 0; i < numOfJunctionSensors; i++)
//  {
//    if(sensorValJunction[i] == 1)
//      return 1;
//  }
//  return 0;
//}////////////////

void botKill()
{
  MOTORFF = 1;
  MOTORFB = 1;
  PWMF = 666;

  MOTORBF = 1;
  MOTORBB = 1;
  PWMB = 666;

  MOTORLF = 1;
  MOTORLB = 1;
  PWML = 666;

  MOTORRF = 1;
  MOTORRB = 1;
  PWMR = 666;
}
