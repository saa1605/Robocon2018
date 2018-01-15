#include <io128.h>
#include <sra128.h>
#include <psx.h>
#include <psx2.h>
#include <pinDefsAuto.h>
#include "mcp.h"
#include <math.h>

int pidMode = 1, toggle = 0;

#define numOfSensors 8

float alignKPError = 0, alignKDError = 0, alignKPRotation = 0, alignKDRotation = 0;
int alignOpt = 600;

float fastKPError = 0.32, fastKDError = 0, fastKPRotation = 0.20, fastKDRotation = 0;
int fastOpt = 2500;

int frontOnline = 0, backOnline = 0, leftOnline = 0, rightOnline = 0;    /////////////////////////////

//float normalKPError = 0.24, normalKDError = 0.04, normalKPRotation = 0.096, normalKDRotation = 0;
//int normalOpt = 600;
//
//float fastKPError = 0.26, fastKDError = 0.05, fastKPRotation = 0.4, fastKDRotation = 0;
//int fastOpt = 9500;

int sensorMinLeft[numOfSensors]  = {4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000};
int sensorMaxLeft[numOfSensors]  = {0, 0, 0, 0, 0, 0, 0, 0};
int sensorMinRight[numOfSensors] = {4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000};
int sensorMaxRight[numOfSensors] = {0, 0, 0, 0, 0, 0, 0, 0};

int sensorMinFront[numOfSensors]  = {4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000};     /////////////////
int sensorMaxFront[numOfSensors]  = {0, 0, 0, 0, 0, 0, 0, 0};                        //////////////////
int sensorMinBack[numOfSensors] = {4000, 4000, 4000, 4000, 4000, 4000, 4000, 4000};      ////////////////
int sensorMaxBack[numOfSensors] = {0, 0, 0, 0, 0, 0, 0, 0};                             /////////////

#define maxPWM 350
//float kPError = 0.0069, kDError = 0.0, kPRotation = 0.096, kDRotation = 0;

#define joystickBuffer 25
#define maxJoystick (128 - joystickBuffer)
#define diagonalJoystickBuffer 35

int errorRight = 0, errorLeft = 0, errorFront = 0, errorBack = 0, error = 0, prevError = 0;          
int rotation = 0, prevRotation = 0;

int sensorRawRight[numOfSensors], sensorValRight[numOfSensors], sensorRawLeft[numOfSensors], sensorValLeft[numOfSensors];
int sensorRawFront[numOfSensors], sensorValFront[numOfSensors], sensorRawBack[numOfSensors], sensorValBack[numOfSensors];

int turnFlag = 0;  //0-Left  1-Right  2-Forward  3-Back
bool firstTurn = 1;   //0-after 1st turn and 1-before                                  

//float kPError = normalKPError, kDError = normalKDError, kPRotation = normalKPRotation, kDRotation = normalKDRotation;
//int opt = normalOpt;
float kPError = fastKPError, kDError = fastKDError, kPRotation = fastKPRotation, kDRotation = fastKDRotation;
int opt = fastOpt;

int junc_count = 0;
bool flag_debounce = true;

int decimal = 0;   //for aruco correction

int sLeft, sRight, sFront, sBack;

int pwm = 666; //For PS2

int LX = 0, LY = 0, RX = 0, RY = 0; //For PS2

bool flap = 1;    

// Main loop, read and display data
//-----------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup Started");
  psx_init(&PORTG,0,&PORTG,3,&PORTG,1,&PORTG,2);
  baseMotorsInitialize();
  clampPistonInit();
  arucoInterfaceInit();
  spiMasterInit();
  openClamp();                      
  PORTD &= ~(1 << PD6); //Throwing Motor brake
  Serial.println("Setup Over");
}

void loop()
{
  if(ReadController())
  {
    if(!lineFollow())
      baseMotorsMotion(); 

//     if(bit_is_set(PINF,0))
//     {
//       closeClamp();
//       flap = 0;
//     }
//     else
//     {
////       if(flap)
//         openClamp();
//     }

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
	Serial.print(turnFlag);Serial.print(' ');
	Serial.print(toggle);Serial.print(' ');
	Serial.print(pidMode);Serial.print(' ');
	Serial.print(kPRotation);Serial.print(' ');
	Serial.print(kDRotation);Serial.print('\t');
	Serial.print(kPError);Serial.print(' ');
	Serial.print(kDError);Serial.print(' ');
  Serial.print('\t');

//  Serial.print(sFront);Serial.print(' ');
//  Serial.print(sBack);Serial.print(' ');
//  Serial.print(sLeft);Serial.print(' ');
//  Serial.print(sRight);Serial.print(' ');
//  Serial.print('\t');

//  for(int i = 0; i < numOfSensors; i++)
//  {
//    Serial.print(sensorValFront[i]);Serial.print(' ');
//  }
//  Serial.print("\t"); 
//  for(int i = 0; i < numOfSensors; i++)
//  {
//    Serial.print(sensorValBack[i]);Serial.print(' ');
//  }
//  Serial.print("\t");  
//  
//  for(int i = 0; i < numOfSensors; i++)
//  {
//    Serial.print(sensorValLeft[i]);Serial.print(' ');
//  }
//  Serial.print("\t"); 
//  for(int i = 0; i < numOfSensors; i++)
//  {
//    Serial.print(sensorValRight[i]);Serial.print(' ');
//  }
//  Serial.print("\t");

   Serial.print("  errBack = ");
   Serial.print(errorBack);
   Serial.print(" errFront ");
   Serial.print(errorFront);
   Serial.print("  Err = ");
   Serial.print(error);
   Serial.print("  Rot = ");
   Serial.print(rotation);
   Serial.print("  firstTurn = ");
   Serial.print(firstTurn);
   Serial.print("  junc_count = ");
   Serial.print(junc_count);
   Serial.print(" sensorValFront[3] ");
   Serial.print(sensorValFront[3]);   
   Serial.print(" sensorValFront[4] ");
   Serial.print(sensorValFront[4]);
   Serial.print("  turnFlag = ");
   Serial.print(turnFlag);
  
   Serial.println("");
}

void baseMotorsInitialize()
{
  DDRC |= 0xFF;
  PORTC |= 0xFF;
  pwm0_init();
  pwm1_init();
  DDRD |= (1 << PD6)|(1 << PD7);  //For Relay braking
  PORTD |= (1 << PD6)|(1 << PD7); //Active Low
}

void clampPistonInit()
{
  DDRE |= (1 << PE6)|(1 << PE7);
}

void arucoInterfaceInit()
{
  DDRF &= ~((1 << PF0)|(1 << PF1)|(1 << PF2)|(1 << PF3));   //PF0 - detection, PF 1,2,3 - Decimal
  PORTF &= ~((1 << PF0)|(1 << PF1)|(1 << PF2)|(1 << PF3));  //PF0 - detection, PF 1,2,3 - Decimal
  DDRD &= ~(1 << PD3);    //for sign of decimal
  PORTD &= ~(1 << PD3);   //set = -ve and clear = +ve
}

void baseMotorsMotion()
{
  PORTD |= (1 << PD7);  //for unbraking base motors
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
        kPRotation += 0.01;
      else
        kPError  += 0.01;
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
        kPRotation -= 0.01;
      else
        kPError  -= 0.01;
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
    calibrateRight();
    calibrateLeft();
    calibrateFront();
    calibrateBack();
  }

  if(sFront < -30)
  {
    sFront *= (-1);
    MOTORFF = 0;
    MOTORFB = 1;
    if(sFront > 666)
      sFront = 666;
    sFront = map(sFront, 0, 666, 0, 255);
    PWMF = sFront;
  }
  else if(sFront > 30)
  {
    MOTORFF = 1;
    MOTORFB = 0;
    if(sFront > 666)
      sFront = 666;
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
  Serial.println("Calibrating Front");
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
}

void calibrateBack()
{
  Serial.println("Calibrating Back");
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
}

void calibrateRight()
{
  Serial.println("Calibrating Right");
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
  Serial.println("Right Calibrated");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(i);
    Serial.print(". ");
    Serial.print(sensorMinRight[i]);
    Serial.print('\t');
    Serial.print(sensorMaxRight[i]);
    Serial.print('\t');
  }
  Serial.print('\n');
}

void calibrateLeft()
{
  Serial.println("Calibrating Left");
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
     for(int j = 0; j < numOfSensors; j++)
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
  Serial.println("Left Calibrated");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(i);
    Serial.print(". ");
    Serial.print(sensorMinLeft[i]);
    Serial.print('\t');
    Serial.print(sensorMaxLeft[i]);
    Serial.print('\t');
  }
  Serial.print('\n');
}

void calcErrorFront() //extreme right 3500 and left -3500
{
  frontOnline = 0;
  unsigned long avg = 0, sum = 0, pos = (numOfSensors - 1) * 1000.0;
  for (int j = 0; j < numOfSensors; j++)
  {
    if (sensorValFront[j] > 600)
      frontOnline++;
    if (sensorValFront[j] > 300)
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
    if(backOnline>=1)
    {
      firstTurn = 0;
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

void pidAlign(int pidAlign)
{
  if(pidAlign)
  {
    kPError = alignKPError; kDError = alignKDError; kPRotation = alignKPRotation; kDRotation = alignKDRotation;
    opt = alignOpt;
  }
  else
  {
    kPError = fastKPError; kDError = fastKDError; kPRotation = fastKPRotation; kDRotation = fastKDRotation;
    opt = fastOpt;
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

int lineFollow()
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
  //(sensorValFront[3] > 400 || sensorValFront[4] > 400)
  if(leftOnline == 0 && rightOnline >= 2 && frontOnline >= 2 && backOnline == 0 && firstTurn == 1)  //the 1st turn
  {
    turnFlag = 2;
    firstTurn = 2;
  }
//(sensorValLeft[2] > 400 || sensorValLeft[3] > 400) && (sensorValRight[2] > 400 || sensorValRight[3] > 400)
 if(leftOnline >= 2 && rightOnline >= 2 && frontOnline >= 2 && backOnline >= 2 && firstTurn == 0)   //junction counter
 {
   if(flag_debounce)
   {
       if(turnFlag == 1)    //Rightwards linefollow
         junc_count--;
       else
         junc_count++;
       
     flag_debounce = false;
   }
 }
 else
   flag_debounce = true;

 if(junc_count == 1 && turnFlag == 2) //tranfer junction for TZ1
 {
   botKill();
   while(bit_is_clear(PINF,0));
   closeClamp();
   while(bit_is_set(PINF,0));
   turnFlag = 0;
 }
//  else if(junc_count == 2 && turnFlag == 0) //throwing junction of TZ1
//  {
//    botKill();
//    turnFlag = 3;
//    pidAlign(1);
//    opt = 0;
//    while((-500 < error < 500) && (-500 < rotation < 500))
//    {
//      readSensorFront();
//      readSensorBack();
//      calcSensorValFront();
//      calcSensorValBack();
//      calcErrorFront();
//      calcErrorBack();
//      calcErrorAndRotation();
//      calcSpeedToMotors();
//      uploadSpeedLineFollow(maxPWM);
//    }
//    botKill();
//    ///throwFromTZ1();
//    turnFlag = 1;
//    pidAlign(0);
//  }

  calcErrorAndRotation();
  calcSpeedToMotors();

  if(psx_button_press(PSB_TRIANGLE))
  {
    uploadSpeedLineFollow(maxPWM);
    return 1;
  }

  if(psx_button_click(PSB_CIRCLE,flag_CIRCLE))
    firstTurn = 1;
  
  if(psx_button_click(PSB_SQUARE,flag_SQUARE))
    turnFlag = 0;

  if(psx_button_click(PSB_CROSS,flag_CROSS))
  {
		turnFlag++;
		if(turnFlag == 4)
			turnFlag = 0;
	}
  
  else
    return 0;
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

//ISR(INT4_vect)
//{
//
//  if (bit_is_set(PINE, 5))
//  {
//    ticks--;
//  }
//  else if (bit_is_clear(PINE, 5))
//  {
//    ticks++;
//  }
//}ISR(INT4_vect)
//{
//
//  if (bit_is_set(PINE, 5))
//  {
//    ticks--;
//  }
//  else if (bit_is_clear(PINE, 5))
//  {
//    ticks++;
//  }
//}
