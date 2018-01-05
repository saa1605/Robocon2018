#include <pinDefsAuto.h>

#include <io128.h>
#include <sra128.h>

#include<avr/io.h>
#include<util/delay.h>
#include<avr/interrupt.h>

#include "Pid.h"

#include <psx.h>
#include <psx2.h>

#define joystickBuffer 35

#define lowerRotationalConstrain = 50;
#define higherRotationalConstrain = 250;

#define lowerTranslationalConstrain = 50;
#define higherTranslationalConstrain = 250;

int opt = 150;

int LX = 0, LY = 0, RX = 0, RY = 0;
int sLeft, sRight, sFront, sBack;
int pwm = 666;

void setup()
{
  psx_init(&PORTG, 0, &PORTG, 3, &PORTG, 1, &PORTG, 2);
  DDRC |= 0xFF;
  PORTC |= 0xFF;
  DDRB |= 0xF0;
  baseMotorsInitialize();
  DDRF |= 0xFF;
  PORTF |= 0xFF;
  spiMasterInit();

  PORTC = 0x00;

  Serial.begin(9600);

  PORTC = 0xFF;
  delay(500);
  PORTC = 0x00;
  delay(500);
  Serial.print("fuck");
}

void loop()
{
  psx_read_gamepad();
  if (LX = 128 && LY == (-128) && RX == 128 && RY == (-128))
  {
    LX = (psx_stick(PSS_LX)) - 127;
    LY = 127 - (psx_stick(PSS_LY));
    RX = (psx_stick(PSS_RX)) - 127;
    RY =  127 - (psx_stick(PSS_RY));

    Serial.println("Controller Disconnected");
    botKill();
    psx_init(&PORTG, 0, &PORTG, 3, &PORTG, 1, &PORTG, 2);
  }
  else
  {
    getPidError();
    baseMotorsMotion();
    printData();
  }
}

void baseMotorsInitialize()
{
  pwm0_init();
  pwm1_init();
  DDRF |= 0xFF;
  PORTF |= 0xFF;
}

void baseMotorsMotion()
{
  sLeft = 0;
  sRight = 0;
  sFront = 0;
  sBack = 0;
  
  LX = (psx_stick(PSS_LX)) - 127;
  LY = 127 - (psx_stick(PSS_LY));
  RX = (psx_stick(PSS_RX)) - 127;
  RY =  127 - (psx_stick(PSS_RY));
  
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
    if (RY > 0)
    {
      sLeft = map(RY, joystickBuffer, 127, 0, pwm);
      sRight = map(RY, joystickBuffer, 127, 0, pwm);
    }
    else if (RY < 0)
    {
      sLeft = 0 - map(RY, -127, 0 - joystickBuffer, pwm, 0);
      sRight = 0 - map(RY, -127, 0 - joystickBuffer, pwm, 0);
    }
  }

//  Serial.print(LX);
//  Serial.print("  ");
//  Serial.print(RY);
//  Serial.print("  ");
//  Serial.print(sFront);
//  Serial.print("  ");
//  Serial.print(sBack);
//  Serial.print("  ");
//  Serial.print(sLeft);
//  Serial.print("  ");
//  Serial.print(sRight);
//  Serial.println("  ");

  if(sFront < -30)
  {
    sFront *= (-1);
    MOTORFA = 0;
    MOTORFB = 1;
    if (sFront > 666)
      sFront = 666;
    PWMF = map(sFront, 0, 666, 0, 255);
//    PWMF = sFront;
  }
  else if(sFront > 30)
  {
    MOTORFA = 1;
    MOTORFB = 0;
    if (sFront > 666)
      sFront = 666;
    PWMF = map(sFront, 0, 666, 0, 255);
//    PWMF = sFront;
  }
  else
  {
    MOTORFA = 1;
    MOTORFB = 1;
    PWMF = 666;
  }
  
  if (sBack < -30)
  {
    sBack *= (-1);
    MOTORBA = 0;
    MOTORBB = 1;
    if (sBack > 666)
      sBack = 666;
    PWMB = sBack;
  }
  else if (sBack > 30)
  {
    MOTORBA = 1;
    MOTORBB = 0;
    if (sBack > 666)
      sBack = 666;
    PWMB = sBack;
  }
  else
  {
    MOTORBA = 1;
    MOTORBB = 1;
    if (sBack > 666)
      sBack = 666;
    PWMB = 666;
  }
  
  if (sLeft < -30)
  {
    sLeft *= (-1);
    MOTORLA = 0;
    MOTORLB = 1;
    if (sLeft > 666)
      sLeft = 666;
    PWML = sLeft;
  }
  else if (sLeft > 30)
  {
    MOTORLA = 1;
    MOTORLB = 0;
    if (sLeft > 666)
      sLeft = 666;
    PWML = sLeft;
  }
  else
  {
    MOTORLA = 1;
    MOTORLB = 1;
    PWML = 666;
  }
  
  if (sRight < -30)
  {
    sRight *= (-1);
    MOTORRA = 0;
    MOTORRB = 1;
    if (sRight > 666)
      sRight = 666;
    PWMR = sRight;
  }
  else if (sRight > 30)
  {
    MOTORRA = 1;
    MOTORRB = 0;
    if (sRight > 666)
      sRight = 666;
    PWMR = sRight;
  }
  else
  {
    MOTORRA = 1;
    MOTORRB = 1;
    PWMR = 666;
  }

  if (psx_button_press(PSB_CROSS))
  {
    MOTORLA = 1;
    MOTORLB = 0;
    MOTORRA = 1;
    MOTORRB = 0;
    MOTORFA = 1;
    MOTORFB = 0;
    MOTORBA = 1;
    MOTORBB = 0;
    
    PWML = constrain(opt + lineRotationalPid, lowerRotationalConstrain, higherRotationalConstrain);
    PWMR = constrain(opt + 35 - lineRotationalPid, lowerRotationalConstrain, higherRotationalConstrain);

    PWMF = constrain(lineTranstionalPid, lowerTranslationalConstrain, higherTranslationConstrain);
    PWMB = constrain(lineTranstionalPid, lowerTranslationalConstrain, higherTranslationalConstrain);
    
    Serial.println("line");
  }

  if (psx_button_click(PSB_R1,flag_R1))
  {
    if (pidMode == 1)
      lKp = 0;
    else if (pidMode == 2)
      lKi = 0;
    else if (pidMode == 3)
      lKd = 0;
  }
  
  if (psx_button_click(PSB_L1,flag_L1))
  {
    if (pidMode == 1)
      lKp += 2;
    else if (pidMode == 2)
      lKi += 0.05;
    else if (pidMode == 3)
      lKd += 0.1;
  }

  if (psx_button_click(PSB_L2,flag_L2))
  {
    if (pidMode == 1)
      lKp -= 2;
    else if (pidMode == 2)
      lKi -= 0.05;
    else if (pidMode == 3)
      lKd -= 0.1;
  }

  if (psx_button_click(PSB_PAD_LEFT,flag_PAD_LEFT))
    pidMode = 1;

  if (psx_button_click(PSB_PAD_RIGHT,flag_PAD_RIGHT))
    pidMode = 2;

  if (psx_button_click(PSB_CIRCLE,flag_CIRCLE))
    pidMode = 3;
}

void botKill()
{
  MOTORFA = 1;
  MOTORFB = 1;
  PWMF = 666;

  MOTORBA = 1;
  MOTORBB = 1;
  PWMB = 666;

  MOTORLA = 1;
  MOTORLB = 1;
  PWML = 666;

  MOTORRA = 1;
  MOTORRB = 1;
  PWMR = 666;
}

void botLeft()
{
  MOTORLA = 1;
  MOTORLB = 0;

  MOTORRA = 1;
  MOTORRB = 0;

  MOTORFA = 0;
  MOTORFB = 0;

  MOTORBA = 0;
  MOTORBB = 0;
}

void botRight()
{
  MOTORLA = 1;
  MOTORLB = 0;

  MOTORRA = 1;
  MOTORRB = 0;

  MOTORFA = 0;
  MOTORFB = 0;

  MOTORBA = 0;
  MOTORBB = 0;
}

void botBrake()
{
  MOTORLA = 1;
  MOTORLB = 1;

  MOTORRA = 1;
  MOTORRB = 1;

  MOTORFA = 1;
  MOTORFB = 1;

  MOTORBA = 1;
  MOTORBB = 1;
}

void printData()
{
//  Serial.print(pidMode);Serial.print('\t');
  Serial.print(pidFlag);Serial.print('\t');
  Serial.print(lKp);Serial.print('\t');
  Serial.print(lKi);Serial.print('\t');
  Serial.print(lKd);Serial.print('\t');
//  Serial.print(lineError);Serial.print('\t');
  Serial.print(pidError);Serial.print('\t');
  
  for(int i = 0; i < 8; i++)
  {
    Serial.print(sensorReadingsFront[i]);Serial.print('\t');
  }
  Serial.print('\t');
  
  for(int i = 0; i < 8; i++)
  {
    Serial.print(sensorReadingsBack[i]);Serial.print('\t');
  }
  Serial.println(" ");
}

void first_turn()
{
  /*if(((rightFirst + rightSecond) > sideSensorThreshold * 2)  && ((leftFirst + leftSecond) < sideSensorThreshold * 2))
    {
    bot_brake();
    _delay_ms(250);
    while(weightedSumFront >= bottomThreshold)
    {
      bot_spot_right();
    }//commense turning action
    }*/
}
