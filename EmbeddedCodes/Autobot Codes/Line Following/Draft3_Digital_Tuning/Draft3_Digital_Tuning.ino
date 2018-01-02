#include <pinDefsManual.h>

#include <io128.h>
#include <sra128.h>

#include "Pid.h"

#include <psx.h>
#include <psx2.h>

#define joystickBuffer 35

#define opt 0

int LX = 0, LY = 0, RX = 0, RY = 0;

float pidError = 0;

float lowerPWMConstrain = 0;
float higherPWMConstrain = 0;

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

  Serial.begin(9600);
}

void loop()
{
  psx_read_gamepad();
  if (LX = 128 && LY == (-128) && RX == 128 && RY == (-128))
  {
    LX = (psx_stick(PSS_LX)) - 127;
    LY = 127 - (psx_stick(PSS_LY));
    RX = (psx_stick(PSS_RX)) - 127;
    RY =  127 - (psx_stick(PSS_RY)) ;

    Serial.println("Controller Disconnected");
    botKill();
    psx_init(&PORTG, 0, &PORTG, 3, &PORTG, 1, &PORTG, 2);
  }
  
  pidError = getPidError();
  baseMotorsMotion();
//  printData();
}

void baseMotorsInitialize()
{
  pwm1_init();
  DDRF |= 0xFF;
  PORTF |= 0xFF;
}

void baseMotorsMotion()
{
  LX = (psx_stick(PSS_LX)) - 127;
  LY = 127 - (psx_stick(PSS_LY));
  RX = (psx_stick(PSS_RX)) - 127;
  RY =  127 - (psx_stick(PSS_RY)) ;
  
//    if (LX > joystickBuffer || LX < 0 - joystickBuffer)
//    {
//      if (LX > 0)
//      {
//        sFront = map(LX, joystickBuffer, 127, 0, pwm);
//        sBack = map(LX, joystickBuffer, 127, 0, pwm);
//      }
//      else if (LX < 0)
//      {
//        sFront = 0 - map(LX, -127, 0 - joystickBuffer, pwm, 0);
//        sBack = 0 - map(X, -127, 0 - joystickBuffer, pwm, 0);
//      }
//    }
//  
//    else if (RY > joystickBuffer || RY < 0 - joystickBuffer)
//    {
//      if (RY > 0)
//      {
//        sLeft = map(RY, joystickBuffer, 127, 0, pwm);
//        sRight = map(RY, joystickBuffer, 127, 0, pwm);
//      }
//      else if (RY < 0)
//      {
//        sLeft = 0 - map(RY, -127, 0 - joystickBuffer, pwm, 0);
//        sRight = 0 - map(RY, -127, 0 - joystickBuffer, pwm, 0);
//      }
//    }

    if(pidError > 0)
      botLeft();
    else if(pidError < 0)
      botRight();
    else
      botBrake();

  PWML = constrain(opt + pidError, lowerPWMConstrain, higherPWMConstrain);
  PWMR = constrain(opt - pidError, lowerPWMConstrain, higherPWMConstrain);

  if (psx_button_press(PSB_CROSS))
  {
    botKill();
    Serial.println("Bot Brake");
  }

  if (psx_button_press(PSB_PAD_UP))
  {
    if (pidFlag == 1)
      lKp += 1;
    else if (pidFlag == 2)
      lKi += 0.05;
    else if (pidFlag == 3)
      lKd += 0.1;
    while (psx_button_press(PSB_PAD_UP));
  }

  if (psx_button_press(PSB_PAD_DOWN))
  {
    if (pidFlag == 1)
      lKp -= 1;
    else if (pidFlag == 2)
      lKi -= 0.05;
    else if (pidFlag == 3)
      lKd -= 0.1;
    while (psx_button_press(PSB_PAD_DOWN));
  }

  if (psx_button_press(PSB_SQUARE))
  {
    pidFlag = 1;
    pidMode = 1;
    while (psx_button_press(PSB_SQUARE));
  }

  if (psx_button_press(PSB_TRIANGLE))
  {
    pidFlag = 2;
    pidMode = 2;
    while (psx_button_press(PSB_TRIANGLE));
  }

  if (psx_button_press(PSB_CIRCLE))
  {
    pidFlag = 3;
    pidMode = 3;
    while (psx_button_press(PSB_CIRCLE));
  }
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

  MOTORFA = 0;
  MOTORFB = 0;

  MOTORBA = 0;
  MOTORBB = 0;
}

void printData()
{
  Serial.println(pidError);
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
