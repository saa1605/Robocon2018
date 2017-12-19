#include <io128.h>
#include <sra128.h>

#include "Pid.h"

#define MOTORLA   REGISTER_BIT(PORTC,0) //whether to give the Encoder_PID to front and back motors
#define MOTORLB   REGISTER_BIT(PORTC,1)
#define PWML      OCR1A

#define MOTORRA   REGISTER_BIT(PORTC,2)
#define MOTORRB   REGISTER_BIT(PORTC,3)
#define PWMR      OCR1B

#define opt 0

float pidError = 0;

float lowerPWMConstrain = 0;
float higherPWMConstrain = 0;

void setup()
{
  Serial.begin(9600);
  DDRC |= 0xFF;
  PORTC |= 0xFF;
  DDRB |= 0xF0;
  pwm1_init();
  DDRF |= 0xFF;
  PORTF |= 0xFF;
  spiMasterInit();
}

void loop()
{
  pidError = getPidError();
  
  if (pidError > 0)
    bot_left();
  else if (pidError < 0)
    bot_right();
  else bot_brake();
  
  PWML = constrain(opt + pidError, lowerPWMConstrain, higherPWMConstrain);
  PWMR = constrain(opt - pidError, lowerPWMConstrain, higherPWMConstrain);

  printData();
}

void bot_left()
{
  MOTORLA = 0;
  MOTORLB = 1;
  
  MOTORRA = 1;
  MOTORRB = 0;
}

void bot_right()
{
  MOTORLA = 1;
  MOTORLB = 0;
  
  MOTORRA = 0;
  MOTORRB = 1;
}

void bot_brake()
{
  MOTORLA = 1;
  MOTORLB = 1;
  
  MOTORRA = 1;
  MOTORRB = 1;
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
