#include <io128.h>
#include <sra128.h>

#include "PID.h"

#define MOTORLA   REGISTER_BIT(PORTC,0) //whether to give the Encoder_PID to front and back motors
#define MOTORLB   REGISTER_BIT(PORTC,1)
#define PWML      OCR1A

#define MOTORRA   REGISTER_BIT(PORTC,2)
#define MOTORRB   REGISTER_BIT(PORTC,3)
#define PWMR      OCR1B

#define opt 0

float PID_Error = 0;

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
}

void loop()
{
  PID_Error = get_PID_Error();
  
  if (PID_Error > 0)
    bot_Left();
  else if (PID_Error < 0)
    bot_Right();
  else bot_Brake();
  
  PWML = constrain(opt + PID_Error, lowerPWMConstrain, higherPWMConstrain);
  PWMR = constrain(opt - PID_Error, lowerPWMConstrain, higherPWMConstrain);

  print_data();
}

void bot_Left()
{
  MOTORLA = 0;
  MOTORLB = 1;
  
  MOTORRA = 1;
  MOTORRB = 0;
}

void bot_Right()
{
  MOTORLA = 1;
  MOTORLB = 0;
  
  MOTORRA = 0;
  MOTORRB = 1;
}

void bot_Brake()
{
  MOTORLA = 1;
  MOTORLB = 1;
  
  MOTORRA = 1;
  MOTORRB = 1;
}

void print_data()
{
  Serial.println(PID_Error);
}

