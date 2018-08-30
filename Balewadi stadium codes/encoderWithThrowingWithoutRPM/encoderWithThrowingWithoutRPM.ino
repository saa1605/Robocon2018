#include <io128.h>
#include <sra128.h>
#include"avr/io.h"
#include"avr/interrupt.h"
#include <pinDefsAuto.h>

bool flagThrow = false;
int throwCount = 0;  //true - to count rpm and false - to not
int PWM = 0;
volatile long int ticks = 0;   //power the bot keeping the stencil always

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
void reverseRotation()
{
  MOTORFR = 0;
  MOTORBR = 1;
}
void brakeRotation()
{
  MOTORFR = 1;
  MOTORBR = 1;
}
void stopRotation()
{
  MOTORFR = 0;
  MOTORBR = 0;
}

void setup()
{
  Serial.begin(115200);
  encoderInterruptInitialize();
  DDRD |= (1 << PD0)|(1 << PD2)|(1 << PD6)|(1 << PD7); //Throwing Motor and Relay Brakes
  PORTD |= (1 << PD1)|(1 << PD3)|(1 << PD6)|(1 << PD7); //Switches and Relay Brakes
  DDRE |= (1 << PE6)|(1 << PE7); //Clamp Piston
  
  pwm0_init();
  PORTD &= ~(1 << PD7); //Base Motors Braking
  closeClamp();
}

void loop()
{
  while(bit_is_set(PIND,3));
  throwAtTZ1();
//    openClamp();
  while(bit_is_set(PIND,1));
  closeClamp();
}

void encoderInterruptInitialize()
{
  DDRE &= ~(1 << PE4);
  DDRE &= ~(1 << PE5);
  PORTE |= (1 << PE4)|(1 << PE5);
  EICRB |= (1 << ISC41);
  EIMSK |= (1 << INT4);
}

void throwAtTZ1()
{
  throwCount = 0;
  flagThrow = true;
  forwardRotation();  //to give forward direction to the motor
  PWM = 160;
  PORTD |= (1 << PD6);  //unbrake the throwing motor
  while(bit_is_set(PIND,6))
  {
//    Serial.print("mapped : "); Serial.print(ticks % 600);
//    Serial.print(", PWM : "); Serial.print(PWM);
//    Serial.print(", flagThrow : "); Serial.print(flagThrow);
//    Serial.print(", throwCount : "); Serial.print(throwCount);
//    Serial.println("");
    
    if((515 < ticks%600) && (ticks%600 < 525) && throwCount == 2)   //change the "ticks%600 == 250" for the tick count at which it is required to throw
    {
      throwCount = 0;
      openClamp();
      stopRotation();
      PWM = 0;
      PORTD &= ~(1 << PD6);   //brake the throwing motor
    }
    
    set_pwm0(PWM);
  }
//  Serial.println("ThrowingSuccessfull");
}

ISR(INT4_vect)
{
  if(bit_is_set(PINE,5))
    ticks--;
  else if(bit_is_clear(PINE,5))  
    ticks++;

  if(ticks%600 == 0)
  {
    if(flagThrow)
    {
      throwCount ++;
      flagThrow = false;
    }
  }
  else
    flagThrow = true;
}

