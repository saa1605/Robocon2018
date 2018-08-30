#include <io128.h>
#include <sra128.h>
#include"avr/io.h"
#include"avr/interrupt.h"
#include <pinDefsAuto.h>

boolean flagRPM = true;  //true - to count rpm and false - to not
int PWM = 0;
volatile int ticks = 0, prevTicks = 0, count = 0;    //power the bot keeping the stencil always
volatile float RPM = 0;

int lowerRPM = 0, higherRPM = 1, countLimit = 100;
int lowerTicks = 154, higherTicks = 152;    //throwing at ticks = 153;

int setRPM = 0, throwError = 0, throwPWM = 0, throwKp = 8, throwKd = 3;

int timerOverFlowCounter = 0;

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

int main()
{
  Serial.begin(115200);
  TIMER3Initialize();
  encoderInterruptInitialize();
  DDRD |= (1 << PD0) | (1 << PD2) | (1 << PD6) | (1 << PD7); //Throwing Motor and Relay Brakes
  PORTD |= (1 << PD1) | (1 << PD3) | (1 << PD6) | (1 << PD7); //Switches and Relay Brakes
  DDRE |= (1 << PE7); //Clamp Piston

  pwm0_init();
  PORTD &= ~(1 << PD7); //Base Motors Braking
  closeClamp();

  while (1)
  {
    Serial.println(ticks);
//    while (bit_is_set(PIND, 3));
//    throwAtTZ1();
//    //    openClamp();
//    while (bit_is_set(PIND, 1));
//    closeClamp();
  }
}

void TIMER3Initialize()
{
  TCCR3B |= (1 << WGM32) | (1 << CS31) | (1 << CS30);
  TCNT3 = 0;
  OCR3A = 4999;
  ETIMSK |= (1 << OCIE3A);
  sei();
}

void encoderInterruptInitialize()   //PE4 - Z, PE5 - A, PE6 - B
{
  DDRE &= ~((1 << PE4) | (1 << PE5) | (1 << PE6));
  PORTE |= (1 << PE4) | (1 << PE5) | (1 << PE6);
  EICRB |= (1 << ISC41)|(1 << ISC40) | (1 << ISC51);
  EIMSK |= (1 << INT4) |  (1 << INT5);
}

void throwAtTZ1()
{
  Serial.println("ThrowingStarted");
  flagRPM = true;
  count = 0;
  forwardRotation();  //to give forward direction to the motor
  PWM = 160;
  PORTD |= (1 << PD6);  //unbrake the throwing motor
  
//  while(flagRPM)   //to adjust rpm
//  {
//    throwError = setRPM - RPM;
//    throwPWM = throwError * throwKp;
////    throwPWM = map(throwPWM, -100, 100, -255, 255);
//    
//    OCR0 = PWM + throwPWM;
//  }
  
  while (bit_is_set(PIND, 6))   //to check the ticks
  {
    if ((lowerTicks <= ticks && ticks <= higherTicks) && !(flagRPM))
    {
      count = 0;
      openClamp();
      stopRotation();
      OCR0 = 0;
      PORTD &= ~(1 << PD6);   //brake the throwing motor
    }
  }
  Serial.print("RPM : "); Serial.print(RPM);
  Serial.println("");
  Serial.print("ThrowingSuccessfull");
}

ISR(TIMER3_COMPA_vect)
{
  RPM = abs(ticks - prevTicks); //* 0.15625;
  timerOverFlowCounter++;
//  prevTicks = ticks;
//
//  if (lowerRPM < RPM && RPM < higherRPM && flagRPM)
//  {
//    count++;
//    if (count == countLimit)
//      flagRPM = false;
//  }
}

ISR(INT5_vect)
{
  if (bit_is_set(PINE, 6))
  {
    ticks--;
  }
  else if (bit_is_clear(PINE, 6))
  {
    ticks++;
  }
}

ISR(INT4_vect)
{
  ticks = 0;
}

