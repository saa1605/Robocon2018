#include <io128.h>
#include <sra128.h>
#include "avr/io.h"
#include "avr/interrupt.h"
#include <pinDefsAutoNew.h>
#define debug
volatile long int ticks = 0;
volatile int pidTicks = 0, throwCount = 0;

bool firstTimeAlign = true;
int armError = 0, prevArmError = 0, integralError = 0, diffError = 0;
int armKp = 3, armKi = 0, armKd = 8;//kp=2

int armPID = 0, armPWM = 0;
int pidCount = 0;

int throws = 0;

int TZ1Ticks = -223, TZ2Ticks = -221, TZ3Ticks = -206;
int throwPWM3 = 225;

int calibrate = 0;   //0-PWM, 1-TZ1, 2-TZ2, 3-TZ3

void setup()
{
  Serial.begin(115200);
  baseMotorsInit();
  encoderInterruptInit();
  throwingMotorInit();
  clampPistonInit();

  baseBrake();   //for braking the base motor
  throwBrake();   //for braking the throwing motor

  DDRA &= ~((1 << PA4)|(1 << PA5)|(1 << PA6)|(1 << PA7));   //for buttons A4567
  PORTA |= ((1 << PA4)|(1 << PA5)|(1 << PA6)|(1 << PA7));   //for buttons A4567

  DDRA|=(1<<PA0)|(1<<PA1); //For Power Led
  PORTA&= ~((1<<PA0)|(1<<PA1));

  Serial.println("throwAtTZ Testing");
  PORTC = 0xFF;
  _delay_ms(200);
  PORTC = 0x00;
  _delay_ms(200);

  while(1)
  {
    if(bit_is_clear(PINA,7))
    {
      openClamp();
      break;
    }
    else
    {
      Serial.println("Press A7 to OPEN Clamp");
    }
  }
  _delay_ms(500);
}

void loop()
{
  if(bit_is_clear(PINA,4))
  {
    while(1)
    {
      Serial.println("");
      Serial.println("Press A7 to alignArm");
      while(bit_is_set(PINA,7));
      alignArm();
      Serial.println("");
      Serial.println("Press A7 to closeClamp");
      while(bit_is_set(PINA,7));
      closeClamp();
      _delay_ms(500);

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
      
      Serial.print("A4 : TZ1 ");Serial.print(" A5 : TZ2 ");Serial.print(" A6 : TZ3");
      Serial.println("");
      while(1)
      {
        if(bit_is_clear(PINA,4))
        {
          Serial.println("TZ1");
          throwAtTZ(1);
          break;
        }
        else if(bit_is_clear(PINA,5))
        {
          Serial.println("TZ2");
          throwAtTZ(2);
          break;
        }
        else if(bit_is_clear(PINA,6))
        {
          Serial.println("TZ3");
          throwAtTZ(3);
          break;
        }
      }
    }
  }
  
  else if(bit_is_clear(PINA,7))   //TZTicks Calibration
  {
    PORTC = 0xFF;
    _delay_ms(200);
    PORTC = 0x00;
    _delay_ms(200);
    while(bit_is_set(PINA,7))
    {
      if(bit_is_clear(PINA,6))
      {
        if(calibrate == 0)
        {
          calibrate = 1;
          PORTC = 0b00001000;
        }
        else if(calibrate == 1)
        {
          calibrate = 2;
          PORTC = 0b00010100;
        }
        else if(calibrate == 2)
        {
          calibrate = 3;
          PORTC = 0b00101010;
        }
        else if(calibrate == 3)
        {
          calibrate = 0;
          PORTC = 0b11111111;
        }
        
        _delay_ms(100);
        while(bit_is_clear(PINA,6));
      }
      
      if(bit_is_clear(PINA,4))
      {
        if(calibrate == 0)
          throwPWM3 -= 1;
        else if(calibrate == 1)
          TZ1Ticks -= 1;
        else if(calibrate == 2)
          TZ2Ticks -= 1;
        else if(calibrate == 3)
          TZ3Ticks -= 1;
        
        PORTC = 0xFF;
        _delay_ms(100);
        PORTC = 0x00;
        _delay_ms(100);
        while(bit_is_clear(PINA,4));
      }

      if(bit_is_clear(PINA,5))
      {
        if(calibrate == 0)
          throwPWM3 += 1;
        else if(calibrate == 1)
          TZ1Ticks += 1;
        else if(calibrate == 2)
          TZ2Ticks += 1;
        else if(calibrate == 3)
          TZ3Ticks += 1;
        
        PORTC = 0xFF;
        _delay_ms(100);
        PORTC = 0x00;
        _delay_ms(100);
        while(bit_is_clear(PINA,5));
      }

      Serial.print(" throwPWM3 : ");Serial.print(throwPWM3);
      Serial.print(" TZ1 : ");Serial.print(TZ1Ticks);
      Serial.print(" TZ2 : ");Serial.print(TZ2Ticks);
      Serial.print(" TZ3 : ");Serial.print(TZ3Ticks);
      Serial.println("");
    }
    _delay_ms(500);
  }
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
  _delay_ms(2);
}

void throwUnbrake()
{ 
  THROWBRAKE = 1;    //unbraking the throwing motor
  _delay_ms(2);
}

void baseBrake()
{
  BASEBRAKE = 0;    //braking the base motor
  _delay_ms(2);
}

void baseUnbrake()
{
  BASEBRAKE = 1;    //unbraking the base motor
  _delay_ms(2);
}

void clampPistonInit()
{
  OPENCLAMP_DIR = 1;   //Clamp Piston
  OPENCLAMP = 0;
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
  
  throwUnbrake();   //for unbraking the throwing motor

  armError = adjust();
  
  if(armError > 0)
  {
    backwardRotation();
  }
  else if(armError < 0) 
  {
    forwardRotation();
  }
  else
  {
    forwardRotation();
  }
  PWMT = map(50, 0, 255, 0, 666);
  throwCount = 0;
  while(throwCount == 0)
  {
    Serial.print("ticks: ");Serial.print(ticks);
    Serial.print(" throwCount: ");Serial.print(throwCount);
    Serial.println("");
  }
  PWMT = 0;
  stopRotation();
  Serial.print("afterwhilethrowCount:");
  int pidCount = 100;
  Serial.println(throwCount);
  while(pidCount != 0)
  {
    armError = adjust();
    while(armError <= -2 || 2 <= armError)
    {
      pidCount = 100;
      Serial.println(ticks);
      throwUnbrake();   //for unbraking the throwing motor
      armError = adjust();
      
      diffError = armError - prevArmError;
      armPID = abs(armKp * armError + armKd * diffError);
      
      armPWM = map(armPID, 8, 400, 25, 80);
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
    Serial.print(" pidCount = ");Serial.println(pidCount);
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
    PWMT = map(throwPWM3, 0, 255, 0, 666);
    
    while(1)
    {
      #ifdef debug
      Serial.print("ticks:\t");
      Serial.print(ticks);  
      Serial.println(" ");
      #endif
      
      if(ticks <= TZ3Ticks && throwCount >= 5)
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
  _delay_ms(100);
  stopRotation();
  PWMT = 0;
  throwBrake();   //for braking the throwing motor
  throws++;
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
  Serial.print("throwCount printed from ISR : ");
  Serial.println(throwCount);
  Serial.print("ticks printed from ISR : ");
  Serial.println(ticks);
  
}

