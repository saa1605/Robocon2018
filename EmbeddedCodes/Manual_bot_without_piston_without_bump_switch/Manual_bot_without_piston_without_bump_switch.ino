#include <io128.h>
#include <sra128.h>

#include <psx.h>
#include <psx2.h>

#define joystickBuffer 35

#define MOTORFA   REGISTER_BIT(PORTC,0)
#define MOTORFB   REGISTER_BIT(PORTC,1)
#define PWMF      OCR0

#define MOTORLA   REGISTER_BIT(PORTC,3)
#define MOTORLB   REGISTER_BIT(PORTC,2)
#define PWML      OCR1A

#define MOTORBA   REGISTER_BIT(PORTC,5)
#define MOTORBB   REGISTER_BIT(PORTC,4)
#define PWMB      OCR1B

#define MOTORRA   REGISTER_BIT(PORTC,7)
#define MOTORRB   REGISTER_BIT(PORTC,6)
#define PWMR      OCR1C

#define LMGA      REGISTER_BIT(PORTD,4)
#define LMGB      REGISTER_BIT(PORTD,5)
#define LMGBRAKE  REGISTER_BIT(PORTD,7)


int sLeft, sRight, sFront, sBack;

int pwm = 666;
void setup()
{
  psx_init(&PORTG, 0, &PORTG, 3, &PORTG, 1, &PORTG, 2);
  DDRC = 0xFF;
  DDRB |= 0b11110000;
  baseMotorsInitialize();
  LMGInitialize();
  DDRD |= 1 << PD0;
  PORTD |= 1 << PD0;

  Serial.begin(9600);
}
void loop()
{
  if (readController()) {
    psx_read_gamepad();
    baseMotorsMotion();
    LMGMotion();
  }

  else {
    botKill();
    psx_init(&PORTG, 0, &PORTG, 3, &PORTG, 1, &PORTG, 2);

  }
}

void LMGMotion()
{
  if (psx_button_press(PSB_PAD_UP))
  {
    LMGA = 1;
    LMGB = 0;
    LMGBRAKE = 1;
    Serial.println("Lmg up");
  }
  else if (psx_button_press(PSB_PAD_DOWN))
  {
    LMGA = 0;
    LMGB = 1;
    LMGBRAKE = 1;
    Serial.println("LMG down");
  }
  else
  {
    LMGA = 1;
    LMGB = 1;
    LMGBRAKE = 0;
  }
}
void LMGInitialize(){
  DDRD|=(1<<PD4)|(1<<PD5)|(1<<PD7);
  PORTD&=~(1<<PD4);
  PORTD&=~(1<<PD5);
  PORTD&= ~(1<<PD7);
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

  int stickLX = (psx_stick(PSS_LX)) - 127;

  int stickRY =  127-(psx_stick(PSS_RY)) ;


  if (stickLX > joystickBuffer || stickLX < 0 - joystickBuffer)
  {
    if (stickLX > 0)
    {
      sFront = map(stickLX, joystickBuffer, 127, 0, pwm);
      sBack = map(stickLX, joystickBuffer, 127, 0, pwm);
    }
    else if (stickLX < 0)
    {
      sFront = 0 - map(stickLX, -127, 0 - joystickBuffer, pwm, 0);
      sBack = 0 - map(stickLX, -127, 0 - joystickBuffer, pwm, 0);
    }
  }

  else if (stickRY > joystickBuffer || stickRY < 0 - joystickBuffer)
  {
    if (stickRY > 0)
    {
      sLeft = map(stickRY, joystickBuffer, 127, 0, pwm);
      sRight = map(stickRY, joystickBuffer, 127, 0, pwm);
    }
    else if (stickRY < 0)
    {
      sLeft = 0 - map(stickRY, -127, 0 - joystickBuffer, pwm, 0);
      sRight = 0 - map(stickRY, -127, 0 - joystickBuffer, pwm, 0);
    }
  }

  Serial.print(stickLX);
  Serial.print("\t");
  Serial.print(stickRY);
  Serial.print("\t");
  Serial.print(sFront);
  Serial.print("\t");
  Serial.print(sBack);
  Serial.print("\t");
  Serial.print(sLeft);
  Serial.print("\t");
  Serial.println(sRight);
  //  Serial.print("\t");
  //  Serial.print(PWMF);
  //  Serial.print("\t");
  //  Serial.print(PWMB);
  //  Serial.print("\t");
  //  Serial.print(PWML);
  //  Serial.print("\t");
  //  Serial.println(PWMR);

  if (sFront < -30)
  {
    sFront *= (-1);
    MOTORFA = 0;
    MOTORFB = 1;
    if (sFront > 666)
      sFront = 666;
    //    PWMF = map(sFront, 0, 666, 0, 255);
    PWMF = sFront;
  }
  else if (sFront > 30)
  {
    MOTORFA = 1;
    MOTORFB = 0;
    if (sFront > 666)
      sFront = 666;
    PWMF = sFront;
    //    PWMF = map(sFront, 0, 666, 0, 255);
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
}

void botKill() {
  LMGA = 1;
  LMGB = 1;
  LMGBRAKE = 0;

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

