#include <pinDefsManual.h>

#include <io128.h>
#include <sra128.h>

#include <psx.h>
#include <psx2.h>

#define joystickBuffer 35


int sLeft, sRight, sFront, sBack;

int pwm = 666;
void setup()
{
  psx_init(&PORTG, 0, &PORTG, 3, &PORTG, 1, &PORTG, 2);
  DDRC = 0xFF;
  DDRB |= 0b11110000;
  baseMotorsInitialize();
  LMGInitialize();
  pistonInitialize();
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
    pistonMotion();

  }

  else {
    botKill();
    psx_init(&PORTG, 0, &PORTG, 3, &PORTG, 1, &PORTG, 2);

  }
}

void LMGInitialize() {
  DDRD |= (1 << LMGA) | (1 << LMGB) | (1 << LMGBRAKE);
  PORTD &= ~(1 << LMGA);
  PORTD &= ~(1 << LMGB);
  PORTD &= ~(1 << LMGBRAKE);
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

  int stickRY =  127 - (psx_stick(PSS_RY)) ;


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

if(psx_button_press(PSB_PAD_RIGHT)){
    MOTORFA=1;
    MOTORFB=0;
    PWMF=63;

    MOTORLA=1;
    MOTORLB=0;
    PWML=166;

    MOTORBA=0;
    MOTORBB=1;
    PWMB=166;

    MOTORRA=0;
    MOTORRB=1;
    PWMR=166;

    Serial.println("Spot right");
  }

  if(psx_button_press(PSB_PAD_LEFT)){
    MOTORFA=0;
    MOTORFB=1;
    PWMF=63;//HALF PWM FOR SLOW SPOT TURN

    MOTORLA=0;
    MOTORLB=1;
    PWML=166;

    MOTORBA=1;
    MOTORBB=0;
    PWMB=166;

    MOTORRA=1;
    MOTORRB=0;
    PWMR=166;    

    Serial.println("Spot right");
    
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

void pistonInitialize() {
  DDRE |= (1 << PISTON1A) | (1 << PISTON1B);
  PORTE &= ~(1 << PISTON1A);
  PORTE &= ~(1 << PISTON1B);

  DDRB |= (1 << PISTON2A) | (1 << PISTON2B);
  PORTB &= ~(1 << PISTON2A);
  PORTB &= ~(1 << PISTON2B);
}

void pistonMotion() {
  if (psx_button_press(PSB_CIRCLE)) {
    PISTON1A = 1;
    PISTON1B = 0;
    Serial.println("Piston1 out");
  }
  else if (psx_button_press(PSB_SQUARE)) {
    PISTON1A = 0;
    PISTON1B = 0;
    Serial.println("Piston1 in");
  }


  if (psx_button_press(PSB_CROSS)) {
    PISTON2A = 1;
    PISTON2B = 0;
    Serial.println("Piston2 out");
  }
  else if (psx_button_press(PSB_TRIANGLE)) {
  PISTON2A = 0;
  PISTON2B = 0;
  Serial.println("Piston2 in");

  }
}

