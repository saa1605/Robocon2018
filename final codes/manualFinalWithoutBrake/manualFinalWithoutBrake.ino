#include <pinDefsManualNew.h>

#include <io128.h>
#include <sra128.h>

#include"avr/io.h"
#include"avr/interrupt.h"

#include <psx.h>
#include <psx2.h>

#define joystickBuffer 35

int LX = 0, LY = 0, RX = 0, RY = 0;


int sLeft, sRight, sFront, sBack;
int slowFlag = 1;
int pwm = 600;
void setup()
{
  psx_init(&PORTG, 0, &PORTG, 3, &PORTG, 1, &PORTG, 2);
  DDRC = 0xFF;
  PORTC = 0xFF;
  baseMotorsInitialize();

  pistonInitialize();
  EICRB |= (1 << ISC61) | (1 << ISC60) | (1 << ISC71) | (1 << ISC70); //FOR INTERRUPT 2 AND 3(IF CHANGING CHANGE
  EIMSK |= (1 << INT6) | (1 << INT7);; //VECTORS AND PIN DEFS
  LMGInitialize();
  sei();
  Serial.begin(115200);
  DDRD |= (1 << PD7);
  BASEBRAKE = 1;
}
void loop()
{
  psx_read_gamepad();
  if (LX = 128 && LY == (-128) && RX == 128 && RY == (-128)) {
    LX = (psx_stick(PSS_LX)) - 127;
    LY = 127 - (psx_stick(PSS_LY));
    RX = (psx_stick(PSS_RX)) - 127;
    RY =  127 - (psx_stick(PSS_RY)) ;

    Serial.println("Controller Disconnected");
    botKill();
    psx_init(&PORTG, 0, &PORTG, 3, &PORTG, 1, &PORTG, 2);
  }
  else {
    baseMotorsMotion();
    LMGMotion();
    pistonMotion();
  }
}

void LMGInitialize() {
  DDRD |= (1 << LMGA) | (1 << LMGB) | (1 << LMGBRAKE);
  PORTD &= ~((1 << LMGA) | (1 << LMGB) | (1 << LMGBRAKE));
  DDRE &= ~((1 << PE6) | (PE7));
  PORTE |= (1 << PE6) | (1 << PE7);

  DDRB |= (1 << PB4);
  PORTB |= (1 << PB4); // PWM of LMG
}

void LMGMotion()
{
  if (psx_button_press(PSB_PAD_UP))
  {
    if (!BSUP) {
      LMGA = 1;
      LMGB = 0;
      LMGBRAKE = 1;
      Serial.println("Lmg up");
    }
    else if (BSUP) {
      LMGA = 0;
      LMGB = 0;
      LMGBRAKE = 0;
      Serial.println("up brake");

    }
  }
  else if (psx_button_press(PSB_PAD_DOWN))
  {
    if (!BSDOWN) {
      LMGA = 0;
      LMGB = 1;
      LMGBRAKE = 1;
      Serial.println("LMG down");
    }

    else if (BSDOWN) {
      LMGA = 0;
      LMGB = 0;
      LMGBRAKE = 0;
      Serial.println("down brake");

    }
  }
  else
  {
    LMGA = 0;
    LMGB = 0;
    LMGBRAKE = 0;
    Serial.println("brake");
  }
}


void baseMotorsInitialize()
{
  pwm3_init();
  pwm1_init();
  DDRB |= (1 << PB5) | (1 << PB6); //PWM pins as output
  DDRE |= (1 << PE3) | (1 << PE4);
}

void baseMotorsMotion()
{
  sLeft = 0;
  sRight = 0;
  sFront = 0;
  sBack = 0;

  int stickLX = (psx_stick(PSS_LX)) - 127;
  int stickLY = 127 - (psx_stick(PSS_LY));
  int stickRX = (psx_stick(PSS_RX)) - 127;
  int stickRY =  127 - (psx_stick(PSS_RY)) ;

  LX = stickLX;
  LY = stickLY;
  RX = stickRX;
  RY = stickRY;

  if (stickRX < joystickBuffer && stickRX > 0 - joystickBuffer&&(!psx_button_press(PSB_L1))&&(!psx_button_press(PSB_R1)))
  {

    MOTORFA = 0;
    MOTORFB = 0;

    MOTORBA = 0;
    MOTORBB = 0;

    MOTORLA = 0;
    MOTORLB = 0;

    MOTORRA = 0;
    MOTORRB = 0;
    _delay_ms(50);
  }

  if (stickLY < joystickBuffer && stickLY > 0 - joystickBuffer&&(!psx_button_press(PSB_L1))&&(!psx_button_press(PSB_R1)))
  {

    MOTORFA = 0;
    MOTORFB = 0;

    MOTORBA = 0;
    MOTORBB = 0;

    MOTORLA = 0;
    MOTORLB = 0;

    MOTORRA = 0;
    MOTORRB = 0;
    _delay_ms(50);
   
  }

  if (stickRX > joystickBuffer || stickRX < 0 - joystickBuffer)
  {
    if (stickRX > 0)
    {
      sFront = map(stickRX, joystickBuffer, 127, 0, pwm);
      sBack = map(stickRX, joystickBuffer, 127, 0, pwm);
    }
    else if (stickRX < 0)
    {
      sFront = 0 - map(stickRX, -127, 0 - joystickBuffer, pwm, 0);
      sBack = 0 - map(stickRX, -127, 0 - joystickBuffer, pwm, 0);
    }

    if (psx_button_press(PSB_R1)) {
      sLeft = 200;
      sRight = -200;
    }
    else if (psx_button_press(PSB_L1)) {
      sLeft = -200;
      sRight = 200;
    }
  }

  else if (stickLY > joystickBuffer || stickLY < 0 - joystickBuffer)
  {
    if (stickLY > 0)
    {
      sLeft = map(stickLY, joystickBuffer, 127, 0, pwm);
      sRight = map(stickLY, joystickBuffer, 127, 0, pwm);
    }
    else if (stickLY < 0)
    {
      sLeft = 0 - map(stickLY, -127, 0 - joystickBuffer, pwm, 0);
      sRight = 0 - map(stickLY, -127, 0 - joystickBuffer, pwm, 0);
    }

    if (psx_button_press(PSB_R1)) {
      sFront = 200;
      sBack = -200;
    }
    else if (psx_button_press(PSB_L1)) {
      sFront = -200;
      sBack = 200;
    }
  }
  //
  Serial.print(stickRX);
  Serial.print("\t");
  Serial.print(stickLY);
  Serial.print("\t");
  Serial.print(sFront);
  Serial.print("\t");
  Serial.print(sBack);
  Serial.print("\t");
  Serial.print(sLeft);
  Serial.print("\t");
  Serial.println(sRight);
  Serial.print("\t");
  Serial.print(slowFlag);
  Serial.print("\t");
  //  Serial.print(PWMF);
  //  Serial.print("\t");
  //  Serial.print(PWMB);
  //  Serial.print("\t");
  //  Serial.print(PWML);
  //  Serial.print("\t");
  //  Serial.println(PWMR);

  if (psx_button_click(PSB_R2, flag_R2)) {
    slowFlag = 1;
  }
  else if (psx_button_click(PSB_L2, flag_L2)) {
    slowFlag = 0;
  }
  else if (psx_button_press(PSB_R3)) {
    slowFlag = 2;
  }
  if ((!psx_button_press(PSB_R1)) && (!psx_button_press(PSB_L1))) {

    if (sFront < -30)
    {
      sFront *= (-1);
      MOTORFA = 0;
      MOTORFB = 1;
      if (sFront > pwm)
        sFront = pwm;
      if (slowFlag == 0) {
        PWMF = sFront;
      }
      else if (slowFlag == 1) {
        PWMF = map(sFront, 0, pwm, 0, 333);
      }
      else if (slowFlag == 2) {
        PWMF = map(sFront, 0, pwm, 0, 170);
      }
    }
    else if (sFront > 30)
    {
      MOTORFA = 1;
      MOTORFB = 0;
      if (sFront > pwm)
        sFront = pwm;
      if (slowFlag == 0) {
        PWMF = sFront;
      }
      else if (slowFlag == 1) {
        PWMF = map(sFront, 0, pwm, 0, 333);
      }
      else if (slowFlag == 2) {
        PWMF = map(sFront, 0, pwm, 0, 170);
      }
    }
    else
    {
      MOTORFA = 0;
      MOTORFB = 0;
      PWMF = pwm;
    }

    if (sBack < -30)
    {
      sBack *= (-1);
      MOTORBA = 0;
      MOTORBB = 1;
      if (sBack > pwm)
        sBack = pwm;
      if (slowFlag == 0) {
        PWMB = sBack;
      }
      else if (slowFlag == 1) {
        PWMB = map(sBack, 0, pwm, 0, 333);
      }
      else if (slowFlag == 2) {
        PWMB = map(sBack, 0, pwm, 0, 170);
      }
    }
    else if (sBack > 30)
    {
      MOTORBA = 1;
      MOTORBB = 0;
      if (sBack > pwm)
        sBack = pwm;
      if (slowFlag == 0) {
        PWMB = sBack;
      }
      else if (slowFlag == 1) {
        PWMB = map(sBack, 0, pwm, 0, 333);
      }
      else if (slowFlag == 2) {
        PWMB = map(sBack, 0, pwm, 0, 170);
      }
    }
    else
    {
      MOTORBA = 0;
      MOTORBB = 0;
      PWMB = pwm;
    }

    if (sRight < -30)
    {
      sRight *= (-1);
      MOTORRA = 0;
      MOTORRB = 1;
      if (sRight > pwm)
        sRight = pwm;
      if (slowFlag == 0) {
        PWMR = sRight;
      }
      else if (slowFlag == 1) {
        PWMR = map(sRight, 0, pwm, 0, 333);
      }
      else if (slowFlag == 2) {
        PWMR = map(sRight, 0, pwm, 0, 170);
      }
    }
    else if (sRight > 30)
    {
      MOTORRA = 1;
      MOTORRB = 0;
      if (sRight > pwm)
        sRight = pwm;
      if (slowFlag == 0) {
        PWMR = sRight;
      }
      else if (slowFlag == 1) {
        PWMR = map(sRight, 0, pwm, 0, 333);
      }
      else if (slowFlag == 2) {
        PWMR = map(sRight, 0, pwm, 0, 170);
      }
    }
    else
    {
      MOTORRA = 0;
      MOTORRB = 0;
      PWMR = pwm;
    }

    if (sLeft < -30)
    {
      sLeft *= (-1);
      MOTORLA = 0;
      MOTORLB = 1;
      if (sLeft > pwm)
        sLeft = pwm;
      if (slowFlag == 0) {
        PWML = sLeft;
      }
      else if (slowFlag == 1) {
        PWML = map(sLeft, 0, pwm, 0, 333);
      }
      else if (slowFlag == 2) {
        PWML = map(sLeft, 0, pwm, 0, 170);
      }
    }
    else if (sLeft > 30)
    {
      MOTORLA = 1;
      MOTORLB = 0;
      if (sLeft > pwm)
        sLeft = pwm;
      if (slowFlag == 0) {
        PWML = sLeft;
      }
      else if (slowFlag == 1) {
        PWML = map(sLeft, 0, pwm, 0, 333);
      }
      else if (slowFlag == 2) {
        PWML = map(sLeft, 0, pwm, 0, 170);
      }
    }
    else
    {
      MOTORLA = 0;
      MOTORLB = 0;
      PWML = pwm;
    }
  }

  _delay_ms(50);     // Delay for switching

  if (psx_button_press(PSB_R1) && !(stickRX > joystickBuffer || stickRX < 0 - joystickBuffer) && !(stickLY > joystickBuffer || stickLY < 0 - joystickBuffer)) {
    if (slowFlag == 0) {
      MOTORFA = 1;
      MOTORFB = 0;
      PWMF = 333;

      MOTORLA = 1;
      MOTORLB = 0;
      PWML = 333;

      MOTORBA = 0;
      MOTORBB = 1;
      PWMB = 333;

      MOTORRA = 0;
      MOTORRB = 1;
      PWMR = 333;
    }
    else if (slowFlag == 1) {
      MOTORFA = 1;
      MOTORFB = 0;
      PWMF = 100;

      MOTORLA = 1;
      MOTORLB = 0;
      PWML = 100;

      MOTORBA = 0;
      MOTORBB = 1;
      PWMB = 100;

      MOTORRA = 0;
      MOTORRB = 1;
      PWMR = 100;
    }
    else if (slowFlag == 2) {
      MOTORFA = 1;
      MOTORFB = 0;
      PWMF = 70;

      MOTORLA = 1;
      MOTORLB = 0;
      PWML = 70;

      MOTORBA = 0;
      MOTORBB = 1;
      PWMB = 70;

      MOTORRA = 0;
      MOTORRB = 1;
      PWMR = 70;
    }

    Serial.println("Spot right");
  }

  //  _delay_ms(50);     // Delay for switching

  else if (psx_button_press(PSB_L1) && !(stickRX > joystickBuffer || stickRX < 0 - joystickBuffer) && !(stickLY > joystickBuffer || stickLY < 0 - joystickBuffer)) {
    if (slowFlag == 0) {
      MOTORFA = 0;
      MOTORFB = 1;
      PWMF = 333; //HALF PWM FOR SLOW SPOT TURN

      MOTORLA = 0;
      MOTORLB = 1;
      PWML = 333;

      MOTORBA = 1;
      MOTORBB = 0;
      PWMB = 333;

      MOTORRA = 1;
      MOTORRB = 0;
      PWMR = 333;
    }
    else if (slowFlag == 1) {
      MOTORFA = 0;
      MOTORFB = 1;
      PWMF = 100; //HALF PWM FOR SLOW SPOT TURN

      MOTORLA = 0;
      MOTORLB = 1;
      PWML = 100;

      MOTORBA = 1;
      MOTORBB = 0;
      PWMB = 100;

      MOTORRA = 1;
      MOTORRB = 0;
      PWMR = 100;
    }
    else if (slowFlag == 2) {
      MOTORFA = 0;
      MOTORFB = 1;
      PWMF = 70; //HALF PWM FOR SLOW SPOT TURN

      MOTORLA = 0;
      MOTORLB = 1;
      PWML = 70;

      MOTORBA = 1;
      MOTORBB = 0;
      PWMB = 70;

      MOTORRA = 1;
      MOTORRB = 0;
      PWMR = 70;
    }

    Serial.println("Spot right");

  }

  _delay_ms(50);     // Delay for switching
}



void botKill() {
  _delay_ms(50);     // Delay for switching
  LMGA = 0;
  LMGB = 0;
  LMGBRAKE = 0;

  MOTORFA = 0;
  MOTORFB = 0;
  PWMF = pwm;

  MOTORBA = 0;
  MOTORBB = 0;
  PWMB = pwm;

  MOTORLA = 0;
  MOTORLB = 0;
  PWML = pwm;

  MOTORRA = 0;
  MOTORRB = 0;
  PWMR = pwm;


}

void pistonInitialize() {
  DDRA |= (1 << PISTON1A) | (1 << PISTON1B);
  PORTA &= ~(1 << PISTON1A);
  PORTA &= ~(1 << PISTON1B);

  DDRA |= (1 << PISTON2A) | (1 << PISTON2B);
  PORTA &= ~(1 << PISTON2A);
  PORTA &= ~(1 << PISTON2B);
}

void pistonMotion() {
  if (psx_button_press(PSB_CIRCLE)) {
    PISTON1A = 1;
    PISTON1B = 0;
    Serial.print("Piston out");
  }
  else if (psx_button_press(PSB_SQUARE)) {
    PISTON1A = 0;
    PISTON1B = 0;
    Serial.print("Piston in");
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


ISR(INT6_vect) {
  LMGA = 0;
  LMGB = 0;
  LMGBRAKE = 0;
  Serial.println("UP BRAKE");

}

ISR(INT7_vect) {
  LMGA = 0;
  LMGB = 0;
  LMGBRAKE = 0;
  Serial.println("DOWN BRAKE");
}
