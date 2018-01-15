#include <pinDefsManual.h>

#include <io128.h>
#include <sra128.h>

#include"avr/io.h"
#include"avr/interrupt.h"

#include <psx.h>
#include <psx2.h>

#define joystickBuffer 35

int LX = 0, LY = 0, RX = 0, RY = 0;

boolean doneFlag = true;

int sLeft, sRight, sFront, sBack;
boolean slowFlag = 1;
int pwm = 666;
void setup()
{
  psx_init(&PORTG, 0, &PORTG, 3, &PORTG, 1, &PORTG, 2);
  DDRC = 0xFF;
  PORTC = 0xFF;
  DDRB |= 0b11110000;
  baseMotorsInitialize();
  LMGInitialize();
  pistonInitialize();
  DDRD &= ~(1 << BSUP);
  DDRD &= ~(1 << BSDOWN);
  PORTD |= (1 << BSUP);
  PORTD |= (1 << BSDOWN);
  //  MCUCR = 0x02;
  EICRB |= (1 << ISC41) | (1 << ISC51); //FOR INTERRUPT 2 AND 3(IF CHANGING CHANGE
  EIMSK |= (1 << INT4) | (1 << INT5);; //VECTORS AND PIN DEFS
  sei();
  DDRA |= (1 << 0); //SERVO INITIALIZATION
  PORTA &= ~(1 << 0);
  Serial.begin(9600);


}
void loop()
{
  //  pwm1_init();
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
    if (psx_button_click(PSB_START, flag_START))
    {
      servoInit();
      flapOpen();
      zeroAllReg();
      pwm1_init();
    }

    if (psx_button_click(PSB_SELECT, flag_SELECT))
    {
      servoInit();
      flapClose();
      PORTA &= ~(1 << 0);
      zeroAllReg();
      pwm1_init();
    }
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
    if (BSUP) {
      LMGA = 1;
      LMGB = 0;
      LMGBRAKE = 1;
      Serial.println("Lmg up");
    }
    else if (!BSUP) {
      LMGA = 0;
      LMGB = 0;
      LMGBRAKE = 0;
      Serial.println("up brake");

    }
  }
  else if (psx_button_press(PSB_PAD_DOWN))
  {
    if (BSDOWN) {
      LMGA = 0;
      LMGB = 1;
      LMGBRAKE = 1;
      Serial.println("LMG down");
    }

    else if (!BSDOWN) {
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
  int stickLY = 127 - (psx_stick(PSS_LY));
  int stickRX = (psx_stick(PSS_RX)) - 127;
  int stickRY =  127 - (psx_stick(PSS_RY)) ;

  LX = stickLX;
  LY = stickLY;
  RX = stickRX;
  RY = stickRY;



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
  //  Serial.print(stickRX);
  //  Serial.print("\t");
  //  Serial.print(stickLY);
  //  Serial.print("\t");
  //  Serial.print(sFront);
  //  Serial.print("\t");
  //  Serial.print(sBack);
  //  Serial.print("\t");
  //  Serial.print(sLeft);
  //  Serial.print("\t");
  //  Serial.println(sRight);
  //  Serial.print("\t");
  //  Serial.print(PWMF);
  //  Serial.print("\t");
  //  Serial.print(PWMB);
  //  Serial.print("\t");
  //  Serial.print(PWML);
  //  Serial.print("\t");
  //  Serial.println(PWMR);

  if (psx_button_press(PSB_R2)) {
    slowFlag = 1;
  }
  else if (psx_button_press(PSB_L2)) {
    slowFlag = 0;
  }

  if (sFront < -30)
  {
    sFront *= (-1);
    MOTORFA = 0;
    MOTORFB = 1;
    if (sFront > 666)
      sFront = 666;
    if (slowFlag == 0) {
      PWMF = map(sFront, 0, 666, 0, 255);
    }
    else {
      PWMF = map(sFront, 0, 666, 0, 64);
    }
    //    PWMF = sFront;
  }
  else if (sFront > 30)
  {
    MOTORFA = 1;
    MOTORFB = 0;
    if (sFront > 666)
      sFront = 666;
    //    PWMF = sFront;
    if (slowFlag == 0) {
      PWMF = map(sFront, 0, 666, 0, 255);
    }
    else {
      PWMF = map(sFront, 0, 666, 0, 64);
    }
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
    if (slowFlag == 0) {
      PWMB = sBack;
    }
    else {
      PWMB = map(sBack, 0, 666, 0, 167);
    }
  }
  else if (sBack > 30)
  {
    MOTORBA = 1;
    MOTORBB = 0;
    if (sBack > 666)
      sBack = 666;
    if (slowFlag == 0) {
      PWMB = sBack;
    }
    else {
      PWMB = map(sBack, 0, 666, 0, 167);
    }
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
    if (slowFlag == 0) {
      PWML = sLeft;
    }
    else {
      PWML = map(sLeft, 0, 666, 0, 167);
    }
  }
  else if (sLeft > 30)
  {
    MOTORLA = 1;
    MOTORLB = 0;
    if (sLeft > 666)
      sLeft = 666;
    if (slowFlag == 0) {
      PWML = sLeft;
    }
    else {
      PWML = map(sLeft, 0, 666, 0, 167);
    }
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
    if (slowFlag == 0) {
      PWMR = sRight;
    }
    else {
      PWMR = map(sRight, 0, 666, 0, 167);
    }
  }
  else if (sRight > 30)
  {
    MOTORRA = 1;
    MOTORRB = 0;
    if (sRight > 666)
      sRight = 666;
    if (slowFlag == 0) {
      PWMR = sRight;
    }
    else {
      PWMR = map(sRight, 0, 666, 0, 167);
    }
  }
  else
  {
    MOTORRA = 1;
    MOTORRB = 1;
    PWMR = 666;
  }

  if (psx_button_press(PSB_R1)) {
    if (slowFlag == 0) {
      MOTORFA = 1;
      MOTORFB = 0;
      PWMF = 63;

      MOTORLA = 1;
      MOTORLB = 0;
      PWML = 166;

      MOTORBA = 0;
      MOTORBB = 1;
      PWMB = 166;

      MOTORRA = 0;
      MOTORRB = 1;
      PWMR = 166;
    }
    else {
      MOTORFA = 1;
      MOTORFB = 0;
      PWMF = 32;

      MOTORLA = 1;
      MOTORLB = 0;
      PWML = 83;

      MOTORBA = 0;
      MOTORBB = 1;
      PWMB = 83;

      MOTORRA = 0;
      MOTORRB = 1;
      PWMR = 83;
    }

    Serial.println("Spot right");
  }

  if (psx_button_press(PSB_L1)) {
    if (slowFlag == 0) {
      MOTORFA = 0;
      MOTORFB = 1;
      PWMF = 63; //HALF PWM FOR SLOW SPOT TURN

      MOTORLA = 0;
      MOTORLB = 1;
      PWML = 166;

      MOTORBA = 1;
      MOTORBB = 0;
      PWMB = 166;

      MOTORRA = 1;
      MOTORRB = 0;
      PWMR = 166;
    }
    else {
      MOTORFA = 0;
      MOTORFB = 1;
      PWMF = 32; //HALF PWM FOR SLOW SPOT TURN

      MOTORLA = 0;
      MOTORLB = 1;
      PWML = 83;

      MOTORBA = 1;
      MOTORBB = 0;
      PWMB = 83;

      MOTORRA = 1;
      MOTORRB = 0;
      PWMR = 83;
    }

    Serial.println("Spot right");

  }
}



void botKill() {
  LMGA = 0;
  LMGB = 0;
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

void flapOpen()
{
  OCR2 = 10;
  _delay_ms(150);
}

void flapClose()
{
  OCR2 = 20;
  _delay_ms(150);
}
void servoInit()
{
  //  DDRA|=(1<<0); //SERVO INITIALIZATION

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


  PORTA |= (1 << 0);
  DDRB |= (1 << 7);
  TCCR2 = 0b01101101;
  TCNT2 = 0x00;
  OCR2 = 0x00;

}
ISR(INT4_vect) {
  LMGA = 0;
  LMGB = 0;
  LMGBRAKE = 0;
  Serial.println("UP");

}

ISR(INT5_vect) {
  LMGA = 0;
  LMGB = 0;
  LMGBRAKE = 0;
  Serial.println("DOWN");
}

void zeroAllReg()
{
  PWM2_DIR = 0;
  TCCR2 = 0x00; //stop
  TCNT2 = 0x00; //setup
  OCR2 = 0x00;
  TCCR2 = 0x00; //start Timer //fPWM = 1kHZ; prescaler = 8; No ICR
}

