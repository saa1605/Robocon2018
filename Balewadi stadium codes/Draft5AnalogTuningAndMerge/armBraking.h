int prevArmError = 0;
int armPID = 0;
volatile long int ticks = 0;
//volatile int ticks = 0;
int armPWM = 0;
float armKp = 0.1;
float armKi = 0.05;
float armKd = 0.1;
int accumulationOfError = 0;
int armError = 0;

void rotateBack()
{
  PORTD |= (1 << PD6);
  PORTD &= ~(1 << PD0);
  PORTD |= (1 << PD2);
}

void rotateFront()
{
  PORTD |= (1 << PD6);
  PORTD &= ~(1 << PD2);
  PORTD |= (1 << PD0);
}

void autoBotBrake()
{
  PORTD |= (1 << PD0);
  PORTD |= (1 << PD2);
  PORTD &= ~(1 << PD6);
}

void alignArm()
{
  while((-5<ticks<5)&&bit_is_set(PINF,0))
  {
    armError = ticks;
    Serial.print(armError); Serial.print("\t");

//        accumulationOfError += armError;
    armPID = abs(armKp * armError); //+ armKi*accumulationOfError;
    Serial.print(armPID); Serial.print("\t");

    armPWM = map(armPID, 0, 300, 40, 120);
//        armPWM = constrain(armPWM,40,120);
    Serial.print(armPWM); Serial.print("\t");
    OCR0 = armPWM;
    prevArmError = armError;
    if (armError > 0)
    {
      rotateBack();
    }
    else if (armError < 0)
    {
      rotateFront();
    }
    Serial.println();
  }
  autoBotBrake();
}


