int armPID = 0;
volatile int ticks = 0;
//#define throwingPwm OCR0
int armPWM = 0;
float armKi = 0.001;
float armKp=0.5;
int accumulationOfError = 0;
bool gripperopen = true;
int armError = 0;
void rotate()
{
  PORTD |=(1<<PD6);

  PORTD &=~(1<<PD2);
}

void brake()
{
  PORTD |=(1<<PD0);
  PORTD |=(1<<PD2);
  PORTD &= ~(1<<PD6);  
}
void armInit()
{
  pwm0_init();
//  OCR0 = 180;
  DDRD|=(1<PD0)|(1<<PD2)|(1<<PD6)|(1<<PD7); // motor dir and brake o/p
  PORTD|=(1<<PD6)|(1<<PD1)|(1<<PD3)|(1<<PD7); //brake is low and D1,D3 pulled high
  PORTE &= ~(1<<PE7);
  DDRE|=(PE6)|(1<<PE7); // gripper dir output
  PORTE |= (1 << PE4) | (1 << PE5); //encoder pulled high
  Serial.begin(115200);
  EICRB |= (1 << ISC41); //falling edge
  EIMSK |= (1 << INT4); // interrupt on E4
  PORTD|=(1<<2);
  sei();  
}

void armRotation()
{
  while (1)
  {
   brake();
     if(bit_is_clear(PIND,1))
     {
      while(ticks%600<250)
      {
      armError = 300 - (ticks%600);
      accumulationOfError += armError;
      Serial.print(armError);Serial.print("\t");
      armPID = armKp*armError + armKi*accumulationOfError;
      
      Serial.print(armPID);Serial.print("\t");
      armPWM = map(armPID,0,300,20,120);
      Serial.println();
      OCR0 = armPWM;
      
      rotate();
      }
      brake();
     }

      
  }      
}


