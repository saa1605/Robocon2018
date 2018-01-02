#include"avr/io.h"
#include"avr/interrupt.h"


 volatile int ticks=0;
int main()
{

<<<<<<< HEAD
  DDRD=0b00000110;
=======
  DDRD=0x00;
>>>>>>> 3b6ab28f5d6b8dc1fc057e31e2e7a70c52fd19df
  Serial.begin(115200);
  PORTD |= (1<<PD2)|(1<<PD3);
  MCUCR = 0x11;
//  EIMSK = (1<<INT0);
 GICR|= 1<<INT0;
  sei();

  while(1)
  {
    Serial.print(digitalRead(PIND2));
//    Serial.println(ticks);
  }
  
}

ISR(INT0_vect)
{
  Serial.print("Enable");
  if(bit_is_set(PIND,3))
  {
    ticks++;
  }
  else if(bit_is_clear(PIND,3))
  {
    ticks--;
  }
}

