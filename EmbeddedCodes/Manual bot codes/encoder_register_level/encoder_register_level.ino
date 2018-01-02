#include"avr/io.h"
#include"avr/interrupt.h"


 volatile int ticks=0;
int main()
{

  DDRD=0x00;
  Serial.begin(115200);
  PORTD |= (1<<PD2)|(1<<PD3);
  MCUCR = 0x02;
//  EIMSK = (1<<INT0);
 GICR|= 1<<INT0;
  sei();

  while(1)
  {
    Serial.println(ticks);
  }
  
}

ISR(INT0_vect)
{
  
  if(bit_is_set(PIND,3))
  {
    ticks++;
  }
  else if(bit_is_clear(PIND,3))
  {
    ticks--;
  }
}

