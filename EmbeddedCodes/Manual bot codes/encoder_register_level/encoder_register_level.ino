#include"avr/io.h"
#include"avr/interrupt.h"


 volatile int ticks=0;
int main()
{
  DDRD=0b00001000;
  DDRB|=(1<<PD2)
//=======
  Serial.begin(115200);
  DDRC=0xFF;
  
  PORTD |=(1<<PD3);
  PORTD |(1<<PD2);
  MCUCR = 0x08;
  //EIMSK = (1<<INT0);
 GICR|= 1<<INT0;
  sei();

  while(1)
  {
  /*PORTC=0xFF;
  delay(1000);
  }*/
  
}
}

ISR(INT0_vect)
{
  //Serial.print("Enable");
  if(bit_is_set(PIND,3))
  {
    ticks++;
    Serial.println(ticks);
  }
  else if(bit_is_clear(PIND,3))
  {
    ticks--;
    Serial.println(ticks);
  }
}

