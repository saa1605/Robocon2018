#include"avr/io.h"
#include"avr/interrupt.h"


 volatile int ticks=0;
int main()
{
  DDRD=0b00001000;
  DDRD&=~(1<<PD2);
  DDRD&=~(1<<PD3);
  DDRD&=~(1<<PD4);
//=======
  Serial.begin(115200);
  DDRC=0xFF;
  PORTD|=(1<<0)|(1<<1);
  PORTD |=(1<<PD3);
  PORTD |(1<<PD2);
  PORTD |=(1<<PD4);
  MCUCR |=(1<<ISC01);
  //EIMSK = (1<<INT0);
  GICR|= (1<<INT0);
  GICR|=(1<<INT2);
  sei();

  while(1)
  {
    if(ticks==0){
        Serial.println(ticks);
        PORTC = 0xff;
      }
    }
    if(bit_is_clear(PIND,0))
    {
      PORTD &=~(1<<3);
       PORTD|=(1<<4); 
      PORTD|=(1<<3); 
    }
    if(bit_is_clear(PIND,1))
    {
      PORTD|=(1<<2);    
      PORTD &=~(1<<2);
    }
}

ISR(INT1_vect)
{
  //Serial.print("Enable");
  if(bit_is_set(PIND,4))
  {
    ticks++;
//    Serial.println(ticks);
  }
  else if(bit_is_clear(PIND,4))
  {
    ticks--;
//    Serial.println(ticks);
  }
  if(bit_is_set(PIND,2))
  {
    ticks = 0;  
  }
}
