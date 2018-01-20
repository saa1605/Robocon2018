 #include"avr/io.h"
#include"avr/interrupt.h"


 volatile int ticks=0;
int main()
{
  PORTE |= (1<<PE4)|(1<<PE5);
  Serial.begin(115200);
  EICRB = (1<<ISC51);
  EIMSK = (1<<INT5);
// GICR|= 1<<INT0;
  sei();

  while(1)
  {
    Serial.println(ticks);
    Serial.print("\t");
    if(bit_is_clear(PINE,4))Serial.println("0");
    else Serial.println("1");
  }
  
}

ISR(INT5_vect)
{
  
  if(bit_is_set(PINE,4))
  {
    ticks++;
  }
  else if(bit_is_clear(PINE,4))
  {
    ticks--;
  }
}

