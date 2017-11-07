#include <IO16.h>
#include <SRA16.h>

//
//void setup()
//{
//
//}
//
//void loop()
//{
//  {
//    if(bit_is_clear(PIND,2))
//    {
//      bot_forward();
//    }
//    else
//    {
//      bot_spot_right();
//    }
//  }   
//}

//
int main(void) {
  DDRD|=(1<<PD1)|(1<<PD0);
  DDRD&=~(1<<PD2);
  PORTD |= (1<<PD0);
  DDRC = 0xFF;
  PORTC = 0x00;
  
  pwm1_init();
  set_pwm1a(399);
  set_pwm1b(399);

//
//while(1)
//{
//  
////PORTD |=(1<<PD0);
////PORTD &= ~(1<<PD1);
////  
//if(bit_is_set(PIND,2))
//{
//  PORTC=0xFF;
// }
// else
// PORTC=0x00;
//}

while(1)
 {
    if(bit_is_clear(PIND,2))
    {
      bot_forward();
    }
    else
    {
      bot_spot_right();
    }
  } 
}
