#include <IO16.h>
#include <SRA16.h>

int main(void)
{
  DDRB |= (1 << PB1) | (1 << PB0);
  DDRB &= ~(1 << PB2);
  PORTB |= (1 << PB0);
  DDRD |= 0xF0;
  PORTD |= 0x0F;
  DDRC = 0xFF;
  PORTC = 0x00;

  pwm1_init();
  set_pwm1a(399);
  set_pwm1b(399);
  bool flag = true;
  //  bool currentState = false;
  while (1)
  {
    if (bit_is_clear(PINB, 2))
    {
      if (flag)
      {
        bot_forward();
        _delay_ms(300);
        flag = false;
      }
    }
//    else if (bit_is_clear(PIND, 1))
//    {
//      if (flag)
//      {
//        flag = false;
//      }
//      else
//        flag = true;
//    }
    else if (bit_is_clear(PIND, 3))
    {
      bot_backward();
      _delay_ms(151);
      flag = true;
    }
    //    else if (bit_is_clear(PIND,0))
    //    {
    //      flag = true;
    //    }
    else
    {
      bot_stop();
    }
  }
}
