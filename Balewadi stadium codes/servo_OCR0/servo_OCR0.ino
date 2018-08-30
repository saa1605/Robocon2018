void setup()
{
  DDRA = 0x00;
  PORTA = 0xFF;

  DDRC = 0xFF;
  PORTC = 0x00;
  
  servoInit();
}

void loop()
{
  if(bit_is_clear(PINA,4))
  {
    flapOpen();
    PORTC = 0xFF;
  }
  else if(bit_is_clear(PINA,7))
  {
    flapClose();
    PORTC = = 0x00;
  }
}

void servoInit()
{
  DDRA |= (1 << 0); //SERVO INITIALIZATION

  pwm0_init();
  
  TCCR3A = 0b10101010;
  TCCR3B = 0b00010101;
  ICR3H  = 0x01;
  ICR3L  = 0x35;
  DDRE |= (1 << PE3);
  DDRE |= (1 << PE4);
}

void flapOpen()
{
  OCR0 = 5;
  _delay_ms(200);
}

void flapClose()
{
  OCR0 = 11;
  _delay_ms(200);
}

