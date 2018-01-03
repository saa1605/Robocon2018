int16_t convertEncoderBit()
{
 return MSBData<<8 | LSBData;
}

int16_t getEncoderReadings1()
{
  PORTD &= ~ (1 << PD6);
  MSBData = spiTransfer(x);
  LSBData = spiTransfer(x);
  PORTD |= (1 << PD6);
  return convertEncoderBit();
}

int16_t getEncoderReadings2()
{
  PORTD &= ~ (1 << PD7);
  MSBData = spiTransfer(x);
  LSBData = spiTransfer(x);
  PORTD |= (1 << PD7);
  return convertEncoderBit();
}

int getEncoderPosition()
{
  int16_t encoderReading1 = getEncoderReadings1();
  int16_t encoderReading2 = getEncoderReadings2();
  return encoderReading1 - encoderReading2;
}
