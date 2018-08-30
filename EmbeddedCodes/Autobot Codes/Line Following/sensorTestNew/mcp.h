void spiMasterInit(void)
{
  DDRB |= (1 << MOSI) | (1 << SCLK) | (1 << SS);
  PORTB |= (1 << SS);
  DDRF |= (1 << PF0) | (1 << PF1) | (1 << PF2) | (1 << PF3);
  PORTF |= (1 << PF0) | (1 << PF1) | (1 << PF2) | (1 << PF3);
  SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0)| (1 << SPR1);
}

unsigned char spiTransfer(unsigned char data)
{
  SPDR = data;
 
  while(!(SPSR & (1 << SPIF)));
  return(SPDR); 
}

int readAdc(int channel)
{
//  uint8_t x = 0xFF;
  uint8_t LSBData;
  uint8_t MSBData;
  uint16_t adcValue;
  
  byte channelbit = channel << 6;
  
  if(channel < 4)
    spiTransfer(0b00000110);
  else
   spiTransfer(0b00000111);
  
  MSBData = spiTransfer(channelbit);
  LSBData = spiTransfer(0xFF); //x
  MSBData = MSBData & 0b00001111;
  adcValue = (MSBData << 8) | LSBData;
  return (adcValue);
}

int getSensorReading(int slaveSelect, int sensorNum)
{
    int sensorVal;
    PORTF &= ~(1 << slaveSelect);
    sensorVal = readAdc(sensorNum);
    PORTF |= (1 << slaveSelect);
    return (sensorVal);
}
