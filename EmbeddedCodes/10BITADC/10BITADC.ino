#include <IO16.h>
#include <SRA16.h>



char adcResult[4];
int main(void)
{
  DDRA=0x00;

  Serial.begin(9600);
  ADCSRA |= (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2); //Select PRESCALER
  ADMUX |= (1<<ADLAR);//MODE OF STORAGE 
  ADMUX |= (1<<REFS0); //SELECT REFRENCE VOLTAGE
  ADCSRA |= (1<<ADIE) | (1<<ADEN);//ENABLE INTERRUPT AND ENABLE ADC

  sei(); //ENABLE GLOBAL INTERRUPT

  ADCSRA |= (1<<ADSC); //START CONVERSION
  while(1)
  {
    
  }
}

ISR(ADC_vect)
{
  uint8_t theLow=ADCL;
  uint16_t result_10 = ADCH <<2| theLow>>6;
  itoa(result_10,adcResult,10);
  Serial.println(adcResult);i7777777
  ADCSRA |= (1<<ADSC);
  
}

