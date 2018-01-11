//Base Motors #defines Start
#define MOTORFF   REGISTER_BIT(PORTC,0)
#define MOTORFB   REGISTER_BIT(PORTC,1)
#define PWMF      OCR0

#define MOTORLF   REGISTER_BIT(PORTC,2)
#define MOTORLB   REGISTER_BIT(PORTC,3)
#define PWML      OCR1A

#define MOTORBF   REGISTER_BIT(PORTC,4)
#define MOTORBB   REGISTER_BIT(PORTC,5)
#define PWMB      OCR1B

#define MOTORRF   REGISTER_BIT(PORTC,6)
#define MOTORRB   REGISTER_BIT(PORTC,7)
#define PWMR      OCR1C
//Base Motors #defines End

//Sensorboard #defines Start
#define MOSI PB2
#define MISO PB3
#define SCLK PB1
#define SS PB0

#define rightSensor 4	//PF7
#define leftSensor 5	//PF6

#define frontSensor 7	 //PF7
#define backSensor 6	 //PF6
//Sensorboard #defines End

//Throwing Motor and Clamp Piston #defines Start
#define MOTOROC REGISTER_BIT(PORTE,6)
#define MOTORCC REGISTER_BIT(PORTE,7)
#define MOTORFR REGISTER_BIT(PORTD,0)
#define MOTORBR REGISTER_BIT(PORTD,2)
//Throwing Motors and Clamp Piston #defines End