//Base Motors #defines Start
#define MOTORFF   REGISTER_BIT(PORTC,1)		//FF-1, FB-0 = anticlockwise if seen in direction of forward
#define MOTORFB   REGISTER_BIT(PORTC,0)
#define PWMF      OCR0

#define MOTORLF   REGISTER_BIT(PORTC,3)		//LF-1, LB-0 = anticlockwise if seen in direction of left
#define MOTORLB   REGISTER_BIT(PORTC,2)
#define PWML      OCR1A

#define MOTORBF   REGISTER_BIT(PORTC,4)		//same direction as forward
#define MOTORBB   REGISTER_BIT(PORTC,5)
#define PWMB      OCR1B

#define MOTORRF   REGISTER_BIT(PORTC,6)		//same direction as left
#define MOTORRB   REGISTER_BIT(PORTC,7)
#define PWMR      OCR1C
//Base Motors #defines End

//Sensorboard #defines Start
#define MOSI PB2
#define MISO PB3
#define SCLK PB1
#define SS PB0

#define rightSensor 4	//PF4
#define leftSensor 5	//PF5

#define frontSensor 7	 //PF7
#define backSensor 6	 //PF6
//Sensorboard #defines End

//Clamp Piston #defines Start
#define MOTOROC REGISTER_BIT(PORTE,7)
//Clamp Piston #defines End

//Throwing Motors #defines Start
// #define MOTORFR REGISTER_BIT(PORTD,2)	
// #define MOTORBR REGISTER_BIT(PORTD,0)
#define MOTORFR REGISTER_BIT(PORTA,1)
#define MOTORBR REGISTER_BIT(PORTA,0)
//Throwing Motors #defines End
