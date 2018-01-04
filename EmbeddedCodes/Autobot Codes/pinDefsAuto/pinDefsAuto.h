
#define MOTORFA   REGISTER_BIT(PORTC,0)
#define MOTORFB   REGISTER_BIT(PORTC,1)
#define PWMF      OCR0

#define MOTORLA   REGISTER_BIT(PORTC,3)
#define MOTORLB   REGISTER_BIT(PORTC,2)
#define PWML      OCR1A

#define MOTORBA   REGISTER_BIT(PORTC,5)
#define MOTORBB   REGISTER_BIT(PORTC,4)
#define PWMB      OCR1B

#define MOTORRA   REGISTER_BIT(PORTC,7)
#define MOTORRB   REGISTER_BIT(PORTC,6)
#define PWMR      OCR1C

#define MOSI PB2
#define MISO PB3
#define SCLK PB1
#define SS PB0

#define frontSlaveSelect PF3
#define backSlaveSelect PF2