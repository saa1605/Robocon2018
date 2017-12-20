#ifndef pinDefsForNewBoardAtmega128Robocon2017_h
#define pinDefsForNewBoardAtmega128Robocon2017_h

#include <io128.h>
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#ifndef _IO_REG
#define _IO_REG
typedef struct
{
  unsigned int bit0:1;
  unsigned int bit1:1;
  unsigned int bit2:1;
  unsigned int bit3:1;
  unsigned int bit4:1;
  unsigned int bit5:1;
  unsigned int bit6:1;
  unsigned int bit7:1;
} _io_reg;
#endif

#define FRONTSENSORSCS				2  //CS1
#define BACKSENSORSCS				3  //CS2

#define LEFTSENSORSCS				2  //CS1
#define RIGHTSENSORSCS				3  //CS2

#define SENSORSANDSWITCHESCS		4  //CS3

#define MOTORFA				    	REGISTER_BIT(PORTC,1)
#define MOTORFB    					REGISTER_BIT(PORTC,0)
#define PWMF						OCR1A

#define MOTORLA    					REGISTER_BIT(PORTC,3)
#define MOTORLB    					REGISTER_BIT(PORTC,2)
#define PWML						OCR1B

#define MOTORBA    					REGISTER_BIT(PORTC,5)
#define MOTORBB 			   		REGISTER_BIT(PORTC,4)
#define PWMB			        	OCR3A

#define MOTORRA    					REGISTER_BIT(PORTC,7)
#define MOTORRB    					REGISTER_BIT(PORTC,6)
#define PWMR						OCR3B

#define ALPHAA 		   				REGISTER_BIT(PORTA,5)
#define ALPHAB    					REGISTER_BIT(PORTA,4)
#define ALPHAA_DIR    				REGISTER_BIT(DDRA,5)
#define ALPHAB_DIR    				REGISTER_BIT(DDRA,4)
#define ALPHAPWM					OCR3C
#define ALPHAPWM_DIR				REGISTER_BIT(DDRE,4)

//#define ALPHABRAKE    	        	REGISTER_BIT(PORTF,5)
//#define ALPHABRAKE_DIR    			REGISTER_BIT(DDRF,5)


#define SHOOTINGMOTORA    			REGISTER_BIT(PORTD,6)
#define SHOOTINGMOTORA_DIR    		REGISTER_BIT(DDRD,6)
#define SHOOTINGMOTORPWM			OCR1C

#define SHOOTINGDIGI_DIR    		REGISTER_BIT(DDRF,1)
#define SHOOTINGDIGI      			REGISTER_BIT(PORTF,1)

#define LOADINGMOTORA    			REGISTER_BIT(PORTD,4)
#define LOADINGMOTORB    			REGISTER_BIT(PORTD,5)
#define LOADINGMOTORA_DIR    		REGISTER_BIT(DDRD,4)
#define LOADINGMOTORB_DIR    		REGISTER_BIT(DDRD,5)
#define LOADINGMOTORPWM 			OCR0
#define LOADINGMOTORPWM_DIR    		REGISTER_BIT(DDRB,4)

#define LOADINGDIGI_DIR    			REGISTER_BIT(DDRF,0)
#define LOADINGDIGI      			REGISTER_BIT(PORTF,0)
#define LOADINGDIGIINT_DIR 			REGISTER_BIT(DDRD,2)
#define LOADINGDIGIINT    			REGISTER_BIT(PORTD,2)

// #define BUMPSWITCHALPHAUP   		REGISTER_BIT(PORTA, 2)
// #define BUMPSWITCHALPHADOWN   		REGISTER_BIT(PORTA, 2)
// #define BUMPSWITCHALPHAUP_DIR   	REGISTER_BIT(DDRA,2)
// #define BUMPSWITCHALPHADOWN_DIR   	REGISTER_BIT(DDRA,2)

#define BUMPSWITCHALPHAUP   		REGISTER_BIT(PORTA, 2)
#define BUMPSWITCHALPHADOWN   		REGISTER_BIT(PORTA, 3)
#define BUMPSWITCHALPHAUP_DIR   	REGISTER_BIT(DDRA,2)
#define BUMPSWITCHALPHADOWN_DIR   	REGISTER_BIT(DDRA,3)

#define LED0         				REGISTER_BIT(PORTD, 3)
#define LED1         				REGISTER_BIT(PORTD, 2)
#define LED2         				REGISTER_BIT(PORTA, 1)
#define LED3         				REGISTER_BIT(PORTA, 0)
#define LED0_DIR     				REGISTER_BIT(DDRD,3)
#define LED1_DIR     				REGISTER_BIT(DDRD,2)
#define LED2_DIR     				REGISTER_BIT(DDRA,1)
#define LED3_DIR     				REGISTER_BIT(DDRA,0)

#endif