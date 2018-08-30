#include <io128.h>
#include <sra128.h>

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

int front[4];
int back[4];
int mid[4];
int thresh=150;

void setup() {
  // put your setup code here, to run once:
  adc_init();
}


void loop() {
  // put your main code here, to run repeatedly:
  getValues();

  
}

void getValues(){
  for (int i = 0; i < 4; i++) {
    if (adc_start(i) < thresh) {
      front[i] = 0;
    }

    else {
      front[i] = 1;
    }

  }

  for (int i = 4; i < 8; i++) {
    if (adc_start(i) < thresh) {
      mid[i-4] = 0;
    }

    else {
      mid[i-4] = 1;
    }

  }

   for (int i = 8; i < 12; i++) {
    if (adc_start(i) < thresh) {
      back[i-8] = 0;
    }

    else {
      back[i-8] = 1;
    }

  }   
}

  void allign(){
    int frontError=(-3*front[0])+(-1*front[1])+(1*front[2])+(3*front[3]);
    int midError=(-3*mid[0])+(-1*MID[1])+(1*mid[2])+(3*mid[3]);
    int backError=(-3*back[0])+(-1*back[1])+(1*back[2])+(3*back[3]); 

     int error =frontError-backError; 

     if(midError==0&&mid[2]==1){
      if(error>0){
        bot_right();
      }
      else if(error<0){
        bot_left();
      }
      
     }

     else {
      if(frontError>backError({
        
      }
     }
  }

