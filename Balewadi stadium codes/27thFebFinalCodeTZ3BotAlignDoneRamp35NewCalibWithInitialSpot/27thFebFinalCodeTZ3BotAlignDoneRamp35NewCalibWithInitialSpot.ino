/**********************
 * code working for 23rd feb
Changes for test
1. buttos on G0, G1, G2, G3
2. G0 Sensor test
3. G1 base motors Test
4. G2 line follow test
5. G3 throw test
**********************/
#include <io128.h>
#include <sra128.h>
#include "avr/io.h"
#include "avr/interrupt.h"
#include <pinDefsAutoNew.h>
#include "mcp.h"
#include <math.h>
#include <Wire.h>
#include <VL53L0X.h>

#define debug   //comment for NO //Serial.prints

VL53L0X sensor; 

bool tz1 = false;   //false - TZ1 and true - TZ2
bool rongBay = false;   //false - TZ2 and true - TZ3
int onlineThresh = 500;
int maxErrorPWM = 250, normalMaxErrorPWM = 260, alignMaxErrorPWM = 290;

int transferDone = false;
int sureCount = 0, unsureCount = 0, usCount = 0;

volatile long int ticks = 0;
volatile int pidTicks = 0, throwCount = 0;

bool firstTimeTZ2 = true, transferFlag = true;

bool firstTimeAlign = true, aligned = false, stopAlign = false, alignTemp = true;
int armError = 0, prevArmError = 0, integralError = 0, diffError = 0;
int armKp = 5, armKi = 0, armKd = 10;

int armPID = 0, armPWM = 0;
int pidCount = 0;

int throwingPWM = 0;

int pidMode = 1, toggle = 0;  //for pid tuning

#define numOfSensors 8

float alignKPError = 0.20, alignKDError = 0, alignKPRotation = 0.30, alignKDRotation = 0, sensitiveRot = 0.45;         ////alignRot:0.32
int alignOpt = 1500;

float fastKPError = 0.42, fastKDError = 0, fastKPRotation = 0.25, fastKDRotation = 0;  ////fastKPRotation 0..25
int fastOpt = 15000;
int frontOnline = 0, backOnline = 0, leftOnline = 0, rightOnline = 0;



//calibrated values of 26th Feb
//int sensorMinLeft[numOfSensors] = {1069, 628, 611, 594, 603, 626, 624, 1832};
//int sensorMaxLeft[numOfSensors] = {2909, 2631, 2629, 2334, 2527, 2668, 2651, 3037};
//int sensorMinRight[numOfSensors] = {1757, 1842, 2064, 1896, 1841, 1842, 1352, 1933};
//int sensorMaxRight[numOfSensors] = {3043, 3043, 3043, 3044, 3038, 3039, 2826, 3040};
//int sensorMinFront[numOfSensors] = {1713, 940, 1005, 1548, 665, 981, 603, 2108};
//int sensorMaxFront[numOfSensors] = {2925, 2618, 2751, 3035, 2148, 2909, 2195, 3049};
//int sensorMinBack[numOfSensors] = {1711, 1010, 906, 726, 1439, 813, 1653, 661};
//int sensorMaxBack[numOfSensors] = {3052, 2940, 2887, 2738, 3050, 2798, 3051, 2326};

//Calibrated values 28th feb
int sensorMinLeft[numOfSensors] = {884, 1847, 1493, 1571, 1222, 1072, 1677, 2023};
int sensorMaxLeft[numOfSensors] = {2756, 2878, 2877, 2877, 2874, 2670, 2873, 2873};

int sensorMinFront[numOfSensors] = {285, 1358, 1146, 865, 623, 990, 969, 1820};
int sensorMaxFront[numOfSensors] = {1679, 2621, 2550, 2165, 2137, 2397, 2579, 2886};

int sensorMinRight[numOfSensors] = {1844, 1933, 2082, 1920, 1919, 1933, 1525, 1981};
int sensorMaxRight[numOfSensors] = {2827, 2886, 2887, 2887, 2869, 2870, 2622, 2871};

//int sensorMinBack[numOfSensors] = {2115, 1270, 993, 1311, 1723, 1107, 1779, 391};
//int sensorMaxBack[numOfSensors] = {2901, 2605, 2585, 2902, 2885, 2493, 2809, 1950};


//new
int sensorMinBack[numOfSensors] = {2283, 1698, 1095, 274, 987, 1341, 1380, 1579};
int sensorMaxBack[numOfSensors] = {2899, 2899, 2800, 2034, 2817, 2915, 2861, 2916};


int maxPWM = 666, alignMaxPWM = 220, fastMaxPWM = 666, slowAlignPWM = 190, mediumAlignPWM = 330, botAlignPWM = 100, rampSlowestPWM = 130;

int errorRight = 0, errorLeft = 0, errorFront = 0, errorBack = 0, error = 0, prevError = 0;          
int rotation = 0, prevRotation = 0;

int sensorRawRight[numOfSensors], sensorValRight[numOfSensors], sensorRawLeft[numOfSensors], sensorValLeft[numOfSensors];
int sensorRawFront[numOfSensors], sensorValFront[numOfSensors], sensorRawBack[numOfSensors], sensorValBack[numOfSensors];

int turnFlag = -1;  //0-Left  1-Right  2-Forward  3-Back
int firstTurn = -1;   //0-after 1st turn and 1-before                                  

float kPError = fastKPError, kDError = fastKDError, kPRotation = fastKPRotation, kDRotation = fastKDRotation;
int opt = fastOpt;

int juncCount = 0, shortPlus = 0;
bool juncFlag = true, shortPlusFlag = true, frontPlusFlag = true;
int frontPlus = 0; 

int sLeft, sRight, sFront, sBack;
int32_t distance = 0, minDistanceThreshold = 55, maxDistanceThreshold = 265;   //for VL53L0X (TOF)

int throws = 0;   //to count the no of throws
//int TZ1Ticks = -219, TZ2Ticks = -221, TZ3Ticks = -215;    //-229, -223, -212
//int TZ1ThrowCount = 1, TZ2ThrowCount = 1, TZ3ThrowCount = 3;    //2 4 5
//int throwPWM1 = 255, throwPWM2 = 255, throwPWM3 = 215;    //255 215 180
//
int maxAlignCount = 4;
int TZ1Ticks = -230, TZ2Ticks = -225, TZ3Ticks = -217;    //-228, -215, -213
int TZ1ThrowCount = 3, TZ2ThrowCount = 4, TZ3ThrowCount = 5;    //2 4 5
int throwPWM1 = 255, throwPWM2 = 255, throwPWM3 = 200;    //255 215 180
int allBreak = false;
// Main loop, read and display data
//-----------------------------------------------------------------------

void setup()
{ 
  #ifdef debug
  Serial.begin(115200);
  #endif
  baseMotorsInit();
  encoderInterruptInit();
  throwingMotorInit();
  clampPistonInit();
  piInterfaceInit();
  spiMasterInit();
 
  pidAlign(0);  //fast line following in the begining
  throwBrake();   //for braking the throwing motor

  DDRA &= ~((1 << PA4)|(1 << PA5)|(1 << PA6)|(1 << PA7));   //for buttons A4567
  PORTA |= ((1 << PA4)|(1 << PA5)|(1 << PA6)|(1 << PA7));   //for buttons A4567

  DDRG &= ~((1 << PG0)|(1 << PG1)|(1 << PG2)|(1 << PG3));   //for buttons G0123
  PORTG |= ((1 << PG0)|(1 << PG1)|(1 << PG2)|(1 << PG3));   //for buttons G0123

  DDRA |= (1 << PA0)|(1 << PA1);   //for LED Flashing and led strip
  PORTA &= ~((1 << PA0)|(1 << PA1));   //Active High
  
  initTOF();
}

void loop()
{
  if(bit_is_clear(PINA,7))    //Full Run
  {
    PORTC = 0xFF;
    _delay_ms(20);
    PORTC = 0x00;
    alignToLineFollow();  
    botKill();
    pidAlign(0);
    PORTC = 0xFF;
    _delay_ms(20);
    PORTC = 0x00;
    turnFlag = 0;
    firstTurn = 1;
    if(transferFlag) 
    {
      openClamp();        //Note
    }
    while(1)
    {
      alignArm();
      lineFollow();
      SerialPrint();
//      if(transferDone == false && firstTurn == 0)
//      {
//      	transferShuttle();
//      }
      if(throws >= 20)
      {
        botKill();
        while(1)
        {
          PORTC = 0xFF;
          _delay_ms(100);
          PORTC = 0x00;
          _delay_ms(100);
        }
      }
    }
  }
  else if(bit_is_clear(PINA,4))
  {
    while(1)
    { 
      distance=readTOF();
      if(distance<minDistanceThreshold)
      {
        openClamp();
      }
      else
      {
        closeClamp();
      }
    }
  }
  else if(bit_is_clear(PINA,5))
  {
    PORTC = 0xFF;
    transferFlag = false;
    firstTimeTZ2 = true;

    PORTA |= (1 << PA1); //LED STRIP ON
    int colorCount = 0;
    while(colorCount < 500)
    {
      if(!TZ2)
        colorCount++;
      else if(!TZ1)
        colorCount++;
      else if(!TZ3)
        colorCount++;
      else
        colorCount = 0;
    }
    PORTA&=~(1 << PA1); //LED STRIP OFF
    
    if(!TZ1)
      tz1 = false;    //TZ1
    else if(!TZ2)
      tz1 = true;    //TZ2
    else if(!TZ3)
    {
      rongBay = true;   //TZ3
      tz1 = true;
    }
    aligned = true;
    PORTC = 0x00;
  }
  else if(bit_is_clear(PINA,6))
  {
//    PORTC |= 0xFF;
//    tz1 = true;
//    firstTimeTZ2 = false;
//   _delay_ms(20);
//   PORTC = 0x00;
   PORTC|= 0xff;
   _delay_ms(300);
   PORTC = 0x00;
   while(bit_is_set(PING,0))
   {
     Serial.print(" Medium = ");Serial.print(mediumAlignPWM);
     Serial.print("  ramp = ");Serial.print(rampSlowestPWM);
     Serial.println("");
    if(bit_is_clear(PINA,4))
    {
      mediumAlignPWM +=10;
      PORTC|= 0xff;
     _delay_ms(1000);
     PORTC = 0x00;
    }
    else if(bit_is_clear(PINA,5))
    {
      rampSlowestPWM +=10;
      PORTC|= 0xff;
     _delay_ms(1000);
     PORTC = 0x00;
    }
    else if(bit_is_clear(PINA,7))
    {
      rampSlowestPWM -=10;
      PORTC|= 0xff;
     _delay_ms(1000);
     PORTC = 0x00;
    }
   }
   PORTC|= 0xff;
   _delay_ms(1000);
   PORTC = 0x00;
  }
  else if(bit_is_clear(PING,0))
  {
    int printVal = 0;
    while(1)
    {
      readSensorFront();
      readSensorBack();
      readSensorLeft();
      readSensorRight();
      
      calcSensorValFront();
      calcSensorValBack();
      calcSensorValLeft();
      calcSensorValRight();

      if(printVal == 1)
      {
        for(int i = 0; i < numOfSensors; i++)
        {
         Serial.print(sensorValFront[i]);Serial.print(' ');
        }
        Serial.print("\t"); 
        for(int i = 0; i < numOfSensors; i++)
        {
         Serial.print(sensorValBack[i]);Serial.print(' ');
        }
        Serial.print("\t");  

        for(int i = 0; i < numOfSensors; i++)
        {
         Serial.print(sensorValLeft[i]);Serial.print(' ');
        }
        Serial.print("\t");
        for(int i = 0; i < numOfSensors; i++)
        {
         Serial.print(sensorValRight[i]);Serial.print(' ');
        }
        Serial.print("\t");
      }
      else if(printVal == 0)
      {
         for(int i = 0; i < numOfSensors; i++)
         {
          Serial.print(sensorRawFront[i]);Serial.print(' ');
         }
         Serial.print("\t"); 
         for(int i = 0; i < numOfSensors; i++)
         {
          Serial.print(sensorRawBack[i]);Serial.print(' ');
         }
         Serial.print("\t");  
        
         for(int i = 0; i < numOfSensors; i++)
         {
          Serial.print(sensorRawLeft[i]);Serial.print(' ');
         }
         Serial.print("\t");
         for(int i = 0; i < numOfSensors; i++)
         {
          Serial.print(sensorRawRight[i]);Serial.print(' ');
         }
         Serial.print("\t");
      }
      Serial.println("");

      if(bit_is_clear(PINA,7))
      {
        for(int i = 0; i<8; i++)
        {
          sensorMinLeft[i]  = 0;
          sensorMaxLeft[i]  = 0;
          sensorMinRight[i] = 0;
          sensorMaxRight[i] = 0;
          
          sensorMinFront[i]  = 0;
          sensorMaxFront[i]  = 0;                        
          sensorMinBack[i] = 0;     
          sensorMaxBack[i] = 0;
        }
        _delay_ms(1000);
        toggle = 0;       //calibrate white
        calibrateLeft();
        calibrateRight();
        calibrateFront();
        calibrateBack();
      
        while(bit_is_set(PINA,7));
        _delay_ms(1000);
        toggle = 1;      //calibrate black
        calibrateLeft();
        calibrateRight();
        calibrateFront();
        calibrateBack();
        Serial.println("Take A Pic of the calibrated values and press A7");
        while(bit_is_set(PINA,7));
        _delay_ms(200);
      }
      else if(bit_is_clear(PINA, 4))
      {
        printVal = 1;
      }
      else if(bit_is_clear(PINA, 5))
      {
        printVal = 0;
      }
    }
  }
  else if(bit_is_clear(PING,1))
  {
    while(1)
    {
        if(bit_is_clear(PINA,4))    //front
        {
          baseUnbrake();
          while(bit_is_clear(PINA,4))
          {
            MOTORRF = 1;
            MOTORRB = 0;
            PWMR = 250;
          
            MOTORLF = 1;
            MOTORLB = 0;
            PWML = 250;
          }
          MOTORRF = 0;
          MOTORRB = 0;

          MOTORLF = 0;
          MOTORLB = 0;
          PWMR = 0;
          PWML = 0;
          
          baseBrake();
        }

        else if(bit_is_clear(PINA,5))   //back
        {
          baseUnbrake();
          while(bit_is_clear(PINA,5))
          {
            MOTORRF = 0;
            MOTORRB = 1;
            PWMR = 250;
          
            MOTORLF = 0;
            MOTORLB = 1;
            PWML = 250;
          }
          MOTORRF = 0;
          MOTORRB = 0;
          
          MOTORLF = 0;
          MOTORLB = 0;
          PWMR = 0;
          PWML = 0;
          
          baseBrake();
        }

        else if(bit_is_clear(PINA,6))   ///right
        {
          baseUnbrake();
          while(bit_is_clear(PINA,6))
          {
            MOTORFF = 1;
            MOTORFB = 0;
            PWMF = 250;
          
            MOTORBF = 1;
            MOTORBB = 0;
            PWMB = 250;
          }
          MOTORFF = 0;
          MOTORFB = 0;
          
          MOTORBF = 0;
          MOTORBB = 0;
          PWMF = 0;
          PWMB = 0;
          
          baseBrake();
        }

        else if(bit_is_clear(PINA,7))  		//left
        {
          baseUnbrake();
          while(bit_is_clear(PINA,7))
          {
            MOTORFF = 0;
            MOTORFB = 1;
            PWMF = 250;
          
            MOTORBF = 0;
            MOTORBB = 1;
            PWMB = 250;
          }
          MOTORFF = 0;
          MOTORFB = 0;
          
          MOTORBF = 0;
          MOTORBB = 0;
          PWMF = 0;
          PWMB = 0;
          
          baseBrake();
        }
    }
  }
  else if(bit_is_clear(PING,2))
  {
    PORTC = 0xFF;
    pidAlign(0);
    while(1)
    {
      calcAllError();
      if(bit_is_clear(PINA,4))
      {
        turnFlag = 0;
        while(bit_is_clear(PINA,4))
        {
         calcAllError();
         calcSpeedToMotors();
         uploadSpeedLineFollow(maxErrorPWM); 
        }
        botKill();
      }
      else if(bit_is_clear(PINA,5))
      {
        turnFlag = 1;
        while(bit_is_clear(PINA,5))
        {
         calcAllError();
         calcSpeedToMotors();
         uploadSpeedLineFollow(maxErrorPWM); 
        }
        botKill();
      }
      else if(bit_is_clear(PINA,6))
      {
        turnFlag = 2;
        while(bit_is_clear(PINA,6))
        {
         calcAllError();
         calcSpeedToMotors();
         uploadSpeedLineFollow(maxErrorPWM); 
        }
        botKill();
      }
      else if(bit_is_clear(PINA,7))
      {
        turnFlag = 3;
        while(bit_is_clear(PINA,7))
        {
         calcAllError();
         calcSpeedToMotors();
         uploadSpeedLineFollow(maxErrorPWM); 
        }
        botKill();
      }
    }
  }
  else if(bit_is_clear(PING,3))
  {
    openClamp();
    Serial.println("");
    Serial.println("Press A7 to alignArm");
    Serial.println("Press G0 for Ticks Calibration");
    while(1)
    {
      if(bit_is_clear(PINA,7))
      {
        alignAfterFailure();
        
        //Serial.println("");
        //Serial.println("Press A7 to closeClamp");
        while(bit_is_set(PINA,7));
        closeClamp();
        _delay_ms(500);
        throwUnbrake();   //unbraking the throwing motor
        backwardRotation();   //to give backwards direction to the motor
        PWMT = map(20, 0, 255, 0, 666);
        while(ticks > -70)                  //////////////*******************************
        {
          PORTC = 0x0F;
        }
        PORTC = 0x00;
        stopRotation();
        PWMT = 0;
        throwBrake();   //for braking the throwing motor
        
        //Serial.print("G1 : TZ1 ");//Serial.print(" G2 : TZ2 ");//Serial.print(" G3 : TZ3");
        //Serial.println("");
        while(1)
        {
          
          if(bit_is_clear(PING,1))
          {
            //Serial.println("TZ1");
            throwAtTZ(1);
            break;
          }
          else if(bit_is_clear(PING,2))
          {
            //Serial.println("TZ2");
            throwAtTZ(2);
            break;
          }
          else if(bit_is_clear(PING,3))
          {
            //Serial.println("TZ3");
            throwAtTZ(3);
            break;
          }
        }
      }
      else if(bit_is_clear(PING, 0))
      {
        PORTC |= 0xff;
        _delay_ms(1000);
        PORTC = 0x00;
        Serial.println("A4 - decrease TZ1 ticks");
        Serial.println("A5 - increase TZ1 ticks");
        Serial.println("A6 - decrease TZ2 ticks");
        Serial.println("A7 - increase TZ2 ticks");
        Serial.println("G0 - decrease TZ3 ticks");
        Serial.println("G1 - increase TZ3 ticks");
        Serial.println("G2 - decrease TZ3 PWM by 5");
        Serial.println("G3 - increase TZ3 PWM by 5");
        Serial.println("To break, press G0 + G1");
        Serial.println("");

        while(1)
        {
	        if(bit_is_clear(PINA,4))
	        {
	        	while(bit_is_clear(PINA, 4));
	        	TZ1Ticks--;
		        Serial.print("Now TZ1Ticks : ");
		        Serial.println(TZ1Ticks);
            PORTC |= 0xff;
            _delay_ms(200);
            PORTC = 0x00;
	        }
	        else if(bit_is_clear(PINA,5))
	        {
	        	while(bit_is_clear(PINA, 5));
	        	TZ1Ticks++;
		        Serial.print("Now TZ1Ticks : ");
		        Serial.println(TZ1Ticks);
             PORTC |= 0xff;
              _delay_ms(200);
              PORTC = 0x00;
	        }
	        else if(bit_is_clear(PINA, 6))
	        {
	        	while(bit_is_clear(PINA, 6));
	        	TZ2Ticks--;
		        Serial.print("Now TZ2Ticks : ");
		        Serial.println(TZ2Ticks);
            PORTC |= 0xff;
            _delay_ms(200);
            PORTC = 0x00;
	        }
	        else if(bit_is_clear(PINA, 7))
	        {
	        	while(bit_is_clear(PINA, 7));
	        	TZ2Ticks++;
		        Serial.print("Now TZ2Ticks : ");
		        Serial.println(TZ2Ticks);
            PORTC |= 0xff;
            _delay_ms(200);
            PORTC = 0x00;
	        }
	        else if(bit_is_clear(PING, 0))
	        {
	        	while(bit_is_clear(PING, 0))
	        	{
	        		if(bit_is_clear(PING, 1))
	        		{
                Serial.print("TZ3Ticks was : ");
                Serial.println(TZ3Ticks);
	        			TZ3Ticks++;
	        			allBreak = true;
                Serial.print("Breaked");
	        			break;
                while(bit_is_clear(PING, 1));
	        		}
	        	}
	        	TZ3Ticks--;
		        Serial.print("Now TZ3Ticks : ");
		        Serial.println(TZ3Ticks);
            PORTC |= 0xff;
            _delay_ms(1000);
            PORTC = 0x00;
	        }
	        else if(bit_is_clear(PING, 1))
	        {
	        	while(bit_is_clear(PING, 1));
	        	TZ3Ticks++ ;
		        Serial.print("Now TZ3Ticks : ");
		        Serial.println(TZ3Ticks);
            PORTC |= 0xff;
            _delay_ms(200);
            PORTC = 0x00;
	        }
	        else if(bit_is_clear(PING, 2))
	        {
	        	while(bit_is_clear(PING, 2));
	        	throwPWM3-=5;
		        Serial.print("Now throwPWM3 : ");
		        Serial.println(throwPWM3);
            PORTC |= 0xff;
            _delay_ms(200);
            PORTC = 0x00;
	        }
	        else if(bit_is_clear(PING, 3))
	        {
	        	while(bit_is_clear(PING, 3));
	        	throwPWM3+=5;
		        Serial.print("Now throwPWM3 : ");
		        Serial.println(throwPWM3);
            PORTC |= 0xff;
            _delay_ms(200);
            PORTC = 0x00;
	        }
	        if(allBreak)
          {
            allBreak = false;
	        	break;
          }
	      }
	      PORTC = 0x00;
      }
    }
  }
  else
  {
    botKill();
  }
}

void SerialPrint()
{
  #ifdef debug
//  Serial.print("  firstTurn = ");Serial.print(firstTurn);
//  Serial.print("  turnFlag = ");Serial.print(turnFlag);
//  Serial.print("maxPWM : ");Serial.print(maxPWM);
//  Serial.print("  juncCount = ");Serial.print(juncCount);
//  Serial.print("  tz1 = ");Serial.print(tz1);
//  Serial.print("  rongBay = ");Serial.print(rongBay);
//  Serial.print("  shortPlus =  ");Serial.print(shortPlus);
//  Serial.print("  frontOnline = ");Serial.print(frontOnline);
//  Serial.print("  backOnline = ");Serial.print(backOnline);
//  Serial.print("  leftOnline = ");Serial.print(leftOnline);
//  Serial.print("  rightOnline = ");Serial.print(rightOnline);
//  Serial.print("  errorFront = ");Serial.print(errorFront);
//  Serial.print("  errorBack = ");Serial.print(errorBack);
//  Serial.print("  errorRight = ");Serial.print(errorRight);
//  Serial.print("  errorLeft = ");Serial.print(errorLeft);
//  Serial.print("\t");
//  Serial.println("");
  #endif
}

void encoderInterruptInit()
{
  DDRE &= ~(1 << PE5);    //Channel Z
  EICRB |= (1 << ISC51) | (1 << ISC50);    //rising egde
  EIMSK |= (1 << INT5);

  DDRE &= ~(1 << PE6);    //Channel A
  EICRB |= (1 << ISC61) | (1 << ISC60);   //rising egde
  EIMSK |= (1 << INT6);
  DDRE &= ~(1 << PE7);    //Channel B
  sei();
}

void baseMotorsInit()
{
  DDRC |= 0xFF;
  PORTC |= 0xFF;
  pwm1_init();
  pwm3_init();
  BASEBRAKE_DIR = 1;   //For Relay braking of Base
  baseUnbrake();    //Active Low
}

void baseBrake()
{
  BASEBRAKE = 0;    //braking the base motor
  _delay_ms(5);
}

void baseUnbrake()
{
  BASEBRAKE = 1;    //unbraking the base motor
  _delay_ms(5);
}

void throwingMotorInit()
{
  pwm1_init();
  MOTORFR_DIR = 1;
  MOTORBR_DIR = 1;    //Throwing Motor
  THROWBRAKE_DIR = 1;    //For Relay braking of Throwing
  throwUnbrake();   //unbraking the throwing motor
}

void throwBrake()
{
  THROWBRAKE = 0;    //braking the throwing motor
  _delay_ms(5);
}

void throwUnbrake()
{ 
  THROWBRAKE = 1;    //unbraking the throwing motor
  _delay_ms(5);
}

void clampPistonInit()
{
  OPENCLAMP_DIR = 1;   //Clamp Piston
  OPENCLAMP = 0;
}

void piInterfaceInit()
{
  TZ1_DIR = 0;
  TZ2_DIR = 0;
  TZ3_DIR = 0;
  
  TZ1_PORT = 1;
  TZ2_PORT = 1;
  TZ3_PORT = 1;
}

void readSensorFront()    //front dekha toh left wala 0
{
  sensorRawFront[0] = getSensorReading(frontSensor, 4);
  sensorRawFront[1] = getSensorReading(frontSensor, 5);
  sensorRawFront[2] = getSensorReading(frontSensor, 6);
  sensorRawFront[3] = getSensorReading(frontSensor, 7);
  sensorRawFront[4] = getSensorReading(frontSensor, 0);
  sensorRawFront[5] = getSensorReading(frontSensor, 1);
  sensorRawFront[6] = getSensorReading(frontSensor, 2);
  sensorRawFront[7] = getSensorReading(frontSensor, 3);
}

void readSensorBack()   //front dekha toh left wala 0
{
  sensorRawBack[0] = getSensorReading(backSensor, 3);
  sensorRawBack[1] = getSensorReading(backSensor, 2);
  sensorRawBack[2] = getSensorReading(backSensor, 1);
  sensorRawBack[3] = getSensorReading(backSensor, 0);
  sensorRawBack[4] = getSensorReading(backSensor, 7);
  sensorRawBack[5] = getSensorReading(backSensor, 6);
  sensorRawBack[6] = getSensorReading(backSensor, 5);
  sensorRawBack[7] = getSensorReading(backSensor, 4);
}

void readSensorRight()   //left dekha toh left wala 0
{
  sensorRawRight[0] = getSensorReading(rightSensor, 7);
  sensorRawRight[1] = getSensorReading(rightSensor, 6);
  sensorRawRight[2] = getSensorReading(rightSensor, 5);
  sensorRawRight[3] = getSensorReading(rightSensor, 4);
  sensorRawRight[4] = getSensorReading(rightSensor, 3);
  sensorRawRight[5] = getSensorReading(rightSensor, 2);
  sensorRawRight[6] = getSensorReading(rightSensor, 1);
  sensorRawRight[7] = getSensorReading(rightSensor, 0);
}

void readSensorLeft()   //left dekha toh left wala 0
{
  sensorRawLeft[0] = getSensorReading(leftSensor, 0);
  sensorRawLeft[1] = getSensorReading(leftSensor, 1);
  sensorRawLeft[2] = getSensorReading(leftSensor, 2);
  sensorRawLeft[3] = getSensorReading(leftSensor, 3);
  sensorRawLeft[4] = getSensorReading(leftSensor, 4);
  sensorRawLeft[5] = getSensorReading(leftSensor, 5);
  sensorRawLeft[6] = getSensorReading(leftSensor, 6);
  sensorRawLeft[7] = getSensorReading(leftSensor, 7);
}

void calibrateFront()
{
  unsigned int time = 10;
  if(toggle == 0)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorFront();
        for(int j = 0; j < numOfSensors; j++)
        {
          sensorMinFront[j] += sensorRawFront[j];
        }
     }
     for(int j = 0; j < numOfSensors; j++)
     {
        sensorMinFront[j] = sensorMinFront[j]/10;
     }
  }
  else if(toggle == 1)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorFront();
        for(int j = 0; j < numOfSensors; j++)
        {
          sensorMaxFront[j] += sensorRawFront[j];
        }
     }
     for(int j = 0; j < numOfSensors; j++)
     {
        sensorMaxFront[j] = sensorMaxFront[j]/10;
     }
  }
  
  Serial.print("int sensorMinFront[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMinFront[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
  
  Serial.print("int sensorMaxFront[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMaxFront[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
}

void calibrateBack()
{
  unsigned int time = 10;
  if(toggle == 0)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorBack();
        for(int j = 0; j < numOfSensors; j++)
        {
          sensorMinBack[j] += sensorRawBack[j];
        }
     }
     for(int j = 0; j < numOfSensors; j++)
     {
        sensorMinBack[j] = sensorMinBack[j]/10;
     }
      
  }
  else if(toggle == 1)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorBack();
        for(int j = 0; j < numOfSensors; j++)
        {
          sensorMaxBack[j] += sensorRawBack[j];
        }
     }
     for(int j = 0; j < numOfSensors; j++)
     {
        sensorMaxBack[j] = sensorMaxBack[j]/10;
     }
  }

  Serial.print("int sensorMinBack[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMinBack[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
  
  Serial.print("int sensorMaxBack[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMaxBack[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
  _delay_ms(2000);
}

void calibrateRight()
{
  unsigned int time = 10;

  if(toggle == 0)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorRight();
      for(int j = 0; j < numOfSensors; j++)
      {
        sensorMinRight[j] += sensorRawRight[j];
      }
     }
     for(int j = 0; j < numOfSensors; j++)
     {
        sensorMinRight[j] = sensorMinRight[j]/10;
     }
      
  }
  else if(toggle == 1)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorRight();
        for(int j = 0; j < numOfSensors; j++)
        {
          sensorMaxRight[j] += sensorRawRight[j];
        }
     }
     for(int j = 0; j < numOfSensors; j++)
     {
        sensorMaxRight[j] = sensorMaxRight[j]/10;
     }
  }

  Serial.print("int sensorMinRight[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMinRight[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
  
  Serial.print("int sensorMaxRight[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMaxRight[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
}

void calibrateLeft()
{
  unsigned int time = 10;

  if(toggle == 0)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorLeft();
        for(int j = 0; j < numOfSensors; j++)
        {
          sensorMinLeft[j] += sensorRawLeft[j];
        }
     }
     for(int j = 0; j  < numOfSensors; j++)
     {
        sensorMinLeft[j] = sensorMinLeft[j]/10;
     }
      
  }
  else if(toggle == 1)
  {
    for(int i = 0; i < time; i++)
    {
      readSensorLeft();
        for(int j = 0; j < numOfSensors; j++)
        {
          sensorMaxLeft[j] += sensorRawLeft[j];
        }
     }
     for(int j = 0; j < numOfSensors; j++)
     {
        sensorMaxLeft[j] = sensorMaxLeft[j]/10;
     }
  }

  Serial.print("int sensorMinLeft[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMinLeft[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
  
  Serial.print("int sensorMaxLeft[numOfSensors] = {");
  for(int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMaxLeft[i]);
    if(i < 7)
    {
      Serial.print(",");Serial.print(" ");
    }
  }
  Serial.println("};");
}


void calcSensorValFront()
{
  for (int j = 0; j < numOfSensors; j++)
  {
    sensorValFront[j] = map(sensorRawFront[j], sensorMinFront[j], sensorMaxFront[j], 1000, 0);
    if (sensorValFront[j] < 0)
      sensorValFront[j] = 0;
    else if (sensorValFront[j] > 1000)
      sensorValFront[j] = 1000;
  }
}

void calcSensorValBack()
{
  for (int j = 0; j < numOfSensors; j++)
  {
    sensorValBack[j] = map(sensorRawBack[j], sensorMinBack[j], sensorMaxBack[j], 1000, 0);
    if (sensorValBack[j] < 0)
      sensorValBack[j] = 0;
    else if (sensorValBack[j] > 1000)
      sensorValBack[j] = 1000;
  }
}
 
void calcSensorValRight()
{
  for (int j = 0; j < numOfSensors; j++)
  {
    sensorValRight[j] = map(sensorRawRight[j], sensorMinRight[j], sensorMaxRight[j], 1000, 0);
    if (sensorValRight[j] < 0)
      sensorValRight[j] = 0;
    else if (sensorValRight[j] > 1000)
      sensorValRight[j] = 1000;
  }
}

void calcSensorValLeft()
{
  for (int j = 0; j < numOfSensors; j++)
  {
    sensorValLeft[j] = map(sensorRawLeft[j], sensorMinLeft[j], sensorMaxLeft[j], 1000, 0);
    if (sensorValLeft[j] < 0)
      sensorValLeft[j] = 0;
    else if (sensorValLeft[j] > 1000)
      sensorValLeft[j] = 1000;
  }
}

void calcErrorFront()
{
  frontOnline = 0;
  unsigned long avg = 0, sum = 0, pos = (numOfSensors - 1) * 1000.0;
  for (int j = 0; j < numOfSensors; j++)
  {
    if (sensorValFront[j] > onlineThresh)
      frontOnline++;
    if (sensorValFront[j] > 250)
    {
      avg += (long)(sensorValFront[j]) * ((j) * 1000);
      sum += sensorValFront[j];
    }
  }
  if (sum != 0)
  {
    pos = avg / sum;
  }

  if(frontOnline == 0)
  { 
    if(errorFront > 0)
      pos = (numOfSensors - 1) * 1000.0;                    // If it last read to the right of center, return the max.
    else
      pos = 0;                        // If it last read to the left of center, return 0.
  }
  errorFront = pos - (numOfSensors - 1) * 500.0;
}

void calcErrorBack()
{
  backOnline = 0;
  unsigned long avg = 0, sum = 0, pos = (numOfSensors - 1) * 1000.0;
  for (int j = 0; j < numOfSensors; j++)
  {
    if (sensorValBack[j] > onlineThresh)
      backOnline++;
    if (sensorValBack[j] > 300)
    {
      avg += (long)(sensorValBack[j]) * ((j) * 1000);
      sum += sensorValBack[j];
    }
  }
  if (sum != 0)
  {
    pos = avg / sum;
  }

  if(backOnline == 0)
  { 
    if(errorBack > 0)
      pos = (numOfSensors - 1) * 1000.0;                    // If it last read to the right of center, return the max.
    else
      pos = 0;                        // If it last read to the left of center, return 0.
  }
  errorBack = pos - (numOfSensors - 1) * 500.0;
  if(firstTurn == 2)
  {
    errorBack = 0;
    if(backOnline >= 1)
    {
      firstTurn = 0;
      pidAlign(0);
    }
  }
}

void calcErrorLeft()
{
  leftOnline = 0;
  unsigned long avg = 0, sum = 0, pos = (numOfSensors - 1) * 1000.0;
  for (int j = 0; j < numOfSensors; j++)
  {
    if (sensorValLeft[j] > onlineThresh)
      leftOnline++;
    if (sensorValLeft[j] > 300)
    {
      avg += (long)(sensorValLeft[j]) * ((j) * 1000);
      sum += sensorValLeft[j];
    }
  }
  if (sum != 0)
  {
    pos = avg / sum;
  }

  if(leftOnline == 0)
  {
    if(errorLeft > 0)
      pos = (numOfSensors - 1) * 1000.0;      // If it last read to the right of center, return the max.
    else
      pos = 0;      // If it last read to the left of center, return 0.
  }
  errorLeft = pos - (numOfSensors - 1) * 500.0;
  if(turnFlag == 0 && firstTurn == 1 && (leftOnline == 0 || leftOnline >= 3))  //leftOnline >= 3
  {
    errorLeft = 0;
  }       
}

void calcErrorRight()
{
  rightOnline = 0;
  unsigned long avg = 0, sum = 0, pos = (numOfSensors - 1) * 1000.0;
  for (int j = 0; j < numOfSensors; j++)
  {
    if (sensorValRight[j] > onlineThresh)
      rightOnline++;
    if (sensorValRight[j] > 300)
    {
      avg += (long)(sensorValRight[j]) * ((j) * 1000);
      sum += sensorValRight[j];
    }
  }
  if (sum != 0)
  {
    pos = avg / sum;
  }
  if(rightOnline == 0)
  {
    if(errorRight > 0)
      pos = (numOfSensors - 1) * 1000.0;      // If it last read to the right of center, return the max.
    else
      pos = 0;        // If it last read to the left of center, return 0.
  }
  errorRight = pos - (numOfSensors - 1) * 500.0;
}

void pidAlign(int pidAlign)   //pidAlign(0) - Fast and pidAlign(1) - Slow
{
	switch(pidAlign)
  {
  	case 0:
    maxPWM = fastMaxPWM;
    maxErrorPWM = normalMaxErrorPWM;
    break;
  
  	case 1:
    maxPWM = alignMaxPWM;
    maxErrorPWM = alignMaxErrorPWM;
  	break;
  
  	case 2:
    maxPWM = botAlignPWM;
    maxErrorPWM = alignMaxErrorPWM;
  	break;
  
  	case 3:
    maxPWM = mediumAlignPWM;
    maxErrorPWM = alignMaxErrorPWM;
  	break;
  
  	case 4:
    maxPWM = slowAlignPWM;
    maxErrorPWM = alignMaxErrorPWM;
  	break;
  
  	case 5:
    maxPWM = rampSlowestPWM;
    maxErrorPWM = alignMaxErrorPWM;
  	break;
    
  	default:break;
  }
}

void calcSpeedToMotors()
{
  switch(turnFlag)
  {
    case 0:
    //to left
    sFront = 0 - (maxPWM);
    sLeft  =     (errorLeft);
    sBack  = 0 - (maxPWM);
    sRight =  	 (errorRight);

    if(sFront > 0)
      sFront = 0;

    if(sBack > 0)
      sBack = 0;
    break;

    case 1:
    //to right
    sFront =  (maxPWM);
    sLeft  =  (errorLeft);  
    sBack  =  (maxPWM);
    sRight =  (errorRight);

    if(sFront < 0)
      sFront = 0;

    if(sBack < 0)
      sBack = 0;
    break;

    case 2:
    //to front
    sFront  =  (errorFront);  
    sLeft   =  (maxPWM);
    sBack   =  (errorBack);
    sRight  =  (maxPWM);

    if(sLeft < 0)
      sLeft = 0;

    if(sRight < 0)
      sRight = 0;
    break;
    
    case 3:
    //to back
    sFront  =      (errorFront);  
    sLeft   =  0 - (maxPWM);
    sBack   =   	 (errorBack);
    sRight  =  0 - (maxPWM);

    if(sLeft > 0)
      sLeft = 0;

    if(sRight > 0)
      sRight = 0;
    break;

    default:break;
  }
  prevError = error;
  prevRotation = rotation;
}

void uploadSpeedLineFollow(int maxPWMLine)
{
  baseUnbrake();   //for unbraking the base motor
  
  if(turnFlag == 0 || turnFlag == 1)
  {
    sLeft  = map(sLeft, -3500, 3500, 0 - maxPWMLine, maxPWMLine);
    sRight = map(sRight, -3500, 3500, 0 - maxPWMLine, maxPWMLine);  
  }
  else if (turnFlag == 2 || turnFlag == 3)
  {
    sFront = map(sFront, -3500,3500, 0 - maxPWMLine, maxPWMLine);
    sBack = map(sBack, -3500, 3500, 0 - maxPWMLine, maxPWMLine);
  }
  
  if(sFront < -30)
  {
    sFront *= (-1);
    MOTORFF = 0;
    MOTORFB = 1;
    if(maxPWM != 0 && sFront > maxPWM)
      sFront = maxPWM;
    PWMF = sFront;
  }
  else if(sFront > 30)
  {
    MOTORFF = 1;
    MOTORFB = 0;
    if(maxPWM != 0 && sFront > maxPWM)
      sFront = maxPWM;
    PWMF = sFront;
  }
  else
  {
    MOTORFF = 1;
    MOTORFB = 1;
    PWMF = 666;
  }

  if(sBack < -30)
  {
    sBack *= (-1);
    MOTORBF = 0;
    MOTORBB = 1;
    if(maxPWM != 0 && sBack > maxPWM)
      sBack = maxPWM;
    PWMB = sBack;
  }
  else if(sBack > 30)
  {
    MOTORBF = 1;
    MOTORBB = 0;
    if(maxPWM != 0 && sBack > maxPWM)
      sBack = maxPWM;
    PWMB = sBack;
  }
  else
  {
    MOTORBF = 1;
    MOTORBB = 1;
    if(maxPWM != 0 && sBack > maxPWM)
      sBack = maxPWM;
    PWMB = 666;
  }

  if(sLeft < -30)
  {
    sLeft *= (-1);
    MOTORLF = 0;
    MOTORLB = 1;
    if(maxPWM != 0 && sLeft > maxPWM)
      sLeft = maxPWM;
    PWML = sLeft;
  }
  else if(sLeft > 30)
  {
    MOTORLF = 1;
    MOTORLB = 0;
    if(maxPWM != 0 && sLeft > maxPWM)
      sLeft = maxPWM;
    PWML = sLeft;
  }
  else
  {
    MOTORLF = 1;
    MOTORLB = 1;
    PWML = 666;
  }

  if(sRight < -30)
  {
    sRight *= (-1);
    MOTORRF = 0;
    MOTORRB = 1;
    if(maxPWM != 0 && sRight > maxPWM)
      sRight = maxPWM;
    PWMR = sRight;
  }
  else if(sRight > 30)
  {
    MOTORRF = 1;
    MOTORRB = 0;
    if(maxPWM != 0 && sRight > maxPWM)
      sRight = maxPWM;
    PWMR = sRight;
  }
  else
  {
    MOTORRF = 1;
    MOTORRB = 1;
    PWMR = 666;
  }
}

void botAlign(int currentDir)
{
  PWMT = 0;
  stopRotation();
  throwBrake();   //for braking the throwing motor
  int alignCount = 0;
  botKill();
  bool breakOut = false;
  int rcount = 0;
  if(currentDir == 0)
  {
    botAlignPWM = 110;
  }
  else
  {
    botAlignPWM = 120;
  }
  pidAlign(2);
  int maxrcount = 20;
  if(currentDir == 2)
  {
    maxrcount = 10;
  }
  else
  {
    maxrcount = 20;
  }
  while(rcount <= maxrcount)
  {
    rcount++;
    calcAllError();
  }
  switch(currentDir)
  {
    case 0:
    calcAllError();
    errorFront += 1000;
    errorBack -= 1000;
    maxPWM = 0;
    maxErrorPWM = 400;
    turnFlag = 2;
    rcount = 0; 
    
    alignCount = 0;
    if(!rongBay)
    {
      while(!breakOut)
      {
        rcount = 0;
        alignCount++;
        while(abs(errorFront) > 200 || abs(errorBack)>200)
        {
          rcount++;
          calcAllError();
          errorFront += 1000;
          errorBack -= 1000;
          baseUnbrake();   //for unbraking the base motor
          if(errorFront < -200)
          {
            MOTORFF = 0;
            MOTORFB = 1;
            PWMF = 140;
          }
          else if(errorFront > 200)
          {
            MOTORFF = 1;
            MOTORFB = 0;
            PWMF = 140;
          }
          else
          {
            MOTORFF = 1;
            MOTORFB = 1;
            PWMF = 666;
          }
          if(errorBack < -200)
          {
            MOTORBF = 0;
            MOTORBB = 1;
            PWMB = 140;
          }
          else if(errorBack > 200)
          {
            MOTORBF = 1;
            MOTORBB = 0;
            PWMB = 140;
          }
          else
          {
            MOTORBF = 1;
            MOTORBB = 1;
            PWMB = 666;
          }
          
        }
        if(alignCount>=3)
        {
          break;
        }
        botKill();
        _delay_ms(20);
        
      }
    }
    else
    {
      while(!breakOut)
      {
        rcount = 0;
        alignCount++;
        while(abs(errorFront) > 200 || abs(errorBack)>200)
        {
          rcount++;
          calcAllError();
          errorFront += 800;
          errorBack -= 800;
          baseUnbrake();   //for unbraking the base motor
          if(errorFront < -200)
          { 
            MOTORFF = 0;
            MOTORFB = 1;
            PWMF = 130;
          }
          else if(errorFront > 200)
          {
            MOTORFF = 1;
            MOTORFB = 0;
            PWMF = 130;
          }
          else
          {
            MOTORFF = 1;
            MOTORFB = 1;
            PWMF = 666;
          }
          if(errorBack < -200)
          {
            MOTORBF = 0;
            MOTORBB = 1;
            PWMB = 130;
          }
          else if(errorBack > 200)
          {
            MOTORBF = 1;
            MOTORBB = 0;
            PWMB = 130;
          }
          else
          {
            MOTORBF = 1;
            MOTORBB = 1;
            PWMB = 666;
          }
          
        }
        if(alignCount>=3)
        {
          break;
        }
        botKill();
        _delay_ms(20);
      }
    }
    
    break;
    case 1:
    alignCount = 0;
    maxAlignCount = 1;
    while(((errorBack+errorFront) > 400) || ((errorBack+errorFront) < -400) || (frontOnline == 0 || backOnline == 0))
    {
      alignCount++ ;
      if((errorBack+errorFront) > 400 && (frontOnline != 0 && backOnline != 0))
      {
        turnFlag = 1;
        while((errorBack+errorFront) > 400)
        {
          calcAllError();
          calcSpeedToMotors();
          uploadSpeedLineFollow(maxErrorPWM);
        }
      }
      else if(((errorBack+errorFront) < -400) || (frontOnline == 0 || backOnline == 0))
      {
        turnFlag = 0;
        while(((errorBack+errorFront) < -400))
        {
          calcAllError();
          calcSpeedToMotors();
          uploadSpeedLineFollow(maxErrorPWM);
        }
      }
      botKill();
      _delay_ms(20);
      calcAllError();
      if(alignCount >= maxAlignCount)
      {
        PORTC |= 0xff;
        _delay_ms(100);
        PORTC = 0x00;
        break;
      }
    }
    
    botKill();
    break;

    case 2:
    alignCount = 0;
    while(((errorRight+errorLeft) > 400) || ((errorRight+errorLeft) < -400) || (leftOnline == 0 || rightOnline == 0))
    {
      alignCount++ ;
      if((errorRight+errorLeft) > 400 && (leftOnline != 0 && rightOnline != 0))
      {
        turnFlag = 2;
        while((errorRight+errorLeft) > 400)
        {
          calcAllError();
          calcSpeedToMotors();
          uploadSpeedLineFollow(maxErrorPWM);
        }
      }
      else if(((errorRight+errorLeft) < -400) || (leftOnline == 0 || rightOnline == 0))
      {
        turnFlag = 3;
        while(((errorRight+errorLeft) < -400) || (leftOnline == 0 || rightOnline == 0))
        {
          calcAllError();
          calcSpeedToMotors();
          uploadSpeedLineFollow(maxErrorPWM);
//          transferShuttle();
//          if(transferDone && leftOnline!=0 && rightOnline!=0 && backOnline!=0 && frontOnline!=0)
//          {
//            break;
//          }
        }
      }
      botKill(); 
      _delay_ms(20);
      calcAllError();
      if(alignCount >= maxAlignCount)
      {
        PORTC |= 0xff;
        _delay_ms(100);
        PORTC = 0x00;
        break;
      }
    }
    botKill();
    maxPWM = 0;
    maxErrorPWM = 666;
    botKill();
    break;

    default:break;
  }
  pidAlign(0);
  botKill();
  maxAlignCount = 4;
}

void calcAllError()
{
  readSensorFront();
  readSensorBack();
  calcSensorValFront();
  calcSensorValBack();
  calcErrorFront();
  calcErrorBack();

  readSensorLeft();
  readSensorRight();
  calcSensorValLeft();
  calcSensorValRight();
  calcErrorLeft();
  calcErrorRight();
}

bool isShortPlusOnLeft()
{
  bool temp =  false;
  for(int i = 0; i<5; i++)
  {
    for(int j = i+3 ; j<8 ; j++)
    {
      if(sensorValLeft[i]>onlineThresh && sensorValLeft[j]>onlineThresh)
      {
        temp = true;
        break;
      }
    }
    if(temp)
    {
      break;
    }
  }
  if(temp)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

bool isShortPlusOnRight()
{
  bool temp =  false;
  for(int i = 0; i<5; i++)
  {
    for(int j = i+3 ; j<8 ; j++)
    {
      if(sensorValRight[i]>onlineThresh && sensorValRight[j]>onlineThresh)
      {
        temp = true;
        break;
      }
    }
    if(temp)
    {
      break;
    }
  }
  if(temp)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

bool isShortPlusOnFront()
{
  bool temp =  false;
  for(int i = 0; i<5; i++)
  {
    for(int j = i+3 ; j<8 ; j++)
    {
      if(sensorValFront[i]>onlineThresh && sensorValFront[j]>onlineThresh)
      {
        temp = true;
        break;
      }
    }
    if(temp)
    {
      break;
    }
  }
  if(temp)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void lineFollow()
{
  calcAllError();
  
  if(leftOnline ==  0 && rightOnline >= 1 && frontOnline >= 1 && backOnline == 0 && firstTurn == 1)  //the 1st turn
  {
    turnFlag = 2;
    firstTurn = 2;
  }

  if((leftOnline == 0 && firstTurn == 1))                      //slow for first turn
  {
    pidAlign(1);
  }


  if(((frontOnline >= 1 && backOnline >= 1 && (turnFlag == 0 || turnFlag == 1))  ||  ( leftOnline >= 1 && rightOnline >= 1 && turnFlag == 2)) && firstTurn==0)   //throw junction counter
  {
    if(juncFlag)
    {
      if(turnFlag == 1)    //Rightwards linefollow
        juncCount--;
      else
        juncCount++;
       
      juncFlag = false;
    }
  }
  else if((backOnline == 0 && frontOnline == 0) || (leftOnline == 0 && rightOnline == 0))
    juncFlag = true;

  shortJuncCheck();       //shortPlus
  juncCheck();              //Junction
  calcSpeedToMotors();
  uploadSpeedLineFollow(maxErrorPWM);
}

void shortJuncCheck()
{
  if(((isShortPlusOnLeft() && turnFlag == 0) || (isShortPlusOnRight() && turnFlag == 1)) && firstTurn == 0)    ///short junc counter
  {
    if(shortPlusFlag)
    {
      if(turnFlag == 1)    //Rightwards linefollow
        shortPlus--;
      else
        shortPlus++;
      if(turnFlag == 0)
      {
        if((rongBay == 0 && shortPlus == 1) || (rongBay == 1 && shortPlus == 3))    //1 4
        {
          pidAlign(3);
        }
        
        else if((rongBay == 0 && shortPlus == 2))   //2 5
        {
          pidAlign(5);
        }
        else if(rongBay == 1 && shortPlus == 5)
        {
          pidAlign(5);
        }
//        else if((rongBay == 1 && shortPlus == 4))
//        {
//          pidAlign(4);
//        }
      }
      else if(turnFlag == 1)
      {
        if(shortPlus == 0)
        {
          pidAlign(5);
        }
        else if(shortPlus == 1)
        {
          pidAlign(3);
        }
        else if(shortPlus == 4)
        {
          pidAlign(0);
        }
      }
      shortPlusFlag = false;
    }
  }
  else
  {
    shortPlusFlag = true;
  }  
  
  if(turnFlag == 2)
  {
    if(isShortPlusOnFront())
    {
      if(frontPlusFlag)
      {
        frontPlusFlag = false;
        frontPlus++;
        if(frontPlus == 1)
        {
          if(tz1 == false)
          {
            botKill();
            maxPWM = 0;
          }
          else
          {
            pidAlign(3);
          }
        }
        else if(frontPlus == 2)
        {
          pidAlign(5);
        }
      }
    }
    else 
    {
      frontPlusFlag = true;
    }
  }
}

void juncCheck()
{ 
  if((juncCount == 1 && turnFlag == 2 && (!tz1)) || (juncCount == 1 && turnFlag == 1))    //transferFlag junction for TZ1
  {
    shortPlus = 0;
    frontPlus = 1;
    botKill();
    pidAlign(1);
    juncFlag = false;
    botAlign(turnFlag);
    botKill();
    
    if(transferFlag)
    {
      
//      openClamp();
      if(!transferDone)
      {
	      stopAlign = true;  
	      alignArm();
	    }
      startTransferTZ1:    //label for goto
       
      initTOF();
      while(!transferDone)
      {
        Serial.println(distance);
      	transferShuttle();
      }
     
      PORTA |= (1 << PA1);    //LED STRIP OFF
      int colorCount = 0;
      while(colorCount < 250)
      {
        if(!TZ2)
          colorCount++;
        else if(!TZ1)
          colorCount++;
        else if(!TZ3)
          colorCount++;
        else{
          distance = readTOF();
          if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
          {
            openClamp();
            alignAfterFailure();
            transferDone = false;

            sureCount = 0;
            unsureCount = 0;
            usCount = 0;
            goto startTransferTZ1;
          }
          colorCount = 0;
          
        }
      }
      PORTA &= ~(1 << PA1);   //LED STRIP OFF
      
      throwBrake();   //for braking the throwing motor
      
      if(!TZ1)
      {
        tz1 = false;    //TZ1
        turnFlag = 0;
        pidAlign(0);
      }
      else if(!TZ2)
      {
        tz1 = true;    //TZ2
        turnFlag = 2;
        pidAlign(3);
        maxAlignCount = 1;
      }
      else if(!TZ3)
      {
        rongBay = true;   //TZ3
        tz1 = true;
        turnFlag = 2;
        pidAlign(3);
        maxAlignCount = 1;
      }
    }
    else
    {
      turnFlag = 0;
      pidAlign(0);
    }
    
  }
  else if(juncCount == 2 && turnFlag == 0 && (!tz1))    //throwing junction of TZ1
  {
    botKill();
    juncFlag = false;
    botAlign(turnFlag);
    botKill();
    
    throwAtTZ(1);
    transferFlag = true;
    turnFlag = 1;
    shortPlus = 2;
    sureCount = 0;
    unsureCount = 0;
    usCount = 0;
    transferDone = false;
    pidAlign(0);
  }
  else if((juncCount == 2 && turnFlag == 2) || (juncCount == 3 && turnFlag == 1))        //transfer zone for TZ2
  {
    juncCount = 3;
    shortPlus = 0;
    botKill();
    
    juncFlag = false;
    pidAlign(1);
    if(!firstTimeTZ2)
    {
      botAlign(turnFlag);
      botKill();
      
      pidAlign(1);
      juncFlag = false;
      
      if(!transferDone)
      {
        stopAlign = true;  
        alignArm();
      }
      startTransferTZ2:    //label for goto

      initTOF();
      
      while(!transferDone)
      {
        Serial.println(distance);
        transferShuttle();
      }
      PORTA |= (1 << PA1);    //LED STRIP OFF
      int colorCount = 0;
      while(colorCount < 250)
      {
        if(!TZ2)
          colorCount++;
        else if(!TZ1)
          colorCount++;
        else if(!TZ3)
          colorCount++;
        else{
          distance = readTOF();
          if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
          {
            openClamp();
            alignAfterFailure();
            transferDone = false;

            sureCount = 0;
            unsureCount = 0;
            usCount = 0;
            goto startTransferTZ2;
          }
          colorCount = 0;
        }
      }
      PORTA &= ~(1 << PA1);   //LED STRIP OFF

      throwBrake();   //for braking the throwing motor
      
      if(!TZ2 || !TZ1)
        rongBay = false;    //TZ2
      else if(!TZ3)
        rongBay = true;   //TZ3
       pidAlign(0);
    }
    else
    {
      maxPWM = 450;
      maxErrorPWM = 270;
    }
    firstTimeTZ2 = false;
    turnFlag = 0;
    
   
  }
  else if(juncCount == 4 && turnFlag == 0)                                            //throwing zone for TZ2
  {
    if(shortPlus<=1)
    {
      shortPlus = 2;
    }
    if(!rongBay)
    {
      botKill();
      juncFlag = false;
      botAlign(turnFlag);
      
      throwAtTZ(2);
      transferFlag = true;
      turnFlag = 1;
      pidAlign(1);
      sureCount = 0;
      unsureCount = 0;
      usCount = 0;
      transferDone = false;
      maxErrorPWM = alignMaxErrorPWM;
    }
  }
  else if(juncCount == 5 && turnFlag == 0)                                              //throwing zone for TZ3
  {
    botKill();
    PORTA &= ~(1 << PA0);    //LED OFF
    shortPlus = 5;
    
    juncFlag = false;
    botAlign(turnFlag);
    _delay_ms(500);
    throwAtTZ(3);
    transferFlag = true;
    turnFlag = 1;
    transferDone = false;

    sureCount = 0;
    unsureCount = 0;
    usCount = 0;
    pidAlign(1);  
  }
}

void transferShuttle()
{
  if(!transferDone)
  {
    if(sureCount < 15)
    {
      distance = readTOF();
      if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
        sureCount++;
      else
        sureCount = 0;
    }
    else
    {
      closeClamp();
      aligned  = true;
    }
  
    if(unsureCount < 25 && sureCount >=15)
    {
      distance = readTOF();
      if(minDistanceThreshold < distance && maxDistanceThreshold > distance)
        usCount++;
      else
        usCount = 0;
      unsureCount++;
    }
  
    if(usCount >= 25)
    {
      openClamp();
      sureCount = 0;
      unsureCount = 0;
      usCount = 0;
    }
    else if(unsureCount >= 25  && sureCount >= 15)
    {
      throwUnbrake();   //unbraking the throwing motor
      backwardRotation();   //to give backwards direction to the motor
      if(ticks > -80)               //////////////*******************************************************************************
      {
        PWMT = map(20, 0, 255, 0, 666);
      }
      else
      {
        stopRotation();
        PWMT = 0;
        transferDone = true;
      }
    }
  }
}

int adjust()
{
  if(ticks > 0)
  {
    if(ticks > 180)
      pidTicks = (ticks - 360);
    else
      pidTicks = ticks;
  }
  else
  {
    if(ticks < -180)
      pidTicks = (ticks + 360);
    else
      pidTicks = ticks;
  }
  return pidTicks;
}

void alignAfterFailure()
{
  throwUnbrake();   //for unbraking the throwing motor

  armError = adjust();
  if(armError > 0)
  {
    backwardRotation();
  }
  else if(armError <= 0) 
  {
    forwardRotation();
  }
  PWMT = map(60, 0, 255, 0, 666);

  throwCount = 0;
  while(throwCount == 0)
  {
    #ifdef debug
    Serial.print("ticks: ");Serial.print(ticks);
    Serial.print(" throwCount: ");Serial.print(throwCount);
    Serial.println("");
    #endif
  }
  PWMT = 0;
  stopRotation(); 
  
  int pidCount = 100;
  while(pidCount != 0)
  {
    armError = adjust();
    while(armError < -4 || 1 < armError)
    {
      pidCount = 100;
      #ifdef debug
      Serial.println(ticks);
      #endif
      throwUnbrake();   //for unbraking the throwing motor
      armError = adjust();
      
      diffError = armError - prevArmError;
      armPID = abs(armKp * armError + armKd * diffError);
      
      armPWM = map(armPID, 8, 400, 30, 80);
      armPWM = constrain(armPWM, 30, 80);
      
      if(armError > 0)
      {
        backwardRotation();
      }
      else if(armError < 0) 
      {
        forwardRotation();
      }

      PWMT = map(armPWM, 0, 255, 0, 666);
      prevArmError = armError;
    }
    PWMT = 0;
    stopRotation();
    throwBrake();   //for braking the throwing motor
    #ifdef debug
    Serial.print(" pidCount = ");Serial.println(pidCount);
    #endif
    pidCount--;
  }
}

void alignArm()
{
  if(!aligned)    //only when arm is not aligned
  {
    if(firstTimeAlign)    //only till Z is not triggered 
    {
      if(alignTemp)   //only to give directions once
      {
        alignTemp = false;
        throwCount = 0;
        throwUnbrake();   //for unbraking the throwing motor

        armError = adjust();
        if(armError > 0)
        {
          backwardRotation();
        }
        else if(armError <= 0)
        {
          forwardRotation();
        }
        PWMT = map(60, 0, 255, 0, 666);
      }
      
      if(throwCount > 0)   //rotate till Z
      {
        PWMT = 0;
        stopRotation();
        alignTemp = true;
        firstTimeAlign = false;
        pidCount = 100;
        #ifdef debug
//        Serial.print(" throwCount: ");Serial.print(throwCount);
//        Serial.println("");
        #endif
      }
    }
    
    else
    {
      if(!stopAlign)
      {
        armError = adjust();
        if(armError < -4 || 1 < armError)
        {
          throwUnbrake();   //for unbraking the throwing motor
          armError = adjust();
          
          diffError = armError - prevArmError;
          armPID = abs(armKp * armError + armKd * diffError);
          
          armPWM = map(armPID, 8, 400, 30, 80);
          armPWM = constrain(armPWM, 30, 80);
          
          if(armError > 0)
          {
            backwardRotation();
          }
          else if(armError < 0) 
          {
            forwardRotation();
          }
          
          PWMT = map(armPWM, 0, 255, 0, 666);
          prevArmError = armError;
        }
        
        else
        {
          PWMT = 0;
          stopRotation();
          throwBrake();   //for braking the throwing motor
        }
      }
      else
      {
        pidCount = 50;
        while(pidCount != 0)
        {
          armError = adjust();
          while(armError < -4 || 1 < armError)
          {
            pidCount = 50;
            throwUnbrake();   //for unbraking the throwing motor
            armError = adjust();
            
            diffError = armError - prevArmError;
            armPID = abs(armKp * armError + armKd * diffError);
            
            armPWM = map(armPID, 8, 400, 30, 80);
            armPWM = constrain(armPWM, 30, 80);
            
            if(armError > 0)
            {
              backwardRotation();
            }
            else if(armError < 0) 
            {
              forwardRotation();
            }
      
            PWMT = map(armPWM, 0, 255, 0, 666);
            prevArmError = armError;
          }
          PWMT = 0;
          stopRotation();
          throwBrake();   //for braking the throwing motor
          pidCount--;
        }
        
        stopAlign = false;
        firstTimeAlign = true;
        aligned = true;     //to Stop Aligning
      }
    }
  }
}

void throwAtTZ(int zone)
{
  throwCount = 0;
  throwUnbrake();   //unbraking the throwing motor
  
  if(zone == 1)
  {
    backwardRotation();   //to give backwards direction to the motor
    PWMT = map(throwPWM1, 0, 255, 0, 666);
    
    while(1)
    {
      #ifdef debug
      Serial.print("ticks:\t");
      Serial.print(ticks);  
      Serial.println(" ");
      #endif

      if(throwCount == TZ1ThrowCount)
        PORTA |= (1 << PA0);    //LED ON
      else
        PORTA &= ~(1 << PA0);    //LED OFF
      
      if(ticks <= TZ1Ticks && throwCount >= TZ1ThrowCount)
      {
        #ifdef debug
        Serial.print("finalticks : ");
        Serial.println(ticks);
        #endif

        openClamp();
        break;
      }
    }
  }
  
  else if(zone == 2)
  {
    backwardRotation();   //to give backwards direction to the motor
    PWMT = map(throwPWM2, 0, 255, 0, 666);
    
    while(1)
    {
      #ifdef debug
      Serial.print("ticks:\t");
      Serial.print(ticks);  
      Serial.println(" ");
      #endif
      
      if(throwCount == TZ2ThrowCount)
        PORTA |= (1 << PA0);    //LED ON
      else
        PORTA &= ~(1 << PA0);    //LED OFF
      
      if(ticks <= TZ2Ticks && throwCount >= TZ2ThrowCount)
      {
        #ifdef debug
        Serial.print("finalticks : ");
        Serial.println(ticks);
        #endif
        
        openClamp();
        break;
      }
    }
  }

  else if(zone == 3)
  {
    //_delay_ms(2000);
    backwardRotation();   //to give backwards direction to the motor
    PWMT = map(throwPWM3, 0, 255, 0, 666);
    
    while(1)
    {
      #ifdef debug
      Serial.print("ticks:\t");
      Serial.print(ticks);  
      Serial.println(" ");
      #endif

      if(throwCount == TZ3ThrowCount)
        PORTA |= (1 << PA0);    //LED ON
      else
        PORTA &= ~(1 << PA0);    //LED OFF
      
      if(ticks <= TZ3Ticks && throwCount >= TZ3ThrowCount)
      {
        #ifdef debug
        Serial.print("finalticks : ");
        Serial.println(ticks);
        #endif

        openClamp();
        break;
      }
    }
  }
  throwCount = 0;
  PORTA &= ~(1 << PA0);   //LED OFF
  _delay_ms(100);
  stopRotation();
  PWMT = 0;
  throwBrake();   //for braking the throwing motor
  throws++;
  aligned = false;
}

void closeClamp()
{
  OPENCLAMP = 0;
}

void openClamp()
{
  OPENCLAMP = 1;
}

void forwardRotation()
{
  MOTORFR = 1;
  MOTORBR = 0;
}

void backwardRotation()
{
  MOTORFR = 0;
  MOTORBR = 1;
}

void stopRotation()
{
  MOTORFR = 0;
  MOTORBR = 0;
}

void botKill()
{
  MOTORFF = 0;
  MOTORFB = 0;
  PWMF = 0;

  MOTORBF = 0;
  MOTORBB = 0;
  PWMB = 0;

  MOTORLF = 0;
  MOTORLB = 0;
  PWML = 0;

  MOTORRF = 0;
  MOTORRB = 0;
  PWMR = 0;

  baseBrake();   //braking the base motor
}

void initTOF()
{
  int32_t temp = 0;
  
  Wire.begin();
  
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();

  while(1)
  {
    temp = sensor.readRangeContinuousMillimeters();
    if(temp == 65535)
    {
      PORTC = 0xF0;
      _delay_ms(200);
      PORTC = 0x0F;
      _delay_ms(200);

      Wire.begin();
  
      sensor.init();
      sensor.setTimeout(500);
      sensor.startContinuous();
    }
    
    else
    {
      PORTC = 0x00;
      break;
    }
  }
}

int32_t readTOF()
{
  return sensor.readRangeContinuousMillimeters(); 
}

ISR(INT6_vect)
{
  if(bit_is_set(PINE,7))
  {
    ticks--;
  }
  else if(bit_is_clear(PINE,7))
  {
    ticks++;
  }
}

ISR(INT5_vect)
{
  ticks = 0;
  throwCount++;
}

void bot_spot_right()
{
  baseUnbrake();
  
  PWML = 70;
  PWMR = 70;
  PWMF = 70;
  PWMB = 70;
  MOTORFF=1;
  MOTORFB=0;
  MOTORBF=1;
  MOTORBB=0;
  MOTORRF=1;
  MOTORRB=0;
  MOTORLF=1;
  MOTORLB=0;
}

void bot_spot_left()
{
  baseUnbrake();
  
  PWML = 100;
  PWMR = 100;
  PWMF = 100;
  PWMB = 100;
  MOTORFF=0;
  MOTORFB=1;
  MOTORBF=1;
  MOTORBB=0;
  MOTORRF=1;
  MOTORRB=0;
  MOTORLF=0;
  MOTORLB=1;
}

void alignToLineFollow()
{
  calcAllError();
  while(sensorValLeft[3] < onlineThresh && sensorValLeft[4] < onlineThresh && sensorValFront[3] < onlineThresh && sensorValFront[4] < onlineThresh)
  {
    calcAllError();
    bot_spot_left();
  }  
}


