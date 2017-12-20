 // No rights reserved.

 //   Redistribution and use in source and binary forms, with or without
 //   modification, are permitted

#ifndef MPUI2CNEW_H
#define MPUI2CNEW_H

// #include <Wire.h>
// #include <C:\Program Files (x86)/arduino-1.0.5-r2/libraries/Wire/Wire.cpp>
#include <Math.h>
#include <avr/io.h>
#include <stdlib.h>
#include <compat/deprecated.h>
#include <util/delay.h>
#include <avr/eeprom.h>
//#include <io.h>
#include "pinDefsForNewBoardAtmega128Robocon2017.h"
#include <avr/interrupt.h>
#include <inttypes.h>

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C


#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

#define    PI   3.1415

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#ifndef I2C_i2c
#define I2C_i2c
I2C i2c = I2C();
#endif

int read_MPU();
//I2C i2c;

uint8_t Buf[14];

float Pitch = 0;
float Roll = 0;
float Yaw = 0;
float YawU = 0;

float alpha = 0, beta = 0, alphaSetPoint = 0;

float alphaAccel = 0.4;
float alphaMagnetMPU = 0.4;

unsigned int xOffset=0;
unsigned int yOffset=0;
unsigned int zOffset=0;

int xRaw=0;
int yRaw=0;
int zRaw=0;

float xFiltered=0;
float yFiltered=0;
float zFiltered=0;

float xFilteredOld=0;
float yFilteredOld=0;
float zFilteredOld=0;

float xAccel=0;
float yAccel=0;
float zAccel=0;

int xMagnetRaw=0;
int yMagnetRaw=0;
int zMagnetRaw=0;

float xMagnetFiltered=0;
float yMagnetFiltered=0;
float zMagnetFiltered=0;

float xMagnetFilteredOld=0;
float yMagnetFilteredOld=0;
float zMagnetFilteredOld=0;

int xMagnetMax=0;
int yMagnetMax=0;
int zMagnetMax=0;

int xMagnetMin=10000;
int yMagnetMin=10000;
int zMagnetMin=10000;

float xMagnetMap=0;
float yMagnetMap=0;
float zMagnetMap=0;
// I2C i2c;
// declare
int16_t mx,my,mz,ax,ay,az,gx,gy,gz;


// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  i2c.read(Address, Register, Nbytes , Data);
}


// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  i2c.write(Address, Register, Data);
}

void FilterAD()
{
  // read from AD and subtract the offset
  xRaw=ax;//analogRead(xAxis)-xOffset;
  yRaw=ay;//analogRead(yAxis)-yOffset;
  zRaw=az;//analogRead(zAxis)-zOffset;

  //Digital Low Pass - compare: (for accelerometer)
  //http://en.wikipedia.org/wiki/Low-pass_filter
  xFiltered= xFilteredOld + alphaAccel * (xRaw - xFilteredOld);
  yFiltered= yFilteredOld + alphaAccel * (yRaw - yFilteredOld);
  zFiltered= zFilteredOld + alphaAccel * (zRaw - zFilteredOld);

  xFilteredOld = xFiltered;
  yFilteredOld = yFiltered;
  zFilteredOld = zFiltered;



  //read from Compass
  //Digital Low Pass for Noise Reduction for Magnetic Sensor
  xMagnetRaw=mx;
  yMagnetRaw=my;
  zMagnetRaw=mz;
  xMagnetFiltered= xMagnetFilteredOld + alphaMagnetMPU * (xMagnetRaw - xMagnetFilteredOld);
  yMagnetFiltered= yMagnetFilteredOld + alphaMagnetMPU * (yMagnetRaw - yMagnetFilteredOld);
  zMagnetFiltered= zMagnetFilteredOld + alphaMagnetMPU * (zMagnetRaw - zMagnetFilteredOld);

  xMagnetFilteredOld = xMagnetFiltered;
  yMagnetFilteredOld = yMagnetFiltered;
  zMagnetFilteredOld = zMagnetFiltered;

}

void getAzimuth()
{
  //this part is required to normalize the magnetic vector
  if(xMagnetFiltered > xMagnetMax)
    xMagnetMax = xMagnetFiltered;
  else if(xMagnetFiltered < xMagnetMin)
    xMagnetMin = xMagnetFiltered;
  
  if(yMagnetFiltered > yMagnetMax)
    yMagnetMax = yMagnetFiltered;
  else if(yMagnetFiltered < yMagnetMin)
    yMagnetMin = yMagnetFiltered;
  
  if(zMagnetFiltered > zMagnetMax)
    zMagnetMax = zMagnetFiltered;
  else if(zMagnetFiltered < zMagnetMin)
    zMagnetMin = zMagnetFiltered;

  float norm;

  xMagnetMap = float(map(xMagnetFiltered, xMagnetMin, xMagnetMax, -10000, 10000))/10000.0;
  yMagnetMap = float(map(yMagnetFiltered, yMagnetMin, yMagnetMax, -10000, 10000))/10000.0;
  zMagnetMap = float(map(zMagnetFiltered, zMagnetMin, zMagnetMax, -10000, 10000))/10000.0;


  //normalize the magnetic vector
  norm = sqrt( sq(xMagnetMap) + sq(yMagnetMap) + sq(zMagnetMap));
  xMagnetMap /= norm;
  yMagnetMap /= norm;
  zMagnetMap /= norm;

  //compare Applications of Magnetic Sensors for Low Cost Compass Systems by Michael J. Caruso
  //for the compensated Yaw equations...
  //http://www.ssec.honeywell.com/magnetic/datasheets/lowcost.pdf
  Yaw  = atan2( (-yMagnetMap*cos(Roll) + zMagnetMap*sin(Roll) ) , xMagnetMap*cos(Pitch) + zMagnetMap*sin(Pitch)*sin(Roll)+ zMagnetMap*sin(Pitch)*cos(Roll)) *180/PI;
  YawU = atan2(  -yMagnetMap, xMagnetMap) * 180/PI;
}

void AD2Degree()
{ 
  // 3.3 = Vref; 1023 = 10Bit AD; 0.8 = Sensivity Accelerometer
  // (compare datasheet of your accelerometer)
  // the *Accel must be between -1 and 1; you may have to
  // to add/subtract +1 depending on the orientation of the accelerometer
  // (like me on the zAccel)
  // they are not necessary, but are useful for debugging
  xAccel=xFiltered *3.3 / (1023.0*0.8);       
  yAccel=yFiltered *3.3 / (1023.0*0.8);       
  zAccel=zFiltered *3.3 / (1023.0*0.8)+1.0;

  // Calculate Pitch and Roll (compare Application Note AN3461 from Freescale
  // http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
  // Microsoft Excel switches the values for atan2
  // -> this info can make your life easier :-D
  //angled are radian, for degree (* 180/3.14159)
  Roll   = atan2(  yAccel ,   sqrt((xAccel)*(xAccel) + (zAccel)*(zAccel)));  
  Pitch  = atan2(  xAccel ,   sqrt((yAccel)*(yAccel) + (zAccel)*(zAccel)));
}

float getPitch()
{
  return (int(10*Pitch*180/PI)/10.0);
}

float getRoll()
{
  return (int(10*Roll*180/PI)/10.0);
}

void getAccelOffset()
{ //you can make approx 60 iterations because we use an unsigned int 
  //otherwise you get an overflow. But 60 iterations should be fine
  for (int i=1; i <= 60; i++){ 
    I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);    
  xOffset +=     -(Buf[0]<<8 | Buf[1]);//analogRead(xAxis);   
  yOffset +=     -(Buf[2]<<8 | Buf[3]);// analogRead(yAxis);
  zOffset += az =  Buf[4]<<8 | Buf[5] ;// analogRead(zAxis);
  } 
  xOffset /=60;   
  yOffset /=60;
  zOffset /=60;
  /*
  Serial.print("xOffset: FIFIFIFIFIFIFIFIFIFIFIFI");
   Serial.print(xOffset);
   Serial.print("   yOffset: ");
   Serial.print(yOffset);
   Serial.print("   zOffset: ");
   Serial.println(zOffset);
   */
  // _delay_ms(1000);
}

void MPU_initialize()
{
  // Arduino initializations
  i2c.begin();
  i2c.timeOut(3);

  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);


  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);

  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);

  // _delay_ms(1000);

  getAccelOffset();
  // read_MPU();
}

int read_MPU()
{
  // Display time
  //  Serial.print (millis()-ti,DEC);//***********************************************************
  //Serial.print ("\t");
  // Wire.beginTransmission(MPU9250_ADDRESS);
  // int error = Wire.endTransmission();
  // LED0 = 0;
  // LED1 = 0;
  // LED2 = 0;
  // LED3 = 0;
  // if(error != 0)
  //   return 0;
  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf);

  // Create 16 bits values from 8 bits data

  // Accelerometer
  ax=-(Buf[0]<<8 | Buf[1]);
  ay=-(Buf[2]<<8 | Buf[3]);
  az=Buf[4]<<8 | Buf[5];

  // Gyroscope
  gx=-(Buf[8]<<8 | Buf[9]);
  gy=-(Buf[10]<<8 | Buf[11]);
  gz=Buf[12]<<8 | Buf[13];

  // Display values

  // :::  Magnetometer ::: 


  // Read register Status 1 and wait for the DRDY: Data Ready
 //  unsigned long st_time = millis();
 //  uint8_t ST1;
 //  do
 //  {
 //    if(millis() - st_time > 3)
 //      return 0;
 //    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
 //  }
 //  while (!(ST1&0x01));

 // // Read magnetometer data  
 //  uint8_t Mag[7];  
 //  I2Cread(MAG_ADDRESS,0x03,7,Mag);

 //  // Create 16 bits values from 8 bits data

 //  // Magnetometer
 //  mx=-(Mag[3]<<8 | Mag[2]);
 //  my=-(Mag[1]<<8 | Mag[0]);
 //  mz=-(Mag[5]<<8 | Mag[4]);

  FilterAD();
  AD2Degree();
  // getAzimuth();
  Roll = (int(10*Roll*180/PI)/10.0);
  Pitch = (int(10*Roll*180/PI)/10.0);
  alpha = Roll - alphaSetPoint;
  beta  = Pitch;
  return 1;
  // if(i2c.available())
  // {
  //   return 1;
  // }
  // else
  // {
  //   return 0;
  // }
}

void calibrateMPUAlpha()
{
  read_MPU();
  // _delay_ms(150);
  alphaSetPoint = alpha + alphaSetPoint;
}

#endif