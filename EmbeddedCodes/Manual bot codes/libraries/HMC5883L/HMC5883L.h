#ifndef HMC5883L_H
#define HMC5883L_H


#include <Wire.h>
#include <I2C.h>
// #include <I2C.cpp>
#include <util/delay.h>

#define HMC5883L  0x1E
#define PI  3.14159

int HMCx = 0, HMCy = 0, HMCz = 0, HMCHeading = 0;
int HMCHeadingzy, HMCHeadingyx, HMCHeadingxz;

int xHMCMagnetRaw=0;
int yHMCMagnetRaw=0;
int zHMCMagnetRaw=0;

float xHMCMagnetFiltered=0;
float yHMCMagnetFiltered=0;
float zHMCMagnetFiltered=0;

float xHMCMagnetFilteredOld=0;
float yHMCMagnetFilteredOld=0;
float zHMCMagnetFilteredOld=0;

int xHMCMagnetMax=0;
int yHMCMagnetMax=0;
int zHMCMagnetMax=0;

int xHMCMagnetMin=10000;
int yHMCMagnetMin=10000;
int zHMCMagnetMin=10000;

float xHMCMagnetMap=0;
float yHMCMagnetMap=0;
float zHMCMagnetMap=0;

float alphaHMCMagnetHMC = 0.4;
float headingSetPoint = 0;
int read_HMC5883L();
void calibrate_HMC();


#ifndef I2C_i2c
#define I2C_i2c
I2C i2c = I2C();
#endif

void HMC5883L_initialize()
{
  i2c.begin();
  i2c.timeOut(3);
  i2c.write(HMC5883L,0x02,0x00); //configure device for continuous mode
  // calibrate_HMC();
}

int read_HMC5883L()
{

  Wire.beginTransmission(HMC5883L);
  int error = Wire.endTransmission();
  if(error != 0)
    return 0;
  i2c.read(HMC5883L,0x03,6); //read 6 bytes (x,y,z) from the device
  HMCx  = i2c.receive() << 8;
  HMCx |= i2c.receive();
  HMCy  = i2c.receive() << 8;
  HMCy |= i2c.receive();
  HMCz  = i2c.receive() << 8;
  HMCz |= i2c.receive();
  xHMCMagnetRaw = HMCx;
  yHMCMagnetRaw = HMCy;
  zHMCMagnetRaw = HMCz;

  xHMCMagnetFiltered= xHMCMagnetFilteredOld + alphaHMCMagnetHMC * (xHMCMagnetRaw - xHMCMagnetFilteredOld);
  yHMCMagnetFiltered= yHMCMagnetFilteredOld + alphaHMCMagnetHMC * (yHMCMagnetRaw - yHMCMagnetFilteredOld);
  zHMCMagnetFiltered= zHMCMagnetFilteredOld + alphaHMCMagnetHMC * (zHMCMagnetRaw - zHMCMagnetFilteredOld);

  xHMCMagnetFilteredOld = xHMCMagnetFiltered;
  yHMCMagnetFilteredOld = yHMCMagnetFiltered;
  zHMCMagnetFilteredOld = zHMCMagnetFiltered;
  HMCx = xHMCMagnetFiltered;
  HMCy = yHMCMagnetFiltered;
  HMCz = zHMCMagnetFiltered;

  HMCHeading   = (atan2(HMCy, HMCx) * 180) / PI - headingSetPoint;
  HMCHeadingzy = (atan2(HMCz, HMCy) * 180) / PI - headingSetPoint;
  HMCHeadingyx = (atan2(HMCy, HMCx) * 180) / PI - headingSetPoint;
  HMCHeadingxz = (atan2(HMCx, HMCz) * 180) / PI - headingSetPoint;

  // Normalize to 0-360
  // if (HMCHeading < 0)
  // {
  //   HMCHeading = 360 + HMCHeading;
  // }
  if (HMCHeading < -180)
  {
    HMCHeading = HMCHeading + 360;
  }
  else if (HMCHeading > 180)
  {
    HMCHeading = HMCHeading - 360;
  }
  // if(i2c.available())
  // return 1;
  // else
  // return 0;
  
  return 1;
}

void calibrate_HMC()
{
  i2c.read(HMC5883L,0x03,6); //read 6 bytes (x,y,z) from the device
  HMCx  = i2c.receive() << 8;
  HMCx |= i2c.receive();
  HMCy  = i2c.receive() << 8;
  HMCy |= i2c.receive();
  HMCz  = i2c.receive() << 8;
  HMCz |= i2c.receive();
  xHMCMagnetRaw = HMCx;
  yHMCMagnetRaw = HMCy;
  zHMCMagnetRaw = HMCz;

  xHMCMagnetFiltered= xHMCMagnetFilteredOld + alphaHMCMagnetHMC * (xHMCMagnetRaw - xHMCMagnetFilteredOld);
  yHMCMagnetFiltered= yHMCMagnetFilteredOld + alphaHMCMagnetHMC * (yHMCMagnetRaw - yHMCMagnetFilteredOld);
  zHMCMagnetFiltered= zHMCMagnetFilteredOld + alphaHMCMagnetHMC * (zHMCMagnetRaw - zHMCMagnetFilteredOld);

  xHMCMagnetFilteredOld = xHMCMagnetFiltered;
  yHMCMagnetFilteredOld = yHMCMagnetFiltered;
  zHMCMagnetFilteredOld = zHMCMagnetFiltered;
  HMCx = xHMCMagnetFiltered;
  HMCy = yHMCMagnetFiltered;
  HMCz = zHMCMagnetFiltered;

  HMCHeading = (atan2(HMCy, HMCx) * 180) / PI;
  headingSetPoint = HMCHeading;
}

#endif