//#define MOSI PB2
//#define MISO PB3 
//#define SCLK PB1 
//#define SS PB0

// uint8_t x = 0xFF;
// uint8_t LSBData;
// uint8_t MSBData;
// int16_t adcValue;

#include "SensorReadings.h"
#include "EncoderReadings.h"

//#define lKp 0
//#define lKi 0
//#define lKd 0
float lKp = 0, lKd = 0, lKi = 0;

#define eKp 0
#define eKi 0
#define eKd 0

#define lineIntegralLimit 0
#define encoderIntegralLimit 0

#define lineRotationalWeight 0.5
#define smoothRotationalLineWeight 0

#define lineTranslationalWeight 0.5
#define smoothTranslationalLineWeight 0

#define encoderWeight 0
#define smoothEncoderWeight 0

float lineRotationalError = 0, lineRotationalDiff = 0, lineRotationalIntegral = 0, lineRotationalPid = 0, prevLineRotationalError = 0;
float lineTranslationalError = 0, lineTranslationalDiff = 0, lineTranslationalIntegral = 0, lineTranslationalPid = 0, prevLineTranslationalError = 0;
float encoderError = 0, encoderDiff = 0, encoderIntegral = 0, encoderPid = 0, prevEncoderError = 0;

int pidMode = 1;
// float linePos = getLinePosition();
// float encoderPos = getEncoderPosition();

void calcLineRotationalPid(float lineRotationalPos)
{
  lineRotationalError = lineRotationalPos;
  lineRotationalDiff = lineRotationalError - prevLineRotationalError;
  lineRotationalIntegral += lineRotationalError;
  if(lineRotationalIntegral > lineRotationalIntegralLimit)
    lineRotationalIntegral = lineRotationalIntegralLimit;
  else if(lineRotationalIntegral < (-1)*lineRotationalIntegralLimit)
    lineRotationalIntegral = (-1)*lineRotationalIntegralLimit;
  if(pidMode == 1)
    lineRotationalPid = 0.5*lKp*lineRotationalError;
  else if(pidMode == 2)
    lineRotationalPid = 0.45*lKp*lineRotationalError + 0.85*lKi*lineRotationalIntegral;
  else if(pidMode == 3)
    lineRotationalPid = 0.6*lKp*lineRotationalError + 0.5*lKi*lineRotationalIntegral + 0.13*lKd*lineRotationalDiff;
  prevLineRotationalError = lineRotationalError;
}

void calcLineTranslationalPid(float lineTranslationalPos)
{
  lineTranslationalError = lineTranslationalPos;
  lineTranslationalDiff = lineTranslationalError - prevLineTranslationalError;
  lineTranslationalIntegral += lineTranslationalError;
  if(lineRotationalIntegral > lineRotationalIntegralLimit)
    lineTranslationalIntegral = lineTranslationalIntegralLimit;
  else if(lineTranslationalIntegral < (-1)*lineTranslationalIntegralLimit)
    lineTranslationalIntegral = (-1)*lineTranslationalIntegralLimit;
  if(pidMode == 1)
    lineTranslationalPid = 0.5*lKp*lineTranslationalError;
  else if(pidMode == 2)
    lineTranslationalPid = 0.45*lKp*lineTranslationalError + 0.85*lKi*lineTranslationalIntegral;
  else if(pidMode == 3)
    lineTranslationalPid = 0.6*lKp*lineTranslationalError + 0.5*lKi*lineTranslationalIntegral + 0.13*lKd*lineTranslationalDiff;
  prevLineTranslationalError = lineTranslationalError;
}

void calcEncoderPid(float encoderPos)
{
  encoderError = encoderPos;
  encoderDiff = encoderError - prevEncoderError;
  encoderIntegral += encoderError;
  if(encoderIntegral > encoderIntegralLimit)
    encoderIntegral = encoderIntegralLimit;
  else if(encoderIntegral < (-1)*encoderIntegralLimit)
    encoderIntegral = (-1)*encoderIntegralLimit;
  encoderPid = eKp*encoderError + eKi*encoderIntegral + eKd*encoderDiff;
  prevEncoderError = encoderError;
}

getPidError()
{
  getLinePosition();
  float encoderPos = getEncoderPosition();
  calcLineRotationalPid(lineRotationalPos);
  calcLineTranslationalPid(lineTranslationalPos);
  calcEncoderPid(encoderPos);
//  if(isSmooth)
//  {
//    return (smoothLineWeight*linePid + smoothEncoderWeight*encoderPid);
//  }
//  else
//  {
//    return (lineRotationalWeight*lineRotationalPid + lineTranslationalWeight*lineTranslationalPid + encoderWeight*encoderPid); 
//  }
}
