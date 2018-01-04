//#define MOSI PB2
//#define MISO PB3 
//#define SCLK PB1 
//#define SS PB0

uint8_t x = 0xFF;
uint8_t LSBData;
uint8_t MSBData;
int16_t adcValue;

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

#define lineWeight 1
#define smoothLineWeight 1

#define encoderWeight 0
#define smoothEncoderWeight 0

float lineError = 0, lineDiff = 0, lineIntegral = 0, linePid = 0, prevLineError = 0;
float encoderError = 0, encoderDiff = 0, encoderIntegral = 0, encoderPid = 0, prevEncoderError = 0;

int pidFlag = 1, pidMode = 1;
// float linePos = getLinePosition();
// float encoderPos = getEncoderPosition();

void calcLinePid(float linePos)
{
  lineError = linePos;
  lineDiff = lineError - prevLineError;
  lineIntegral += lineError;
  if(lineIntegral > lineIntegralLimit)
    lineIntegral = lineIntegralLimit;
  else if(lineIntegral < (-1)*lineIntegralLimit)
    lineIntegral = (-1)*lineIntegralLimit;
  if(pidMode == 1)
    linePid = 0.5*lKp*lineError;
  else if(pidMode == 2)
    linePid = 0.45*lKp*lineError + 0.85*lKi*lineIntegral;
  else if(pidMode == 3)
    linePid = 0.6*lKp*lineError + 0.5*lKi*lineIntegral + 0.13*lKd*lineDiff;
  prevLineError = lineError;
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

float getPidError()
{
  float linePos = getLinePosition();
  float encoderPos = getEncoderPosition();
  calcLinePid(linePos);
  calcEncoderPid(encoderPos);
  if(isSmooth)
  {
    return (smoothLineWeight*linePid + smoothEncoderWeight*encoderPid);
  }
  else
  {
    return (lineWeight*linePid + encoderWeight*encoderPid); 
  }
}
