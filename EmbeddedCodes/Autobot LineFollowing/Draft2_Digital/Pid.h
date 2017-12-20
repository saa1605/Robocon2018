#include "SensorReadings.h"
#include "EncoderReadings.h"

#define lKp 0
#define lKi 0
#define lKd 0

#define eKp 0
#define eKi 0
#define eKd 0

#define lineIntegralLimit 0
#define encoderIntegralLimit 0

#define lineWeight 0
#define smoothLineWeight 0

#define encoderWeight 0
#define smoothEncoderWeight 0

float lineError = 0, lineDiff = 0, lineIntegral = 0, linePid = 0, prevLineError = 0;
float encoderError = 0, encoderDiff = 0, encoderIntegral = 0, encoderPid = 0, prevEncoderError = 0;

float linePos = getLinePosition();
float encoderPos = getEncoderPosition();

void calcLinePid()
{
  lineError = linePos;
  lineDiff = lineError - prevLineError;
  lineIntegral += lineError;
  if(lineIntegral > lineIntegralLimit)
    lineIntegral = lineIntegralLimit;
  else if(lineIntegral < (-1)*lineIntegralLimit)
    lineIntegral = (-1)*lineIntegralLimit;
  linePid = lKp*lineError + lKi*lineIntegral + lKd*lineDiff;
  prevLineError = lineError;
}

void calcEncoderPid()
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
  calcLinePid();
  calcEncoderPid();
  if(isSmooth)
  {
    return (smoothLineWeight*linePid + smoothEncoderWeight*encoderPid); 
  }
  else
  {
    return (lineWeight*linePid + encoderWeight*encoderPid); 
  }
}
