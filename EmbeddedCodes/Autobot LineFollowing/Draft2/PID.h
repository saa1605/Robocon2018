#include "SensorReadings.h"
//#include "EncoderReadings.h"  ---------------------------

#define LKp 0
#define LKi 0
#define LKd 0

#define EKp 0
#define EKi 0
#define EKd 0

#define LineIntegralLimit 0
#define EncoderIntegralLimit 0

#define LineWeight 0
#define SmoothLineWeight 0

#define EncoderWeight 0
#define SmoothEncoderWeight 0

float LineError = 0, LineDiff = 0, LineIntegral = 0, LinePID = 0, PrevLineError = 0;
float EncoderError = 0, EncoderDiff = 0, EncoderIntegral = 0, EncoderPID = 0, PrevEncoderError = 0;

float LinePos = get_line_position();
float EncoderPos = 0;
//float EncoderPos = get_encoder_position();  ------------------

void calc_Line_PID()
{
  LineError = LinePos;
  LineDiff = LineError - PrevLineError;
  LineIntegral += LineError;
  if(LineIntegral > LineIntegralLimit)
    LineIntegral = LineIntegralLimit;
  else if(LineIntegral < (-1)*LineIntegralLimit)
    LineIntegral = (-1)*LineIntegralLimit;
  LinePID = LKp*LineError + LKi*LineIntegral + LKd*LineDiff;
  PrevLineError = LineError;
}

void calc_Encoder_PID()
{
  EncoderError = EncoderPos;
  EncoderDiff = EncoderError - PrevEncoderError;
  EncoderIntegral += EncoderError;
  if(EncoderIntegral > EncoderIntegralLimit)
    EncoderIntegral = EncoderIntegralLimit;
  else if(EncoderIntegral < (-1)*EncoderIntegralLimit)
    EncoderIntegral = (-1)*EncoderIntegralLimit;
  EncoderPID = EKp*EncoderError + EKi*EncoderIntegral + EKd*EncoderDiff;
  PrevEncoderError = EncoderError;
}

float get_PID_Error()
{
  calc_Line_PID();
  calc_Encoder_PID();
  if(isSmooth)
  {
    return (SmoothLineWeight*LinePID + SmoothEncoderWeight*EncoderPID); 
  }
  else
  {
    return (LineWeight*LinePID + EncoderWeight*EncoderPID); 
  }
}
