#include <io128.h>
#include <sra128.h>
#include <pinDefsAutoNew.h>
#include "mcp.h"

#define numOfSensors 8

int sensorMinLeft[numOfSensors]  = {4000,  4000,  4000,  4000, 4000,  4000,  4000,  4000};
int sensorMaxLeft[numOfSensors]  = {0, 0, 0, 0, 0, 0, 0, 0};
int sensorMinRight[numOfSensors] = {4000,  4000,  4000,  4000,  4000,  4000,  4000, 4000};
int sensorMaxRight[numOfSensors] = {0, 0, 0, 0, 0, 0, 0, 0};

int sensorMinFront[numOfSensors]  = {4000,  4000,  4000,  4000,  4000,  4000,  4000,  4000};
int sensorMaxFront[numOfSensors]  = {0, 0, 0, 0, 0, 0, 0, 0};
int sensorMinBack[numOfSensors] = {4000,  4000,  4000,  4000,  4000,  4000,  4000,  4000};
int sensorMaxBack[numOfSensors] = {0, 0, 0, 0, 0, 0, 0, 0};

//calibrated values of 3rd Feb
//int sensorMinLeft[numOfSensors] = {563, 575, 575, 556, 545, 554, 570, 912};
//int sensorMaxLeft[numOfSensors] = {2743, 2851, 2896, 2712, 2258, 2593, 2875, 3071};
//int sensorMinRight[numOfSensors] = {645, 594, 871, 557, 561, 660, 572, 554};
//int sensorMaxRight[numOfSensors] = {2998, 2918, 3029, 2430, 2556, 2791, 2883, 2310};
//int sensorMinFront[numOfSensors] = {934, 553, 541, 539, 579, 548, 546, 570};
//int sensorMaxFront[numOfSensors] = {3106, 2965, 2848, 2815, 1499, 2648, 2656, 2916};
//int sensorMinBack[numOfSensors] = {1167, 839, 905, 945, 591, 536, 591, 556};
//int sensorMaxBack[numOfSensors] = {2992, 2903, 2935, 2925, 1955, 1730, 2769, 2366};

//calibrated values of 1st Feb
//int sensorMinLeft[numOfSensors] = {572, 573, 573, 563, 571, 579, 595, 992};
//int sensorMaxLeft[numOfSensors] = {2378, 2402, 2404, 2115, 2289, 2467, 2713, 3004};
//int sensorMinRight[numOfSensors] = {907, 665, 912, 576, 652, 809, 653, 660};
//int sensorMaxRight[numOfSensors] = {2894, 2859, 2930, 2167, 2382, 2655, 2757, 2828};
//int sensorMinFront[numOfSensors] = {1126, 595, 578, 582, 585, 591, 769, 828};
//int sensorMaxFront[numOfSensors] = {3118, 2875, 2743, 2773, 2609, 2630, 2885, 2847};
//int sensorMinBack[numOfSensors] = {957, 861, 700, 647, 613, 554, 599, 576};
//int sensorMaxBack[numOfSensors] = {2979, 2890, 2942, 2941, 1939, 1641, 2681, 2229};

//calibrated values of 31st Jan Very Special
//int sensorMinLeft[numOfSensors] = {568, 565, 566, 558, 561, 572, 585, 649};
//int sensorMaxLeft[numOfSensors] = {2197, 2223, 2253, 1978, 1829, 2362, 2583, 2892};
//int sensorMinRight[numOfSensors] = {810, 622, 660, 593, 645, 797, 646, 646};
//int sensorMaxRight[numOfSensors] = {2922, 2804, 2833, 2568, 2344, 2553, 2638, 2671};
//int sensorMinFront[numOfSensors] = {559, 576, 543, 601, 806, 746, 646, 994};
//int sensorMaxFront[numOfSensors] = {2084, 2522, 1488, 1874, 2903, 2895, 2901, 2989};
//int sensorMinBack[numOfSensors] = {612, 615, 581, 572, 593, 568, 585, 975};
//int sensorMaxBack[numOfSensors] = {2811, 2875, 2593, 2588, 2728, 2649, 2750, 3023};

//calibrated values of 31st Jan special
//int sensorMinLeft[numOfSensors] = {575, 553, 556, 551, 563, 572, 583, 735};
//int sensorMaxLeft[numOfSensors] = {2641, 2181, 2242, 1975, 1912, 2300, 2434, 2861};
//int sensorMinRight[numOfSensors] = {640, 602, 622, 588, 626, 768, 625, 632};
//int sensorMaxRight[numOfSensors] = {2858, 2759, 2831, 2690, 2459, 2666, 2695, 2738};
//int sensorMinFront[numOfSensors] = {560, 577, 542, 600, 695, 690, 669, 985};
//int sensorMaxFront[numOfSensors] = {2244, 2545, 1532, 1750, 2867, 2897, 2863, 2905};
//int sensorMinBack[numOfSensors] = {582, 584, 568, 560, 560, 558, 565, 689};
//int sensorMaxBack[numOfSensors] = {2735, 2902, 2637, 2675, 2718, 2633, 2781, 2980};

//calibrated values of 31st Jan
//int sensorMinLeft[numOfSensors] = {597, 556, 562, 602, 568, 577, 588, 638};
//int sensorMaxLeft[numOfSensors] = {1018, 1988, 2163, 886, 2087, 2337, 2545, 2840};
//int sensorMinRight[numOfSensors] = {774, 626, 646, 595, 747, 898, 749, 748};
//int sensorMaxRight[numOfSensors] = {2821, 2736, 2812, 2598, 2417, 2682, 2734, 2682};
//int sensorMinFront[numOfSensors] = {571, 589, 551, 611, 958, 961, 938, 1275};
//int sensorMaxFront[numOfSensors] = {2362, 2785, 1826, 2030, 3033, 3033, 3033, 3033};
//int sensorMinBack[numOfSensors] = {601, 610, 575, 573, 579, 571, 589, 983};
//int sensorMaxBack[numOfSensors] = {2782, 2901, 2626, 2594, 2727, 2712, 2873, 3088};

//calobrated values of 30th Jan
//int sensorMinLeft[numOfSensors] = {582, 542, 548, 587, 590, 601, 763, 979};
//int sensorMaxLeft[numOfSensors] = {655, 1790, 2170, 998, 2640, 2701, 2944, 2983};
//int sensorMinRight[numOfSensors] = {638, 595, 604, 576, 570, 612, 590, 590};
//int sensorMaxRight[numOfSensors] = {2882, 2804, 2807, 2673, 2468, 2740, 2850, 2757};
//int sensorMinFront[numOfSensors] = {551, 561, 535, 590, 657, 709, 683, 1024};
//int sensorMaxFront[numOfSensors] = {2085, 2395, 1330, 1762, 2902, 2874, 2868, 2978};
//int sensorMinBack[numOfSensors] = {624, 610, 565, 564, 590, 583, 602, 1087};
//int sensorMaxBack[numOfSensors] = {2843, 2925, 2545, 2557, 2765, 2727, 2792, 3004};

//calibrated values of 29th Jan
//int sensorMinLeft[numOfSensors]  = {484, 572, 575, 582, 640, 624, 654, 739};
//int sensorMaxLeft[numOfSensors]  = {592, 2568, 2611, 2740, 2769, 2672, 2927, 2979};
//int sensorMinRight[numOfSensors] = {688, 612, 619, 587, 580, 601, 603, 601};
//int sensorMaxRight[numOfSensors] = {2829, 2745, 2787, 2517, 2295, 2528, 2608, 2655};
//
//int sensorMinFront[numOfSensors]  = {565, 575, 548, 609, 883, 864, 868, 1153};
//int sensorMaxFront[numOfSensors]  = {2036, 2219, 1322, 1798, 2946, 2932, 2966, 3010};
//int sensorMinBack[numOfSensors] = {704, 673, 581, 582, 595, 589, 658, 1317};
//int sensorMaxBack[numOfSensors] = {2871, 2902, 2500, 2524, 2717, 2710, 2832, 3071};

int sensorRawRight[numOfSensors], sensorValRight[numOfSensors], sensorRawLeft[numOfSensors], sensorValLeft[numOfSensors];
int sensorRawFront[numOfSensors], sensorValFront[numOfSensors], sensorRawBack[numOfSensors], sensorValBack[numOfSensors];

int toggle = 0;   //0 - for calibrating white and 1 - for calibrating black

void setup()
{
  Serial.begin(115200);
  Serial.println("Setup Start");
  spiMasterInit();
  DDRD &= ~(1 << PD3);  //D3 button
  PORTD |= (1 << PD3);  //D3 button
  Serial.println("Setup End");
}

void loop()
{
  readSensorFront();
  readSensorBack();
  readSensorLeft();
  readSensorRight();

  calcSensorValFront();
  calcSensorValBack();
  calcSensorValLeft();
  calcSensorValRight();

  SerialPrint();

  if (bit_is_clear(PIND, 3))
  {
    _delay_ms(1000);
    toggle = 0;       //calibrate white
    calibrateLeft();
    calibrateRight();
    calibrateFront();
    calibrateBack();

    while (bit_is_set(PIND, 3));
    _delay_ms(1000);
    toggle = 1;      //calibrate black
    calibrateLeft();
    calibrateRight();
    calibrateFront();
    calibrateBack();
    while (bit_is_set(PIND, 3));
  }
}

void SerialPrint()
{
  for (int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorRawFront[i]); Serial.print(' ');
  }
  Serial.print("\t");
  for (int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorRawBack[i]); Serial.print(' ');
  }
  Serial.print("\t");

  for (int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorRawLeft[i]); Serial.print(' ');
  }
  Serial.print("\t");
  for (int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorRawRight[i]); Serial.print(' ');
  }
  Serial.print("\t");

  //  for(int i = 0; i < numOfSensors; i++)
  //  {
  //   Serial.print(sensorValFront[i]);Serial.print(' ');
  //  }
  //  Serial.print("\t");
  //  for(int i = 0; i < numOfSensors; i++)
  //  {
  //   Serial.print(sensorValBack[i]);Serial.print(' ');
  //  }
  //  Serial.print("\t");
  //
  //  for(int i = 0; i < numOfSensors; i++)
  //  {
  //   Serial.print(sensorValLeft[i]);Serial.print(' ');
  //  }
  //  Serial.print("\t");
  //  for(int i = 0; i < numOfSensors; i++)
  //  {
  //   Serial.print(sensorValRight[i]);Serial.print(' ');
  //  }
  //  Serial.print("\t");

  Serial.println("");
}

void readSensorFront() //front dekha toh left wala 0
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

void readSensorBack() //front dekha toh left wala 0
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

void readSensorRight()  //left dekha toh left wala 0
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

void readSensorLeft()  //left dekha toh left wala 0
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

void calibrateFront()
{
  //  Serial.println("Calibrating Front");
  unsigned int time = 10;
  if (toggle == 0)
  {
    for (int i = 0; i < time; i++)
    {
      readSensorFront();
      for (int j = 0; j < numOfSensors; j++)
      {
        sensorMinFront[j] += sensorRawFront[j];
      }
    }
    for (int j = 0; j < numOfSensors; j++)
    {
      sensorMinFront[j] = sensorMinFront[j] / 10;
    }
  }
  else if (toggle == 1)
  {
    for (int i = 0; i < time; i++)
    {
      readSensorFront();
      for (int j = 0; j < numOfSensors; j++)
      {
        sensorMaxFront[j] += sensorRawFront[j];
      }
    }
    for (int j = 0; j < numOfSensors; j++)
    {
      sensorMaxFront[j] = sensorMaxFront[j] / 10;
    }
  }
  //  Serial.println("Front Calibrated");
  //  Serial.print("Min : ");
  //  for(int i = 0; i < numOfSensors; i++)
  //  {
  //    Serial.print(sensorMinFront[i]);Serial.print(",");
  //    Serial.print(" ");
  //  }
  //  Serial.print(" Max : ");
  //  for(int i = 0; i < numOfSensors; i++)
  //  {
  //    Serial.print(sensorMaxFront[i]);Serial.print(",");
  //    Serial.print(" ");
  //  }

  Serial.print("int sensorMinFront[numOfSensors] = {");
  for (int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMinFront[i]);
    if (i < 7)
    {
      Serial.print(","); Serial.print(" ");
    }
  }
  Serial.println("};");

  Serial.print("int sensorMaxFront[numOfSensors] = {");
  for (int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMaxFront[i]);
    if (i < 7)
    {
      Serial.print(","); Serial.print(" ");
    }
  }
  Serial.println("};");
}

void calibrateBack()
{
  //  Serial.println("Calibrating Back");
  unsigned int time = 10;
  if (toggle == 0)
  {
    for (int i = 0; i < time; i++)
    {
      readSensorBack();
      for (int j = 0; j < numOfSensors; j++)
      {
        sensorMinBack[j] += sensorRawBack[j];
      }
    }
    for (int j = 0; j < numOfSensors; j++)
    {
      sensorMinBack[j] = sensorMinBack[j] / 10;
    }

  }
  else if (toggle == 1)
  {
    for (int i = 0; i < time; i++)
    {
      readSensorBack();
      for (int j = 0; j < numOfSensors; j++)
      {
        sensorMaxBack[j] += sensorRawBack[j];
      }
    }
    for (int j = 0; j < numOfSensors; j++)
    {
      sensorMaxBack[j] = sensorMaxBack[j] / 10;
    }
  }
  //  Serial.println("Back Calibrated");
  //  Serial.print("Min : ");
  //  for(int i = 0; i < numOfSensors; i++)
  //  {
  //    Serial.print(sensorMinBack[i]);Serial.print(",");
  //    Serial.print(" ");
  //  }
  //  Serial.print(" Max : ");
  //  for(int i = 0; i < numOfSensors; i++)
  //  {
  //    Serial.print(sensorMaxBack[i]);Serial.print(",");
  //    Serial.print(" ");
  //  }
  //  Serial.print('\n');
  //  _delay_ms(2000);

  Serial.print("int sensorMinBack[numOfSensors] = {");
  for (int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMinBack[i]);
    if (i < 7)
    {
      Serial.print(","); Serial.print(" ");
    }
  }
  Serial.println("};");

  Serial.print("int sensorMaxBack[numOfSensors] = {");
  for (int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMaxBack[i]);
    if (i < 7)
    {
      Serial.print(","); Serial.print(" ");
    }
  }
  Serial.println("};");
  _delay_ms(2000);
}

void calibrateRight()
{
  //  Serial.println("Calibrating Right");
  unsigned int time = 10;

  if (toggle == 0)
  {
    for (int i = 0; i < time; i++)
    {
      readSensorRight();
      for (int j = 0; j < numOfSensors; j++)
      {
        sensorMinRight[j] += sensorRawRight[j];
      }
    }
    for (int j = 0; j < numOfSensors; j++)
    {
      sensorMinRight[j] = sensorMinRight[j] / 10;
    }

  }
  else if (toggle == 1)
  {
    for (int i = 0; i < time; i++)
    {
      readSensorRight();
      for (int j = 0; j < numOfSensors; j++)
      {
        sensorMaxRight[j] += sensorRawRight[j];
      }
    }
    for (int j = 0; j < numOfSensors; j++)
    {
      sensorMaxRight[j] = sensorMaxRight[j] / 10;
    }
  }
  //  Serial.println("Right Calibrated");
  //  Serial.print("Min : ");
  //  for(int i = 0; i < numOfSensors; i++)
  //  {
  //    Serial.print(sensorMinRight[i]);Serial.print(",");
  //    Serial.print(" ");
  //  }
  //  Serial.print(" Max : ");
  //  for(int i = 0; i < numOfSensors; i++)
  //  {
  //    Serial.print(sensorMaxRight[i]);Serial.print(",");
  //    Serial.print(" ");
  //  }
  //  Serial.print('\n');

  Serial.print("int sensorMinRight[numOfSensors] = {");
  for (int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMinRight[i]);
    if (i < 7)
    {
      Serial.print(","); Serial.print(" ");
    }
  }
  Serial.println("};");

  Serial.print("int sensorMaxRight[numOfSensors] = {");
  for (int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMaxRight[i]);
    if (i < 7)
    {
      Serial.print(","); Serial.print(" ");
    }
  }
  Serial.println("};");
}

void calibrateLeft()
{
  //  Serial.println("Calibrating Left");
  unsigned int time = 10;

  if (toggle == 0)
  {
    for (int i = 0; i < time; i++)
    {
      readSensorLeft();
      for (int j = 0; j < numOfSensors; j++)
      {
        sensorMinLeft[j] += sensorRawLeft[j];
      }
    }
    for (int j = 0; j  < numOfSensors; j++)
    {
      sensorMinLeft[j] = sensorMinLeft[j] / 10;
    }

  }
  else if (toggle == 1)
  {
    for (int i = 0; i < time; i++)
    {
      readSensorLeft();
      for (int j = 0; j < numOfSensors; j++)
      {
        sensorMaxLeft[j] += sensorRawLeft[j];
      }
    }
    for (int j = 0; j < numOfSensors; j++)
    {
      sensorMaxLeft[j] = sensorMaxLeft[j] / 10;
    }
  }
  //  Serial.println("Left Calibrated");
  //  Serial.print("Min : ");
  //  for(int i = 0; i < numOfSensors; i++)
  //  {
  //    Serial.print(sensorMinLeft[i]);Serial.print(",");
  //    Serial.print(" ");
  //  }
  //  Serial.print("Max : ");
  //  for(int i = 0; i < numOfSensors; i++)
  //  {
  //    Serial.print(sensorMaxLeft[i]);Serial.print(",");
  //    Serial.print(" ");
  //  }
  //  Serial.print('\n');

  Serial.print("int sensorMinLeft[numOfSensors] = {");
  for (int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMinLeft[i]);
    if (i < 7)
    {
      Serial.print(","); Serial.print(" ");
    }
  }
  Serial.println("};");

  Serial.print("int sensorMaxLeft[numOfSensors] = {");
  for (int i = 0; i < numOfSensors; i++)
  {
    Serial.print(sensorMaxLeft[i]);
    if (i < 7)
    {
      Serial.print(","); Serial.print(" ");
    }
  }
  Serial.println("};");
}

