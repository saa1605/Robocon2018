#include <SPI.h>  
#include <Pixy.h>
 
Pixy pixy;    // This is the main Pixy object

void setup()
{
  Serial.begin(9600);
  Serial.print("Starting...\n");

  pixy.init();
  
  Serial.println("Initiation Complete");
}

void pixy_parameters()
{
  uint16_t x, y, width, height, angle;
  static int i = 0;
  uint16_t blocks;
//  char buf[32];
  
  blocks = pixy.getBlocks();    // grab blocks!
  
  if(blocks)    // If there are detect blocks, print them!
  {
    i++;
    
    if(i%50 == 0)   // do this (print) every 50 frames because printing 
    {               // every frame would bog down the Arduino
    
      for(int j = 0; j < blocks; j++)
      {
        x = pixy.blocks[j].x;
        y = pixy.blocks[j].y;
        width = pixy.blocks[j].width;
        height = pixy.blocks[j].height;
        angle = pixy.blocks[j].angle;
        Serial.print("x = ");Serial.print(x);Serial.print(" ");
        Serial.print("y = ");Serial.print(y);Serial.print(" ");
        Serial.print("width = ");Serial.print(width);Serial.print(" ");
        Serial.print("height = ");Serial.print(height);Serial.print(" ");
        Serial.print("angle = ");Serial.print(angle);Serial.print(" ");
        Serial.print("\n");
      }
    }
  }
}

void loop()
{
  pixy_parameters();
}
