#include <glcd.h>
#include "fonts/allFonts.h"        
#include "bitmaps/allBitmaps.h"    

Image_t icon;

gText textArea;              
gText textAreaArray[3];     
gText countdownArea =  gText(GLCD.CenterX, GLCD.CenterY, 1, 1, Arial_14); 

unsigned long startMillis;
unsigned int  loops = 0;
unsigned int  iter = 0;
         int  theDelay = 20; 

void setup()
{
  GLCD.Init();
  if(GLCD.Height >= 64)   
    icon = ArduinoIcon64x64;  
  else
    icon = ArduinoIcon64x32;  

  GLCD.ClearScreen(); 

  GLCD.SelectFont(System5x7, BLACK); 
  GLCD.CursorTo(2, 2);
  GLCD.print("The Engineering");
  GLCD.CursorTo(5, 3);
  GLCD.print("Projects");
}


void  loop()
{  
  
}



