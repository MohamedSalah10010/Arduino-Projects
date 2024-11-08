# include "LiquidCrystal.h"

#define IN1 10
#define IN2 9
         
const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
int sensorval;
int sensorpin = 1;

void setup() {
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(sensorpin,INPUT);
  
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  
}

void loop() {
  sensorval = analogRead(sensorpin);
  float mV = ( sensorval/1024.0)*5000;             // EQUATION TO CONVERT THE LM35 READS INTO MILLIVOLTS
  float celsius = mV/10;                           // TO CALC THE CELSIUS FROM mV
 
  lcd.setCursor(0,0);        
  lcd.print("Temp is: ");                       // DISPLAYING THE TEMPRETURE ON THE FIRST LINE OF THE LCD
  lcd.println(celsius); 
 
 
 
if(celsius>25)
  {
   digitalWrite(IN1,HIGH) ;
   digitalWrite(IN2,LOW) ;           // TURNING THE COOLER ON -FAN CLOCKWISE- WHEN THE TEMP IS ABOVE 25 DEGREE CEL
   delay(500);
  
  lcd.setCursor(0, 1);
  lcd.println("cooler is turned");        // DISPLAYING THIS MESSAGE ON THE SECOND LINE OF THE LCD
  
  
   }
   
 else if (celsius<10){
   digitalWrite(IN1,LOW) ;             // TURNING THE HEATER ON -FAN ANTICLOCKWISE- WHEN THE TEMP IS UNDER 10 DEGREE CEL
   digitalWrite(IN2,HIGH) ;
   delay(500);
 
  lcd.setCursor(0, 1);
  lcd.println("heater is turned");         // DISPLAYING THIS MESSAGE ON THE SECOND LINE OF THE LCD

   }

   else { 
   digitalWrite(IN1,LOW) ;                  // TURNING OFF THE FAN
   digitalWrite(IN2,LOW) ;
   delay(500) ;
  
  lcd.setCursor(0, 1);
  lcd.println("ALL IS FINE     ");             // DISPLAYING THIS MESSAGE ON THE SECOND LINE OF THE LCD
  }

  
}
