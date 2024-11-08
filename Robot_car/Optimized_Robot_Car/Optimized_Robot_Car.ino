int mover(int right_speed,int left_speed){          // Main function to move the car only in forward direction (NO BACKWARD)
  digitalWrite(13,HIGH);                            // Input 1 pin
  digitalWrite(12,LOW);                             // Input 2 pin
  digitalWrite(9,HIGH);                             // Input 3 pin
  digitalWrite(8,LOW);                              // Input 4 pin
  analogWrite(11, right_speed);                     // Enable 1 pin
  analogWrite(10, left_speed);                      // Enable 2 pin
}
void setup() {
  for (int pin = 6; pin <=13; pin ++){              // Identifies each pin as input or output    
    if (pin == 6 or pin ==7){                       // Input pins for the data from the sensor
      pinMode(pin,INPUT);
    }
    else{                                           // Output pins of the for the driver
      pinMode(pin,OUTPUT);
    }}
}
void loop() {
  if (digitalRead(7) == 0 && digitalRead(6) == 0){       // Moves Forward
    mover(150,150);}
  if (digitalRead(7) == 1 && digitalRead(6) == 0){       // Steers Right
    mover(70,150);}
  if (digitalRead(7) == 0 && digitalRead(6) == 1){       // Steers Left
    mover(150,70);}
  if (digitalRead(7) == 1 && digitalRead(6) == 1){       // Bonus Line (Steers Right)
    mover(70,150);
    delay(300);}                                         // Time to be waited after detecting bonus line   
}
