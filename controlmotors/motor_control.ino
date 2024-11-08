 int en1= 6;//for speed control of motor 1
 int en2= 5;//for speed control of motor 2
 int m1= 12;  // for direction control
 int m2= 11; 
 int m3= 10;  
 int m4= 9; 
 char com;

void setup(){

 pinMode(en1,OUTPUT);
 pinMode(en2,OUTPUT);
 pinMode(m1,OUTPUT);
 pinMode(m2,OUTPUT);
 pinMode(m3,OUTPUT);
 pinMode(m4,OUTPUT);
 Serial.begin(9600);//Set the baud rate to the Bluetooth module defult.
}
void loop(){
  if(Serial.available() > 0){ 
    com = Serial.read(); 
   // Stop(); 
    switch(com){
    case 'F':Forward();break;
    case 'B':Back();break;
    case 'L':Left();break;
    case 'R':Right();break;
    }
void Forward(){
  analogWrite(6,255);// For forward movement
  digitalWrite(12,HIGH);
  digitalWrite(11,LOW);
  analogWrite(5,255);
  digitalWrite(10,HIGH);
  digitalWrite(9,LOW);
  Serial.println("FORWARD");
}
void Right(){ 
analogWrite(6,255); //for right direction movement. 
  digitalWrite(12,HIGH);
  digitalWrite(11,LOW);
  analogWrite(5,0);
  digitalWrite(10,HIGH);
  digitalWrite(9,LOW);
  Serial.println("RIGHT");
}
  void Left(){
  analogWrite(6,00);// for left direction
  digitalWrite(12,HIGH); 
  digitalWrite(11,LOW);
  analogWrite(5,255);
  digitalWrite(10,HIGH);
  digitalWrite(9,LOW);
  Serial.println("LEFT");
  }
  void Back()
  { analogWrite(6,255); //for backward direction movement. 
  digitalWrite(12,LOW);
  digitalWrite(11,HIGH);
  analogWrite(5,255);
  digitalWrite(10,LOW);
  digitalWrite(9,HIGH);
  Serial.println("BACKWARD");
  }
  void Stop(){
  analogWrite(6,0);//for stoping. 
  digitalWrite(12,LOW);
  digitalWrite(11,LOW);
  analogWrite(5,0);
  digitalWrite(10,LOW);
  digitalWrite(9,LOW);
  Serial.println("STOP");
  }
