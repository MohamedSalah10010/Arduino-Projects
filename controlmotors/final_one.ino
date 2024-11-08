 int enB= 10;//for speed control of motor 1
 int enA= 11;//for speed control of motor 2
 int m1= 8;  // for direction control for motor 1
 int m2= 9; 
 int m3= 7;  // for direction control for motor 2
 int m4= 6; 
void setup() {
  // put your setup code here, to run once:
pinMode(6, OUTPUT);
pinMode(7, OUTPUT);
pinMode(8, OUTPUT);
pinMode(9, OUTPUT);
pinMode(10, OUTPUT);
pinMode(11, OUTPUT);
}
void loop(){
  // declration of forward function
  forward();
  delay(2000);
  
 
}

void forward(){  //for forward direction movement. 
  
  analogWrite(enA,255);// speed of 100 for motor 1
  digitalWrite(m1,HIGH); // CW direction for motor 1
  digitalWrite(m2,LOW);
  
  analogWrite(enB,130);// speed of 130 for motor 2
  digitalWrite(m4,LOW);// CCW direction for motor 2
  digitalWrite(m3,HIGH);
  
  
}
