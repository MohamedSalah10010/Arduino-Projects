 int en1= 11;//for speed control of motor 1
 int en2= A0;//for speed control of motor 2
 int m1= 13;  // for direction control
 int m2= 12; 
 int m3= 10;  
 int m4= 9; 
int val= 200;
void setup() {
  // put your setup code here, to run once:
pinMode(13, OUTPUT);
pinMode(12, OUTPUT);
pinMode(11, OUTPUT);
pinMode(10, OUTPUT);
pinMode(9, OUTPUT);
pinMode(A0, OUTPUT);
}
void loop(){

  forward();
  delay(2000);
 
}

void forward(){  //for forward direction movement. 

  analogWrite(11,100);
  
 digitalWrite(m1,LOW);
  digitalWrite(m2,HIGH);
  
  analogWrite(en2, val);
  
  digitalWrite(m3,LOW);
  digitalWrite(m4,HIGH);
  
}
