const int in1 = 7;
const int in2 = 6;
const int in3 = 5;
const int in4 = 4;
const int enA = 10;
const int enB = 11;
const int pwm = 2;
char data;
String Direction="";

void setup() {
  pinMode(pwm,OUTPUT) ;    //we have to set PWM pin as output
  pinMode(in2,OUTPUT) ;  //Logic pins are also set as output
  pinMode(in2,OUTPUT) ;
  pinMode(enA,OUTPUT) ;    //we have to set PWM pin as output
  pinMode(in3,OUTPUT) ;  //Logic pins are also set as output
  pinMode(in4,OUTPUT) ;
  pinMode(enB,OUTPUT) ;
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 100);
  analogWrite(enB, 200);
}

void loop() {
  while(Serial.available())
    {
      delay(200);
      data=Serial.read();
      Direction+=data;
    }
  if(Direction=="left")                                       // Left direction command
    {
      digitalWrite(in1,HIGH);
      digitalWrite(in2,LOW);
      digitalWrite(in3,HIGH);
      digitalWrite(in4,LOW);
      Serial.println("Motor run left direction");
    }
    if(Direction=="right")                                      // Right direction command
    {
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);
       digitalWrite(in3,LOW);
      digitalWrite(in4,HIGH);
      Serial.println("Motor run right direction");
    }
    Direction="";

}
