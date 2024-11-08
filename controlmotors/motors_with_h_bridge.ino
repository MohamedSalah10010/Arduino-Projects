/*#define enA 7
#define in1 6
#define in2 5
#define in3 4
#define in4 3
#define enB 2*/
char data;
String Direction="";
const int in1=6;
const int in2=5;
const int in3=4;
const int in4=3;
const int enA=7;
const int enB=2;
void setup() {
  // put your setup code here, to run once:
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 100);
  analogWrite(enB, 120);
}

void loop() {
 while(Serial.available())
    {
      delay(10);
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
