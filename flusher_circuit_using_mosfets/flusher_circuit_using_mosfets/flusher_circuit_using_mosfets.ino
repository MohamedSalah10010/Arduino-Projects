
const int front = 13;     // the input pins
const int rear = 12;
const int right= 11;
const int left = 10;
const int hazard= 9;

const int ledfront =  7;      // the Output pins
const int ledrear =  6;
const int ledright =  5;
const int ledleft =  4;
const int ledhazard =  3;
const int lednormal=  2;


void setup() {
  pinMode(ledfront, OUTPUT);
  pinMode(ledrear, OUTPUT);
  pinMode(ledright, OUTPUT);
  pinMode(ledleft, OUTPUT);
  pinMode(ledhazard, OUTPUT);
  pinMode(lednormal, OUTPUT);
  pinMode(front, INPUT);
  pinMode(rear, INPUT);
  pinMode(right, INPUT);
  pinMode(left, INPUT);
  pinMode(hazard, INPUT);
}

void loop() {

digitalWrite(lednormal, HIGH);


  if (digitalRead(front) == HIGH) 
  {
    // turn LED on:
    digitalWrite(ledfront, HIGH);
  }
  else {
    // turn LED off:
    digitalWrite(ledfront, LOW);
  }

 if (digitalRead(rear)== HIGH)
 {
    // turn LED on:
    digitalWrite(ledrear, HIGH);
  } else 
  {
    // turn LED off:
    digitalWrite(ledrear, LOW);
  }

  if (digitalRead(right) == HIGH) 
  {
    // turn LED on:
    digitalWrite(ledright, HIGH);
    delay(500);
    digitalWrite(ledright, LOW);
    delay(500);
  } 
  else {
    // turn LED off:
    digitalWrite(ledright, LOW);
  }
 if (digitalRead(left) == HIGH)
 {
    // turn LED on:
    digitalWrite(ledleft, HIGH);
    delay(500);
    digitalWrite(ledleft, LOW);
    delay(500);
  } else 
  {
    // turn LED off:
    digitalWrite(ledleft, LOW);
  }
  
 if ( digitalRead(hazard)== HIGH) 
 {
    digitalWrite(ledleft, HIGH);
     digitalWrite(ledright, HIGH);
    delay(500);
    digitalWrite(ledleft, LOW);
    digitalWrite(ledright, LOW);
    delay(500);
  } 
  else {
    // turn LED off:
    digitalWrite(ledleft, LOW);
    digitalWrite(ledright, LOW);
  }
  
}
