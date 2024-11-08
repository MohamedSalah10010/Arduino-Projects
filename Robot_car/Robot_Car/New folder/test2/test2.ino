#define in1 13              // input 1  pin
#define in2 12              // input 2  pin
#define en_right 11         // right motor speed controller pin
#define en_left 10          // left motor speed controller  pin
#define in3 9               // input 3  pin
#define in4 8               // input 4  pin
#define sen_right 7         // right sensor input pin
#define sen_left 6          // left sensor input pin
#define high_speed 80      // speed of the motor in the OUTSIDE of the turn
#define turn_speed 120
#define low_speed  0      // speed of the motor in the INSIDE of the turn 

int right_sensor_state;     // stores right sensor data
int left_sensor_state;      // stores left sensor data

void setup() {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(en_right, OUTPUT);
  pinMode(en_left, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(sen_right, INPUT);     
  pinMode(sen_left, INPUT);    
  
}

void loop() {
  right_sensor_state = digitalRead(sen_right);                // reads right sensor data
  left_sensor_state = digitalRead(sen_left);                  // reads left sensor data
  
  if (right_sensor_state == 0 && left_sensor_state == 0){       //Forward
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    analogWrite(en_right, high_speed);
    analogWrite(en_left, high_speed);
  }
  if (right_sensor_state == 1 && left_sensor_state == 0){       //Steer Right
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    analogWrite(en_right, low_speed);
    analogWrite(en_left, turn_speed);
  }
  if (right_sensor_state == 0 && left_sensor_state == 1){       //Steer Left
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    analogWrite(en_right, turn_speed);
    analogWrite(en_left, low_speed);
  }
  if (right_sensor_state == 1 && left_sensor_state == 1){       // Bonus Round - Steer Right
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
    digitalWrite(in3,HIGH);
    digitalWrite(in4,LOW);
    analogWrite(en_right, 70);                      
    analogWrite(en_left, 150);                                         // Time to be waited after entering bonus line
  }
}
