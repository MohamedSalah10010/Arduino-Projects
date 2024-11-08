/*
  Button

  Turns on and off a light emitting diode(LED) connected to digital pins,
  when pressing a pushbutton attached to pin pins.

  The circuit:
  - LEDs attached from pins 5 and 7 to ground through 220 ohm resistors
  - pushbuttons attached to pins 4, 6, and 8 from +5V
  - 10K resistors attached to pins 4, 6, and 8 from ground

  - Note: on most Arduinos there is already an LED on the board
    attached to pin 13.

  created 2005
  by DojoDave <http://www.0j0.org>
  modified 30 Aug 2011
  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Button
*/

// constants won't change. They're used here to set pin numbers:
const int buttonPin_1 = 4;  // the number of the pushbutton pin
const int buttonPin_2 = 6;
const int buttonPin_3 = 8;
const int buttonPin_4 = 9;
const int rightled = 5;  
const int leftled = 7;  
const int highled = 10; 
// variables will change:
int left_flag = 0;  // variable for reading the pushbutton status
int right_flag = 0;  // variable for reading the pushbutton status
int hazard_flag = 0;
int LIGHT_flag = 0;
int buttonState_1 = HIGH;
int buttonState_2 = HIGH;  // variable for reading the pushbutton status
int buttonState_3 = HIGH;
int buttonState_4 = HIGH;  // variable for reading the pushbutton status
int lastButtonState_1 = HIGH;
int lastButtonState_2 = HIGH;
int lastButtonState_3 = HIGH;
int lastButtonState_4 = HIGH;
void setup() {
  // initialize the LED pins as outputs:
  pinMode(rightled, OUTPUT);
  pinMode(leftled, OUTPUT);
  pinMode(highled, OUTPUT);
  // initialize the pushbutton pins as inputs:
  pinMode(buttonPin_1, INPUT_PULLUP);
  pinMode(buttonPin_2, INPUT_PULLUP);
  pinMode(buttonPin_3, INPUT_PULLUP);
  pinMode(buttonPin_4, INPUT_PULLUP);
}

void loop() {
  int reading_1 = digitalRead(buttonPin_1);
  int reading_2 = digitalRead(buttonPin_2);
  int reading_3 = digitalRead(buttonPin_3);

  if (reading_1 == LOW && lastButtonState_1 == HIGH) {
    // delay(50);  // simple debounce delay
    if (digitalRead(buttonPin_1) == LOW) {
      right_flag = !right_flag;
      if (right_flag) {
        left_flag = 0;
        hazard_flag = 0;
      }
    }
  }
  if (reading_2 == LOW && lastButtonState_2 == HIGH) {
    // delay(50);  // simple debounce delay
    if (digitalRead(buttonPin_2) == LOW) {
      left_flag = !left_flag;
      if (left_flag) {
        right_flag = 0;
        hazard_flag = 0;
      }
    }
  }
  if (reading_3 == LOW && lastButtonState_3 == HIGH) {
    // delay(50);  // simple debounce delay
    if (digitalRead(buttonPin_3) == LOW) {
      hazard_flag = !hazard_flag;
      if (hazard_flag) {
        right_flag = 0;
        left_flag = 0;
      }
    }
  }

  lastButtonState_1 = reading_1;
  lastButtonState_2 = reading_2;
  lastButtonState_3 = reading_3;

  if (right_flag) {
    flusher(rightled);
  } else {
    digitalWrite(rightled, LOW);
  }
  
  if (left_flag) {
    flusher(leftled);
  } else {
    digitalWrite(leftled, LOW);
  }
  
  if (hazard_flag) {
    hazard(rightled, leftled);
  } else {
    digitalWrite(rightled, LOW);
    digitalWrite(leftled, LOW);
  }
}

void flusher(int ledpin) {
  digitalWrite(ledpin, HIGH);
  delay(400);
  digitalWrite(ledpin, LOW);
  delay(400);
}

void hazard(int rightLed, int leftLed) {
  digitalWrite(rightLed, HIGH);
  digitalWrite(leftLed, HIGH);
  delay(400);
  digitalWrite(rightLed, LOW);
  digitalWrite(leftLed, LOW);
  delay(400);
}
