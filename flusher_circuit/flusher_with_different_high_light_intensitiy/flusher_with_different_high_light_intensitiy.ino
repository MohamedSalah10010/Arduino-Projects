// constants won't change. They're used here to set pin numbers:
const int buttonPinLeft = 9;    // left
const int buttonPinRight = 8;   // right
const int buttonPinHazard = 10;  // hazard
const int buttonPinHighLight = 4; // high light
const int rightLedPin = 3;  
const int leftLedPin = 5;  
const int highLedPin = 6; 

// variables will change:
int leftFlag = 0;    // variable for reading the pushbutton status
int rightFlag = 0;   // variable for reading the pushbutton status
int hazardFlag = 0;
int lightFlag = 0;
int buttonStateHighLight = LOW;  // variable for reading the pushbutton status
int lastButtonStateHighLight = LOW;

// the following variables are long's because the time, measured in milliseconds,
// will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTimeHighLight = 0;
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

// Timing for flasher and hazard functions
unsigned long lastFlashTime = 0;
const unsigned long flashInterval = 500;  // Interval at which to blink (milliseconds)

// LED states
bool leftLedState = LOW;
bool rightLedState = LOW;

void setup() {
  // initialize the LED pins as outputs:
  pinMode(rightLedPin, OUTPUT);
  pinMode(leftLedPin, OUTPUT);
  pinMode(highLedPin, OUTPUT);
  // initialize the pushbutton pins as inputs:
  pinMode(buttonPinLeft, INPUT_PULLUP);
  pinMode(buttonPinRight, INPUT_PULLUP);
  pinMode(buttonPinHazard, INPUT_PULLUP);
  pinMode(buttonPinHighLight, INPUT_PULLUP);
  digitalWrite(rightLedPin, LOW);
  digitalWrite(leftLedPin, LOW);
  digitalWrite(highLedPin, LOW);
}

void loop() {
  int readingLeft = digitalRead(buttonPinLeft);
  int readingRight = digitalRead(buttonPinRight);
  int readingHazard = digitalRead(buttonPinHazard);
  int readingHighLight = digitalRead(buttonPinHighLight);

  // Handle left flasher
  if (readingLeft == LOW) {
    leftFlag = 1;
    rightFlag = 0;
    hazardFlag = 0;
  } else {
    leftFlag = 0;
  }

  // Handle right flasher
  if (readingRight == LOW) {
    rightFlag = 1;
    leftFlag = 0;
    hazardFlag = 0;
  } else {
    rightFlag = 0;
  }

  // Handle hazard lights
  if (readingHazard == LOW) {
    hazardFlag = 1;
    leftFlag = 0;
    rightFlag = 0;
  } else {
    hazardFlag = 0;
  }

  // Handle high light button with debounce
  if ((millis() - lastDebounceTimeHighLight) > debounceDelay) {
    if (readingHighLight != lastButtonStateHighLight) {
      lastDebounceTimeHighLight = millis();
      if (readingHighLight == LOW && lastButtonStateHighLight == HIGH) {
        lightFlag++;
        highLedController(highLedPin);
      }
      lastButtonStateHighLight = readingHighLight;
    }
  }

  // Update flasher and hazard lights
  if (millis() - lastFlashTime >= flashInterval) {
    lastFlashTime = millis(); // Update the time

    if (leftFlag) {
      leftLedState = !leftLedState;
      digitalWrite(leftLedPin, leftLedState);
    } else {
      digitalWrite(leftLedPin, LOW);
    }

    if (rightFlag) {
      rightLedState = !rightLedState;
      digitalWrite(rightLedPin, rightLedState);
    } else {
      digitalWrite(rightLedPin, LOW);
    }

    if (hazardFlag) {
      leftLedState = !leftLedState;
      rightLedState = !rightLedState; // Synchronize hazard lights
      digitalWrite(leftLedPin, leftLedState);
      digitalWrite(rightLedPin, leftLedState);
    } else if (!leftFlag && !rightFlag) {
      digitalWrite(leftLedPin, LOW);
      digitalWrite(rightLedPin, LOW);
    }
  }
}

void highLedController(int highLedPin) {
  if (lightFlag > 2) {
    lightFlag = 0;
  }
  if (lightFlag == 2) {
    analogWrite(highLedPin, 255); // Full intensity
  }
  if (lightFlag == 1) {
    analogWrite(highLedPin, 150); // Half intensity
  }
  if (lightFlag == 0) {
    analogWrite(highLedPin, 0); // Turn off
  }
}
