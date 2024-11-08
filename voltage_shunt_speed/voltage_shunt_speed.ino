//====== voltage \ shunt and speed=============
  #include "EasyNextionLibrary.h"
//============================= SOC and Range ==================================
#define voltagePin A0
const float R1 = 100000.0;
const float R2 = 10000.0;
const float maxVoltage = 54.5;
const float minVoltage = 3.4;
const float batteryCapacityWh = 80.0 * 52;
const float avgConsumptionWhPerKm = 70;
unsigned long lastSOCTime = 0;
int soc = 0;
int rangeAvailable = 0;
int batteryVoltage = 0;
float bias = 0.145;
//============================= Shunt & Current ===================================
const float shuntResistorValue = 0.00025;
const float batteryCapacity = 160;
////#define shuntPin A1
//int current = 0;
//const float slope = 0.273; // Slope from calibration formula
//const float intercept = -1.384; // Intercept from calibration formula
//int sensorValue = 0;   // variable to store the sensor reading
//int readings[10];      // array to hold 10 readings
//int total = 0;         // variable to store the sum of readings
//int avg = 0;       // variable to store the avg of readings
const int analogPin = A1;  // Pin where the analog sensor is connected

//const float a = 0.0426;  // Adjusted coefficient
//const int baseReading = 857;
//const float baseCurrent = 0.4;
//const float maxCurrent = 21.0;
//const int maxAnalogReading = 879;  // Maximum analog reading for max current
int sensorPin = A1;   // select the input pin for the sensor
int sensorValue = 0;  // variable to store the sensor reading
int readings[10];     // array to hold 10 readings
int total = 0;        // variable to store the sum of readings
int average = 0;      // variable to store the average of readings
float current = 0.0;  // variable to store the calculated current

//============================= RPM (Hall effect sensors) ===================================
// Constants
const int rightDigitalPin = 2;  // Pin connected to the right sensor
const int timeout = 30;         // Timeout period in milliseconds

// Variables
volatile int rightCount = 0;             // Number of magnet detections
volatile unsigned long rightLast = 0;    // Last detection time
volatile unsigned long rightPeriod = 0;  // Time between detections
volatile int rightDigitalVal = 0;        // Current sensor state
volatile int rightState = LOW;           // Previous sensor state
volatile int rightResetter = 0;          // Counter for timeout

int rightRPM = 0;              // Right wheel RPM
int rightVelocity = 0;         // Right wheel velocity
int distanceCovered = 0;       // Distance covered by the wheel
int last_distanceCovered = 0;  // Last recorded distance

//Laps

#define buttonPin 18  // Button pin
static int numberOfLaps = 0;
bool lapCounted = false;  // Flag to prevent multiple lap counts

// Variables for start location
float startLat = 0.0;
float startLng = 0.0;
bool startLocationSet = false;

//Laps
int buttonState = 0;                 // Variable to store the current button state
int lastButtonState = 0;             // Variable to store the previous button state
int totalLaps = 0;                   // Counter for button presses
unsigned long lastDebounceTime = 0;  // Last time the button state was toggled
unsigned long debounceDelay = 50;    // Debounce delay in milliseconds




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial3.begin(9600);
  Serial1.begin(9600);
  // speed pins
  pinMode(rightDigitalPin, INPUT);

  attachInterrupt(digitalPinToInterrupt(buttonPin), getLapsCounterISR, CHANGE);  // Attach the interrupt to the button pin
}

void loop() {
  // put your main code here, to run repeatedly:
  printSOC();
  printCurrent();
  printRPM();
  
}


void printSOC() {
  int sensorValue = analogRead(voltagePin);
  float voltage = sensorValue * (5.0 / 1023.0) - bias;
  batteryVoltage = voltage * ((R1 + R2) / R2);
  soc = map(batteryVoltage, minVoltage, maxVoltage, 0, 100);

  // Calculate the remaining energy in the battery (in Wh)
  float remainingEnergyWh = (soc / 100.0) * batteryCapacityWh;
  // Calculate the range available (in km)
  rangeAvailable = (remainingEnergyWh / avgConsumptionWhPerKm) * 0.8;
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage);



  //============send to screen =============
  Serial3.print("GPS.volt.val=");
  Serial3.print(batteryVoltage);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);

  //  Serial.print(" V, State of Charge: ");
  //  Serial.print(soc);

  // send to screen
  Serial3.print(F("main.SOC.val="));
  Serial3.print(soc);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);

  Serial.print(" %, Range Available: ");
  Serial.print(rangeAvailable);
  Serial.println(" km");

  Serial3.print("GPS.remaining.val=");
  Serial3.print(rangeAvailable);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);

  Serial3.print(F("GPS.cover.val="));
  Serial3.print(distanceCovered);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
  //=======send to screen =================
}


//float analogToCurrent(int analogReading) {
//    if (analogReading >= maxAnalogReading) {
//    return maxCurrent;
//  } else if (analogReading <= baseReading) {
//    return baseCurrent;
//  } else {
//    // Parabolic interpolation
//    return a * pow(analogReading - baseReading, 2) + baseCurrent;
//  }
//}
void printCurrent() {
  total = 0;  // Reset total before taking new readings

  // Take 10 readings
  for (int i = 0; i < 10; i++) {
    readings[i] = analogRead(sensorPin);
    total += readings[i];
    delay(10);  // Small delay between readings
  }

  // Calculate the average
  average = total / 10;

  // Calculate current using piecewise linear interpolation
  if (average <= 190) {
    current = 0.4;  // Clamp to 0.4 A for any reading <= 180
    //  } else if (average <= 226) {
    //    // Interpolate between (180, 0.4 A) and (190, 10 A)
    //    current = 0.4 + (average - 180) * (10.0 - 0.4) / (190 - 180);
  } else if (average <= 228) {
    // Interpolate between (190, 10 A) and (210, 12 A)
    current = 10.0 + (average - 190) * (12.0 - 10.0) / (210 - 190);
    //  } else if (average <= 23) {
    //    // Interpolate between (210, 12 A) and (225, 21 A)
    //    current = 12.0 + (average - 210) * (21.0 - 12.0) / (225 - 210);
  } else {
    current = 21.0;  // Clamp to 21 A for any reading > 225
  }


  // Print the average reading and the corresponding current
  Serial.print("Average Reading: ");
  Serial.print(average);
  Serial.print(" - Calculated Current: ");
  Serial.println(current);

  //  delay(500); // Delay before the next set of readings





  // =======send to screen
  Serial3.print("GPS.current.val=");
  Serial3.print(current);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
}


void printRPM() {
  rightDigitalVal = digitalRead(rightDigitalPin);  // Read right sensor data

  // Handle rising edge detection (LOW to HIGH transition)
  if (rightDigitalVal == HIGH && rightState == LOW) {
    rightCount++;
    rightResetter = 0;
    rightPeriod = millis() - rightLast;
    rightLast = millis();
    rightState = HIGH;
  }
  // Handle falling edge detection (HIGH to LOW transition)
  else if (rightDigitalVal == LOW && rightState == HIGH) {
    rightState = LOW;
  }
  // Handle timeout condition
  else {
    rightResetter++;
    if (rightResetter >= timeout) {  // If there is no reading for the timeout period, reset the readings
      rightPeriod = 0;
      rightLast = 0;
      rightResetter = 0;
      rightRPM = 0;
    }
  }

  // Calculate RPM and velocity only if period is valid (non-zero)
  if (rightPeriod > 0) {
    rightRPM = (60000.0 / (rightPeriod * 2.0));  // Multiply by 2.0 because there are two magnets
    rightVelocity = rightRPM * 0.20724;          // Velocity calculation based on RPM
  }

  // Calculate distance covered
  last_distanceCovered = distanceCovered;
  distanceCovered = (rightCount / 2.0) * 0.55 * 3.14;  // Divide by 2 because two magnets equal one full wheel revolution
  if (distanceCovered == 0) {
    distanceCovered = last_distanceCovered;
  }

  // Print results to serial monitor
  Serial.print("Right RPM: ");
  Serial.print(rightRPM);
  Serial.print("\tRight Velocity: ");
  Serial.print(rightVelocity);
  Serial.print("\tDistance: ");
  Serial.println(distanceCovered);

  Serial3.print(F("main.kmh.val="));
  Serial3.print(rightVelocity);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print(F("main.rpm.val="));
  Serial3.print(rightRPM);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print(F("GPS.cover.val="));
  Serial3.print(distanceCovered);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
}

void getLapsCounterISR() {
  Serial.print("Laps: ");
  Serial.print(totalLaps);
  if ((millis() - lastDebounceTime) > debounceDelay) {
    int reading = digitalRead(buttonPin);
    if (reading != lastButtonState) {
      lastDebounceTime = millis();
      if (reading == HIGH) {
        delay(100);
        // Serial.print("Laps: ");
        //Serial.print(totalLaps);
        Serial3.print("main.laps.val++");
        Serial3.write(0xff);
        Serial3.write(0xff);
        Serial3.write(0xff);
      }
    }
    lastButtonState = reading;
  }
}


// void sendToNextion() {
//   // Connect TX:14    RX:15

//   Serial3.print(F("main.SOC.val="));
//   Serial3.print(soc);
//   Serial3.write(0xff);
//   Serial3.write(0xff);
//   Serial3.write(0xff);
//   //laps
//   Serial3.print(F("main.laps.val="));
//   Serial3.print(totalLaps);
//   Serial3.write(0xff);
//   Serial3.write(0xff);
//   Serial3.write(0xff);
//   Serial3.print("GPS.remaining.val=");
//   Serial3.print(rangeAvailable);
//   Serial3.write(0xff);
//   Serial3.write(0xff);
//   Serial3.write(0xff);

//   // send current, voltage
//   Serial3.print("GPS.current.val=");
//   Serial3.print(current);
//   Serial3.write(0xff);
//   Serial3.write(0xff);
//   Serial3.write(0xff);

//   Serial3.print("GPS.volt.val=");
//   Serial3.print(batteryVoltage);
//   Serial3.write(0xff);
//   Serial3.write(0xff);
//   Serial3.write(0xff);
//   current =5; batteryVoltage=45;
// }
