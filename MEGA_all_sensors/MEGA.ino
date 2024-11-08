//============================= SOC and Range ==================================
#define voltagePin A0
const float R1 = 100000.0;
const float R2 = 10000.0;
const float maxVoltage = 54.5;
const float minVoltage = 3.4;
const float batteryCapacityWh = 80.0 * 52;
const float averageConsumptionWhPerKm = 70;
unsigned long lastSOCTime = 0;
float soc = 0;
float rangeAvailable = 0;
float batteryVoltage = 0;
//============================= Shunt & Current ===================================
const float shuntResistorValue = 0.00025;
const float batteryCapacity = 160;
#define shuntPin A1
float current = 0;
//============================= RPM (Hall effect sensors) ===================================
//right sensor variables
#define rightDigitalPin 2
int rightVelocity, rightRPM, rightDigitalVal, rightCount, rightResetter;
bool rightState;
unsigned long rightPeriod, rightLast;
//left sensor variables
#define leftDigitalPin 3
int leftVelocity, leftRPM, leftDigitalVal, leftResetter, leftCount;
bool leftState;
unsigned long leftPeriod, leftLast;
// General variables for the sensors
int velocity;      // stores the velocity calculated from the RPM.
int averageRPM;    // stores the average reading of the left and right RPMs.
int resetter;      // a variable that is used to reset the readings if there are no pulses measured.
int timeout = 30;  // when this value is reached, all readings will be reset
int distanceCovered = 0;
int last_distanceCovered = 0;

//============================= ULTRASONIC ===================================
#include <NewPing.h>
#define FRONT_TRIG 6
#define FRONT_ECHO 7
#define LEFT_TRIG 8
#define LEFT_ECHO 9
#define RIGHT_TRIG 10
#define RIGHT_ECHO 11
#define BACK_TRIG 12
#define BACK_ECHO 13
#define MAX_DISTANCE 50 
//LEDS
#define FRONT_LED 22
#define BACK_LED 23
#define LEFT_LED 24
#define RIGHT_LED 25
NewPing sonarFront(FRONT_TRIG, FRONT_ECHO, MAX_DISTANCE);
NewPing sonarLeft(LEFT_TRIG, LEFT_ECHO, MAX_DISTANCE);
NewPing sonarRight(RIGHT_TRIG, RIGHT_ECHO, MAX_DISTANCE);
NewPing sonarBack(BACK_TRIG, BACK_ECHO, MAX_DISTANCE);

int distanceFront, distanceLeft, distanceRight, distanceBack;

//============================= RFID ===================================
/*
   SPI SS            53
   SPI MOSI          51
   SPI MISO          50
   SPI SCK           52
*/
#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 53
#define RST_PIN -1

MFRC522 rfid(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;
const byte correctID[4] = { 0xA3, 0x96, 0x37, 0x0F };
bool flagID = false;

//============================= GPS ===================================
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
// Connect the GPS TX (transmit) pin to Digital 5
// Connect the GPS RX (receive) pin to Digital 4
SoftwareSerial mySerial(5, 4);
Adafruit_GPS GPS(&mySerial);
static const float locationThreshold = 0.0001;  // Adjust based on GPS accuracy
// Pin and lap variables
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

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to Serial and listen to the raw GPS sentences
#define GPSECHO true

//============================= Heart Sensor ===================================
/*
SDA: 20 SCL:21
*/

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
MAX30105 particleSensor;
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100];   //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100];   //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength = 20;  //data length
int32_t spo2;               //SPO2 value
int8_t validSPO2;           //indicator to show if the SPO2 calculation is valid
int32_t heartRate;          //heart rate value
int8_t validHeartRate;      //indicator to show if the heart rate calculation is valid

unsigned long previousMillis = 0;  // stores last time sensor check was made
bool isSensorPresent = false;
byte sampleCount = 0;  // keeps track of the number of samples collected


void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
Serial1.begin(9600);
  //===================Hall effect=========================
  pinMode(rightDigitalPin, INPUT);
  pinMode(leftDigitalPin, INPUT);
  //===================US LEDS=========================
  // pinMode(LEFT_LED, OUTPUT);
  // pinMode(RIGHT_LED, OUTPUT);
  // pinMode(FRONT_LED, OUTPUT);
  // pinMode(BACK_LED, OUTPUT);
   //===================GPS=========================
  // GPS.begin(9600);
  // pinMode(buttonPin, INPUT_PULLUP);  // Button pin as input with internal pull-up resistor
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // //  // Set the update rate
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
  // delay(1000);

  // //===================Heart Sensor=========================
  // // SDA: 20    SCL: 21
  // checkHeartSensor();
  //===================RFID=========================
  SPI.begin();      // Init SPI bus
  rfid.PCD_Init();  // Init MFRC522
}


uint32_t timer = millis();

void loop() {
  unsigned long currentMillis = millis();

  // // // 1. Waiting for the correct ID (RFID)
  // while (!flagID) {
  //   checkID();
  // }
  // 2. Get distances
  printDistance();
  // //3. Get Speed
  // printRPM();
  // //4. Get SOC  every 3 seconds
  //  if (currentMillis - lastSOCTime >= 5000) {
  //    printSOC();
  //    lastSOCTime = currentMillis;
  //  }
  // // 5. Get Current
  // printCurrent();
  // // 6. Get Heart Rate, SPO2, TEMP
  // // printHeartBeat();
  // //7. Get laps count
  // // getLapsCounter();
  // //8. get location
  // // printLocation();
  // sendToNextion();
}


void printRPM() {
  rightDigitalVal = digitalRead(rightDigitalPin);  //read right sensor data
  if (rightDigitalVal == HIGH && rightState == false) {
    rightCount++;
    rightResetter = 0;
    rightState = true;
    rightPeriod = millis() - rightLast;
    rightLast = millis();
  } else if (rightDigitalVal == LOW && rightState == true) {
    rightState = false;
  } else {
    rightResetter++;
    if (rightResetter >= timeout) {  // if there is no reading for the timeout period, reset the readings
      rightPeriod = 0;
      rightLast = 0;
      rightResetter = 0;
    }
  }

  rightRPM = 60000 / rightPeriod;
  rightVelocity = rightRPM * 0.20724;

  leftDigitalVal = digitalRead(leftDigitalPin);  //read left sensor data
  if (leftDigitalVal == LOW && leftState == false) {
    leftCount++;
    leftResetter = 0;
    leftState = true;
    leftPeriod = millis() - leftLast;
    leftLast = millis();
  } else if (leftDigitalVal == HIGH && leftState == true) {
    leftState = false;
  } else {
    leftResetter++;
    if (leftResetter >= timeout) {  // if there is no reading for the timeout period, reset the readings
      leftPeriod = 0;
      leftLast = 0;
      leftResetter = 0;
    }
  }
  leftRPM = 60000 / leftPeriod;
  leftVelocity = leftRPM * 0.20724;

  if (leftRPM == 0) {  // We are trying to get the average of both RPMs, so that if the left sensor is gone, we still can get the RPM from only the right wheel.
    averageRPM = rightRPM;
  } else if (rightRPM == 0 || rightRPM - leftRPM > 300) {  // same as above but we add another condition to check if the sensed RPM is real. if not, ignore the right sensor.
    averageRPM = leftRPM;
  } else {
    averageRPM = (rightRPM + leftRPM) / 2.0;  // calculate the average RPM if both RPMs are real.
  }
  velocity = averageRPM * 0.20724;  // get the velocity using the average RPM. (0.3) is the diameter of the wheel. (3.0 / 25.0) * 3.14 * 0.55

  last_distanceCovered = distanceCovered;
  distanceCovered = ((leftCount + rightCount) / 2.0) * 0.55 * 3.14;
  if (distanceCovered == 0) {
    distanceCovered = last_distanceCovered;
  }


  Serial.print("Right RPM: ");
  Serial.print(rightRPM);
  Serial.print("\tRight Velocity: ");
  Serial.print(rightVelocity);
  Serial.print("\tleft RPM: ");
  Serial.print(leftRPM);
  Serial.print("\tleft Velocity: ");
  Serial.print(leftVelocity);
  Serial.print("\tVelocity: ");
  Serial.print(velocity);
  Serial.print("\tDistance: ");
  Serial.println(distanceCovered);
}
void printSOC() {
  int sensorValue = analogRead(voltagePin);
  float voltage = sensorValue * (5.0 / 1023.0);
  batteryVoltage = voltage * ((R1 + R2) / R2);
  soc = map(batteryVoltage, minVoltage, maxVoltage, 0, 100);

  // Calculate the remaining energy in the battery (in Wh)
  float remainingEnergyWh = (soc / 100.0) * batteryCapacityWh;
  // Calculate the range available (in km)
  rangeAvailable = (remainingEnergyWh / averageConsumptionWhPerKm) * 0.8;
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage);
  Serial.print(" V, State of Charge: ");
  Serial.print(soc);
  Serial.print(" %, Range Available: ");
  Serial.print(rangeAvailable);
  Serial.println(" km");
}
void printCurrent() {
  // Measure voltage drop across shunt resistor
  int analogValue = analogRead(shuntPin);
  Serial.print(analogValue);
  float voltageDrop = analogValue * (5.0 / 1024);  // Voltage in volts
  // Calculate current
  current = voltageDrop / shuntResistorValue;  // Current in amperes

  Serial.print("Current: ");
  Serial.println(current);
}
void printDistance() {
  distanceFront = sonarFront.ping_cm();
  distanceLeft = sonarLeft.ping_cm();
  distanceRight = sonarRight.ping_cm();
  distanceBack = sonarBack.ping_cm();

  Serial.print("Front Distance = ");
  Serial.print(distanceFront);
  Serial.println(" cm");
  Serial.print("Left Distance = ");
  Serial.print(distanceLeft);
  Serial.println(" cm");
  Serial.print("Right Distance = ");
  Serial.print(distanceRight);
  Serial.println(" cm");
  Serial.print("Back Distance = ");
  Serial.print(distanceBack);
  Serial.println(" cm");

  if (distanceFront > 0 && distanceFront < MAX_DISTANCE) {
    digitalWrite(FRONT_LED, HIGH);
  } else {
    digitalWrite(FRONT_LED, LOW);
  }

  if (distanceBack > 0 && distanceBack < MAX_DISTANCE) {
    digitalWrite(BACK_LED, HIGH);
  } else {
    digitalWrite(BACK_LED, LOW);
  }

  if (distanceRight > 0 && distanceRight < MAX_DISTANCE) {
    digitalWrite(RIGHT_LED, HIGH);
  } else {
    digitalWrite(RIGHT_LED, LOW);
  }

  if (distanceLeft > 0 && distanceLeft < MAX_DISTANCE) {
    digitalWrite(LEFT_LED, HIGH);
  } else {
    digitalWrite(LEFT_LED, LOW);
  }
}
void sendToNextion() {
  // Connect TX:14    RX:15
  Serial3.print(F("main.kmh.val="));
  Serial3.print(velocity);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print(F("main.rpm.val="));
  Serial3.print(averageRPM);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.print(F("main.SOC.val="));
  Serial3.print(soc);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
  //laps
  Serial3.print(F("main.laps.val="));
  Serial3.print(totalLaps);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
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
  // send current, voltage
  Serial3.print("GPS.current.val=");
  Serial3.print(current);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);

  Serial3.print("GPS.volt.val=");
  Serial3.print(batteryVoltage);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
  current =5; batteryVoltage=45;
}
void checkID() {
  // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
  if (!rfid.PICC_IsNewCardPresent())
    return;

  // Verify if the NUID has been readed
  if (!rfid.PICC_ReadCardSerial())
    return;

  if (rfid.uid.uidByte[0] == correctID[0] || rfid.uid.uidByte[1] == correctID[1] || rfid.uid.uidByte[2] == correctID[2] || rfid.uid.uidByte[3] == correctID[3]) {
    flagID = true;
    Serial3.print(F("page start"));
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
  } 
  Serial.println(flagID);

  Serial.println(F("The NUID tag is:"));
  Serial.print(F("In hex: "));
  printHex(rfid.uid.uidByte, rfid.uid.size);
  Serial.println();
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}
void printHex(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

void printHeartBeat() {
  unsigned long currentMillis = millis();

  if (isSensorPresent) {
    if (sampleCount < bufferLength) {
      if (particleSensor.available()) {
        redBuffer[sampleCount] = particleSensor.getRed();
        irBuffer[sampleCount] = particleSensor.getIR();
        particleSensor.nextSample();  // We're finished with this sample so move to next sample
        sampleCount++;
      } else {
        particleSensor.check();  // Check the sensor for new data
      }
    }
    if (sampleCount >= bufferLength) {
      // Calculate heart rate and SpO2
      maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

      Serial.print(F("HR="));
      Serial.print(heartRate, DEC);
      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);
      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);
      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);

      // Reset the sample count to start collecting new samples
      sampleCount = 0;
    }
  } else {
    // check if the sensor is present every second
    if (currentMillis - previousMillis >= 1000) {
      previousMillis = currentMillis;
      checkHeartSensor();
    }
  }
}
void checkHeartSensor() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    isSensorPresent = false;
  } else {
    isSensorPresent = true;
    particleSensor.setup();
    particleSensor.enableDIETEMPRDY();
  }
}
// to be tested
void printLocation() {
  char c = GPS.read();
  // if you want to Serial, this is a good time to do it!
  //if ((c) && (GPSECHO))
  //Serial.write(c);

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flagID to false

    if (!GPS.parse(GPS.lastNMEA()))  // this also sets the newNMEAreceived() flagID to false
      return;                        // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis();  // reset the timer

    Serial.print("\nTime: ");
    if (GPS.hour < 10) {
      Serial.print('0');
    }
    Serial.print(GPS.hour, DEC);
    Serial.print(':');
    if (GPS.minute < 10) {
      Serial.print('0');
    }
    Serial.print(GPS.minute, DEC);
    Serial.print(':');
    if (GPS.seconds < 10) {
      Serial.print('0');
    }
    Serial.print(GPS.seconds, DEC);
    Serial.print('.');
    if (GPS.milliseconds < 10) {
      Serial.print("00");
    } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
      Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC);
    Serial.print('/');
    Serial.print(GPS.month, DEC);
    Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: ");
    Serial.print((int)GPS.fix);
    Serial.print(" quality: ");
    Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4);
      Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4);
      Serial.println(GPS.lon);

      Serial.print("Speed (knots): ");
      Serial.println(GPS.speed);
      Serial.print("Angle: ");
      Serial.println(GPS.angle);
      Serial.print("Altitude: ");
      Serial.println(GPS.altitude);
      Serial.print("Satellites: ");
      Serial.println((int)GPS.satellites);
      Serial.print("Antenna status: ");
      Serial.println((int)GPS.antenna);
    }
  }
}
void getLapsCounter() {
  int reading = digitalRead(buttonPin);  // Read the current state of the button
                                         // If the button state has been stable for the debounce delay, consider it a valid press
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != lastButtonState) {
      lastDebounceTime = millis();  // Update the debounce time
      if (reading == LOW && lastButtonState == HIGH) {
        totalLaps++;  // Increment the press count
        Serial.print("Button pressed ");
        Serial.print(totalLaps);
        Serial.println(" times");
      }
      // Save the current button state as the last state
      lastButtonState = reading;
    }
  }
}
// void getLapsGPS() {
//   char c = GPS.read();
//   // Read button press to set start location
//   if (GPS.fix) {
//     if (digitalRead(buttonPin) == LOW && !startLocationSet) {
//       startLat = GPS.latitudeDegrees;
//       startLng = GPS.longitudeDegrees;
//       startLocationSet = true;
//       Serial.println("Start location set.");
//     }
//   }
//   // Only process if start location is set
//   if (startLocationSet) {
//     Serial.print("Latitude= ");
//     Serial.print(GPS.latitudeDegrees, 6);
//     Serial.print(" Longitude= ");
//     Serial.println(GPS.longitudeDegrees, 6);

//     // Check if we passed the start location
//     if (!lapCounted && isLocationWithinThreshold(GPS.latitudeDegrees, GPS.longitudeDegrees, startLat, startLng, locationThreshold)) {
//       numberOfLaps++;
//       lapCounted = true;
//       Serial.print("Current Lap: ");
//       Serial.println(numberOfLaps);
//     }
//     // Reset lapCounted flag when we move away from the start location
//     else if (lapCounted && !isLocationWithinThreshold(GPS.latitudeDegrees, GPS.longitudeDegrees, startLat, startLng, locationThreshold)) {
//       lapCounted = false;
//     }
//   }
// }
// Function to calculate the distance between two lat/lng coordinates
//float calculateDistance(float lat1, float lng1, float lat2, float lng2) {
//  const float R = 6371000;  // Radius of the Earth in meters
//  float dLat = (lat2 - lat1) * M_PI / 180.0;
//  float dLng = (lng2 - lng1) * M_PI / 180.0;
//  lat1 = lat1 * M_PI / 180.0;
//  lat2 = lat2 * M_PI / 180.0;
//
//  float a = sin(dLat / 2) * sin(dLat / 2) + sin(dLng / 2) * sin(dLng / 2) * cos(lat1) * cos(lat2);
//  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
//  return R * c;  // Distance in meters
//}
//
//// Function to check if two locations are within a certain threshold
//bool isLocationWithinThreshold(float lat1, float lng1, float lat2, float lng2, float threshold) {
//  float distance = calculateDistance(lat1, lng1, lat2, lng2);
//  return distance < threshold;
//}
