const int HA_PIN = 2;
//const int HB_PIN = 3;
//const int HC_PIN = 4;
const unsigned long DELAY_TIME = 1000; // Delay time in milliseconds

volatile unsigned int pulseCount = 0;
unsigned long prevTime = 0;
unsigned int rpm = 0;

void pulseCounter() {
  Serial.print("I'm in interrupt: ");
  Serial.println(pulseCount);
  pulseCount++;
}

void setup() {
  pinMode(HA_PIN, INPUT);
  Serial.begin(9600);
  //pinMode(HB_PIN, INPUT_PULLUP);
  //pinMode(HC_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(HA_PIN), pulseCounter, RISING);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - prevTime >= DELAY_TIME) {
    detachInterrupt(digitalPinToInterrupt(HA_PIN));
    rpm = (pulseCount * 60) / (DELAY_TIME / 1000);
    pulseCount = 0;
    prevTime = currentTime;

    // Print the RPM to the serial monitor
    Serial.print("RPM: ");
    Serial.println(rpm);

    attachInterrupt(digitalPinToInterrupt(HA_PIN), pulseCounter, RISING);
  }
}
