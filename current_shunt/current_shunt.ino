// Define the analog pin
const int analogPin = A1;

// Calibration constants
const float slope = 0.273; // Slope from calibration formula
const float intercept = -1.384; // Intercept from calibration formula

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Read the analog value from pin A1
  int analogValue = analogRead(analogPin);
  
  // Convert the analog value to current using the calibration formula
  float current = slope * analogValue + intercept;
  
  // Print the analog value and the corresponding current to the Serial Monitor
  Serial.print("Analog Value: ");
  Serial.print(analogValue);
  Serial.print(" -> Current (A): ");
  Serial.println(current);
  
  // Delay before the next reading
  delay(1000); // 1 second delay
}
