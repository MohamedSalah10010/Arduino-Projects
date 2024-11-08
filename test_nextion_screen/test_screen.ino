int batteryVoltage=1;

//====== voltage \ shunt and speed=============
//============================= SOC and Range ==================================
#define voltagePin A0
const float R1 = 100000.0;
const float R2 = 10000.0;
const float maxVoltage = 54.5;
const float minVoltage = 3.4;
const float batteryCapacityWh = 80.0 * 52;
const float averageConsumptionWhPerKm = 70;
unsigned long lastSOCTime = 0;
int soc = 0;
int rangeAvailable = 0;
int batteryVoltage = 0;
float bias = 0.145;
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

void setup() {
  // put your setup code here, to run once:

  Serial3.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
 Serial3.print("GPS.volt.val=");
  Serial3.print(batteryVoltage++);
  Serial3.write(0xff);
  Serial3.write(0xff);
  Serial3.write(0xff);
}
