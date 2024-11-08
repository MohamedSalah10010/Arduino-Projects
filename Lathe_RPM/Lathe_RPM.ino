

// The amount of time (in milliseconds) between tests
#define TEST_DELAY   999


volatile int counter = 0;
int RPM,kmh;

void setup() {

  Serial.begin(9600);
  
  attachInterrupt(0,count,RISING);

}
void loop() {
  delay(1000);  //Delay almost 1 second.  
 
  
  RPM = (counter * 60);
  kmh = RPM * 3.14 * 0.0006 * 60; // km/h = rpm * diameter in km * PI * 60
  Serial.print("RPM: "); // Counter * 60 seconds.
  Serial.print(RPM);
  Serial.print("          KMH: "); // Counter * 60 seconds.
  Serial.println(kmh);
  counter = 0;
  
 
  
}
 
void count()
{
 Serial.print("I'm in intrrupt my counter now is:");
 Serial.println(counter) ;
 counter++;
}
