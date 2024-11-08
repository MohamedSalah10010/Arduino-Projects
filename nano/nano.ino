int analog_read;
void setup() {
  // put your setup code here, to run once:
Serial.begin (9600);

}

void loop() {
  // put your main code here, to run repeatedly:
analog_read= analogRead(A0);


Serial.print("sensor value:  ");
Serial.println(analog_read);
delay(1000);

}
