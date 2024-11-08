//This program is for the Arduino. Before using this program on the please include the library
// link:https://github.com/nRF24/RF24

#include<SPI.h>
#include<RF24.h>


// ce, csn pins
RF24 radio(9, 8) ;

void setup(void) {
  radio.begin() ;
  radio.setPALevel(RF24_PA_MAX) ;
  radio.setChannel(0x76) ;
  radio.openWritingPipe(0xF0F0F0F0E1LL) ;
  radio.enableDynamicPayloads() ;
  radio.powerUp() ;

  Serial.begin(115200);
}

void loop(void) {

  if (Serial.available()) {

    uint8_t data[] = {Serial.read()};
    // Dont put this on the stack:
    uint8_t buf[32];


    radio.write(&data, sizeof(data));
  }

  delay(20) ;

}
