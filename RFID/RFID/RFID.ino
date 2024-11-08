#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 53

MFRC522 mfrc522(SS_PIN, -1); // Create MFRC522 instance without using RST_PIN.

bool flag = 0; // Initial flag state

void setup() {
  Serial.begin(9600); // Initialize serial communications
  SPI.begin(); // Initialize SPI bus
  mfrc522.PCD_Init(); // Initialize MFRC522
  Serial.println("Scan your RFID card");
}

void loop() {
  // If flag is 0, block the while loop
  while (flag == 0) {
    Serial.println("Flag is 0, blocking the loop...");
    delay(1000); // Add some delay to avoid flooding the serial output

    // Check for new cards
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      flag = 1; // Raise the flag when a card is detected
      Serial.println("Card detected!");

      // Print the UID of the card
      Serial.print("Card UID: ");
      for (byte i = 0; i < mfrc522.uid.size; i++) {
        Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
        Serial.print(mfrc522.uid.uidByte[i], HEX);
      }
      Serial.println();
    }
  }

  // Code to be executed when the flag is 1
  // (This part of the code will keep running once the flag is raised)
  Serial.println("Flag is 1, loop is unblocked!");
  delay(1000); // Add some delay to avoid flooding the serial output
}
