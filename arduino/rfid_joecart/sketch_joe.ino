#include "user_config.h" // It must be located above ros2arduino.h.
#include <ros2arduino.h>
#include <inttypes.h>  // Force include here
#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 10
#define RST_PIN 9
#define BUTTON_PIN 4

MFRC522 mfrc522(SS_PIN, RST_PIN);

void setup() {
  Serial.begin(9600);
  SPI.begin();
  mfrc522.PCD_Init();
  pinMode(BUTTON_PIN,INPUT); 

  Serial.println("Place your card near the reader...");
}

void loop() {
  int Button_State = digitalRead(BUTTON_PIN);
  //delay(500);
  //Serial.print(Button_State);
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return;
  }
  if (!mfrc522.PICC_ReadCardSerial()) {
    return;
  }
  
  String content = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    if (mfrc522.uid.uidByte[i] < 0x10) {
      content += "0"; // Pad with zero if necessary
    }
    content += String(mfrc522.uid.uidByte[i], HEX); // Add each byte's hex value to the string
  }
  
  if (Button_State == LOW) {
    //Serial.print(Button_State);
    Serial.print("ADD: ");
    Serial.println(content); // Print the complete UID as a hex string with ADD prefix
  }
  else if (Button_State == HIGH) {
    //Serial.print(Button_State);
    Serial.print("REMOVE: ");
    Serial.println(content); // Print the complete UID as a hex string with REMOVE prefix
  }

  delay(500);
}
