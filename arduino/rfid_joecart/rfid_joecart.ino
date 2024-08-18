#include <inttypes.h>  // Force include here
#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 10
#define RST_PIN 9
#define BUTTON_PIN 4

MFRC522 mfrc522(SS_PIN, RST_PIN);

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Changed to INPUT_PULLUP

  Serial.begin(9600);
  while (!Serial);  // Wait for serial port to connect
  SPI.begin();
  mfrc522.PCD_Init();
  
  // Check if MFRC522 is initialized correctly
  if (!mfrc522.PCD_PerformSelfTest()) {
    Serial.println("MFRC522 initialization failed. Check wiring.");
    while (1);  // Halt if initialization failed
  }
  
  delay(4);
  mfrc522.PCD_DumpVersionToSerial();

  Serial.println("Place your card near the reader...");
}

void loop() {
  int Button_State = digitalRead(BUTTON_PIN);
  // Serial.print("Button State: ");
  // Serial.println(Button_State);

  while (!mfrc522.PCD_PerformSelfTest()) {
    Serial.println("ERROR");
    delay(500);  // Halt if initialization failed
  }

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

  delay(1500);
}
