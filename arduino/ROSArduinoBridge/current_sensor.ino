#include "current_sensor.h"

float readCurrentSensor(int pin) {
  int sensorValue = analogRead(pin);
  float voltage = sensorValue * (5.0 / 1023.0);
  float current = voltage * 1000 / 100; // Assuming ACS712 30A version, adjust if using a different sensor

  return current;
}

bool isCurrentExceeded() {
  float leftCurrent = readCurrentSensor(LEFT_CURRENT_SENSOR_PIN);
  float rightCurrent = readCurrentSensor(RIGHT_CURRENT_SENSOR_PIN);

  Serial.print(">Left current: ");
  Serial.println(leftCurrent);
  Serial.print(">Right current: ");
  Serial.println(rightCurrent);

  return (leftCurrent > CURRENT_LIMIT) || (rightCurrent > CURRENT_LIMIT);
}