#include "current_sensor.h"

#define ALPHA 0.01 // Smoothing factor (0-1)

float filteredCurrent = 0;

float readCurrentSensor(int pin) {
  int sensorValue = analogRead(pin);
  float voltage = sensorValue * (5000 / 1023.0);
  float instantCurrent = abs((voltage - 2500) / 100) - 1.15;

  if (instantCurrent < 0) {
    instantCurrent = 0;
  }
  
  filteredCurrent = ALPHA * instantCurrent + (1 - ALPHA) * filteredCurrent;
  
  return filteredCurrent;
}

bool isCurrentExceeded() {
  float leftCurrent = readCurrentSensor(LEFT_CURRENT_SENSOR_PIN);
  float rightCurrent = readCurrentSensor(RIGHT_CURRENT_SENSOR_PIN);

  // Serial.print(">Left current: ");
  // Serial.println(leftCurrent);
  // Serial.print(">Right current: ");
  // Serial.println(rightCurrent);

  return (leftCurrent > CURRENT_LIMIT) || (rightCurrent > CURRENT_LIMIT);
}