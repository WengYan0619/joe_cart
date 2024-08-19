#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#define LEFT_CURRENT_SENSOR_PIN A0
#define RIGHT_CURRENT_SENSOR_PIN A1
#define CURRENT_LIMIT 1000 // Set this to your desired current limit in mA

float readCurrentSensor(int pin);
bool isCurrentExceeded();

#endif