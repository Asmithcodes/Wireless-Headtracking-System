#ifndef SENSORS_H
#define SENSORS_H

#include <MPU9250_asukiaaa.h>
#include <MPU6050.h>
#include "config.h"

// External sensor objects
extern MPU9250_asukiaaa mySensorMPU6500;
extern MPU6050 mySensorMPU6050;

// External state variables
extern float gyroZOffset;
extern float gyroZBuffer[FILTER_SIZE];
extern int filterIndex;
extern unsigned long lastTime;
extern bool mpu6500Ok;
extern bool mpu6050Ok;
extern float yawGyro;

// Function declarations
bool initializeMPU6500();
bool initializeMPU6050();
void calibrateGyroscope();
float readYawFromGyro();
float readPitchFromAccel();
void resetYaw();

#endif
