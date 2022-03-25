#pragma once

#include <Wire.h>
#include <MechaQMC5883.h>
#include "I2CScanner.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoEigenDense.h>

void sensorsInit();

template <typename T>
void printVector3(T v){
  Serial.print(v.x());
  Serial.print(", y = ");
  Serial.print(v.y());
  Serial.print(", z = ");
  Serial.print(v.z());
  Serial.print(", w = ");
  Serial.print(v.w());
  Serial.println();
}

void mpu6050Init(mpu6050_accel_range_t accel_range, mpu6050_gyro_range_t gyro_range, mpu6050_bandwidth_t filter_bandwidth);
void getAccelGyroTemp(Eigen::Vector3f& accelRaw, Eigen::Vector3f& gyroRaw, int& t) ;   // in m/s^2, rad/s and Celcius
Eigen::Vector3f getAccelDownVector();

void magnetometerInit();
void getMagnetometerRaw(Eigen::Vector3i& magnetometerRaw);
