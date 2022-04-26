#pragma once

#include <Wire.h>
#include <MechaQMC5883.h>
#include "I2CScanner.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoEigenDense.h>

void sensorsInit();

inline void printVector3f(Eigen::Vector3f& v){
  Serial.print(v.x());
  Serial.print(",");
  Serial.print(v.y());
  Serial.print(",");
  Serial.print(v.z());
  Serial.print("\n");
}

inline float getSquaredMagnitude(Eigen::Vector3f& v){
  return v.x() * v.x() + v.y() * v.y() + v.z() * v.z();
}

void mpu6050Init(mpu6050_accel_range_t accel_range, mpu6050_gyro_range_t gyro_range, mpu6050_bandwidth_t filter_bandwidth);
void getAccelGyroTemp(Eigen::Vector3f& accelRaw, Eigen::Vector3f& gyroRaw, int& t) ;   // in m/s^2, rad/s and Celcius
Eigen::Vector3f getAccelDownVector();

struct MagCalData{
  Eigen::Vector3f hardIron;
  Eigen::Quaternion<float> softIron;
  float scale;
};

void magnetometerInit();
void getMagnetometerRaw(Eigen::Vector3i& magnetometerRaw);
MagCalData getMagCalData(Eigen::Quaternion<float>& gravityCorrection);
Eigen::Vector3f getCorrectMag(Eigen::Vector3f& rawMag, MagCalData& magCalData);
float getCompassHeading(Eigen::Vector3f& mag);
void printMagCalDataCode(MagCalData& data);