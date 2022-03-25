#include "visualize_sensors.h"

Adafruit_MPU6050 mpu;
MechaQMC5883 qmc;

void sensorsInit(){
  Wire.begin();
  mpu6050Init(MPU6050_RANGE_8_G, MPU6050_RANGE_500_DEG, MPU6050_BAND_21_HZ);
  magnetometerInit();
}

// Accel and Gyro

void mpu6050Init(mpu6050_accel_range_t accel_range, mpu6050_gyro_range_t gyro_range, mpu6050_bandwidth_t filter_bandwidth){
  while (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    delay(100);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- numberOfReadings0 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void getAccelGyroTemp(Eigen::Vector3f& accelRaw, Eigen::Vector3f& gyroRaw, int& t){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accelRaw << a.acceleration.x, a.acceleration.y, a.acceleration.z;
  gyroRaw << a.gyro.x, a.gyro.y, a.gyro.z;
  t = temp.temperature;
}

Eigen::Vector3f getAccelDownVector(){

  // Get some readings and calculate the average while the device is resting on a flat surface

  int numberOfReadings = 100;

  float xSum = 0.0;
  float ySum = 0.0;
  float zSum = 0.0;

  for(int i = 0; i < numberOfReadings; i++){
    Eigen::Vector3f accel;
    Eigen::Vector3f gyro;
    int temp;

    getAccelGyroTemp(accel, gyro, temp);
    xSum += accel.x();
    ySum += accel.y();
    zSum += accel.z();
  }

  return Eigen::Vector3f(xSum / numberOfReadings, ySum / numberOfReadings, zSum / numberOfReadings);

}


// Magnet

void magnetometerInit(){
  qmc.init();
}
void getMagnetometerRaw(Eigen::Vector3i& magnetometerRaw){
  int x, y, z;
  qmc.read(&x, &y, &z);
  magnetometerRaw << x, y, z;
}
