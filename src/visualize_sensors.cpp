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

MagCalData getMagCalData(Eigen::Quaternion<float>& gravityCorrection){

  const int dataPoints = 1000;

  Eigen::Vector3f* data = new Eigen::Vector3f[dataPoints];

  float maxX, maxY, maxZ, minX, minY, minZ;

  for(int i = 0; i < dataPoints; i++){
    Eigen::Vector3i magnetometerRaw;
    getMagnetometerRaw(magnetometerRaw);

    data[i] << magnetometerRaw.x(), magnetometerRaw.y(), magnetometerRaw.z();

    if(i == 0){
      maxX = data[0].x();
      maxY = data[0].y();
      maxZ = data[0].z();
      minX = data[0].x();
      minY = data[0].y();
      minZ = data[0].z();
    }
    else{
      if(data[i].x() > maxX) maxX = data[i].x();
      else if(data[i].x() < minX) minX = data[i].x();

      if(data[i].y() > maxY) maxY = data[i].y();
      else if(data[i].y() < minY) minY = data[i].y();

      if(data[i].z() > maxZ) maxZ = data[i].z();
      else if(data[i].z() < minZ) minZ = data[i].z();
    }

    printVector3f(data[i]);
    delay(50);
  }

  auto hardIron = Eigen::Vector3f((maxX + minX) / 2, (maxY + minY) / 2, (maxZ + minZ) / 2);

  for(int i = 0; i < dataPoints; i++){
    data[i] = data[i] - hardIron;
    data[i] = gravityCorrection * data[i];
  }

  Eigen::Vector3f* majorPoint = &data[0];
  Eigen::Vector3f* minorPoint = &data[0];

  for(int i = 1; i < dataPoints; i++){
    if(getSquaredMagnitude(data[i]) > getSquaredMagnitude(*majorPoint)){
      majorPoint = &data[i];
    }
    else if(getSquaredMagnitude(data[i]) < getSquaredMagnitude(*minorPoint)){
      minorPoint = &data[i];
    }
  }

  Eigen::Quaternion<float> softIron = Eigen::Quaternion<float>::FromTwoVectors(*majorPoint, Eigen::Vector3f(1, 0, 0));
  float scale = std::sqrt(getSquaredMagnitude(*minorPoint) / getSquaredMagnitude(*majorPoint));

  for(int i = 0; i < dataPoints; i++){
    data[i] = softIron * data[i];
    data[i] = Eigen::Vector3f(data[i].x() * scale, data[i].y(), data[i].z());
    data[i] = softIron.inverse() * data[i];
  }

  delete[] data;

  return MagCalData{
    .hardIron = hardIron,
    .softIron = softIron,
    .scale = scale
  };

}

Eigen::Vector3f getCorrectMag(Eigen::Vector3f& rawMag, MagCalData& magCalData){

  Eigen::Vector3f v = rawMag - magCalData.hardIron;
  v = magCalData.softIron * v;
  v = Eigen::Vector3f(v.x() * magCalData.scale, v.y(), v.z());
  v = magCalData.softIron.inverse() * v;
  return v;
  
}

float getCompassHeading(Eigen::Vector3f& mag){

  Eigen::Vector3f north = ((Eigen::Vector3f(0, 0, 1).cross(-mag)).cross(Eigen::Vector3f(0, 0, 1)));
  float angle = std::atan(north.y() / north.x()) * 360 / (2 * PI);

  float dot = -mag.dot(Eigen::Vector3f(1, 0, 0));

  if(dot > 0 && angle < 0){
    angle += 360;
  }
  else if(dot < 0){
    angle += 180;
  }

  return angle;
}

void printMagCalDataCode(MagCalData& data){
  Serial.println("magCalData = MagCalData{");
  Serial.print("\t.hardIron = Eigen::Vector3f(");
  Serial.print(data.hardIron.x());
  Serial.print(", ");
  Serial.print(data.hardIron.y());
  Serial.print(", ");
  Serial.print(data.hardIron.z());
  Serial.println("),");

  Serial.print("\t.softIron = Eigen::Quaternion<float>(");
  Serial.print(data.softIron.x());
  Serial.print(", ");
  Serial.print(data.softIron.y());
  Serial.print(", ");
  Serial.print(data.softIron.z());
  Serial.print(",");
  Serial.print(data.softIron.w());
  Serial.println("),");

  Serial.print("\t.scale = ");
  Serial.println(data.scale);
  Serial.println("};");

}
