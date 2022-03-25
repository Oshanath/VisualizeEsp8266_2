#include "visualize_sensors.h"
#include "LedControl.h"

#define LED_Clock D5
#define LED_Chip_Select D8
#define LED_Data_IN D7

I2CScanner scanner;
LedControl lc=LedControl(LED_Data_IN, LED_Clock, LED_Chip_Select, 1);


void ledInit(){
  /*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  lc.setIntensity(0,8);
  /* and clear the display */
  lc.clearDisplay(0);
}

void ledDisplayValue(int value){
  int digits[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int index = 7;

  while(value != 0){
    digits[index] = value % 10;
    value = floor(value / 10);
    index--;
  }

  for(int i = 0; i < 8; i++){
    lc.setDigit(0, 7 - i, digits[i], false);
  }
}

Eigen::Vector3f down;
Eigen::Quaternion<float> gravityCorrection;

void setup() {
  Serial.begin(9600);
  while(!Serial){}
  sensorsInit();
  scanner.Init();
  ledInit();
  down = getAccelDownVector();
  gravityCorrection = Eigen::Quaternion<float>::FromTwoVectors(down, Eigen::Vector3f(0, 0, 1));

}

void loop() {
  // scanner.Scan();

  Eigen::Vector3i magnetometerRaw;
  getMagnetometerRaw(magnetometerRaw);

  Eigen::Vector3f accel;
  Eigen::Vector3f gyro;
  int temp;
  getAccelGyroTemp(accel, gyro, temp);

  accel = gravityCorrection * accel;

  Serial.print("x = ");
  Serial.print(accel.x());
  Serial.print(", y = ");
  Serial.print(accel.y());
  Serial.print(", z = ");
  Serial.print(accel.z());
  Serial.println();

  delay(500);
}