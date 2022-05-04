#include "visualize_sensors.h"
#include "LedControl.h"
#include <ESP8266WiFi.h>
#include <json.hpp>
#include <sstream>
#include "joystick.h"
#include "RGBLed.h"
#include "arduino-timer.h"

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
MagCalData magCalData;
const char* ssid = "DESKTOP-L2QLGKV 4322";
const char* password = "11111111";
// const char* ssid = "iPhone XR";
// const char* password = "wesapakaya";
// const char* host = "10.10.17.7";
const char* host = "192.168.1.8";
const uint16_t port = 17;
WiFiClient client;
Joystick joystick(D5, A0);
RGBLed rgbLed(D6, D7, D8);
auto timer = timer_create_default();

float angle = 0;

bool sendCompassAngle(void* a){
  Serial.println("Sending mag data via wifi.");
  if (client.connected()) { 
    json::JSON obj;
    obj["type"] = "compass";
    obj["angle"] = angle;

    std::stringstream ss; 
    ss << obj;
    client.println(ss.str().c_str());
  }

  return true;
}

void setup() {
  Serial.begin(9600);
  while(!Serial);

  rgbLed.setColor(RED);
  sensorsInit();
  rgbLed.setColor(0, 0, 0);

  scanner.Init();
  ledInit();
  down = getAccelDownVector();
  gravityCorrection = Eigen::Quaternion<float>::FromTwoVectors(down, Eigen::Vector3f(0, 0, 1));

  Serial.println("Starting mag cal");

  // rgbLed.setColor(BLUE);
  // magCalData = getMagCalData(gravityCorrection);
  // rgbLed.setColor(0, 0, 0);

  magCalData = MagCalData{
        .hardIron = Eigen::Vector3f(-545.00, -1005.00, -489.00),
        .softIron = Eigen::Quaternion<float>(0.00, 0.42, -0.47,0.77),
        .scale = 0.74
  };

  printMagCalDataCode(magCalData);
  digitalWrite(D4, LOW);

  // Connect to Wi-Fi network with SSID and password
  rgbLed.setColor(GREEN);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("WiFi connected.");
  rgbLed.setColor(0, 0, 0);

  rgbLed.setColor(PURPLE);
  Serial.print("connecting to ");
  Serial.print(host);
  Serial.print(':');
  Serial.println(port);

  // Use WiFiClient class to create TCP connections
  while (!client.connect(host, port)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("Connected");
  rgbLed.setColor(0, 0, 0);

  timer.every(500, sendCompassAngle);
}

void loop() {
  // scanner.Scan();
  timer.tick();
  rgbLed.setColor(0, 0, 0);
  Eigen::Vector3i magnetometerRawi;
  getMagnetometerRaw(magnetometerRawi);
  Eigen::Vector3f magnetometerRaw(magnetometerRawi.x(), magnetometerRawi.y(), magnetometerRawi.z());

  Eigen::Vector3f accel;
  Eigen::Vector3f gyro;
  int temp;
  getAccelGyroTemp(accel, gyro, temp);

  accel = gravityCorrection * accel;

  Eigen::Vector3f mag = getCorrectMag(magnetometerRaw, magCalData);
  
  angle = getCompassHeading(mag);
  // Serial.println(angle);
  // ledDisplayValue(angle);

  if(joystick.isClicked()){
    // This will send a string to the server
    Serial.println("Sending mag data via wifi.");
    if (client.connected()) { 
      json::JSON obj;
      obj["type"] = "mag";
      obj["x"] = mag.x();
      obj["y"] = mag.y();
      obj["z"] = mag.z();

      std::stringstream ss; 
      ss << obj;
      client.println(ss.str().c_str());
    }

  }
}