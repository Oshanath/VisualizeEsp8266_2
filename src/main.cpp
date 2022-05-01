#include "visualize_sensors.h"
#include "LedControl.h"
#include <ESP8266WiFi.h>
#include <json.hpp>
#include <sstream>

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


void setup() {
  Serial.begin(9600);
  while(!Serial){}
  sensorsInit();
  scanner.Init();
  ledInit();
  down = getAccelDownVector();
  gravityCorrection = Eigen::Quaternion<float>::FromTwoVectors(down, Eigen::Vector3f(0, 0, 1));
  
  pinMode(D4, OUTPUT);
  pinMode(D4, INPUT);

  digitalWrite(D4, HIGH);
  Serial.println("Starting mag cal");
  // magCalData = getMagCalData(gravityCorrection);
  // magCalData = MagCalData{
  //       .hardIron = Eigen::Vector3f(3339.50, 41.50, -1058.50),
  //       .softIron = Eigen::Quaternion<float>(0.00, -0.54, 0.07,0.84),
  //       .scale = 0.62
  // };
  magCalData = MagCalData{
        .hardIron = Eigen::Vector3f(2512.50, -979.50, 274.00),
        .softIron = Eigen::Quaternion<float>(0.00, -0.38, 0.35,0.86),
        .scale = 0.54
  };
  printMagCalDataCode(magCalData);
  digitalWrite(D4, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  // Serial.println("");
  Serial.println("WiFi connected.");
  // Serial.print("IP address: ");
  // Serial.println(WiFi.localIP());

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
}

void loop() {
  // scanner.Scan();

  Eigen::Vector3i magnetometerRawi;
  getMagnetometerRaw(magnetometerRawi);
  Eigen::Vector3f magnetometerRaw(magnetometerRawi.x(), magnetometerRawi.y(), magnetometerRawi.z());

  Eigen::Vector3f accel;
  Eigen::Vector3f gyro;
  int temp;
  getAccelGyroTemp(accel, gyro, temp);

  accel = gravityCorrection * accel;

  Eigen::Vector3f mag = getCorrectMag(magnetometerRaw, magCalData);
  
  float angle = getCompassHeading(mag);
  // Serial.println(angle);
  ledDisplayValue(angle);

  if(!digitalRead(D3)){
    // This will send a string to the server
    Serial.println("Sending data via wifi.");
    if (client.connected()) { 
      json::JSON obj;
      obj["x"] = mag.x();
      obj["y"] = mag.y();
      obj["z"] = mag.z();

      std::stringstream ss; 
      ss << obj;
      client.println(ss.str().c_str());
    }
    delay(1000);

  }
}