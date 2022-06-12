#include "visualize_sensors.h"
#include "LedControl.h"
#include <ESP8266WiFi.h>
#include <json.hpp>
#include <sstream>
#include "joystick.h"
#include "RGBLed.h"
#include "arduino-timer.h"
#include "navigator.h"
#include "Map.h"

#define stepButton D4

I2CScanner scanner;
Navigator navigator;

const char* ssid = "DESKTOP-GAC9QP0 6425";
const char* password = "11111111";
// const char* host = "10.10.17.7";
const char* host = "192.168.8.175";
const uint16_t port = 17;
WiFiClient client;
// RGBLed rgbLed(D0, D9, D10);
auto timer = timer_create_default();

void sendViaWifi(json::JSON& obj){
  if (client.connected()) { 
    std::stringstream ss; 
    ss << obj;
    client.println(ss.str().c_str());
  }
}

void sendViaWifi(json::JSON&& obj){
  if (client.connected()) { 
    std::stringstream ss; 
    ss << obj;
    client.println(ss.str().c_str());
  }
}

json::JSON getPosJson(){
  json::JSON obj;
  obj["type"] = "pos";
  obj["x"] = navigator.pos.x();
  obj["y"] = navigator.pos.y();
  obj["z"] = navigator.pos.z();
  return obj;
}

json::JSON getAngleJson(){
  json::JSON obj;
  obj["type"] = "angle";
  obj["angle"] = navigator.angle;
  return obj;
}

json::JSON getOrientation(){

  Eigen::Vector3f orientationVector = navigator.orientation * Eigen::Vector3f(1, 0, 0);

  // printVector3f(orientationVector);
  Serial.println(navigator.angle);

  json::JSON obj;
  obj["type"] = "pos";
  obj["x"] = navigator.steps;
  obj["y"] = 0;
  obj["z"] = 0;
  return obj;
}

bool printPosition(void* a){
  
  sendViaWifi(getOrientation());
  return true;
}

unsigned long start;
unsigned long end;

void connectToWifi(){
  // Connect to Wi-Fi network with SSID and password
  // rgbLed.setColor(GREEN);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("WiFi connected.");
  // rgbLed.setColor(0, 0, 0);

  // rgbLed.setColor(PURPLE);
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
  // rgbLed.setColor(0, 0, 0);
}
long s;

void setup() {
  Serial.begin(9600);
  SPI.begin();
  while(!Serial);

  // rgbLed.setColor(RED);
  sensorsInit();
  // rgbLed.setColor(0, 0, 0);

  scanner.Init();
  navigator.down = getAccelDownVector(); 
  navigator.gravityCorrection = Eigen::Quaternion<float>::FromTwoVectors(navigator.down, Eigen::Vector3f(0, 0, 1));

  Serial.println("Starting mag cal");

  // rgbLed.setColor(BLUE);
  // navigator.magCalData = getMagCalData(navigator.gravityCorrection);
  // rgbLed.setColor(0, 0, 0);

  navigator.magCalData = MagCalData{
        .hardIron = Eigen::Vector3f(1068.00, -245.00, 381.00),
        .softIron = Eigen::Quaternion<float>(0.00, -0.01, -0.88,0.47),
        .scale = 0.72
  };

  navigator.initialAccel = navigator.down.norm();
  navigator.setInitialMag();
  
  Eigen::Vector3f accel;
  Eigen::Vector3f gyro;
  int temp;

  getAccelGyroTemp(accel, gyro, temp);
  navigator.initialGyro = gyro;

  printMagCalDataCode(navigator.magCalData);

  pinMode(stepButton, INPUT_PULLUP);

  // connectToWifi();

  timer.every(500, printPosition);

  start = millis();
  s = millis();
}

float epsilon = 0.01f;
bool flag = false;

Map currentMap;

void loop() {

  if(!currentMap.started){
    Serial.println("Press step button to start map.");
    if(!digitalRead(stepButton)){
      navigator.startMap();
      currentMap.started = true;
      navigator.initialMag = navigator.mag;
      Serial.println("Map started");
      delay(500);
    }
  }

  // scanner.Scan();
  timer.tick();
  int temp;

  getAccelGyroTemp(navigator.accel, navigator.gyro, temp);

  navigator.setCorrectGyro();
  navigator.accel = navigator.gravityCorrection * navigator.accel;

  // correcting the accel axes
  navigator.accel(2) = -navigator.accel.z();
  auto q2 = Eigen::Quaternion<float>::FromTwoVectors(Eigen::Vector3f(0, 1, 0), Eigen::Vector3f(1, 0, 0));
  navigator.accel = q2 * navigator.accel;

  navigator.mag = navigator.getMagVector();
  
  navigator.angle = getCompassHeading(navigator.mag);

  navigator.calculateAccelNoGravity();

  end = millis();
  unsigned long delta = end - start;
  navigator.calculateOrientation(delta);
  navigator.calculatePosition(delta);
  start = end;

  if(!digitalRead(stepButton) and !flag){
    // rgbLed.setColor(YELLOW);
    navigator.step();
    flag = true;
  }
  else if(digitalRead(stepButton) and flag){
    // rgbLed.setColor(0, 0, 0);
    flag = false;
  }

}