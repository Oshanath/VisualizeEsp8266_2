#include "visualize_sensors.h"
#include "LedControl.h"
#include <ESP8266WiFi.h>
#include <json.hpp>
#include <sstream>
#include "joystick.h"
#include "RGBLed.h"
#include "arduino-timer.h"
#include "navigator.h"

I2CScanner scanner;
Navigator navigator;

const char* ssid = "DESKTOP-L2QLGKV 4322";
const char* password = "11111111";
// const char* host = "192.168.1.8";
const char* host = "192.168.8.175";
const uint16_t port = 17;
WiFiClient client;
RGBLed rgbLed(D6, D7, D8);
auto timer = timer_create_default();

bool printPosition(void* a){
  printVector3f(navigator.pos);
  return true;
}

unsigned long start;
unsigned long end;

void connectToWifi(){
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
}

void setup() {
  Serial.begin(9600);
  while(!Serial);

  rgbLed.setColor(RED);
  sensorsInit();
  rgbLed.setColor(0, 0, 0);

  scanner.Init();
  navigator.down = getAccelDownVector(); 
  navigator.gravityCorrection = Eigen::Quaternion<float>::FromTwoVectors(navigator.down, Eigen::Vector3f(0, 0, 1));

  Serial.println("Starting mag cal");

  // rgbLed.setColor(BLUE);
  // navigator.magCalData = getMagCalData(navigator.gravityCorrection);
  // rgbLed.setColor(0, 0, 0);

  navigator.magCalData = MagCalData{
        .hardIron = Eigen::Vector3f(1210.00, -334.00, 646.00),
        .softIron = Eigen::Quaternion<float>(0.00, 0.32, -0.42,0.85),
        .scale = 0.61
  };

  navigator.initialAccel = navigator.down.norm();
  navigator.setInitialMag();
  
  Eigen::Vector3f accel;
  Eigen::Vector3f gyro;
  int temp;

  getAccelGyroTemp(accel, gyro, temp);
  navigator.initialGyro = gyro;

  printMagCalDataCode(navigator.magCalData);

  pinMode(D5, INPUT_PULLUP);

  // connectToWifi();

  timer.every(200, printPosition);

  start = millis();

}

float epsilon = 0.01f;

void loop() {
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

  if(digitalRead(D5) == HIGH){
    Serial.println("off");
    rgbLed.setColor(0, 0, 0);
  }
  else if(digitalRead(D5) == LOW){
    Serial.println("on");
    rgbLed.setColor(PURPLE);
  } 

}