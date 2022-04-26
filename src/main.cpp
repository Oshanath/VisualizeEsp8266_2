#include "visualize_sensors.h"
#include "LedControl.h"
#include <ESP8266WiFi.h>

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

WiFiServer server(80);
std::string header;
unsigned long currentTime = millis();
unsigned long previousTime = 0;
const long timeoutTime = 2000;

// Set your Static IP address
IPAddress local_IP(192, 168, 1, 184);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

void setup() {
  Serial.begin(9600);
  while(!Serial){}
  sensorsInit();
  scanner.Init();
  ledInit();
  down = getAccelDownVector();
  gravityCorrection = Eigen::Quaternion<float>::FromTwoVectors(down, Eigen::Vector3f(0, 0, 1));
  while(!Serial);
  
  pinMode(D4, OUTPUT);

  digitalWrite(D4, HIGH);
  Serial.println("Starting mag cal");
  // magCalData = getMagCalData(gravityCorrection);
  magCalData = MagCalData{
        .hardIron = Eigen::Vector3f(3339.50, 41.50, -1058.50),
        .softIron = Eigen::Quaternion<float>(0.00, -0.54, 0.07,0.84),
        .scale = 0.62
  };
  printMagCalDataCode(magCalData);
  digitalWrite(D4, LOW);

  const char* ssid = "DESKTOP-L2QLGKV 4322";
  const char* password = "11111111";

  // Configures static IP address
  // if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
  //   Serial.println("STA Failed to configure");
  // }

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
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

  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            //Send data
            client.println("Data dataa");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }

}