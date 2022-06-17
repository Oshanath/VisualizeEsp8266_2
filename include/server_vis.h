#ifndef ServerS_H
#define ServerS_H
// Include this in only one file

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <json.hpp>
#include <sstream>
#include <FS.h>

class ServerS{
public:
    WiFiClient client;
    const char* ssid = "DESKTOP-GAC9QP0 6425";
    const char* password = "11111111";
    const char* host = "10.10.17.7";
    // const char* host = "192.168.8.175";
    const uint16_t port = 8082;

    HTTPClient http;


    void connectToWifi();
    void sendJson(json::JSON& obj);
    void sendString(std::string s);
    std::string sendGetRequest(std::string endPoint);
    void getFile(std::string name);

};

void ServerS::connectToWifi(){
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("WiFi connected.");

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
}

void ServerS::sendJson(json::JSON& obj){
  if (client.connected()) { 
    std::stringstream ss; 
    ss << obj;
    client.println(ss.str().c_str());
  }
}

void ServerS::sendString(std::string s){
  if (client.connected()) { 
    client.println(s.c_str());
  }
}

std::string ServerS::sendGetRequest(std::string endPoint){

    String payload = "Error getting data";

    if(client.connected()){

        std::string serverName = "http://" + std::string(host) + ":8082" + endPoint;

        http.begin(client, serverName.c_str());
        int httpResponseCode = http.GET();

        if (httpResponseCode>0) {
            Serial.print("HTTP Response code: ");
            Serial.println(httpResponseCode);
            payload = http.getString();
        }
        else {
            Serial.print("Error code: ");
            Serial.println(httpResponseCode);
        }
        http.end();
    }

    return std::string(payload.c_str());

}

void ServerS::getFile(std::string name){

    bool result = SPIFFS.begin();
    File f = SPIFFS.open(name.c_str(), "w");

    if(f){
        Serial.println("getting payload");
        http.writeToStream(&f);
        Serial.println(f.size());
    }

}

#endif