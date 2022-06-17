#include <string>
#include "helpers.h"
#include "Map.h"
#include "arduino-timer.h"
#include "maps.h"
#include "navigator.h"
#include "server_vis.h"

#define mtr_input D5

int numberOfMaps = -1;
std::vector<int> nodesPerMap;

std::string jsonString = "{\"maps\": 2,\"data\": [{\"id\": 1,\"name\":\"boarding\",\"nodes\": 6},{\"id\": 0,\"name\":\"embedded lab\",\"nodes\": 7}]}";

// Interface -------------------------------------------------------------------------------------

unsigned int mainMenu = 0;      // Select Maps Option,Download Maps
unsigned int subMenu = 0;       // List of maps
unsigned int secondSubMenu = 0; // List of Nodes

unsigned int mainState = 0; // Iteration state 0:Main Menu , 1: Maps, 2:Nodes
unsigned int map_no = 0;    // map number for naming
unsigned int node_no = 0;   // node Number for naming

unsigned int idleState = 1;

int timerCode;

#define btn_select D4
#define btn_cycForward D7
#define btn_cycBackward D6
#define btn_back D3
#define stepButton btn_cycForward
// Interface -------------------------------------------------------------------------------------

unsigned long start;
unsigned long end;
long s;
ServerS server_vis;

I2CScanner scanner;
Navigator navigator;
Map currentMap = getEmbeddedLabMap();
float angleDifference = 0.0f;
auto timer = timer_create_default();
bool motorState = false;
bool flag = false;

bool motorVibrate(void *argument);
void receiveEvent(int howMany);
void sendMessage(std::string message);
void processUI();
void startNavigating(int node);
bool isIdle();
bool isMenu();
bool isNavigating();
void stopNavigation();
void initJson(std::string json);

void setup() {

  pinMode(btn_select, INPUT_PULLUP);
  pinMode(btn_cycForward, INPUT_PULLUP);
  pinMode(btn_cycBackward, INPUT_PULLUP);
  pinMode(btn_back, INPUT_PULLUP);


  Serial.begin(9600);
  while (!Serial)
    ;
  sensorsInit();

  pinMode(D5, OUTPUT);

  scanner.Init();

  // navigator.magCalData = getMagCalData();

  // navigator.magCalData =
  //     MagCalData{.hardIron = Eigen::Vector3f(1068.00, -245.00, 381.00),
  //                .softIron = Eigen::Quaternion<float>(0.00, -0.01, -0.88, 0.47),
  //                .scale = 0.72};

  navigator.magCalData = MagCalData{
        .hardIron = Eigen::Vector3f(854.50, -474.50, -4256.50),
        .softIron = Eigen::Quaternion<float>(0.00, -0.70, 0.10,0.70),
        .scale = 0.65
  };

  navigator.setInitialMag();
  printMagCalDataCode(navigator.magCalData);

  // server_vis.connectToWifi();

  start = millis();

  // printSPT(currentMap.currentShortestPathTree);

  sendMessage("init");
  while(numberOfMaps == -1)
    ;

  initJson(jsonString);

  while(numberOfMaps < 0)
    ;

  Serial.println(numberOfMaps);
  
  for(int i = 0; i < numberOfMaps; i++){
    Serial.print(nodesPerMap[i]);
  }
}

void loop() {
  // Serial.println(angleDifference);
  // if (!currentMap.started) {
  //   if (!digitalRead(stepButton)) {
  //     navigator.startMap();
  //     currentMap.started = true;
  //     navigator.initialMag = navigator.mag;
  //     delay(500);
  //   }
  // }
  // sendMessage("test");
  // delay(1000);

  printVector3f(navigator.mag);
  // Serial.println(angleDifference);

// scanner.Scan();
  timer.tick();

  navigator.mag = navigator.getMagVector();
  navigator.angle = getCompassHeading(navigator.mag);

  if(isNavigating() or isIdle()){
    // Read steps
    if (!digitalRead(stepButton) and !flag) {
    navigator.step(currentMap, stopNavigation);
    flag = true;
    Serial.print("steps = ");
    Serial.print(navigator.steps);
    Serial.print(", active node = ");
    Serial.print(currentMap.activeNode);
    Serial.print(", angle = ");
    Serial.println(angleDifference);
    } else if (digitalRead(stepButton) and flag) {
      flag = false;
    }
  }

  angleDifference = navigator.getAngleDifference(currentMap);


  // wait 400ms after pressing a button
  if(millis() - start > 400){
    processUI();
  }
}

bool motorVibrate(void *argument) {
  int angle = angleDifference;
  int cal_delay = 500 + (47.0 / 18.0) * (angle - 180);

  if (motorState) {
    digitalWrite(mtr_input, LOW);
    motorState = false;
  } else {
    // if (angle < 10) {
    //   digitalWrite(mtr_input, LOW);
    // } else {
    //   digitalWrite(mtr_input, HIGH);
    // }

    digitalWrite(mtr_input, HIGH);

    motorState = true;
  }
  timerCode = timer.in(cal_delay, motorVibrate);

  return true;
}

void initJson(std::string s){
  json::JSON obj = json::JSON::Load(s);
  numberOfMaps = obj["maps"].ToInt();
  nodesPerMap.reserve(obj["data"].size());

  for(int i = 0; i < obj["data"].size(); i++){
    int index = obj["data"][i]["id"].ToInt();
    nodesPerMap[index] = obj["data"][i]["nodes"].ToInt();
  }
}

void sendMessage(std::string message) {
  Wire.beginTransmission(9);
  Wire.write(message.c_str());
  Wire.endTransmission();
}

void processUI(){
  unsigned int cycForward_val = digitalRead(btn_cycForward);
  unsigned int cycBackward_val = digitalRead(btn_cycBackward);
  unsigned int select_val = digitalRead(btn_select);
  unsigned int back_val = digitalRead(btn_back);

  if(!cycForward_val or !cycBackward_val or !select_val or !back_val){
    start = millis();
  }

  // Serial.print(cycForward_val);
  // Serial.print(cycBackward_val);
  // Serial.print(select_val);
  // Serial.println(back_val);

  // Serial.print("main state = ");
  // Serial.print(mainState);
  // Serial.print(" , idle state = ");
  // Serial.println(idleState);

  std::string def;
  std::string fileName;
  std::string temp = "";
  bool messagePending = false;

  if (!idleState && cycForward_val == 0 && !isNavigating())
  {
    Serial.println("forward");
      messagePending = true;
      if (mainState == 0)
      {
          if (mainMenu < 1)
          {
              mainMenu += 1;
          }
          else
          {
              mainMenu = 0;
          }
          def = "main";
          fileName = std::to_string(mainMenu);
      }
      else if (mainState == 1)
      {
          subMenu += 1;
          if(subMenu == numberOfMaps) subMenu = 0;
          def = "map";
          fileName = std::to_string(subMenu);
      }
      else
      {
          secondSubMenu += 1;
          if(secondSubMenu == nodesPerMap[subMenu]) secondSubMenu = 0;
          def = "node" + std::to_string(subMenu);
          fileName = std::to_string(secondSubMenu);
      }
  }

  else if (!idleState && cycBackward_val == 0 && !isNavigating())
  {
    messagePending = true;
      if (mainState == 0)
      {
          if (mainMenu > 0)
          {
              mainMenu -= 1;
          }
          else
          {
              mainMenu = 1;
          }
          def = "main";
          fileName = std::to_string(mainMenu);
      }
      else if (mainState == 1)
      {
          if (subMenu > 0)
          {
              subMenu -= 1;
          }
          def = "map";
          fileName = std::to_string(subMenu);
      }
      else
      {
          if (secondSubMenu > 0)
          {
              secondSubMenu -= 1;
          }
          def = "node" + std::to_string(subMenu);
          fileName = std::to_string(secondSubMenu);
      }
  }

  else if (!idleState && select_val == 0)
  {
    messagePending = true;
      if (mainMenu == 1)
      {
          def = "Downloading";
          fileName = "maps";
          mainMenu = 0;
          mainState = 0;          
      }
      else if (mainState == 0)
      {
          mainState = 1;
          subMenu = 0;
          def = "map";
          fileName = std::to_string(subMenu);
      }
      else if (mainState == 1)
      {
          mainState = 2;
          map_no = subMenu;
          def = "node" + std::to_string(map_no);
          fileName = std::to_string(secondSubMenu);
      }
      else
      {
          node_no = secondSubMenu;
          mainState++;
          startNavigating(node_no);

      }
      
      if(def + fileName == "Downloadingmaps"){
        Serial.print("Playing Audio:");
        Serial.println("main0");
        sendMessage("main0");
      }
  }

  else if (!idleState && back_val == 0)
  {
    messagePending = true;
      std::string temp = "";

      if (mainState == 0)
      {
          idleState = 1;
      }
      else if (mainState == 1)
      {
          def = "main";
          mainState = 0;
          mainMenu = 0;
          fileName = std::to_string(mainMenu);
      }
      else if(mainState == 2)
      {
          def = "map";
          mainState = 1;
          subMenu = 0;
          secondSubMenu = 0;
          fileName = std::to_string(subMenu);
      }
      else{
        def = "node";
        mainState = 2;
        fileName = std::to_string(subMenu);
        fileName += std::to_string(secondSubMenu);
        stopNavigation();
      }
  }

  else if (idleState && select_val == 0)
  {
    messagePending = true;
      idleState = 0;
      mainState = 0;
      mainMenu = 0;
      def = "main";
      fileName = std::to_string(mainMenu);

  }
  
  if(messagePending){
    Serial.print("Playing Audio:");
    std::string message = (def + temp + fileName).c_str();
    Serial.println(message.c_str());
    messagePending = false;
    // send message
    sendMessage(message);
  }
}

void startNavigating(int node){
  currentMap.startNavigating(node);
  timerCode = timer.in(500, motorVibrate);
}

void stopNavigation(){
  timer.cancel();
  digitalWrite(mtr_input, LOW);
}

bool isIdle(){
  return idleState;
}

bool isMenu(){
  return !isIdle() and (mainState == 0 or mainState == 1 or mainState == 2);
}

bool isNavigating(){
  return mainState == 3;
}
