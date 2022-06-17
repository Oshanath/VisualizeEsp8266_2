#include "helpers.h"

void lcdPrintSteps(LiquidCrystal_I2C lcd, Navigator& navigator, Map& currentMap) {
  lcd.setCursor(1, 0);
  lcd.print(navigator.steps);

  lcd.setCursor(0, 1);
  lcd.print(navigator.pos.x());

  lcd.setCursor(6, 1);
  lcd.print(navigator.pos.y());

  lcd.setCursor(4, 0);
  lcd.print(currentMap.name.c_str());
  lcd.print("-");
  lcd.print(currentMap.activeNode);
}

void printSPT(std::vector<int>& spt){

  for(int i = 0; i < spt.size(); i++){
    Serial.print(spt[i]);
    Serial.print(" -> ");
    Serial.println(i);
  }

}
