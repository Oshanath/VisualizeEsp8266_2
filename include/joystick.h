#pragma once

#include "Arduino.h"

class Joystick{

    uint8_t clickPin;
    uint8_t analogPin;
    float threshold;

public:
    Joystick(uint8_t clickPin, uint8_t analogPin, float threshold = 50)
     :  clickPin(clickPin), analogPin(analogPin), threshold(threshold){

        pinMode(clickPin, INPUT);
        pinMode(clickPin, INPUT_PULLUP);

        pinMode(analogPin, INPUT);
    }

    bool isClicked(){
        return !digitalRead(clickPin);
    }

    bool isLeft(){
        return (analogRead(analogPin) < threshold);
    }

    bool isRight(){
        return (analogRead(analogPin) > 1024 - threshold);
    }

};