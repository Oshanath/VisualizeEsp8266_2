#pragma once

#include "Arduino.h"

enum Color{
    RED, GREEN, BLUE, PURPLE, YELLOW, ORANGE, PINK, LIGHT_BLUE, BROWN
};

class RGBLed{
    uint8_t r;
    uint8_t g;
    uint8_t b;

    uint8_t rPin;
    uint8_t gPin;
    uint8_t bPin;

    float reduceFactor = 0.1f;

    void applyColor(){
        analogWrite(rPin, r);
        analogWrite(gPin, g);
        analogWrite(bPin, b);
    }

public:
    RGBLed(uint8_t rPin, uint8_t gPin, uint8_t bPin) : rPin(rPin), gPin(gPin), bPin(bPin), r(0), g(0), b(0){
        pinMode(rPin, OUTPUT);
        pinMode(gPin, OUTPUT);
        pinMode(bPin, OUTPUT);

        applyColor();
    }

    void setColor(uint8_t r, uint8_t g, uint8_t b){
        this->r = r * reduceFactor;
        this->g = g * reduceFactor;
        this->b = b * reduceFactor;

        applyColor();
    }

    void setColor(Color c){
        switch (c)
        {
        case RED:
            setColor(255, 0, 0);
            break;

        case GREEN:
            setColor(0, 255, 0);
            break;

        case BLUE:
            setColor(0, 0, 255);
            break;

        case PURPLE:
            setColor(204, 0, 204);
            break;

        case YELLOW:
            setColor(255, 0, 0);
            break;

        case ORANGE:
            setColor(255, 153, 51);
            break;

        case PINK:
            setColor(255, 102, 255);
            break;

        case LIGHT_BLUE:
            setColor(51, 255, 255);
            break;

        case BROWN:
            setColor(153, 76, 0);
            break;
        
        default:
            setColor(0, 0, 0);
            break;
        }
    }
};