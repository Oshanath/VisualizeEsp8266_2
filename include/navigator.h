#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <ArduinoEigenDense.h>
#include "visualize_sensors.h"
#include "Map.h"
#include <vector>

class Navigator{

public:
    MagCalData magCalData;
    float angle;

    Eigen::Vector3f gyro;
    Eigen::Vector3f mag;

    float initialAccel;
    Eigen::Vector3f initialMag;
    Eigen::Vector3f initialGyro;

    Eigen::Vector3f pos;
    Eigen::Vector3f gyros[2];
    Eigen::Quaternionf orientation;
    float averageStep;

    bool trigger;

    int steps;
    long lastStep;

    Navigator();
    Eigen::Vector3f getMagVector() const;
    void setInitialMag();
    void calculateAccelNoGravity();
    void calculatePosition();
    void setCorrectGyro();
    void calculateOrientation(long millis);
    void step(Map& map, void (*stopNavigation)());
    void startMap();
    Eigen::Vector3f getForwardVector();
    float getAngleDifference(Map& map);
};

#endif