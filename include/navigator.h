#include <ArduinoEigenDense.h>
#include "visualize_sensors.h"
#include <vector>

class Navigator{

public:
    Eigen::Vector3f down;
    Eigen::Quaternion<float> gravityCorrection;
    MagCalData magCalData;
    float angle;
    Eigen::Vector3f accel;
    Eigen::Vector3f accelNoGravity;
    Eigen::Vector3f gyro;
    Eigen::Vector3f mag;

    float initialAccel;
    Eigen::Vector3f initialMag;
    Eigen::Vector3f initialGyro;

    Eigen::Vector3f accels[2];
    Eigen::Vector3f vels[2];
    Eigen::Vector3f pos;
    Eigen::Vector3f gyros[2];
    Eigen::Quaternionf orientation;

    bool trigger;

    int steps;
    long lastStep;

    Navigator();
    Eigen::Vector3f getMagVector() const;
    void setInitialMag();
    void calculateAccelNoGravity();
    void calculatePosition(unsigned long millis);
    void setCorrectGyro();
    void calculateOrientation(long millis);
    void step();
    void startMap();
};
