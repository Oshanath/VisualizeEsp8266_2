#include "navigator.h"

#define epsilon 0.1f
#define accelEpsilon 2.0f
#define velEpsilon 1.0f

Navigator::Navigator(): accels({
        Eigen::Vector3f(0, 0, 0),
        Eigen::Vector3f(0, 0, 0)}),
        
        vels({
        Eigen::Vector3f(0, 0, 0),
        Eigen::Vector3f(0, 0, 0)}),
        
        pos(Eigen::Vector3f(0, 0, 0)),
        gyros({
        Eigen::Vector3f(0, 0, 0),
        Eigen::Vector3f(0, 0, 0)
        }),
        orientation(Eigen::Quaternionf(1, 0, 0, 0)),
        angle(0.0f),
        trigger(false),
        steps(0),
        lastStep(0)
        {}

Eigen::Vector3f Navigator::getMagVector() const{
    Eigen::Vector3i magnetometerRawi;
    getMagnetometerRaw(magnetometerRawi);
    Eigen::Vector3f magnetometerRaw(magnetometerRawi.x(), magnetometerRawi.y(), magnetometerRawi.z());
    return getCorrectMag(magnetometerRaw, magCalData);
}

void Navigator::setInitialMag(){
    initialMag = getMagVector();
}

void Navigator::calculateAccelNoGravity(){
    accelNoGravity = orientation * accel;
    accelNoGravity = accelNoGravity - initialAccel * Eigen::Vector3f(0, 0, -1);
    accelNoGravity = orientation.inverse() * accelNoGravity;
}


void Navigator::calculatePosition(unsigned long millis){

    accels[0] = accels[1];
    accels[1] = accelNoGravity;
    accels[1] = -accels[1];

    vels[0] = vels[1];
    vels[1] = vels[1] + 0.5 * float(millis) * 0.001 * (accels[1] + accels[0]);

    if(!trigger or (abs(gyro.x()) < epsilon and abs(gyro.y()) < epsilon and abs(gyro.z()) < epsilon)){
        vels[1] = Eigen::Vector3f(0, 0, 0);
    }

    pos = pos + (0.5 * float(millis) * 0.001 * (vels[1] + vels[0]));

}

void Navigator::calculateOrientation(long millis){
    gyros[0] = gyros[1];
    gyros[1] = gyro;
    Eigen::Vector3f eulerRad = 0.5 * millis / 1000 * (gyros[0] + gyros[1]);
    Eigen::Quaternionf q = Eigen::AngleAxisf(eulerRad[2], Eigen::Vector3f(0, 0, 1)) * 
                            Eigen::AngleAxisf(eulerRad[1], Eigen::Vector3f(0, 1, 0)) *
                            Eigen::AngleAxisf(eulerRad[0], Eigen::Vector3f(1, 0, 0));
    orientation = orientation * q;

    // // If device is still and flat correct orientation using gravity and magnetic north

    // if(abs(gyro.x()) < epsilon && abs(gyro.y()) < epsilon && abs(gyro.z()) < epsilon){
    //     float angle = 2 * acos(Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0, 0, -1), accel).w()) / 3.14 * 180;
    //     if(angle < 10){
    //         Eigen::Vector3f newAccel = orientation * Eigen::Vector3f(0, 0, -1);
    //         auto q = Eigen::Quaternionf::FromTwoVectors(newAccel, accel);
    //         orientation = orientation * q;

    //         auto north = ((Eigen::Vector3f(0, 0, 1).cross(-mag)).cross(Eigen::Vector3f(0, 0, -1)));
    //         auto theoreticalMag = orientation.inverse() * initialMag;
    //         auto theoreticalNorth = ((Eigen::Vector3f(0, 0, 1).cross(-theoreticalMag)).cross(Eigen::Vector3f(0, 0, -1)));
    //         orientation = Eigen::Quaternionf::FromTwoVectors(theoreticalNorth, north).inverse() * orientation;

    //     }
    // }
}

void Navigator::setCorrectGyro(){
    gyro = gyro - initialGyro;
    Eigen::Vector3f temp(-gyro.y(), gyro.x(), gyro.z());
    gyro = temp;
}

void Navigator::step(){

    long now = millis();

    if(now - lastStep > 200){
        steps++;
        lastStep = now;
        Serial.println(steps);
    }

}

void Navigator::startMap(){

    Eigen::Vector3f north = ((Eigen::Vector3f(0, 0, 1).cross(mag)).cross(Eigen::Vector3f(0, 0, 1)));
    orientation = Eigen::Quaternionf::FromTwoVectors(north, Eigen::Vector3f(1, 0, 0));

}
