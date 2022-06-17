#include "navigator.h"

#define epsilon 0.1f
#define accelEpsilon 2.0f
#define velEpsilon 1.0f

Navigator::Navigator(): 
        pos(Eigen::Vector3f(0, 0, 0)),
        gyros({
        Eigen::Vector3f(0, 0, 0),
        Eigen::Vector3f(0, 0, 0)
        }),
        orientation(Eigen::Quaternionf(1, 0, 0, 0)),
        angle(0.0f),
        trigger(false),
        steps(0),
        lastStep(0),
        averageStep(0.431818)
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

// void Navigator::calculateAccelNoGravity(){
//     accelNoGravity = orientation * accel;
//     accelNoGravity = accelNoGravity - initialAccel * Eigen::Vector3f(0, 0, -1);
//     accelNoGravity = orientation.inverse() * accelNoGravity;
// }


void Navigator::calculatePosition(){

    pos += getForwardVector() * averageStep;

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

void Navigator::step(Map& currentMap, void (*stopNavigation)()){

    long now = millis();
    int activeNode = currentMap.activeNode;

    if(now - lastStep > 200){
        steps++;
        lastStep = now;
        calculatePosition();
        printVector3f(pos);

        for(int neighbour : currentMap.getActiveNode().neighbours){
            Node& neighbourNode = currentMap.nodes[neighbour];

            if((pos - Eigen::Vector3f(neighbourNode.x, neighbourNode.y, 0)).norm() < 0.4){
                currentMap.activeNode = neighbour;
                break;
            }
        }

        if(activeNode != currentMap.activeNode){
            currentMap.updateShortestPathTree();
        }

        if(activeNode == currentMap.currentDestination){
            stopNavigation();
        }

    }

}

void Navigator::startMap(){

    Eigen::Vector3f north = ((Eigen::Vector3f(0, 0, 1).cross(mag)).cross(Eigen::Vector3f(0, 0, 1)));
    orientation = Eigen::Quaternionf::FromTwoVectors(north, Eigen::Vector3f(1, 0, 0));

}

Eigen::Vector3f Navigator::getForwardVector(){
    Eigen::Vector3f north = ((Eigen::Vector3f(0, 0, 1).cross(mag)).cross(Eigen::Vector3f(0, 0, 1)));
    auto q = Eigen::Quaternionf::FromTwoVectors(north, Eigen::Vector3f(1, 0, 0));
    return q * Eigen::Vector3f(1, 0, 0);
}

float Navigator::getAngleDifference(Map& map){

    auto forward3 = getForwardVector();
    Eigen::Vector2f forward(forward3.x(), forward3.y());
    int nextNode = map.getNextNode();
    Eigen::Vector2f destination(map.nodes[nextNode].x, map.nodes[nextNode].y);
    Eigen::Vector2f position(pos.x(), pos.y());

    return abs(acos(forward.normalized().dot((destination - position).normalized()))) / 3.14 * 180;

}
