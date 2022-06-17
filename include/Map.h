#ifndef MAP_H
#define MAP_H

#include "Node.h"
#include <vector>
#include <Arduino.h>
#include <string>
#include <ArduinoEigenDense.h>
#include "arduino-timer.h"

class Map{
public:
    std::vector<Node> nodes;
    int activeNode;
    bool started;
    std::string name;
    std::vector<int> currentShortestPathTree;
    int currentDestination;

    Map(std::string name);
    void createNode(int closestApproachable, float x, float y);
    Node& getActiveNode();
    void rotate(float azimuthInDegrees);
    void updateShortestPathTree();
    void startNavigating(int destination);
    int getNextNode();
    void addEdge(int from, int to);

};

#endif