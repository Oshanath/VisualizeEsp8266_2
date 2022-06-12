#pragma once
#include "Node.h"
#include <vector>

class Map{
public:
    std::vector<Node> nodes;
    bool started;

    Map(): started(false){
        
    }

    void createNode(int closestApproachable, float x, float y, std::string name){
        nodes.emplace_back();
        int index = nodes.size() - 1;
        nodes[closestApproachable].neighbours.push_back(index);
        nodes[index].neighbours.push_back(closestApproachable);
        nodes[index].x = x;
        nodes[index].y = y;
        nodes[index].name = name;
    }

};

/*
Sync with RFID. Get the magnetic north at the same time.
Set the device's tracking orientation to point to the current direction relative to the north. (make north 1, 0 direction)
Rotate the current north to the absolute north. Absolute north is with the map itself.
*/