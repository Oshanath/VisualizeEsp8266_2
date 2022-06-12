#pragma once
#include <string>
#include <vector>

class Node{

public:
    std::string name;
    float x;
    float y;
    std::vector<int> neighbours;

    Node(){
        
    }

};