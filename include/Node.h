#pragma once
#include <string>
#include <vector>

class Node{

public:
    float x;
    float y;
    std::vector<int> neighbours;

    Node(float x, float y): x(x), y(y){}

};