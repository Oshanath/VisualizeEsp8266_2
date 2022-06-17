#include "Map.h"
#include <Arduino.h>

Map getBoardingHouseMap(){
    Map map("Bodima");
    map.createNode(0, 1.07, 0);
    map.createNode(1, 3.14, 0);
    map.createNode(1, 1.07, 4.35);
    map.createNode(3, 1.07, 8.07);
    map.createNode(3, 3.14, 4.35);
    map.rotate(90);
    map.updateShortestPathTree();
    return map;
}

Map getEmbeddedLabMap(){
    Map map("Embedded lab");
    map.createNode(0, 0, -3);
    map.createNode(1, 6, -3);
    map.createNode(2, 6, 0);
    map.createNode(3, 7.5, 0);
    map.createNode(4, 7.5, 2.1);
    map.createNode(5, 2.1, 2.2);
    map.addEdge(0, 3);
    map.rotate(198);
    map.updateShortestPathTree();
    return map;
}