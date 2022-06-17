#include "Map.h"
#include <map>
#include <queue>

void Map::createNode(int closestApproachable, float x, float y){
    nodes.emplace_back(x, y);
    int index = nodes.size() - 1;
    nodes[closestApproachable].neighbours.push_back(index);
    nodes[index].neighbours.push_back(closestApproachable);
}

Map::Map(std::string name): started(false), name(name), nodes({Node(0, 0)}), activeNode(0){}

void Map::rotate(float azimuthInDegrees){

    Eigen::Rotation2Df rotation(-azimuthInDegrees / 180 * 3.14);

    for(Node& node : nodes){
        Eigen::Vector2f location(node.x, node.y);
        location = rotation * location;
        node.x = location.x();
        node.y = location.y();
    }
}

Node& Map::getActiveNode(){
    return nodes[activeNode];
}

void Map::updateShortestPathTree(){

    int done = 0;
    std::vector<int> parents(nodes.size());
    std::multimap<float, int> distances;         // <distance, index>

    parents[activeNode] = -1;

    for(int i = 0; i < nodes.size(); i++){
        if(i == activeNode) distances.insert(std::pair<float, int>(0, i));
        else distances.insert(std::pair<float, int>(std::numeric_limits<float>::infinity(), i));
    }

    while(done != nodes.size()){
        auto node = distances.begin();
        std::pair<float, int> pair = *node;
        float distance = pair.first;
        int index = pair.second;

        for(int neighbour : nodes[index].neighbours){

            float distanceToNeighbour = -1;
            std::map<float, int>::iterator neighbourIterator;

            // Find distance to neighbour
            for(auto i = distances.begin(); i != distances.end(); i++){
                if(i->second == neighbour){
                    distanceToNeighbour = i->first;
                    neighbourIterator = i;
                    break;
                }
            }

            if(distanceToNeighbour < 0){
                continue;
            }

            Node& n = nodes[neighbour];
            float uvDistance = (Eigen::Vector2f(nodes[index].x, nodes[index].y) - Eigen::Vector2f(n.x, n.y)).norm();
            if(distance + uvDistance < distanceToNeighbour){
                distances.erase(neighbourIterator);
                distances.insert(std::pair<float, int>(distance + uvDistance, neighbour));
                parents[neighbour] = index;
            }
        }

        distances.erase(node);
        done++;
    }

    currentShortestPathTree = parents;

}

void Map::startNavigating(int destination){

    currentDestination = destination;
    updateShortestPathTree();

}

int Map::getNextNode(){

    int currentNode = currentDestination;

    while(true){
        if(currentShortestPathTree[currentNode] == activeNode){
            return currentNode;
        }
        else{
            int next = currentShortestPathTree[currentNode];
            if(next == -1) {
                return currentNode;
            }
            else currentNode = next;
            
        }
    }

    return -1;

}

void Map::addEdge(int from, int to){

    nodes[from].neighbours.push_back(to);
    nodes[to].neighbours.push_back(from);

}
