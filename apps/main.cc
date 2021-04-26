//
// Created by Ojas Upalekar on 4/19/21.
//
#include "Pathfinder.h"
#include <iostream>
int main(){
    using pathfinder::Pathfinder;
    Pathfinder AStar(40, 40);
    AStar.setDiagonals(true);
    AStar.CreateNodes();
    //AStar.setObstacle(5,5,true);
    //AStar.setObstacle(6,5,true);
    //AStar.setObstacle(5,4,true);
    AStar.setStartNode(2,5);
    AStar.setEndNode(38,5);
    AStar.SolveAStar();
    if (AStar.DoesPathExist()) {
        std::cout<<"Path exists";
    }
    AStar.PrintPath();
    //std::cout<<"("<<AStar.getStartNode()->x_<<","<<AStar.getStartNode()->y_<<")";
    return 0;
}
