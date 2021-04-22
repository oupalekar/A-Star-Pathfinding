//
// Created by Ojas Upalekar on 4/19/21.
//
#include "pathfinder.h"
#include <random>
int main(){
    using pathfinder::pathfinder;
    pathfinder AStar(10,10);
    AStar.CreateNodes();
    AStar.setObstacle(5,5,true);
    int i = 0;
    while (i < 10) {
        int x = rand() % 10;
        int y = rand() % 10;
        AStar.setObstacle(x,y,true);
        i++;
    }
    AStar.SolveAStar();
    AStar.PrintPath();
    
    return 0;
}
