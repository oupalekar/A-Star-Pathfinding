//
// Created by Ojas Upalekar on 4/22/21.
//
#include <catch2/catch.hpp>
#include "Pathfinder.h"

using pathfinder::Pathfinder;

TEST_CASE("Test constructor") {
   Pathfinder AStar(10, 10);
   SECTION("Rows/columns") {
       REQUIRE(AStar.getNumOfCols() == 10);
       REQUIRE(AStar.getNumOfRows() == 10);
   }
   SECTION("start/end nodes") {
       REQUIRE(AStar.getStartNode() == nullptr);
       REQUIRE(AStar.getEndNode() == nullptr);
   }
   SECTION("vector of nodes") {
       REQUIRE(AStar.getNodes().size() * AStar.getNodes()[0].size() == 100);
   }
}

TEST_CASE("Create Nodes") {
    Pathfinder AStar(10, 10);
    AStar.CreateNodes();
    SECTION("Check x,y"){
        REQUIRE(AStar.getNodes()[0][0]->x_ == 0);
        REQUIRE(AStar.getNodes()[0][0]->y_ == 0);
    }
    SECTION("Check booleans") {
        REQUIRE_FALSE(AStar.getNodes()[0][0]->is_visited_);
        REQUIRE_FALSE(AStar.getNodes()[0][0]->is_obstacle_);
    }
    SECTION("Check neighbors") {
        REQUIRE(AStar.getNodes()[0][0]->neighbors_.size() == 2);
        REQUIRE(AStar.getNodes()[1][1]->neighbors_.size() == 4);
    }
    SECTION("Check neighbors diagonals") {
        AStar.setDiagonals(true);
        AStar.CreateNodes();
        REQUIRE(AStar.getNodes()[0][0]->neighbors_.size() == 3);
        REQUIRE(AStar.getNodes()[1][1]->neighbors_.size() == 8);
    }
}

TEST_CASE("Pathfinding path") {
    Pathfinder AStar(10, 10);
    AStar.CreateNodes();
    AStar.setObstacle(5,5, true);
    AStar.SolveAStar();
    REQUIRE(AStar.getNodes()[6][5]->parent->x_ == 6);
    REQUIRE(AStar.getNodes()[6][5]->parent->y_ == 4);
}

