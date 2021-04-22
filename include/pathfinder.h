//
// Created by Ojas Upalekar on 4/19/21.
//

#ifndef A_STAR_PATHFINDER_H
#define A_STAR_PATHFINDER_H

#include <cstdlib>
#include <string>
#include <vector>

namespace pathfinder {
  using std::vector;
  using std::pair;
  class pathfinder {
      
      struct Node {
          bool is_visited_ = false; //Have we visited this node
          bool is_obstacle_ = false; //Is the node an obstacle?
          int x_; // X coordinate on the grid
          int y_; // Y coordinate on the grid
          double global_goal_; //Distance to goal
          double local_goal_;  
          vector<Node*> neighbors_; //All nodes that can be traversed to from this node
          Node* parent; //Parent node
          
          bool operator==(const Node* node) const {
              return x_ == node->x_ && y_ == node->y_;
          }
      };
    public:
      bool isValid(const Node* node) const;
      
      bool isCellBlocked(const Node* node) const;

      static float CalculateHeuristic(const Node* a, const Node* b);
      
      void CreateNodes();
      
      void SolveAStar();
      
      void setObstacle(size_t x, size_t y, bool isBlocked);

      static float CalculateDistance(const pathfinder::Node* a, const pathfinder::Node* b);
      
    private:
      
      vector<vector<Node*>> nodes = vector<vector<Node*>>(num_of_rows, vector<Node*>(num_of_cols));
      Node* start_node_ = nullptr;
      Node* end_node_ = nullptr;
      size_t num_of_rows = 10;
      size_t num_of_cols = 10;
      bool diagonals_ = false;
  };
}


#endif //A_STAR_PATHFINDER_H
