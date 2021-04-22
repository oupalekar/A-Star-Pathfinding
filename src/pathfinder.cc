//
// Created by Ojas Upalekar on 4/19/21.
//
#include "pathfinder.h"
#include <cmath>
#include <list>


namespace pathfinder {


  bool pathfinder::isValid(const pathfinder::Node* node) const {
      return node->x_ >= 0 && node->x_ <= num_of_rows && node->y_ >= 0 && node->y_ <= num_of_cols;
  }
  bool pathfinder::isCellBlocked(const pathfinder::Node* node) const {
      return isValid(node) && node->is_obstacle_;
  }

  float pathfinder::CalculateHeuristic(const pathfinder::Node* a, const pathfinder::Node* b) {
      float to_return = sqrt(pow(a->x_ - b->x_,2) + pow(a->y_ - b->y_, 2));
      return to_return;
  }
  
  float pathfinder::CalculateDistance(const pathfinder::Node* a, const pathfinder::Node* b) {
      return CalculateHeuristic(a,  b);
  }

  void pathfinder::CreateNodes() {
      //There are 10 * 10 nodes
      //Initialize Nodes
      for (size_t x = 0; x < num_of_cols; x++) {
          for (size_t y = 0; y < num_of_rows; y++) {
              nodes[x][y]->x_ = x;
              nodes[x][y]->y_ = y;
              nodes[x][y]->is_obstacle_ = false;
              nodes[x][y]->is_visited_ = false;
              nodes[x][y]->parent = nullptr;
          }
      }
      
      //Create the connections
      for (size_t x = 0; x < num_of_cols; x++) {
          for (size_t y = 0; y < num_of_rows; y++) {
              //We don't want the edges to have connections outside the grid so:
              if (y > 0) {
                  nodes[x][y]->neighbors_.push_back(nodes[x][y-1]);
              }
              if (y < num_of_rows - 1) {
                  nodes[x][y]->neighbors_.push_back(nodes[x][y+1]);
              }
              if (x > 0) {
                  nodes[x][y]->neighbors_.push_back(nodes[x+1][y]);
              }
              if (x < num_of_cols - 1) {
                  nodes[x][y]->neighbors_.push_back(nodes[x-1][y]);
              }
              
              if (diagonals_) {
                  if (y > 0 && x > 0) {
                      nodes[x][y]->neighbors_.push_back(nodes[x-1][y-1]);
                  }
                  if (y < num_of_rows - 1 && x > 0) {
                      nodes[x][y]->neighbors_.push_back(nodes[x-1][y+1]);
                  }
                  if (y > 0 && x < num_of_cols - 1) {
                      nodes[x][y]->neighbors_.push_back(nodes[x+1][y-1]);
                  }
                  if (y < num_of_rows - 1 && y < num_of_cols - 1) {
                      nodes[x][y]->neighbors_.push_back(nodes[x+1][y+1]);
                  }  
              }
          }
      }
      
      //Manually setup start and end nodes
      start_node_ = nodes[num_of_cols/5][num_of_rows/2]; //[2][5]
      end_node_ = nodes[4 * num_of_rows / 5][num_of_rows/2]; //[8][5]
  }

  void pathfinder::setObstacle(size_t x, size_t y, bool isBlocked) {
      if (!(x > 0  && x < num_of_rows)) {
          throw std::invalid_argument("Invalid x value");
      }
      if (!(y > 0 && y < num_of_rows)) {
          throw std::invalid_argument("Invalid y value");
      }
      nodes[x][y]->is_obstacle_ = isBlocked;
  }

  void pathfinder::SolveAStar() {
      //Set values to default
      CreateNodes();
      for(const vector<Node*>& vecs : nodes) {
          for(Node* node : vecs) {
              node->is_visited_ = false;
              node->global_goal_ = INFINITY; //Biggest possible value, and the best value is the least
              node->local_goal_ = INFINITY;
              node->parent = nullptr;
          }
      }
      
      Node* currentNode = start_node_;
      start_node_->local_goal_ = 0; //Set to zero because the local goal is calculated by distance from parent, start has no parent
      start_node_->global_goal_ = CalculateHeuristic(start_node_,end_node_);
      
      //There will be a list to test as we explore nodes around the starting node
      std::list<Node*> nodes_to_test;
      nodes_to_test.push_back(start_node_);
      
      //Now we can check nodes in the list
      while (!nodes_to_test.empty() && currentNode != end_node_) {
          //We want to check the nodes that have the smalles global goal because those are the nodes closest to the end node so we can sort it
          nodes_to_test.sort( [](const Node& a, const Node&b) { return a.global_goal_ < b.global_goal_;});
          
          //So we think that the first value in the list is the closest, but we might have visited it so:
          if((!nodes_to_test.empty()) && nodes_to_test.front()->is_visited_) {
              nodes_to_test.pop_front(); //Pop out the front value because we've visited it
          }

          if (!nodes_to_test.empty()) {
              break;
          }
          
          currentNode = nodes_to_test.front();
          currentNode->is_visited_ = true; // We've visited the first node!
          
          for(Node* neighborNode : currentNode->neighbors_) {
              if(!isCellBlocked(neighborNode) && !neighborNode->is_visited_) {
                  nodes_to_test.push_back(neighborNode);
              }
              
              float possibleLocalGoal = currentNode->local_goal_ + CalculateDistance(neighborNode, currentNode);
              //We have to replace the localGoal of the neighbor IF the possibleLocal is less than the current local
              //Remember than initially, we set the localGoal of all nodes except the start one to INFINITY
              //It also means that parent is the current node because it had an effect on the neighbor node
              if (possibleLocalGoal < neighborNode->local_goal_) {
                  neighborNode->local_goal_ = possibleLocalGoal;
                  neighborNode->parent = currentNode;
                  
                  
                  //Additionally we have to update the global goal because this might be a path we want to take
                  float possibleGlobalGoal = currentNode->global_goal_ + CalculateHeuristic(neighborNode, end_node_);
                  if (possibleLocalGoal < neighborNode->global_goal_)  {
                      neighborNode->global_goal_ = possibleGlobalGoal;
                  }
              }
          }
      }
  }


} //namespace pathfinder