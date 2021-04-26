//
// Created by Ojas Upalekar on 4/19/21.
//
#include "Pathfinder.h"
#include <cmath>
#include <list>
#include <iostream>

namespace pathfinder {
  Pathfinder::Pathfinder(size_t numOfRows, size_t numOfCols) :num_of_rows(numOfRows), num_of_cols(numOfCols), nodes(vector<vector<Node*>>(num_of_cols, vector<Node*>(num_of_rows))) {
  }

  bool Pathfinder::isValid(const Pathfinder::Node* node) const {
      return node->x_ >= 0 && node->x_ <= num_of_rows && node->y_ >= 0 && node->y_ <= num_of_cols;
  }
  bool Pathfinder::isCellBlocked(const Pathfinder::Node* node) const {
      return isValid(node) && node->is_obstacle_;
  }

  float Pathfinder::CalculateHeuristic(const Pathfinder::Node* a, const Pathfinder::Node* b) {
      float to_return = sqrt(pow(a->x_ - b->x_,2) + pow(a->y_ - b->y_, 2));
      return to_return;
  }
  
  float Pathfinder::CalculateDistance(const Pathfinder::Node* a, const Pathfinder::Node* b) {
      return CalculateHeuristic(a,  b);
  }

  void Pathfinder::CreateNodes() {
      //There are 10 * 10 nodes
      //Initialize Nodes
      for (size_t x = 0; x < num_of_cols; x++) {
          for (size_t y = 0; y < num_of_rows; y++) {
              nodes[x][y] = new Node;
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
                  nodes[x][y]->neighbors_.push_back(nodes[x+1][y]);
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
     //start_node_ = nodes[num_of_cols/5][num_of_rows/2]; //[2][5]
      //end_node_ = nodes[4 * num_of_rows / 5][num_of_rows/2]; //[8][5]
  }

  void Pathfinder::setObstacle(size_t x, size_t y, bool isBlocked) {
      if (!(x >= 0  && x < num_of_rows)) {
          std::cout<<x<<std::endl;
          throw std::invalid_argument("Invalid x value");
      }
      if (!(y >= 0 && y < num_of_rows)) {
          throw std::invalid_argument("Invalid y value");
      }
      nodes[x][y]->is_obstacle_ = isBlocked;
  }

  void Pathfinder::SolveAStar() {
      //Set values to default
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
          //We want to check the nodes that have the smallest global goal because those are the nodes closest to the end node so we can sort it
          nodes_to_test.sort( [](const Node* a, const Node*b) { return a->global_goal_ < b->global_goal_;});
          
          //So we think that the first value in the list is the closest, but we might have visited it so:
          while((!nodes_to_test.empty()) && nodes_to_test.front()->is_visited_) {
              nodes_to_test.pop_front(); //Pop out the front value because we've visited it
          }

          if (nodes_to_test.empty()) {
              break;
          }
          
          currentNode = nodes_to_test.front();
          currentNode->is_visited_ = true; // We've visited the first node!
          
          for(Node* neighborNode : currentNode->neighbors_) {
              if(!neighborNode->is_obstacle_ && !neighborNode->is_visited_) {
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
                  neighborNode->global_goal_ =  neighborNode->global_goal_ + CalculateHeuristic(neighborNode, end_node_);
              }
          }
      }
  }

  void Pathfinder::PrintPath() {
      if (end_node_ != nullptr) {
          Node *node = end_node_;
          while (node->parent != nullptr) {
              std::cout << "(" << node->x_ << "," << node->y_ << ")" << std::endl;
              node = node->parent;
          }
      }
  }
  
  bool Pathfinder::DoesPathExist() {
      if (end_node_ != nullptr) {
          Node *node = end_node_;
          while (node->parent != nullptr) {
              node = node->parent;
          }
          if(node->x_ == start_node_->x_ && node->y_ == start_node_->y_) {
              return false;
          }
      }
      
      return true;
  }

  size_t Pathfinder::getNumOfRows() const {
      return num_of_rows;
  }

  size_t Pathfinder::getNumOfCols() const {
      return num_of_cols;
  }

  const vector<vector<Pathfinder::Node*>>& Pathfinder::getNodes() const {
      return nodes;
  }

  Pathfinder::Node *Pathfinder::getStartNode() const {
      return start_node_;
  }

  Pathfinder::Node *Pathfinder::getEndNode() const {
      return end_node_;
  }

  void Pathfinder::setDiagonals(bool diagonals) {
      diagonals_ = diagonals;
  }

  void Pathfinder::setStartNode(size_t x, size_t y) {
      start_node_ = nodes[x][y];
  }

  void Pathfinder::setEndNode(size_t x, size_t y) {
      end_node_ = nodes[x][y];
  }


} //namespace Pathfinder