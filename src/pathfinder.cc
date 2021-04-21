//
// Created by Ojas Upalekar on 4/19/21.
//
#include "pathfinder.h"
#include <cmath>

namespace pathfinder {


  bool pathfinder::isValid(const vector<vector<size_t>> &grid, const pathfinder::Cell &cell) const {
      return cell.first >= 0 && cell.first <= num_of_rows && cell.second >= 0 && cell.second <= num_of_cols;
  }

  bool pathfinder::isCellBlocked(const vector<vector<size_t>> &grid, const pathfinder::Cell &cell) const {
      return isValid(grid, cell) && grid[cell.first][cell.second] == 1;
  }

  bool pathfinder::isDestinationReached(const pathfinder::Cell &destination, const pathfinder::Cell &cell) {
      return cell.first == destination.first && cell.second == destination.second;
  }

  float pathfinder::CalculateHeuristic(const pathfinder::Cell &start, const pathfinder::Cell &destination) {
      float to_return = sqrt(pow(start.first - destination.first,2) + pow(start.second - destination.second, 2);
      return to_return;
  }

  void pathfinder::CreatePath(const vector<vector<size_t>> &grid, const pathfinder::Cell &destination) {
        
  }


} //namespace pathfinder