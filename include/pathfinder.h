//
// Created by Ojas Upalekar on 4/19/21.
//

#ifndef A_STAR_PATHFINDER_H
#define A_STAR_PATHFINDER_H

#include <cstdlib>
#include <vector>

namespace pathfinder {
  using std::vector;
  using std::pair;
  class pathfinder {
      typedef pair<int, int> Cell;
      
      struct Node {
          Cell parent;
          double f, g, h;
          Node(): parent(-1, -1), f(-1), g(-1), h(-1) {}
      };
    public:
      bool isValid(const vector<vector<size_t>>& grid, const Cell& cell) const;
      
      bool isCellBlocked(const vector<vector<size_t>>& grid, const Cell& cell) const;
      
      static bool isDestinationReached(const Cell& destination, const Cell& cell);
      
      float CalculateHeuristic(const Cell& start, const Cell& destination);
      
      void CreatePath(const vector<vector<size_t>>& grid, const Cell& destination);

    private:
      size_t num_of_rows = 10;
      size_t num_of_cols = 10;
  };
}


#endif //A_STAR_PATHFINDER_H
