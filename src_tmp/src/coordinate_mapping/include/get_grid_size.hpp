#ifndef GET_GRID_SIZE_HPP
#define GET_GRID_SIZE_HPP

#include <utility>

inline std::pair<int, int> get_grid_size() {
  int grid_cols = 4;
  int grid_rows = 3;
  return {grid_cols, grid_rows};
}

#endif // GET_GRID_SIZE_HPP