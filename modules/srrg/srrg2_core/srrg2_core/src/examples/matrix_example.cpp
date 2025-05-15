#include "srrg_data_structures/matrix.h"
#include <iostream>

#define EXAMPLE_LOG std::cerr << "srrg2_core::examples::matrix_example| "

using namespace std;
using namespace srrg2_core;

struct MyStruct {
  int count;
  vector<float> descriptor;
};

using MyMatrix = Matrix_<MyStruct>;

int main(int argc, char** argv) {
  int rows = 10;
  int cols = 10;

  MyMatrix grid;
  // Resize method
  grid.resize(rows, cols);

  for (size_t r = 0; r < grid.rows(); ++r) {
    // You can access the cells in a Matrix using :
    // 1) at(int row_index, int col_index)
    // 2) at(Eigen::Vector2i indicies)
    // 3) rowPtr(row_index) + col_index (Get a pointer to the desired cell)
    // 4) vector iterators (begin(),end())
    // 5) round bracket operator ()
    // 6) square bracket operator [] with single index which access the underlying vector
    //    This index can be extracted using the indexAt(row_index, col_index) method

    MyStruct* rowptr = grid.rowPtr(r);
    for (size_t c = 0; c < grid.cols(); ++c, ++rowptr) {
      rowptr->count = r * c;
      rowptr->descriptor.push_back(0.1f * r);
      rowptr->descriptor.push_back(0.1f * c);
    }
  }

  for (auto it = grid.begin(); it != grid.end(); ++it) {
    EXAMPLE_LOG << "Cell counter : " << it->count << std::endl;
    EXAMPLE_LOG << "Cell descriptor : " << it->descriptor[0] << " " << it->descriptor[1]
                << std::endl;
    // Get cell indicies
    EXAMPLE_LOG << "Cell indicies : " << grid.pos(&(*it)).transpose() << std::endl;
    std::cerr << std::endl;
  }

  // Get the offsets for the eight neightbors of a cell
  const int* offsets = grid.eightNeighborOffsets();
  MyStruct* cellptr  = grid.rowPtr(rows - 1);
  for (int i = 0; i < 8; ++i) {
    MyStruct* ptr                 = cellptr + offsets[i];
    Eigen::Vector2i cell_indicies = grid.pos(ptr);

    // Check if a cell is inside the grid
    if (!grid.inside(cell_indicies)) {
      EXAMPLE_LOG << "Cell " << cell_indicies.transpose() << " is outside of the grid" << std::endl;
    }

    // Check if a cell is on the border of the grid
    if (grid.onBorder(cell_indicies)) {
      EXAMPLE_LOG << "Cell " << cell_indicies.transpose() << " is at the border of the grid"
                  << std::endl;
    }
  }

  // Copy operator
  MyMatrix other_grid = grid;

  for (decltype(rows) row = 0; row < rows; ++row) {
    for (decltype(cols) col = 0; col < cols; ++col) {
      size_t vector_index = grid.indexAt(row, col);
      EXAMPLE_LOG << "Matrix::at method : " << grid.at(row, col).count
                  << " | Matrix square bracket operator : " << grid[vector_index].count
                  << std::endl;
    }
  }
}
