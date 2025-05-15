#pragma once
#include "srrg_image/image.h"
#include <limits>

namespace srrg2_core {

  struct PathMatrixCell {
    PathMatrixCell(const float distance_   = 0.f,
                   const float cost_       = 1.f,
                   const float heuristic_  = 0.f,
                   PathMatrixCell* parent_ = nullptr) :
      distance(distance_),
      cost(cost_),
      heuristic(heuristic_),
      parent(parent_) {
    }
    float distance;  // bb distance computed by distance map
    float cost;      // bb transversal cost Dijkstra
    float heuristic; // bb A* heuristic
    PathMatrixCell* parent = nullptr;
  };

  // a path map is a map where each cell points to another cell
  // used to implement grid searches like dijkstra, or distance map
  // using an 8 neighborhood connectivity

  class PathMatrix : public Matrix_<PathMatrixCell> {
  public:
    enum Type { Parent, Cost, Distance };

    PathMatrix() : Matrix_<PathMatrixCell>() {
    }

    // use the Vector2i pos(cell_ptr) to access the indices if a cell from its ptr
    // within a path map

    PathMatrix(const size_t& rows, const size_t& cols) : Matrix_<PathMatrixCell>(rows, cols) {
    }

    // copies the field selected with channel in the path matrix
    // if channel is Parent, the indices are specified through an
    // IntImage. The parent elements are represented through the linear
    // position in a matrix:
    // if dest(r,c)=-1, parent=0;
    // if (dest(r,c))=k, parent=&(data()[k])
    // throws on invalind dest type
    void toImage(BaseImage& dest, const Type channel) const;

    // opposite of the above
    void fromImage(const BaseImage& target, const Type channel);

    float maxValue(const Type channel) const;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace srrg2_core
