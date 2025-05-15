#include "path_matrix_clusterer_search.h"
#include <iomanip>
#include <iostream>

namespace srrg2_core {
  using namespace std;

  ClustererPathSearch::ClustererPathSearch() {
    _regions_image = 0;
  }

  void ClustererPathSearch::setColor(int color_) {
    std::stringstream stream;
    stream << std::setw(6) << std::setfill('0') << std::hex << color_;
    std::string result(stream.str());

    unsigned long r_value = std::strtoul(result.substr(0, 2).c_str(), 0, 16);
    unsigned long g_value = std::strtoul(result.substr(2, 2).c_str(), 0, 16);
    unsigned long b_value = std::strtoul(result.substr(4, 2).c_str(), 0, 16);

    _color = Vector3uc(r_value, g_value, b_value);
  }

  void ClustererPathSearch::init() {
    if (!_path_map)
      throw std::runtime_error("no output map selected");

    if (!_regions_image)
      throw std::runtime_error("no regions_image selected");

    int rows = _regions_image->rows();
    int cols = _regions_image->cols();
    if (rows < 3 || cols < 3) {
      throw std::runtime_error("map too small to compute clustrer map");
    }

    _cluster_image.resize(rows, cols);

    _cluster_image.fill(Vector3uc::Zero());

    PathMatrix& output = *_path_map;
    output.resize(rows, cols);
    PathMatrixCell init_value(std::numeric_limits<float>::max(), 1.0f, 0.0f, nullptr);
    output.fill(init_value);
    fillFromImage();
    _clusters.clear();
    _max_index = 0;
  }

  void ClustererPathSearch::fillFromImage() {
    PathMatrix& output = *_path_map;
    int rows           = output.rows();
    int cols           = output.cols();
    for (int r = 0; r < rows; ++r) {
      const int* regions_ptr   = _regions_image->rowPtr(r);
      PathMatrixCell* cell_ptr = output.rowPtr(r);
      for (int c = 0; c < cols; ++c, ++regions_ptr, ++cell_ptr) {
        int idx = *regions_ptr;
        if (!idx) {
          cell_ptr->cost = 1;
        } else {
          cell_ptr->cost = std::numeric_limits<float>::max();
        }
      }
    }
  }

  void ClustererPathSearch::expandRegion(PathMatrixCell* cell) {
    // cerr << __PRETTY_FUNCTION__ << ": r=" <<cell->r << " " << "c=" << cell->c << endl;
    PathMatrix& output = *_path_map;

    int r_min = std::numeric_limits<int>::max(), r_max = std::numeric_limits<int>::min();
    int c_min = std::numeric_limits<int>::max(), c_max = std::numeric_limits<int>::min();

    // cerr << "startq: "  << maxQSize << endl;
    // int currentDistance = 0;
    cell->parent   = cell;
    cell->distance = 0;
    std::deque<PathMatrixCell*> queue;
    queue.push_back(cell);
    Vector2i sums = output.pos(cell);

    if (sums[0] < r_min)
      r_min = sums[0];
    if (sums[0] > r_max)
      r_max = sums[0];

    if (sums[1] < c_min)
      c_min = sums[1];
    if (sums[1] > c_max)
      c_max = sums[1];

    Vector2iVector pixels;

    int count = 0;
    while (!queue.empty()) {
      PathMatrixCell* current = queue.front();
      queue.pop_front();
      Vector2i idx_current = output.pos(current);
      if (output.onBorder(idx_current))
        continue;
      for (int i = 0; i < 8; i++) {
        PathMatrixCell* child = current + output.eightNeighborOffsets()[i];
        if (child->cost == std::numeric_limits<float>::max())
          continue;
        if (child->parent)
          continue;
        child->parent      = cell;
        Vector2i child_idx = output.pos(child);
        sums += child_idx;

        _cluster_image.at(child_idx) = _color;
        pixels.push_back(child_idx);

        if (child_idx[0] < r_min)
          r_min = child_idx[0];
        if (child_idx[0] > r_max)
          r_max = child_idx[0];

        if (child_idx[1] < c_min)
          c_min = child_idx[1];
        if (child_idx[1] > c_max)
          c_max = child_idx[1];

        ++count;
        _num_operations++;
        queue.push_back(child);
      }
    }
    Cluster cluster;
    cluster.cell        = cell;
    cluster.mean_r      = (float) sums[0] / (float) count;
    cluster.mean_c      = (float) sums[1] / (float) count;
    cluster.lower       = Eigen::Vector2i(r_min, c_min);
    cluster.upper       = Eigen::Vector2i(r_max, c_max);
    cluster.point_count = count;
    cluster.pixels      = pixels;
    cluster.color       = _color;
    _clusters.push_back(cluster);
    ++_max_index;
  }

  bool ClustererPathSearch::compute() {
    PathMatrix& output = *_path_map;
    int rows           = output.rows();
    int cols           = output.cols();

    _num_operations = 0;
    for (int r = 0; r < rows; ++r) {
      PathMatrixCell* cell_ptr = output.rowPtr(r);
      for (int c = 0; c < cols; ++c, ++cell_ptr) {
        if (output.onBorder(r, c))
          continue;
        if (cell_ptr->cost == std::numeric_limits<float>::max())
          continue;
        if (cell_ptr->parent)
          continue;
        expandRegion(cell_ptr);
      }
    }
    return true;
  }

} // namespace srrg2_core
