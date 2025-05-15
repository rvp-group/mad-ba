#include "path_matrix_distance_search.h"

namespace srrg2_core {
  using namespace std;

  void PathMatrixDistanceSearch::reset() {
    // ensure valid params
    if (!_path_map) {
      std::cerr << __PRETTY_FUNCTION__ << std::endl;
      throw std::runtime_error("[PathMatrixDistanceSearch::reset] please set a path map");
    }
    if (param_max_distance_squared_pxl.value() < 0) {
      std::cerr << __PRETTY_FUNCTION__ << std::endl;
      throw std::runtime_error(
        "[PathMatrixDistanceSearch::reset] please set max_distance_squared_pxl");
    }
    const size_t& rows = _path_map->rows();
    const size_t& cols = _path_map->cols();
    if (rows < 3 || cols < 3) {
      std::cerr << "invalid dimensions" << std::endl;
      throw std::runtime_error(
        "[PathMatrixDistanceSearch::reset] map is too small to compute a distance map");
    }

    // fill structures
    _goal_map.resize(_path_map->rows(), _path_map->cols());
    _goal_map.fill(-1);
    _queue = PathSearchQueue();

    _path_map_changed_flag   = false;
    _parent_map_changed_flag = true;
  }

  void PathMatrixDistanceSearch::compute() {
    if (_path_map_changed_flag) {
      reset();
    }
    if (_queue.empty()) {
      std::cerr << __PRETTY_FUNCTION__ << ": Warning queue empty" << std::endl;
    }
    while (!_queue.empty()) {
      PathMatrixCell* current    = _queue.top().cell;
      PathMatrixCell* parent     = current->parent;
      Eigen::Vector2i parent_pos = _path_map->pos(parent);
      _queue.pop();

      Eigen::Vector2i current_pos = _path_map->pos(current);
      if (_path_map->onBorder(current_pos)) {
        continue;
      }
      const int* neighbor_offsets = _path_map->eightNeighborOffsets();

      for (int i = 0; i < 8; ++i) {
        PathMatrixCell* neighbor     = current + neighbor_offsets[i];
        if (neighbor->distance<0)
          continue;
        Eigen::Vector2i neighbor_pos = _path_map->pos(neighbor);
        int d                        = (parent_pos - neighbor_pos).squaredNorm();
        if (d < param_max_distance_squared_pxl.value() && d < neighbor->distance) {
          neighbor->parent   = parent;
          neighbor->distance = d;
          PathSearchEntry q;
          q.cell = neighbor;
          q.cost = neighbor->distance;
          _queue.push(q);
        }
      }
    }
    _parent_map_changed_flag = true;
    _path_map_changed_flag   = false;
    // ImageUInt8 dest_image;
    // _path_map->toImage(dest_image, PathMatrix::Distance);
    // cv::Mat dest_cv_mat;
    // dest_image.toCv(dest_cv_mat);
    // cv::imshow("dist",dest_cv_mat);
    // cv::waitKey(1);
  }

  const MatrixInt& PathMatrixDistanceSearch::parentMap() {
    if (_path_map_changed_flag) {
      //cerr << "parent_map: compute" << endl;
      compute();
    }
    if (_parent_map_changed_flag) {
      _parent_map.resize(_goal_map.rows(), _goal_map.cols());
      std::size_t s         = _path_map->data().size();
      PathMatrixCell* start = &(_path_map->data()[0]);
      _parent_map.fill(-1);

      for (std::size_t i = 0; i < s; ++i) {
        const PathMatrixCell& c = _path_map->data()[i];
        if (!c.parent) {
          continue;
        }
        std::ptrdiff_t idx = c.parent - start;
        if (idx >= (std::ptrdiff_t) _goal_map.data().size()) {
          cerr << __PRETTY_FUNCTION__ << "Error index out of bounds!" << idx << " "
               << _goal_map.data().size() << endl;
        }
        _parent_map.data()[i] = _goal_map.data()[idx];
      }
    }
    _parent_map_changed_flag = false;
    return _parent_map;
  }

} // namespace srrg2_core
