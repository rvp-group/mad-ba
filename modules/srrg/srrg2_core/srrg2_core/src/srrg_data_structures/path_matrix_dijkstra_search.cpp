#include "path_matrix_dijkstra_search.h"

namespace srrg2_core {
  using namespace std;

  void PathMatrixDijkstraSearch::reset() {
    // ensure valid params
    if (!_path_map) {
      std::cerr << __PRETTY_FUNCTION__ << std::endl;
      throw std::runtime_error("[PathMatrixDijkstraSearch::reset] please set a path map");
    }
    if (param_max_cost.value() < 0) {
      std::cerr << __PRETTY_FUNCTION__ << std::endl;
      throw std::runtime_error("[PathMatrixDijkstraSearch::reset] please set max_cost");
    }
    const size_t& rows = _path_map->rows();
    const size_t& cols = _path_map->cols();
    if (rows < 3 || cols < 3) {
      std::cerr << "invalid dimensions" << std::endl;
      throw std::runtime_error(
        "[PathMatrixDijkstraSearch::reset] map is too small to compute a distance map");
    }
    float max_value_distance = 0.f;
    for (size_t r = 0; r < rows; ++r) {
      for (size_t c = 0; c < cols; ++c) {
        PathMatrixCell& p = _path_map->at(r, c);
        p.parent          = 0;
        p.cost            = param_max_cost.value();
        if (p.distance > max_value_distance) {
          max_value_distance = p.distance;
        }
      }
    }
    _max_value_distance    = max_value_distance;
    _queue                 = PathSearchQueue();
    _path_map_changed_flag = false;
  }

  void PathMatrixDijkstraSearch::compute() {
    if (_path_map_changed_flag) {
      reset();
    }
    if (_queue.empty()) {
      std::cerr << __PRETTY_FUNCTION__ << ": Warning queue empty" << std::endl;
    }
    if (!_search_param_changed_flag) {
      return;
    }
    // cerr << "min_distance : " << _max_value_distance << endl;
    // cerr << "cst coeff: [";
    // for (size_t i = 0; i < param_cost_polynomial.size(); ++i) {
    //   cerr << param_cost_polynomial.value(i) << " ";
    // }
    // cerr << "]" << endl;

    int num_expansions           = 0;
    static constexpr float sqrt2 = sqrt(2.f);
    while (!_queue.empty()) {
      // pop element
      PathSearchEntry q_current = _queue.top();
      PathMatrixCell* current   = q_current.cell;
      float current_cost        = current->cost;
      _queue.pop();
      ++num_expansions;
      // if element has been already updated we neglect the expansion
      if (current_cost < q_current.cost) {
        continue;
      }
      // if point on border, no expansion
      Eigen::Vector2i current_pos = _path_map->pos(current);
      if (_path_map->onBorder(current_pos)) {
        continue;
      }

      // expand neighbors
      const int* neighbor_offsets = _path_map->eightNeighborOffsets();
      for (int i = 0; i < 8; ++i) {
        PathMatrixCell* neighbor = current + neighbor_offsets[i];
        // if too close to obstacle, we drop the insertion
        if (neighbor->distance <= param_min_distance.value()) {
          continue;
        }

        // if the neighbor is already better than the current, no insertion
        if (neighbor->cost < current_cost) {
          continue;
        }

        // we check if the new path is better than the previous path
        Eigen::Vector2i neighbor_pos = _path_map->pos(neighbor);

        // d either 1 or 2 (for diagonal elements)
        int d = (current_pos - neighbor_pos).squaredNorm();

        // compute the traversal cost
        float distance_to_neighbor = 1;
        if (d > 1) {
          distance_to_neighbor = sqrt2;
        }

        const float& distance_to_obstacle = neighbor->distance;
        float traversal_cost = traversalCost(distance_to_neighbor, distance_to_obstacle);

        // new cost through parent. If better than the previous one
        // we canditate the neighbor for the expansion
        float new_cost = current_cost + traversal_cost;

        if (new_cost < neighbor->cost) {
          neighbor->parent = current;
          neighbor->cost   = new_cost;
          _queue.push(PathSearchEntry(neighbor->cost, neighbor));
        }
      }
    }
    // std::cerr << "I expanded: " << num_expansions << " cells" << std::endl;
    _search_param_changed_flag = false;
  }

} // namespace srrg2_core
