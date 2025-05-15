#include "path_matrix_astar_search.h"

namespace srrg2_core {
  using namespace std;

  void PathMatrixAstarSearch::reset() {
    // ensure valid params
    if (!_path_map) {
      std::cerr << __PRETTY_FUNCTION__ << std::endl;
      throw std::runtime_error("[PathMatrixAstarSearch::reset] please set a path map");
    }
    const size_t& rows = _path_map->rows();
    const size_t& cols = _path_map->cols();
    if (rows < 3 || cols < 3) {
      std::cerr << "invalid dimensions" << std::endl;
      throw std::runtime_error(
        "[PathMatrixAstarSearch::reset] map is too small to compute a distance map");
    }
    for (size_t i = 0; i < _path_map->size(); ++i) {
      PathMatrixCell& cell = _path_map->at(i);
      cell.parent          = nullptr;
      cell.cost            = std::numeric_limits<float>::max();
    }
    _queue                 = PathSearchQueue();
    _path_map_changed_flag = false;
  }

  void PathMatrixAstarSearch::compute() {
    if (_path_map_changed_flag) {
      reset();
    }
    if (_queue.empty()) {
      std::cerr << __PRETTY_FUNCTION__ << ": Warning queue empty" << std::endl;
    }

    // bb priority is sum of traveled path and heuristic
    int num_expansions           = 0;
    int max_expansions           = 10000;
    static constexpr float sqrt2 = sqrt(2.f);
    PathMatrixCell* current      = nullptr;
    const int* neighbor_offsets  = _path_map->eightNeighborOffsets();
    _status = PathMatrixCostSearch::SearchStatus::GoalNotFound;
    while (!_queue.empty() && _status == PathMatrixCostSearch::SearchStatus::GoalNotFound &&
           num_expansions < max_expansions) {
      PathSearchEntry entry = _queue.top();
      current               = entry.cell;
      _queue.pop();
      Vector2i current_pos = _path_map->pos(current);
      const auto& it       = std::find(_goals.begin(), _goals.end(), current_pos);
      if (it != _goals.end()) {
        _status = PathMatrixCostSearch::SearchStatus::GoalFound;
        break;
      }

      for (int i = 0; i < 8; ++i) {
        auto neighbor         = current + neighbor_offsets[i];
        Vector2i neighbor_pos = _path_map->pos(neighbor);
        int d2                = (neighbor_pos - current_pos).squaredNorm();
        float linear_distance = 1;
        if (d2 > 1) {
          linear_distance = sqrt2;
        }
        float arc_cost           = traversalCost(linear_distance, neighbor->distance);
        float cost_till_neighbor = current->cost + arc_cost;
        if (cost_till_neighbor < neighbor->cost) {
          neighbor->cost = cost_till_neighbor;

          neighbor->parent    = current;
          float expected_cost = neighbor->cost + neighbor->heuristic;
          _queue.push(PathSearchEntry(expected_cost, neighbor));
        }
      }
      ++num_expansions;
    }
    _path_pxl.clear();
    _cost_to_global_goal = -1;
    if (_status == PathMatrixCostSearch::SearchStatus::GoalNotFound) {
      return;
    }
    _cost_to_global_goal  = current->heuristic + current->cost;
    float delta_heuristic = _initial_heuristic - current->heuristic;
    if (delta_heuristic < 0) {
      cerr << "**********************************************************" << endl;
      cerr << "DELTA H<0 (" << delta_heuristic << ") ! NEED GLOBAL REPLAN" << endl;
      cerr << "**********************************************************" << endl;
      _status = PathMatrixCostSearch::SearchStatus::HeuristicMismatch;
      return;
    }

    _goal     = _path_map->pos(current->parent->parent);
    auto cell = current;
    while (cell != cell->parent) {
      Vector2i pos = _path_map->pos(cell);
      _path_pxl.push_back(pos);
      cell = cell->parent;
    }
    std::reverse(_path_pxl.begin(), _path_pxl.end());
    // cerr << "astar path_size: " << _local_path_pxl.size() << endl;
    _search_param_changed_flag = false;
  }

} // namespace srrg2_core
