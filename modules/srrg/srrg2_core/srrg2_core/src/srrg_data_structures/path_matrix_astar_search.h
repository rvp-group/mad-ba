#pragma once
#include "path_matrix_cost_search.h"
#include "path_matrix_dijkstra_search.h"
#include "path_matrix_search_base.h"
#include "srrg_pcl/point_types.h"
#include "srrg_property/property.h"

namespace srrg2_core {
  class PathMatrixAstarSearch : public PathMatrixCostSearch {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void compute();

    // resets the calculation of the distance map to the values stored in the path map
    void reset();
    // performs an additive estimation of the points in a distance map
    template <typename PointContainerType_>
    inline int setGoal(PointContainerType_& points) {
      // TODO manage add flag
      int num_goals = 0;
      _goals.clear();
      using PointType_ = typename PointContainerType_::value_type;
      for (std::size_t idx = 0; idx < points.size(); ++idx) {
        const PointType_& p = points[idx];
        Eigen::Vector2i ip(p.coordinates().x(), p.coordinates().y());
        if (!_path_map->inside(ip)) {
          continue;
        }
        _goals.emplace_back(ip);
        ++num_goals;
      }
      _path_map_changed_flag = false;
      return num_goals;
    }

    inline bool setStart(const Vector2i& start_) {
      if (_path_map_changed_flag) {
        reset();
      }
      if (!_path_map->inside(start_)) {
        std::cerr << "point outside _path_map: " << start_.transpose() << std::endl;
        return false;
      }
      _initial_heuristic   = _path_map->at(start_).heuristic;
      PathMatrixCell& cell = _path_map->at(start_);
      if (cell.distance < 0) {
        return false;
      }
      cell = PathMatrixCell(cell.distance, 0.0f, cell.cost, &cell);
      _queue.push(PathSearchEntry(cell.heuristic, &cell));
      _path_map_changed_flag     = false;
      _search_param_changed_flag = true;
      return true;
    }

    const StdVectorEigenVector2i& pathPxl() const {
      return _path_pxl;
    }

    PathMatrixCostSearch::SearchStatus status() const {
      return _status;
    }

    float costToGlobalGoal() const {
      return _cost_to_global_goal;
    }

    const Vector2i& goal() const {
      return _goal;
    }

  protected:
    PathSearchQueue _queue;
    StdVectorEigenVector2i _path_pxl;
    PathMatrixCostSearch::SearchStatus _status = PathMatrixCostSearch::SearchStatus::GoalNotFound;
    bool _search_param_changed_flag            = true;
    float _cost_to_global_goal                 = -1;
    Vector2i _goal                             = Vector2i::Zero();
    float _initial_heuristic                   = 0;
    StdVectorEigenVector2i _goals;
  }; // namespace srrg2_core

  using PathMatrixAstarSearchPtr = std::shared_ptr<PathMatrixAstarSearch>;
} // namespace srrg2_core
