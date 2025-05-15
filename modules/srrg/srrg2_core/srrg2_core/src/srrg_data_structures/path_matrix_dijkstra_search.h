#pragma once

#include "path_matrix_cost_search.h"
#include "path_matrix_search_base.h"
#include <srrg_pcl/point_types.h>
#include <srrg_property/property.h>
#include <srrg_property/property_vector.h>
#include <stdexcept>

namespace srrg2_core {
  class PathMatrixDijkstraSearch : public PathMatrixCostSearch {
  public:
    PARAM(PropertyFloat, max_cost, "when to stop the search", 1e6, &this->_parent_map_changed_flag);

    virtual void compute();

    // resets the calculation of the minimum cost map to the values stored in the path map
    void reset();

    // sets the source from where to compute the minimal paths
    // might be more than one
    template <typename PointContainerType_>
    inline int setGoals(PointContainerType_& points,
                        bool add                              = false,
                        const std::vector<float>& goal_costs_ = std::vector<float>()) {
      if (_path_map_changed_flag) {
        reset();
      }
      using PointType_   = typename PointContainerType_::value_type;
      int num_good_goals = 0;
      if (!goal_costs_.empty() && goal_costs_.size() != points.size()) {
        throw std::runtime_error("not usable!!!");
      }
      for (std::size_t idx = 0; idx < points.size(); ++idx) {
        const PointType_& p = points[idx];
        Eigen::Vector2i ip(p.coordinates().x(), p.coordinates().y());
        if (!_path_map->inside(ip)) {
          std::cerr << "candidate point outside the map" << std::endl;
          continue;
        }
        PathMatrixCell& cell = _path_map->at(ip);
        if (cell.distance < param_min_distance.value()) {
          std::cerr << "cell.distance < param_min_distance.value" << std::endl;
          std::cerr << "ip: " << ip.transpose() << std::endl;
          std::cerr << "cell.distance: " << cell.distance << std::endl;
          std::cerr << "param_min_distance: " << param_min_distance.value() << std::endl;
          continue;
        }
        cell.cost   = goal_costs_.empty() ? 0.f : goal_costs_[idx];
        cell.parent = &cell;
        _queue.push(PathSearchEntry(cell.cost, &cell));
        ++num_good_goals;
      }
      _search_param_changed_flag = true;
      return num_good_goals;
    }

  protected:
    PathSearchQueue _queue;
    bool _parent_map_changed_flag   = true;
    bool _search_param_changed_flag = true;
    float _max_value_distance       = 2.0f * sqrt(2);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using PathMatrixDijkstraSearchPtr = std::shared_ptr<PathMatrixDijkstraSearch>;

} // namespace srrg2_core
