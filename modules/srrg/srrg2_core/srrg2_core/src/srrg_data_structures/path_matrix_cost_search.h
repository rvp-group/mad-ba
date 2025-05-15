#pragma once
#include "path_matrix_search_base.h"
#include <srrg_property/property.h>
#include <srrg_property/property_vector.h>

namespace srrg2_core {
  class PathMatrixCostSearch : public PathMatrixSearchBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(PropertyFloat,
          min_distance,
          "min distance from an obstacle",
          3.f,
          &this->_search_param_changed_flag);

    PARAM(PropertyVector_<float>,
          cost_polynomial,
          "cost=p0*distance_to_next + p1*d + p2*d^2 + ... pn*d^n",
          std::vector<float>(),
          &this->_search_param_changed_flag);
    enum SearchStatus { GoalFound = 1, GoalNotFound = -1, HeuristicMismatch = -2 };
    float traversalCost(float distance_to_next, float distance_to_obstacle) const;

  protected:
    bool _search_param_changed_flag = true;
  };
} // namespace srrg2_core
