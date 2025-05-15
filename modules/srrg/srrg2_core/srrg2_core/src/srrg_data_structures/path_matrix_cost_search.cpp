#include "path_matrix_cost_search.h"

namespace srrg2_core {
  float PathMatrixCostSearch::traversalCost(float distance_to_next,
                                            float distance_to_obstacle) const {
    if (distance_to_obstacle <= param_min_distance.value()) {
      return std::numeric_limits<float>::max();
    }
    //    const float distance_to_obstacle_modified = _max_value_distance - distance_to_obstacle;
    const float distance_to_obstacle_modified = distance_to_obstacle - param_min_distance.value();
    if (distance_to_obstacle_modified < 0) {
      return std::numeric_limits<float>::max();
    }

    float c = 0;
    for (size_t i = 0; i < param_cost_polynomial.size(); ++i) {
      float coeff = param_cost_polynomial.value(i);
      if (i == 0) {
        c += coeff * distance_to_next;
      } else {
        c += coeff * 1 / pow(distance_to_obstacle_modified, i);
      }
    }
    if (c < 0) {
      throw std::runtime_error("error, the cost must be positive");
    }
    return c;
  }
} // namespace srrg2_core
