#pragma once
#include "path_matrix_search_base.h"
#include "srrg_pcl/point_types.h"
#include "srrg_property/property.h"

namespace srrg2_core {

  class PathMatrixDistanceSearch : public PathMatrixSearchBase {
  public:
    PARAM(PropertyFloat,
          max_distance_squared_pxl,
          "squared distance [pixel^2] to stop when performing a search",
          1.0f,
          &this->_path_map_changed_flag);

    virtual void compute();

    // resets the calculation of the distance map to the values stored in the path map
    void reset();

    // performs an addittive estimation of the points in a distance map
    template <typename PointContainerType_>
    inline void setGoals(PointContainerType_& points, bool clear = true) {
      if (_path_map_changed_flag) {
        reset();
      }
      if (clear) {
        _path_map->fill(
          PathMatrixCell(param_max_distance_squared_pxl.value(), 0.0f, 0.0f, nullptr));
      }
      // TODO manage add flag

      using PointType_ = typename PointContainerType_::value_type;
      for (std::size_t idx = 0; idx < points.size(); ++idx) {
        const PointType_& p = points[idx];
        Eigen::Vector2i ip(p.coordinates().x(), p.coordinates().y());
        if (!_path_map->inside(ip)) {
          // std::cout << "point outside _path_map: " << ip.transpose() << std::endl;
          continue;
        }
        _goal_map(ip)        = (int) idx;
        PathMatrixCell& cell = (*_path_map)(ip);
        if (cell.distance<0)
          continue;
        cell                 = PathMatrixCell(0.0f, 1.0f, 0.0f, &cell);
        _queue.push(PathSearchEntry(0.f, &cell));
      }
      _path_map_changed_flag = false;
    }

    // returns the original indices image
    const MatrixInt& goalsMap() const {
      return _goal_map;
    }

    // returns the indices image wth the neighbohrs
    const MatrixInt& parentMap();

  protected:
    PathSearchQueue _queue;
    MatrixInt _goal_map;
    MatrixInt _parent_map;
    bool _parent_map_changed_flag = true;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using PathMatrixDistanceSearchPtr = std::shared_ptr<PathMatrixDistanceSearch>;
} // namespace srrg2_core
