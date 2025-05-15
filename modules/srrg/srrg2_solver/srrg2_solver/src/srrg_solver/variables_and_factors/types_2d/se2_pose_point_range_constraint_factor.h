#pragma once
#include <srrg_geometry/geometry2d.h>

#include "srrg_solver/solver_core/constraint_factor.h"
#include "variable_point2.h"
#include "variable_se2.h"

namespace srrg2_solver {

  /** @brief 2D Range only factor. Estimate both the pose of the robot and the position of the 2D
   * landmark.
   */
  class SE2PosePointRangeConstraintFactor
    : public ConstraintFactor_<1, VariableSE2Right, VariablePoint2> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void setMinRange(const float& min_range_) {
      _min_range = min_range_;
    }
    void constraintAndJacobian(bool error_only_ = false) final;

    void serialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) override {
      BaseType::serialize(odata, context);
      odata.setFloat("min_range", _min_range);
    }

    void deserialize(srrg2_core::ObjectData& odata, srrg2_core::IdContext& context) override {
      BaseType::deserialize(odata, context);
      const float min_range_ = odata.getFloat("min_range");
      setMinRange(min_range_);
    }

  protected:
    float _min_range = 0.f;
  };

  using SE2PosePointRangeConstraintFactorPtr = std::shared_ptr<SE2PosePointRangeConstraintFactor>;

} // namespace srrg2_solver
