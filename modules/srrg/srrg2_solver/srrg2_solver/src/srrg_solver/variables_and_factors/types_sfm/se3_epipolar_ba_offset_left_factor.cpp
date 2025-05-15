#include "se3_epipolar_ba_offset_left_factor.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_geometry/geometry3d.h"
#include "se3_epipolar_ba_offset_left_factor_impl.cpp"
#include "se3_epipolar_ba_factor_jacobian_helpers.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;

  
  void SE3EpipolarBAOffsetEulerLeftFactor::resetCompute(bool chi_only) {
    if (! _resetCompute(chi_only))
      return;
    Isometry3f X_from=variables().at<0>()->estimate()*variables().at<2>()->estimate();
    Isometry3f X_to=variables().at<1>()->estimate()*variables().at<3>()->estimate();
    _Je=essentialLeftJacobian(X_from, X_to, essentialEulerLeft);
  }

  void SE3EpipolarBAOffsetQuaternionLeftFactor::resetCompute(bool chi_only) {
    if (! _resetCompute(chi_only))
      return;
    Isometry3f X_from=variables().at<0>()->estimate()*variables().at<2>()->estimate();
    Isometry3f X_to=variables().at<1>()->estimate()*variables().at<3>()->estimate();
    _Je=essentialLeftJacobian(X_from, X_to, essentialQuatLeft);
  }

  

} // namespace srrg2_solver

