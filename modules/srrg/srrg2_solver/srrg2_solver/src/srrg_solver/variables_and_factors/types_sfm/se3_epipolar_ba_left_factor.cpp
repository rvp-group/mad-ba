#include "se3_epipolar_ba_left_factor.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_geometry/geometry3d.h"
#include "se3_epipolar_ba_left_factor_impl.cpp"
#include "se3_epipolar_ba_factor_jacobian_helpers.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;

  
  void SE3EpipolarBAEulerLeftFactor::resetCompute(bool chi_only) {
    if (! _resetCompute(chi_only))
      return;
    _Je=essentialLeftJacobian(variables().at<0>()->estimate(),
                              variables().at<1>()->estimate(),
                              essentialEulerLeft);
  }

  void SE3EpipolarBAQuaternionLeftFactor::resetCompute(bool chi_only) {
    if (! _resetCompute(chi_only))
      return;
    _Je=essentialLeftJacobian(variables().at<0>()->estimate(),
                              variables().at<1>()->estimate(),
                              essentialQuatLeft);
  }

  

} // namespace srrg2_solver

