#include "se3_epipolar_ba_right_factor_impl.cpp"
#include "se3_epipolar_ba_factor_jacobian_helpers.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;

  void SE3EpipolarBAEulerRightFactor::resetCompute(bool chi_only) {
    SE3EpipolarBARightFactor_<VariableSE3EulerRight>::resetCompute(chi_only);
    if (! _resetCompute(chi_only))
      return;
    essentialJacobiansRight(_Jei, _Jej,
                            this->variables().template at<0>()->estimate(),
                            this->variables().template at<1>()->estimate(),
                            Eigen::Isometry3f::Identity(),
                            Eigen::Isometry3f::Identity(),
                            essentialEulerRight);
  }

  void SE3EpipolarBAQuaternionRightFactor::resetCompute(bool chi_only) {
    SE3EpipolarBARightFactor_<VariableSE3QuaternionRight>::resetCompute(chi_only);
    if (! _resetCompute(chi_only))
      return;
    
    essentialJacobiansRight(_Jei, _Jej,
                            this->variables().template at<0>()->estimate(),
                            this->variables().template at<1>()->estimate(),
                            Eigen::Isometry3f::Identity(),
                            Eigen::Isometry3f::Identity(),
                            essentialQuaternionRight);
  }

} // namespace srrg2_solver

