//! include all the types you declared here
#include "instances.h"
#include "variable_surfel.h"
#include "variable_surfel_ad.h"
#include "se3_pose_surfel_factor_ad.h"
//! we try to instantiate all solvers
namespace srrg2_solver {
  using namespace srrg2_core;

  // this is the function you have to call to initialize
  // the serialization subsystem
  void variables_and_factors_types_registerTypes() {
    BOSS_REGISTER_CLASS(VariableSurfel);
    BOSS_REGISTER_CLASS(VariableSurfel1D);
    BOSS_REGISTER_CLASS(VariableSurfelAD);
    BOSS_REGISTER_CLASS(VariableSurfelAD1D);
    BOSS_REGISTER_CLASS(SE3PoseSurfelQuaternionErrorFactorAD);
    BOSS_REGISTER_CLASS(SE3PoseSurfelQuaternionErrorFactorAD1D);
  }
} // namespace srrg2_solver
