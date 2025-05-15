//! include all the types you declared here
#include "instances.h"
#include "all_types.h"

//! we try to instantiate all solvers
namespace srrg2_solver {
  using namespace srrg2_core;

  // this is the function you have to call to initialize
  // the serialization subsystem
  void variables_and_factors_2d_registerTypes() {
    BOSS_REGISTER_CLASS(VariableSE2Right);
    BOSS_REGISTER_CLASS(VariableSE2RightAD);
    BOSS_REGISTER_CLASS(SE2PosePoseGeodesicErrorFactor);
    BOSS_REGISTER_CLASS(SE2PosePoseChordalErrorFactor);
    BOSS_REGISTER_CLASS(SE2PosePointErrorFactor);
    BOSS_REGISTER_CLASS(SE2CircumferenceConstraintFactor);

    BOSS_REGISTER_CLASS(SE2PosePointBearingErrorFactor);
    BOSS_REGISTER_CLASS(SE2PriorErrorFactor);
    BOSS_REGISTER_CLASS(SE2Point2PointErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE2Plane2PlaneErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE2Point2PointWithSensorErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE2Plane2PlaneWithSensorErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE2PosePointRangeConstraintFactor);
  }
} // namespace srrg2_solver
