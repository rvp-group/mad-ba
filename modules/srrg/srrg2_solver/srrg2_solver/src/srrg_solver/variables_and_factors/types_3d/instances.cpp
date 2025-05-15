//! include all the types you declared here
#include "instances.h"
#include "all_types.h"

//! we try to instantiate all solvers
namespace srrg2_solver {
  using namespace srrg2_core;

  // this is the function you have to call to initialize
  // the serialization subsystem
  void variables_and_factors_3d_registerTypes() {
    BOSS_REGISTER_CLASS(VariableMatchable);
    BOSS_REGISTER_CLASS(VariableSE3QuaternionRight);
    BOSS_REGISTER_CLASS(VariableSE3QuaternionRightAD);
    BOSS_REGISTER_CLASS(VariableSE3QuaternionLeft);
    BOSS_REGISTER_CLASS(VariableSE3QuaternionLeftAD);
    BOSS_REGISTER_CLASS(VariableSE3EulerRight);
    BOSS_REGISTER_CLASS(VariableSE3EulerRightAD);
    BOSS_REGISTER_CLASS(VariableSE3EulerLeft);
    BOSS_REGISTER_CLASS(VariableSE3EulerLeftAD);
    BOSS_REGISTER_CLASS(SE3PoseMotionErrorFactorDataDriven);

    // ia pose pose
    BOSS_REGISTER_CLASS(SE3PosePoseChordalEulerLeftErrorFactor);
    BOSS_REGISTER_CLASS(SE3PosePoseChordalHessianFactor);
    BOSS_REGISTER_CLASS(SE3PosePoseGeodesicErrorFactor);

    // ia pose point
    BOSS_REGISTER_CLASS(SE3PosePointOffsetErrorFactor);
    BOSS_REGISTER_CLASS(SE3PosePointErrorFactor);

    // // ia pose matchables
    BOSS_REGISTER_CLASS(SE3PoseMatchableEulerLeftErrorFactor);
    BOSS_REGISTER_CLASS(SE3Matchable2MatchableEulerLeftErrorFactor);
    BOSS_REGISTER_CLASS(SE3Matchable2MatchableEulerLeftErrorFactorCorrespondenceDriven);

    // // many factors
    BOSS_REGISTER_CLASS(SE3Plane2PlaneErrorFactor);
    BOSS_REGISTER_CLASS(SE3Plane2PlaneErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE3Plane2PlaneWithSensorErrorFactor);
    BOSS_REGISTER_CLASS(SE3Plane2PlaneWithSensorErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE3Point2PointErrorFactor);
    BOSS_REGISTER_CLASS(SE3Point2PointErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE3Point2PointWithSensorErrorFactor);
    BOSS_REGISTER_CLASS(SE3Point2PointWithSensorErrorFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE3PriorErrorFactorAD);
    BOSS_REGISTER_CLASS(SE3PriorOffsetErrorFactorAD);

    BOSS_REGISTER_CLASS(ImuErrorFactorAD);
    BOSS_REGISTER_CLASS(GpsErrorFactorAD);
    BOSS_REGISTER_CLASS(VariableImuBiasAD);
  }
} // namespace srrg2_solver
