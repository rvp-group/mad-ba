#include "instances.h"
#include "all_types.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  void variables_and_factors_sfm_registerTypes() {
    BOSS_REGISTER_CLASS(VariableSO3Relaxed);
    BOSS_REGISTER_CLASS(VariableSO3RelaxedAD);
    BOSS_REGISTER_CLASS(VariableSO3QuaternionRight);
    BOSS_REGISTER_CLASS(VariableSO3QuaternionRightAD);
    BOSS_REGISTER_CLASS(SO3RelaxedPosePoseErrorFactor);
    BOSS_REGISTER_CLASS(SO3PosePoseQuaternionRightErrorFactorAD);
    BOSS_REGISTER_CLASS(SO3RelaxedPoseConstraintFactor);
    BOSS_REGISTER_CLASS(SO3RelaxedPoseConstraintFactorAD);

    BOSS_REGISTER_CLASS(SE3EpipolarBAEulerLeftFactor);
    BOSS_REGISTER_CLASS(SE3EpipolarBAEulerLeftFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE3EpipolarBAEulerRightFactor);
    BOSS_REGISTER_CLASS(SE3EpipolarBAEulerRightFactorCorrespondenceDriven);

    BOSS_REGISTER_CLASS(SE3EpipolarBAQuaternionLeftFactor);
    BOSS_REGISTER_CLASS(SE3EpipolarBAQuaternionLeftFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE3EpipolarBAQuaternionRightFactor);
    BOSS_REGISTER_CLASS(SE3EpipolarBAQuaternionRightFactorCorrespondenceDriven);

    BOSS_REGISTER_CLASS(SE3EpipolarBAOffsetEulerLeftFactor);
    BOSS_REGISTER_CLASS(SE3EpipolarBAOffsetEulerLeftFactorCorrespondenceDriven);
    BOSS_REGISTER_CLASS(SE3EpipolarBAOffsetQuaternionLeftFactor);
    BOSS_REGISTER_CLASS(SE3EpipolarBAOffsetQuaternionLeftFactorCorrespondenceDriven);

    BOSS_REGISTER_CLASS(SE3PairwiseScaleErrorFactorEulerLeftAD);
    BOSS_REGISTER_CLASS(SE3PairwiseScaleErrorFactorEulerRightAD);
    BOSS_REGISTER_CLASS(SE3PairwiseScaleErrorFactorQuaternionLeftAD);
    BOSS_REGISTER_CLASS(SE3PairwiseScaleErrorFactorQuaternionRightAD);
    // BOSS_REGISTER_CLASS(SE3EpipolarTranslationFactor);
    BOSS_REGISTER_CLASS(EssentialTranslationFactor);
  }

} // namespace srrg2_solver
