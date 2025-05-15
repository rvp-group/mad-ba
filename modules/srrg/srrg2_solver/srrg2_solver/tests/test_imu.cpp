#include <gtest/gtest.h>

#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/variables_and_factors/types_3d/all_types.h"
#include "srrg_solver/variables_and_factors/types_3d/instances.h"
#include <cmath>
#include <srrg_solver/solver_core/iteration_algorithm_dl.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

using namespace srrg2_core;
using namespace srrg2_solver;

const float _imu_freq = 300; // ldg assuming imu @ 300 Hz
using ImuMeasurements =
  std::vector<PreintegratedImuMeasurements, Eigen::aligned_allocator<PreintegratedImuMeasurements>>;

PreintegratedImuMeasurements generateImuMotion(std::vector<Isometry3f>& poses_,
                                               ImuMeasurements& imu_measurements_,
                                               const int& num_poses);

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(DUMMY_DATA, ImuFactor) {
  std::vector<Isometry3f> poses;
  ImuMeasurements measurements;
  // PreintegratedImuMeasurements closure = generateImuMotion(poses, measurements, 100);
  generateImuMotion(poses, measurements, 100);

  Solver solver;
  solver.param_termination_criteria.setValue(nullptr);
  solver.param_max_iterations.pushBack(30);
  IterationAlgorithmBasePtr alg(new IterationAlgorithmDL);
  solver.param_algorithm.setValue(alg);
  FactorGraphPtr graph(new FactorGraph);

  using VarPoseType   = VariableSE3QuaternionRightAD;
  using VarVectorType = VariablePoint3AD;
  using ImuBiasVar    = VariableImuBiasAD;
  using FactorType    = ImuErrorFactorAD;

  VarPoseType* prev_pose_var = new VarPoseType();
  prev_pose_var->setEstimate(Isometry3f::Identity());
  prev_pose_var->setGraphId(0);
  prev_pose_var->setStatus(VariableBase::Status::Fixed);
  graph->addVariable(VariableBasePtr(prev_pose_var));

  VarVectorType* prev_vel_var = new VarVectorType();
  prev_vel_var->setEstimate(Vector3f::Zero());
  prev_vel_var->setGraphId(1);
  prev_vel_var->setStatus(VariableBase::Status::Fixed);
  graph->addVariable(VariableBasePtr(prev_vel_var));

  ImuBiasVar* bias_prev = new ImuBiasVar();
  bias_prev->setGraphId(2);
  bias_prev->setEstimate(Vector6f::Zero());
  bias_prev->setStatus(VariableBase::Fixed);
  graph->addVariable(VariableBasePtr(bias_prev));

  size_t graph_id = 4;
  for (size_t i = 0; i < measurements.size(); ++i) {
    PreintegratedImuMeasurements imu_meas = measurements[i];
    Isometry3f T                          = poses[i + 1];
    VarPoseType* curr_pose_var = new VarPoseType();
    curr_pose_var->setEstimate(T);
    curr_pose_var->setGraphId(graph_id++);
    graph->addVariable(VariableBasePtr(curr_pose_var));

    VarVectorType* curr_vel_var = new VarVectorType();
    curr_vel_var->setGraphId(graph_id++);
    curr_vel_var->setEstimate(Vector3f::Zero());
    graph->addVariable(VariableBasePtr(curr_vel_var));

    ImuBiasVar* bias_curr = new ImuBiasVar();
    bias_curr->setGraphId(graph_id++);
    bias_curr->setEstimate(Vector6f::Zero());
    graph->addVariable(VariableBasePtr(bias_curr));

    FactorType* imu_factor = new FactorType();
    imu_factor->setVariableId(0, prev_pose_var->graphId());
    imu_factor->setVariableId(1, prev_vel_var->graphId());
    imu_factor->setVariableId(2, curr_pose_var->graphId());
    imu_factor->setVariableId(3, curr_vel_var->graphId());
    imu_factor->setVariableId(4, bias_prev->graphId());
    imu_factor->setVariableId(5, bias_curr->graphId());
    imu_factor->setMeasurement(imu_meas);
    // Eigen::Matrix<float, 15, 15> info_mat = Eigen::Matrix<float, 15, 15>::Identity();
    // info_mat.block<6, 6>(9, 9) *= 1e4;
    // imu_factor->setInformationMatrix(info_mat);
    graph->addFactor(FactorBasePtr(imu_factor));
    prev_pose_var = curr_pose_var;
    prev_vel_var  = curr_vel_var;
    bias_prev     = bias_curr;
  }

  // std::cerr << measurements[0].informationMatrix() << std::endl;
  // FactorType* odom_factor = new FactorType();
  // odom_factor->setVariableId(0, 0);
  // odom_factor->setVariableId(1, 1);
  // odom_factor->setVariableId(2, prev_pose_var->graphId());
  // odom_factor->setVariableId(3, prev_vel_var->graphId());
  // odom_factor->setVariableId(4, 2);
  // odom_factor->setVariableId(5, bias_prev->graphId());
  // bias_prev->setBiasAccelerometerEstimate(closure.biasAcc());
  // bias_prev->setBiasGyroscopeEstimate(closure.biasOmega());
  // bias_prev->setStatus(VariableBase::Fixed);
  // odom_factor->setMeasurement(closure);
  // graph->addFactor(FactorBasePtr(odom_factor));

  solver.setGraph(graph);
  solver.compute();
  std::cerr << solver.iterationStats() << std::endl;

  const auto& estimate_pose       = prev_pose_var->estimate();
  const auto& gt_pose             = poses.back();
  const auto diff_T               = estimate_pose.inverse() * gt_pose;
  const Matrix3f diff_rot         = diff_T.linear();
  const Vector3f diff_translation = diff_T.translation();
  const Vector3f diff_angles      = geometry3d::logMapSO3(diff_rot);
  const Vector3f bias_acc         = bias_prev->biasAccelerometer();
  const Vector3f bias_omega       = bias_prev->biasGyroscope();

  std::cerr << "last estimate: \n" << estimate_pose.matrix() << std::endl;
  std::cerr << "last gt pose: \n" << gt_pose.matrix() << std::endl;
  std::cerr << "error translation [m]: " << diff_T.translation().squaredNorm() << std::endl;
  std::cerr << "error rotation [rad]: " << diff_angles.squaredNorm() << std::endl;
  std::cerr << "bias acceleration [pertiche]: " << bias_acc.transpose() << std::endl;
  std::cerr << "bias gyro [pertiche]: " << bias_omega.transpose() << std::endl;

  ASSERT_LT(diff_translation.x(), 2e-4);
  ASSERT_LT(diff_translation.y(), 2e-4);
  ASSERT_LT(diff_translation.z(), 2e-4);
  ASSERT_LT(diff_angles.x(), 6e-4);
  ASSERT_LT(diff_angles.y(), 6e-4);
  ASSERT_LT(diff_angles.z(), 6e-4);
}

PreintegratedImuMeasurements generateImuMotion(std::vector<Isometry3f>& poses_,
                                               ImuMeasurements& imu_measurements_,
                                               const int& num_poses) {
  const float dt = 1.0 / _imu_freq;

  Vector3f a_0                = Vector3f(0.01f, 0.01f, 0.01f);
  Vector3f w_0                = Vector3f(0.1, 0.1, 0.1);
  Vector3f v                  = Vector3f(0.f, 0.f, 0.f);
  Vector3f ba                 = 1e-8 * Vector3f::Ones();
  Vector3f bg                 = 1e-8 * Vector3f::Ones();
  Matrix3f R                  = Matrix3f::Identity();
  Vector3f p                  = Vector3f::Zero();
  int integration_interval    = _imu_freq / 10;
  const float noise_acc       = 0.00175f;
  const float noise_gyro      = 0.00175f;
  const float noise_bias_acc  = 0.00167f;
  const float noise_bias_gyro = 0.00167f;
  poses_.resize(num_poses + 1);
  imu_measurements_.resize(num_poses);
  poses_.at(0).setIdentity();
  PreintegratedImuMeasurements closure;
  closure.setNoiseAccelerometer(Vector3f::Constant(noise_acc));
  closure.setNoiseGyroscope(Vector3f::Constant(noise_gyro));
  closure.setNoiseBiasAccelerometer(Vector3f::Constant(noise_bias_acc));
  closure.setNoiseBiasGyroscope(Vector3f::Constant(noise_bias_gyro));
  for (int i = 0; i < num_poses; ++i) {
    const float dt2 = dt * dt;
    PreintegratedImuMeasurements Z;
    Vector3f a            = a_0 + noise_acc * Vector3f::Random();
    Vector3f w            = w_0 + noise_gyro * Vector3f::Random();
    Vector3f biased_acc   = a + ba;
    Vector3f biased_omega = w + bg;
    Z.setNoiseAccelerometer(Vector3f::Constant(noise_acc));
    Z.setNoiseGyroscope(Vector3f::Constant(noise_gyro));
    Z.setBiasAcc(ba);
    Z.setBiasOmega(bg);
    Z.setNoiseBiasAccelerometer(Vector3f::Constant(noise_bias_acc));
    Z.setNoiseBiasGyroscope(Vector3f::Constant(noise_bias_gyro));
    for (int j = 0; j < integration_interval; ++j) {
      const Vector3f dtheta = w * dt;
      p += v * dt + 0.5f * R * a * dt2;
      v += R * a * dt;
      R *= geometry3d::expMapSO3(dtheta);
      fixRotation(R);
      Z.integrate(biased_acc, biased_omega, dt);
      closure.integrate(a, w, dt);
    }

    ba += integration_interval * noise_bias_acc * Vector3f::Random();
    bg += integration_interval * noise_bias_gyro * Vector3f::Random();
    poses_.at(i + 1)               = Isometry3f::Identity();
    poses_.at(i + 1).linear()      = R;
    poses_.at(i + 1).translation() = p;
    imu_measurements_.at(i)        = Z;
  }

  std::cerr << closure.covarianceMatrix() << std::endl;
  Matrix15f Sigma = closure.covarianceMatrix();
  Eigen::JacobiSVD<Matrix15f> svd(Sigma);
  std::cerr << FG_GREEN("Singular value to verify that is positive semi-definite : ")
            << svd.singularValues().transpose() << std::endl;
  closure.setBiasAcc(ba);
  closure.setBiasOmega(bg);
  return closure;
}
