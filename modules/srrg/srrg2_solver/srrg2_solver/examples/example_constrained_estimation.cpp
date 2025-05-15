#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/iteration_algorithm_gn.h"
#include "srrg_solver/solver_core/solver.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_circumference_constraint_factor.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_prior_error_factor.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_point2.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_se2.h"
#include <random>
#include <srrg_system_utils/shell_colors.h>

using namespace srrg2_core;
using namespace srrg2_solver;

// Executable name and log
const std::string exe_name = "example_se2_constrained_estimation";
#define LOG std::cerr << exe_name + "|"

// Define types to simplify the code
using VariablePoseType = VariableSE2Right;

// Generate synthetic world and fill the factor graph
class Problem {
public:
  void generateProblem();
  void generateFactorGraph(FactorGraphPtr& graph, bool constrained_ = false);
  Isometry2f xGt() {
    return x_gt;
  }
  void printError(FactorGraphPtr& graph) {
    for (const auto& var : graph->variables()) {
      VariablePoseType* v = dynamic_cast<VariablePoseType*>(var.second);
      if (v) {
        Isometry2f x_est = v->estimate();
        std::cerr << "x: " << geometry2d::t2v(x_est).transpose() << std::endl;
        std::cerr << FG_BMAGENTA("error: ") << geometry2d::t2v(x_est.inverse() * x_gt).norm()
                  << std::endl;
      }
    }
  }

private:
  void transitionModel(Isometry2f& x_,
                       const Vector2f& u_,
                       float dt_,
                       Matrix3f& sigma_x,
                       Matrix2f& sigma_u) {
    Vector3f x_vec = geometry2d::t2v(x_);
    float x        = x_vec.x();
    float y        = x_vec.y();
    float theta    = x_vec.z();
    float v        = u_.x();
    float omega    = u_.y();
    sigma_u(0, 0) += v * v;
    sigma_u(1, 1) += omega * omega;
    x_vec << x + v * dt_ * cos(theta), y + v * dt_ * sin(theta), theta + omega * dt_;
    x_         = geometry2d::v2t(x_vec);
    Matrix3f A = Matrix3f::Identity();
    A << 1, 0, -v * sin(theta), 0, 1, v * cos(theta), 0, 0, 1;
    Matrix3_2f B = Matrix3_2f::Identity();
    B << cos(theta), 0, sin(theta), 0, 0, 1;
    sigma_x = A * sigma_x * A.transpose() + B * sigma_u * B.transpose();
  }

  void transitionModel(Isometry2f& x_, const Vector2f& u_, float dt_) {
    Vector3f x_vec = geometry2d::t2v(x_);
    float x        = x_vec.x();
    float y        = x_vec.y();
    float theta    = x_vec.z();
    float v        = u_.x();
    float omega    = u_.y();
    x_vec << x + v * dt_ * cos(theta), y + v * dt_ * sin(theta), theta + omega * dt_;
    x_ = geometry2d::v2t(x_vec);
  }
  Isometry2f x         = Isometry2f::Identity();
  Isometry2f x_gt      = Isometry2f::Identity();
  Isometry2f x_gps     = Isometry2f::Identity();
  Isometry2f x_initial = Isometry2f::Identity();
  Matrix3f sigma_x     = Matrix3f::Zero();
  Matrix3f sigma_gps   = Matrix3f::Zero();
};

int main(int argc, char** argv) {
  // Instanciate a factor graph
  Problem problem;
  problem.generateProblem();
  FactorGraphPtr graph(new FactorGraph);
  // Fill the graph
  problem.generateFactorGraph(graph);
  // Instanciate a solver
  Solver solver;
  // Configure the solver - Set max iterations, termination criteria and algorithm
  size_t num_iterations = 50;

  solver.param_max_iterations.pushBack(num_iterations);
  std::shared_ptr<PerturbationNormTerminationCriteria> termination(
    new PerturbationNormTerminationCriteria);
  solver.param_termination_criteria.setValue(termination);
  std::shared_ptr<IterationAlgorithmGN> gn_algorithm(new IterationAlgorithmGN);
  solver.param_algorithm.setValue(gn_algorithm);
  // Connect graph to the solver and compute
  solver.setGraph(graph);
  solver.compute();
  // Visualize statistics and exit
  auto stats = solver.iterationStats();
  std::cerr << stats << std::endl;
  problem.printError(graph);

  // Instanciate a factor graph
  FactorGraphPtr constrained_graph(new FactorGraph);
  // Fill the graph
  problem.generateFactorGraph(constrained_graph, true);
  std::shared_ptr<PerturbationNormAndConstraintViolationTerminationCriteria>
    constrained_termination(new PerturbationNormAndConstraintViolationTerminationCriteria);
  constrained_termination->param_epsilon_constraint.setValue(1e-5f);
  solver.param_termination_criteria.setValue(constrained_termination);
  solver.setGraph(constrained_graph);
  solver.compute();
  // Visualize statistics and exit
  stats = solver.iterationStats();
  stats = solver.iterationStats();
  std::cerr << stats << std::endl;
  problem.printError(constrained_graph);
  return 0;
}

void Problem::generateProblem() {
  x                   = Isometry2f::Identity();
  float theta_initial = M_PI / 10.f;
  x.linear()          = geometry2d::a2r(theta_initial);
  x_initial           = x;

  Matrix2f sigma_u = Matrix2f::Identity();
  sigma_x          = Matrix3f::Identity();
  sigma_u *= 0.1f;
  sigma_x *= 0.001f;
  Vector2f u(1.f, 0.f);
  float dt = 1.f;
  transitionModel(x, u, dt, sigma_x, sigma_u);

  transitionModel(x_gt, u, dt);
  Vector3f x_gt_vec = geometry2d::t2v(x_gt);
  sigma_gps         = Matrix3f::Identity();
  sigma_gps *= 0.001;

  std::random_device rd;
  std::mt19937 gen(rd());
  Vector3f x_gps_vec = x_gt_vec;
  for (int i = 0; i < x_gt_vec.size(); ++i) {
    std::normal_distribution<float> d(x_gt_vec(i), sqrt(sigma_gps(i, i)));
    x_gps_vec(i) = d(gen);
  }
  x_gps = geometry2d::v2t(x_gps_vec);
}

void Problem::generateFactorGraph(FactorGraphPtr& graph, bool constrained_) {
  float r             = 1.f;
  VariablePoseType* v = new VariablePoseType;
  // v2t is a function that maps a 3D vector to an element of SE2 - See srrg2_core
  cerr << "x_initial: " << geometry2d::t2v(x_initial).transpose() << endl;
  v->setEstimate(x_initial);
  graph->addVariable(VariableBasePtr(v));

  std::shared_ptr<SE2PriorErrorFactor> odometry_factor(new SE2PriorErrorFactor);
  odometry_factor->setVariableId(0, 0);
  odometry_factor->setMeasurement(x);
  odometry_factor->setInformationMatrix(sigma_x.inverse());
  cerr << "x: " << geometry2d::t2v(x).transpose() << endl;
  graph->addFactor(FactorBasePtr(odometry_factor));

  std::shared_ptr<SE2PriorErrorFactor> gps_factor(new SE2PriorErrorFactor);
  gps_factor->setVariableId(0, 0);
  gps_factor->setMeasurement(x_gps);
  gps_factor->setInformationMatrix(sigma_gps.inverse());
  cerr << "x_gps: " << geometry2d::t2v(x_gps).transpose() << endl;
  graph->addFactor(FactorBasePtr(gps_factor));

  if (constrained_) {
    std::shared_ptr<SE2CircumferenceConstraintFactor> circumference_factor(
      new SE2CircumferenceConstraintFactor);
    circumference_factor->setVariableId(0, 0);
    circumference_factor->setRadius(r);
    SE2CircumferenceConstraintFactor::ConstraintVectorType multiplier_initialization =
      SE2CircumferenceConstraintFactor::ConstraintVectorType::Identity();
    multiplier_initialization *= 1.f;
    circumference_factor->setMultiplierInitalization(multiplier_initialization);
    circumference_factor->setRho(1000.f, 5000.f, 100.f);
    circumference_factor->_constraint_type = srrg2_solver::Equality;
    graph->addFactor(FactorBasePtr(circumference_factor));
  }
}
