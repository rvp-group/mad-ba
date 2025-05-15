#include "instances.h"
#include "constraint_factor_impl.cpp"
#include "factor_graph.h"
#include "iteration_algorithm_ddl.h"
#include "iteration_algorithm_dl.h"
#include "iteration_algorithm_gn.h"
#include "iteration_algorithm_gn_ls.h"
#include "iteration_algorithm_gn_pd.h"
#include "iteration_algorithm_lm.h"
#include "robustifier_policy.h"
#include "solver.h"
#include "solver_action_draw.h"
#include "termination_criteria.h"

namespace srrg2_solver {

  void solver_registerTypes() {
    BOSS_REGISTER_CLASS(RobustifierSaturated);
    BOSS_REGISTER_CLASS(RobustifierCauchy);
    BOSS_REGISTER_CLASS(RobustifierClamp);
    BOSS_REGISTER_CLASS(RobustifierHuber);
    BOSS_REGISTER_CLASS(RobustifierPolicyByType);
    BOSS_REGISTER_CLASS(Solver);
    BOSS_REGISTER_CLASS(SolverActionDraw);
    BOSS_REGISTER_CLASS(SimpleTerminationCriteria);
    BOSS_REGISTER_CLASS(PerturbationNormTerminationCriteria);
    BOSS_REGISTER_CLASS(PerturbationNormAndConstraintViolationTerminationCriteria);
    BOSS_REGISTER_CLASS(RelativeGradientChiTerminationCriteria);
    BOSS_REGISTER_CLASS(RelativeGradientChiAndConstraintViolationTerminationCriteria);
    BOSS_REGISTER_CLASS(IterationAlgorithmGN);
    BOSS_REGISTER_CLASS(IterationAlgorithmLM);
    BOSS_REGISTER_CLASS(IterationAlgorithmDL);
    BOSS_REGISTER_CLASS(IterationAlgorithmDDL);
    BOSS_REGISTER_CLASS(IterationAlgorithmGNLS);
    BOSS_REGISTER_CLASS(IterationAlgorithmGNPD);
    BOSS_REGISTER_CLASS(FactorGraph);
    BOSS_REGISTER_CLASS(SolverVerboseAction);
    BOSS_REGISTER_CLASS(SolverPreemptAction);
  }
} // namespace srrg2_solver
