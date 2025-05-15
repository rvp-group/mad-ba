#include "solver_action_base.h"
#include "solver.h"

namespace srrg2_solver {

  /*! @brief Base solver action interface, might be pre or post a solver iteration. In the derived
    class you need to override the doAction() method */
  SolverActionBase::~SolverActionBase(){}

  

  SolverVerboseAction::SolverVerboseAction() {
    param_event.setValue(Solver::SolverEvent::IterationEnd);
  }

  void SolverVerboseAction::doAction()  {
    std::cerr << _solver_ptr->lastIterationStats() << std::endl;
  }

  SolverPreemptAction::SolverPreemptAction() {
    param_event.setValue(Solver::SolverEvent::IterationEnd);
  }

  void SolverPreemptAction::doAction()  {
    _solver_ptr->preemptGlobal();
  }

} // namespace srrg2_solver
