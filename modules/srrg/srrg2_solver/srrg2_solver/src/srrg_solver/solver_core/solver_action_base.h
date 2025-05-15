#pragma once
#include <memory>
#include <set>
#include <vector>
#include <srrg_system_utils/system_utils.h>
#include <srrg_config/property_configurable.h>
#include <srrg_config/configurable.h>

namespace srrg2_solver {
  using namespace srrg2_core;
  
  class Solver;
  /*! @brief Base solver action interface, might be pre or post a solver iteration. In the derived
    class you need to override the doAction() method */
  class SolverActionBase: public Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(PropertyInt, event, "event to which react", -1, 0);
    PARAM(PropertyInt, priority, "priority of the event, higher is 0", 0, 0);
    
    virtual void init(Solver& solver_) {
      _solver_ptr=&solver_;
    }
    
    virtual ~SolverActionBase();

    /*! Perform the action */
    virtual void doAction() = 0;

  protected:
    Solver* _solver_ptr = nullptr; /*!< Pointer to the solver at which the action is assigned */
  };

  using SolverActionBasePtr =
    std::shared_ptr<SolverActionBase>; /*!< Shared pointer to SolverActionBase */
  using SolverActionBasePtrVector = std::vector<SolverActionBasePtr>;
  using SolverActionBasePtrSet    = std::set<SolverActionBasePtr>;

  
  // ia talking action
  class SolverVerboseAction : public SolverActionBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SolverVerboseAction();

    void doAction() override;
    
  };

  using SolverVerboseActionPtr = std::shared_ptr<SolverVerboseAction>;

  // action to install preemption points for interactive use
  class SolverPreemptAction : public SolverActionBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SolverPreemptAction();

    void doAction() override;
    
  };

  using SolverPreemptActionPtr = std::shared_ptr<SolverPreemptAction>;

} // namespace srrg2_solver
