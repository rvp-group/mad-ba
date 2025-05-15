#pragma once
#include <srrg_config/configurable.h>
#include <srrg_property/property.h>

#include "solver_stats.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  class Solver;
  /*! @brief Base termination criteria interface, do nothing.
    In the derived class you must override the hasToStop() method */
  class TerminationCriteria : public Configurable {
  public:
    /*! @return false if optimization have to stop */
    virtual bool hasToStop() const {
      return false;
    }
    /*! Set the pointer to the solver at which the termination criteria is attached
     @param[in] solver_ pointer to the solver
     */
    void setSolver(const Solver* solver_) {
      _solver = solver_;
    }

  protected:
    /*! Call the corresponding function in Solver */
    void getRHS(std::vector<float>& b_) const;
    /*! Call the corresponding function in Solver */
    void getPerturbation(std::vector<float>& dx_) const;
    const Solver* _solver = 0; /*!<Pointer to the solver
                                     at which the termination criteria is
                                     attached */
  };

  using TerminationCriteriaPtr = std::shared_ptr<TerminationCriteria>;

  /*! @brief Termination criteria that monitors the evolution of chi2
   and number of outliers/inliers
   inliers/outliers should be stable
   their errors as well */
  class SimpleTerminationCriteria : public TerminationCriteria {
  public:
    PARAM(PropertyFloat, epsilon, "ratio of decay of chi2 between iteration", 1e-3, 0);
    bool hasToStop() const override;
  };

  /*! @brief Perturbation norm based termination criteria, stop if the norm of the gradient is below
   * the threshold */
  class PerturbationNormTerminationCriteria : public TerminationCriteria {
  public:
    PARAM(PropertyFloat, epsilon, "maximum perturbation norm", 1e-5, 0);
    bool hasToStop() const override;
  };

  /*! @brief Perturbation norm based termination criteria, stop if the norm of the perturbation
   (solution of the linearized system) is below the threshold */
  class RelativeGradientChiTerminationCriteria : public TerminationCriteria {
  public:
    PARAM(PropertyFloat,
          epsilon,
          "threshold for formula ||gradient|| < epsilon (1 + || error ||)",
          0.001,
          0);
    bool hasToStop() const override;
  };

  class RelativeGradientChiAndConstraintViolationTerminationCriteria : public TerminationCriteria {
  public:
    PARAM(PropertyFloat,
          epsilon_gradient,
          "threshold for formula ||gradient|| < epsilon_gradient (1 + || error ||)",
          0.001,
          0);
    PARAM(PropertyFloat,
          epsilon_violation,
          "threshold for formula ||constraint_violation|| < epsilon_constraint",
          0.001,
          0);
    bool hasToStop() const override;

  private:
  };

  class PerturbationNormAndConstraintViolationTerminationCriteria : public TerminationCriteria {
  public:
    PARAM(PropertyFloat,
          epsilon,
          "threshold for formula ||maximum perturbation norm|| < epsilon",
          0.001,
          0);
    PARAM(PropertyFloat,
          epsilon_constraint,
          "threshold for formula ||constraint_violation|| < epsilon",
          0.001,
          0);
    bool hasToStop() const override;

  private:
  };
} // namespace srrg2_solver
