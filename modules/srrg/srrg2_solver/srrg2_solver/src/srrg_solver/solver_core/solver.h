#pragma once
#include "factor_graph_interface.h"
#include "iteration_algorithm_gn.h"
#include "robustifier_policy.h"
#include "solver_action_base.h"
#include "srrg_solver/solver_core/internals/linear_solvers/sparse_block_linear_solver_cholmod_full.h"
#include "termination_criteria.h"
#include <srrg_config/preemptible.h>
#include <srrg_config/property_configurable_vector.h>
#include <srrg_system_utils/chrono.h>

namespace srrg2_solver {

  using namespace srrg2_core;

  using VariablePair       = std::pair<VariableBase*, VariableBase*>;
  using VariablePairVector = std::vector<VariablePair>;
  using MatrixBlockVector  = std::vector<MatrixBlockBase*>;

  using FactorRawPtrVector      = std::vector<FactorBase*>;
  using LevelFactorPtrVectorMap = std::map<size_t, FactorRawPtrVector>;
  /*! @brief Non linear solver acting on factor graphs */
  class Solver : public Configurable, public Preemptible {
  public:
    friend class IterationAlgorithmBase;
    friend class TerminationCriteria;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*! Solver status, used to understand whether the optimization was successful or not*/
    enum SolverStatus : int { Error = -1, Ready = 0, Processing = 1, Success = 2 };
    enum SolverEvent : int {
      ComputeStart   = 0,
      ComputeEnd     = 1,
      IterationStart = 2,
      IterationEnd   = 3,
      LevelStart     = 4,
      LevelEnd       = 5
    };
    PARAM_VECTOR(PropertyVector_<int>,
                 max_iterations,
                 "maximum iterations if no stopping criteria is set",
                 &this->_compute_changed_flag);
    PARAM(PropertyBool, verbose, "turn it off to make the world a better place", false, 0);

    PARAM(PropertyConfigurable_<TerminationCriteria>,
          termination_criteria,
          "term criteria ptr, if 0 solver will do max iterations",
          TerminationCriteriaPtr(new SimpleTerminationCriteria),
          &this->_compute_changed_flag);

    PARAM(PropertyConfigurable_<IterationAlgorithmBase>,
          algorithm,
          "pointer to the optimization algorithm (GN/LM or others)",
          IterationAlgorithmBasePtr(new IterationAlgorithmGN),
          &this->_compute_changed_flag);

    PARAM(PropertyConfigurable_<SparseBlockLinearSolver>,
          linear_solver,
          "pointer to linear solver used to compute Hx=b",
          SparseBlockLinearSolverPtr(new SparseBlockLinearSolverCholmodFull),
          nullptr);

    PARAM_VECTOR(PropertyConfigurableVector_<RobustifierPolicyBase>,
                 robustifier_policies,
                 "policies used to assign robust kernels",
                 0);

    PARAM_VECTOR(PropertyConfigurableVector_<SolverActionBase>,
                 actions,
                 "actions to be performed on certain events",
                 0);

    PARAM(PropertyFloat,
          mse_threshold,
          "Minimum mean square error variation to perform global optimization",
          -1.f,
          0);

    Solver();
    virtual ~Solver();
    bool cmdLoadGraph(std::string& response, const std::string& filename);
    bool cmdSaveGraph(std::string& response, const std::string& filename);
    bool cmdCompute(std::string& response);

    bool loadGraph(const std::string& filename);
    bool saveGraph(const std::string& filename);

    /*! @return The factor graph interface shared pointer */
    FactorGraphInterfacePtr graphPtr() {
      return _graph_ptr;
    }

    /*! @return The factor graph interface shared pointer */
    FactorGraphInterface& graph() {
      if (!_graph)
        throw std::runtime_error("graph is not set, cannot dereference it");
      return *_graph;
    }

    /*! @param[in] graph_ factor graph interface shared pointer */
    void setGraph(FactorGraphInterfacePtr graph_) {
      _graph                      = graph_.get();
      _graph_ptr                  = graph_;
      this->_compute_changed_flag = true;
    }

    void setGraph(FactorGraphInterface& graph_) {
      _graph                      = &graph_;
      _graph_ptr                  = nullptr;
      this->_compute_changed_flag = true;
    }

    /*! @return Variables modified during the last optimization */
    inline const std::vector<VariableBase*>& activeVariables() const {
      return _active_variables;
    }

    /*! @return Factors which were active during the last optimization */
    inline const LevelFactorPtrVectorMap& activeFactorsPerLevel() const {
      return _active_factors;
    }

    /*! @return final chi square of the last optimization */
    inline float chi2() const {
      return lastIterationStats().chi_kernelized;
    }

    inline float normalizedChi2() const {
      const auto& istat = lastIterationStats();
      return istat.chi_kernelized / (istat.dim_factors - istat.dim_variables);
    }

    void printAllocation() const;

    /*! Compute the marginal covariance/cross-correlation of a subset of variables
      @param[out] covariance_matrices_ covariance/cross-correlation blocks
      @param[in] variables_ vector of variables pair, if a pair contain the same variable the
      marginal covariance is computed
      @return false on failure
    */
    bool computeMarginalCovariance(MatrixBlockVector& covariance_matrices_,
                                   const VariablePairVector& variables_);
    /*! Extract blocks of the fisher information matrix (approximate hessian) for a subset of
      variables
      @param[out] information_matrices_ fisher information blocks
      @param[in] variables_ vector of variables pair, for each pair the corresponding block of the
      approximate hessian is returned
    */
    void extractFisherInformationBlocks(MatrixBlockVector& covariance_matrices_,
                                        const VariablePairVector& variables_);

    /*! Extract a single diagonal block of a variable. With template sugar
      Returns null of the variable is not defined, it is fixed
    */
    template <typename VariableType1_, typename VariableType2_>
    Eigen::Matrix<float, VariableType1_::PerturbationDim, VariableType2_::PerturbationDim>
    extractFisherInformationBlock(const VariableType1_& variable1_,
                                  const VariableType2_& variable2_) const;

    template <typename VariableType_>
    inline Eigen::Matrix<float, VariableType_::PerturbationDim, VariableType_::PerturbationDim>
    extractFisherInformationBlock(const VariableType_& variable_) const {
      return extractFisherInformationBlock(variable_, variable_);
    }

    // checks if a variable is determined, by computing the H matrix for its block,
    // and assuming all other active factors and variables in the pool are set
    bool lambdaRatio(float& lambda_min, float& lambda_max, VariableBase* v, int level = 0);

    inline void suppressPerturbation() {
      _suppress_perturbation = true;
    }

    /*! @return Statistics of the last optimization*/
    inline const IterationStatsVector& iterationStats() const {
      return _iteration_stats;
    }
    /*! Clear stats */
    inline void clearIterationStats() {
      _current_iteration = 0;
      _iteration_stats.clear();
    }
    /*! @return Statistics of the measurements */
    inline const FactorStatsVector& measurementStats(const int& level_ = 0) const {
      auto it = _factor_stats.find(level_);
      return it->second;
    }
    /*! @return Statistics of the last iteration in the optimization */
    inline const IterationStats& lastIterationStats() const {
      return *_iteration_stats.rbegin();
    }
    /*! @return current iteration */
    inline const int& currentIteration() const {
      return _current_iteration;
    }

    inline const int status() const {
      return _status;
    }

    /*! Allocate matrix block structure */
    void allocateStructures();

    /*! Solve the optimization problem */
    void compute() override;

    void reset() override;

    bool cmdStats(std::string& response);

    /*! Current level of the optimization, used when the problem is hierarchical */
    inline int currentLevel() const {
      return _current_level;
    }

    bool levelChanged() const {
      return _level_changed;
    }

    inline const SparseBlockMatrix& H() const {return _H;}

    /*! Compute variable ordering */
    void computeVariablesAccessPattern(std::vector<VariableBase::Id>& dest, size_t level=0);

  protected:
    /*! Extract a single diagonal block of a variable.
      Returns null of the variable is not defined, it is fixed
    */
    const MatrixBlockBase* _extractFisherInformationBlock(bool& transposed,
                                                          const VariableBase& variable1_,
                                                          const VariableBase& variable2_) const;

    /*! Assign to each factor type the corresponding robustifier, following the robustifier_policy
     */
    void assignRobustifiers();
    /*! Connect the variables to the corresponding factors in the graph
      @return false if something goes wrong
     */
    bool updateChi(IterationStats& istat);
    bool updateConstraintViolation(IterationStats& istat);
    bool buildQuadraticForm(IterationStats& istat);
    bool solveQuadraticForm(IterationStats& istat);
    void applyPerturbation(IterationStats& istat);
    void getDiagonal(std::vector<float>& diagonal) const;
    void setDiagonal(const std::vector<float>& diagonal);
    void getRHS(std::vector<float>& b) const;
    void setPerturbation(const std::vector<float>& dx) const;
    void getPerturbation(std::vector<float>& dx) const;
    void push();
    void pop();
    void discardTop();

    /*! Determine which factors partecipate to the optimization */
    void computeActiveFactors(bool inlier_only = false);
    /*! Determine the active variables in the current level of the optimization */
    void computeActiveVariables();
    /*! Compute variable ordering */
    void computeOrdering();
    /*! Allocate approximate hessian and gradient vector blocks */
    void allocateLinearSystem();

    SparseBlockMatrix _H; /*!< Approximate hessian matrix */
    SparseBlockMatrix _b; /*!< Gradient vector */

    std::vector<VariableBase*> _active_variables; /*!< Container of variables that partecipate to
                                                     the optimizaion in the current level*/
    std::set<VariableBase::Id> _active_variables_set; /*!< Container of variables that partecipate
                                                     to the optimizaion in the current level*/
    LevelFactorPtrVectorMap
      _active_factors; /*!< Container of factors that partecipate to the optimizaion in each level*/

    std::vector<int> _variable_layout; /*!< Block dimension for each variable */
    std::vector<IntPair> _initial_block_layout;
    FactorGraphInterfacePtr _graph_ptr = nullptr;
    FactorGraphInterface* _graph       = nullptr;

    bool _suppress_perturbation = false;
    int _active_variables_dim   = 0;
    void initActions();
    void callActions(SolverEvent event);

    void bindConfigProperties();

    std::map<int, FactorStatsVector>
      _factor_stats; /*!< Factor stats for each level of the optimization */
    IterationStatsVector _iteration_stats;
    bool _structure_changed_flag = true; /*!< This flag will be set to true if the structure
                                           of linear system has changed */
    bool _compute_changed_flag = true;   /*!< This flag will be set to true if the overall
                                           structure of the problem as changed */
    int _current_level           = 0;
    size_t _max_iterations_total = 0;

    int _current_iteration = 0; /*!< Current iteration of the solver*/

    SolverStatus _status = SolverStatus::Ready; /*!< Status of the solver, exposed to let the
                                                  outside world if everything was ok or not */
    bool _level_changed = 0;
  };

  using SolverPtr = std::shared_ptr<Solver>; /*!<Shared pointer to Solver*/
} // namespace srrg2_solver

#include "solver.hpp"
