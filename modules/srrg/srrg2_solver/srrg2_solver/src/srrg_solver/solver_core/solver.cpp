#include "solver.h"
#include "factor_graph.h"
#include "srrg_solver/solver_core/internals/sparse_block_matrix/matrix_block_factory.h"
#include <Eigen/Eigenvalues>
#include <fstream>
#include <srrg_config/configurable_command.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>
#include <sstream>
namespace srrg2_core {
  class ConfigurableShell;
}

namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;

  bool verbose_qf = false;

  std::string solver_debug_H_filename = "";

  Solver::Solver() {
    addCommand(
      new ConfigurableCommand_<Solver, typeof(&Solver::cmdLoadGraph), std::string, std::string>(
        this, "load", "loads a graph from a json file", &Solver::cmdLoadGraph));

    addCommand(
      new ConfigurableCommand_<Solver, typeof(&Solver::cmdSaveGraph), std::string, std::string>(
        this, "save", "saves a graph to a json file", &Solver::cmdSaveGraph));
    addCommand(new ConfigurableCommand_<Solver, typeof(&Solver::cmdStats), std::string>(
      this, "stats", "prints the stats of the last computation", &Solver::cmdStats));
    addCommand(new ConfigurableCommand_<Solver, typeof(&Solver::cmdCompute), std::string>(
      this, "compute", "optimizes the factor graph", &Solver::cmdCompute));

    MatrixBlockFactory* factory = MatrixBlockFactory::instance();
    factory->addAllocator<1, 1>();
    factory->addAllocator<6, 6>();
    factory->addAllocator<9, 9>();
    factory->addAllocator<9, 1>();
    factory->addAllocator<1, 9>();
    factory->addAllocator<6, 3>();
    factory->addAllocator<6, 1>();
    factory->addAllocator<3, 6>();
    factory->addAllocator<3, 3>();
    factory->addAllocator<3, 2>();
    factory->addAllocator<3, 1>();
    factory->addAllocator<2, 3>();
    factory->addAllocator<2, 2>();
    factory->addAllocator<2, 1>();
    factory->addAllocator<1, 2>();
    factory->addAllocator<1, 3>();
    factory->addAllocator<1, 6>();
    // ia allocator for the matchable Jj
    factory->addAllocator<5, 5>();
    factory->addAllocator<7, 5>();
    factory->addAllocator<5, 7>();
    factory->addAllocator<5, 1>();
    factory->addAllocator<1, 5>();
    // ia allocator for the matchable Ji
    factory->addAllocator<5, 6>();
    factory->addAllocator<6, 5>();

    // ia allocator for the chordal
    factory->addAllocator<12, 12>();
    factory->addAllocator<12, 1>();
    factory->addAllocator<1, 12>();

    // ldg allocator for similiarities
    factory->addAllocator<7, 7>();
    factory->addAllocator<7, 1>();
    factory->addAllocator<1, 7>();

    // bb allocator for interior point
    factory->addAllocator<2, 4>();
    factory->addAllocator<4, 2>();
    factory->addAllocator<4, 4>();
    factory->addAllocator<4, 3>();
    factory->addAllocator<3, 4>();
    factory->addAllocator<4, 1>();
    factory->addAllocator<1, 4>();
  }

  Solver::~Solver() {
  }

  bool Solver::cmdSaveGraph(std::string& response, const std::string& filename) {
    response = "saving graph to file [" + filename + "]";
    return saveGraph(filename);
  }

  bool Solver::cmdLoadGraph(std::string& response, const std::string& filename) {
    response = "loading graph from file [" + filename + "]";
    return loadGraph(filename);
  }

  bool Solver::cmdCompute(std::string& response) {
    response = "optimizing the loaded factor graph";
    compute();
    return true;
  }

  bool Solver::cmdStats(std::string& response) {
    std::ostringstream os;
    os << className() + "|stats" << std::endl;
    os << iterationStats() << std::endl;
    response = os.str();
    return true;
  }

  void Solver::reset() {
    Configurable::reset();
    Preemptible::reset();
  }

  bool Solver::saveGraph(const std::string& filename) {
    if (!_graph_ptr) {
      return false;
    }
    FactorGraphPtr graph = std::dynamic_pointer_cast<FactorGraph>(_graph_ptr);
    if (!graph) {
      std::cerr << "no real graph, cannot save" << std::endl;
    }
    graph->write(filename);
    return true;
  }

  bool Solver::loadGraph(const std::string& filename) {
    FactorGraphPtr graph_ = FactorGraph::read(filename);
    if (!graph_) {
      return false;
    }
    setGraph(graph_);
    return true;
  }

  void Solver::computeActiveFactors(bool inlier_only) {
    // we determine the active factors:
    // a factor is active if
    // at least one variable in it is active and no variables are non_active
    // in doing so we reset the indices of all variables encountered
    _active_factors.clear();
    IdFactorPtrContainer& facts = _graph->factors();
    for (auto it = facts.begin(); it != facts.end(); ++it) {
      FactorBase* f = it.value();
      assert(f && "factor null");
      if (!f->enabled()) {
        continue;
      }
      bool is_active = true;
      bool all_fixed = true;
      for (int pos = 0; pos < f->numVariables(); ++pos) {
        VariableBase* const v = f->variable(pos);
        if (!v) {
          std::cerr << "Solver::computeActiveRegion| unable to cast variable for factor #"
                    << f->graphId() << ". Check VariableType assigned to factor" << std::endl;
          throw std::runtime_error("cast failed");
        }
        assert(v && "variable null at factor");
        v->_hessian_index = -1;
        switch (v->status()) {
          case VariableBase::NonActive:
            is_active = false;
            break;
          case VariableBase::Active:
            all_fixed = false;
            break;
          default:;
        }
      }
      if (is_active && !all_fixed) {
        if (inlier_only && f->stats().status != FactorStats::Status::Inlier)
          continue;
        size_t level = f->level();
        _active_factors[level].push_back(f);
      }
    }
    _factor_stats.clear();
    for (auto& l_it : _active_factors) {
      // a factor might have multiple factors inside
      std::sort(l_it.second.begin(),
                l_it.second.end(),
                [](const FactorBase* a, const FactorBase* b) -> bool {
                  return a->graphId() < b->graphId();
                });
      int num_factors_per_level = 0;
      for (auto& f : l_it.second) {
        num_factors_per_level += f->size();
      }
      _factor_stats[l_it.first] = FactorStatsVector(num_factors_per_level);
    }
  }

  void Solver::computeActiveVariables() {
    _active_variables_set.clear();
    const std::vector<FactorBase*>& active_factors_level = _active_factors[this->currentLevel()];
    for (auto it = active_factors_level.begin(); it != active_factors_level.end(); ++it) {
      FactorBase* f = *it;
      assert(f && "Solver::computeActiveRegion|active factor null");
      for (int pos = 0; pos < f->numVariables(); ++pos) {
        VariableBase* v = f->variable(pos);
        assert(v && "Solver::computeActiveRegion|variable at active factor null");
        if (v->status() == VariableBase::Active) {
          _active_variables_set.insert(v->graphId());
        }
      }
    }
    _active_variables.clear();
    _active_variables.reserve(_active_variables_set.size());
    _active_variables_dim = 0;
    for (auto v_id : _active_variables_set) {
      VariableBase* v   = _graph->variable(v_id);
      v->_hessian_index = _active_variables.size();
      _active_variables.emplace_back(v);
      _active_variables_dim += v->perturbationDim();
    }
  }

  void Solver::computeOrdering() {
    std::vector<IntPair> block_layout;
    // auto it = _active_factors.find(this->currentLevel());
    // if (it == _active_factors.end()) {
    //   return;
    // }
    std::vector<FactorBase*>& active_factors_level = _active_factors[this->currentLevel()];
    for (auto it = active_factors_level.begin(); it != active_factors_level.end(); ++it) {
      FactorBase* factor = *it;
      assert(factor && "Solver::computeOrdering|factor null");
      int nvars = factor->numVariables();
      for (int r = 0; r < nvars; ++r) {
        VariableBase* var_r = factor->variable(r);
        assert(var_r && "Solver::computeOrdering|var_r null");
        if (var_r->_hessian_index < 0) {
          continue;
        }
        int idx_r = var_r->_hessian_index;
        for (int c = r; c < nvars; ++c) {
          VariableBase* var_c = factor->variable(c);
          assert(var_c && "Solver::computeOrdering|var_c null");
          if (var_c->_hessian_index < 0) {
            continue;
          }
          int idx_c = var_c->_hessian_index;

          if (idx_r < idx_c) {
            std::swap(idx_r, idx_c);
          }
          block_layout.push_back(std::make_pair(idx_r, idx_c));
        }
      }
    }
    // sort, column major
    std::sort(
      block_layout.begin(), block_layout.end(), [](const IntPair& a, const IntPair& b) -> bool {
        return a.second < b.second || (a.second == b.second && a.first < b.first);
      });
    // remove duplicates
    std::vector<IntPair>::iterator last = std::unique(block_layout.begin(), block_layout.end());
    block_layout.erase(last, block_layout.end());
    // compute ordering
    std::vector<int> ordering;
    ordering.resize(_active_variables.size());
    SparseBlockLinearSolverPtr _linear_solver = param_linear_solver.value();
    _linear_solver->computeOrderingHint(ordering, block_layout);
    std::vector<VariableBase*> src(_active_variables);
    for (size_t i = 0; i < _active_variables.size(); ++i) {
      _active_variables[i] = src[ordering[i]];
    }
  }

  void Solver::printAllocation() const {
    cerr << "H: br=" << _H.blockRows() << endl;
    cerr << "H: bc=" << _H.blockCols() << endl;
    cerr << "H, nnzb: " << _H.numNonZeroBlocks() << endl;

    cerr << "H: r=" << _H.rows() << endl;
    cerr << "H: c=" << _H.cols() << endl;
    cerr << "H, nnz: " << _H.numNonZeros() << endl;
  }

  void Solver::allocateLinearSystem() {
    // we determine the block layout of the linear system
    // and reassign the indices based on the ordering in _active_variables
    _variable_layout.clear();
    std::vector<FactorBase*>& active_factors_level = _active_factors[this->currentLevel()];
    _variable_layout.resize(_active_variables.size());
    for (size_t i = 0; i < _active_variables.size(); ++i) {
      VariableBase* var   = _active_variables[i];
      var->_hessian_index = i;
      _variable_layout[i] = var->perturbationDim();
    }

    // 4. make the layout of H and b
    _H = SparseBlockMatrix(_variable_layout, _variable_layout);
    _b = SparseBlockMatrix(_variable_layout, std::vector<int>(1, 1));
    // 4. populate the target indices of all factors with the right
    //   H and b blocks
    for (auto it = active_factors_level.begin(); it != active_factors_level.end(); ++it) {
      FactorBase* factor = *it;
      factor->clearTargetBlocks();

      int nvars = factor->numVariables();
      for (int r = 0; r < nvars; ++r) {
        VariableBase* row_var = factor->variable(r);
        if (row_var->_hessian_index < 0) {
          continue;
        }

        MatrixBlockBase* bblock = _b.blockAt(row_var->_hessian_index, 0, true);
        factor->setRHSTargetBlock(r, bblock);

        for (int c = r; c < nvars; ++c) {
          VariableBase* col_var = factor->variable(c);
          if (col_var->_hessian_index < 0) {
            continue;
          }

          // we need to decide of a block is transposed or not
          // depending on the hessian indices
          MatrixBlockBase* hblock = 0;
          bool is_transposed      = false;
          if (row_var->_hessian_index <= col_var->_hessian_index) {
            hblock = _H.blockAt(row_var->_hessian_index, col_var->_hessian_index, true);
          } else {
            hblock        = _H.blockAt(col_var->_hessian_index, row_var->_hessian_index, true);
            is_transposed = true;
          }
          factor->setHTargetBlock(r, c, hblock, is_transposed);
        }
      }
    }
    SparseBlockLinearSolverPtr _linear_solver = param_linear_solver.value();

    _linear_solver->setStructureChanged();
  }

  void Solver::assignRobustifiers() {
    std::vector<FactorBase*>& active_factors_level = _active_factors[this->currentLevel()];
    for (auto it = active_factors_level.begin(); it != active_factors_level.end(); ++it) {
      FactorBase* factor = *it;
      for (size_t i = 0; i < param_robustifier_policies.size(); ++i) {
        RobustifierPolicyBasePtr policy = param_robustifier_policies.value(i);
        if (!policy) {
          continue;
        }
        RobustifierBasePtr r = policy->getRobustifier(factor);
        if (r) {
          factor->setRobustifier(r.get());
          break;
        }
      }
    }
  }

  bool Solver::computeMarginalCovariance(MatrixBlockVector& covariance_matricies_,
                                         const VariablePairVector& variables_) {
    assert(param_linear_solver.value() && "Solver::computeMarginalCovariance|linear solver not in");
    assert(_graph && "Solver::computeMarginalCovariance|no graph set");
    if (_iteration_stats.empty()) {
      throw std::runtime_error("Solver::computeMarginalCovariance|need to call compute() ");
    }

    SparseBlockLinearSolverPtr _linear_solver = param_linear_solver.value();
    SparseBlockMatrix x(_H.blockRowDims(), _H.blockColDims());
    covariance_matricies_.clear();
    covariance_matricies_.reserve(variables_.size());
    // tg compute blocks to be inverted based on the variables hessian indices
    std::vector<IntPair> block_structure;
    block_structure.reserve(variables_.size());
    for (const VariablePair& vp : variables_) {
      block_structure.emplace_back(vp.first->_hessian_index, vp.second->_hessian_index);
    }
    // tg sort blocks
    std::sort(block_structure.begin(),
              block_structure.end(),
              [](const IntPair& a, const IntPair& b) -> bool {
                return a.second < b.second || (a.second == b.second && a.first < b.first);
              });
    // compute block inverse
    if (!_linear_solver->computeBlockInverse(x, block_structure)) {
      return false;
    }
    // float covariance_scale = normalizedChi2();
    // cerr << "cscale: " << covariance_scale << endl;
    for (const VariablePair& vp : variables_) {
      MatrixBlockBase* cov_block =
        x.blockRelease(vp.first->_hessian_index, vp.second->_hessian_index);
      if (!cov_block) {
        throw std::runtime_error("Solver::computeMarginalCovariance | no block found, maybe "
                                 "you ask two times the same variables covariance");
      }
      // cov_block->scale(covariance_scale);
      covariance_matricies_.emplace_back(cov_block);
    }

    return true;
  }

  void Solver::extractFisherInformationBlocks(MatrixBlockVector& information_matricies_,
                                              const VariablePairVector& variables_) {
    assert(param_linear_solver.value() &&
           "Solver::extractFisherInformationBlocks|linear solver not in");
    assert(_graph && "Solver::extractFisherInformationBlocks|no graph set");
    if (_iteration_stats.empty()) {
      throw std::runtime_error("Solver::extractFisherInformationBlocks|need to call compute() ");
    }
    information_matricies_.clear();
    information_matricies_.reserve(variables_.size());
    for (const VariablePair& vp : variables_) {
      MatrixBlockBase* info_block =
        _H.blockRelease(vp.first->_hessian_index, vp.second->_hessian_index);
      if (!info_block) {
        throw std::runtime_error("Solver::extractFisherInformationBlocks | no block found, maybe "
                                 "you ask two times the same variables covariance");
      }
      information_matricies_.emplace_back(info_block);
    }
  }

  /*! Extract a single diagonal block of a variable.
    Returns null of the variable is not defined, it is fixed
  */
  const MatrixBlockBase*
  Solver::_extractFisherInformationBlock(bool& transposed,
                                         const VariableBase& variable1_,
                                         const VariableBase& variable2_) const {
    assert(param_linear_solver.value() &&
           "Solver::extractFisherInformationBlocks|linear solver not in");
    assert(_graph && "Solver::extractFisherInformationBlocks|no graph set");
    if (_iteration_stats.empty()) {
      throw std::runtime_error("Solver::extractFisherInformationBlocks|need to call compute() ");
    }
    auto idx1  = variable1_._hessian_index;
    auto idx2  = variable2_._hessian_index;
    transposed = false;
    if (idx1 > idx2) {
      std::swap(idx1, idx2);
      transposed = true;
    }
    return _H.blockAt(idx1, idx2);
  }

  
  void Solver::computeVariablesAccessPattern(std::vector<VariableBase::Id>& dest, size_t this_level) {
    computeActiveFactors();
    if (this_level>=_active_factors.size())
      throw std::runtime_error("Solver::computeVariableAccessPattern| level error");
    dest.clear();
    std::vector<FactorBase*>& active_factors_level = _active_factors[this_level];
    for (FactorBase* outer_factor : active_factors_level) {
      outer_factor->setBegin();
      FactorBase* factor;
      while (outer_factor->getNext(factor)) {
        factor->updateVariablesAccessPattern(dest);
      }
    }
  }

  
  bool Solver::updateChi(IterationStats& istat) {
    Chrono chi_update_t("chi_update", istat.t_extra, false);
    istat.iteration                 = _current_iteration;
    bool chi_only                   = true;
    istat.num_inliers               = 0;
    istat.num_outliers              = 0;
    istat.chi_inliers               = 0;
    istat.chi_outliers              = 0;
    istat.num_suppressed            = 0;
    istat.chi_kernelized            = 0;
    istat.dim_factors               = 0;
    istat.dim_variables             = _active_variables_dim;
    istat.constraint_violation_norm = 0;
    istat.num_constraints           = 0;
    // tg I have to do this to get the proper index in the vector of factors stats
    size_t current_level = this->currentLevel();
    // tg get proper initial index in factors stats
    int factor_idx                                 = 0;
    istat.level                                    = current_level;
    std::vector<FactorBase*>& active_factors_level = _active_factors[current_level];
    FactorStatsVector& factor_level_stats          = _factor_stats[current_level];
    for (FactorBase* outer_factor : active_factors_level) {
      outer_factor->resetCompute(chi_only);
      outer_factor->setBegin();
      FactorBase* factor;
      while (outer_factor->getNext(factor)) {
        if (factor->variablesUpdated()) {
          factor->resetStats();
          factor->compute(chi_only);
        }
        factor_level_stats[factor_idx] = factor->stats();
        FactorStats& mstat             = factor_level_stats[factor_idx];
        ++factor_idx;
        //        bb update constraint violation
        float constraint_violation = mstat.constraint_violation;
        if (constraint_violation > 0) {
          ++istat.num_constraints;
          istat.constraint_violation_norm =
            std::max(istat.constraint_violation_norm, constraint_violation);
        }
        switch (mstat.status) {
          case FactorStats::Status::Inlier:
            istat.num_inliers++;
            istat.chi_inliers += mstat.chi;
            istat.chi_kernelized += mstat.kernel_chi;
            istat.dim_factors += factor->measurementDim();
            break;
          case FactorStats::Status::Kernelized:
            istat.num_outliers++;
            istat.chi_outliers += mstat.chi;
            istat.chi_kernelized += mstat.kernel_chi;
            istat.dim_factors += factor->measurementDim();
            break;
          case FactorStats::Status::Suppressed:
            // ds: fallthrough to return false intended?
            // gg: this happens when a point is outside the camera, or in general
            //     when the error function fails
            istat.num_suppressed++;
            break;
          default:
            return false;
        }
      }
      outer_factor->finalizeCompute(chi_only);
    }
    for (auto it : _active_variables) {
      it->clearUpdated();
    }
    istat.t_extra += chi_update_t.duration();
    return true;
  }

  bool Solver::buildQuadraticForm(IterationStats& istat) {
    Chrono quadratic_form_t("quadratic_form", istat.t_linearize, false);
    _H.setZero();
    _b.setZero();

    std::vector<float> b_values;

    istat.reset();
    istat.iteration = _current_iteration;
    // tg I have to do this to get the proper index in the vector of factors stats
    size_t current_level = this->currentLevel();
    // tg get proper initial index in factors stats
    int factor_idx                                 = 0;
    istat.level                                    = current_level;
    std::vector<FactorBase*>& active_factors_level = _active_factors[current_level];
    FactorStatsVector& factor_level_stats          = _factor_stats[current_level];
    istat.dim_variables                            = _active_variables_dim;
    for (FactorBase* outer_factor : active_factors_level) {
      outer_factor->resetCompute();
      outer_factor->setBegin();
      FactorBase* factor;
      while (outer_factor->getNext(factor)) {
        factor->resetStats();
        factor->compute();
        factor_level_stats[factor_idx] = factor->stats();
        FactorStats& mstat             = factor_level_stats[factor_idx];
        ++factor_idx;
        switch (mstat.status) {
          case FactorStats::Status::Inlier:
            istat.num_inliers++;
            istat.chi_inliers += mstat.chi;
            istat.chi_kernelized += mstat.chi;
            istat.dim_factors += factor->measurementDim();
            break;
          case FactorStats::Status::Kernelized:
            istat.num_outliers++;
            istat.chi_outliers += mstat.chi;
            istat.chi_kernelized += mstat.kernel_chi;
            istat.dim_factors += factor->measurementDim();
            break;
          case FactorStats::Status::Suppressed:
            istat.num_suppressed++;
            break;
          default:
            return false;
        }
        outer_factor->finalizeCompute();
      }
    }
    _b.getDenseVector(b_values);
    double b_sum = 0;
    for (const auto& v : b_values)
      b_sum += v;

    if (solver_debug_H_filename.length()) {
      ofstream os(solver_debug_H_filename);
      os << "bcc, chi2: " << istat.chi_inliers << " " << _H.numNonZeroBlocks() << " " << b_sum
         << endl;
      _H.printLayout(SparseBlockMatrix::PrintValues, os);
      os.close();
    }

    for (auto it : _active_variables) {
      it->clearUpdated();
    }

    istat.t_linearize += quadratic_form_t.duration();
    return true;
  }

  bool Solver::solveQuadraticForm(IterationStats& istat) {
    // _H.printLayout(SparseBlockMatrix::PrintMode::PrintValues);
    Chrono solve_linear_t("solver_linear", istat.t_solve, false);
    SparseBlockLinearSolverPtr _linear_solver = param_linear_solver.value();
    _linear_solver->setCoefficientsChanged();
    _linear_solver->compute();
    if (_linear_solver->status() != SparseBlockLinearSolver::SolutionGood) {
      if (param_verbose.value())
        std::cerr << "Solver::solveQuadraticForm|WARNING, solver error [ "
                  << _linear_solver->stringStatus() << " ]\n";
      return false;
    }
    return true;
  }

  void Solver::applyPerturbation(IterationStats& istat) {
    if (_suppress_perturbation) {
      _suppress_perturbation = false;
      return;
    }
    Chrono perturbation_t("perturbation", istat.t_update, false);
    SparseBlockLinearSolverPtr _linear_solver = param_linear_solver.value();
    for (auto it = _active_variables.begin(); it != _active_variables.end(); ++it) {
      VariableBase* var = *it;
      int hessian_index = var->_hessian_index;
      if (hessian_index < 0) {
        continue;
      }
      const MatrixBlockBase* update = _linear_solver->x().blockAt(hessian_index, 0);
      var->applyPerturbationRaw(update->storage());
      var->normalize();
    }

    updateChi(istat);
  }

  void Solver::getDiagonal(std::vector<float>& diagonal) const {
    _H.getDiagonal(diagonal);
  }

  void Solver::setDiagonal(const std::vector<float>& diagonal) {
    _H.setDiagonal(diagonal);
  }

  void Solver::getPerturbation(std::vector<float>& dx) const {
    const SparseBlockLinearSolverPtr _linear_solver = param_linear_solver.value();
    _linear_solver->x().getDenseVector(dx);
  }

  void Solver::setPerturbation(const std::vector<float>& dx) const {
    const SparseBlockLinearSolverPtr _linear_solver = param_linear_solver.value();
    int idx                                         = 0;
    SparseBlockMatrix dX(_b.blockRowDims(), _b.blockColDims());
    for (int rb = 0; rb < dX.blockRows(); ++rb) {
      MatrixBlockBase* row_block = dX.blockAt(rb, 0, true);
      int block_dim              = dX.blockDims(rb, 0).first;
      for (int r = 0; r < block_dim; ++r, ++idx) {
        row_block->at(r, 0) = dx.at(idx);
      }
    }
    _linear_solver->setX(dX);
  }

  void Solver::getRHS(std::vector<float>& b) const {
    _b.getDenseVector(b);
  }

  void Solver::push() {
    for (auto it = _active_variables.begin(); it != _active_variables.end(); ++it) {
      (*it)->push();
    }
  }

  void Solver::pop() {
    for (auto it = _active_variables.begin(); it != _active_variables.end(); ++it) {
      (*it)->pop();
    }
  }

  void Solver::discardTop() {
    for (auto it = _active_variables.begin(); it != _active_variables.end(); ++it) {
      (*it)->discardTop();
    }
  }

  bool Solver::lambdaRatio(float& lambda_min, float& lambda_max, VariableBase* v, int level_) {
    lambda_min = lambda_max = 100;
    if (v->status() == VariableBase::Fixed) {
      return true;
    }

    if (v->status() == VariableBase::NonActive)
      return false;

    if (!_active_variables_set.count(v->graphId()))
      return false;

    int h_index = v->_hessian_index;
    if (h_index < 0) {
      return false;
    }

    MatrixBlockBase* Hxx = _H.blockAt(h_index, h_index);
    assert(Hxx);
    if (!Hxx)
      throw std::runtime_error("no H");

    int dim = v->perturbationDim();
    if (Hxx->rows() != v->perturbationDim()) {
      throw std::runtime_error("dim mismatch" + to_string(dim) + " instead of" +
                               to_string(Hxx->rows()));
    }
    Hxx->setZero();
    for (auto f : _graph->factors(v)) {
      if (!f->enabled())
        continue;

      if (level_ != -1 && f->level() != level_)
        continue;

      bool active = std::binary_search(_active_factors[_current_level].begin(),
                                       _active_factors[_current_level].end(),
                                       f,
                                       [](const FactorBase* a, const FactorBase* b) -> bool {
                                         return a->graphId() < b->graphId();
                                       });
      if (!active)
        continue;

      int index = -1;
      // retrieve the index of this variable
      for (int i = 0; i < f->numVariables(); ++i) {
        if (f->variableId(i) == v->graphId()) {
          index = i;
        }
      }
      if (index == -1)
        return false;

      f->setRobustifier(nullptr);
      f->compute();
    }
    Eigen::MatrixXf eH(dim, dim);
    for (int r = 0; r < dim; ++r) {
      for (int c = 0; c < dim; ++c) {
        eH(r, c) = Hxx->at(r, c);
      }
    }
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> es(dim);
    es.compute(eH);
    lambda_min = es.eigenvalues()(0);
    lambda_max = es.eigenvalues()(dim - 1);
    return true;
  }

  void Solver::compute() {
    assert(param_linear_solver.value() && "Solver::compute|linear solver not in");
    assert(_graph && "Solver::compute|graph not set");
    initActions();
    // if the graph is changed we need to adjust the active factors and the iteration stats
    if (_compute_changed_flag) {
      computeActiveFactors();
      _max_iterations_total = 0;
      for (size_t i = 0; i < param_max_iterations.size(); ++i)
        _max_iterations_total += param_max_iterations.value(i);

      if (!_max_iterations_total) {
        std::cerr << FG_YELLOW("Solver::compute|number of iterations == 0") << std::endl;
      }
      _iteration_stats.clear();
      _iteration_stats.reserve(_max_iterations_total);

      if (!param_algorithm.value()) {
        throw std::runtime_error("Solver::compute|no algorithm set");
      }
      _compute_changed_flag = false;
    }
    param_algorithm->setSolver(this);
    TerminationCriteriaPtr tc = param_termination_criteria.value();
    if (tc) {
      tc->setSolver(this);
    }

    // here we do the optimization, one level at a time, from highest to lowest
    _iteration_stats.clear();
    _current_iteration = 0;
    _status            = Solver::SolverStatus::Processing;
    int highest_level  = param_max_iterations.size() - 1;
    callActions(ComputeStart);
    // process the levels from highest to lowest
    for (_current_level = highest_level; _current_level >= 0; --_current_level) {
      int max_level_iterations = param_max_iterations.value(_current_level);
      // prevent to cancel the internal structures for no iterations on current level
      if (!max_level_iterations) {
        continue;
      }

      // prepare for new level
      for (auto it : _graph->factors()) {
        it.second->_current_level = _current_level;
      }
      assignRobustifiers();
      computeActiveVariables();
      computeOrdering();
      allocateLinearSystem();
      param_linear_solver.value()->bindLinearSystem(&_H, &_b);

      _level_changed = true;
      callActions(LevelStart);
      for (int level_iteration = 0; level_iteration < max_level_iterations; ++level_iteration) {
        // ia perform all the pre-iteration actions
        callActions(IterationStart);
        // ia do the trick
        const bool solution_ok = param_algorithm->oneRound();
        callActions(IterationEnd);

        ++_current_iteration;

        // ia check if algorithm failed
        if (!solution_ok) {
          if (param_verbose.value())
            cerr << "Solver::compute|solver fail" << endl;
          _status = Solver::SolverStatus::Error;
          callActions(LevelEnd);
          callActions(ComputeEnd);
          return;
        }

        // ia check if we have to stop
        if (tc && tc->hasToStop()) {
          callActions(LevelEnd);
          callActions(ComputeEnd);
          break;
        }
        _level_changed = false;
      }
      callActions(LevelEnd);
    }
    _current_level = 0;
    _status        = Solver::SolverStatus::Success;
    callActions(ComputeEnd);
  }

  void Solver::initActions() {
    std::vector<SolverActionBasePtr>& actions = param_actions.value();
    std::sort(actions.begin(),
              actions.end(),
              [](const SolverActionBasePtr& a1, const SolverActionBasePtr& a2) -> bool {
                if (!a1 && !a2)
                  return false;
                if (a1 && !a2)
                  return true;
                if (a2 && !a1)
                  return false;
                auto e1 = a1->param_event.value();
                auto e2 = a2->param_event.value();
                if (e1 != e2)
                  return e1 < e2;

                auto p1 = a1->param_priority.value();
                auto p2 = a2->param_priority.value();
                return p1 < p2;
              });
    for (auto act : actions) {
      if (act)
        act->init(*this);
    }
  }

  void Solver::callActions(SolverEvent event) {
    std::vector<SolverActionBasePtr>& actions = param_actions.value();
    for (auto act : actions) {
      if (act && act->param_event.value() == event) {
        act->doAction();
      }
    }
  }

} // namespace srrg2_solver
