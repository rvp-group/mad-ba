#include "constraint_factor.h"
#include "factor_impl.cpp"
#include "factor_updaters.h"
#include <Eigen/Dense>
namespace srrg2_solver {
  using namespace srrg2_core;

  template <int ConstraintDim_, typename... VariableTypes_>
  bool ConstraintFactor_<ConstraintDim_, VariableTypes_...>::isValid() const {
    return _is_valid;
  }

  template <int ConstraintDim_, typename... VariableTypes_>
  void ConstraintFactor_<ConstraintDim_, VariableTypes_...>::compute(bool chi_only, bool force) {
    this->_stats.status               = FactorStats::Status::Suppressed;
    this->_stats.chi                  = 0;
    this->_stats.constraint_violation = 0;
    if (!this->isActive() && !force) {
      return;
    }
    constraintAndJacobian(chi_only);
    if (_is_initialized) {
      for (int i = 0; i < ConstraintDim_; ++i) {
        auto m = this->_multiplier[i] + this->_rho[i] * this->_constraint[i];

        switch (this->_constraint_type) {
          case Equality:
            this->_multiplier[i] = m;
            break;
          case Inequality:
            this->_multiplier[i] = std::max(0.f, m);
            break;
          default:
            throw std::runtime_error("constrant_factor_impl: please set the FactorConstraintType");
        }
      }
    }

    if (!isValid()) {
      return;
    }

    ConstraintVectorType constraint_violation =
      ConstraintVectorType::Zero(); // residual of the constraint evaluation 0 or <=0
    float max_constraint_violation = .0f;
    for (int i = 0; i < ConstraintDim_; ++i) {
      switch (this->_constraint_type) {
        case Equality:
          // bb whatever value different from zero is a violation
          constraint_violation[i]  = std::max(fabs(this->_constraint[i]), .0f);
          max_constraint_violation = std::max(fabs(this->_constraint[i]), max_constraint_violation);
          break;
        case Inequality:
          // bb only positive values of the constraint are violations
          constraint_violation[i]  = std::max(this->_constraint[i], .0f);
          max_constraint_violation = std::max(this->_constraint[i], max_constraint_violation);
          break;
        default:
          throw std::runtime_error("constrant_factor_impl: please set the FactorConstraintType");
      }
      // bb compare constraint to multiplier and decide whether Jacobian is zero
      // bb this can be derived by using an auxiliary variable for inequalities
      float constraint_tolerance = this->_constraint[i] + this->_multiplier[i] / this->_rho[i];
      //      float deactivation_threshold = 1e-4f;
      if (this->_constraint_type == Inequality && (constraint_tolerance > 0)) {
        // bb enabled_constraint positive when Jacobian needs to be computed
        this->_active_constraints[i] = 1.f;
      } else if (this->_constraint_type == Inequality && _deactivation_enabled &&
                 _active_set_enabled) {
        this->_active_constraints[i] = 0.f;
      }
    }
    if (this->_is_initialized) {
      for (int i = 0; i < ConstraintDim_; ++i) {
        // bb c_previous - c_current >= 0
        float x_d = std::max(
          0.f, (_constraint_violation[i] - constraint_violation[i]) / _constraint_violation[i]);
        // bb c_current - c_previous >= 0
        float x_i = std::max(
          0.f, (constraint_violation[i] - _constraint_violation[i]) / constraint_violation[i]);
        // bb low pass filter on rho
        float current_rho = _rho_bar + x_d * (_rho_max - _rho_bar) + x_i * (_rho_min - _rho_bar);
        //        float current_rho = _rho_bar + x_i * (_rho_max - _rho_bar) + x_d * (_rho_min -
        //        _rho_bar);
        _rho[i] = _alpha_rho_filter * current_rho + (1 - _alpha_rho_filter) * _rho[i];
      }
    } else {
      this->_is_initialized = true;
    }
    this->_constraint_violation       = constraint_violation;
    this->_stats.constraint_violation = max_constraint_violation;

    if (chi_only) {
      return;
    }
    // std::cerr << _J << std::endl;
    updateH();
  }

  template <int ConstraintDim_, typename... VariableTypes_>
  template <int r, int c>
  void ConstraintFactor_<ConstraintDim_, VariableTypes_...>::_updateHBlock() {
    const int BlockRows = ThisType::template perturbationDim<r>();
    const int BlockCols = ThisType::template perturbationDim<c>();
    int idx             = BaseType::template blockOffset<r, c>();
    // a block is null when a variable is fixed
    if (!this->_H_blocks[idx]) {
      return;
    }
    auto J_r                             = ThisType::template jacobian<r>();
    auto J_c                             = ThisType::template jacobian<c>();
    WeightingMatrixType weighting_matrix = WeightingMatrixType::Zero();
    for (int i = 0; i < ConstraintDim_; ++i) {
      if (this->_constraint_type == Inequality) {
        if (this->_active_constraints[i] <= 1e-4f) {
          J_r.row(i) *= 0.f;
          J_c.row(i) *= 0.f;
        }
      }
      weighting_matrix(i, i) = this->_rho[i] * this->_weighting_matrix(i, i);
    }

    if (!this->_H_transpose[idx]) {
      assert(this->_H_blocks[idx]->rows() == BlockRows &&
             this->_H_blocks[idx]->cols() == BlockCols && "dimension mismatch");
      Eigen::Map<Eigen::Matrix<float, BlockRows, BlockCols>> target_H(
        this->_H_blocks[idx]->storage());
      target_H.noalias() += J_r.transpose() * weighting_matrix * J_c;
    } else {
      assert(this->_H_blocks[idx]->rows() == BlockCols &&
             this->_H_blocks[idx]->cols() == BlockRows && "dimension mismatch");

      Eigen::Map<Eigen::Matrix<float, BlockCols, BlockRows>> target_H(
        this->_H_blocks[idx]->storage());
      target_H.noalias() += J_c.transpose() * weighting_matrix * J_r;
    }
  }

  template <int ConstraintDim_, typename... VariableTypes_>
  template <int r>
  void ConstraintFactor_<ConstraintDim_, VariableTypes_...>::_updateBBlock() {
    const int BlockRows = ThisType::template perturbationDim<r>();
    using namespace std;
    if (!this->_b_blocks[r]) {
      // a block is null if the variable is fixed
      return;
    }
    auto J_r = ThisType::template jacobian<r>();
    for (int i = 0; i < ConstraintDim_; ++i) {
      if (this->_constraint_type == Inequality && this->_active_constraints[i] <= 1e-4f) {
        J_r.row(i) *= 0.f;
      }
    }
    Eigen::Map<Eigen::Matrix<float, BlockRows, 1>> target_b(this->_b_blocks[r]->storage());
    assert(this->_b_blocks[r]->rows() == BlockRows && "dimension mismatch");
    ConstraintVectorType m = ConstraintVectorType::Zero();
    for (int i = 0; i < ConstraintDim_; ++i) {
      m[i] = this->_multiplier[i] + this->_rho[i] * this->_constraint[i];
    }
    target_b.noalias() -= J_r.transpose() * m;
  }

  template <int ConstraintDim_, typename... VariableTypes_>
  void ConstraintFactor_<ConstraintDim_, VariableTypes_...>::updateH() {
    RowUpdater<ThisType, NumVariables - 1, NumVariables - 1>::update(*this);
  }

  // -------------------------------------------------------------------------------------------
  // //
  // -------------------------------------------------------------------------------------------
  // //
  // -------------------------------------------------------------------------------------------
  // //
  template <int ConstraintDim_, typename... VariableTypes_>
  void ConstraintFactor_<ConstraintDim_, VariableTypes_...>::serialize(ObjectData& odata,
                                                                       IdContext& context) {
    Identifiable::serialize(odata, context);
    odata.setInt("graph_id", BaseType::graphId());
    odata.setBool("enabled", BaseType::enabled());
    odata.setInt("level", this->level());

    ArrayData* adata = new ArrayData;
    for (int pos = 0; pos < NumVariables; ++pos) {
      adata->add((int) BaseType::variableId(pos));
    }
    odata.setField("variables", adata);
    odata.setEigen<ConstraintVectorType>("rho", _rho);
    odata.setFloat("rho_max", _rho_max);
    odata.setFloat("rho_min", _rho_min);
    odata.setFloat("rho_bar", _rho_bar);
    odata.setEigen<ConstraintVectorType>("multiplier_initialization", _multiplier_initialization);
    odata.setInt("constraint_type", (int) _constraint_type);

    int w_rows       = _weighting_matrix.rows();
    int w_cols       = _weighting_matrix.cols();
    ArrayData* wdata = new ArrayData;
    for (int r = 0; r < w_rows; ++r) {
      for (int c = r; c < w_cols; ++c) {
        wdata->add(_weighting_matrix(r, c));
      }
    }
    odata.setField("weights", wdata);
  }

  template <int ConstraintDim_, typename... VariableTypes_>
  void ConstraintFactor_<ConstraintDim_, VariableTypes_...>::deserialize(ObjectData& odata,
                                                                         IdContext& context) {
    Identifiable::deserialize(odata, context);
    FactorBase::_graph_id = odata.getInt("graph_id");
    if (odata.getField("enabled")) {
      FactorBase::_enabled = odata.getBool("enabled");
    }
    if (odata.getField("level")) {
      FactorBase::setLevel(odata.getInt("level"));
    }
    ArrayData* adata = dynamic_cast<ArrayData*>(odata.getField("variables"));
    int pos          = 0;
    for (auto it = adata->begin(); it != adata->end(); ++it) {
      ThisType::_variables.setGraphId(pos, (*it)->getInt());
      ++pos;
    }

    this->_rho     = odata.getEigen<ConstraintVectorType>("rho");
    this->_rho_max = odata.getFloat("rho_max");
    this->_rho_min = odata.getFloat("rho_min");
    this->_rho_bar = odata.getFloat("rho_bar");
    this->_multiplier_initialization =
      odata.getEigen<ConstraintVectorType>("multiplier_initialization");
    this->_multiplier      = _multiplier_initialization;
    this->_constraint_type = (FactorConstraintType) odata.getInt("constraint_type");

    int w_rows       = _weighting_matrix.rows();
    int w_cols       = _weighting_matrix.cols();
    ArrayData* wdata = dynamic_cast<ArrayData*>(odata.getField("weights"));
    int k            = 0;
    for (int r = 0; r < w_rows; ++r) {
      for (int c = r; c < w_cols; ++c, ++k) {
        this->_weighting_matrix(r, c) = (*wdata)[k].getFloat();
        this->_weighting_matrix(c, r) = _weighting_matrix(r, c);
      }
    }
  }

} // namespace srrg2_solver
