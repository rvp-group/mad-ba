#pragma once
#include "factor.h"

namespace srrg2_solver {

  enum FactorConstraintType { Equality = 0x0, Inequality = 0x1 };

  template <int ConstraintDim_, typename... VariableTypes_>
  class ConstraintFactor_ : public Factor_<VariablePtrTuple_<VariableTypes_...>> {
  public:
    using ThisType = ConstraintFactor_<ConstraintDim_, VariableTypes_...>;

    template <typename T_>
    friend class StarFactorCreator_;

    using VariableTupleType = VariablePtrTuple_<VariableTypes_...>;
    using BaseType          = Factor_<VariableTupleType>;

    static constexpr int NumVariables         = BaseType::NumVariables;
    static constexpr int ConstraintDim        = ConstraintDim_;
    using ConstraintVectorType                = Eigen::Matrix<float, ConstraintDim, 1>;
    static constexpr int TotalPerturbationDim = BaseType::TotalPerturbationDim;
    using TotalPerturbationVectorType         = typename BaseType::TotalPerturbationVectorType;

    using TotalJacobianMatrixType =
      typename Eigen::Matrix<float, ConstraintDim, TotalPerturbationDim>;

    using WeightingMatrixType = Eigen::Matrix<float, ConstraintDim, ConstraintDim>; /*!< Type of the
                                                                                 weighting matrix */

    template <typename ThisType_, int r, int c>
    friend struct ColUpdater;

    template <typename ThisType_, int r, int c>
    friend struct RowUpdater;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*! @return Dimension of the i-th variable*/
    template <int i>
    static constexpr int perturbationDim() {
      return VariableTupleType::template perturbationDim<i>();
    }

    /*! @return Offset of the i-th variable perturbation in the jacobian */
    template <int i>
    static constexpr int perturbationOffset() {
      return VariableTupleType::template perturbationOffset<i>();
    }

    /*! @return Type of the i-th jacobian */
    template <int i>
    using JacobianMatrixType = typename Eigen::Map<
      const Eigen::Matrix<float, ConstraintDim, ThisType::template perturbationDim<i>()>>;

    /*! @return The i-th jacobian (read only) */
    template <int i>
    inline const JacobianMatrixType<i> jacobian() const {
      return JacobianMatrixType<i>(_J.data() +
                                   ThisType::template perturbationOffset<i>() * ConstraintDim);
    }
    /*! Type of the i-th jacobian */
    template <int i>
    using MutableJacobianMatrixType = typename Eigen::Map<
      Eigen::Matrix<float, ConstraintDim, ThisType::template perturbationDim<i>()>>;

    /*! @return The i-th jacobian */
    template <int i>
    inline MutableJacobianMatrixType<i> jacobian() {
      return MutableJacobianMatrixType<i>(_J.data() + ThisType::template perturbationOffset<i>() *
                                                        ConstraintDim);
    }

    /*! @return measurement dim */
    int measurementDim() const final {
      return ConstraintDim;
    }

    /*! @return The error vector */
    inline const ConstraintVectorType& constraint() const {
      return _constraint;
    }

    /*! @return The total jacobian */
    inline const TotalJacobianMatrixType& totalJacobian() const {
      return _J;
    }

    inline void reset() {
      //      _rho                       = 0.f; // gain for updating the multiplier
      //      _rho_max                   = 100.f;
      //      _rho_min                   = 0.1f;
      //      _rho_bar                   = 0.1f;
      //      _multiplier_initialization = ConstraintVectorType::Zero();
      //      _constraint_violation      = std::numeric_limits<float>::max();
      //      _constraint = ConstraintVectorType::Zero(); // residual of the constraint evaluation 0
      //      or <=0
      if (_active_set_enabled) {
        _active_constraints = ConstraintVectorType::Zero();
      } else {
        _active_constraints = ConstraintVectorType::Ones();
      }
      _is_initialized = false;
      //      _multiplier         = ConstraintVectorType::Zero();  // scale, updated before
      //                                                           // compute
      //      _J                = TotalJacobianMatrixType::Zero(); // d_constraint/d_x
      //      _weighting_matrix = WeightingMatrixType::Identity();
      //      _is_valid         = true;
      //      _is_initialized   = false;
    }

    /*! @return Weighting matrix (read only)*/
    inline const WeightingMatrixType& weightingMatrix() const {
      return _weighting_matrix;
    }

    /*! Setter for the information matrix
     @param[in] information_matrix inverse covariance matrix to be assigned to the factor
     */
    inline void setWeightingMatrix(const WeightingMatrixType& weighting_matrix) {
      _weighting_matrix = weighting_matrix;
    }

    /*! Checks if the computation is good for this factor */
    bool isValid() const override;

    virtual void constraintAndJacobian(bool error_only) = 0;

    void compute(bool chi_only = false, bool force = false) override;

    /*! Serialize the measurement contained in the factor */
    void serialize(ObjectData& odata, IdContext& context) override;
    /*! Deserialize the measurement contained in the factor */
    void deserialize(ObjectData& odata, IdContext& context) override;

    /*! Auxiliary function to update the approximate hessian blocks that corresponds
      to the variables involved in the factor */
    template <int r, int c>
    inline void _updateHBlock();

    /*! Auxiliary function to update the gradient vector blocks that corresponds
      to the variables involved in the factor */
    template <int r>
    inline void _updateBBlock();

    /*! Auxiliary function to update the approximate hessian that corresponds
      to the variables involved in the factor */
    inline void updateH();

    enum FactorConstraintType _constraint_type = Equality;

    ConstraintVectorType rho() const {
      return _rho;
    }

    float rhoMax() const {
      return _rho_max;
    }

    float rhoMin() const {
      return _rho_min;
    }

    void setRho(const float& rho_bar_, const float& rho_max_ = 100.f, const float& rho_min_ = .1f) {
      _rho     = rho_bar_ * ConstraintVectorType::Ones();
      _rho_bar = rho_bar_;
      _rho_max = rho_max_;
      _rho_min = rho_min_;
    }

    void setRho(const float& rho_bar_,
                const float& rho_max_,
                const float& rho_min_,
                const ConstraintVectorType& rho_) {
      _rho     = rho_;
      _rho_bar = rho_bar_;
      _rho_max = rho_max_;
      _rho_min = rho_min_;
    }

    ConstraintVectorType multiplierInitalization() const {
      return _multiplier_initialization;
    }

    ConstraintVectorType multiplier() {
      return _multiplier;
    }

    void setMultiplierInitalization(const ConstraintVectorType& multiplier_initalization_) {
      _multiplier_initialization = multiplier_initalization_;
      _multiplier                = _multiplier_initialization;
    }

    const ConstraintVectorType& activeConstraints() {
      return _active_constraints;
    }

    void setDeactivationEnabled(bool deactivation_enabled) {
      _deactivation_enabled = deactivation_enabled;
    }

    void disableActiveSet() {
      _active_set_enabled = false;
      _active_constraints = ConstraintVectorType::Ones();
    }

    void enableActiveSet() {
      _active_set_enabled = true;
      _active_constraints = ConstraintVectorType::Zero();
    }

  protected:
    float _rho_max                                  = 100.f;
    float _rho_min                                  = 0.1f;
    float _rho_bar                                  = 0.1f;
    ConstraintVectorType _multiplier_initialization = ConstraintVectorType::Zero();
    ConstraintVectorType _constraint_violation =
      ConstraintVectorType::Zero(); // residual of the constraint evaluation 0 or <=0
    ConstraintVectorType _constraint =
      ConstraintVectorType::Zero(); // residual of the constraint evaluation 0 or <=0
    ConstraintVectorType _rho                = ConstraintVectorType::Zero(); // penalty coefficient
    ConstraintVectorType _active_constraints = ConstraintVectorType::Zero();
    ConstraintVectorType _multiplier = ConstraintVectorType::Zero(); // scale, updated before
                                                                     // compute
    TotalJacobianMatrixType _J            = TotalJacobianMatrixType::Zero(); // d_constraint/d_x
    WeightingMatrixType _weighting_matrix = WeightingMatrixType::Identity();
    bool _is_valid                        = true;
    bool _is_initialized                  = false;
    bool _deactivation_enabled            = false;
    bool _active_set_enabled              = true;
    float _alpha_rho_filter               = 1.f;
  };

} // namespace srrg2_solver
