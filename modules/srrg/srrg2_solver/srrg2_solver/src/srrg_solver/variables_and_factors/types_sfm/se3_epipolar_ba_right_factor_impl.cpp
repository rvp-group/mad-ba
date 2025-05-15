#include "se3_epipolar_ba_right_factor.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_geometry/geometry3d.h"
namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;

  template <typename VariableType_>
  bool SE3EpipolarBARightFactor_<VariableType_>::_resetCompute(bool chi_only) {
    _is_valid=true;
    _Hii_adder.reset();
    _Hij_adder.reset();
    _Hjj_adder.reset();
    _bi_adder.reset();
    _bj_adder.reset();
    _chi_adder.reset();
    _chi_kernel_adder.reset();
    if (!this->isActive()) {
      return false;
    }
    const auto& var_i = this->variables().template at<0>();
    const auto& var_j = this->variables().template at<1>();
    const auto& Xi               = var_i->estimate();
    const auto& Xj               = var_j->estimate();
    const Isometry3f Xij    = Xi.inverse()*Xj;
    Vector3f nij      = Xij.translation();
    float n           = nij.norm();
    if (n<1e-9) {
      _is_valid=false;
      return false;
    }
    nij *= 1./n;
    _E          = Xij.linear().transpose()*geometry3d::skew(nij);
    return true;
  }

  template <typename VariableType_>
  void SE3EpipolarBARightFactor_<VariableType_>::compute(bool chi_only, bool force) {
    this->_stats.status               = FactorStats::Status::Suppressed;
    this->_stats.chi                  = 0;
    this->_stats.constraint_violation = -1;
    if (_fixed.status!=POINT_STATUS::Valid ||
        _moving.status!=POINT_STATUS::Valid)
      return;;
    if (! _is_valid)
      return;
    
    float e;
    JacobianMatrixType Ji, Jj;
    errorAndJacobian(e, Ji, Jj, _fixed.coordinates(), _moving.coordinates(), chi_only);
    this->_stats.chi = pow(e,2);
    this->robustify();
    _chi_adder.add(this->_stats.chi);
    _chi_kernel_adder.add(this->_stats.kernel_chi);
    if (chi_only)
      return;
    _Hii_adder.add(this->_kernel_scales[1]*Ji.transpose()*Ji);
    _Hij_adder.add(this->_kernel_scales[1]*Ji.transpose()*Jj);
    _Hjj_adder.add(this->_kernel_scales[1]*Jj.transpose()*Jj);
    _bi_adder.add(this->_kernel_scales[1]*e*Ji.transpose());
    _bj_adder.add(this->_kernel_scales[1]*e*Jj.transpose());
  }

  template <typename VariableType_>
  void SE3EpipolarBARightFactor_<VariableType_>::finalizeCompute(bool chi_only){
    if (chi_only)
      return;
    //h_ii
    const auto& Hii_=_Hii_adder.sum();
    const auto& Hij_=_Hij_adder.sum();
    const auto& Hjj_=_Hjj_adder.sum();
    const auto& bi_=_bi_adder.sum();
    const auto& bj_=_bj_adder.sum();
    // cerr << "Hii" << endl << Hii_ << endl;
    // cerr << "Hij" << endl << Hij_ << endl;
    // cerr << "Hjj" << endl << Hjj_ << endl;
    // cerr << "bi" << endl << bi_.transpose() << endl;
    // cerr << "bj" << endl << bj_.transpose() << endl;
    auto Hii=this->_H_blocks[this->template blockOffset<0,0>()];
    if (Hii) {
      Eigen::Map<Matrix6f> target(Hii->storage());
      target.noalias()+=Hii_;
    }
    auto Hjj=this->_H_blocks[this->template blockOffset<1,1>()];
    if (Hjj) {
      Eigen::Map<Matrix6f> target(Hjj->storage());
      target.noalias()+=Hjj_;
    }
    auto Hij=this->_H_blocks[this->template blockOffset<0,1>()];
    if (Hij) {
      Eigen::Map<Matrix6f> target(Hij->storage());
      if (this->_H_transpose[this->template blockOffset<0,1>()]){
        target.noalias()+=Hij_.transpose();
      } else
        target.noalias()+=Hij_;
    }
    auto bi = this->_b_blocks[0];
    if (bi) {
      Eigen::Map<Vector6f> target(bi->storage());
      target.noalias()-= bi_;
    }

    auto bj = this->_b_blocks[1];
    if (bj) {
      Eigen::Map<Vector6f> target(bj->storage());
      target.noalias()-= bj_;
    }
  }

  template <typename VariableType_>
  void SE3EpipolarBARightFactor_<VariableType_>::errorAndJacobian(float& e,
                                                       JacobianMatrixType& Ji,
                                                       JacobianMatrixType& Jj,
                                                       const Vector3f& zi,
                                                       const Vector3f& zj,
                                                       bool error_only) const  {
    e=zj.transpose()*_E*zi;
    if (error_only)
      return;
    // put here the coeffs of zj*zi', and assemble them in a row vector
    Eigen::Matrix<float, 1, 9> zij;
    for (int r=0; r<3; ++r)
      for (int c=0; c<3; ++c)
        zij(0,r*3+c)=zj(r)*zi(c);
    // do the product w.r.t Je and get jacobian. If no understand do this on paper.
    Ji = zij*_Jei;
    Jj = zij*_Jej;
  }

} // namespace srrg2_solver

