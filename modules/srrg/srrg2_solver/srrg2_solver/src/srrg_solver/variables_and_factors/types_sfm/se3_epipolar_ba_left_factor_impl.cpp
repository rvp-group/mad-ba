#pragma once
namespace srrg2_solver {

  template <typename VariableType_>
  bool SE3EpipolarBALeftFactor_<VariableType_>::_resetCompute(bool chi_only) {
    _is_valid=true;
    _H_adder.reset();
    _b_adder.reset();
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
    // cerr << className() <<" jacobian: " << endl;
    // cerr << _Je << endl;
    // exit(0);
  }

  template <typename VariableType_>
  void SE3EpipolarBALeftFactor_<VariableType_>::compute(bool chi_only, bool force) {
    this->_stats.status               = FactorStats::Status::Suppressed;
    this->_stats.chi                  = 0;
    this->_stats.constraint_violation = -1;
    if (_fixed.status!=POINT_STATUS::Valid ||
        _moving.status!=POINT_STATUS::Valid)
      return;
    if (! _is_valid)
      return;
    
    float e;
    JacobianMatrixType J;
    errorAndJacobian(e, J, _fixed.coordinates(), _moving.coordinates(), chi_only);
    this->_stats.chi = pow(e,2);
    this->robustify();
    _chi_adder.add(this->_stats.chi);
    _chi_kernel_adder.add(this->_stats.kernel_chi);
    if (chi_only)
      return;

    _H_adder.add(this->_kernel_scales[1]*J.transpose()*J);
    _b_adder.add(this->_kernel_scales[1]*e*J.transpose());
  }

  template <typename VariableType_>
  void SE3EpipolarBALeftFactor_<VariableType_>::finalizeCompute(bool chi_only){
    if (chi_only)
      return;
    //h_ii
    const auto& H=_H_adder.sum();
    const auto& b=_b_adder.sum();
    auto Hii=this->_H_blocks[this->template blockOffset<0,0>()];
    if (Hii) {
      Eigen::Map<Matrix6f> target(Hii->storage());
      target.noalias()+=H;
    }
    auto Hjj=this->_H_blocks[this->template blockOffset<1,1>()];
    if (Hjj) {
      Eigen::Map<Matrix6f> target(Hjj->storage());
      target.noalias()+=H;
    }
    auto Hij=this->_H_blocks[this->template blockOffset<0,1>()];
    if (Hij) {
      Eigen::Map<Matrix6f> target(Hij->storage());
      target.noalias()-= H; // symmetric, no need to check for transpose;
    }
    auto bi = this->_b_blocks[0];
    if (bi) {
      Eigen::Map<Vector6f> target(bi->storage());
      target.noalias()-= b;
    }

    auto bj = this->_b_blocks[1];
    if (bj) {
      Eigen::Map<Vector6f> target(bj->storage());
      target.noalias()+= b;
    }
  }

  template <typename VariableType_>
  void SE3EpipolarBALeftFactor_<VariableType_>::errorAndJacobian(float& e,
                                                          JacobianMatrixType& Ji,
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
    Ji = zij*_Je;
    // using namespace std;
    // cerr << "zi: " << zi.transpose() << " zj: " << zj.transpose() << " e:" << e
    //      << " J: " << Ji << endl;
    // exit(0);
  }

}
