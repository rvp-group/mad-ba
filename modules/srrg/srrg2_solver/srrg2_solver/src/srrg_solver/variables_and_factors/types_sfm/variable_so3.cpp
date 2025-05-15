#include "variable_so3.h"
#include <Eigen/SVD>

namespace srrg2_solver {
  using namespace srrg2_core;
  void
  VariableSO3Relaxed::applyPerturbation(const VariableSO3Relaxed::PerturbationVectorType& pert) {
    this->_updated = true;
    auto m         = unflatten(pert);
    this->_estimate += m;
  }
  void VariableSO3Relaxed::normalize() {
    //    Eigen::JacobiSVD<Matrix3f, Eigen::ComputeFullU|Eigen::ComputeFullV> svd(this->_estimate,
    //    Eigen::ComputeFullU|Eigen::ComputeFullV);
    //    _estimate=svd.matrixU()*svd.matrixV().transpose();
    //    Eigen::Matrix3f W=Matrix3f::Identity();
    //    if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0.0) {
    //      W(2, 2) = -1.0;
    //    }
    //    this->_estimate=svd.matrixU()*W*svd.matrixV().transpose();
  }

} // namespace srrg2_solver
