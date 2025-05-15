#include "essential_translation_factor.h"
#include "srrg_solver/solver_core/error_factor_impl.cpp"
#include "srrg_geometry/geometry3d.h"
namespace srrg2_solver {
  using namespace srrg2_core;
  using namespace std;

    
  void EssentialTranslationFactor::errorAndJacobian(bool chi_only) {
    Matrix3f S=geometry3d::skew(tij_essential)*Ri.transpose();
    auto ti=_variables.at<0>()->estimate();
    auto tj=_variables.at<1>()->estimate();
    auto tji=tj-ti;
    //auto nji=tji.normalized();
    _e=S*tji;
    //cerr << _e.transpose() << endl;
    if (chi_only)
      return;
    // cerr << "tij_essential: " << tij_essential.transpose() << endl;
    // cerr << "Ri: " << endl << Ri << endl;
    // cerr << "nji: " << nji.transpose() << endl;
    // cerr << "S: " << endl << S << endl;
    // cerr << "e: " << _e.transpose() << endl;
    
    //auto Jn=Jnormalize(tji);
    jacobian<0>()=-S;//*Jn;
    jacobian<1>()=S;//*Jn;
  }

  void EssentialTranslationFactor::resetCompute(bool chi_only)  {
    tij_essential.normalize();
  }

} // namespace srrg2_solver

