#include "srrg_geometry/geometry3d.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  // this crap calculates the essential at the perturbation in dualvalue
  inline Eigen::Matrix<DualValuef, 3, 3> essentialEulerLeft(const Isometry3_<DualValuef>& Ti,
                                                            const Isometry3_<DualValuef>& Tj,
                                                            const Vector6_<DualValuef>& dti,
                                                            const Vector6_<DualValuef>& dtj){
    const Vector6_<DualValuef> neg_dti=-dti;
    const Isometry3_<DualValuef>Tij=Ti.inverse()*geometry3d::ta2t(neg_dti)*geometry3d::ta2t(dtj)*Tj;
    Vector3_<DualValuef> trans=Tij.translation().normalized();
    return Tij.linear().transpose()*geometry3d::skew(trans);
  }

  
  // this crap calculates the essential at the perturbation in dualvalue
  inline Eigen::Matrix<DualValuef, 3, 3> essentialQuatLeft(const Isometry3_<DualValuef>& Ti,
                                                           const Isometry3_<DualValuef>& Tj,
                                                           const Vector6_<DualValuef>& dti,
                                                           const Vector6_<DualValuef>& dtj){
    const Isometry3_<DualValuef>Tij=Ti.inverse()*geometry3d::v2t(dti).inverse()*geometry3d::v2t(dtj)*Tj;
    Vector3_<DualValuef> trans=Tij.translation().normalized();
    return Tij.linear().transpose()*geometry3d::skew(trans);
  }

  // this returns a 9x6 matrix. The columns are the derivatives of the essential w.r.t.
  // the null perturbation perturbation ti[col]

  template <typename FN_>
  inline Eigen::Matrix<float, 9, 6> essentialLeftJacobian(const Eigen::Isometry3f& Ti_,
                                                          const Eigen::Isometry3f& Tj_,
                                                          FN_ fn) {
    Isometry3_<DualValuef> Ti, Tj;
    for (int r=0; r<4; ++r){
      for (int c=0; c<4; ++c){
        Ti.matrix()(r,c)=DualValuef(Ti_.matrix()(r,c), 0.);
        Tj.matrix()(r,c)=DualValuef(Tj_.matrix()(r,c), 0.);
      }
    }
    Eigen::Matrix<float, 9, 6> ret;
    Vector6_<DualValuef> dti, dtj;
    for (int k=0; k<6; ++k){
      dti[k]=DualValuef(0,0);
      dtj[k]=DualValuef(0,0);
    }
    
    for (int k=0; k<6; ++k) {
      dti[k]=DualValuef(0,1);
      Matrix3_<DualValuef> ei=fn(Ti, Tj, dti, dtj);
      dti[k]=DualValuef(0,0);
      for (int r=0; r<3; ++r)
        for (int c=0; c<3; ++c)
          ret(r*3+c,k)=ei(r,c).derivative;
    }
    return ret;
  }


  
  // this crap calculates the essential at the perturbation in dualvalue
  inline Matrix3_<DualValuef> essentialEulerRight(const Isometry3_<DualValuef>& Ti,
                                                  const Isometry3_<DualValuef>& Tj,
                                                  const Isometry3_<DualValuef>& iOi,
                                                  const Isometry3_<DualValuef>& Oj,
                                                  const Vector6_<DualValuef>& dti,
                                                  const Vector6_<DualValuef>& dtj){
    const Vector6_<DualValuef> neg_dti = -dti;
    Isometry3_<DualValuef> Tij_base=Ti.inverse()*Tj;
    const Isometry3_<DualValuef>Tij=iOi*geometry3d::ta2t(neg_dti)*Tij_base*geometry3d::ta2t(dtj)*Oj;
    Vector3_<DualValuef> nij=Tij.translation().normalized();
    return Tij.linear().transpose()*geometry3d::skew(nij);
  }

  inline Matrix3_<DualValuef> essentialQuaternionRight(const Isometry3_<DualValuef>& Ti,
                                                       const Isometry3_<DualValuef>& Tj,
                                                       const Isometry3_<DualValuef>& iOi,
                                                       const Isometry3_<DualValuef>& Oj,
                                                       const Vector6_<DualValuef>& dti,
                                                       const Vector6_<DualValuef>& dtj){
    Isometry3_<DualValuef> Tij_base=Ti.inverse()*Tj;
    const Isometry3_<DualValuef>Tij=iOi*geometry3d::v2t(dti).inverse()*Tij_base*geometry3d::v2t(dtj)*Oj;
    Vector3_<DualValuef> nij=Tij.translation().normalized();
    return Tij.linear().transpose()*geometry3d::skew(nij);
  }

  // this returns a 9x6 matrix. The columns are the derivatives of the essential w.r.t.
  // the null perturbation perturbation ti[col]
  template <typename FnType_>
  inline void essentialJacobiansRight(Eigen::Matrix<float, 9, 6>& Ji,
                                      Eigen::Matrix<float, 9, 6>& Jj,
                                      const Eigen::Isometry3f& Ti_,
                                      const Eigen::Isometry3f& Tj_,
                                      const Eigen::Isometry3f& Oi_,
                                      const Eigen::Isometry3f& Oj_,
                                      FnType_ fn) {

    Isometry3f iOi_=Oi_.inverse();
    Isometry3_<DualValuef> Ti, Tj, iOi, Oj;
    for (int r=0; r<4; ++r){
      for (int c=0; c<4; ++c){
        Ti.matrix()(r,c)=DualValuef(Ti_.matrix()(r,c), 0.);
        Tj.matrix()(r,c)=DualValuef(Tj_.matrix()(r,c), 0.);
        iOi.matrix()(r,c)=DualValuef(iOi_.matrix()(r,c), 0.);
        Oj.matrix()(r,c)=DualValuef(Oj_.matrix()(r,c), 0.);
      }
    }
    Vector6_<DualValuef> dti, dtj;
    for (int k=0; k<6; ++k){
      dti[k]=DualValuef(0,0);
      dtj[k]=DualValuef(0,0);
    }
    
    for (int k=0; k<6; ++k) {
      dti[k]=DualValuef(0,1);
      Matrix3_<DualValuef> ei=fn(Ti, Tj, iOi, Oj, dti, dtj);
      dti[k]=DualValuef(0,0);
      dtj[k]=DualValuef(0,1);
      Matrix3_<DualValuef> ej=fn(Ti, Tj, iOi, Oj, dti, dtj);
      dtj[k]=DualValuef(0,0);
      for (int r=0; r<3; ++r) {
        for (int c=0; c<3; ++c) {
          Ji(r*3+c,k)=ei(r,c).derivative;
          Jj(r*3+c,k)=ej(r,c).derivative;
        }
      }
    }
  }

}
