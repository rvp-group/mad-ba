#pragma once
#include "geometry_defs.h"

namespace srrg2_core {
  namespace geometry2d {

    //! 2D rotation matrix from angle
    template <typename Scalar_>
    inline Matrix2_<Scalar_> a2r(const Scalar_& theta_) {
      Scalar_ c = cos(theta_);
      Scalar_ s = sin(theta_);
      Matrix2_<Scalar_> R;
      R << c, -s, s, c;
      return R;
    }

    //! angle from 2D rotation matrix
    template <typename Scalar_>
    inline Scalar_ r2a(const Matrix2_<Scalar_>& R_) {
      return atan2(R_(1, 0), R_(0, 0));
    }

    //! skew symmetric matrix from one vector
    template <typename Scalar_>
    inline Vector2_<Scalar_> skew(const Vector2_<Scalar_>& v_) {
      return Vector2_<Scalar_>(v_(1), -v_(0));
    }

    //! SE2 vector to 2D transform (moving from manifold)
    template <typename Scalar_>
    inline Isometry2_<Scalar_> v2t(const Vector3_<Scalar_>& pose_) {
      Isometry2_<Scalar_> iso = Isometry2_<Scalar_>::Identity();
      iso.translation().x()   = pose_.x();
      iso.translation().y()   = pose_.y();
      iso.linear()            = a2r(pose_(2));
      return iso;
    }

    //! @brief ldg Sim2 vector to 2D similiarity (moving from manifold)
    template <typename Scalar_>
    inline Similiarity2_<Scalar_> v2s(const Vector4_<Scalar_>& sim_vec_) {
      const Isometry2_<Scalar_>& iso_ = v2t(Vector3f(sim_vec_.template head<3>()));
      Similiarity2_<Scalar_> sim_;
      sim_.linear()         = iso_.linear();
      sim_.translation()    = iso_.translation();
      sim_.inverseScaling() = std::exp(sim_vec_(3));
      return sim_;
    }

    //! 2D transform to SE2 vector (moving to manifold)
    template <typename Scalar_>
    inline Vector3_<Scalar_> t2v(const Isometry2_<Scalar_>& iso_) {
      Vector3_<Scalar_> v;
      v.head(2)                  = iso_.translation();
      const Matrix2_<Scalar_>& R = iso_.linear();
      v(2)                       = r2a(R);
      return v;
    }

    //! @brief ldg similiarity 2D transform to SE2 vector (moving to manifold)
    template <typename Scalar_>
    inline Vector4_<Scalar_> s2v(const Similiarity2_<Scalar_>& sim_) {
      Vector4_<Scalar_> v;
      v.template head<2>()       = sim_.translation();
      const Matrix2_<Scalar_>& R = sim_.linear();
      v(2)                       = r2a(R);
      v(3)                       = std::log(sim_.inverseScaling());
      return v;
    }

    //! 2D euclidean to polar coordinates conversion [theta, rho]
    template <typename Scalar_>
    inline Vector2_<Scalar_> euclidean2polar(const Vector2_<Scalar_>& src_) {
      Vector2_<Scalar_> polar;
      polar(1) = src_.norm();
      polar(0) = atan2(src_(1), src_(0));
      return polar;
    }

    //! polar [theta, rho] to 2D euclidean coordinates conversion
    template <typename Scalar_>
    inline Vector2_<Scalar_> polar2euclidean(const Vector2_<Scalar_>& src_,
                                             const bool swap_ = false) {
      Vector2_<Scalar_> euclidean;
      euclidean(0) = src_(1) * cos(src_(0));
      euclidean(1) = src_(1) * sin(src_(0));
      return euclidean;
    }
  } // namespace geometry2d
} // namespace srrg2_core
