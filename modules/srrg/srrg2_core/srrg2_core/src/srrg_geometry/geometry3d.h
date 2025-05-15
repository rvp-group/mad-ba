#pragma once
#include "ad.h"
#include "geometry2d.h"
#include "geometry_defs.h"

namespace srrg2_core {
  namespace geometry3d {

    // [Bart]
    // actually this(template variable) is allowed since c++14
    // should we use c++14 flag?
    // Without the c++14 flag, this delaration generates a warning
    // ds does it make sense to have this templated? I would declare it as
    // double and'e basta ia I took the Dominik path and sticazzi static const
    // double _epsilon = 1e-8; gg addeg c++14 flag :)
    template <typename Scalar_>
    static const Scalar_ _epsilon = Scalar_(1e-8);

    //! skew symmetric matrix
    template <typename Scalar_>
    __host__ __device__ inline Matrix3_<Scalar_> skew(const Vector3_<Scalar_>& v_) {
      Matrix3_<Scalar_> S;
      S << Scalar_(0.), -v_[2], v_[1], v_[2], Scalar_(0.), -v_[0], -v_[1], v_[0], Scalar_(0.);
      return S;
    }

    //! write in dest the skew symmetric matrix from Dim
    template <typename Scalar_>
    inline void makeSkew(Matrix3_<Scalar_>& dest_, const Vector3_<Scalar_>& v_) {
      dest_ << Scalar_(0.), -v_[2], v_[1], v_[2], Scalar_(0.), -v_[0], -v_[1], v_[0], Scalar_(0.);
    }

    //! rotation matrix in x
    template <typename Scalar_>
    inline Matrix3_<Scalar_> rotationX(const Scalar_& angle_) {
      Matrix3_<Scalar_> R;
      const Scalar_ s = sin(angle_);
      const Scalar_ c = cos(angle_);
      R << Scalar_(1.), Scalar_(0.), Scalar_(0.), Scalar_(0.), c, -s, Scalar_(0.), s, c;
      return R;
    }

    //! rotation matrix in y
    template <typename Scalar_>
    inline Matrix3_<Scalar_> rotationY(const Scalar_& angle_) {
      Matrix3_<Scalar_> R;
      const Scalar_ s = sin(angle_);
      const Scalar_ c = cos(angle_);
      R << c, Scalar_(0.), s, Scalar_(0.), Scalar_(1.), Scalar_(0.), -s, Scalar_(0.), c;
      return R;
    }

    //! rotation matrix in z
    template <typename Scalar_>
    inline Matrix3_<Scalar_> rotationZ(const Scalar_& angle) {
      Matrix3_<Scalar_> R;
      const Scalar_ s = sin(angle);
      const Scalar_ c = cos(angle);
      R << c, -s, Scalar_(0.), s, c, Scalar_(0.), Scalar_(0.), Scalar_(0.), Scalar_(1.);
      return R;
    }

    //! 3D rotation matrix from angles
    template <typename Scalar_>
    inline Matrix3_<Scalar_> a2r(const Vector3_<Scalar_>& angles_) {
      return rotationX(angles_[0]) * rotationY(angles_[1]) * rotationZ(angles_[2]);
    }

    //!  angles from 3D rotation matrix
    // mc this will produce errors if works with autodiff
    // mc use quaternion instead or reproduce eigen EulerAngles.h here
    template <typename Scalar_>
    inline Vector3_<Scalar_> r2a(const Matrix3_<Scalar_>& r_) {
      return r_.eulerAngles(0, 1, 2);
    }

    //! 3D rotation from normalized quaternion
    template <typename Scalar_>
    inline Matrix3_<Scalar_> nq2r(const Vector3_<Scalar_>& q_) {
      Scalar_ n = q_.squaredNorm();
      if (n > Scalar_(1.)) {
        return Matrix3_<Scalar_>::Identity();
      }
      Scalar_ w(sqrt(Scalar_(1.) - n));
      Eigen::Quaternion<Scalar_> full_q(w, q_[0], q_[1], q_[2]);
      return full_q.toRotationMatrix();
    }

    // mc
    //! normalized quaternion to 3D rotation
    //! stable version which works also with autodiff
    template <typename Scalar_>
    inline Eigen::Quaternion<Scalar_> r2q(const Matrix3_<Scalar_>& R_) {
      Eigen::Quaternion<Scalar_> q;
      Scalar_ t = R_.trace();
      if (t > Scalar_(0.)) {
        t     = sqrt(t + Scalar_(1.0));
        q.w() = Scalar_(0.5) * t;
        t     = Scalar_(0.5) / t;
        q.x() = (R_.coeff(2, 1) - R_.coeff(1, 2)) * t;
        q.y() = (R_.coeff(0, 2) - R_.coeff(2, 0)) * t;
        q.z() = (R_.coeff(1, 0) - R_.coeff(0, 1)) * t;
      } else {
        int i = 0;
        if (R_.coeff(1, 1) > R_.coeff(0, 0)) {
          i = 1;
        }
        if (R_.coeff(2, 2) > R_.coeff(i, i)) {
          i = 2;
        }
        int j = (i + 1) % 3;
        int k = (j + 1) % 3;

        t = sqrt(R_.coeff(i, i) - R_.coeff(j, j) - R_.coeff(k, k) + Scalar_(1.0));
        q.coeffs().coeffRef(i) = Scalar_(0.5) * t;
        t                      = Scalar_(0.5) / t;
        q.w()                  = (R_.coeff(k, j) - R_.coeff(j, k)) * t;
        q.coeffs().coeffRef(j) = (R_.coeff(j, i) + R_.coeff(i, j)) * t;
        q.coeffs().coeffRef(k) = (R_.coeff(k, i) + R_.coeff(i, k)) * t;
      }
      return q;
    }

    //! 3D rotation from quaternion
    template <typename Scalar_>
    inline Matrix3_<Scalar_> q2r(const Eigen::Quaternion<Scalar_>& q_) {
      return q_.toRotationMatrix();
    }

    // mc
    //! normalized quaternion from 3D rotation
    //! stable version which works also with autodiff
    template <typename Scalar_>
    inline Vector3_<Scalar_> r2nq(const Matrix3_<Scalar_>& R_) {
      Vector3_<Scalar_> nq;
      Eigen::Quaternion<Scalar_> q = r2q(R_);
      q.normalize();
      if (q.w() >= Scalar_(0.)) {
        nq << q.x(), q.y(), q.z();
      } else {
        nq << -q.x(), -q.y(), -q.z();
      }
      return nq;
    }

    template <typename Scalar_>
    inline Vector4_<Scalar_> r2nqw(const Matrix3_<Scalar_>& R_) {
      Vector4_<Scalar_> nq;
      Eigen::Quaternion<Scalar_> q = r2q(R_);
      q.normalize();
      if (q.w() >= Scalar_(0.)) {
        nq << q.w(), q.x(), q.y(), q.z();
      } else {
        nq << -q.w(), -q.x(), -q.y(), -q.z();
      }
      return nq;
    }

    //! isometry from translation+angles
    template <typename Scalar_>
    inline Isometry3_<Scalar_> ta2t(const Vector6_<Scalar_>& pose_) {
      Isometry3_<Scalar_> T = Isometry3_<Scalar_>::Identity();
      T.translation()       = pose_.head(3);
      T.linear()            = a2r(Vector3_<Scalar_>(pose_.tail(3)));
      return T;
    }

    //! translation+angles from isometry
    template <typename Scalar_>
    inline Vector6_<Scalar_> t2ta(const Isometry3_<Scalar_>& iso_) {
      Vector6_<Scalar_> v;
      v.head(3)             = iso_.translation();
      Matrix3_<Scalar_> mat = iso_.linear();
      v.tail(3)             = r2a(mat);
      return v;
    }

    //! @brief ldg similiarity Sim3 from translation+angles+scaling
    template <typename Scalar_>
    inline Similiarity3_<Scalar_> tas2s(const Vector7_<Scalar_>& sim_vec_) {
      Similiarity3_<Scalar_> sim_;
      sim_.translation()    = sim_vec_.head(3);
      sim_.linear()         = a2r(Vector3_<Scalar_>(sim_vec_.template block<3, 1>(3, 0)));
      sim_.inverseScaling() = std::exp(sim_vec_(6));
      return sim_;
    }

    //! @brief ldg translation+angles+scaling from similiarity Sim3
    template <typename Scalar_>
    inline Vector7_<Scalar_> s2tas(const Similiarity3_<Scalar_>& sim_) {
      Vector7_<Scalar_> sim_vec_;
      sim_vec_.template head<3>()         = sim_.translation();
      const Matrix3_<Scalar_>& mat_       = sim_.linear();
      sim_vec_.template block<3, 1>(3, 0) = r2a(mat_);
      sim_vec_(6)                         = std::log(sim_.inverseScaling());
      return sim_vec_;
    }

    //! isometry from translation+normalized quaternion
    template <typename Scalar_>
    inline Isometry3_<Scalar_> tnq2t(const Vector6_<Scalar_>& pose_) {
      Isometry3_<Scalar_> T = Isometry3_<Scalar_>::Identity();
      T.translation()       = pose_.head(3);
      T.linear()            = nq2r(Vector3_<Scalar_>(pose_.tail(3)));
      return T;
    }

    //! translation+normalized quaternion from isometry
    template <typename Scalar_>
    inline Vector6_<Scalar_> t2tnq(const Isometry3_<Scalar_>& iso_) {
      Vector6_<Scalar_> v;
      v.head(3)             = iso_.translation();
      Matrix3_<Scalar_> mat = iso_.linear();
      v.tail(3)             = r2nq(mat);
      return v;
    }

    template <typename Scalar_>
    inline Vector7_<Scalar_> t2tnqw(const Isometry3_<Scalar_>& iso_) {
      Vector7_<Scalar_> v;
      v.head(3)             = iso_.translation();
      Matrix3_<Scalar_> mat = iso_.linear();
      v.tail(4)             = r2nqw(mat);
      return v;
    }

    //! SE3 vector to 3D transform (moving from manifold)
    template <typename Scalar_>
    inline Isometry3_<Scalar_> v2t(const Vector6_<Scalar_>& pose_) {
      return tnq2t(pose_);
    }

    //! @brief ldg Sim3 vector to 3D similiarity j(moving from manifold)
    template <typename Scalar_>
    inline Similiarity3_<Scalar_> v2s(const Vector7_<Scalar_>& sim_vec_) {
      const Isometry3_<Scalar_>& iso_ = tnq2t(Vector6_<Scalar_>(sim_vec_.template head<6>()));
      Similiarity3_<Scalar_> sim_;
      sim_.translation()    = iso_.translation();
      sim_.linear()         = iso_.linear();
      sim_.inverseScaling() = ad::exp(sim_vec_(6));
      return sim_;
    }

    //! 3D transform to SE3 vector (moving to manifold)
    template <typename Scalar_>
    inline Vector6_<Scalar_> t2v(const Isometry3_<Scalar_>& iso) {
      return t2tnq(iso);
    }

    //! 3D transform to SE3 vector (moving to manifold)
    template <typename Scalar_>
    inline Vector7_<Scalar_> s2v(const Similiarity3_<Scalar_>& sim_) {
      Isometry3_<Scalar_> iso_ = Isometry3_<Scalar_>::Identity();
      iso_.translation()       = sim_.translation();
      iso_.linear()            = sim_.linear();
      Vector7_<Scalar_> sim_vec_;
      sim_vec_.template head<6>() = t2tnq(iso_);
      sim_vec_(6)                 = ad::log(sim_.inverseScaling());
      return sim_vec_;
    }

    //! 3D isometry to vector (x, y, z, qw, qx, qy, qz)
    template <typename Scalar_>
    inline Vector7_<Scalar_> t2w(const Isometry3_<Scalar_>& iso) {
      return t2tnqw(iso);
    }

    //! 3D isometry to vector (x, y, z, qx, qy, qz, qw)
    template <typename Scalar_>
    inline Vector7_<Scalar_> t2tqxyzw(const Isometry3_<Scalar_>& iso) {
      Eigen::Quaternion<Scalar_> q(iso.linear());
      q.normalize();

      Vector7_<Scalar_> vector = Vector7_<Scalar_>::Zero();
      vector.head(3)           = iso.translation();
      vector[3]                = q.x();
      vector[4]                = q.y();
      vector[5]                = q.z();
      vector[6]                = q.w();
      return vector;
    }

    //! vector (x, y, z, qx, qy, qz, qw) to 3D isometry
    template <typename Scalar_>
    inline Isometry3_<Scalar_> tqxyzq2t(const Vector7_<Scalar_>& v_) {
      Isometry3_<Scalar_> t;
      t               = Quaternion_<Scalar_>(v_[6], v_[3], v_[4], v_[5]).toRotationMatrix();
      t.translation() = v_.head(3);
      return t;
    }

    //! euclidean(x,y,z) to polar(r,theta,phi) transforms
    template <typename Scalar_>
    inline Vector3_<Scalar_> euclidean2polar(const Vector3_<Scalar_>& src_) {
      Vector3_<Scalar_> polar;
      polar(0) = src_.norm(); // rho
      if (polar(0) > _epsilon<Scalar_>) {
        polar(1) = acos(src_(2) / polar(0));
        polar(2) = atan2(src_(1), src_(0));
        return polar;
      } else {
        return Vector3_<Scalar_>::Zero();
      }
    }

    //! polar(r,theta,phi) to euclidean(x,y,z) transforms
    template <typename Scalar_>
    inline Vector3_<Scalar_> polar2euclidean(const Vector3_<Scalar_>& src_) {
      Vector3_<Scalar_> euclidean;
      const Scalar_ s_theta = sin(src_(1));
      const Scalar_ c_theta = cos(src_(1));
      const Scalar_ s_phi   = sin(src_(2));
      const Scalar_ c_phi   = cos(src_(2));
      const Scalar_& r      = src_(0);
      euclidean.x()         = r * s_theta * c_phi;
      euclidean.y()         = r * s_theta * s_phi;
      euclidean.z()         = r * c_theta;
      return euclidean;
    }

    //! homogeneous division (required in projection function)
    template <typename Scalar_>
    inline Vector3_<Scalar_> hom(const Vector3_<Scalar_>& src_) {
      const Scalar_& z = src_(2);
      if (z < _epsilon<Scalar_>) {
        throw std::runtime_error("[Geometry3d::hom]: z-coord less than epsilon");
      }
      return src_ * (Scalar_(1.) / z);
    }

    //! 3d to 2d transformation
    template <typename Scalar_>
    inline Isometry2_<Scalar_> get2dFrom3dPose(const Isometry3_<Scalar_>& src_) {
      const Matrix3_<Scalar_>& rotation_matrix = src_.linear();
      const Vector3_<Scalar_>& translation     = src_.translation();
      assert(rotation_matrix.row(2).transpose() == Vector3_<Scalar_>(0, 0, 1));
      assert(rotation_matrix.col(2) == Vector3_<Scalar_>(0, 0, 1));

      const Vector3_<Scalar_> euler_angles = r2a(rotation_matrix);
      Vector3_<Scalar_> pose_vector(translation(0), translation(1), euler_angles(2));

      return geometry2d::v2t(pose_vector);
    }

    //! 3d to 2d transformation
    template <typename Scalar_>
    inline Isometry3_<Scalar_> get3dFrom2dPose(const Isometry2_<Scalar_>& src_) {
      Isometry3_<Scalar_> dest;
      dest.matrix().setIdentity();
      dest.linear()(0, 0)   = src_.linear()(0, 0);
      dest.linear()(0, 1)   = src_.linear()(0, 1);
      dest.linear()(1, 0)   = src_.linear()(1, 0);
      dest.linear()(1, 1)   = src_.linear()(1, 1);
      dest.translation()(0) = src_.translation()(0);
      dest.translation()(1) = src_.translation()(1);
      return dest;
    }

    template <typename Scalar_>
    inline Isometry3_<Scalar_> interpolateIsometries(const Scalar_& t_,
                                                     const Isometry3_<Scalar_>& start_,
                                                     const Isometry3_<Scalar_>& end_) {
      Vector7_<Scalar_> start_vector = t2w(start_);
      Vector7_<Scalar_> end_vector   = t2w(end_);
      Vector6_<Scalar_> result_vector;
      // bdc linearly interpolate translation (TODO: fix since this sucks)
      const Scalar_ one_minus_t = Scalar_(1.0) - t_;
      result_vector(0)          = start_vector(0) * one_minus_t + end_vector(0) * t_;
      result_vector(1)          = start_vector(1) * one_minus_t + end_vector(1) * t_;
      result_vector(2)          = start_vector(2) * one_minus_t + end_vector(2) * t_;
      Quaternion_<Scalar_> start(
        start_vector(3), start_vector(4), start_vector(5), start_vector(6));
      Quaternion_<Scalar_> end(end_vector(3), end_vector(4), end_vector(5), end_vector(6));
      // bdc slerp interpolation for rotation
      Quaternion_<Scalar_> interpolated_quaterion = start.slerp(t_, end);
      result_vector(3)                            = interpolated_quaterion.x();
      result_vector(4)                            = interpolated_quaterion.y();
      result_vector(5)                            = interpolated_quaterion.z();
      return v2t(result_vector);
    }

    template <typename Scalar_>
    inline Matrix3_<Scalar_> yawPitch2Rot(const Scalar_& yaw_, const Scalar_& pitch_) {
      const Scalar_ c_a = cos(yaw_);
      const Scalar_ s_a = sin(yaw_);
      const Scalar_ c_b = cos(pitch_);
      const Scalar_ s_b = sin(pitch_);
      Matrix3_<Scalar_> R;
      R << c_a * c_b, -s_a, c_a * s_b, s_a * c_b, c_a, s_a * s_b, -s_b, 0.0, c_b;
      return R;
    }

    //! @brief flatten isometry by cols
    template <typename Scalar_>
    inline Vector12_<Scalar_> flattenByCols(const Isometry3_<Scalar_>& t_) {
      Vector12_<Scalar_> v         = Vector12_<Scalar_>::Zero();
      v.template block<3, 1>(0, 0) = t_.matrix().template block<3, 1>(0, 0);
      v.template block<3, 1>(3, 0) = t_.matrix().template block<3, 1>(0, 1);
      v.template block<3, 1>(6, 0) = t_.matrix().template block<3, 1>(0, 2);
      v.template block<3, 1>(9, 0) = t_.matrix().template block<3, 1>(0, 3);

      return v;
    }

    template <typename Scalar_>
    inline Isometry3_<Scalar_> fromFlattenByCols(const Vector12_<Scalar_>& v_,
                                                 const bool reconditionate_rotation_ = true) {
      Isometry3_<Scalar_> t                 = Isometry3_<Scalar_>::Identity();
      t.matrix().template block<3, 1>(0, 0) = v_.template block<3, 1>(0, 0);
      t.matrix().template block<3, 1>(0, 1) = v_.template block<3, 1>(3, 0);
      t.matrix().template block<3, 1>(0, 2) = v_.template block<3, 1>(6, 0);
      t.matrix().template block<3, 1>(0, 3) = v_.template block<3, 1>(9, 0);

      if (reconditionate_rotation_) {
        const Matrix3_<Scalar_>& R = t.linear();
        Eigen::JacobiSVD<Matrix3_<Scalar_>> svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Matrix3_<Scalar_> R_enforced = svd.matrixU() * svd.matrixV().transpose();
        t.linear()                   = R_enforced;
      }

      return t;
    }

    // implementing exp and log rotation SO3 axis-angle params
    template <typename Scalar_>
    inline Matrix3_<Scalar_> expMapSO3(const Vector3_<Scalar_> omega_) {
      Matrix3_<Scalar_> R;
      Scalar_ theta_square = omega_.dot(omega_);
      Scalar_ theta        = sqrt(theta_square);
      Matrix3_<Scalar_> W  = geometry3d::skew(omega_);
      Matrix3_<Scalar_> K  = W / theta;
      if (theta_square < Scalar_(1e-8)) {
        R = Matrix3_<Scalar_>::Identity() + W;
      } else {
        Scalar_ one_minus_cos = Scalar_(2) * sin(theta / Scalar_(2)) * sin(theta / Scalar_(2));
        R = Matrix3_<Scalar_>::Identity() + sin(theta) * K + one_minus_cos * K * K;
      }
      return R;
    }

    template <typename Scalar_>
    inline Matrix3_<Scalar_> jacobianExpMapSO3(const Vector3_<Scalar_> omega_) {
      Matrix3_<Scalar_> Jr;
      Scalar_ theta_square = omega_.dot(omega_);
      Scalar_ theta        = sqrt(theta_square);
      Matrix3_<Scalar_> W  = geometry3d::skew(omega_);
      if (theta_square < Scalar_(1e-8)) {
        Jr = Matrix3_<Scalar_>::Identity() - Scalar_(0.5) * W;
      } else {
        Scalar_ one_minus_cos = Scalar_(2) * sin(theta / Scalar_(2)) * sin(theta / Scalar_(2));
        Matrix3_<Scalar_> K   = W / (theta * theta);
        Jr =
          Matrix3_<Scalar_>::Identity() - one_minus_cos * K + (theta - sin(theta)) * theta * K * K;
      }
      return Jr;
    }

    template <typename Scalar_>
    inline Vector3_<Scalar_> logMapSO3(const Matrix3_<Scalar_>& R) {
      const Scalar_ &R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
      const Scalar_ &R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
      const Scalar_ &R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

      // Get trace(R)
      const Scalar_ tr = R.trace();
      const Scalar_ pi(M_PI);
      const Scalar_ two(2);

      Vector3_<Scalar_> omega;
      // when trace == -1, i.e., when theta = +-pi, +-3pi, +-5pi, etc.
      // we do something special
      if (tr + Scalar_(1.0) < Scalar_(1e-10)) {
        if (abs(R33 + Scalar_(1.0)) > Scalar_(1e-5)) {
          omega = (pi / sqrt(two + two * R33)) * Vector3_<Scalar_>(R13, R23, Scalar_(1.0) + R33);
        } else if (abs(R22 + Scalar_(1.0)) > Scalar_(1e-5)) {
          omega = (pi / sqrt(two + two * R22)) * Vector3_<Scalar_>(R12, Scalar_(1.0) + R22, R32);
        } else {
          omega = (pi / sqrt(two + two * R11)) * Vector3_<Scalar_>(Scalar_(1.0) + R11, R21, R31);
        }
      } else {
        Scalar_ magnitude;
        const Scalar_ tr_3 = tr - Scalar_(3.0); // always negative
        if (tr_3 < Scalar_(-1e-7)) {
          Scalar_ theta = acos((tr - Scalar_(1.0)) / two);
          magnitude     = theta / (two * sin(theta));
        } else {
          // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
          // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
          magnitude = Scalar_(0.5) - tr_3 * tr_3 / Scalar_(12.0);
        }
        omega = magnitude * Vector3_<Scalar_>(R32 - R23, R13 - R31, R21 - R12);
      }
      return omega;
    }

  } // namespace geometry3d
} // namespace srrg2_core
