#pragma once
#include "ad.h"
#include "geometry3d.h"

namespace srrg2_core {

  template <typename Scalar_>
  class Quaternionad_ : public Eigen::Quaternion<Scalar_> {
  public:
    using ScalarType=Scalar_;
    static const ScalarType QUATERNION_DOT_THRESHOLD;
    Quaternionad_<Scalar_>(const ScalarType& qw_, const ScalarType& qx_, const ScalarType& qy_, const ScalarType& qz_) {
      this->w() = qw_;
      this->x() = qx_;
      this->y() = qy_;
      this->z() = qz_;
    }
    Quaternionad_<Scalar_>(const Vector4_<Scalar_>& vector_wxyz_) {
      this->w() = vector_wxyz_(0);
      this->x() = vector_wxyz_(1);
      this->y() = vector_wxyz_(2);
      this->z() = vector_wxyz_(3);
    }
    Quaternionad_<Scalar_>(const Quaternionad_<Scalar_>& q_) {
      this->w() = q_.w();
      this->x() = q_.x();
      this->y() = q_.y();
      this->z() = q_.z();
    }

    void normalize() {
      const ScalarType n = sqrt(this->w()*this->w() +
                                         this->x()*this->x() +
                                         this->y()*this->y() +
                                         this->z()*this->z());
      this->w() /= n;
      this->x() /= n;
      this->y() /= n;
      this->z() /= n;
    }

    ScalarType dotProduct(const Quaternionad_<Scalar_>& q1_) {
      return (this->x() * q1_.x() + this->y() * q1_.y() + this->z() * q1_.z() + this->w() * q1_.w());
    }

    Quaternionad_<Scalar_> slerp(const ScalarType& t_, const Quaternionad_<Scalar_>& q1_) {
      // Normalize to avoid undefined behavior.
      Quaternionad_<Scalar_> q1_cpy(q1_);
      q1_cpy.normalize();
      this->normalize();

      ScalarType dot = this->dotProduct(q1_);

      if(dot < ScalarType(0.f)) {
        q1_cpy *= ScalarType(-1.0);
        dot = -dot;
      }
      //bdc if really close
      if(dot > QUATERNION_DOT_THRESHOLD) {
        //bdc linear interpolate
        return Quaternionad_<Scalar_>(*this + (q1_cpy - *this) * t_);
      }

      //bdc from here on, acos is safe to be used
      ScalarType theta_0 = acos(dot);
      ScalarType theta = theta_0 * t_;
      ScalarType sin_theta = sin(theta);
      ScalarType sin_theta_0 = sin(theta_0);
      ScalarType s0 = cos(theta) - dot * sin_theta / sin_theta_0;
      ScalarType s1 = sin_theta / sin_theta_0;

      return Quaternionad_<Scalar_>(((*this) * s0) + (q1_ * s1));
    }

    inline Quaternionad_<Scalar_>& operator=(const Quaternionad_<Scalar_>& q) {
      this->x() = q.x();
      this->y() = q.y();
      this->z() = q.z();
      this->w() = q.w();
      return *this;
    }

    inline Quaternionad_<Scalar_>& operator+=(const Quaternionad_<Scalar_>& q) {
      this->x() += q.x();
      this->y() += q.y();
      this->z() += q.z();
      this->w() += q.w();
      return *this;
    }

    inline Quaternionad_<Scalar_>& operator-=(const Quaternionad_<Scalar_>& q) {
      this->x() -= q.x();
      this->y() -= q.y();
      this->z() -= q.z();
      this->w() -= q.w();
      return *this;
    }


    inline Quaternionad_<Scalar_>& operator*=(const ScalarType& s) {
      this->x() *= s;
      this->y() *= s;
      this->z() *= s;
      this->w() *= s;
      return *this;
    }

    inline Quaternionad_<Scalar_>& operator/=(const ScalarType& s) {
      this->x() /= s;
      this->y() /= s;
      this->z() /= s;
      this->w() /= s;
      return *this;
    }
  };


  template <typename Scalar_>
  inline Quaternionad_<Scalar_> operator+(const Quaternionad_<Scalar_>& op_) {
    return op_;
  }

  template <typename Scalar_>
  inline Quaternionad_<Scalar_> operator-(const Quaternionad_<Scalar_>& q_) {
    return Quaternionad_<Scalar_>(-q_.w(), -q_.x(), -q_.y(), -q_.z());
  }

  template <typename Scalar_>
  inline Quaternionad_<Scalar_> operator+(const Quaternionad_<Scalar_>& op1,
                                          const Quaternionad_<Scalar_>& op2) {
    return Quaternionad_<Scalar_>(op1.w() + op2.w(),
                                  op1.x() + op2.x(),
                                  op1.y() + op2.y(),
                                  op1.z() + op2.z());
  }

  template <typename Scalar_>
  inline Quaternionad_<Scalar_> operator-(const Quaternionad_<Scalar_>& op1,
                                          const Quaternionad_<Scalar_>& op2) {
    return Quaternionad_<Scalar_>(op1.w() - op2.w(),
                                  op1.x() - op2.x(),
                                  op1.y() - op2.y(),
                                  op1.z() - op2.z());
  }

  template <typename Scalar_>
  inline Quaternionad_<Scalar_> operator*(const Quaternionad_<Scalar_>& op1,
                                          const Scalar_& op2) {
    return Quaternionad_<Scalar_>(op1.w() * op2,
                                  op1.x() * op2,
                                  op1.y() * op2,
                                  op1.z() * op2);
  }

  template <typename Scalar_>
  inline Quaternionad_<Scalar_> operator/(const Quaternionad_<Scalar_>& op1,
                                          const Scalar_& op2) {
    return Quaternionad_<Scalar_>(op1.w() / op2,
                                  op1.x() / op2,
                                  op1.y() / op2,
                                  op1.z() / op2);
  }

  template <typename Scalar_>
  Scalar_ const Quaternionad_<Scalar_>::QUATERNION_DOT_THRESHOLD=Scalar_(0.9995);
  
  using Quaternionadf=Quaternionad_<DualValuef>;
}
