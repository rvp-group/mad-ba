#pragma once
#include <cmath>
#include <Eigen/Geometry>

namespace srrg2_core {
  namespace ad {
    /**basic functions for float, redefined to please the compiler*/
    inline float sin(float f) {
      return sinf(f);
    }

    inline float cos(float f) {
      return cosf(f);
    }

    inline float asin(float f) {
      return asinf(f);
    }

    inline float acos(float f) {
      return acosf(f);
    }

    inline float exp(float f) {
      return expf(f);
    }

    inline float log(float f) {
      return logf(f);
    }

    inline float sqrt(float f) {
      return sqrtf(f);
    }

    inline float atan2(float y, float x) {
      return atan2f(y, x);
    }

    inline float pow(float x, float y) {
      return powf(x, y);
    }

    /**basic functions for double, redefined to please the compiler*/
    inline double sin(double f) {
      return sinl(f);
    }

    inline double cos(double f) {
      return cosl(f);
    }

    inline double exp(double f) {
      return expl(f);
    }

    inline double log(double f) {
      return logl(f);
    }

    inline double sqrt(double f) {
      return sqrtl(f);
    }

    inline double atan2(double y, double x) {
      return atan2l(y, x);
    }

    inline double pow(double x, double y) {
      return powl(x, y);
    }

    inline float normalizeAngle(const float& angle_) {
      return atan2(sin(angle_), cos(angle_));
    }
  
    inline double normalizeAngle(const double& angle_) {
      return atan2(sin(angle_), cos(angle_));
    }


    /**dual value, stores the elements of autodiff
       it reperesent a pair
       u, u'
       and defines all common operators	      
       +,-,*,/, and so on

       DualValue is parametiric w.r.t the base type (float or double)
    */

    template <typename T>
    class DualValue_ {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef T BaseType;
      T value;
      T derivative;

      inline operator T() const {
        return value;
      }

      DualValue_() {
        value = 0, derivative = 0;
      }

      DualValue_(const T& v) {
        value = v;
        derivative = 0;
      }

      DualValue_(const T& v, const T& d) {
        value = v;
        derivative = d;
      }

      inline DualValue_& operator=(const T& v) {
        value = v;
        derivative = 0;
        return *this;
      }

      inline DualValue_& operator+=(const DualValue_& op) {
        value += op.value;
        derivative += op.derivative;
        return *this;
      }

      inline DualValue_& operator-=(const DualValue_& op) {
        value -= op.value;
        derivative -= op.derivative;
        return *this;
      }

      inline DualValue_& operator*=(const DualValue_& op) {
        value *= op.value;
        derivative = derivative * op.value + value * op.derivative;
        return *this;
      }

      inline DualValue_& operator/=(const DualValue_& op) {
        value /= op.value;
        derivative = (derivative * op.value - value * op.derivative) / (op.value * op.value);
        return *this;
      }

      inline bool operator>(const DualValue_& v) const {
        return value > v.value;
      }

      inline bool operator>=(const DualValue_& v) const {
        return value >= v.value;
      }

      inline bool operator==(const DualValue_& v) const {
        return value == v.value;
      }

      inline bool operator!=(const DualValue_& v) const {
        return value != v.value;
      }

      inline bool operator<(const DualValue_& v) const {
        return value < v.value;
      }

      inline bool operator<=(const DualValue_& v) const {
        return value <= v.value;
      }

    };

    template <typename T>
    inline std::ostream& operator<<(std::ostream& os, const DualValue_<T>& v) {
      os << "(v: " << v.value << " " << " d: " << v.derivative << ")";
      return os;
    }

    typedef DualValue_<float> DualValuef;
    typedef DualValue_<double> DualValued;

    template <typename T>
    inline DualValue_<T> abs(const DualValue_<T>& op) {
      return DualValue_<T>(std::abs(op.value), (op.value > 0 ? op.derivative : -op.derivative));
    }

    template <typename T>
    inline DualValue_<T> operator+(const DualValue_<T>& op) {
      return op;
    }

    template <typename T>
    inline DualValue_<T> operator-(const DualValue_<T>& op2) {
      return DualValue_<T>(-op2.value, -op2.derivative);
    }

    template <typename T>
    inline DualValue_<T> operator+(const DualValue_<T>& op1, const DualValue_<T>& op2) {
      return DualValue_<T>(op1.value + op2.value, op1.derivative + op2.derivative);
    }

    template <typename T>
    inline DualValue_<T> operator-(const DualValue_<T>& op1, const DualValue_<T>& op2) {
      return DualValue_<T>(op1.value - op2.value, op1.derivative - op2.derivative);
    }

    template <typename T>
    inline DualValue_<T> operator*(const DualValue_<T>& op1, const DualValue_<T>& op2) {
      return DualValue_<T>(op1.value * op2.value, op1.derivative * op2.value + op1.value * op2.derivative);
    }

    template <typename T>
    inline DualValue_<T> operator/(const DualValue_<T>& op1, const DualValue_<T>& op2) {
      return DualValue_<T>(op1.value / op2.value, (op1.derivative * op2.value - op1.value * op2.derivative) / (op2.value * op2.value));
    }

    template <typename T>
    inline DualValue_<T> sin(const DualValue_<T>& op) {
      return DualValue_<T>(sin(op.value), cos(op.value) * op.derivative);
    }

    template <typename T>
    inline DualValue_<T> asin(const DualValue_<T>& op) {
      return DualValue_<T>(asin(op.value), 1./(sqrt(1 - op.value*op.value)) * op.derivative);
    }

    template <typename T>
    inline DualValue_<T> cos(const DualValue_<T>& op) {
      return DualValue_<T>(cos(op.value), -sin(op.value) * op.derivative);
    }

    template <typename T>
    inline DualValue_<T> acos(const DualValue_<T>& op) {
      return DualValue_<T>(acos(op.value), -1./(sqrt(1 - op.value*op.value)) * op.derivative);
    }


    template <typename T>
    inline DualValue_<T> log(const DualValue_<T>& op) {
      return DualValue_<T>(log(op.value), 1. / fabs(op.value) * op.derivative);
    }

    template <typename T>
    inline DualValue_<T> exp(const DualValue_<T>& op) {
      return DualValue_<T>(exp(op.value), exp(op.value) * op.derivative);
    }

    template <typename T>
    inline DualValue_<T> sqrt(const DualValue_<T>& op) {
      return DualValue_<T>(sqrt(op.value), 0.5 / sqrt(op.value) * op.derivative);
    }

    template <typename T>
    inline DualValue_<T> atan2(const DualValue_<T>& op1, const DualValue_<T>& op2) {
      return DualValue_<T>(atan2(op1.value, op2.value),
                           1. / (1 + pow(op1.value / op2.value, 2))
                           * (op1.derivative * op2.value - op1.value * op2.derivative)
                           / (op2.value * op2.value));
    }



    template <typename DestType_, typename SrcType_>
    void convertMatrix(DestType_& dest, const SrcType_& src) {
      if (src.cols() != dest.cols()) {
        throw std::runtime_error("cols size mismatch");
      }
      if (src.rows() != dest.rows()) {
        throw std::runtime_error("rows size mismatch");
      }
      typedef typename DestType_::Scalar DestScalar;
      for (int c = 0; c < src.cols(); ++c) {
        for (int r = 0; r < src.rows(); ++r) {
          dest(r, c) = DestScalar(src(r, c));
        }
      }
    }
  }

  using DualValuef = ad::DualValue_<float> ;
  using DualValued = ad::DualValue_<double> ;
}

