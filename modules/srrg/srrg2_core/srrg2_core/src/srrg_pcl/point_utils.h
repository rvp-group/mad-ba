#pragma once
namespace srrg2_core {

  // ia add covariance??
  template <class PointType_>
  struct BaseAggregator_ {
    using Point  = PointType_;
    using Scalar = typename Point::Scalar;

    BaseAggregator_() {
      reset();
    }

    inline void reset() {
      _accumulator.setZero();
      _accumulator.good = true;
      _count            = Scalar(0.);
    }

    void add(const Point& p, const Scalar& s = Scalar(1)) {
      if (!_accumulator.good) {
        return;
      }
      if (!p.good) {
        _accumulator.good = false;
      }
      _accumulator += p;
      _count += s;
    }

    inline bool isGood() {
      if (!_accumulator.good) {
        return false;
      }
      if (_count <= Scalar(0.)) {
        return false;
      }
      return true;
    }

    inline const Point& get() {
      _accumulator *= Scalar(1.) / _count;
      _accumulator.normalize();
      return _accumulator;
    }

  protected:
    Scalar _count;
    Point _accumulator;
  };
} // namespace srrg2_core
