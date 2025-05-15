#pragma once
#include "srrg_geometry/geometry3d.h"
#include "srrg_geometry/geometry_defs.h"

#include "point_cloud.h"

using namespace srrg2_core;
using namespace std;

struct Stuff {
  const char* x = 0;
};

ostream& operator<<(ostream& os, const Stuff& u) {
  if (u.x)
    os << u.x << " ";
  return os;
}

using Coordinates3f = PointCoordinatesField_<float, 3>;
using Normal3f      = PointDirectionField_<float, 3>;
using Scalarf       = PointScalarField_<float>;
using StuffField    = PointDefaultField_<Stuff>;

struct MyPointWithNormal3f
  : public PointBase_<3, Coordinates3f, Normal3f, Scalarf, StuffField> {
  using BaseType = PointBase_<3, Coordinates3f, Normal3f, Scalarf, StuffField>;
  MyPointWithNormal3f() {
    setZero();
  }

  MyPointWithNormal3f(const BaseType& other) : BaseType(other) {
  }

  inline MyPointWithNormal3f& operator=(const BaseType& other) {
    BaseType::operator=(other);
    return *this;
  }

  inline Coordinates3f::ValueType& coordinates() {
    return value<0>();
  }
  inline const Coordinates3f::ValueType& coordinates() const {
    return value<0>();
  }

  inline Normal3f::ValueType& normal() {
    return value<1>();
  }
  inline const Normal3f::ValueType& normal() const {
    return value<1>();
  }

  inline Scalarf::ValueType& weight() {
    return value<2>();
  }
  inline const Scalarf::ValueType& weight() const {
    return value<2>();
  }

  inline StuffField::ValueType& stuff() {
    return value<3>();
  }
  inline const StuffField::ValueType& stuff() const {
    return value<3>();
  }

  friend std::ostream& operator<<(std::ostream& os_,
                                  const MyPointWithNormal3f point_);
};

std::ostream& operator<<(std::ostream& os_, const MyPointWithNormal3f point_) {
  point_.toStream(os_);
  return os_;
}

using MyPointWithNormal3fCloud =
  PointCloud_<std::vector<MyPointWithNormal3f,
                          Eigen::aligned_allocator<MyPointWithNormal3f>>>;
