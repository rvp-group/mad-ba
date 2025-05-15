#pragma once

/*
#include "../geometry_defs.h"

//ds TODO: decide if we want the .hpp - I think it does not provide much more clarity and detaches functions from the context

namespace srrg2_core {

template <typename Scalar_, int Dim_>
struct Point_{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<Scalar_, Dim_, 1> coordinates;
  typedef Scalar_ Scalar;
  static const int Dim=Dim_;
  inline void setZero(){
    coordinates.setZero();
  }
  
  inline Point_<Scalar_, Dim_>& transformInPlace(const Isometry3_<Scalar_>& t){
    coordinates=t*coordinates;
    return *this;
  }
  inline Point_<Scalar_, Dim_>  transform(const Isometry3_<Scalar_>& t) const {
    Point_<Scalar_, Dim_> p;
    p.coordinates=t*coordinates;
    return p;
  }
  inline Point_<Scalar_, Dim_>& normalize(){
    return *this;
  }
  inline Point_<Scalar_, Dim_>  operator*(const Scalar_& s) const {
    Point_<Scalar_, Dim_> p;
    return p*s;
  }
  inline Point_ <Scalar_, Dim_>& operator*=(const Scalar_& s){
    coordinates*=s;
    return *this;
  }
  inline Point_<Scalar_, Dim_>  operator+(const Point_ <Scalar_, Dim_>& other) const {
    Point_ <Scalar_, Dim_> p(*this);
    p.coordintes+=other.coordinates;
    return p;
  }
  inline Point_<Scalar_, Dim_>& operator+=(const Point_ <Scalar_, Dim_>& other){
    coordinates+=other.coordinates;
    return *this;
  }
  
  static void registerType() {
    typedef Point_<Scalar_, Dim_> ThisType;
    addField(ThisType, coordinates);
  }
};

template <typename Scalar_, int Dim_>
struct PointWithIntensity_{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Scalar_ Scalar;
  static const int Dim=Dim_;
  Eigen::Matrix<Scalar_, Dim_, 1> coordinates;
  float intensity;

  inline void setZero(){
    coordinates.setZero();
    intensity=Scalar_(0);
  }

  inline PointWithIntensity_<Scalar_, Dim_>& transformInPlace(const Isometry3_<Scalar_>& t){
    coordinates=t*coordinates;
    return *this;
  }
  inline PointWithIntensity_<Scalar_, Dim_>  transform(const Isometry3_<Scalar_>& t) const {
    PointWithIntensity_<Scalar_, Dim_> p;
    p.coordinates=t*coordinates;
    p.intensity=intensity;
    return p;
  }
  inline PointWithIntensity_<Scalar_, Dim_>& normalize(){
    return *this;
  }
  inline PointWithIntensity_ <Scalar_, Dim_>  operator*(const Scalar_& s) const {
    PointWithIntensity_<Scalar_, Dim_> p(*this);
    p.coordinates*=s;
    p.intensity*=s;
    return p;
  }
  inline PointWithIntensity_ <Scalar_, Dim_>& operator*=(const Scalar_& s){
    coordinates*=s;
    intensity*=s;
    return *this;
  }
  inline PointWithIntensity_ <Scalar_, Dim_>  operator+(const PointWithIntensity_ <Scalar_, Dim_>& other) const {
    PointWithIntensity_ <Scalar_, Dim_> p(*this);
    p.coordintes+=other.coordinates;
    p.intensity+=other->intensity;
    return p;
  }
  inline PointWithIntensity_ <Scalar_, Dim_>& operator+=(const PointWithIntensity_ <Scalar_, Dim_>& other){
    coordinates+=other.coordinates;
    intensity+=other.intensity;
    return *this;
  }

  
  static void registerType() {
    typedef PointWithIntensity_<Scalar_, Dim_> ThisType;
    addField(ThisType, coordinates);
    addField(ThisType, intensity);
  }
};

template <typename Scalar_, int Dim_>
struct PointWithRGB_{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const int Dim=Dim_;
  typedef Scalar_ Scalar;
  Eigen::Matrix<Scalar_, Dim_, 1> coordinates;
  Eigen::Vector3f rgb;

  inline void setZero(){
    coordinates.setZero();
    rgb.setZero();
  }

  inline PointWithRGB_<Scalar_, Dim_>& transformInPlace(const Isometry3_<Scalar_>& t){
    coordinates=t*coordinates;
    return *this;
  }
  inline PointWithRGB_<Scalar_, Dim_>  transform(const Isometry3_<Scalar_>& t) const {
    PointWithRGB_<Scalar_, Dim_> p;
    p.coordinates=t*coordinates;
    p.rgb=rgb;
    return p;
  }
  inline PointWithRGB_<Scalar_, Dim_>& normalize(){
    return *this;
  }
  inline PointWithRGB_ <Scalar_, Dim_>  operator*(const Scalar_& s) const {
    PointWithRGB_<Scalar_, Dim_> p(*this);
    p.coordinates*=s;
    p.rgb*=s;
    return p;
  }
  inline PointWithRGB_ <Scalar_, Dim_>& operator*=(const Scalar_& s){
    coordinates*=s;
    rgb*=s;
    return *this;
  }
  inline PointWithRGB_ <Scalar_, Dim_>  operator+(const PointWithRGB_ <Scalar_, Dim_>& other) const {
    PointWithRGB_ <Scalar_, Dim_> p(*this);
    p.coordintes+=other.coordinates;
    p.rgb+=other->rgb;
    return p;
  }
  inline PointWithRGB_ <Scalar_, Dim_>& operator+=(const PointWithRGB_ <Scalar_, Dim_>& other){
    coordinates+=other.coordinates;
    rgb+=other.rgb;
    return *this;
  }
  
  static void registerType() {
    typedef PointWithRGB_<Scalar_, Dim_> ThisType;
    addField(ThisType, coordinates);
    addField(ThisType, rgb);
  }
};

template <typename Scalar_, int Dim_>
struct PointWithNormal_{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const int Dim=Dim_;
  typedef Scalar_ Scalar;
  Eigen::Matrix<Scalar_, Dim_, 1> coordinates;
  Eigen::Matrix<Scalar_, Dim_, 1> normal;

  inline void setZero() {
    coordinates.setZero();
    normal.setZero();
  }
  
  inline PointWithNormal_<Scalar_, Dim_>& transformInPlace(const Isometry3_<Scalar_>& t){
    coordinates=t*coordinates;
    normal=t.linear()*normal;
    return *this;
  }
  inline PointWithNormal_<Scalar_, Dim_>  transform(const Isometry3_<Scalar_>& t) const {
    PointWithNormal_<Scalar_, Dim_> p;
    p.coordinates=t*coordinates;
    p.normal=t*normal;
    return p;
  }
  inline PointWithNormal_<Scalar_, Dim_>& normalize(){
    if (normal.squaredNorm()>Scalar_(0.))
      normal.normalize();
    return *this;
  }
  inline PointWithNormal_ <Scalar_, Dim_>  operator*(const Scalar_& s) const {
    PointWithNormal_<Scalar_, Dim_> p(*this);
    p.coordinates*=s;
    p.normal*=s;
    return p;
  }
  inline PointWithNormal_ <Scalar_, Dim_>& operator*=(const Scalar_& s){
    coordinates*=s;
    normal*=s;
    return *this;
  }
  inline PointWithNormal_ <Scalar_, Dim_>  operator+(const PointWithNormal_ <Scalar_, Dim_>& other) const {
    PointWithNormal_ <Scalar_, Dim_> p(*this);
    p.coordintes+=other.coordinates;
    p.normal+=other.normal;
    return p;
  }
  inline PointWithNormal_ <Scalar_, Dim_>& operator+=(const PointWithNormal_ <Scalar_, Dim_>& other){
    coordinates+=other.coordinates;
    normal+=other.normal;
    return *this;
  }
  static void registerType() {
    typedef PointWithNormal_<Scalar_, Dim_> ThisType;
    addField(ThisType, coordinates);
    addField(ThisType, normal);
  }
};

template <typename Scalar_, int Dim_>
struct PointWithNormalIntensity_{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const int Dim=Dim_;
  typedef Scalar_ Scalar;
  Eigen::Matrix<Scalar_, Dim_, 1> coordinates;
  Eigen::Matrix<Scalar_, Dim_, 1> normal;
  float intensity;

  inline void setZero() {
    coordinates.setZero();
    normal.setZero();
    intensity=Scalar_(0);
  }

  inline PointWithNormalIntensity_<Scalar_, Dim_>& transformInPlace(const Isometry3_<Scalar_>& t){
    coordinates=t*coordinates;
    normal=t.linear()*normal;
    return *this;
  }
  inline PointWithNormalIntensity_<Scalar_, Dim_>  transform(const Isometry3_<Scalar_>& t) const {
    PointWithNormalIntensity_<Scalar_, Dim_> p;
    p.coordinates=t*coordinates;
    p.normal=t*normal;
    return p;
  }
  inline PointWithNormalIntensity_<Scalar_, Dim_>& normalize(){
    if (normal.squaredNorm()>Scalar_(0.))
        normal.normalize();
    return *this;
  }
  inline PointWithNormalIntensity_ <Scalar_, Dim_>  operator*(const Scalar_& s) const {
    PointWithNormalIntensity_<Scalar_, Dim_> p(*this);
    p.coordinates*=s;
    p.normal*=s;
    p.intensity*=s;
    return p;
  }
  inline PointWithNormalIntensity_ <Scalar_, Dim_>& operator*=(const Scalar_& s){
    coordinates*=s;
    normal*=s;
    intensity*=s;
    return *this;
  }
  inline PointWithNormalIntensity_ <Scalar_, Dim_>  operator+(const PointWithNormalIntensity_ <Scalar_, Dim_>& other) const {
    PointWithNormalIntensity_ <Scalar_, Dim_> p(*this);
    p.coordintes+=other.coordinates;
    p.normal+=other->normal;
    p.intensity+=intensity;
    return p;
  }
  inline PointWithNormalIntensity_ <Scalar_, Dim_>& operator+=(const PointWithNormalIntensity_ <Scalar_, Dim_>& other){
    coordinates+=other.coordinates;
    normal+=normal.rgb;
    intensity+=other.intensity;
    return *this;
  }
  static void registerType() {
    typedef PointWithNormalIntensity_<Scalar_, Dim_> ThisType;
    addField(ThisType, coordinates);
    addField(ThisType, normal);
    addField(ThisType, intensity);
  }
};

template <typename Scalar_, int Dim_>
struct PointWithNormalRGB_{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const int Dim=Dim_; 
  typedef Scalar_ Scalar;
  Eigen::Matrix<Scalar_, Dim_, 1> coordinates;
  Eigen::Matrix<Scalar_, Dim_, 1> normal;
  Eigen::Vector3f rgb;

  inline PointWithNormalRGB_<Scalar_, Dim_>& transformInPlace(const Isometry3_<Scalar_>& t){
    coordinates=t*coordinates;
    normal=t.linear()*normal;
    return *this;
  }
  inline PointWithNormalRGB_<Scalar_, Dim_>  transform(const Isometry3_<Scalar_>& t) const {
    PointWithNormalRGB_<Scalar_, Dim_> p;
    p.coordinates=t*coordinates;
    p.normal=t*normal;
    return p;
  }
  inline PointWithNormalRGB_<Scalar_, Dim_>& normalize(){
    if (normal.squaredNorm()>Scalar_(0.))
        normal.normalize();
    return *this;
  }
  inline PointWithNormalRGB_ <Scalar_, Dim_>  operator*(const Scalar_& s) const {
    PointWithNormalRGB_<Scalar_, Dim_> p(*this);
    p.coordinates*=s;
    p.normal*=s;
    p.rgb*=s;
    return p;
  }
  inline PointWithNormalRGB_ <Scalar_, Dim_>& operator*=(const Scalar_& s){
    coordinates*=s;
    normal*=s;
    rgb*=s;
    return *this;
  }
  inline PointWithNormalRGB_ <Scalar_, Dim_>  operator+(const PointWithNormalRGB_ <Scalar_, Dim_>& other) const {
    PointWithNormalRGB_ <Scalar_, Dim_> p(*this);
    p.coordintes+=other.coordinates;
    p.normal+=other->normal;
    p.rgb+=rgb;
    return p;
  }
  inline PointWithNormalRGB_ <Scalar_, Dim_>& operator+=(const PointWithNormalRGB_ <Scalar_, Dim_>& other){
    coordinates+=other.coordinates;
    normal+=normal.rgb;
    rgb+=other.rgb;
    return *this;
  }

  static void registerType() {
    typedef PointWithNormalRGB_<Scalar_, Dim_> ThisType;
    addField(ThisType, coordinates);
    addField(ThisType, normal);
    addField(ThisType, rgb);
  }
};

typedef Point_<int, 2> Point2i;
typedef PointWithIntensity_<int, 2> PointWithIntensity2i;
typedef PointWithRGB_<int, 2> PointWithRGB2i;
typedef PointWithNormal_<int, 2> PointWithNormal2i;
typedef PointWithNormalIntensity_<int, 2> PointWithNormalIntensity2i;
typedef PointWithNormalRGB_<int, 2> PointWithNormalRGB2i;

typedef Point_<float, 2> Point2f;
typedef PointWithIntensity_<float, 2> PointWithIntensity2f;
typedef PointWithRGB_<float, 2> PointWithRGB2f;
typedef PointWithNormal_<float, 2> PointWithNormal2f;
typedef PointWithNormalIntensity_<float, 2> PointWithNormalIntensity2f;
typedef PointWithNormalRGB_<float, 2> PointWithNormalRGB2f;

typedef Point_<double, 2> Point2d;
typedef PointWithIntensity_<double, 2> PointWithIntensity2d;
typedef PointWithRGB_<double, 2> PointWithRGB2d;
typedef PointWithNormal_<double, 2> PointWithNormal2d;
typedef PointWithNormalIntensity_<double, 2> PointWithNormalIntensity2d;
typedef PointWithNormalRGB_<double, 2> PointWithNormalRGB2d;

typedef Point_<float, 3> Point3f;
typedef PointWithIntensity_<float, 3> PointWithIntensity3f;
typedef PointWithRGB_<float, 3> PointWithRGB3f;
typedef PointWithNormal_<float, 3> PointWithNormal3f;
typedef PointWithNormalIntensity_<float, 3> PointWithNormalIntensity3f;
typedef PointWithNormalRGB_<float, 3> PointWithNormalRGB3f;

typedef Point_<double, 3> Point3d;
typedef PointWithIntensity_<double, 3> PointWithIntensity3d;
typedef PointWithRGB_<double, 3> PointWithRGB3d;
typedef PointWithNormal_<double, 3> PointWithNormal3d;
typedef PointWithNormalIntensity_<double, 3> PointWithNormalIntensity3d;
typedef PointWithNormalRGB_<double, 3> PointWithNormalRGB3d;

void registerPointTypes();
}
*/
