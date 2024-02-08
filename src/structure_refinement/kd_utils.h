#pragma once

#include <iterator>
#include <Eigen/Core>

template <typename IteratorType, typename PredicateType>
IteratorType split(const IteratorType& begin, const IteratorType& end, const PredicateType& predicate) {
  IteratorType lower = begin;
  std::reverse_iterator<IteratorType> upper = std::make_reverse_iterator(end); 
  while (lower != upper.base()) {
    Eigen::Vector3d& v_lower = *lower;
    Eigen::Vector3d& v_upper = *upper;
    if (predicate(v_lower)) {
      ++lower;
    }
    else {
      std::swap(v_lower,v_upper);
      ++upper;
    }
  }
  return upper.base();
}

template <typename IteratorType>
int computeMeanAndCovariance(Eigen::Vector3d& mean, Eigen::Matrix3d& cov,
  const IteratorType& begin, const IteratorType& end) {
  // mean computed as 1/(end-start) Sum_k={start..end} x_k
  // cov computed as  (1/(end-start) Sum_k={start..end} x_k* x_k^T ) - mean*mean.transpose();
  mean.setZero();
  cov.setZero();
  int k = 0;
  for (IteratorType it = begin; it != end; ++it) {
    const Eigen::Vector3d& v = *it;
    mean += v;
    cov += v * v.transpose();
    ++k;
  }
  mean *= (1./k);
  cov *= (1./k);
  cov -= mean * mean.transpose();
  cov *= double(k)/double(k-1);
  
  return k;
}

template <typename IteratorType>
int computeBoundingBox(Eigen::Vector3d& bbox, const Eigen::Vector3d& center,
  const Eigen::Matrix3d& R, const IteratorType& begin, const IteratorType& end) {
  bbox.setZero();
  int k = 0;
  
  Eigen::Vector3d bbox_neg = Eigen::Vector3d::Zero();
  Eigen::Vector3d bbox_pos = Eigen::Vector3d::Zero();
  for (IteratorType it = begin; it != end; ++it) {
    const Eigen::Vector3d v = R*(*it-center);
    for (int i = 0; i < 3; ++i) {
      bbox_neg(i) = std::min<double>(bbox_neg(i),v(i));
      bbox_pos(i) = std::max<double>(bbox_pos(i),v(i));
    }
    ++k;
  }
  
  bbox = bbox_pos - bbox_neg;
  return k;
}
