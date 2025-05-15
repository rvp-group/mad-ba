#pragma once
#include "srrg_geometry/geometry_defs.h"

// --------------------- ROS -> SRRG ---------------------//

#define ROS_TO_EIGEN_QUATERNION(dest_, src_) \
  { dest_ = Eigen::Quaternionf(src_.w, src_.x, src_.y, src_.z); }

#define ROS_TO_EIGEN_VEC3(dest_, src_) \
  { dest_ = srrg2_core::Vector3f(src_.x, src_.y, src_.z); }

#define ROS_TO_EIGEN_POSE(dest_, src_)                          \
  {                                                             \
    ROS_TO_EIGEN_VEC3(dest_.translation(), src_.pose.position); \
    srrg2_core::Quaternionf q;                                  \
    ROS_TO_EIGEN_QUATERNION(q, src_.pose.orientation);          \
    dest_.linear() = q.matrix();                                \
  }

#define ROS_TO_SRRG_POSE(dest_, src_)                                 \
  {                                                                   \
    srrg2_core::Isometry3f pose = srrg2_core::Isometry3f::Identity(); \
    ROS_TO_EIGEN_POSE(pose, src_);                                    \
    dest_.setValue(pose);                                             \
  }

#define ROS_TO_SRRG_QUATERNION(dest_, src_) \
  {                                         \
    Eigen::Quaternionf q;                   \
    ROS_TO_EIGEN_QUATERNION(q, src_)        \
    dest_.setValue(q.matrix());             \
  }

#define ROS_TO_SRRG_VEC3(dest_, src_) \
  {                                   \
    srrg2_core::Vector3f v;           \
    ROS_TO_EIGEN_VEC3(v, src_);       \
    dest_.setValue(v);                \
  }

#define ROS_TO_SRRG_VECX(dest_, src_)                                 \
  {                                                                   \
    size_t size = src_.size();                                        \
    dest_.value().resize(size);                                       \
    memcpy(dest_.value().data(), src_.data(), sizeof(double) * size); \
  }

#define ROS_TO_SRRG_MAT3(dest_, src_)                       \
  {                                                         \
    srrg2_core::Matrix3d m = srrg2_core::Matrix3d::Zero();  \
    for (int r = 0; r < 3; ++r) {                           \
      for (int c = 0; c < 3; ++c) {                         \
        m(r, c) = src_[r * 3 + c];                          \
      }                                                     \
    }                                                       \
    dest_.setValue((srrg2_core::Matrix3f) m.cast<float>()); \
  }

#define ROS_TO_SRRG_MAT6(dest_, src_)                       \
  {                                                         \
    srrg2_core::Matrix6d m(src_.data());                    \
    dest_.setValue((srrg2_core::Matrix6f) m.cast<float>()); \
  }

// --------------------- SRRG -> ROS ---------------------//

#define EIGEN_TO_ROS_QUATERNION(dest_, src_) \
  {                                          \
    dest_.x = src_.x();                      \
    dest_.y = src_.y();                      \
    dest_.z = src_.z();                      \
    dest_.w = src_.w();                      \
  }

#define EIGEN_TO_ROS_VEC3(dest_, src_) \
  {                                    \
    dest_.x = src_.x();                \
    dest_.y = src_.y();                \
    dest_.z = src_.z();                \
  }

#define EIGEN_TO_ROS_POSE(dest_, src_)                          \
  {                                                             \
    EIGEN_TO_ROS_VEC3(dest_.pose.position, src_.translation()); \
    srrg2_core::Quaternionf q(src_.linear());                   \
    EIGEN_TO_ROS_QUATERNION(dest_.pose.orientation, q);         \
  }

#define SRRG_TO_ROS_POSE(dest_, src_) \
  { EIGEN_TO_ROS_POSE(dest_, src_.value()); }

#define SRRG_TO_ROS_QUATERNION(dest_, src_)                         \
  {                                                                 \
    const Eigen::Quaternionf& q = Eigen::Quaternionf(src_.value()); \
    EIGEN_TO_ROS_QUATERNION(dest_, q)                               \
  }

#define SRRG_TO_ROS_VEC3(dest_, src_)             \
  {                                               \
    const srrg2_core::Vector3f& v = src_.value(); \
    EIGEN_TO_ROS_VEC3(dest_, v);                  \
  }

#define SRRG_TO_ROS_VECX(dest_, src_)                                 \
  {                                                                   \
    size_t size = src_.value().size();                                \
    dest_.resize(size);                                               \
    memcpy(dest_.data(), src_.value().data(), sizeof(double) * size); \
  }

#define SRRG_TO_ROS_MAT(dest_, src_)                         \
  {                                                          \
    const auto& m=src_.value();                              \
    int k=0;                                                 \
    for (int r=0; r<m.rows(); ++r)                           \
      for (int c=0; c<m.cols(); ++c, ++k)                    \
        dest_[k]=m(r,c);                                     \
  }                                                          \

// --------------------- HEADERS ---------------------//

#define ROS_TO_SRRG_HEADER(dest_, src_)                  \
  {                                                      \
    dest_.frame_id.setValue(src_.header.frame_id);       \
    dest_.seq.setValue(src_.header.seq);                 \
    dest_.timestamp.setValue(src_.header.stamp.toSec()); \
  }

#define SRRG_TO_ROS_HEADER(dest_, src_)                        \
  {                                                            \
    dest_.header.frame_id = src_.frame_id.value();             \
    dest_.header.seq      = src_.seq.value();                  \
    dest_.header.stamp    = ros::Time(src_.timestamp.value()); \
  }
