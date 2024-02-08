#pragma once

#include "kd_utils.h"

#include <Eigen/Core>
#include <iostream>
#include <list>
#include <memory>

#include <atomic>

template <typename ContainerType_>
class TreeNode3D {
public:
  using ContainerType    = ContainerType_;
  using ContainerTypePtr = std::shared_ptr<ContainerType>;
  using IteratorType     = typename ContainerType::iterator;
  using AnswerType       = std::vector<Eigen::Vector3d*>;
  using PtrType          = TreeNode3D*;
  using ThisType         = TreeNode3D<ContainerType>;

  inline TreeNode3D(const ContainerTypePtr vec,
             const IteratorType begin,
             const IteratorType end,
             const double bbox_threshold,
             const double flatness_threshold,
             const int level,
             const int max_parallel_level,
             TreeNode3D* parent,
             TreeNode3D* plane_predecessor);

  inline ~TreeNode3D() {
    if (left_)
      delete left_;
    if (right_)
      delete right_;
  }

  void applyTransform(const Eigen::Matrix3d& r, const Eigen::Vector3d& t);

  inline TreeNode3D* bestMatchingLeafFast(const Eigen::Vector3d& query);
  
  inline void build(const ContainerTypePtr vec,
             const IteratorType begin,
             const IteratorType end,
             const double bbox_threshold,
             const double flatness_threshold,
             const int level,
             const int max_parallel_level,
             TreeNode3D* parent,
             TreeNode3D* plane_predecessor);

  static inline ThisType* makeSubtree(const ContainerTypePtr vec,
                               const IteratorType begin,
                               const IteratorType end,
                               const double bbox_threshold,
                               const double flatness_threshold,
                               const int level,
                               const int max_parallel_level,
                               TreeNode3D* parent,
                               TreeNode3D* plane_predecessor);

  inline void getLeafs(std::back_insert_iterator<std::vector<TreeNode3D*>> it);

  int num_points_;
  bool matched_ = false;
  PtrType left_ = nullptr;
  PtrType right_ = nullptr;
  PtrType parent_ = nullptr;
  Eigen::Vector3d mean_;
  Eigen::Vector3d bbox_;
  Eigen::Matrix3d eigenvectors_;

protected:
  TreeNode3D(){};
};

#include "kdtree.hpp"
