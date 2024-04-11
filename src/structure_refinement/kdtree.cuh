#pragma once

#include "kd_utils.h"

#include <Eigen/Core>
#include <iostream>
#include <list>
#include <memory>

#include <atomic>

class Managed {
public:
  void *operator new(size_t len) {
    void *ptr;
    cudaMallocManaged(&ptr, len);
    cudaDeviceSynchronize();
    return ptr;
  }

  void operator delete(void *ptr) {
    cudaDeviceSynchronize();
    cudaFree(ptr);
  }
};

template <typename ContainerType_>
class TreeNode3D : public Managed {
public:
  using ContainerType    = ContainerType_;
  using ContainerTypePtr = std::shared_ptr<ContainerType>;
  using IteratorType     = typename ContainerType::iterator;
  using AnswerType       = std::vector<Eigen::Vector3d*>;
  using PtrType          = TreeNode3D*;
  using ThisType         = TreeNode3D<ContainerType>;

  inline TreeNode3D(
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

  __host__ __device__ inline TreeNode3D* bestMatchingLeafFast(const Eigen::Vector3d& query);

  __host__ __device__ int doNothing(int a ){ return 2*a;}

  // __host__ __device__ void TreeNode3D<ContainerType_>::bestMatchingEveryLeaf(const TreeNode3D<ContainerType_> & kdTreeToMatch);

 __host__ __device__ void bestTest(const Eigen::Vector3d& query) {
  TreeNode3D* node = this;
  while (node->left_ || node->right_) {
    const Eigen::Vector3d& _split_plane_normal = node->eigenvectors_.col(2);
    node = node->left_;//((query - node->mean_).dot(_split_plane_normal) < double(0.0)) ? node->left_ : node->right_;
  }
}

  
  inline void build(
             const IteratorType begin,
             const IteratorType end,
             const double bbox_threshold,
             const double flatness_threshold,
             const int level,
             const int max_parallel_level,
             TreeNode3D* parent,
             TreeNode3D* plane_predecessor);

  static inline ThisType* makeSubtree(
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
  int surfel_id_ = -1;
  int poseId = -1;
  // TreeNode3D(const TreeNode3D &s){
  //   num_points_ = s.num_points_;
  //   matched_ = s.matched_;
  //   mean_ = s.mean_;
  //   bbox_ = s.bbox_;
  //   eigenvectors_ = s.eigenvectors_;
  // }
// protected:
  TreeNode3D(){};
};

#include "kdtree.hpp"
