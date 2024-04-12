#pragma once

#include "kd_utils.h"
#include "kdtree.cuh"

#include <Eigen/Eigenvalues>
#include <future>
#include <fstream>

using namespace std;

template <typename ContainerType_>
inline TreeNode3D<ContainerType_>::TreeNode3D(
                                       const IteratorType begin,
                                       const IteratorType end,
                                       const double bbox_threshold,
                                       const double flatness_threshold,
                                       const int level,
                                       const int max_parallel_level,
                                       TreeNode3D* parent,
                                       TreeNode3D* plane_predecessor,
                                       int pointCloudId) {
  build(
        begin,
        end,
        bbox_threshold,
        flatness_threshold,
        level,
        max_parallel_level,
        parent,
        plane_predecessor,
        pointCloudId);
}

template <typename ContainerType_>
inline void TreeNode3D<ContainerType_>::build(
                                       const IteratorType begin,
                                       const IteratorType end,
                                       const double bbox_threshold,
                                       const double flatness_threshold,
                                       const int level,
                                       const int max_parallel_level,
                                       TreeNode3D* parent,
                                       TreeNode3D* plane_predecessor,
                                       int pointCloudId) {
  pointcloud_id_ = pointCloudId;
  parent_ = parent;
  Eigen::Matrix3d cov;
  computeMeanAndCovariance(mean_, cov, begin, end);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
  es.computeDirect(cov);
  // es.compute(cov);
  eigenvectors_ = es.eigenvectors();
  // Check the direction of a vector and flip it, so that it points to a LiDAR
  if (eigenvectors_.col(0).dot(mean_) > 0) {
      eigenvectors_ *= -1;
  }
  num_points_ = computeBoundingBox(bbox_, mean_, eigenvectors_.transpose(), begin, end);

  if (bbox_(2) < bbox_threshold) {
    if (plane_predecessor) {
      eigenvectors_.col(0) = plane_predecessor->eigenvectors_.col(0);
    }
    else {
      if (num_points_ < 3) {
        TreeNode3D* node = this;
        while (node->parent_ && node->num_points_ < 3)
          node = node->parent_;
        eigenvectors_.col(0) = node->eigenvectors_.col(0);
      }
    }

    Eigen::Vector3d& nearest_point = *begin;
    double shortest_dist = std::numeric_limits<double>::max();
    for (IteratorType it = begin; it != end; ++it) {
      const Eigen::Vector3d& v = *it;
      const double dist = (v - mean_).norm();
      if (dist < shortest_dist) {
        nearest_point = v;
        shortest_dist = dist;
      }
    }
    mean_ = nearest_point;

    return;
  }
  if (!plane_predecessor) {
    if (bbox_(0) < flatness_threshold)
      plane_predecessor = this;
  }

  const Eigen::Vector3d& _split_plane_normal = eigenvectors_.col(2);
  IteratorType middle = split(begin, end, [&](const Eigen::Vector3d& p) -> bool {
    return (p - mean_).dot(_split_plane_normal) < double(0);
  });

  if (level >= max_parallel_level) {
    left_ = new TreeNode3D(
                           begin,
                           middle,
                           bbox_threshold,
                           flatness_threshold,
                           level + 1,
                           max_parallel_level,
                           this,
                           plane_predecessor,
                           pointCloudId);

    right_ = new TreeNode3D(
                            middle,
                            end,
                            bbox_threshold,
                            flatness_threshold,
                            level + 1,
                            max_parallel_level,
                            this,
                            plane_predecessor,
                            pointCloudId);
  } else {
    std::future<ThisType*> l = std::async(ThisType::makeSubtree,
                                         
                                          begin,
                                          middle,
                                          bbox_threshold,
                                          flatness_threshold,
                                          level + 1,
                                          max_parallel_level,
                                          this,
                                          plane_predecessor,
                                          pointCloudId);

    std::future<ThisType*> r = std::async(ThisType::makeSubtree,
                                        
                                          middle,
                                          end,
                                          bbox_threshold,
                                          flatness_threshold,
                                          level + 1,
                                          max_parallel_level,
                                          this,
                                          plane_predecessor,
                                          pointCloudId);
    left_ = l.get();
    right_ = r.get();
  }

}

template <typename ContainerType_>
inline TreeNode3D<ContainerType_>* TreeNode3D<ContainerType_>::makeSubtree(
                                                                    const IteratorType begin,
                                                                    const IteratorType end,
                                                                    const double bbox_threshold,
                                                                    const double flatness_threshold,
                                                                    const int level,
                                                                    const int max_parallel_level,
                                                                    TreeNode3D* parent,
                                                                    TreeNode3D* plane_predecessor,
                                                                    int pointCloudId) {
  return new TreeNode3D(
                        begin,
                        end,
                        bbox_threshold,
                        flatness_threshold,
                        level,
                        max_parallel_level,
                        parent,
                        plane_predecessor,
                        pointCloudId);
}

template <typename ContainerType_>
inline TreeNode3D<ContainerType_>*
TreeNode3D<ContainerType_>::bestMatchingLeafFast(const Eigen::Vector3d& query) {
  TreeNode3D* node = this;
  while (node->left_ || node->right_) {
    const Eigen::Vector3d& _split_plane_normal = node->eigenvectors_.col(2);
    node = ((query - node->mean_).dot(_split_plane_normal) < double(0)) ? node->left_
                                                                             : node->right_;
  }

  return node;
}

template <typename ContainerType_>
inline void TreeNode3D<ContainerType_>::getLeafs(std::back_insert_iterator<std::vector<TreeNode3D*>> it) {
  if (!left_ && !right_) {
    ++it = this;
    return;
  }
  if (left_)
    left_->getLeafs(it);
  if (right_)
    right_->getLeafs(it);
}

template <typename ContainerType_>
inline void TreeNode3D<ContainerType_>::applyTransform(const Eigen::Matrix3d& r,
                                                const Eigen::Vector3d& t) {
  mean_         = r * mean_ + t;
  eigenvectors_ = r * eigenvectors_;
  if (left_)
    left_->applyTransform(r, t);
  if (right_)
    right_->applyTransform(r, t);
}

// template <typename ContainerType_>
// inline TreeNode3D<ContainerType_>*
// TreeNode3D<ContainerType_>::bestMatchingEveryLeaf(const TreeNode3D<ContainerType_> & kdTreeToMatch)
// {
//   if (!left_ && !right_) {
//     this->bestMatchingEveryLeaf(kdTreeToMatch);
//     return;
//   }
//   if (left_)
//     left_->bestMatchingEveryLeaf(kdTreeToMatch);
//   if (right_)
//     right_->bestMatchingEveryLeaf(kdTreeToMatch);
// }