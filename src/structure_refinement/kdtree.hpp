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
                                       const float bbox_threshold,
                                       const float flatness_threshold,
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
                                       const float bbox_threshold,
                                       const float flatness_threshold,
                                       const int level,
                                       const int max_parallel_level,
                                       TreeNode3D* parent,
                                       TreeNode3D* plane_predecessor,
                                       int pointCloudId) {
  pointcloud_id_ = pointCloudId;
  parent_ = parent;
  Eigen::Matrix3f cov;
  computeMeanAndCovariance(mean_, cov, begin, end);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
  es.computeDirect(cov);
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

    Eigen::Vector3f& nearest_point = *begin;
    float shortest_dist = std::numeric_limits<float>::max();
    for (IteratorType it = begin; it != end; ++it) {
      const Eigen::Vector3f& v = *it;
      const float dist = (v - mean_).norm();
      if (dist < shortest_dist) {
        nearest_point = v;
        shortest_dist = dist;
      }
    }
    mean_ = nearest_point;

        // ***************** BEGIN OF BEAM SIMULATION ***************** //
    const float beam_divergence_deg = 0.35; // os0
    // const float beam_divergence_deg = 0.18; // os1

    const float beam_divergence = beam_divergence_deg * M_PI / 180.0;
    const int root_num_beams = 11; // must be odd
    const float beam_delta = beam_divergence / float(root_num_beams-1);

    const float mean_azimuth = std::atan2(mean_(1), mean_(0));
    const float mean_elevation = std::asin(mean_(2) / mean_.norm());
    const Eigen::Vector3f mean_direction = mean_ / mean_.norm();

    std::vector<float> waveform_ranges;
    for (int i = -root_num_beams / 2; i <= root_num_beams / 2; ++i) {
      const float azimuth = mean_azimuth + i * beam_delta;
      for (int j = -root_num_beams / 2; j <= root_num_beams / 2; ++j) {
        const float elevation = mean_elevation + j * beam_delta;
        const Eigen::Vector3f direction(std::cos(azimuth) * std::cos(elevation),
                                         std::sin(azimuth) * std::cos(elevation),
                                         std::sin(elevation));
        const Eigen::Vector3f direction_normalized = direction / direction.norm();

        const float mean_dot = mean_direction.dot(direction_normalized);
        const float angle = std::acos(mean_dot);

        // If line is inside the cone
        if (angle < beam_divergence / 2.0) {

          Eigen::Vector3f plane_point = mean_;
          Eigen::Vector3f plane_normal = eigenvectors_.col(0);
          Eigen::Vector3f line_direction = direction_normalized;

          // Compute the intersection point of the plane and the line
          const float denominator = plane_normal.dot(line_direction);
          if (std::abs(denominator) < 1e-6) {
            continue;
          }
          const float numerator = plane_normal.dot(plane_point);
          const float d = numerator / denominator;
          const Eigen::Vector3f intersection_point = d * line_direction;

          const float range = intersection_point.norm();
          waveform_ranges.push_back(range);
        }
      }
    }

    float std_dev = 0;
    const float mean_range = mean_.norm();
    for (const float& range : waveform_ranges) {
      std_dev += std::pow(range - mean_range, 2);
    }
    if (!waveform_ranges.empty()) {
      std_dev = std::sqrt(std_dev / waveform_ranges.size());
    }
    const float meas_sucks_with_std_dev = 0.25; // more than half of sub-beams are below 0.25m of error
    weight_ = std::min(std_dev, meas_sucks_with_std_dev); // cut the std_dev to be at most bad as meas_sucks_with_std_dev
    weight_ = weight_ / meas_sucks_with_std_dev; // normalize weight_ to be between 0 and 1
    weight_ = 1. - weight_; // flip the weight_

    // ***************** END OF BEAM SIMULATION ***************** //

    return;
  }
  if (!plane_predecessor) {
    if (bbox_(0) < flatness_threshold)
      plane_predecessor = this;
  }

  const Eigen::Vector3f& _split_plane_normal = eigenvectors_.col(2);
  IteratorType middle = split(begin, end, [&](const Eigen::Vector3f& p) -> bool {
    return (p - mean_).dot(_split_plane_normal) < float(0);
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
                                                                    const float bbox_threshold,
                                                                    const float flatness_threshold,
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
TreeNode3D<ContainerType_>::bestMatchingLeafFast(const Eigen::Vector3f& query) {
  TreeNode3D* node = this;
  while (node->left_ || node->right_) {
    const Eigen::Vector3f& _split_plane_normal = node->eigenvectors_.col(2);
    node = ((query - node->mean_).dot(_split_plane_normal) < float(0)) ? node->left_
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
inline void TreeNode3D<ContainerType_>::applyTransform(const Eigen::Matrix3f& r,
                                                       const Eigen::Vector3f& t) {
  mean_ = r * mean_ + t;
  eigenvectors_ = r * eigenvectors_;
  if (left_)
    left_->applyTransform(r, t);
  if (right_)
    right_->applyTransform(r, t);
}

template <typename ContainerType_>
inline void TreeNode3D<ContainerType_>::setSurfelId(int id) {
  if (surfel_id_ == -1)
    surfel_id_ = id;
  else {
    std::cout << "Error: trying to change existing surfel id " << std::endl;
    exit(0);
  }
}

template <typename ContainerType_>
inline void TreeNode3D<ContainerType_>::resetSurfelId() {
  surfel_id_ = -1;
}