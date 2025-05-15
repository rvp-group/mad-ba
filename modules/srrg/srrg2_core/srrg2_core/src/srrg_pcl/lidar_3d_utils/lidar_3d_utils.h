#pragma once
#include <vector>

#include "lidar_3d_sensor.h"
#include "srrg_data_structures/matrix.h"
#include "srrg_pcl/point.h"

namespace srrg2_core {
  namespace srrg2_lidar3d_utils {

    //! @brief this class contains some static function that could help to do something
    class Lidar3DUtils {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      //! @brief given a point cloud, separates it on different rings
      //! @param[in] cloud_: src cloud
      //! @param[in] rings_: vector of point clouds. each element of the vector is a ring
      template <LIDAR_TYPE lidar_type_, typename SrcVectorCloudType_>
      static void generateRings(
        const SrcVectorCloudType_& cloud_,
        std::vector<SrcVectorCloudType_, Eigen::aligned_allocator<SrcVectorCloudType_>>& rings_) {
        const size_t cloud_size = cloud_.size();
        if (!cloud_size) {
          throw std::runtime_error("Lidar3DUtils::generateRings|ERROR, empty src cloud");
        }

        // ia instanciate a LidarSensor
        Lidar3DSensor_<lidar_type_> lidar;
        const float interpolation_factor = float(lidar.verticalResolution() - 1) /
                                           (lidar.verticalFOVUpper() - lidar.verticalFOVLower());

        // ia compute start and ending angle of the ring
        float start_ori = -atan2(cloud_[0].coordinates().y(), cloud_[0].coordinates().x());
        float end_ori   = -atan2(cloud_[cloud_size - 1].coordinates().y(),
                               cloud_[cloud_size - 1].coordinates().x()) +
                        2 * float(M_PI);
        if (end_ori - start_ori > 3 * M_PI) {
          end_ori -= 2 * M_PI;
        } else if (end_ori - start_ori < M_PI) {
          end_ori += 2 * M_PI;
        }

        // ia we need an empty thing
        if (rings_.size()) {
          rings_.clear();
        }

        // ia we know the exact number of rings
        rings_.resize(lidar.verticalResolution());

        // ia we suppose that there are a certain amount of points per ring
        const size_t target_number_of_points_per_ring = 4100;
        for (auto& r : rings_) {
          r.reserve(target_number_of_points_per_ring);
        }

        // ia new point
        srrg2_core::Point3f point;
        bool half_passed = false;

        // bdc extract valid points from input cloud
        for (size_t i = 0; i < cloud_size; i++) {
          // bdc rotate velodyne points to be camera compliant
          point.coordinates().x() = cloud_[i].coordinates().y();
          point.coordinates().y() = cloud_[i].coordinates().z();
          point.coordinates().z() = cloud_[i].coordinates().x();

          // bdc skip NaN and INF valued points
          if (std::isnan(point.coordinates().x()) || std::isnan(point.coordinates().y()) ||
              std::isnan(point.coordinates().z())) {
            continue;
          }

          // bdc skip zero valued points
          if (point.coordinates().x() * point.coordinates().x() +
                point.coordinates().y() * point.coordinates().y() +
                point.coordinates().z() * point.coordinates().z() <
              1e-3) {
            continue;
          }

          // bdc calculate vertical point angle and scan ID
          const float angle =
            atan(point.coordinates().y() / sqrt(point.coordinates().x() * point.coordinates().x() +
                                                point.coordinates().z() * point.coordinates().z()));
          int ring_idx = ((angle - lidar.verticalFOVLower()) * interpolation_factor + 0.5f);

          if (ring_idx >= (int) lidar.verticalResolution() || ring_idx < 0) {
            continue;
          }

          // calculate horizontal point angle
          float ori = -atan2(point.coordinates().x(), point.coordinates().z());
          if (!half_passed) {
            if (ori < start_ori - M_PI / 2)
              ori += 2 * M_PI;
            else if (ori > start_ori + M_PI * 3 / 2)
              ori -= 2 * M_PI;

            if (ori - start_ori > M_PI)
              half_passed = true;

          } else {
            ori += 2 * M_PI;

            if (ori < end_ori - M_PI * 3 / 2)
              ori += 2 * M_PI;
            else if (ori > end_ori + M_PI / 2)
              ori -= 2 * M_PI;
          }

          // calculate relative scan time based on point orientation (if you have an
          // IMU)
          //    float realTime = 0.1 * (ori - startOri) / (endOri - startOri);
          //    projectPointToStartOfSweep(point, relTime);

          // ia put the new point inside the proper ring
          rings_[ring_idx].emplace_back(point);
        }
      }

      //! @brief given a point cloud removes the horizontal flat surfaces (e.g. floor-ceiling)
      //!        point cloud should be already organized in a matrix
      //!        naive implementation of the paper serafin2016iros
      //!        (http://jacoposerafin.com/wp-content/uploads/serafin16iros.pdf)
      template <typename MatrixCloudType_>
      static void removeFlatRegions(MatrixCloudType_& organized_cloud_,
                                    const size_t& num_points_threshold_,
                                    const float& ground_distance_threshold_) {
        const size_t& rows = organized_cloud_.rows();
        const size_t& cols = organized_cloud_.cols();
        // bdc super inefficient, but it's exactly how's described in sple paper
        // (Serafin, Olson, Grisetti)
        srrg2_core::Matrix_<int8_t> vertical;
        vertical.resize(rows, cols);
        vertical.fill(0);

        for (size_t c = 0; c < cols; ++c) {
          for (size_t r = 0; r < rows; ++r) {
            // if already labeled, continue
            if (vertical(r, c)) {
              continue;
            }
            auto& current_p         = organized_cloud_(r, c);
            size_t num_valid_points = 0;
            std::vector<size_t> good_points_on_column;
            good_points_on_column.reserve(rows);
            // insert the current point
            good_points_on_column.emplace_back(r);
            // given the current point, look for others on the column
            for (size_t rr = r + 1; rr < rows; ++rr) {
              const auto& candidate_p = organized_cloud_(rr, c);
              const float ground_distance =
                (candidate_p.coordinates().head(2) - current_p.coordinates().head(2)).squaredNorm();
              // good point
              if (ground_distance < ground_distance_threshold_) {
                ++num_valid_points;
                good_points_on_column.emplace_back(rr);
              }
            }
            // vertical strip
            if (num_valid_points >= num_points_threshold_) {
              for (const size_t& row_index : good_points_on_column) {
                vertical(row_index, c) = 1;
              }
            } else {
              current_p.status = srrg2_core::POINT_STATUS::Invalid;
              current_p.setZero();
            }
          }
        }
      }
    };

  } // namespace srrg2_lidar3d_utils
} // namespace srrg2_core
