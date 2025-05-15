#pragma once
#include "srrg_data_structures/correspondence.h"
#include "srrg_messages/instances.h"
#include "srrg_pcl/instances.h"
#include "test_helper.hpp"

namespace srrg2_test {

  // ds test fixture
  template <size_t Dimension_,
            typename RealType_,
            typename PointType_          = srrg2_core::Point_<Dimension_, RealType_>,
            typename PointProjectedType_ = srrg2_core::Point_<Dimension_ - 1, RealType_>>
  class SyntheticWorld : public ::testing::Test {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using VectorType          = srrg2_core::Vector_<RealType_, Dimension_>;
    using VectorProjectedType = srrg2_core::Vector_<RealType_, Dimension_ - 1>;
    using MatrixType          = srrg2_core::MatrixN_<RealType_, Dimension_>;
    using TransformType       = srrg2_core::Isometry_<RealType_, Dimension_>;
    using PointType           = PointType_;
    using PointProjectedType  = PointProjectedType_;
    using VectorPointCloudType =
      srrg2_core::PointCloud_<std::vector<PointType, Eigen::aligned_allocator<PointType>>>;
    using VectorPointProjectedCloudType = srrg2_core::PointCloud_<
      std::vector<PointProjectedType, Eigen::aligned_allocator<PointProjectedType>>>;
    using VectorTransformType = std::vector<TransformType, Eigen::aligned_allocator<TransformType>>;
    enum class SensorType { Laser, Camera };

    SyntheticWorld() {
      clear();
    }

    virtual ~SyntheticWorld() {
    }

    void generateMap(const size_t& number_of_points_,
                     const VectorType& mean_,
                     const VectorType& standard_deviation_,
                     const SensorType& type_) {
      ASSERT_GT(projection_matrix.norm(), 0);

      // ds sample world points
      points_in_world.clear();
      points_in_world.reserve(number_of_points_);
      for (size_t i = 0; i < number_of_points_; ++i) {
        PointType point_in_world;
        point_in_world.coordinates() = generator.getRandomVector(mean_, standard_deviation_);
        points_in_world.emplace_back(point_in_world);
      }

      // ds if no sensor poses are set we sample the first one at identity
      if (sensor_poses.empty()) {
        sensor_poses.push_back(TransformType::Identity());
      }
      const size_t number_of_poses = sensor_poses.size();

      // ds compute projective baseline (assuming no rotation)
      baseline_to_second_sensor_pixelmeters = projection_matrix * offset_second_sensor;
      // ds sample scenario for each sensor pose
      points_in_sensor.resize(number_of_poses);
      points_in_sensor_projected_homo.resize(number_of_poses);
      points_in_sensor_projected_homo_second.resize(number_of_poses);
      points_in_sensor_projected.resize(number_of_poses);
      points_in_sensor_projected_second.resize(number_of_poses);
      points_in_sensor_projected_in_canvas.resize(number_of_poses);
      points_in_sensor_projected_in_canvas_second.resize(number_of_poses);
      correspondences_sensor_to_world.resize(number_of_poses);
      correspondences_canvas_to_sensor.resize(number_of_poses);
      correspondences_canvas_to_world.resize(number_of_poses);
      maximum_projection_value.resize(number_of_poses, 0);

      for (size_t index_pose = 0; index_pose < number_of_poses; ++index_pose) {
        const TransformType& sensor_in_local_map = sensor_poses[index_pose];
        const TransformType local_map_in_sensor  = sensor_in_local_map.inverse();

        // ds current sensor pose
        points_in_sensor[index_pose].reserve(number_of_points_);
        points_in_sensor_projected_homo[index_pose].reserve(number_of_points_);
        points_in_sensor_projected_homo_second[index_pose].reserve(number_of_points_);
        points_in_sensor_projected[index_pose].reserve(number_of_points_);
        points_in_sensor_projected_second[index_pose].reserve(number_of_points_);
        points_in_sensor_projected_in_canvas[index_pose].reserve(number_of_points_);
        points_in_sensor_projected_in_canvas_second[index_pose].reserve(number_of_points_);
        correspondences_sensor_to_world[index_pose].reserve(number_of_points_);
        correspondences_canvas_to_sensor[index_pose].reserve(number_of_points_);
        correspondences_canvas_to_world[index_pose].reserve(number_of_points_);

        // ds pinhole camera check - we can only have a measurement per pixel (no depth buffer)
        std::set<size_t> occupied_rows;
        std::set<size_t> occupied_cols;

        // ds transform and project points into sensor frame
        for (size_t i = 0; i < number_of_points_; ++i) {
          PointType point_in_sensor;
          point_in_sensor.coordinates() = local_map_in_sensor * points_in_world[i].coordinates();
          PointType point_in_sensor_projected_homo;
          point_in_sensor_projected_homo.coordinates() =
            projection_matrix * point_in_sensor.coordinates();
          PointType point_in_sensor_projected_homo_second;
          // srrg I don't know why this minus here make things work
          // maybe because we are left multiplying the sensor
          point_in_sensor_projected_homo_second.coordinates() =
            point_in_sensor_projected_homo.coordinates() - baseline_to_second_sensor_pixelmeters;

          // ds check visibility criteria depending on sensor type TODO proper implementation
          PointProjectedType point_projected;
          PointProjectedType point_projected_second;
          if (type_ == SensorType::Laser) {
            // ds assuming laser - range measurement must be positive
            if (point_in_sensor_projected_homo.coordinates()(0) <= 0 ||
                point_in_sensor_projected_homo_second.coordinates()(0) <= 0) {
              continue;
            }

            // ds set range measurement
            for (size_t j = 0; j < Dimension_ - 1; ++j) {
              point_projected.coordinates()(j) = point_in_sensor_projected_homo.coordinates()(j);
              if (point_projected.coordinates()(j) > maximum_projection_value[index_pose]) {
                maximum_projection_value[index_pose] = point_projected.coordinates()(j);
              }
            }
            for (size_t j = 0; j < Dimension_ - 1; ++j) {
              point_projected_second.coordinates()(j) =
                point_in_sensor_projected_homo_second.coordinates()(j);
            }
          } else if (type_ == SensorType::Camera) {
            // ds assuming pinhole camera - all homogeneous values must be positive
            if (point_in_sensor.coordinates()(0) <= 0 || point_in_sensor.coordinates()(1) <= 0 ||
                point_in_sensor.coordinates()(2) <= min_camera_depth) {
              continue;
            }

            // ds perform homogeneous division for raw measurements
            const RealType_ normalizer =
              point_in_sensor_projected_homo.coordinates()(Dimension_ - 1);
            assert(normalizer ==
                   point_in_sensor_projected_homo_second.coordinates()(Dimension_ - 1));
            for (size_t j = 0; j < Dimension_ - 1; ++j) {
              point_projected.coordinates()(j) =
                point_in_sensor_projected_homo.coordinates()(j) / normalizer;
              if (point_projected.coordinates()(j) > maximum_projection_value[index_pose]) {
                maximum_projection_value[index_pose] = point_projected.coordinates()(j);
              }
              point_projected_second.coordinates()(j) =
                point_in_sensor_projected_homo_second.coordinates()(j) / normalizer;
            }

            // ds check if the pixel is still not available for addition (TODO multidim)
            // ds this is kind of a depth buffer uagh
            const size_t col = std::round(point_projected.coordinates()(0));
            const size_t row = std::round(point_projected.coordinates()(1));
            if (occupied_rows.count(row) != 0 && occupied_cols.count(col) != 0) {
              continue;
            } else {
              occupied_rows.insert(row);
              occupied_cols.insert(col);
            }
          }

          // ds point is valid if still here
          const size_t index_moving = points_in_sensor[index_pose].size();
          points_in_sensor[index_pose].emplace_back(point_in_sensor);
          points_in_sensor_projected_homo[index_pose].emplace_back(point_in_sensor_projected_homo);
          points_in_sensor_projected_homo_second[index_pose].emplace_back(
            point_in_sensor_projected_homo_second);
          points_in_sensor_projected[index_pose].emplace_back(point_projected);
          points_in_sensor_projected_second[index_pose].emplace_back(point_projected_second);
          correspondences_sensor_to_world[index_pose].emplace_back(
            srrg2_core::Correspondence(index_moving, i));

          // ds canvas check - since image coordinates are not normalized and centered
          // dsthey are positive including 0
          bool is_inside_canvas = true;
          for (size_t j = 0; j < Dimension_ - 1; ++j) {
            // ds if either projection is outside of the available image plane
            if (point_projected.coordinates()(j) >= canvas_size(j) ||
                point_projected_second.coordinates()(j) >= canvas_size(j)) {
              is_inside_canvas = false;
            }
          }
          if (is_inside_canvas) {
            const size_t index_fixed = points_in_sensor_projected_in_canvas[index_pose].size();
            assert(index_fixed == points_in_sensor_projected_in_canvas_second[index_pose].size());
            points_in_sensor_projected_in_canvas[index_pose].emplace_back(point_projected);
            points_in_sensor_projected_in_canvas_second[index_pose].emplace_back(
              point_projected_second);
            correspondences_canvas_to_sensor[index_pose].emplace_back(
              srrg2_core::Correspondence(index_fixed, index_moving));
            correspondences_canvas_to_world[index_pose].emplace_back(
              srrg2_core::Correspondence(index_fixed, i));
          }
        }
      }
    }

    void generateTrajectory(const size_t& number_of_poses_,
                            const TransformType& start_,
                            const TransformType& goal_,
                            const VectorType& standard_deviation_) {
      sensor_poses.reserve(number_of_poses_);
      sensor_poses.emplace_back(start_);

      // ds compute total translation and orientation delta required to arrive at goal
      if (number_of_poses_ > 2) {
        const TransformType motion(start_.inverse() * goal_);
        const VectorType translation_step(motion.translation() / (number_of_poses_ - 1));
        TransformType pose(start_);

        // ds slerp along
        for (size_t i = 0; i < number_of_poses_ - 2; ++i) {
          pose.translation() += generator.getRandomVector(translation_step, standard_deviation_);
          sensor_poses.emplace_back(pose);
        }
      }

      sensor_poses.emplace_back(goal_);
      assert(sensor_poses.size() == number_of_poses_);
    }

    void clear() {
      sensor_poses.clear();
      points_in_world.clear();
      points_in_sensor.clear();
      points_in_sensor_projected_homo.clear();
      points_in_sensor_projected_homo_second.clear();
      points_in_sensor_projected.clear();
      points_in_sensor_projected_second.clear();
      points_in_sensor_projected_in_canvas.clear();
      points_in_sensor_projected_in_canvas_second.clear();
      correspondences_sensor_to_world.clear();
      correspondences_canvas_to_sensor.clear();
      correspondences_canvas_to_world.clear();
    }
    /*
        void generateMessages() {
          using namespace srrg2_core;
          size_t number_of_message_packs = sensor_poses.size();

          TransformEventsMessagePtr tf_event(new TransformEventsMessage(topic_tf));

          Isometry3f camera_right_in_left    = Isometry3f::Identity();
          camera_right_in_left.translation() = offset_second_sensor.template cast<float>();

          TransformEvent tf_camera_in_base_frame(
            0, frame_camera_left, sensor_in_robot.template cast<float>(), frame_base);

          TransformEvent tf_camera_right_in_left(
            0, frame_camera_right, camera_right_in_left, frame_camera_left);
          TransformEvent tf_camera_depth_in_left(
            0, frame_camera_depht, camera_right_in_left, frame_camera_left);
          tf_event->events.resize(3);

          tf_event->events.setValue(0, tf_camera_in_base_frame);
          tf_event->events.setValue(1, tf_camera_right_in_left);
          tf_event->events.setValue(2, tf_camera_depth_in_left);

          for (int i = 0; i < number_of_message_packs; ++i) {
            MessagePackPtr message_pack(new MessagePack());
            // messages[0] is TF
            message_pack->messages.push_back(tf_event);
            ImageMessagePtr image_message_left(
              new ImageMessage(topic_camera_left, frame_camera_left, 0, 0.0));
            ImageMessagePtr image_message_right(
              new ImageMessage(topic_camera_left, frame_camera_left, 0, 0.0));
            ImageMessagePtr image_message_depht(
              new ImageMessage(topic_camera_depht, frame_camera_depht, 0, 0.0));
          }
        }
    */
  protected:
    void SetUp() override {
      srrg2_core::point_cloud_registerTypes();
      //      srrg2_core::messages_registerTypes();
    }
    void TearDown() override {
    }

    // ds current world points
    VectorPointCloudType points_in_world;

    // ds sensor projective matrix (from world to sensor frame)
    MatrixType projection_matrix = MatrixType::Zero();

    // ds secondary projective sensor offset (w.r.t. first sensor)
    // ds can be used to generate a rigid stereo scenario (redundant if not used)
    VectorType offset_second_sensor                  = VectorType::Zero();
    VectorType baseline_to_second_sensor_pixelmeters = VectorType::Zero();
    RealType_ min_camera_depth                       = 1;

    // ds sensor poses w.r.t. world
    VectorTransformType sensor_poses;
    srrg2_core::Isometry3_<RealType_> sensor_in_robot;
    srrg2_core::Isometry3_<RealType_> robot_in_sensor;

    // ds current sensor points before/after projection
    std::vector<VectorPointCloudType> points_in_sensor;
    std::vector<VectorPointCloudType> points_in_sensor_projected_homo;
    std::vector<VectorPointCloudType> points_in_sensor_projected_homo_second;
    std::vector<VectorPointProjectedCloudType> points_in_sensor_projected;
    std::vector<VectorPointProjectedCloudType> points_in_sensor_projected_second;
    std::vector<srrg2_core::CorrespondenceVector> correspondences_sensor_to_world;

    // ds optional canvas size and points that land within it
    VectorProjectedType canvas_size = VectorProjectedType::Zero();
    std::vector<VectorPointProjectedCloudType> points_in_sensor_projected_in_canvas;
    std::vector<VectorPointProjectedCloudType> points_in_sensor_projected_in_canvas_second;
    std::vector<srrg2_core::CorrespondenceVector> correspondences_canvas_to_sensor;
    std::vector<srrg2_core::CorrespondenceVector> correspondences_canvas_to_world;

    // ds maximum single, projected value obtained
    std::vector<RealType_> maximum_projection_value;

    // ds random number generator
    srrg2_test::RandomNumberGeneratorUniform<RealType_> generator;
    /*
        // srrg messages
        std::vector<srrg2_core::BaseSensorMessagePtr> messages;
        const std::string topic_tf           = "tf";
        const std::string topic_camera_left  = "camera_left";
        const std::string topic_camera_right = "camera_right";
        const std::string topic_camera_depht = "camera_depht";
        const std::string frame_camera_left  = "camera_left";
        const std::string frame_camera_right = "camera_right";
        const std::string frame_camera_depht = "camera_depht";
        const std::string frame_base         = "base_frame";
    */
  };
  using SyntheticWorldSE3 = SyntheticWorld<3, float>;
  using SyntheticWorldSE2 = SyntheticWorld<2, float>;

} // namespace srrg2_test
