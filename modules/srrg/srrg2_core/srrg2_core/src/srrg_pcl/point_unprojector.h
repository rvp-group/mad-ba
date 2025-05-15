#pragma once
#include <limits>

#include "camera_matrix_owner.h"
#include "point_cloud.h"
#include "point_matrix_processor.h"
#include "srrg_data_structures/matrix.h"
#include "srrg_system_utils/profiler.h"

namespace srrg2_core {

  enum PointUnprojectorMode { NoNormals = 1, WithNormals = 2 };

  template <typename DestPointCloudType_>
  class PointUnprojectorBase_
    : public CameraMatrixOwner_<DestPointCloudType_::PointType::GeometryDim> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType           = PointUnprojectorBase_<DestPointCloudType_>;
    using BaseType           = Configurable;
    using DestPointCloudType = DestPointCloudType_;
    using DestPointType      = typename DestPointCloudType::PointType;
    using DestMatrixType =
      PointCloud_<Matrix_<DestPointType, Eigen::aligned_allocator<DestPointType>>>;
    static constexpr int GeometryDim  = DestPointType::GeometryDim;
    static constexpr int ProjectedDim = DestPointType::GeometryDim - 1;
    using ImageCoordinatesType        = Vector_<float, ProjectedDim>;
    using IsometryType                = Isometry_<float, GeometryDim>;

    PARAM(PropertyFloat,
          range_min,
          "min range allowed for unprojection [m]",
          0.f,
          &this->_config_changed);
    PARAM(PropertyFloat,
          range_max,
          "max range allowed for unprojection [m]",
          std::numeric_limits<float>::max(),
          &this->_config_changed);

    // projects all points to the dest matrix, by preserving the order
    // virtual void compute(DestMatrixType& dest, SrcMatrixType& src)=0;

    // template <typename OutputIterator>
    // virtual void compute(OutputIterator& dest, const SrcMatrixType& src)=0;

  protected:
    //! iteratively copy point field values from channel data (dense)
    template <int dest_field_idx,
              typename SrcMatrixChannelType,
              typename... SrcMatrixChannelRestType>
    static inline void _copyFields(DestPointType& dest,
                                   int r,
                                   int c,
                                   const SrcMatrixChannelType& channel,
                                   const SrcMatrixChannelRestType&... rest) {
      dest.template value<dest_field_idx>() = channel.at(r, c);
      _copyFields<dest_field_idx + 1, SrcMatrixChannelRestType...>(dest, r, c, rest...);
    }
    template <int dest_field_idx>
    static inline void _copyFields(DestPointType& dest, int r, int c) {
      // ds last iteration (nothing to copy)
    }
  };

  template <TRANSFORM_CLASS transform_class_, typename DestPointCloudType_>
  class PointUnprojector_ : public PointUnprojectorBase_<DestPointCloudType_>, public Profiler {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType                                   = PointUnprojectorBase_<DestPointCloudType_>;
    static constexpr TRANSFORM_CLASS transform_class = transform_class_;
    using DestPointCloudType                         = DestPointCloudType_;
    using DestPointType                              = typename DestPointCloudType::PointType;
    using DestMatrixType =
      PointCloud_<Matrix_<DestPointType, Eigen::aligned_allocator<DestPointType>>>;
    static constexpr int GeometryDim  = DestPointType::GeometryDim;
    static constexpr int ProjectedDim = DestPointType::GeometryDim - 1;
    using ImageCoordinatesType        = Vector_<float, ProjectedDim>;
    using IsometryType                = Isometry_<float, GeometryDim>;
    using UnprojectionType            = Transform_<float, GeometryDim>;
    using CameraMatrixType            = Eigen::Matrix<float, GeometryDim, GeometryDim>;

    PARAM(PropertyInt, num_ranges, "number of laser beams", 721, &this->_config_changed);
    PARAM(PropertyFloat, angle_min, "start angle  [rad]", -M_PI, &this->_config_changed);
    PARAM(PropertyFloat, angle_max, "end angle    [rad]", M_PI, &this->_config_changed);
    PARAM(PropertyFloat,
          normal_point_distance,
          "range of points considered while computing normal",
          0.2,
          &this->_config_changed);
    PARAM(PropertyInt,
          normal_min_points,
          "minimum number of points in ball when computing a valid normal",
          5,
          &this->_config_changed);

    PointUnprojector_() : BaseType() {
      setDefaultCameraMatrix();
    }

    void setDefaultCameraMatrix() {
      float sensor_res = 1.f;

      switch (transform_class) {
        case PinholeUnprojection:
          // throw
          // std::runtime_error("PointUnprojector_::setDefaultCameraMatrix| TODO
          // add default Params for Pinhole Camera Matrix");
          break;
        case PolarUnprojection:
          //          if (GeometryDim != 2) {
          //            throw std::runtime_error(
          //              "PointUnprojector_::setDefaultCameraMatrix| TODO add "
          //              "default Params for Polar Camera Matrix of Dim > 2");
          //          }
          sensor_res = (this->param_angle_max.value() - this->param_angle_min.value()) /
                       this->param_num_ranges.value();
          if constexpr (GeometryDim == 2) {
            this->_camera_matrix << 1.f / sensor_res, this->param_num_ranges.value() / 2.f, 0, 1;
            // std::cerr << "WARNING: using sick default camera matrix called in constructor! Adjust "
            //              "other pipeline accordiningly and remove this!"
            //           << std::endl;
          } else {
            this->_camera_matrix << 1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f;
          }
          break;
        default:
          throw std::runtime_error("PointUnprojector_::setDefaultCameraMatrix| "
                                   "Undefined transform_class");
      }
    }

    // computes the points from a packed matrix, to a cloud
    // the row/col structure is preserved
    // the 1st matrix should be a float matrix storing the depth
    // the other matrices contain channels that will be stacked in the point
    // payload offset idx = 1: channels = {Depth, Payloads...} -> point =
    // {Coordinates, Payloads...} payload_offset_idx = 2: channels = {Depth,
    // Payloads...} -> point = {Coordinates, Normal, Payloads...}
    template <PointUnprojectorMode payload_offset_idx = NoNormals,
              typename SrcChannelType,
              typename... SrcChannelsRestType>
    int computeMatrix(DestMatrixType& dest,
                      const SrcChannelType& src,
                      const SrcChannelsRestType&... rest) {
      // PROFILE_TIME("PointUnprojector::computeMatrix");
      dest.resize(src.rows(), src.cols());
      ImageCoordinatesType image_coords;
      UnprojectionType projective_transform = computeProjectiveTransform();

      int num_valid = 0;
      for (size_t r = 0; r < src.rows(); ++r) {
        if (ProjectedDim == 2) {
          image_coords(1) = r;
        }
        for (size_t c = 0; c < src.cols(); ++c) {
          const auto& src_pt     = src.at(r, c);
          DestPointType& dest_pt = dest.at(r, c);
          if (src_pt < this->param_range_min.value() || src_pt > this->param_range_max.value()) {
            dest_pt.status = Invalid;
            continue;
          }
          dest_pt.status                                 = Valid;
          image_coords(0)                                = c;
          dest_pt.template value<0>().head(ProjectedDim) = image_coords;
          dest_pt.template value<0>()(ProjectedDim)      = src_pt;
          dest_pt.template transformInPlace<0, transform_class, UnprojectionType>(
            projective_transform);
          dest_pt.template transformInPlace<0, TRANSFORM_CLASS::Isometry, IsometryType>(
            _camera_in_world);
          this->template _copyFields<payload_offset_idx, const SrcChannelsRestType&...>(
            dest_pt, r, c, rest...);
          ++num_valid;
        }
      }

      return num_valid;
    }

    template <PointUnprojectorMode payload_offset_idx = NoNormals,
              typename OutputIteratorType,
              typename SrcChannelType,
              typename... SrcChannelsRestType>
    int compute(OutputIteratorType dest,
                const SrcChannelType& src,
                const SrcChannelsRestType&... rest) {
      // PROFILE_TIME("PointUnprojector::compute(dense)");
      ImageCoordinatesType image_coords;
      UnprojectionType projective_transform = computeProjectiveTransform();

      int num_valid = 0;
      for (size_t r = 0; r < src.rows(); ++r) {
        if (ProjectedDim == 2) {
          image_coords(1) = r;
        }
        for (size_t c = 0; c < src.cols(); ++c) {
          const auto& src_pt = src.at(r, c);
          DestPointType dest_pt;
          if (src_pt < this->param_range_min.value() || src_pt > this->param_range_max.value()) {
            dest_pt.status = Invalid;
            continue;
          }
          dest_pt.status                                 = Valid;
          image_coords(0)                                = c;
          dest_pt.template value<0>().head(ProjectedDim) = image_coords;
          dest_pt.template value<0>()(ProjectedDim)      = src_pt;
          dest_pt.template transformInPlace<0, transform_class, UnprojectionType>(
            projective_transform);
          dest_pt.template transformInPlace<0, TRANSFORM_CLASS::Isometry, IsometryType>(
            _camera_in_world);
          this->template _copyFields<payload_offset_idx, const SrcChannelsRestType&...>(
            dest_pt, r, c, rest...);
          *dest++ = dest_pt;
          ++num_valid;
        }
      }
      return num_valid;
    }

    //! sparse unprojection utility suitable for feature based registration
    //! approaches
    //! @param[in] points_image_coordinates_depth_ Point3f coordinates that
    //! contain image coordinates (x, y) + depth (meters)
    //! @param[in, out] points_in_camera_frame_ Point3f coordinates that
    //! describe the 3D point in the camera coordinate frame
    template <PointUnprojectorMode payload_offset_idx = NoNormals, typename PointCloudType_>
    size_t compute(const PointCloudType_& points_image_coordinates_depth_,
                   PointCloudType_& points_in_camera_frame_) {
      // PROFILE_TIME("PointUnprojector::compute(sparse)");
      // ds prepare compute cache
      points_in_camera_frame_.clear();
      points_in_camera_frame_.reserve(points_image_coordinates_depth_.size());
      const UnprojectionType projective_transform = computeProjectiveTransform();

      const float& minimum_depth_meters = this->param_range_min.value();
      const float& maximum_depth_meters = this->param_range_max.value();

      // ds for all input points
      for (const DestPointType& point_in_image : points_image_coordinates_depth_) {
        const float& depth_meters = point_in_image.template value<0>()(ProjectedDim);
        assert(depth_meters >= 0);

        // ds skip points that lie out of the unprojection range
        if (depth_meters < minimum_depth_meters || depth_meters > maximum_depth_meters) {
          continue;
        }

        // ds copy all fields (we will overwrite the coordinates with the
        // unprojection)
        DestPointType point_in_camera_frame = point_in_image;

        // ds compute unprojected coordinates
        point_in_camera_frame.status = Valid;
        point_in_camera_frame.template value<0>().head(ProjectedDim) =
          point_in_image.template value<0>().head(ProjectedDim);
        point_in_camera_frame.template value<0>()(ProjectedDim) = depth_meters;
        point_in_camera_frame.template transformInPlace<0, transform_class, UnprojectionType>(
          projective_transform);
        point_in_camera_frame.template transformInPlace<0, TRANSFORM_CLASS::Isometry, IsometryType>(
          _camera_in_world);

        // ds store in result buffer
        points_in_camera_frame_.emplace_back(point_in_camera_frame);
      }
      if (points_in_camera_frame_.empty()) {
        std::cerr << "PointUnprojector::compute|WARNING: no valid unprojections obtained"
                  << std::endl;
      }
      return points_in_camera_frame_.size();
    }

  protected:
    IsometryType _camera_in_world = IsometryType::Identity();
    IsometryType _world_in_camera = IsometryType::Identity();

    UnprojectionType computeProjectiveTransform() {
      UnprojectionType projection_matrix;
      projection_matrix.setIdentity();
      switch (transform_class) {
        case PinholeUnprojection:
          projection_matrix.linear() = this->_camera_matrix;
          break;
        case PolarUnprojection:
          projection_matrix.linear().block(0, 0, GeometryDim - 1, GeometryDim - 1) =
            this->_camera_matrix.block(0, 0, GeometryDim - 1, GeometryDim - 1);
          projection_matrix.translation().head(GeometryDim - 1) =
            this->_camera_matrix.block(0, GeometryDim - 1, GeometryDim - 1, 1);
          break;
        default:
          assert(0 && "unknown_projection");
      }
      return projection_matrix.inverse();
    }
  };

  template <typename DestCloudType_>
  class PointUnprojectorPolar_
    : public PointUnprojector_<TRANSFORM_CLASS::PolarUnprojection, DestCloudType_> {
  public:
    using ThisType = PointUnprojectorPolar_<DestCloudType_>;
    using BaseType = PointUnprojector_<TRANSFORM_CLASS::PolarUnprojection, DestCloudType_>;
  };

  template <typename DestCloudType_>
  class PointUnprojectorPinhole_
    : public PointUnprojector_<TRANSFORM_CLASS::PinholeUnprojection, DestCloudType_> {
  public:
    using ThisType = PointUnprojectorPinhole_<DestCloudType_>;
    using BaseType = PointUnprojector_<TRANSFORM_CLASS::PinholeUnprojection, DestCloudType_>;
  };

} // namespace srrg2_core
