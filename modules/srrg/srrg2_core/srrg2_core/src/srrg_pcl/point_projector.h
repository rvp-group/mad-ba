#pragma once
#include <limits>

#include "camera_matrix_owner.h"
#include "point_cloud.h"
#include "srrg_data_structures/matrix.h"
#include "srrg_system_utils/profiler.h"

namespace srrg2_core {

  template <typename PointCloudType_>
  class PointProjectorBase_ : public CameraMatrixOwner_<PointCloudType_::PointType::GeometryDim> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PARAM(PropertyFloat, range_min, "minimum range [m]", 0.3f, 0);
    PARAM(PropertyFloat, range_max, "maximum range [m]", 20.f, 0);

    using ThisType                   = PointProjectorBase_<PointCloudType_>;
    using PointCloudType             = PointCloudType_;
    using PointType                  = typename PointCloudType::PointType;
    using IteratorType               = typename PointCloudType::iterator;
    using ConstIteratorType          = typename PointCloudType::const_iterator;
    static constexpr int GeometryDim = PointType::GeometryDim;
    using IsometryType               = Isometry_<float, GeometryDim>;

    using ProjectedMatrixType =
      PointCloud_<Matrix_<PointType, typename Eigen::aligned_allocator<PointType>>>;
    using TransformedMatrixType =
      PointCloud_<Matrix_<PointType, typename Eigen::aligned_allocator<PointType>>>;
    using DepthMatrixType         = Matrix_<float>;
    using IteratorMatrixType      = Matrix_<IteratorType>;
    using ConstIteratorMatrixType = Matrix_<ConstIteratorType>;
    using IndexMatrixType         = Matrix_<int>;

    // we group all the parts of the unprojection
    // in one single block, since it is more cache friendly
    // when we have to copy shit load of fields
    struct TargetMatrixEntry {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      int source_idx = -1;
      IteratorType source_it;
      PointType transformed;
      PointType projected;
      float depth = std::numeric_limits<float>::max();
    };

    struct TargetMatrixType
      : public Matrix_<TargetMatrixEntry, Eigen::aligned_allocator<TargetMatrixEntry>> {
      using BaseType = Matrix_<TargetMatrixEntry, Eigen::aligned_allocator<TargetMatrixEntry>>;
      using IndexMatrixType         = typename ThisType::IndexMatrixType;
      using IteratorMatrixType      = typename ThisType::IteratorMatrixType;
      using ConstIteratorMatrixType = typename ThisType::ConstIteratorMatrixType;
      using DepthMatrixType         = typename ThisType::DepthMatrixType;
      using ProjectedMatrixType     = typename ThisType::ProjectedMatrixType;
      using TransformedMatrixType   = typename ThisType::TransformedMatrixType;

      TargetMatrixType() {
      }

      TargetMatrixType(std::size_t rows_, std::size_t cols_) : BaseType(rows_, cols_) {
      }

      void toIndexMatrix(IndexMatrixType& indices) const {
        indices.resize(this->rows(), this->cols());
        auto dest_it = indices.begin();
        for (auto src_it = this->begin(); src_it != this->end(); ++src_it, ++dest_it)
          *dest_it = src_it->source_idx;
      }

      void toIteratorMatrix(IteratorMatrixType& iterators) const {
        iterators.resize(this->rows(), this->cols());
        auto dest_it = iterators.begin();
        for (auto src_it = this->begin(); src_it != this->end(); ++src_it, ++dest_it)
          *dest_it = src_it->source;
      }

      void toDepthMatrix(DepthMatrixType& depths) const {
        depths.resize(this->rows(), this->cols());
        auto dest_it = depths.begin();
        for (auto src_it = this->begin(); src_it != this->end(); ++src_it, ++dest_it)
          *dest_it = src_it->depth;
      }

      void toProjectedMatrix(ProjectedMatrixType& projected) const {
        projected.resize(this->rows(), this->cols());
        auto dest_it = projected.begin();
        for (auto src_it = this->begin(); src_it != this->end(); ++src_it, ++dest_it)
          *dest_it = src_it->projected;
      }

      template <int field_idx, typename DestMatrixType>
      void toProjectedMatrixField(DestMatrixType& projected_field) const {
        projected_field.resize(this->rows(), this->cols());
        auto dest_it = projected_field.begin();
        for (auto src_it = this->begin(); src_it != this->end(); ++src_it, ++dest_it)
          *dest_it = src_it->projected.template value<field_idx>();
      }

      void toTransformedMatrix(TransformedMatrixType& transformed) const {
        transformed.resize(this->rows(), this->cols());
        auto dest_it = transformed.begin();
        for (auto src_it = this->begin(); src_it != this->end(); ++src_it, ++dest_it)
          *dest_it = src_it->transformed;
      }
    };
  };

  template <TRANSFORM_CLASS transform_class_, typename PointCloudType_>
  class PointProjector_ : public PointProjectorBase_<PointCloudType_>, public Profiler {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType                   = PointProjector_<transform_class_, PointCloudType_>;
    using BaseType                   = PointProjectorBase_<PointCloudType_>;
    using PointCloudType             = PointCloudType_;
    using PointType                  = typename PointCloudType::PointType;
    using IteratorType               = typename PointCloudType::iterator;
    using ConstIteratorType          = typename PointCloudType::const_iterator;
    static constexpr int GeometryDim = PointType::GeometryDim;
    using IsometryType               = Isometry_<float, GeometryDim>;
    using ProjectionType             = Transform_<float, GeometryDim>;
    static constexpr TRANSFORM_CLASS transform_class = transform_class_;

    virtual ~PointProjector_() {
    }

    virtual void initCameraMatrix() = 0;

    virtual void adjustProjectionMatrix(ProjectionType& projection_matrix) = 0;

    int
    compute(typename BaseType::TargetMatrixType& target, IteratorType begin_, IteratorType end_) {
      PROFILE_TIME("PointProjector::compute(depth buffer)");
      if (!this->_canvas_rows || !this->_canvas_cols) {
        throw std::runtime_error("PointProjector_::compute| set Canvas sizes "
                                 "before calling the compute!");
      }
      target.resize(this->_canvas_rows, this->_canvas_cols);
      if (this->_config_changed) {
        initCameraMatrix();
        this->_config_changed = 0;
      }
      // clear depth buffer
      typename BaseType::TargetMatrixEntry entry;
      entry.source_it = end_;
      entry.projected.setZero();
      entry.projected.status = Invalid;
      entry.transformed.setZero();
      entry.transformed.status = Invalid;
      target.fill(entry);

      // adjust camera matrix and entries
      ProjectionType projection_matrix;
      projection_matrix.setIdentity();
      adjustProjectionMatrix(projection_matrix);

      int num_good = 0;
      int index    = 0;
      for (auto it = begin_; it != end_; ++it, ++index) {
        const PointType& p = *it;
        if (p.status != Valid) {
          continue;
        }

        // we transform the point
        const PointType tp =
          p.template transform<TRANSFORM_CLASS::Isometry, IsometryType>(this->_world_in_camera);

        // we project the coordinates field only (field 0 by definition)
        const PointType pp =
          tp.template transform<0, transform_class, ProjectionType>(projection_matrix);

        // ds drop negative coordinates on the spot (before rounding to pixel coordinates)
        // ds as the current rounding mode passes points with negative coordinates (-0.5f -> 0)
        if (pp.template value<0>()(0) < 0.0f) {
          continue;
        }
        if (pp.template value<0>()(1) < 0.0f) {
          continue;
        }

        const float& z = pp.template value<0>()(GeometryDim - 1);
        // too far or too close?
        if (z < this->param_range_min.value() || z > this->param_range_max.value()) {
          continue;
        }

        // ds compute image coordinates at pixel precision
        int r = 0, c = 0;
        c = int(pp.template value<0>()(0) + 0.5f);
        if (GeometryDim >= 3) {
          r = int(pp.template value<0>()(1) + 0.5f);
        }

        // is not inside image? skip the point
        if (!target.inside(r, c)) {
          continue;
        }

        // depth_buffer
        typename BaseType::TargetMatrixEntry& target_cell = target.at(r, c);
        if (z > target_cell.depth && target_cell.source_idx) {
          continue;
        }

        // point was not good and now it becomes good
        if (target_cell.source_idx < 0) {
          ++num_good;
        }

        target_cell.source_idx  = index;
        target_cell.source_it   = it;
        target_cell.transformed = tp;
        target_cell.projected   = pp;
        target_cell.depth       = z;
      }
      return num_good;
    }

    // ! point cloud projection to index matrix, avoid points copy and speed up performance
    int compute(typename BaseType::TargetMatrixType::IndexMatrixType& index_matrix,
                IteratorType begin_,
                IteratorType end_) {
      PROFILE_TIME("PointProjector::compute(IndexMatrix)");
      if (!this->_canvas_rows || !this->_canvas_cols) {
        throw std::runtime_error("PointProjector_::compute| set Canvas sizes "
                                 "before calling the compute!");
      }
      index_matrix.resize(this->_canvas_rows, this->_canvas_cols);
      if (this->_config_changed) {
        initCameraMatrix();
        this->_config_changed = 0;
      }
      // adjust camera matrix and entries
      ProjectionType projection_matrix;
      projection_matrix.setIdentity();
      adjustProjectionMatrix(projection_matrix);

      int num_good = 0;
      int index    = -1;
      index_matrix.fill(index);

      PointType projected_point;
      int source_idx = 0;
      for (IteratorType it = begin_; it != end_; ++it, ++source_idx) {
        const PointType& p = *it;
        if (p.status != Valid) {
          continue;
        }

        // we transform the point
        projected_point =
          p.template transform<TRANSFORM_CLASS::Isometry, IsometryType>(this->_world_in_camera);

        // we project the coordinates field only (field 0 by definition)
        projected_point.template transformInPlace<0, transform_class, ProjectionType>(
          projection_matrix);

        // ds drop negative coordinates on the spot (before rounding to pixel coordinates)
        // ds as the current rounding mode passes points with negative coordinates (-0.5f -> 0)
        if (projected_point.template value<0>()(0) < 0.0f) {
          continue;
        }
        if (projected_point.template value<0>()(1) < 0.0f) {
          continue;
        }

        const float& z = projected_point.template value<0>()(GeometryDim - 1);
        // too far or too close?
        if (z < this->param_range_min.value() || z > this->param_range_max.value()) {
          continue;
        }

        // ds compute image coordinates at pixel precision
        int r = 0, c = 0;
        c = int(projected_point.template value<0>()(0) + 0.5f);
        if (GeometryDim >= 3) {
          r = int(projected_point.template value<0>()(1) + 0.5f);
        }

        // is not inside image? skip the point
        if (!index_matrix.inside(r, c)) {
          continue;
        }
        // cell that was not good became good
        int& target_index = index_matrix.at(r, c);
        if (target_index < 0) {
          ++num_good;
        }
        // add the source index to the matrix
        target_index = source_idx;
      }
      return num_good;
    }
    //! point cloud based projection method without depth buffer, only returns valid points
    //! note that this method has changed argument order! output should always be last
    size_t compute(const PointCloudType& points_in_world_,
                   PointCloudType& points_in_camera_,
                   PointCloudType& points_in_image_,
                   std::vector<int>& indices_in_world_) {
      PROFILE_TIME("PointProjector::compute");

      // ds prepare output buffers
      points_in_camera_.clear();
      points_in_camera_.reserve(points_in_world_.size());
      points_in_image_.clear();
      points_in_image_.reserve(points_in_world_.size());
      indices_in_world_.clear();
      indices_in_world_.reserve(points_in_world_.size());

      // ds cache and adjust camera matrix and entries (virtual dispatch)
      ProjectionType projection_matrix;
      projection_matrix.setIdentity();
      adjustProjectionMatrix(projection_matrix);
      const size_t& image_rows(this->_canvas_rows);
      const size_t& image_cols(this->_canvas_cols);
      const float& minimum_depth_meters(this->param_range_min.value());
      const float& maximum_depth_meters(this->param_range_max.value());

      // ds process all points
      for (size_t i = 0; i < points_in_world_.size(); ++i) {
        const PointType& point_in_world(points_in_world_[i]);
        if (point_in_world.status != Valid) {
          continue;
        }

        // ds obtain a deep copy of the source point which we will transform accordingly
        // ds all fields are copied
        PointType point_in_camera(point_in_world);

        // ds we transform the point in place
        point_in_camera.template transformInPlace<TRANSFORM_CLASS::Isometry, IsometryType>(
          this->_world_in_camera);

        // ds drop points with invalid depth
        const float depth_meters = point_in_camera.template value<0>()(GeometryDim - 1);
        if (depth_meters < minimum_depth_meters || depth_meters > maximum_depth_meters) {
          continue;
        }

        // ds obtain a deep copy of the source point which we will transform accordingly
        // ds all fields are copied
        PointType point_in_image(point_in_camera);

        // ds we project the coordinates field only in place (field 0 by definition)
        point_in_image.template transformInPlace<0, transform_class, ProjectionType>(
          projection_matrix);
        assert(point_in_image.template value<0>()(GeometryDim - 1) == depth_meters);

        // ds drop points with negative image coordinates
        assert(GeometryDim >= 2);
        const float& image_coordinate_horizontal = point_in_image.template value<0>()(0);
        const float& image_coordinate_vertical   = point_in_image.template value<0>()(1);
        if (image_coordinate_horizontal < 0 || image_coordinate_vertical < 0) {
          continue;
        }

        // ds if projection is within canvas, we keep it
        if (image_coordinate_horizontal < image_cols && image_coordinate_vertical < image_rows) {
          points_in_image_.emplace_back(point_in_image);
          points_in_camera_.emplace_back(point_in_camera);
          indices_in_world_.emplace_back(i);
        }
      }
      return points_in_camera_.size();
    }
  };

} // namespace srrg2_core
