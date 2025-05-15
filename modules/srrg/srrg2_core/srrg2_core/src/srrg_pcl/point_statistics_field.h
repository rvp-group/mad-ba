#pragma once
#include <memory>

#include "point_default_field.h"
#include "srrg_geometry/geometry_defs.h"

namespace srrg2_core {

  //! traits that define the available operations on a statistics field
  template <typename PointFieldType_>
  struct PointStatisticsFieldTraits_ : PointDefaultFieldTraits_<PointFieldType_> {
    using PointFieldType = PointFieldType_;

    //! not vectorized
    static constexpr size_t Dim = 0;

    //! no operations overloaded available yet (e.g. summing of statistics)
  };

  //! basic statistics field TODO refactor or yet better, burn this monstrosity
  //! note that this field is also used for 2D tracking visualization
  template <typename Scalar_, size_t StateDim_>
  struct PointStatisticsField_ {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr size_t Dim   = StateDim_;
    static constexpr size_t DimSE = (Dim <= 3) ? Dim : 3; // ds capped at SE(3) space
    using StateVectorType         = Vector_<Scalar_, Dim>;
    using StateMatrixType         = MatrixN_<Scalar_, Dim>;
    using ProjectionVectorType    = Vector_<Scalar_, Dim - 1>;
    using IsometryType            = Isometry_<Scalar_, DimSE>;
    size_t unique_id=0;
    // ds stored properties NGHGHGN
    struct CameraMeasurement {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      CameraMeasurement() = delete;
      CameraMeasurement(const StateVectorType& point_in_image_,
                        const StateVectorType& point_in_camera_,
                        const IsometryType& camera_from_world_,
                        const IsometryType& world_from_camera_) :
        point_in_image(point_in_image_),
        point_in_camera(point_in_camera_),
        inverse_depth_meters(1.0 / point_in_camera_(2)),
        camera_from_world(camera_from_world_),
        world_from_camera(world_from_camera_) {
        assert(point_in_camera_(2) > 0);
      }
      const StateVectorType point_in_image;
      const StateVectorType point_in_camera;
      const Scalar_ inverse_depth_meters;
      const IsometryType camera_from_world;
      const IsometryType world_from_camera;
    };
    using CameraMeasurementVector =
      std::vector<CameraMeasurement, Eigen::aligned_allocator<CameraMeasurement>>;

    struct Statistics {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      size_t number_of_optimizations         = 0;
      bool is_inlier                         = false;
      ProjectionVectorType latest_projection = ProjectionVectorType::Zero();
      StateVectorType state                  = StateVectorType::Zero();
      StateMatrixType covariance             = StateMatrixType::Zero();
      CameraMeasurementVector measurements;
    };

    using ValueType  = std::shared_ptr<Statistics>;
    using TraitsType = PointStatisticsFieldTraits_<ValueType>;

    // ds storage (must be instanciated only once for all points in the same track!)
    ValueType value;

    // ds method to allocate the value field (should only be used when creating a landmark!)
    // ds for all points that relate to the same statistics the value can be simply assigned
    void allocate() {
      assert(value == nullptr);
      value = ValueType(new Statistics());
      value->measurements.reserve(100);
    }

    // ds adds a new entry to the statistics (state update)
    void addOptimizationResult(const StateVectorType& state_,
                               const StateMatrixType& covariance_ = StateMatrixType::Zero()) {
      setState(state_);
      setCovariance(covariance_);
      incrementNumberOfOptimizations();
    }

    void setNumberOfOptimizations(const size_t& count_) {
      assert(value);
      value->number_of_optimizations = count_;
    }
    void incrementNumberOfOptimizations() {
      assert(value);
      ++value->number_of_optimizations;
    }
    const size_t& numberOfOptimizations() const {
      assert(value);
      return value->number_of_optimizations;
    }

    void setIsInlier(const bool is_inlier_) {
      assert(value);
      value->is_inlier = is_inlier_;
    }
    const bool& isInlier() const {
      assert(value);
      return value->is_inlier;
    }

    const ProjectionVectorType& projection() const {
      assert(value);
      return value->latest_projection;
    }
    void setProjection(const ProjectionVectorType& measurement_) {
      assert(value);
      value->latest_projection = measurement_;
    }

    const StateVectorType& state() const {
      assert(value);
      return value->state;
    }
    void setState(const StateVectorType& state_) {
      assert(value);
      value->state = state_;
    }

    const StateMatrixType& covariance() const {
      assert(value);
      return value->covariance;
    }
    void setCovariance(const StateMatrixType& covariance_) {
      assert(value);
      value->covariance = covariance_;
    }

    const CameraMeasurementVector& measurements() const {
      assert(value);
      return value->measurements;
    }
    void addMeasurement(const CameraMeasurement& measurement_) const {
      assert(value);
      value->measurements.push_back(measurement_);
    }
  };
  using PointStatisticsField2D = PointStatisticsField_<float, 2>;
  using PointStatisticsField3D = PointStatisticsField_<float, 3>;

} // namespace srrg2_core
