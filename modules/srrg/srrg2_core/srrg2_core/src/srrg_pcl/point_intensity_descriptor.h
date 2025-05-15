#pragma once
#include "point.h"
#include "point_cloud.h"
#include "point_derived.h"
#include "point_descriptor_field.h"
#include "point_statistics_field.h"

namespace srrg2_core {

  //! image point with intensity information and a (binary) descriptor
  //! TODO move from cv::Mat descriptors to bitsets/vectors
  template <int Dim_, typename Scalar_>
  struct PointIntensityDescriptor_ : PointDerived_<Point_<Dim_, Scalar_>,
                                                   PointScalarField_<uchar>,
                                                   PointDescriptorFieldCvMatBinary,
                                                   PointStatisticsField_<Scalar_, Dim_>> {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Scalar                = Scalar_;
    static constexpr size_t Dim = Dim_;
    using VectorType            = Vector_<Scalar, Dim>;
    using CoordinatesField      = PointCoordinatesField_<Scalar, Dim>;
    using IntensityField        = PointScalarField_<uchar>;
    using DescriptorField       = PointDescriptorFieldCvMatBinary;
    using PointStatisticsField  = PointStatisticsField_<Scalar, Dim>;
    using BaseType              = PointDerived_<Point_<Dim, Scalar>,
                                   PointScalarField_<uchar>,
                                   PointDescriptorFieldCvMatBinary,
                                   PointStatisticsField>;

    PointIntensityDescriptor_() {
    }

    PointIntensityDescriptor_(const BaseType& other) : BaseType(other) {
    }

    inline PointIntensityDescriptor_& operator=(const BaseType& other) {
      BaseType::operator=(other);
      return *this;
    }

    inline typename IntensityField::ValueType& intensity() {
      return BaseType::template value<1>();
    }
    inline const typename IntensityField::ValueType& intensity() const {
      return BaseType::template value<1>();
    }

    inline typename DescriptorField::ValueType& descriptor() {
      return BaseType::template value<2>();
    }
    inline const typename DescriptorField::ValueType& descriptor() const {
      return BaseType::template value<2>();
    }

    inline PointStatisticsField& statistics() {
      return BaseType::template field<3>();
    }
    inline const PointStatisticsField& statistics() const {
      return BaseType::template field<3>();
    }
  };

  using PointIntensityDescriptor1f = PointIntensityDescriptor_<1, float>;
  using PointIntensityDescriptor1d = PointIntensityDescriptor_<1, double>;
  using PointIntensityDescriptor1i = PointIntensityDescriptor_<1, int>;

  using PointIntensityDescriptor2f = PointIntensityDescriptor_<2, float>;
  using PointIntensityDescriptor2d = PointIntensityDescriptor_<2, double>;
  using PointIntensityDescriptor2i = PointIntensityDescriptor_<2, int>;

  using PointIntensityDescriptor3f = PointIntensityDescriptor_<3, float>;
  using PointIntensityDescriptor3d = PointIntensityDescriptor_<3, double>;
  using PointIntensityDescriptor3i = PointIntensityDescriptor_<3, int>;

  using PointIntensityDescriptor4f = PointIntensityDescriptor_<4, float>;
  using PointIntensityDescriptor4d = PointIntensityDescriptor_<4, double>;
  using PointIntensityDescriptor4i = PointIntensityDescriptor_<4, int>;

  template <int Dim_, typename Scalar_>
  using PointIntensityDescriptorVectorCloud =
    PointCloud_<std::vector<PointIntensityDescriptor_<Dim_, Scalar_>,
                            Eigen::aligned_allocator<PointIntensityDescriptor_<Dim_, Scalar_>>>>;

  using PointIntensityDescriptor1fVectorCloud = PointIntensityDescriptorVectorCloud<1, float>;
  using PointIntensityDescriptor1dVectorCloud = PointIntensityDescriptorVectorCloud<1, double>;
  using PointIntensityDescriptor1iVectorCloud = PointIntensityDescriptorVectorCloud<1, int>;

  using PointIntensityDescriptor2fVectorCloud = PointIntensityDescriptorVectorCloud<2, float>;
  using PointIntensityDescriptor2dVectorCloud = PointIntensityDescriptorVectorCloud<2, double>;
  using PointIntensityDescriptor2iVectorCloud = PointIntensityDescriptorVectorCloud<2, int>;

  using PointIntensityDescriptor3fVectorCloud = PointIntensityDescriptorVectorCloud<3, float>;
  using PointIntensityDescriptor3dVectorCloud = PointIntensityDescriptorVectorCloud<3, double>;
  using PointIntensityDescriptor3iVectorCloud = PointIntensityDescriptorVectorCloud<3, int>;

  using PointIntensityDescriptor4fVectorCloud = PointIntensityDescriptorVectorCloud<4, float>;
  using PointIntensityDescriptor4dVectorCloud = PointIntensityDescriptorVectorCloud<4, double>;
  using PointIntensityDescriptor4iVectorCloud = PointIntensityDescriptorVectorCloud<4, int>;

} // namespace srrg2_core
