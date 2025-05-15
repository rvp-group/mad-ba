#pragma once
#include "point_cloud.h"
#include "srrg_data_structures/matrix.h"

namespace srrg2_core {

  //! @breif stateless class, container of methods basically
  // ia name is crap
  class PointMatrixProcessor {
  public:
    PointMatrixProcessor() = delete;
    virtual ~PointMatrixProcessor();

    //! @brief accumulator templetized on the dimension of the quantity
    template <int Dim_>
    struct Accumulator_ {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      //! @brief useful typedefs
      static constexpr int Dim   = Dim_;
      using SumVectorType        = Vector_<float, Dim>;
      using SquaredSumMatrixType = MatrixN_<float, Dim>;

      //! @brief accumulator sum
      SumVectorType sum;
      //! @brief accumulator squared sum
      SquaredSumMatrixType squared_sum;

      //! @brief number of VALID points used in the accumulation
      size_t num;

      Accumulator_() {
        setZero();
      }

      void setZero() {
        sum.setZero();
        squared_sum.setZero();
        num = 0;
      }

      inline SquaredSumMatrixType covariance() const {
        SumVectorType mean              = SumVectorType::Zero();
        SquaredSumMatrixType covariance = SquaredSumMatrixType::Zero();
        float inv_num                   = 0;
        if (num) {
          inv_num = 1.0f / (float) (num);
          mean    = sum;
        }

        //        std::cerr << "SquaredSumMatrixType|mean = " <<
        //        mean.transpose() << std::endl; std::cerr <<
        //        "SquaredSumMatrixType|num = " << num << std::endl; std::cerr
        //        << "SquaredSumMatrixType|squared sum = \n" << squared_sum <<
        //        std::endl;

        covariance = squared_sum;
        //        std::cerr << "SquaredSumMatrixType|squared_sum * inv_num = \n"
        //        << covariance << std::endl; std::cerr <<
        //        "SquaredSumMatrixType|mean*mean.transpose() = \n" <<
        //        mean*mean.transpose() << std::endl;
        covariance.noalias() -= mean * mean.transpose() * inv_num;
        return covariance;
      }

      //! @brief basic operators
      inline Accumulator_<Dim_>
      operator+(const Accumulator_<Dim_>& other_) const {
        Accumulator_<Dim_> returned;
        returned.sum         = sum + other_.sum;
        returned.squared_sum = squared_sum + other_.squared_sum;
        returned.num         = num + other_.num;
        return returned;
      }

      inline Accumulator_<Dim_>& operator+=(const Accumulator_<Dim_> other_) {
        sum.noalias() += other_.sum;
        squared_sum.noalias() += other_.squared_sum;
        num += other_.num;
        return *this;
      }

      inline Accumulator_<Dim_>
      operator-(const Accumulator_<Dim_>& other_) const {
        Accumulator_<Dim_> returned;
        returned.sum         = sum - other_.sum;
        returned.squared_sum = squared_sum - other_.squared_sum;
        returned.num         = num - other_.num;
        return returned;
      }

      inline Accumulator_<Dim_>& operator-=(const Accumulator_<Dim_> other_) {
        sum.noalias() -= other_.sum;
        squared_sum.noalias() -= other_.squared_sum;
        num -= other_.num;
        return *this;
      }

      //! @brief stream operators
      // ia works to be checked
      friend std::ostream& operator<<(std::ostream& os_,
                                      const Accumulator_<Dim_>& acc_) {
        os_ << "{";
        os_ << "num = " << acc_.num << " sum = " << acc_.sum.transpose() << "}";
        return os_;
      }
    };

    //! @brief compute integral image of an organized point cloud
    template <typename PointType_>
    static void computeIntegralImage(
      Matrix_<Accumulator_<PointType_::Dimensions>,
              Eigen::aligned_allocator<Accumulator_<PointType_::Dimensions>>>&
        dest_,
      const Matrix_<PointType_, Eigen::aligned_allocator<PointType_>>& src_);

    //! @brief compute integral image of an organized point cloud - only a field
    // ia come hai fatto a farlo andare nonno linux? 
    template <int idx_, typename PointType_>
    static void computeIntegralImage(
      Matrix_<Accumulator_<PointType_::template TypeAt<idx_>::Dim>,
              Eigen::aligned_allocator<
                Accumulator_<PointType_::template TypeAt<idx_>::Dim>>>& dest_,
      const Matrix_<PointType_, Eigen::aligned_allocator<PointType_>>& src_);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} /* namespace srrg2_core */

#include "point_matrix_processor.hpp"
