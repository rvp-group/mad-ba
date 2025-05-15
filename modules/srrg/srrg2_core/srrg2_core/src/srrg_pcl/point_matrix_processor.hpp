#include "srrg_system_utils/shell_colors.h"

namespace srrg2_core {

  /* [........]
   * [...A E..]
   * [...F D..]
   *
   * say we want to compute the value of the element in position D
   * I = D + E + F - A
   *
   * this method has to be re-written in SSE/AVX
   * https://github.com/ermig1979/Simd/blob/master/src/Simd/SimdAvx512bwIntegral.cpp
   *
   * */
  template <typename PointType_>
  void PointMatrixProcessor::computeIntegralImage(
    Matrix_<Accumulator_<PointType_::Dimensions>,
            Eigen::aligned_allocator<Accumulator_<PointType_::Dimensions>>>&
      dest_,
    const Matrix_<PointType_, Eigen::aligned_allocator<PointType_>>& src_) {
    assert(dest_.rows() && dest_.cols() &&
           "PointImageProcessor::computeIntegralImage|invalid dest size");
    assert(src_.rows() && src_.cols() &&
           "PointImageProcessor::computeIntegralImage|invalid src size");

    const size_t rows = src_.rows();
    const size_t cols = src_.cols();

    if (rows != dest_.rows() || cols != dest_.cols())
      throw std::runtime_error(
        "PointImageProcessor::computeIntegralImage|size mismatch");

    for (size_t r = 0; r < rows; ++r) {
      for (size_t c = 0; c < cols; ++c) {
        const auto& point = src_.at(r, c);
        if (point.status != POINT_STATUS::Valid)
          continue;

        auto& acc = dest_.at(r, c);
        point.toPlainVector(acc.sum);

        acc.squared_sum = acc.sum * acc.sum.transpose();
        acc.num         = 1;
      }
    }

    for (size_t r = 0; r < rows; ++r) {
      for (size_t c = 1; c < cols; ++c) {
        dest_.at(r, c) += dest_.at(r, c - 1);
      }
    }

    for (size_t r = 1; r < rows; ++r) {
      for (size_t c = 0; c < cols; ++c) {
        dest_.at(r, c) += dest_.at(r - 1, c);
      }
    }
  }

  template <int idx_, typename PointType_>
  void PointMatrixProcessor::computeIntegralImage(
    Matrix_<Accumulator_<PointType_::template TypeAt<idx_>::Dim>,
            Eigen::aligned_allocator<
              Accumulator_<PointType_::template TypeAt<idx_>::Dim>>>& dest_,
    const Matrix_<PointType_, Eigen::aligned_allocator<PointType_>>& src_) {
    assert(dest_.rows() && dest_.cols() &&
           "PointImageProcessor::computeIntegralImage|invalid dest size");
    assert(src_.rows() && src_.cols() &&
           "PointImageProcessor::computeIntegralImage|invalid src size");

    const size_t rows = src_.rows();
    const size_t cols = src_.cols();

    if (rows != dest_.rows() || cols != dest_.cols())
      throw std::runtime_error(
        "PointImageProcessor::computeIntegralImage|size mismatch");

    for (size_t r = 0; r < rows; ++r) {
      for (size_t c = 0; c < cols; ++c) {
        const auto& point = src_.at(r, c);
        if (point.status != POINT_STATUS::Valid)
          continue;

        auto& acc               = dest_.at(r, c);
        const auto& coordinates = point.template value<idx_>();
        acc.sum                 = coordinates;
        acc.squared_sum         = coordinates * coordinates.transpose();
        acc.num                 = 1;
      }
    }

    for (size_t r = 0; r < rows; ++r) {
      for (size_t c = 1; c < cols; ++c) {
        dest_.at(r, c) += dest_.at(r, c - 1);
      }
    }

    for (size_t r = 1; r < rows; ++r) {
      for (size_t c = 0; c < cols; ++c) {
        dest_.at(r, c) += dest_.at(r - 1, c);
      }
    }
  }

} /* namespace srrg2_core */
