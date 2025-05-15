#include <cstdlib>
#include <iostream>

#include "srrg_data_structures/matrix.h"
#include "srrg_pcl/point_types.h"
#include "srrg_test/test_helper.hpp"

using namespace srrg2_core;

typedef Point3f Point3;
typedef PointNormal3f PointNormal3;

typedef Point3fVectorCloud Point3Vector;
typedef PointNormal3fVectorCloud PointNormal3Vector;

typedef Matrix_<PointNormal3, std::allocator<PointNormal3>> PointNormal3Matrix;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST(DummyData, MatrixBilinearInterpolation) {
  PointNormal3 point;

  int rows = 100;
  int cols = 200;

  PointNormal3Matrix grid;

  grid.resize(rows, cols);
  for (size_t r = 0; r < grid.rows(); ++r) {
    for (size_t c = 0; c < grid.cols(); ++c) {
      grid.at(r, c).coordinates() = Eigen::Vector3f::Random() * 300;
      grid.at(r, c).normal()      = Eigen::Vector3f::Random() * 4;
      grid.at(r, c).normalize();
    }
  }

  ASSERT_EQ(grid.rows(), static_cast<decltype(grid.rows())>(rows));
  ASSERT_EQ(grid.cols(), static_cast<decltype(grid.cols())>(cols));

  Vector3_<Point3::Scalar> ref_coordinates, ref_normal;
  ref_coordinates << -28.6374149323, -71.0976104736, 112.982414246;
  ref_normal << 0.395957678556, -0.305226564407, 0.678615272045;

  PointNormal3 interpolated_point;
  std::cerr << "grid size: " << grid.rows() << " " << grid.cols() << std::endl;

  grid.getSubPixel(interpolated_point, Vector2f(20.5, 10.5));

  std::cerr << std::fixed << std::setprecision(4);
  std::cerr << grid.at(20, 10).coordinates().transpose() << std::endl;
  std::cerr << grid.at(20, 11).coordinates().transpose() << std::endl;
  std::cerr << grid.at(21, 10).coordinates().transpose() << std::endl;
  std::cerr << grid.at(21, 11).coordinates().transpose() << std::endl;
  std::cerr << "interpolated_point.coordinates [ " << interpolated_point.coordinates().transpose()
            << " ]" << std::endl;
  ASSERT_NEAR_EIGEN(interpolated_point.coordinates(), ref_coordinates, 1e-5);

  std::cerr << grid.at(20, 10).normal().transpose() << std::endl;
  std::cerr << grid.at(20, 11).normal().transpose() << std::endl;
  std::cerr << grid.at(21, 10).normal().transpose() << std::endl;
  std::cerr << grid.at(21, 11).normal().transpose() << std::endl;
  std::cerr << "interpolated_point.normal [ " << interpolated_point.normal().transpose() << " ]"
            << std::endl;
  ASSERT_NEAR_EIGEN(interpolated_point.normal(), ref_normal, 1e-5);
}
