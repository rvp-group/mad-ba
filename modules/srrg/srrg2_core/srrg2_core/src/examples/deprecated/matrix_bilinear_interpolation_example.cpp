#include "srrg_data_structures/matrix.h"
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <srrg_pcl/point_types.h>
#include <sys/time.h>

using namespace std;
using namespace srrg2_core;

#warning "Move me to Test folder"

typedef Point3f Point3;
typedef PointNormal3f PointNormal3;

typedef Point3fVectorCloud Point3Vector;
typedef PointNormal3fVectorCloud PointNormal3Vector;

typedef Matrix_<PointNormal3, std::allocator<PointNormal3>> PointNormal3Matrix;

int main(int argc, char** argv) {
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

  Vector3_<Point3::Scalar> ref_coordinates, ref_normal;
  ref_coordinates << -28.6374149323, -71.0976104736, 112.982414246;
  ref_normal << 0.395957678556, -0.305226564407, 0.678615272045;

  PointNormal3 interpolated_point;
  std::cerr << "grid size: " << grid.rows() << " " << grid.cols() << std::endl;

  grid.getSubPixel(interpolated_point, Vector2f(10.5, 20.5));

  std::cerr << std::fixed << std::setprecision(4);
  std::cerr << grid.at(20, 10).coordinates().transpose() << std::endl;
  std::cerr << grid.at(20, 11).coordinates().transpose() << std::endl;
  std::cerr << grid.at(21, 10).coordinates().transpose() << std::endl;
  std::cerr << grid.at(21, 11).coordinates().transpose() << std::endl;
  std::cerr << "interpolated_point.coordinates [ " << interpolated_point.coordinates().transpose()
            << " ]" << std::endl;

  std::cerr << grid.at(20, 10).normal().transpose() << std::endl;
  std::cerr << grid.at(20, 11).normal().transpose() << std::endl;
  std::cerr << grid.at(21, 10).normal().transpose() << std::endl;
  std::cerr << grid.at(21, 11).normal().transpose() << std::endl;
  std::cerr << "interpolated_point.normal [ " << interpolated_point.normal().transpose() << " ]"
            << std::endl;
}
