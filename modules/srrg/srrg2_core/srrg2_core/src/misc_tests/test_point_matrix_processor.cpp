#include <iostream>
#include <iterator>
#include <string>
#include "srrg_system_utils/system_utils.h"
#include "srrg_system_utils/shell_colors.h"
#include "srrg_geometry/geometry_defs.h"
#include "srrg_geometry/geometry3d.h"
#include "srrg_pcl/point_cloud.h"
#include "srrg_pcl/point_example.h"
#include "srrg_pcl/point_matrix_processor.h"
#include "srrg_pcl/point_projector.h"
#include "srrg_pcl/point_unprojector.h"
#include "srrg_pcl/point_projector_pinhole.h"
#include "srrg_pcl/point_projector_polar.h"

#define MATRIX_SIZE 5

#define LOG std::cerr << "test_point_image_processor|"
using namespace srrg2_core;

using UnprojectorType = PointUnprojectorPinhole_<MyPointWithNormal3fCloud>;
using ProjectorType = PointProjectorPinhole_<MyPointWithNormal3fCloud>;
using ProjectedMatrixType = ProjectorType::TargetMatrixType;
using UnprojectedMatrixType = typename UnprojectorType::DestMatrixType;

void fillVector(MyPointWithNormal3fCloud& target, int num_points){
  target.resize(num_points);
  float k=0;
  for (MyPointWithNormal3f& p: target){
    p.coordinates()=Vector3f::Random()*10;
    p.normal()=Vector3f::Random();
    p.normal().normalize();
    p.weight()=k;
    k+=1.f;
    p.stuff().x;
  }
}


//ia usings
//ia you must specify the size of the accumulator, otherwise you get Eigen compiler errors (size mismatch)

//ia if you want to compute integral on specific field
using AccumulatorType0 = PointMatrixProcessor::Accumulator_<MyPointWithNormal3f::TypeAt<0>::Dim>;
using AccumulatorType0Matrix = Matrix_<AccumulatorType0, Eigen::aligned_allocator<AccumulatorType0 > >;

//ia if you want to compute integral on all the fieds of a MyPointWithNormal3f
using AccumulatorMyPointNormal3f = PointMatrixProcessor::Accumulator_<MyPointWithNormal3f::Dimensions>;
using AccumulatorMyPointNormal3fMatrix = Matrix_<AccumulatorMyPointNormal3f, Eigen::aligned_allocator<AccumulatorMyPointNormal3f > >;


int main(int argc, char **argv) {

  AccumulatorMyPointNormal3fMatrix integral_matrix;
  UnprojectorType::DestMatrixType source_int_matrix;
  source_int_matrix.resize(MATRIX_SIZE, MATRIX_SIZE);
  size_t k = 0;
  for (size_t r = 0; r < source_int_matrix.rows(); ++r) {
    for (size_t c = 0; c < source_int_matrix.cols(); ++c) {
      MyPointWithNormal3f p;
      p.coordinates() = Vector3f::Ones() * k;
      p.normal() = Vector3f::Zero();
      p.weight() = k;
      ++k;
      source_int_matrix(r,c) = p;
    }
  }

  integral_matrix.resize(source_int_matrix.rows(), source_int_matrix.cols());
  SystemUsageCounter::tic();
  PointMatrixProcessor::computeIntegralImage<MyPointWithNormal3f>(integral_matrix, source_int_matrix);
  LOG << "time = " << FG_BBLUE(SystemUsageCounter::toc()) << std::endl;


  LOG << "source_int_matrix matrix:\n";
  for (size_t r = 0; r < source_int_matrix.rows(); ++r) {
    std::cerr << "r = " << r << std::endl;
    for (size_t c = 0; c < source_int_matrix.cols(); ++c) {
      std::cerr << "\tc = " << c << " " << source_int_matrix(r,c) << std::endl;
    }
    std::cerr << "\n";
  }

  LOG << "integral matrix:\n";
  for (size_t r = 0; r < integral_matrix.rows(); ++r) {
    std::cerr << "r = " << r << std::endl;
    for (size_t c = 0; c < integral_matrix.cols(); ++c) {
      std::cerr << "\tc = " << c << " " << integral_matrix(r,c) << std::endl;
    }
    std::cerr << "\n";
  }


  LOG << "testing integral image by field" << std::endl;

  //ia you have to declare the size of the accumulator, if the field<IDX>
  //ia has different dimension then you have Eigen curses on size mismatch :)
  AccumulatorType0Matrix integral_by_field;
  integral_by_field.resize(source_int_matrix.rows(), source_int_matrix.cols());
  SystemUsageCounter::tic();
  PointMatrixProcessor::computeIntegralImage<0,MyPointWithNormal3f>(integral_by_field, source_int_matrix);
  LOG << "time = " << FG_BBLUE(SystemUsageCounter::toc()) << std::endl;

  LOG << "integral matrix:\n";
  for (size_t r = 0; r < integral_by_field.rows(); ++r) {
    std::cerr << "r = " << r << std::endl;
    for (size_t c = 0; c < integral_by_field.cols(); ++c) {
      std::cerr << "\tc = " << c << " " << integral_by_field(r,c) << std::endl;
      std::cerr << "\tc = " << c << "covariance\n" << integral_by_field(r,c).covariance() << std::endl;
    }
    std::cerr << "\n";
  }

}




