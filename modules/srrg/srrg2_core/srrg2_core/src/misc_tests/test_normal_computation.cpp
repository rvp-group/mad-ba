#include <iostream>
#include <iterator>
#include <string>

#include "srrg_system_utils/shell_colors.h"
#include "srrg_system_utils/system_utils.h"

#include "srrg_image/image.h"

#include "srrg_geometry/geometry3d.h"
#include "srrg_geometry/geometry_defs.h"
#include "srrg_pcl/normal_computator.h"
#include "srrg_pcl/point_cloud.h"
#include "srrg_pcl/point_projector.h"
#include "srrg_pcl/point_types.h"
#include "srrg_pcl/point_unprojector.h"

using namespace srrg2_core;

using UnprojectorType       = PointUnprojectorPinhole_<PointNormal3fVectorCloud>;
using UnprojectedMatrixType = typename UnprojectorType::DestMatrixType;

const std::string exe_name = "test_normal_computation|";
#define LOG std::cerr << exe_name

int main(int argc, char** argv) {
  if (argc < 2)
    throw std::runtime_error(exe_name + "missing path to file");

  ImageUInt16 raw_depth;
  cv::Mat cv_raw_depth;

  LOG << "Reading depth image0 from file: " << FG_GREEN(argv[1]) << std::endl;
  cv_raw_depth = cv::imread(argv[1], CV_LOAD_IMAGE_ANYDEPTH);
  raw_depth.fromCv(cv_raw_depth);
  LOG << "image size = " << raw_depth.rows() << "x" << raw_depth.cols() << std::endl;
  cv::imshow("depth image", cv_raw_depth * 5);

  // ia converting depth image to Matrix float
  // we need to generate channel matrices for our output
  // by extracting them from the result of projection
  // extract the depths from the result of projection
  ImageFloat depth_image;
  raw_depth.convertTo(depth_image, 1e-3); // ia pass millimeters to meters
  Matrix3f pinhole_camera_matrix;
  pinhole_camera_matrix << 269.853, 0, 157.051, 0, 269.733, 113.118, 0, 0, 1;

  // declare and configure object
  UnprojectorType unprojector;
  unprojector.setCameraPose(Isometry3f::Identity());
  unprojector.setCameraMatrix(pinhole_camera_matrix);

  UnprojectedMatrixType unprojected_matrix;
  unprojected_matrix.resize(raw_depth.rows(), raw_depth.cols());
  unprojected_matrix.fill(PointNormal3f());

  // ia unproject
  unprojector.computeMatrix<WithNormals>(unprojected_matrix, depth_image);

  ImageVector3f point_image;
  point_image.resize(raw_depth.rows(), raw_depth.cols());

  // ia copy the point to bgr values
  for (size_t r = 0; r < point_image.rows(); ++r) {
    for (size_t c = 0; c < point_image.cols(); ++c) {
      point_image.at(r, c) << unprojected_matrix.at(r, c).coordinates();
    }
  }

  cv::Mat cv_color_image;
  point_image.toCv(cv_color_image);
  cv::imshow("points unprojected", cv_color_image);

  // ia cross product normals
  LOG << "normal computation: from a matrix cloud using cross product\n";
  NormalComputator2DCrossProduct<UnprojectedMatrixType, 1> normal_computator_cross;
  SystemUsageCounter::tic();
  normal_computator_cross.computeNormals(unprojected_matrix);
  LOG << "normal time = " << FG_YELLOW(SystemUsageCounter::toc()) << std::endl;

  ImageVector3f normal_image_cp;
  normal_image_cp.resize(raw_depth.rows(), raw_depth.cols());

  for (size_t r = 0; r < normal_image_cp.rows(); ++r) {
    for (size_t c = 0; c < normal_image_cp.cols(); ++c) {
      normal_image_cp.at(r, c) = unprojected_matrix.at(r, c).normal();
    }
  }

  cv::Mat cv_normal_image_cp;
  normal_image_cp.toCv(cv_normal_image_cp);
  cv::imshow("normals - cross product", cv_normal_image_cp);

  // ia sliding window normals
  LOG << "normal computation: from a matrix cloud using slinding window\n";
  unprojected_matrix.fill(PointNormal3f());
  unprojector.computeMatrix<WithNormals>(unprojected_matrix, depth_image);

  NormalComputator2DSlidingWindow<UnprojectedMatrixType, 1> normal_computator_sliding;
  SystemUsageCounter::tic();
  normal_computator_sliding.computeNormals(unprojected_matrix);
  LOG << "normal time = " << FG_YELLOW(SystemUsageCounter::toc()) << std::endl;

  ImageVector3f normal_image_sl;
  normal_image_sl.resize(raw_depth.rows(), raw_depth.cols());
  for (size_t r = 0; r < normal_image_sl.rows(); ++r) {
    for (size_t c = 0; c < normal_image_sl.cols(); ++c) {
      normal_image_sl.at(r, c) = unprojected_matrix.at(r, c).normal();
    }
  }

  cv::Mat cv_normal_image_sl;
  normal_image_sl.toCv(cv_normal_image_sl);
  cv::imshow("normals - sliding window", cv_normal_image_sl);

  // ia unorganized pointcloud
  LOG << "normal computation: from a vector cloud using slinding window\n";
  PointNormal3fVectorCloud unprojected_cloud;
  NormalComputator1DSlidingWindow<PointNormal3fVectorCloud, 1> normal_computator_cloud;
  unprojector.compute<WithNormals>(
    std::back_insert_iterator<PointNormal3fVectorCloud>(unprojected_cloud), depth_image);
  SystemUsageCounter::tic();
  normal_computator_cloud.computeNormals(unprojected_cloud);
  LOG << "normal time = " << FG_YELLOW(SystemUsageCounter::toc()) << std::endl;
  LOG << FG_RED("cannot visualize this stuff using opencv, NormalComputator1DSlidindWindow "
                "should be tested with opengl\n");

  cv::waitKey(0);
  return 0;
}
