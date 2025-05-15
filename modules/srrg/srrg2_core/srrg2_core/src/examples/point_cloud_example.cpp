#include <srrg_pcl/point_types.h>

#include <iostream>

#define EXAMPLE_LOG std::cerr << "srrg2_core::examples::point_cloud_example| "

using namespace std;
using namespace srrg2_core;

int main(int argc, char** argv) {
  const size_t num_points = 1000;
  EXAMPLE_LOG << "Generating a Cloud w/ " << num_points << " points" << endl;

  Point3fVectorCloud point_cloud;
  point_cloud.resize(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    Point3f p;
    p.coordinates() = Vector3f::Random();
    point_cloud[i]  = p;
  }

  EXAMPLE_LOG << "Generating a random Transformation:" << endl;
  Isometry3f T;
  T.setIdentity();
  T.translation() = Vector3f::Random();
  Matrix3f R;
  R = AngleAxisf(rand() * M_PI, Vector3f::UnitX()) * AngleAxisf(rand() * M_PI, Vector3f::UnitY()) *
      AngleAxisf(rand() * M_PI, Vector3f::UnitZ());
  T.linear() = R;
  EXAMPLE_LOG << endl << T.matrix() << endl;

  EXAMPLE_LOG << "Applying Transformation(In Place) to the cloud" << std::endl;
  point_cloud.transformInPlace(T);
  EXAMPLE_LOG << "Done" << std::endl;

  EXAMPLE_LOG << "Applying Transformation to the cloud, retrieving the Transformed one"
              << std::endl;
  Point3fVectorCloud point_cloud_transformed;
  std::back_insert_iterator<Point3fVectorCloud> transformed_iterator(point_cloud_transformed);
  point_cloud.transform<TRANSFORM_CLASS::Isometry>(transformed_iterator, T);
  EXAMPLE_LOG << "Done" << std::endl;

  EXAMPLE_LOG << "Voxelizing cloud" << std::endl;
  Vector3f scales;
  scales << 0.1, 0.1, 0.1;
  Point3fVectorCloud point_cloud_voxelized;
  std::back_insert_iterator<Point3fVectorCloud> voxelized_iterator(point_cloud_voxelized);
  point_cloud.voxelize(voxelized_iterator, scales);
  EXAMPLE_LOG << "Done" << std::endl;

  EXAMPLE_LOG << "Clipping cloud" << std::endl;
  Vector3f ranges;
  ranges << 2.0, 2.0, 2.0;
  Point3f origin;
  origin.coordinates().setZero();
  Point3fVectorCloud point_cloud_clipped;
  std::back_insert_iterator<Point3fVectorCloud> clipped_iterator(point_cloud_clipped);
  point_cloud.clip(clipped_iterator, origin, ranges);
  EXAMPLE_LOG << "Done" << std::endl;

  {
    // ds test recursive field copying
    Point3f point_base;
    point_base.coordinates().setZero();
    EXAMPLE_LOG << "base: " << point_base.coordinates().transpose() << std::endl;
    PointNormal3f point_sub;
    point_sub.coordinates().setOnes();
    EXAMPLE_LOG << "sub: " << point_sub.coordinates().transpose() << std::endl;
    point_base.copyFields<PointNormal3f>(point_sub);
    EXAMPLE_LOG << "base after field copy from sub: " << point_base.coordinates().transpose()
                << std::endl;
  }

  return 0;
}
