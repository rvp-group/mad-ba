#include <iostream>
#include <iterator>
#include <srrg_image/image.h>
#include <srrg_pcl/point_types.h>
#include <srrg_pcl/point_unprojector.h>

using namespace srrg2_core;
using namespace std;

using namespace srrg2_core;
using MatrixCloudPointNormalIntensity3f=srrg2_core::PointCloud_<
  Matrix_<PointNormalIntensity3f, Eigen::aligned_allocator<PointNormalIntensity3f> > >;
using Projector = PointUnprojectorPinhole_<PointNormalIntensity3fVectorCloud>;

int main(void) {

  ImageFloat depth;
  ImageFloat intensity;

  MatrixCloudPointNormalIntensity3f out;

  Projector projector;
  projector.computeMatrix<WithNormals>(out, depth, intensity);

  return 0;

}
