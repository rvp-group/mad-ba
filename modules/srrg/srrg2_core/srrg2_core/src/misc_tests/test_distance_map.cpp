#include "srrg_data_structures/path_matrix_distance_search.h"
#include "srrg_image/image.h"
#include "srrg_pcl/point_types.h"
#include "srrg_system_utils/system_utils.h"

using namespace srrg2_core;
using namespace std;

int main(int argc, char** argv) {
  int rows       = 480;
  int cols       = 640;
  int num_points = 100;

  // create a container with the random points;
  Point2fVectorCloud points;
  points.resize(num_points);
  for (size_t i = 0; i < points.size(); ++i) {
    Point2f p;
    p.coordinates() = Vector2f(rows * drand48(), cols * drand48());
    points[i]       = p;
  }

  PathMatrix distance_map(rows, cols);
  ImageUInt8 path_map_image;
  ImageInt parent_map_image;
  cv::Mat m, p;
  const int d_max = 100;
  cv::namedWindow("distance map");
  cv::namedWindow("parent map");

  int d_curr = 0;

  PathMatrixDistanceSearch dmap_calculator;

  dmap_calculator.setPathMatrix(&distance_map);
  // loop over several max_distance_squared_pxl to let you see that the distance map is growing
  // for visualization purpose
  while (d_curr < d_max) {
    dmap_calculator.param_max_distance_squared_pxl.setValue(d_curr * d_curr);
    double t_start = getTime();
    dmap_calculator.setGoals(points);
    dmap_calculator.compute();
    double t_end = getTime();
    distance_map.toImage(path_map_image, PathMatrix::Distance);
    distance_map.toImage(parent_map_image, PathMatrix::Parent);
    path_map_image.toCv(m);
    parent_map_image.toCv(p);
    cv::imshow("distance map", m);
    cv::imshow("parent map", p);

    cv::waitKey();
    std::cerr << "dist: " << d_curr << " time: " << t_end - t_start << std::endl;
    d_curr++;
  }
  cerr << "terminating" << endl;

  return 0;
}
