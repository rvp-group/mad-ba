#include <srrg_data_structures/matrix.h>
#include <srrg_point_cloud/point_types.h>
#include <srrg_point_cloud/point_transformer_affine.h>
#include <srrg_point_cloud/point_transformer_pinhole.h>
#include <srrg_point_cloud/point_predicates.h>
#include <srrg_point_cloud/point_cloud_ops.h>
#include <srrg_point_cloud/point_utils.h>

#include <srrg_system_utils/system_utils.h>

#include <vector>
#include <iostream>

#include <chrono>

using namespace std;
using namespace srrg2_core;

typedef PointNormal3f Point;
typedef std::vector<Point, Eigen::aligned_allocator<Point> > PointVector;

template <typename PointContainer_, typename TransformTypeForward, typename TransformTypeBackward>
void checkTransform(const std::string test_name,
                    const PointContainer_& cloud,
                    TransformTypeForward& forward,
                    TransformTypeBackward& backward){
  cerr << "********** BEGIN " << test_name << " **********" << endl;
  cerr << " FORWARD                                 " << endl;
  PointContainer_ dest;
  auto transform_start = std::chrono::high_resolution_clock::now();
  PointCloudOps::transform(dest,
                           cloud,
                           forward);
  auto transform_end = std::chrono::high_resolution_clock::now();
  double transform_time = std::chrono::duration_cast<std::chrono::milliseconds>(transform_end- transform_start).count();
  std::cerr << "Transformed " << cloud.size() << " points in " << transform_time << " millisecond!" << std::endl;

  cerr << " BACKWARD                                 " << endl;
  auto btransform_start = std::chrono::high_resolution_clock::now();
  PointCloudOps::transform(dest,
                           dest,
                           backward);
  auto btransform_end = std::chrono::high_resolution_clock::now();
  double btransform_time = std::chrono::duration_cast<std::chrono::milliseconds>(btransform_end- btransform_start).count();
  std::cerr << "Transformed " << cloud.size() << " points in " << btransform_time << " millisecond!" << std::endl;

  std::cerr << "sanity check: " << endl;
  int num_good=0;
  for(size_t i = 0; i < cloud.size(); ++i){
    if (!cloud.at(i).good)
      continue;
    if (!dest.at(i).good)
      continue;
    num_good++;
    if ( (cloud.at(i).coeffs()-dest.at(i).coeffs()).squaredNorm()>1e-3){
      auto c1=dest.at(i).coeffs();
      auto c2=cloud.at(i).coeffs();
      cerr << c1.transpose() << std::endl;
      cerr << c2.transpose() << std::endl;
      auto delta=c1-c2;
      cerr << delta.transpose() << std::endl;
      throw std::runtime_error("error in zero should be zero");
    }
  }
  std::cerr << "all fine (" << num_good << "/" << cloud.size() << ")" << endl;
  cerr << "********** END " << test_name << " **********" << endl;
}


template <typename PointContainer_, typename SelectPredicate>
void checkSelect(const std::string test_name,
                 const PointContainer_& cloud,
                 SelectPredicate& predicate){
  cerr << "********** BEGIN " << test_name << " **********" << endl;
  std::vector<int> indices;

  auto select_start = std::chrono::high_resolution_clock::now();
  PointCloudOps::applyPredicate(indices,
                                cloud,
                                predicate);
  auto select_end = std::chrono::high_resolution_clock::now();
  double select_time = std::chrono::duration_cast<std::chrono::milliseconds>(select_end- select_start).count();
  std::cerr << "Selected (idx) " << cloud.size() << " points in " << select_time << " millisecond!" << std::endl;

  PointContainer_ selected;
  select_start = std::chrono::high_resolution_clock::now();
  PointCloudOps::applyPredicate(selected,
                                cloud,
                                predicate);
  select_end = std::chrono::high_resolution_clock::now();
  select_time = std::chrono::duration_cast<std::chrono::milliseconds>(select_end- select_start).count();
  std::cerr << "Selected (copy)" << selected.size() << " points in " << select_time << " millisecond!" << std::endl;

  typedef typename PointContainer_::value_type Point;
  
  std::cerr << "sanity check: " << indices.size() << "/" << selected.size() << endl;
  
  int num_good=0;
  for(size_t i = 0; i < indices.size(); ++i){
    size_t idx=indices[i];
    const Point& psrc=cloud.at(idx);
    const Point& pdest=selected.at(i);
    if ( (psrc.coeffs()-pdest.coeffs()).squaredNorm()>1e-3){
      auto c1=psrc.coeffs();
      auto c2=pdest.coeffs();
      cerr << c1.transpose() << std::endl;
      cerr << c2.transpose() << std::endl;
      auto delta=c1-c2;
      cerr << delta.transpose() << std::endl;
      throw std::runtime_error("error in zero should be zero");
    }
    ++num_good;
  }
  std::cerr << "all fine (" << num_good << "/" << cloud.size() << ")" << endl;
  cerr << "********** END " << test_name << " **********" << endl;
}


template <typename PointContainer_, typename PartitionPredicate_>
void checkPartition(const std::string test_name,
                    const PointContainer_& cloud,
                    PartitionPredicate_& predicate){
  
  cerr << "********** BEGIN " << test_name << " **********" << endl;
  std::vector<int> partitions;
  std::vector<int> num_items;
  auto partition_start = std::chrono::high_resolution_clock::now();
  PointCloudOps::partition(partitions,
                           num_items,
                           cloud,
                           predicate);

  auto partition_end = std::chrono::high_resolution_clock::now();
  double partition_time = std::chrono::duration_cast<std::chrono::milliseconds>(partition_end- partition_start).count();

  std::cerr << "partitioned" << cloud.size() << " points in " << partition_time << " millisecond!" << std::endl;
  std::cerr << "num_partitions" << num_items.size() << std::endl;
  ;

  size_t num_partitioned_points=0;
  for (size_t k=0; k<num_items.size(); ++k)
    num_partitioned_points+=num_items[k];
  std::cerr << "points partitioned: " << num_partitioned_points << std::endl;
 
  cerr << "********** END " << test_name << " **********" << endl;
}

template <typename PointContainer_, typename PartitionPredicate_>
void checkVoxelize(const std::string test_name,
                   const PointContainer_& cloud,
                   PartitionPredicate_& predicate){
  
  cerr << "********** BEGIN " << test_name << " **********" << endl;
//  typedef typename PointContainer_::value_type PointType;
  PointContainer_ dest;
  auto voxelize_start = std::chrono::high_resolution_clock::now();
  PointCloudOps::voxelize(dest,
                          cloud,
                          predicate);
  auto voxelize_end = std::chrono::high_resolution_clock::now();
  double voxelize_time = std::chrono::duration_cast<std::chrono::milliseconds>(voxelize_end- voxelize_start).count();

  std::cerr << "voxelized" << cloud.size() << " points in " << voxelize_time << " millisecond!" << std::endl;
  std::cerr << "num_voxels" << dest.size() << std::endl;
  ;
  cerr << "********** END " << test_name << " **********" << endl;
}



int main(int argc, char** argv) {

  // generate a Random PointVector
  const int number_of_points = 10000000;
  PointVector src;
  src.resize(number_of_points);
  for(int i = 0; i < number_of_points; ++i){
    Point& p=src[i];
    p=PointNormal3f(Eigen::Vector3f::Random()*10,
                    Eigen::Vector3f::Random());
    p.normalize();
  }

  // make an isometry
  Isometry3_<float> random_T;
  random_T.setIdentity();
  random_T.translation() = Eigen::Vector3f::Random()*10;
  Eigen::Matrix3f rotation;
  rotation = Eigen::AngleAxisf(0.25*M_PI, Vector3f::UnitX())
    * Eigen::AngleAxisf(0.5*M_PI,  Vector3f::UnitY())
    * Eigen::AngleAxisf(0.33*M_PI, Vector3f::UnitZ());
  random_T.linear() = rotation;
  std::cerr << "random T: " << std::endl << random_T.matrix() << std::endl;


  // test affine transformation
  {
    PointTransformerAffine_<Point> affine_forward(random_T);
    PointTransformerAffine_<Point> affine_backward(random_T.inverse());
    checkTransform("Affine", src,affine_forward, affine_backward);

  }
 
  // test pinhole transformation, and back
  {
    Eigen::Matrix3f pinhole_camera_matrix;
    pinhole_camera_matrix<<
      150, 0,   160,
      0,   150, 120,
      0,   0,   1;
    Eigen::Vector2f pinhole_sizes(320,240);
    float pinhole_z_near=0.6;
    float pinhole_z_far=20;
    PointTransformerPinhole_<Point, Project> pinhole_forward(random_T,
                                                             pinhole_camera_matrix,
                                                             pinhole_sizes,
                                                             pinhole_z_near,
                                                             pinhole_z_far);

    PointTransformerPinhole_<Point, Unproject> pinhole_backward(random_T,
                                                                pinhole_camera_matrix,
                                                                pinhole_sizes,
                                                                pinhole_z_near,
                                                                pinhole_z_far);

    checkTransform("Pinhole", src,pinhole_forward, pinhole_backward);
  }

  //test selection (clipping)
  {
    Point center(Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,0,0));
    Point scales(Eigen::Vector3f(.5,.5,.5), Eigen::Vector3f(0,0,0));
    PointInBallPredicate_<Point> in_ball(center, scales);
    checkSelect("InBallSelect", src, in_ball);
  }

  //test partitioning
  {
    Point::CoeffType voxel_sizes;
    voxel_sizes<<
      2, 2, 2, 10, 10, 10;
    PointVoxelCompare_<Point> voxel_compare(voxel_sizes);
    checkPartition("Voxel Partition", src,  voxel_compare);
  }

  //test voxelize
  {
    Point::CoeffType voxel_sizes;
    voxel_sizes<<
      2, 2, 2, 10, 10, 10;
    PointVoxelCompare_<Point> voxel_compare(voxel_sizes);
    checkVoxelize("Voxelize", src,  voxel_compare);
  }


  // test project
  {
    std::cerr << "*********** START Projection ************" << std::endl;
    Eigen::Matrix3f pinhole_camera_matrix;
    pinhole_camera_matrix<<
      150, 0,   160,
      0,   150, 120,
      0,   0,   1;
    Eigen::Vector2f pinhole_sizes(320,240);
    float pinhole_z_near=0.6;
    float pinhole_z_far=20;
    random_T.setIdentity();
    PointTransformerPinhole_<Point, Project> pinhole_forward(random_T,
                                                             pinhole_camera_matrix,
                                                             pinhole_sizes,
                                                             pinhole_z_near,
                                                             pinhole_z_far);

    PointTransformerPinhole_<Point, Unproject> pinhole_backward(random_T,
                                                                pinhole_camera_matrix,
                                                                pinhole_sizes,
                                                                pinhole_z_near,
                                                                pinhole_z_far);

    Matrix_<Point> dest_matrix;
    Matrix_<int> indices;
    double project_start = getTime();
    PointCloudOps::project(dest_matrix, indices, src, pinhole_forward);
    double project_end = getTime();
    
    Matrix_<float> dest_depth;
    dest_depth.resize(dest_matrix.rows(), dest_matrix.cols());
    dest_depth.fill(0.f);

    for(size_t i=0; i<dest_matrix.data().size(); ++i) {
      dest_depth.data()[i]=dest_matrix.data()[i].coordinates.z();
    }
    PointVector dest_cloud;

    double unproject_start = getTime();
    PointCloudOps::unproject(dest_cloud, dest_depth, pinhole_backward);
    double unproject_end = getTime();
    

    Matrix_<Point> dest_matrix2;
    Matrix_<int> indices2;
    double project2_start = getTime();
    PointCloudOps::project(dest_matrix, indices2, dest_cloud, pinhole_forward);
    double project2_end = getTime();

    std::cerr << "Project Time: " << project_end - project_start << std::endl;
    std::cerr << "Unproject Time: " << unproject_end - unproject_start << std::endl;    
    std::cerr << " unprojected points size: " << dest_cloud.size() << endl;
    std::cerr << "Project2 Time: " << project2_end - project2_start << std::endl;
    


    
    std::cerr << "*********** END Projection ************" << std::endl;    
  }

  return 0;
}
