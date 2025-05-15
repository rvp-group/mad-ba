#include <iostream>
#include <iterator>
#include "srrg_pcl/point_cloud.h"
#include "srrg_pcl/point_example.h"
#include "srrg_pcl/point_projector_pinhole.h"
#include "srrg_pcl/point_unprojector.h"
#include "srrg_geometry/geometry_defs.h"
#include "srrg_geometry/geometry3d.h"

using namespace srrg2_core;
using namespace std;

#define NUM_STEPS 100

using UnprojectorType=PointUnprojectorPinhole_<MyPointWithNormal3fCloud>;
using ProjectorType = PointProjectorPinhole_<MyPointWithNormal3fCloud>;
static constexpr TRANSFORM_CLASS projection_class=PinholeProjection;
static constexpr TRANSFORM_CLASS unprojection_class=PinholeUnprojection;
using ProjectedMatrixType=ProjectorType::TargetMatrixType;

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


int main(int argc, char** argv) {

  /* PLAYGROUND */

  // declare 2 vector clouds
  MyPointWithNormal3fCloud vec, other;
  int num_points=1000000;
  fillVector(vec,num_points);
  other=vec;
  
  // an isometry
  Isometry3f iso;
  Vector6f v;
  v<< 0.1, 0.2, 0.3, 0.2, 0.2, 0.2;
  v.setZero();
  iso=srrg2_core::geometry3d::ta2t(v);
  
  // image sizes (for projection)
  int image_cols=320;
  int image_rows=240;


  // camera matrix
  Matrix3f K;
  K <<
    100, 0,   image_cols/2.0f,
    0,   100, image_rows/2.0f,
    0,   0,   1;

  // projective transform and inverse
  using Transform3f = Eigen::Transform<float, 3, Eigen::Affine>;
  Transform3f trans;
  trans.linear()=K*iso.linear();
  trans.translation()=K*iso.translation();

  Transform3f invtrans;
  invtrans.linear()=iso.linear().transpose()*K.inverse();
  invtrans.translation()=-iso.linear().transpose()*iso.translation();

  /* in_place transformation */
  cerr << "test_transform_in_place" << endl;
  for (int k=0; k<NUM_STEPS; ++k){
    other=vec;
    other.transformInPlace<TRANSFORM_CLASS::Isometry>(iso);
    cerr << ".";
  }
  cerr << "DONE" << endl;

  /* in_place transformation field by field */
  cerr << "test_transform_field_in_place" << endl;
  for (int k=0; k<NUM_STEPS; ++k){
    other=vec;
  // project coordinates
    other.transformInPlace<0, projection_class, Transform3f>(trans);
    // transform normals
    other.transformInPlace<1, TRANSFORM_CLASS::Isometry, Isometry3f>(iso);
    //other.transformInPlace<2, TRANSFORM_CLASS::Isometry, Isometry3f>(iso);
    cerr << ".";
  }
  cerr << "DONE" << endl;

  /* transformation (with append) */
  cerr << "test_transform" << endl;
  for (int k=0; k<NUM_STEPS; ++k){
    other.clear();
    std::back_insert_iterator<MyPointWithNormal3fCloud> insertor(other);
    vec.transform<TRANSFORM_CLASS::Isometry>(insertor, iso);
     cerr << ".";
  }
  cerr << "DONE" << endl;

  /* transformation (with append), field by field */
  cerr << "test_transform_field_by_field" << endl;
  for (int k=0; k<NUM_STEPS; ++k){
    other.clear();
    std::back_insert_iterator<MyPointWithNormal3fCloud> insertor(other);
    vec.transform<0, projection_class>(insertor, trans);
    // transform normals (in place, as they are copied)
    other.transformInPlace<1, TRANSFORM_CLASS::Isometry, Isometry3f>(iso);
    cerr << ".";
  }
  cerr << "DONE" << endl;

  /* projective transformation and back (no image sizes, no depth buffer) */
  cerr << "test_projection/unprojection (from cloud)" << endl;
  for (int k=0; k<NUM_STEPS; ++k){
    MyPointWithNormal3fCloud proj;
    std::back_insert_iterator<MyPointWithNormal3fCloud> proj_ins(proj);
    vec.transform<projection_class>(proj_ins, trans);

    MyPointWithNormal3fCloud unproj;
    std::back_insert_iterator<MyPointWithNormal3fCloud> unproj_ins(unproj);
    proj.transform<unprojection_class>(unproj_ins, invtrans);


    MyPointWithNormal3fCloud proj2;
    std::back_insert_iterator<MyPointWithNormal3fCloud> proj2_ins(proj2);
    unproj.transform<projection_class>(proj2_ins, trans);

    MyPointWithNormal3fCloud unproj2;
    std::back_insert_iterator<MyPointWithNormal3fCloud> unproj2_ins(unproj2);
    proj2.transform<unprojection_class>(unproj2_ins, invtrans);

    if (!k) {
      cerr  << "orig: " << vec.size()
            << " proj:" <<  proj.size()
            << " unproj:" <<  unproj.size()
            << " proj2:" <<  proj2.size()
            << " unproj2:" <<  unproj2.size()
            << endl;

      auto it1=unproj.begin();
      auto it2=unproj2.begin();
      while(it1!=unproj.end()) {
        MyPointWithNormal3fCloud::PlainVectorType v1,v2;
        it1->toPlainVector(v1);
        it2->toPlainVector(v2);
        if ((v1-v2).squaredNorm()>1e-3){
          cerr << "proj/unproj error" << endl;
        }
        ++it1;
        ++it2;
      }
    }
    cerr << ".";
  }
  cerr << "DONE" << endl;
  
  /* test projector object */
  cerr << "test_projector" << endl;

  // declare object
  ProjectorType projector;
  // configure
  projector.setCameraPose(iso);
  projector.setCameraMatrix(K);

  // set canvas (will contain result and define the sizes)
  ProjectedMatrixType projected_matrix;
  projected_matrix.resize(image_rows,image_cols);

  // how many points were good?
  int num_good=0;
  for (int k=0; k<NUM_STEPS; ++k){
    num_good = projector.compute(projected_matrix, vec.begin(), vec.end());
    if (k==0)
      cerr << "num_good: " << num_good << endl;
    cerr << ".";
  }
  cerr << "DONE" << endl;

  /* test un projector object */
  cerr << "test_unprojector" << endl;

  // declare and configure object
  UnprojectorType unprojector;
  unprojector.setCameraPose(iso);
  unprojector.setCameraMatrix(K);



  // we need to generate channel matrices for our output
  // by extracting them from the result of projection
  // extract the depths from the result of projection
  Matrix_<float> depths;
  // we initialize de depths by copying the  depths from the
  // projected matrix;
  projected_matrix.toDepthMatrix(depths);
  
  // we fill the remaining channels at our will
  Matrix_<float> weights(image_rows, image_cols);
  weights.fill(1.0f);
  Matrix_<Stuff> stuffs(image_rows, image_cols);
  Stuff s;
  s.x="puke";
  stuffs.fill(s);
  
  // if we want to unproject the elements
  // in an organized cloud, we ask for the type to the projector;
  using UnprojectedMatrixType= PointCloud_<Matrix_<MyPointWithNormal3f, Eigen::aligned_allocator<MyPointWithNormal3f> > >; //typename UnprojectorType::DestMatrixType;
  UnprojectedMatrixType unprojected_matrix;
  unprojected_matrix.resize(image_rows, image_cols);
  int unp_mat_valid=unprojector.computeMatrix<WithNormals>(unprojected_matrix, depths, weights, stuffs);
  cerr << "matrix unprojected valid: " << unp_mat_valid << endl;

    
  // unprojection of cloud (with normals)
  MyPointWithNormal3fCloud unprojected_cloud;
  for (int k=0; k<NUM_STEPS; ++k){
    unprojected_cloud.clear();
    std::back_insert_iterator<MyPointWithNormal3fCloud> insertor(unprojected_cloud);
    unprojector.compute<WithNormals>(insertor, depths);
    if (k==0) {
      cerr << "unprojected size=" << unprojected_cloud.size() << endl;
    }
    cerr << ".";
  }
  cerr << "DONE" << endl;
  /*
  for (int i=0; i<unprojected_cloud.size(); ++i ){
    cout <<  i/image_cols << " " << i%image_cols << " ";
    unprojected_cloud[i].toStream(cout) << endl;
  }
  */
  
  /*Test for the projection/unprojection*/
  
  // reconstructing projected cloud
  cerr << "consistency (unproject->unproject->project)" << endl;
  ProjectedMatrixType reprojected_matrix;
  reprojected_matrix.resize(image_rows, image_cols);
  int num_p_valid=projector.compute(reprojected_matrix,
                                    unprojected_cloud.begin(),
                                    unprojected_cloud.end());
  cerr << "p valid=" << num_p_valid << endl;

  int num_p2p_good=0;
  int num_p2p_bad=0;
  int depth_p2p_fail=0;
  int trans_p2p_good=0;
  int trans_p2p_bad=0;
  for (size_t r=0; r<reprojected_matrix.rows(); ++r) {
    for (size_t c=0; c<reprojected_matrix.cols(); ++c) {
      if (reprojected_matrix.at(r,c).projected.status!=Valid) {
        ++ num_p2p_bad;
        continue;
      } 
        
      ++num_p2p_good;
      float d=reprojected_matrix.at(r,c).depth;
      float d2=depths.at(r,c);
      if (fabs(d-d2)>1e-3){
        ++ depth_p2p_fail;
      }
      MyPointWithNormal3f& original_pt=unprojected_matrix.at(r,c);
      float delta_t=(original_pt.coordinates() - iso*reprojected_matrix.at(r,c).transformed.coordinates()).squaredNorm();
      if (delta_t>1e-3) {
        ++trans_p2p_bad;
      } else {
        ++trans_p2p_good;
      }
    }
  }

  cerr << "Consistency check, good: " << num_p2p_good
       << " bad: " << num_p2p_bad 
       << " depth_fail: " << depth_p2p_fail 
       << " trans_fail: " << trans_p2p_bad << endl;
    
  /* VOXELIZATION TEST */
  cerr << "test_voxelization" << endl;
  vec.normalize();
  for (int k=0; k<NUM_STEPS; ++k){
    
    // we need to set the dimensions for the voxelizer
    MyPointWithNormal3fCloud::PlainVectorType voxel_sizes;
    voxel_sizes << 2, 2, 2, 0.5, 0.5, 0.5, 0;
    MyPointWithNormal3fCloud voxelized;
    std::back_insert_iterator<MyPointWithNormal3fCloud> vox_ins(voxelized);
    // call the voxelize with the insertor
    vec.voxelize(vox_ins, voxel_sizes);
    
    if (!k) {
      cerr  << "orig: " << vec.size()
            << " voxelized:" <<  voxelized.size()
            << endl;
    }
    cerr << ".";
  }
  cerr << "DONE" << endl;

  
  /* CLIP TEST */
  cerr << "test_clip" << endl;
  vec.clear();
  fillVector(vec,num_points);
  vec.normalize<1>();
  for (int k=0; k<NUM_STEPS; ++k){

    // define the origin of the clipping box
    MyPointWithNormal3fCloud::PointType origin;
    MyPointWithNormal3fCloud::PlainVectorType zero;
    zero.setZero();
    origin.fromPlainVector(zero);
    origin.normal() << 0.5, 0.3, 0.3;
    origin.normalize();

    // define the size of clipping box
    MyPointWithNormal3fCloud::PlainVectorType box_sizes;
    box_sizes << 5, 5, 5, 0.5, 0.5, 0.5, 0;

    // do clip (same as voxelize)
    MyPointWithNormal3fCloud clipped;
    std::back_insert_iterator<MyPointWithNormal3fCloud> clip_ins(clipped);
    vec.clip(clip_ins, origin, box_sizes);

    if (!k) {
      cerr  << "orig: " << vec.size()
            << " clipped:" <<  clipped.size()
            << endl;
    }
    cerr << ".";
  }
  cerr << "DONE" << endl;
  

  return 0;
}
