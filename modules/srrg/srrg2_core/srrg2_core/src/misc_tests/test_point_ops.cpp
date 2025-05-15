#include "srrg_pcl/point.h"
#include "srrg_pcl/point_base.h"
#include "srrg_pcl/point_color.h"
#include "srrg_pcl/point_example.h"
#include "srrg_pcl/point_intensity_descriptor.h"
#include "srrg_pcl/point_normal.h"
#include "srrg_pcl/point_normal_color.h"

using namespace srrg2_core;
using namespace std;

int main(int argc, char** argv) {
  cerr << "OFFSETS" << endl;
  cerr << MyPointWithNormal3f::Dimensions << endl;
  cerr << MyPointWithNormal3f::FieldOffset<0> << endl;
  cerr << MyPointWithNormal3f::FieldOffset<1> << endl;
  cerr << MyPointWithNormal3f::FieldOffset<2> << endl;
  cerr << MyPointWithNormal3f::FieldOffset<3> << endl;

  Isometry3f iso;
  Vector6f v;
  v << 1, 2, 3, 0.2, 0.3, 0.4;
  iso = srrg2_core::geometry3d::ta2t(v);

  MyPointWithNormal3f p, p2, p_dio;
  p.coordinates() << 1, 2, 3;
  p.normal() << .1, .2, .3;
  p.normal().normalize();
  p.weight() = 10.f;

  //  p_dio =  iso * p;

  cerr << "in place mult" << endl;
  p *= 0.5f;
  p.toStream(cerr) << endl;

  cerr << "mult with assignment" << endl;
  MyPointWithNormal3f np = p * 3.5f;
  np.toStream(cerr) << endl;

  cerr << "in place scaling field 2" << endl;
  p.operator*=<2>(0.5f);
  p.toStream(cerr) << endl;

  cerr << "in place scaling field 2 (with copy)" << endl;
  p = p.operator*<2>(4.f);
  p.toStream(cerr) << endl;

  cerr << "in place adding" << endl;
  p += p2;
  p.toStream(cerr) << endl;

  cerr << "in place field adding" << endl;
  p.operator+=<2>(p2);
  p.toStream(cerr) << endl;

  iso.setIdentity();
  iso.translation() << 10, 20, 30;

  cerr << "transform in place" << endl;
  p.transformInPlace<TRANSFORM_CLASS::Isometry, Isometry3f>(iso);
  p.toStream(cerr);
  cerr << endl;

  cerr << "transform in place field 0" << endl;
  p.toStream(cerr) << endl;
  p.template transformInPlace<0, TRANSFORM_CLASS::Isometry, Isometry3f>(iso);
  p.toStream(cerr) << endl;

  cerr << "transform field 1 (with copy)" << endl;
  p2 = p.transform<1, TRANSFORM_CLASS::Isometry, Isometry3f>(iso);
  //  p_dio = iso * p;
  // std::cerr << "P_DIO guarda qui\n";
  p2.toStream(cerr) << endl;
  // p_dio.toStream(cerr) << endl;

  using PlainVectorType = typename MyPointWithNormal3f::PlainVectorType<float>;
  PlainVectorType plain;

  p.toPlainVector(plain);
  cerr << plain.transpose() << endl;

  plain = PlainVectorType::Random();
  p.fromPlainVector(plain);
  cerr << "PLAIN_VECTOR" << endl;
  cerr << "p: ";
  p.toStream(cerr) << endl;
  cerr << "v: " << plain.transpose() << endl;

  MyPointWithNormal3f p3 = p.euclidean2polar();
  p3.toPlainVector(plain);
  cerr << plain.transpose() << endl;
  cerr << "v2: " << plain.transpose() << endl;

  MyPointWithNormal3f p4 = p3.polar2euclidean();
  p4.toPlainVector(plain);
  cerr << "v3: " << plain.transpose() << endl;

  Point2f p2f;
  p2f.coordinates() = Vector2f::Random();
  Point4i p4i;
  p4i.coordinates() << 1, -6, 8, 10;

  cerr << "p2f: ";
  p2f.toStream(cerr) << endl;
  cerr << "p4i: ";
  p4i.toStream(cerr) << endl;

  Point4dVectorCloud point4d_vector_cloud;
  point4d_vector_cloud.resize(10);
  float k = 0;
  for (Point4d& p : point4d_vector_cloud) {
    p.coordinates() = (Vector4d::Ones() * k++);
  }
  for (const Point4d& p : point4d_vector_cloud) {
    cerr << "p4d_element: ";
    p.toStream(cerr) << endl;
  }

  PointNormal3f pn3f;
  pn3f.coordinates() = Vector3f::Random();
  pn3f.normal()      = Vector3f::Random();
  pn3f.normal().normalize();
  cerr << "pn3f: ";
  pn3f.toStream(cerr) << endl;

  PointColor2i pc2i;
  pc2i.coordinates() << 1, -10;
  pc2i.color() = Vector3f::Random();
  cerr << "pc2i: ";
  pc2i.toStream(cerr) << endl;

  PointNormalColor3f pnc3f;
  pnc3f.coordinates() << 0.1, 0.2, -99.99;
  pnc3f.normal() = Vector3f::Random();
  pnc3f.normal().normalize();
  pnc3f.color() = Vector3f::Random();
  cerr << "pcn3f: ";
  pnc3f.toStream(cerr) << endl;

  const PointNormal3f rottercazz = pn3f;
  pn3f += rottercazz * 1.f;

  return 0;
}
