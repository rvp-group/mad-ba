#include "memory_vector_interface.h"
#include "file_vector_interface.h"
#include <iostream>

#include <Eigen/Core>

using namespace srrg2_core;
using namespace std;


using PointVector = VectorInterface_<Eigen::Vector3f>;
using PointVectorM = MemoryVectorInterface_<Eigen::Vector3f>;
using PointVectorF = FileVectorInterface_<Eigen::Vector3f>;


int main() {
  int num_points=10;
  {
    PointVectorM points_m;
    points_m.resize(num_points);
    int k=0;
    for(auto& p: points_m) {
      p=Eigen::Vector3f(k,k,k);
      ++k;
    }
    {
      FileHandlePtr points_file_handle(new FileHandle("points.dat",false, true));
      PointVectorF points_f(points_file_handle);
      points_f=points_m;
    }
  }
  
  {
    FileHandlePtr points_file_handle(new FileHandle("points.dat", true));
    PointVectorF points_f(points_file_handle);
    for(const auto& p: points_f) {
      cerr << p.transpose() << endl;
    }
  }


  
}
