#pragma once
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <memory>
#include <srrg_config/configurable.h>
#include <srrg_data_structures/kd_tree.h>
#include <srrg_geometry/geometry3d.h>
#include <srrg_solver/solver_core/variable.h>

namespace srrg2_solver {
  using namespace srrg2_core;

  
  using KDTree3D=KDTree_<float, 3>;
  using Vector3fVector = std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >;
  using Isometry3fVector = std::vector <Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> >; 
  using IntVector     = std::vector<int>;                                
  using IntPair       = std::pair<int, int>;
  using IntPairVector = std::vector<IntPair>;

  struct SimSensorModeBase: public Configurable {
    virtual ~SimSensorModeBase();
    // returns number of items in the layer
    virtual int size() const = 0;
    // call once after populating the layer
    // removes duplicates/too dense items
    // constructs search strucures
    virtual void setup()=0;
    virtual VariableBase::Id graphId(int id) {return id;}
  };
  using SimSensorModeBasePtr = std::shared_ptr<SimSensorModeBase>;



}
