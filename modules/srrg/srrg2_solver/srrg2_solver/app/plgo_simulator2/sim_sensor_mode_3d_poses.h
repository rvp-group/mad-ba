#pragma once
#include "sim_sensor_mode_base.h"

namespace srrg2_solver {
  using namespace srrg2_core;


  
  struct SimSensorMode3DPoses: public SimSensorModeBase {
    PARAM(PropertyFloat,
          min_landmark_distance,
          "minimum distance at which two landmarks can be [m]",
          1.f,
          0);
    void addPose(const Isometry3f& iso);    
    void setup() override;
    void findMatches(IntVector& matches,
                     const Isometry3f& query,
                     const float range);
    int size() const override{return poses.size();}
    Isometry3fVector poses;
    Vector3fVector origins;
    std::unique_ptr<KDTree3D> _search_tree;
  };

  using SimSensorMode3DPosesPtr=std::shared_ptr<SimSensorMode3DPoses>;

}
