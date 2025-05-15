#pragma once
#include "sim_sensor_mode_base.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  struct SimSensorMode3DLandmarks: public SimSensorModeBase {
    PARAM(PropertyFloat,
          min_landmark_distance,
          "minimum distance at which two landmarks can be [m]",
          1.f,
          0);
    void setup() override;
    void addLandmark(const Vector3f& v);    
    void findMatches(IntVector& matches,
                     const Vector3f& query,
                     const float range);
    int size() const override{return landmarks.size();}
    VariableBase::Id graphId(int id) {return id+offset;}

    Vector3fVector landmarks;
    std::vector<int> landmarks_num_times_seen;
    std::unique_ptr<KDTree3D> _search_tree;
    VariableBase::Id offset=0;
  protected:
    void sparsify();

  };

  
  using SimSensorMode3DLandmarksPtr=std::shared_ptr<SimSensorMode3DLandmarks>;

}
