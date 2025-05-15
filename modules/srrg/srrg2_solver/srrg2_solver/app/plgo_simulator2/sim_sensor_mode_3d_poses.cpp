#include "sim_sensor_mode_3d_poses.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  void SimSensorMode3DPoses::addPose(const Isometry3f& iso) {
    poses.push_back(iso);
    origins.push_back(iso.translation());
  }

  void SimSensorMode3DPoses::setup(){
    _search_tree.reset(new KDTree3D(origins, 1));
  }

  void SimSensorMode3DPoses::findMatches(IntVector& answer_indices,
                                      const Isometry3f& query,
                                      const float range) {
    Vector3fVector answer_points;
    answer_indices.clear();
    _search_tree->findNeighbors(answer_points,
                              answer_indices,
                              query.translation(),
                              range,
                              KDTreeSearchType::Complete);

  }

}
