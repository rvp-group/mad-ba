#include "sim_sensor_mode_3d_landmarks.h"

namespace srrg2_solver {
  using namespace srrg2_core;
  void SimSensorMode3DLandmarks::addLandmark(const Vector3f& v) {
    landmarks.push_back(v);
  };
    
  void SimSensorMode3DLandmarks::sparsify(){
    float distance=param_min_landmark_distance.value();
    std::cerr << "pruning landmarks... ";
    Vector3fVector src=landmarks;
    landmarks.clear();
    landmarks.reserve(src.size());
    std::vector<bool> taints(src.size(), false);
    KDTree3D search_tree(src, distance);
    Vector3fVector answer_points;
    IntVector answer_indices;
    for (size_t i=0; i<src.size(); ++i) {
      if (taints[i])
        continue;
      landmarks.push_back(src[i]);
      search_tree.findNeighbors(answer_points, answer_indices, src[i], distance, KDTreeSearchType::Complete);
      for(size_t t=0; t<answer_indices.size(); ++t) {
        int idx=answer_indices[t];
        taints[idx]=true;
      }
    }
    _search_tree.reset(new KDTree3D(landmarks, distance));
    std::cerr << "Pruning  (" <<src.size() << "/" << landmarks.size() << ")" << std::endl;
    
  }

  void SimSensorMode3DLandmarks::setup(){
    sparsify();
    landmarks_num_times_seen.resize(landmarks.size());
    std::fill(landmarks_num_times_seen.begin(), landmarks_num_times_seen.end(), 0);
  }


  void SimSensorMode3DLandmarks::findMatches(IntVector& answer_indices,
                                      const Vector3f& query,
                                      const float range) {
    Vector3fVector answer_points;
    answer_indices.clear();
    _search_tree->findNeighbors(answer_points,
                              answer_indices,
                              query,
                              range,
                              KDTreeSearchType::Complete);
  }

}
