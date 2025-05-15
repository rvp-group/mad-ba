#pragma once
#include "trajectory_generator.h"
#include <map>
#include <string>
#include <srrg_data_structures/kd_tree.h>

namespace srrg2_solver {
  using namespace srrg2_core;
  
  using KDTree3D=KDTree_<float, 3>;
  using Vector3fVector = std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >;
  using Isometry3fVector = std::vector <Eigen::Isometry3f, Eigen::aligned_allocator<Eigen::Isometry3f> >; 
  using IntVector     = std::vector<int>;                                
  using IntPair       = std::pair<int, int>;
  using IntPairVector = std::vector<IntPair>;
  using StringGenMap  = std::map<std::string, TrajectoryGeneratorPtr>;

  struct PLGOSimulator{
  
    
    Isometry3fVector poses; // path of the robot
    Vector3fVector landmarks; // landmarks seen by the robot
    IntPairVector landmark_associations; // associations, in pose order [pose-id, landmark-id]
    IntPairVector pose_associations;     // associations, in pose order, excludes odometry

    StringGenMap generators; // factory for trajectory generators. 

    Isometry3f landmark_sensor_offset; //sensor_in_robot (if needed)
    
    PLGOSimulator();

    std::string listTrajectoryGenerators();
    
    // invokes a generator by name and feeds it with the argv arguments
    bool generateTrajectory(const std::string& name, const float* argv);

    // helper, seeks the matces in a kd tree of points
    // returns the indices of the points seen, from the isometry (query)
    // the search occurs within a range, and is clipped at a fov centered in
    // the z axis of the isometry
    // if fov==0, all matches in the range are returned
    void findMatches(IntVector& indices,
                     const KDTree3D& search_tree,
                     const Isometry3f& query,
                     const float range,
                     const float fov);

    // spawns a set of random landmarks in the frustum of the sensors
    // for each pose in the trajectory
    void generateLandmarks(const float max_range,
                           const float max_fov, 
                           const int landmarks_per_pose);

    // removes all landmarks that are close than a rangee
    void pruneLandmarks(float distance);

    // constructs the pose associations. no fov considered
    // min_steps: min interval between two poses
    // the associations returned  provide matches with only the past poses
    void makePosePoseAssications(const float range, const int min_steps);

    // constructs the landmark associations.
    // range: max sensor range
    // fov: fov in radians w.r.t the sensor
    // min_seen: min times a point is seen before becoming a landmark
    void makePoseLandmarkAssociations(const float range, const float fov, const int min_seen);
  };
}
