#include <srrg_geometry/geometry3d.h>
#include "plgo_simulator.h"
#include <fstream>
namespace srrg2_solver {
  using namespace srrg2_core;

  std::string PLGOSimulator::listTrajectoryGenerators() {
    std::ostringstream os;
    for (auto& it: generators) {
      os << it.first << std::endl;
    }
    return os.str();
  }
  
  PLGOSimulator::PLGOSimulator() {
    generators["sphere"]=TrajectoryGeneratorPtr(new TrajectoryGeneratorSphere(poses));
    generators["sphere_uniform"]=TrajectoryGeneratorPtr(new TrajectoryGeneratorSphereUniform(poses));
    generators["torus"]=TrajectoryGeneratorPtr(new TrajectoryGeneratorTorus(poses));
    generators["manhattan"]=TrajectoryGeneratorPtr(new TrajectoryGeneratorRandomManhattan(poses));
    landmark_sensor_offset.setIdentity();
    landmark_sensor_offset.translation()=Vector3f(0,0,0.5);
    landmark_sensor_offset.linear() <<
      0, 0, 1,
      -1, 0, 0,
      0, -1, 0;
  }

  bool PLGOSimulator::generateTrajectory(const std::string& name, const float* argv) {
    StringGenMap::iterator gen = generators.find(name);
    if (gen==generators.end()) {
      std::cerr << "unknown motion type [" << name << "], aborting" << std::endl;
      return false;
    }
    if (argv) {
      gen->second->parseArgs(argv);
    }
    std::cerr << "creating path... " << std::endl;
    gen->second->compute();
    std::cerr << "done [" << poses.size() << "]" << std::endl;
    return true;
  }


  void PLGOSimulator::findMatches(IntVector& indices,
                                  const KDTree3D& search_tree,
                                  const Isometry3f& query,
                                  const float range,
                                  const float fov) {
    Vector3fVector answer_points;
    IntVector answer_indices;
    Isometry3f iq=query.inverse();
    float range2=range*range;
    indices.clear();
    float cos_val=cos(fov);
    if (fov==0)
      cos_val=-2;
    search_tree.findNeighbors(answer_points, answer_indices, query.translation(), range, KDTreeSearchType::Complete);
    for (size_t i=0; i<answer_points.size(); ++i) {
      Vector3f local_point=iq*answer_points[i];
      if (local_point.squaredNorm()>range2) {
        continue;
      }
      if (local_point.squaredNorm()>1e-3) {
        local_point.normalize();
        if (local_point.z()<cos_val) {
          continue;
        }
      }
      indices.push_back(answer_indices[i]);
    }
  }
 

  void PLGOSimulator::generateLandmarks(const float max_range,
                                        const float max_fov, 
                                        const int landmarks_per_pose) {
    landmarks.reserve(poses.size()*landmarks_per_pose);
    float fov=max_fov;
    if (fov==0)
      fov=2*M_PI;
    std::cerr << "generating landmarks... [fov: " << fov << " max_range:" << max_range << "] ";
    for (const auto& pose: poses) {
      for (int np=0; np<landmarks_per_pose; ++np) {
        
        float a1=(drand48()-0.5)*fov;
        float a2=(drand48()-0.5)*fov;
        float range=drand48()*max_range;
        Matrix3f Rz=geometry3d::rotationZ<float>(a1);
        Matrix3f Ry=geometry3d::rotationY<float>(a2);
        Vector3f p_sensor=Rz*Ry*Vector3f(0, 0, range);
        Vector3f p_local=landmark_sensor_offset*p_sensor;
        Vector3f p_global=pose*p_local;
        landmarks.emplace_back(p_global);
      }
    }
    std::cerr << " done (" << landmarks.size() << ")" << std::endl;;
    
  }

  void PLGOSimulator::pruneLandmarks(float distance) {
    if (distance>=0)
      return;
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
      landmarks.emplace_back(src[i]);
      search_tree.findNeighbors(answer_points, answer_indices, src[i], distance, KDTreeSearchType::Complete);
      for(size_t t=0; t<answer_indices.size(); ++t) {
        int idx=answer_indices[t];
        taints[idx]=true;
      }
    }
    std::cerr << "done (" <<landmarks.size() << "/" << src.size() << ")" << std::endl;
  }

  void PLGOSimulator::makePosePoseAssications(const float range,
                                              const int min_steps) {
    std::cerr << "constructing path tree... ";
    Vector3fVector pose_points(poses.size());
    for (size_t i=0; i<poses.size(); ++i) {
      pose_points[i]=poses[i].translation();
    }
    std::cerr << " done" << std::endl;

    std::cerr << "generating pose_pose associations... ";
    KDTree3D pose_tree(pose_points,range);
    int total_pose_matches=0;
    for (size_t p_idx=0; p_idx<poses.size(); ++ p_idx) {
      IntVector pose_indices;
      /*int num_matches = */
      findMatches(pose_indices,
                  pose_tree,
                  poses[p_idx],
                  range, 0);
      //std::cerr << p_idx << "(" << num_matches << " ) : [ ";
      total_pose_matches+=pose_indices.size();
      for (size_t k=0; k<pose_indices.size(); ++k) {
        int other_idx=pose_indices[k];
        // only observe past poses and ensure their distance on the trajectory is big enough
        if ( ((int)p_idx-other_idx) > min_steps )
          pose_associations.push_back(IntPair(p_idx, other_idx));
      }
      //std::cerr << "]" << endl;
    }
    std::cerr << "done (" << pose_associations.size()<< "/" << total_pose_matches <<")" << std::endl;
  }

  void PLGOSimulator::makePoseLandmarkAssociations(const float range,
                                                   const float fov,
                                                   const int min_seen) {
    std::cerr << "generating landmark associations... ";
    KDTree3D landmark_tree(landmarks, range);
    IntVector landmarks_num_seen(landmarks.size(), 0);
    for (size_t p_idx=0; p_idx<poses.size(); ++ p_idx) {
      IntVector landmark_indices;
      /*int num_matches = */
      findMatches(landmark_indices,
                  landmark_tree,
                  poses[p_idx]*landmark_sensor_offset,
                  range,
                  fov);
      //std::cerr << p_idx << "(" << num_matches << " ) : [ ";
      for (size_t k=0; k<landmark_indices.size(); ++k) {
        int l_idx=landmark_indices[k];
        //std::cerr << l_idx << " ";
        ++landmarks_num_seen[l_idx];
        landmark_associations.push_back(IntPair(p_idx, l_idx));
      }
      //std::cerr << "]" << endl;
    }
    std::cerr << "done (" << landmark_associations.size() <<")" << std::endl;

    if (min_seen > 0) {
      std::cerr << "pruning landmarks not seen enough ... ";
      size_t k=0;
      for (size_t a=0; a<landmark_associations.size(); ++a) {
        int l_idx=landmark_associations[a].second;
        if (landmarks_num_seen[l_idx]>=min_seen) {
          landmark_associations[k]=landmark_associations[a];
          ++k;
        }
      }
      std::cerr << "done (" << k << "/" << landmark_associations.size() << ")" << std::endl;
      landmark_associations.resize(k);
    }
  }

}
