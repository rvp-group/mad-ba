#include <srrg_geometry/geometry3d.h>
#include "sfm_common.h"

namespace srrg2_solver {
  using namespace std;
  
  void Keyframe::finalize(){
    std::sort(measurements.begin(), measurements.end(),
              [&](const LandmarkMeasurement& a, const LandmarkMeasurement& b)->bool{
                return a.id < b.id;
              }
              );
    for (const auto& m: measurements)
      observed_landmark_ids.insert(m.id);
  }

  std::ostream& operator << (std::ostream& os , const Keyframe& kf) {
    os << "KF: " << kf.id << " "
       << geometry3d::t2v(kf.gt_pose).transpose() <<  " "
       << geometry3d::t2v(kf.est_pose).transpose() << endl;
    int k=0;
    for (auto& m: kf.measurements) {
      os << "F: " << k << " " << m.id << " " << m.measurement.transpose() << endl;
      ++k;
    }
    return os;
  }

  void Keyframe::clear() {
    measurements.clear();
    observed_landmark_ids.clear();
    id=-1;
    gt_pose.setIdentity();
    est_pose.setIdentity();
  }

  void readKeyframes(KeyframePtrMap& keyframes, istream& is) {
    KeyframePtr current_keyframe;
    while (is.good()) {
      char line_buffer[1024];
      is.getline(line_buffer, 1024);
      istringstream ls(line_buffer);
      std::string token;
      ls >> token;
      if (token=="KF:") {
        if (current_keyframe) {
          current_keyframe->finalize();
          keyframes[current_keyframe->id]=current_keyframe;
        }
        current_keyframe.reset(new Keyframe);
        ls >> current_keyframe->id;
        Vector6f pose;
        for (int i=0; i<6; ++i)
          ls >> pose[i];
        current_keyframe->gt_pose=geometry3d::v2t(pose);

        for (int i=0; i<6; ++i)
          ls >> pose[i];
        current_keyframe->est_pose=geometry3d::v2t(pose);

      } else if (token=="F:"){
        if (! current_keyframe) {
          throw std::runtime_error("sucks");
        }
        LandmarkMeasurement measurement;
        size_t pos;
        ls >> pos;
        ls >> measurement.id;
        ls >> measurement.measurement.x();
        ls >> measurement.measurement.y();
        ls >> measurement.measurement.z();
        if (pos != current_keyframe->measurements.size()) {
          throw std::runtime_error("parse landmark id mismatch");
        }
        current_keyframe->measurements.push_back(measurement);
      } else {
        std::cerr << __PRETTY_FUNCTION__ << "| unknown token [" << token << "]" << endl;
      }
    }
    if (current_keyframe) {
      current_keyframe->finalize();
      keyframes[current_keyframe->id]=current_keyframe;
    }
    cerr << "Read: " << keyframes.size() << " keyframes" << endl;
    cerr << "first: " << keyframes.begin()->first << endl;
    cerr << "last: " << keyframes.rbegin()->first << endl;

  }
  
  void EssentialGraph::read(std::istream& is) {
    EssentialPairwiseEstimatePtr current_match;
    while (is.good()) {
      char line_buffer[1024];
      is.getline(line_buffer, 1024);
      istringstream ls(line_buffer);
      std::string token;
      ls >> token;
      if (token=="MATCH:") {
        current_match.reset(new EssentialPairwiseEstimate);
        ls >> current_match->from >> current_match->to >> current_match->num_inliers;
        Vector6f relative_estimate;
        for (int i=0; i<6; ++i)
          ls >> relative_estimate[i];
        current_match->relative_estimate=geometry3d::v2t(relative_estimate);

        if (! _keyframes.count(current_match->from)) {
          throw std::runtime_error ("malformed file, no keyframe for from");
        }
        if (! _keyframes.count(current_match->to)) {
          throw std::runtime_error ("malformed file, no keyframe for to");
        }
        _estimates.insert(std::make_pair(current_match->from, current_match));
        _estimates.insert(std::make_pair(current_match->to, current_match));
      } else if (token == "CORR:"){
        if (! current_match) {
          throw std::runtime_error ("malformed file, no keyframe before correspondences");
        }
      } else if (token == "MATCH_ERROR:"){
        if (! current_match) {
          throw std::runtime_error ("malformed file, no keyframe before correspondences");
        }
        ls >> current_match->error;
      } else {
        std::cerr << __PRETTY_FUNCTION__ << "| unknown token [" << token << "]" << endl;
        continue;
      }
    }
    std::cerr << "read " << _estimates.size()/2 << " essential matches" << endl;
  }

  void EssentialGraph::computePartitions(std::map<int, Partition>& partitions) {
    partitions.clear();
    std::map<int, int> id_to_partition;
    for (const auto& e: _estimates) {
      // check if a partition exists
      auto part_from_it=id_to_partition.find(e.second->from);
      auto part_to_it=id_to_partition.find(e.second->to);
      if (part_from_it==id_to_partition.end() && part_to_it==id_to_partition.end()) {
        // new partition
        Partition new_part;
        new_part.insert(e.second->from);
        new_part.insert(e.second->to);
        id_to_partition[e.second->to]=e.second->from;
        id_to_partition[e.second->from]=e.second->from;
        partitions[e.second->from]=new_part;
        // cerr << "new part, id: " << e.second->from << " nodes: " << e.second->from << " " << e.second->to << endl;
      } else if (part_from_it!=id_to_partition.end() && part_to_it==id_to_partition.end()) {
        // partition of first existint
        partitions[part_from_it->second].insert(e.second->to);
        id_to_partition[e.second->to]=part_from_it->second;
        // cerr << "add_to, id: " << part_from_it->second << " node: " << e.second->to << endl;
        // cerr << "nodes: [";
        // for (auto i: partitions[part_from_it->second])
        //   cerr << i << " ";
        // cerr << "]"<< endl;

      } else if (part_from_it==id_to_partition.end() && part_to_it!=id_to_partition.end()) {
        // partition of second existint
        partitions[part_to_it->second].insert(e.second->from);
        id_to_partition[e.second->from]=part_to_it->second;

        // cerr << "add_from, id: " << part_to_it->second << " node: " << e.second->from << endl;
        // cerr << "nodes: [";
        // for (auto i: partitions[part_to_it->second])
        //   cerr << i << " ";
        // cerr << "]"<< endl;
      } else {

        // the nodes already belong to the same partition, nothing to do
        if (part_from_it->second==part_to_it->second)
          continue;

        // on the contrary we join the partitions
        auto& dest=partitions[part_from_it->second];
        auto& src=partitions[part_to_it->second];

        // cerr << "merge id: " << part_from_it->second << " " << part_to_it->second << endl;
        // cerr << "nodes from: [";
        // for (auto i: dest)
        //   cerr << i << " ";
        // cerr << "]"<< endl;

        // cerr << "nodes to: [";
        // for (auto i: src)
        //   cerr << i << " ";
        // cerr << "]"<< endl;

        // update the part->id map
        for (auto i: src) {
          id_to_partition[i]=part_from_it->second;
        }
        
        // calculate the intersection
        std::set_union(dest.begin(),
                       dest.end(),
                       src.begin(),
                       src.end(),
                       std::inserter(dest, dest.begin()));

        // cerr << "target: " << part_from_it->second << endl;;
        // cerr << "nodes final: [";
        // for (auto i: dest)
        //   cerr << i << " ";
        // cerr << "]"<< endl;

        partitions.erase(part_to_it->second);
      }
      
    }
  }

}
