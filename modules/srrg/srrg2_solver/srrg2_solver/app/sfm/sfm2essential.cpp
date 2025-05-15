#include <srrg_system_utils/parse_command_line.h>
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_omni_ba_error_factor.h"
#include "srrg_solver/solver_incremental/factor_graph_incremental_sorter.h"
#include "srrg_geometry/epipolar.h"
#include "sfm_common.h"
using namespace std;
using namespace srrg2_core;
using namespace srrg2_solver;

const char * banner []= {
  "sfm2essential",
  "matches pairwise frames and constructs the essential graph",
  0
};


int main(int argc, char** argv) {
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString input_file          (&cmd_line, "i",    "input-file",             "file where to read the input ", "");
  ArgumentString output_file          (&cmd_line, "o",    "output-file",           "file where to save the output", "");
  ArgumentInt min_common_landmarks          (&cmd_line, "l",    "min-common-landmarks",           "min-landmarks to try computing essential", 30);
  ArgumentInt ransac_num_rounds          (&cmd_line, "r",    "ransac-num-rounds",           "ransac-rounds", 1000);
  ArgumentFloat ransac_inlier_threshold          (&cmd_line, "t",    "ransac-inlier-threshold",           "error in point", 1e-2);
  ArgumentInt ransac_min_inliers          (&cmd_line, "k",    "ransac-min-inliers",           "min inliers for ransac to be good", 20);
  
  cmd_line.parse();

  if (! input_file.isSet()) {
    cerr << "no input file" << endl;
    return -1;
  }

  ifstream is(input_file.value());
  if (! is.good()) {
    cerr << "invalid input" << endl;
    return -1;
  }
  ofstream os(output_file.value());
  if (! os.good()) {
    cerr << "Can't open output stream" << endl;
    return -1;
  }
  KeyframePtrMap keyframes;
  readKeyframes(keyframes, is);
  cerr << "Essential matching begins" << endl;
  for (auto& last_it: keyframes) {
    auto& last_keyframe=last_it.second;
    for (auto past_it=keyframes.begin(); past_it->first <last_it.first ; ++past_it) {
      auto& past_keyframe=past_it->second;
      std::set<int> common_landmarks;
      std::set_intersection(last_keyframe->observed_landmark_ids.begin(),
                            last_keyframe->observed_landmark_ids.end(),
                            past_keyframe->observed_landmark_ids.begin(),
                            past_keyframe->observed_landmark_ids.end(),
                            std::inserter(common_landmarks, common_landmarks.begin()));
      int num_matches=common_landmarks.size();
      if (num_matches <= min_common_landmarks.value()) {
        cerr << "SKIPPING: " << last_it.first << " " << past_it->first << " " << num_matches << endl;
        continue;
      }
      
      cerr << "MATCHING: " << last_it.first << " " << past_it->first << " " << num_matches << endl;
      std::vector<std::pair<int, int> > correspondences;
      epipolar::Vector3fVector
        past_keyframe_matches(num_matches),
        last_keyframe_matches(num_matches);
      int ip=0;
      int il=0;
      int dest_k=0;
      for (const auto& landmark_id: common_landmarks) {
        while (past_keyframe->measurements[ip].id != landmark_id)
          ++ip;
        while (last_keyframe->measurements[il].id != landmark_id)
          ++il;
        if (past_keyframe->measurements[ip].id != last_keyframe->measurements[il].id) {
          throw std::runtime_error("id mismatch in essential");
        }
        correspondences.push_back(std::make_pair(ip, il));
        past_keyframe_matches[dest_k]=past_keyframe->measurements[ip].measurement;
        last_keyframe_matches[dest_k]=last_keyframe->measurements[il].measurement;
        ++dest_k;
      }
      
      Eigen::Isometry3f delta_iso=past_keyframe->gt_pose.inverse()*last_keyframe->gt_pose;
      Eigen::Isometry3f est_iso;
      std::vector<float> errors;
      cerr <<"Transform [ " << past_keyframe->id << " -> " << last_keyframe->id << "] pairs:" << num_matches ;
      float inliers=epipolar::estimateTransformRANSAC(est_iso,
                                                    errors,
                                                    past_keyframe_matches,
                                                    last_keyframe_matches,
                                                    ransac_inlier_threshold.value(),
                                                    ransac_num_rounds.value(),
                                                    false);
      if (inliers<ransac_min_inliers.value()) {
        cerr << " REJECT, inliers: " << inliers << endl;
        continue;
      }
      os << "MATCH: " << past_keyframe->id << " " << last_keyframe->id << " " << inliers <<  " ";
      os << geometry3d::t2v(est_iso).transpose() << endl;
      double error=0;
      for (size_t k=0; k<correspondences.size(); ++k) {
        if (fabs(errors[k])<ransac_inlier_threshold.value()) {
          error += fabs(errors[k]);
          os << "CORR: " << correspondences[k].first << " " << correspondences[k].second << " " << errors[k] << endl;
        }
      }
      os << "MATCH_ERROR: " << error << endl;
      cerr << " ACCEPT, inliers: " << inliers << endl;
      AngleAxisf rot_err(delta_iso.linear().transpose()*est_iso.linear());
      cerr << "Rot err: " <<  rot_err.angle();
      cerr << " D_trans: " <<  delta_iso.translation().transpose();
      cerr << " E_trans: " <<  est_iso.translation().transpose();
      if (est_iso.translation().squaredNorm()>1e-3) {
        cerr << " translation: " << (delta_iso.translation().array()/est_iso.translation().array()).transpose() << endl;
      } else {
        cerr << " pure rotation" << endl;
      }
      cerr << endl;                             
      if (rot_err.angle()>1)
         exit(0);
    }
  }
}
