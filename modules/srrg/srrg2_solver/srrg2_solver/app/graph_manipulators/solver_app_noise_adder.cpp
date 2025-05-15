#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <random>
#include <Eigen/Cholesky>
#include <fstream>
#include <iomanip>
#include <srrg_system_utils/parse_command_line.h>

#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_point_offset_error_factor.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_omni_ba_error_factor.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_pinhole_ba_error_factor.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_rectified_stereo_error_factor.h"

#include "srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_point2_ad.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_pose_point_error_factor.h"

#include "factor_noise_adder.h"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;


const std::string exe_name = "solver_app_noise_adder";
#define LOG std::cerr << exe_name << "|"

using StringNoiseAdderMap=std::map<std::string, FactorNoiseAdderPtr >;

default_random_engine rnd_gen;
std::normal_distribution<float> norm_gen;

StringNoiseAdderMap noise_adders;
void initNoiseAdders() {
  noise_adders.insert(std::make_pair("SE3PosePointOffsetErrorFactor", FactorNoiseAdderPtr(new FactorNoiseAdderEuclidean_<SE3PosePointOffsetErrorFactor>(rnd_gen, norm_gen))));
  
  noise_adders.insert(std::make_pair("SE3PosePoseGeodesicErrorFactor", FactorNoiseAdderPtr(new FactorNoiseAdderSE3Quat_<SE3PosePoseGeodesicErrorFactor>(rnd_gen, norm_gen))));

  noise_adders.insert(std::make_pair("SE2PosePointErrorFactor", FactorNoiseAdderPtr(new FactorNoiseAdderEuclidean_<SE2PosePointErrorFactor>(rnd_gen, norm_gen))));
  
  noise_adders.insert(std::make_pair("SE2PosePoseGeodesicErrorFactor", FactorNoiseAdderPtr(new FactorNoiseAdderSE2_<SE2PosePoseGeodesicErrorFactor>(rnd_gen, norm_gen))));

  noise_adders.insert(std::make_pair("Sim3PosePoseErrorFactorAD", FactorNoiseAdderPtr(new FactorNoiseAdderSim3_<Sim3PosePoseErrorFactorAD>(rnd_gen, norm_gen))));

  noise_adders.insert(std::make_pair("SE3PosePointOmniBAErrorFactor", FactorNoiseAdderPtr(new FactorNoiseAdderNormalize_<SE3PosePointOmniBAErrorFactor>(rnd_gen, norm_gen))));

  noise_adders.insert(std::make_pair("SE3PosePointPinholeBAErrorFactor", FactorNoiseAdderPtr(new FactorNoiseAdderEuclidean_<SE3PosePointPinholeBAErrorFactor>(rnd_gen, norm_gen))));

  noise_adders.insert(std::make_pair("SE3PosePointRectifiedStereoErrorFactor", FactorNoiseAdderPtr(new FactorNoiseAdderEuclidean_<SE3PosePointRectifiedStereoErrorFactor>(rnd_gen, norm_gen))));

}

std::string listNoiseAdders() {
  ostringstream os;
  for (const auto& it: noise_adders) {
    os << it.first << endl;
  }
  return os.str();
}

static const char* banner[] = {
  "adds noise to an (ideal) graph, based on the value of the information matrices",
  "usage: solver_app_noise_adder -i <input> -o <output>",
  0
};

// ia THE PROGRAM
int main(int argc, char** argv) {
  initNoiseAdders();
  ParseCommandLine cmd_line(argv, banner);
  ArgumentInt seed                    (&cmd_line, "s",    "seed",           "starting_seed", 0);
  ArgumentString output_file          (&cmd_line, "o",    "output-file",           "file where to save the output", "");
  ArgumentString input_file          (&cmd_line, "i",    "input-file",             "file where to read the input ", "");
  ArgumentFlag list_adders          (&cmd_line, "l",    "list-adders",             " prints a list of noise adders registered");
  cmd_line.parse();

  if (seed.isSet()){
    rnd_gen.seed(seed.value());
  }
  if (list_adders.isSet()) {
    cerr << "Adders: " << endl;
    cerr << listNoiseAdders() << endl;
    return 0;
  }
  if (! input_file.isSet()) {
    cerr << "no input file provided, aborting" << endl;
    return 0;
  }
  std::cerr << "loding file: [" << input_file.value() << "]... ";
  FactorGraphPtr graph = FactorGraph::read(input_file.value());
  std::cerr << "done, factors:" << graph->factors().size() << " vars: " << graph->variables().size() << std::endl;

  std::cerr << "adding noise...";
  for (auto it = graph->factors().begin();
       it != graph->factors().end(); ++it) {
    FactorBase* factor=it.value();

    auto adder_it=noise_adders.find(factor->className());
    if (adder_it==noise_adders.end()) {
      cerr << "unknown adder [" << factor->className() << "] skupping" << endl;
      continue;
    }
    FactorNoiseAdderPtr adder=adder_it->second;

    adder->addNoise(factor);
  }
  
  std::cerr << "done" << std::endl;

  if (! output_file.isSet()) {
    cerr << "no output file provided, skipping output" << endl;
    return 0;
  }

  std::cerr << "writing output to file [ " << output_file.value() << "]... ";
  graph->write(output_file.value());
  cerr << " done" << endl;
  return 0;
}
