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

#include "srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_point2_ad.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_2d/se2_pose_point_error_factor.h"

#include "srrg_solver/variables_and_factors/types_projective/variable_sim3_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/sim3_pose_pose_error_factor_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_omni_ba_error_factor.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_pinhole_ba_error_factor.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_rectified_stereo_error_factor.h"

#include <Eigen/Cholesky>

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;


const std::string exe_name = "solver_app_info_overwrite";
#define LOG std::cerr << exe_name << "|"


struct FactorInfoOverwriteBase {
  virtual void setInformationMatrix(const std::vector<float>& upper_triangular_block) = 0;
  virtual void getInformationMatrix(std::vector<float>& upper_triangular_block) = 0;
  virtual bool assign(FactorBase* factor) = 0;
  virtual ~FactorInfoOverwriteBase() {};
};

template <typename FactorType_>
struct FactorInfoOverwrite_: public FactorInfoOverwriteBase {
  using FactorType=FactorType_;
  static constexpr int ErrorDim=FactorType_::ErrorDim;
  using InformationMatrixType= typename FactorType_::InformationMatrixType;
  InformationMatrixType info=InformationMatrixType::Identity();
  std::string factor_class_name;
  

  void getInformationMatrix(std::vector<float>& upper_triangular_block) {
    upper_triangular_block.clear();
    for (int r=0; r<info.rows(); ++r)
      for (int c=r; c<info.cols(); ++c)
        upper_triangular_block.push_back(info(r,c));
  }

  void setInformationMatrix(const std::vector<float>& upper_triangular_block) {
    size_t expected_elements=(ErrorDim*(ErrorDim+1))/2;
    if (upper_triangular_block.size()!=expected_elements) {
      throw std::runtime_error("wrong number of elements in the information matrix, current: "
                               + ::to_string(upper_triangular_block.size()) +
                               "expected: " + std::to_string(expected_elements));
    }
    size_t k=0;
    for (int r=0; r<info.rows(); ++r)
      for (int c=r; c<info.cols(); ++c, ++k)
        info(r,c)=info(c,r)=upper_triangular_block[k];
    Eigen::LLT<InformationMatrixType> llt;
    llt.compute(info);
    if (llt.info()!=Eigen::Success) {
      throw std::runtime_error("information matrix is not positive definite");
    }
  }
  
  bool assign(FactorBase* factor_) {
    FactorType* factor=dynamic_cast<FactorType*>(factor_);
    if (! factor) {
      return false;
    }
    factor->setInformationMatrix(info);
    return true;
  }
};

using FactorInfoOverwritePtr = std::shared_ptr<FactorInfoOverwriteBase>;
using StringOverwriteMap=std::map<std::string, FactorInfoOverwritePtr>;

StringOverwriteMap overwriters;
void initOverwriters() {
  overwriters["SE3PosePointOffsetErrorFactor"] =
    FactorInfoOverwritePtr(new FactorInfoOverwrite_<SE3PosePointOffsetErrorFactor>);
  overwriters["SE3PosePointOmniBAErrorFactor"] =
    FactorInfoOverwritePtr(new FactorInfoOverwrite_<SE3PosePointOmniBAErrorFactor>);
  overwriters["SE3PosePointRectifiedStereoErrorFactor"] =
    FactorInfoOverwritePtr(new FactorInfoOverwrite_<SE3PosePointRectifiedStereoErrorFactor>);
  overwriters["SE3PosePointPinholeBAErrorFactor"] =
    FactorInfoOverwritePtr(new FactorInfoOverwrite_<SE3PosePointPinholeBAErrorFactor>);
  overwriters["SE3PosePoseGeodesicErrorFactor"] =
                        FactorInfoOverwritePtr
                        (new FactorInfoOverwrite_<SE3PosePoseGeodesicErrorFactor>);
  overwriters["SE2PosePointErrorFactor"] =
                        FactorInfoOverwritePtr
                        (new FactorInfoOverwrite_<SE2PosePointErrorFactor>);
  overwriters["SE2PosePoseGeodesicErrorFactor"] =
                        FactorInfoOverwritePtr
                        (new FactorInfoOverwrite_<SE2PosePoseGeodesicErrorFactor>);
  overwriters["Sim3PosePoseErrorFactorAD"] =
                        FactorInfoOverwritePtr
                        (new FactorInfoOverwrite_<Sim3PosePoseErrorFactorAD>);
}

std::string listOverwriters() {
  ostringstream os;
  for (const auto& it: overwriters) {
    os << it.first << endl;
  }
  return os.str();
}

static const char* banner[] = {
  "overwrites the information matrix of the factors in a graph, for noise generation and tuning",
  "usage: solver_app_info_overwrite -c <config file> -i <input> -o <output>",
  " the config file is filw where each row has the form ",
  " <factortype> <upper triangular block of information matrix",
  0
};

void readConfig(StringOverwriteMap& active, const std::string filename) {
  active.clear();
  ifstream is;
  is.open(filename);
  int line_num=0;
  while(is.good()) {
    char buf [1024];
    is.getline(buf, 1024);
    istringstream ls(buf);
    ++line_num;
    std::string token;
    ls >> token;
    auto ov_it=overwriters.find(token);
    if (ov_it==overwriters.end()) {
      continue;
    }
    std::vector<float> values;
    while (ls) {
      float v;
      ls >> v;
      if (ls)
        values.push_back(v);
    }
    FactorInfoOverwritePtr ovt=ov_it->second;
    ovt->setInformationMatrix(values);
    active[token] = ovt;
  }
}

void writeConfig(ostream& os, StringOverwriteMap& ovt) {
  for(auto& it: ovt) {
    os << it.first << " ";
    vector<float> values;
    it.second->getInformationMatrix(values);
    for (auto v: values) {
      os << v << " ";
    }
    os << endl;
  }
}

// ia THE PROGRAM
int main(int argc, char** argv) {
  initOverwriters();
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString output_file          (&cmd_line, "o",    "output-file",           "file where to save the output", "");
  ArgumentString input_file          (&cmd_line, "i",    "input-file",             "file where to read the input ", "");
  ArgumentString config_file          (&cmd_line, "c",    "config-file",           "file with the specs of the info to overwrite", "");
  ArgumentString config_gen          (&cmd_line, "j",    "config-gen",           "file with the specs of the info to overwrite", "");
  ArgumentFlag list_overwriters       (&cmd_line, "l",    "list-overwriters",       " prints a list of noise adders registered");
  cmd_line.parse();


  if (config_gen.isSet()) {
    cerr << "generating config file [ " << config_gen.value() << " ]... " << std::endl;
    ofstream os(config_gen.value());
    writeConfig(os, overwriters);
    os.close();
    cerr << "done" << endl;
    return 0;
  }
  
  if (list_overwriters.isSet()) {
    cerr << "Adders: " << endl;
    cerr << listOverwriters() << endl;
    return 0;
  }

  if (! config_file.isSet()) {
    cerr << "please provide a config file" << std::endl;
    return 0;
  }

  StringOverwriteMap active_overwriters;
  readConfig(active_overwriters, config_file.value());
  std::cerr << "read config: [ "<<  config_file.value() << " ]" << std::endl;

  cerr << "read " << active_overwriters.size() << " rules" << endl;
  writeConfig(cerr, active_overwriters);
  cerr << " done " << endl;

  
  if (! input_file.isSet()) {
    cerr << "no input file provided, aborting" << endl;
    return 0;
  }
  std::cerr << "loding file: [" << input_file.value() << "]... ";
  FactorGraphPtr graph = FactorGraph::read(input_file.value());
  std::cerr << "done, factors:" << graph->factors().size() << " vars: " << graph->variables().size() << std::endl;

  std::cerr << "overwriting ...";
  for (auto it = graph->factors().begin();
       it != graph->factors().end(); ++it) {
    FactorBase* factor=it.value();
    auto ov_it=active_overwriters.find(factor->className());
    if (ov_it==active_overwriters.end()) {
      continue;
    }
    ov_it->second->assign(factor);
  }
  
  std::cerr << "done" << std::endl;

  if (! output_file.isSet()) {
    cerr << "no output file provided, skipping output" << endl;
    return 0;
  }
  
  std::cerr << "writing output to file [" << output_file.value() << "]... ";
  graph->write(output_file.value());
  std::cerr << "done, factors:" << graph->factors().size() << " vars: " << graph->variables().size() << std::endl;
  return 0;
}
