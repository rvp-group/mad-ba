#include <srrg_system_utils/parse_command_line.h>
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_omni_ba_error_factor.h"
#include "srrg_solver/solver_incremental/factor_graph_incremental_sorter.h"
#include "sfm_common.h"

using namespace std;
using namespace srrg2_core;
using namespace srrg2_solver;

const char * banner []= {
  "plgo2sfm",
  "generates a dump for sfm from a simulation file",
  0
};

int main(int argc, char** argv) {
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString input_file          (&cmd_line, "i",    "input-file",             "file where to read the input ", "");
  ArgumentString output_file          (&cmd_line, "o",    "output-file",           "file where to save the output", "");
  ArgumentString path_type          (&cmd_line, "t",    "path-type",             "type of the variable used for sorting", "VariableSE3QuaternionRightAD");
  cmd_line.parse();

  if (! input_file.isSet()) {
    cerr << "no input file" << endl;
    return -1;
  }

  
  if (! output_file.isSet()) {
    cerr << "no input file" << endl;
    return -1;
  }
  ofstream os(output_file.value());
  if (! os) {
    cerr << "can't open " << output_file.value() << " for writing" << endl;
  }
  auto graph = FactorGraphPtr(new FactorGraph);
  FactorGraphIncrementalReader reader;
  reader.setGraph(graph);
  reader.setPathType(path_type.value());
  reader.setFilePath(input_file.value());

  std::cerr << "done" << std::endl;
  
  //process the data incrementally
  std::set<VariableBase::Id> new_vars;
  std::set<FactorBase::Id> new_factors;
  Keyframe keyframe;
  while (reader.readEpoch(new_vars, new_factors)) {
    if (new_vars.size()) {
      cerr << "\rEpoch: " << *new_vars.begin() << " " << " v: " <<new_vars.size() << " f:" << new_factors.size() << endl;
      
      VariableSE3QuaternionRight * v_current_keyframe=0;

      keyframe.clear();
      for (auto var_id: new_vars) {
        auto v = graph->variable(var_id);
        VariableSE3QuaternionRight * v_keyframe=dynamic_cast<VariableSE3QuaternionRight*>(v);
        if (v_keyframe) {
          if (v->status()==VariableBase::Active || v->graphId()==0) {
            keyframe.id=var_id;
            keyframe.gt_pose=v_keyframe->estimate();
            v_current_keyframe=v_keyframe;
          }
        }
      }
      for (auto fac_id: new_factors) {
        auto f = graph->factor(fac_id);
        SE3PosePointOmniBAErrorFactor* f_point=dynamic_cast<SE3PosePointOmniBAErrorFactor*>(f);
        if (f_point) {
          if ( f_point->variableId(0) != v_current_keyframe->graphId()) {
            throw std::runtime_error ("bad, bad, bad");
          }
          keyframe.measurements.push_back(LandmarkMeasurement(f_point->variableId(1),
                                                              f_point->measurement()));
        }
      }
      if (! keyframe.empty()) {
        keyframe.finalize();
        os << keyframe;
      }
    }
  }
}
