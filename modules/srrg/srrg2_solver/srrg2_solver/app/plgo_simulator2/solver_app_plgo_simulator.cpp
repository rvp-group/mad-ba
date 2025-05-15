#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <random>
#include <fstream>
#include <iomanip>
#include <srrg_system_utils/parse_command_line.h>

#include "srrg_solver/solver_core/instances.h"
#include "srrg_solver/solver_core/factor_graph.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_point_offset_error_factor.h"
#include "srrg_boss/serializer.h"

#include "plgo_simulator.h"
#include "graph_generator.h"

using namespace srrg2_core;
using namespace srrg2_solver;
using namespace std;



const std::string exe_name = "solver_app_plgo_simulator";
#define LOG std::cerr << exe_name << "|"

PLGOSimulator sim;
std::map<std::string, GraphGeneratorPtr> graph_generators;

std::string generator_list;

void initGraphGenerators() {
  graph_generators.insert(make_pair("se3", GraphGeneratorPtr(new GraphGeneratorSE3(sim))));
  graph_generators.insert(make_pair("sim3", GraphGeneratorPtr(new GraphGeneratorSim3(sim))));

  auto omni_ba=GraphGeneratorPtr(new GraphGeneratorSE3OmniBA(sim));
  graph_generators.insert(make_pair("se3_omnicam", omni_ba));
  graph_generators.insert(make_pair("se3_ba", omni_ba));
                          
  graph_generators.insert(make_pair("se3_pinhole", GraphGeneratorPtr(new GraphGeneratorSE3PinholeBA(sim))));
  graph_generators.insert(make_pair("se3_rectified_stereo", GraphGeneratorPtr(new GraphGeneratorSE3RectifiedStereo(sim))));
  
  ostringstream os;
  for (const auto& it: graph_generators){
    os <<it.first << endl;
  }
  generator_list=os.str();
}

static const char* banner[] = {
  "generates a simulated graph",
  "usage: solver_plgo_simulator [options]",
  0
};


// ia THE PROGRAM
int main(int argc, char** argv) {
  initGraphGenerators();
  BOSS_REGISTER_CLASS(SimRecord);
  ParseCommandLine cmd_line(argv, banner);
  ArgumentFlag list_trajectory_generators  (&cmd_line, "t",  "list-trj-gen",         "lists the trajectory generators");
  ArgumentFlag list_graph_generators  (&cmd_line, "g",  "list-graph-gen",        "lists the graph generators");
  ArgumentString generator_type       (&cmd_line, "gt",  "generator-type",         "type of graph (se3, sim3, ...)", "se3");
  ArgumentString motion_type          (&cmd_line, "mt",  "motion-type",               "type of motion: (sphere, torus, ...)", "sphere");
  ArgumentFloatVector_<6> motion_args (&cmd_line, "ma", "motion-args",           "free arguments for the generator", {0.f, 0.f, 0.f, 0.f, 0.f,0.f});
  ArgumentFlag sensor_at_origin       (&cmd_line, "zs",  "sensor-at-origin",          "puts the sensor at the origin of the robot");
  ArgumentFlag make_point_landmarks   (&cmd_line, "ml",  "make-point-landmarks",      "creates landmarks");
  ArgumentFlag make_odometry          (&cmd_line, "mo",  "make-odometry",             "adds the odometry sensor");
  ArgumentFlag make_pose_sensor       (&cmd_line, "mp",  "make-pose-sensor",          "adds the pose-pose sensor");
  ArgumentFloat landmark_sensor_range (&cmd_line, "lsr", "landmark-sensor-range",     "range at which a landmark is visible [m]", 5.f);
  ArgumentFloat pose_sensor_range     (&cmd_line, "psr", "pose-sensor-range",         "range at which another pose is visible [m]", 5.f);
  ArgumentInt pose_sensor_min_steps     (&cmd_line, "pss", "pose-sensor-min-steps",   "min steps between two poses on the path", 5);
  ArgumentFloat landmark_sensor_fov   (&cmd_line, "lsf", "landmark-sensor-fov",       "fov at which  landmark is visible [rad]", 1.f);
  ArgumentInt   landmarks_per_pose    (&cmd_line, "lpp", "landmarks-per-pose",        "landmarks to deploy for each pose", 10);
  ArgumentInt   landmarks_min_seen    (&cmd_line, "lms", "landmarks-min-seen",        "min times a landmark can be seen", 5);
  ArgumentFloat   landmarks_min_dist    (&cmd_line, "lmd", "min-landmarks-distance",  "min distance between deployed landmarks [m]", 0.5f);
  ArgumentString output_file          (&cmd_line, "o",    "output-file",               "file where to save the output", "");
  
  cmd_line.parse();

  if (list_graph_generators.isSet()) {
    cerr << "available graph generators: " << endl;
    cerr << generator_list << endl;
    return 0;
  }

  if (list_trajectory_generators.isSet()) {
    cerr << "available trajectory generators: " << endl;
    cerr << sim.listTrajectoryGenerators() << endl;
    return 0;
  }

  // gets the motion args if available
  const float* margs=0;
  if (motion_args.isSet()) {
    margs=motion_args.value();
  }
  
  if (sensor_at_origin.isSet()) {
    sim.landmark_sensor_offset.setIdentity();
  }
  
  if (! sim.generateTrajectory(motion_type.value(), margs) ) {
    return -1;
  }

  auto gen_it=graph_generators.find(generator_type.value());
  if (gen_it==graph_generators.end()) {
    std::cerr << "unknown graph generator [" << generator_type.value() << "]" << endl;
    return -1;
  }
  
  if (make_pose_sensor.isSet()) {
    sim.makePosePoseAssications(pose_sensor_range.value(),
                                pose_sensor_min_steps.value());
  }

  if (make_point_landmarks.isSet()) {
    sim.generateLandmarks(landmark_sensor_range.value(),
                          landmark_sensor_fov.value(),
                          landmarks_per_pose.value());
    sim.pruneLandmarks(landmarks_min_dist.value());
    sim.makePoseLandmarkAssociations(landmark_sensor_range.value(),
                                     landmark_sensor_fov.value(),
                                     landmarks_min_seen.value());
  }


  GraphGeneratorPtr gen=gen_it->second;
  gen->has_odom=make_odometry.isSet();
  gen->landmark_min_measurements=3;
  gen->compute();

  if (! output_file.isSet()) {
    std::cerr << "not generating any output" << std::endl;
    return  0;
  }
  Serializer ser;
  std::cerr << "writing output to file[" << output_file.value() <<"]" << endl;;
  ser.setFilePath(output_file.value());
  int var_count=0;
  int fac_count=0;
  for (auto r: gen->records) {
    for (auto f_id: r->factors) {
      FactorBase* f=gen->graph->factor(f_id);
      ser.writeObject(*f);
      ++fac_count;
      cerr << "\r vars: " << var_count << "\tfactors: " << fac_count;
    }
    for (auto v_id: r->vars) {
      VariableBase* v=gen->graph->variable(v_id);
      ser.writeObject(*v);
      ++var_count;
      cerr << "\r vars: " << var_count << "\tfactors: " << fac_count;
    }
  }
  cerr <<  endl;

}
