#include <iostream>
#include <fstream>
#include <sstream>
#include "tricycle_odom_sensor2d_error_factor_ad.h"
#include "variable_point4_ad.h"
#include "srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h"
#include "srrg_solver/solver_core/factor_graph.h"

using namespace std;

using namespace srrg2_core;
using namespace srrg2_solver;
extern char** environ;

int main(int argc, char** argv) {
  if (argc<3) {
    cerr << "usage " << environ[1] << "<input_ticks_file> <output_graph>" << endl;
    return -1;
  }
  ifstream is(argv[1]);
  if (! is.good()) {
    cerr << "no input" << endl;
    return -1;
  }
  
  FactorGraph graph;
  VariableSE2RightAD* sensor_pose=new VariableSE2RightAD;

  sensor_pose->setGraphId(0);
  sensor_pose->setEstimate(geometry2d::v2t(Vector3f(0,0,M_PI)));
  VariablePoint4AD* parameters =  new VariablePoint4AD();
  
  const float k_traction = 0.01;
  const float k_steer = 1;
  const float k_axis  = 1;
  const float k_steer_offset  = 0;
  
  Eigen::Vector4f params_estimate;
  params_estimate << k_steer, k_traction, k_axis, k_steer_offset;
  parameters->setGraphId(1);
  parameters->setEstimate(params_estimate);

  
  graph.addVariable(VariableBasePtr(sensor_pose));
  graph.addVariable(VariableBasePtr(parameters));

  while(is) {
    char buf[1024];
    is.getline(buf,1024);
    istringstream ls(buf);
    float steer;
    int traction;
    Vector3f v;
    ls >> traction >> steer >> v.x() >> v.y() >> v.z();
    TricycleOdomSensor2DErrorFactorAD* factor = new TricycleOdomSensor2DErrorFactorAD;
    factor->setVariableId(0, 1);
    factor->setVariableId(1, 0);
    factor->_tick_steer = steer;
    factor->_dtick_traction = traction;
    factor->setMeasurement(geometry2d::v2t(v));
    graph.addFactor(FactorBasePtr(factor));
  }
  graph.write(argv[2]);
  cerr << "num vars: " << graph.variables().size();
  cerr << "num_factors: " << graph.factors().size();
  cerr << "writing shit in [" << argv[2] << "]" << endl;

}
