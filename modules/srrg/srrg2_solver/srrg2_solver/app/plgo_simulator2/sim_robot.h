#include "trajectory_generator.h"
#include "sim_sensors.h"
#include <srrg_config/property_configurable_vector.h>
#include <srrg_config/preemptible.h>

namespace srrg2_solver {
  using namespace srrg2_core;
  class SimRobot: public Configurable, public Preemptible {
  public:
    SimRobot();
    PARAM_VECTOR(PropertyConfigurableVector_<SimSensorBase>,
                 sensors,
                 "sensors mounted on the robot",
                 0);
    PARAM(PropertyConfigurable_<TrajectoryGeneratorBase>,
          trajectory_generator,
          "type of trajectory traveled by our robot",
          TrajectoryGeneratorPtr(new TrajectoryGeneratorRandomManhattan),
          0);
    PARAM(PropertyConfigurable_<SimSensorMode3DPoses>,
          path_sensor_mode,
          "world slice containing the path",
          SimSensorMode3DPosesPtr(new SimSensorMode3DPoses),
          0);

    void saveGraph(const std::string& filename);
    void setup();
    void reset() override;
    void compute() override;

    bool cmdSaveGraph(std::string& response, const std::string& filename);
    bool cmdSetup(std::string& response);
    bool cmdCompute(std::string& response);
    
    FactorGraphPtr graph; // where we assemble our stuff
    //protected:
    inline Isometry3fVector& path() {return param_trajectory_generator->path();}
  };
}
