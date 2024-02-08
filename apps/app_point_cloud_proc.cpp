// #include "point_cloud_proc.h"
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_pcl/point_cloud.h>
#include <srrg_pcl/point_projector.h>
#include <srrg_pcl/point_unprojector.h>
#include <srrg_pcl/point_normal_curvature.h>
#include <srrg_pcl/instances.h>
#include <srrg_messages_ros/instances.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_config/pipeline_runner.h>
#include <bits/stdc++.h>
#include <srrg_data_structures/matrix.h>
#include <srrg_pcl/point_types.h>
#include <unistd.h>

#include <srrg_image/image.h>
#include <srrg_system_utils/system_utils.h>

#include <iostream>
#include <vector>

#include <srrg_config/configurable_manager.h>
#include <srrg_config/pipeline_runner.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_sorted_sink.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <structure_refinement/point_cloud_proc.h>


using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_lidar3d_utils;
using namespace srrg2_core;
using namespace srrg2_solver;
// using namespace md_slam;

ConfigurableManager manager;
std::shared_ptr<PipelineRunner> runner;


using LidarProjectorType   = PointIntensity3fProjectorOS1_64;
using LidarUnprojectorType = PointIntensity3fUnprojectorOS1_64;
using NormalComputatorType = NormalComputator2DCrossProduct<PointNormal3fMatrixCloud, 1>;

cv::Mat cv_intensity_image, cv_depth_image, cv_camera_image;
size_t msg_number = 0;

void computeThread() {
  runner->compute();
}

int main(int argc, char** argv) {
  // bdc register messages type
  srrgInit(argc, argv, "app_point_cloud_proc");
  // messages_registerTypes();
  // messages_ros_registerTypes();
  ParseCommandLine cmd(argv);
  ArgumentString config_file(&cmd, "c", "config", "config file to load", "");
  ArgumentString bag_file(&cmd, "i", "input", "input bag file to start", "");
  ArgumentFlag verbose(&cmd, "v", "verbose", "if set enables cerr and cout streams", false);
  cmd.parse();

  if (!config_file.isSet()) {
    std::cerr << std::string(environ[0]) + "|ERROR, no config file provided, aborting" << std::endl;
    return -1;
  }

  if (!bag_file.isSet()) {
    std::cerr << std::string(environ[0]) + "|ERROR, no bag (dataset) file provided, aborting"
              << std::endl;
    return -1;
  }

  const std::string dl_folder_path = "/home/kcwian/Workspace/srrg2_source/src/structure_refinement/config";
  const std::string dl_stub_path = crawlForFile("dl.conf", dl_folder_path);
  ConfigurableManager::initFactory(dl_stub_path);
  manager.read(config_file.value());

    // retrieve a runner
  runner = manager.getByName<PipelineRunner>("runner");
  if (!runner) {
    std::cerr << std::string(environ[0]) +
                   "|ERROR, cannot find runner, maybe wrong configuration path!"
              << std::endl;
  }
  std::cout << "AA" << std::endl;
  // retrieve the source from the runner
  auto source = dynamic_pointer_cast<MessageFileSourceBase>(runner->param_source.value());
  if (!source) {
    std::cerr << std::string(environ[0]) +
                   "|ERROR, cannot find source, maybe wrong configuration path!"
              << std::endl;
  }

  auto sink = manager.getByName<MessageSortedSink>("sink");
  if (!sink) {
    std::cerr << std::string(environ[0]) +
                   "|ERROR, cannot find sink, maybe wrong configuration path!"
              << std::endl;
  }

//  retrieve the factor graph from the manager
  auto point_cloud_proc = manager.getByName<structure_refinement::PointCloudProc>("point_cloud_proc");
  if (!point_cloud_proc) {
    std::cerr << std::string(environ[0]) +
                   "|ERROR, cannot find point_cloud_proc, maybe wrong configuration path!"
              << std::endl;
  }

  source->open(bag_file.value());
  // std::thread compute_thread(computeThread);
  runner->compute();

  manager.erase(runner);
  manager.erase(source);
  runner.reset();
  source.reset();
  return 0;
}