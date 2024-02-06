// #include "point_cloud_proc.h"
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_pcl/point_cloud.h>
#include <srrg_pcl/point_projector.h>
#include <srrg_pcl/point_unprojector.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_pcl/point_normal_curvature.h>
#include <srrg_pcl/instances.h>
#include <srrg_messages_ros/instances.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_config/pipeline_runner.h>
#include <bits/stdc++.h>
#include <srrg_data_structures/matrix.h>
#include <srrg_pcl/point_types.h>
#include <unistd.h>

#include <srrg_image/image.h>
#include <srrg_system_utils/shell_colors.h>
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

  // ArgumentString config_filename(&cmd, "c", "config", "configiguration file", "app.conf");
  // ArgumentString source_name(&cmd, "sn", "source-name", "source name in the config", "ros_source");
  // ArgumentFlag generate_config(
  //   &cmd, "j", "generate-config", "if it's set, generates default configuration", true);
  // ArgumentString dataset_filename(&cmd, "d", "dataset", "dataset filename either boss or bag", "");
  // ArgumentDouble lidar_projector_min_depth(
  //   &cmd, "min-d", "min-depth", "lidar projector min depth [m]", 0.3);
  // ArgumentDouble lidar_projector_max_depth(
  //   &cmd, "max-d", "max-depth", "lidar projector max depth [m]", 100.0);
  // ArgumentDouble lidar_projector_horizontal_start_angle(
  //   &cmd, "hs", "horizonal-start", "horizontal starting angle for lidar projection [rad]", M_PI);
  // ArgumentDouble lidar_projector_horizontal_end_angle(
  //   &cmd, "he", "horizonal-end", "horizontal ending angle for projection [rad]", -M_PI);
  // ArgumentInt lidar_projector_num_columns(
  //   &cmd, "col", "column", "lidar projected image column", 1024);
  // ArgumentFlag dump_lidar_depth_images(
  //   &cmd, "dd", "dump-depth", "if it's set, dumps also the lidar depth image", false);
  // cmd.parse();

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

  // while (1){
  //   // if input source if completed
  //   if (sink->isFlushed()){
  //     break;
  //   }
  // }
  //   // retrieve the factor graph from the manager
  // auto graph_manager = manager.getByName<md_slam::MDGraphManager>("graph_manager");
  // if (!graph_manager) {
  //   std::cerr << std::string(environ[0]) +
  //                  "|ERROR, cannot find graph_manager, maybe wrong configuration path!"
  //             << std::endl;
  // }

  // if (generate_config.isSet()) {
  //   LOG << "generating default config\n";
  //   generateConfig(config_filename.value());
  //   LOG << "done!\n";
  //   return 0;
  // }

  // if (!dataset_filename.isSet() || !output_lidar_directory.isSet() ||
  //     !output_camera_directory.isSet()) {
  //   std::cerr << cmd.options();
  //   throw std::runtime_error(
  //     exe_name + "|ERROR, invalid parameters, please specify output directories and dataset");
  // }

  // if (!checkFile(config_filename.value())) {
  //   throw std::runtime_error(exe_name + "|ERROR, invalid configuration file [ " +
  //                            config_filename.value() + " ]");
  // }

  // ia read configuration for the source
  // ConfigurableManager manager;
  // manager.read(config_filename.value());

  // MessageFileSourceBasePtr source   = manager.getByName<MessageFileSourceBase>(source_name.value());
  // MessageSynchronizedSourcePtr sync = manager.getByName<MessageSynchronizedSource>("sync");
  // if (!source || !sync) {
  //   throw std::runtime_error(exe_name + "|cannot get sources");
  // }
  // source->open(dataset_filename.value());

  // ia setup projector
  // LidarProjectorType projector;
  // projector.param_num_columns.setValue(lidar_projector_num_columns.value());
  // projector.param_range_min.setValue(lidar_projector_min_depth.value());
  // projector.param_range_max.setValue(lidar_projector_max_depth.value());
  // projector.param_horizontal_start_angle.setValue(lidar_projector_horizontal_start_angle.value());
  // projector.param_horizontal_end_angle.setValue(lidar_projector_horizontal_end_angle.value());

  // ia read and process the thing
  // BaseSensorMessagePtr msg;
  // while ((msg = sync->getMessage())) {
  //   // ia we need a pack
  //   MessagePackPtr msg_pack = std::dynamic_pointer_cast<MessagePack>(msg);
  //   if (!msg_pack || msg_pack->messages.size() < 2) {
  //     throw std::runtime_error(exe_name + "|dataset should contain at least camera and lidar data");
  //   }

    // for (size_t i = 0; i < msg_pack->messages.size(); ++i) {
    //   PointCloud2MessagePtr lidar_msg =
    //     std::dynamic_pointer_cast<PointCloud2Message>(msg_pack->messages.at(i));
    //   ImageMessagePtr camera_msg =
    //     std::dynamic_pointer_cast<ImageMessage>(msg_pack->messages.at(i));
    //   if (lidar_msg) {
    //     dumpLidar(
    //       projector, lidar_msg, output_lidar_directory.value(), dump_lidar_depth_images.isSet());
    //   } else if (camera_msg) {
    //     dumpCamera(camera_msg, output_camera_directory.value());
    //   } else {
    //     continue;
    //   }
    // }

    // std::cerr << "\rprocessing message [ " << msg_number++ << " ]";
    // cv::imshow("lidar_depth", cv_depth_image);
    // cv::imshow("lidar_intensity", cv_intensity_image);
    // cv::imshow("camera_grayscale", cv_camera_image);
    // cv::waitKey(1);
  // }
  manager.erase(runner);
  manager.erase(source);
  runner.reset();
  source.reset();
  return 0;
}