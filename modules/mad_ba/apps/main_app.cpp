// #include "point_cloud_proc.h"
#include <bits/stdc++.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_config/pipeline_runner.h>
#include <srrg_data_structures/matrix.h>
#include <srrg_image/image.h>
#include <srrg_messages/message_handlers/message_file_source.h>
#include <srrg_messages/message_handlers/message_pack.h>
#include <srrg_messages/message_handlers/message_sorted_sink.h>
#include <srrg_messages/messages/camera_info_message.h>
#include <srrg_messages/messages/image_message.h>
#include <srrg_messages/messages/imu_message.h>
#include <srrg_messages/messages/transform_events_message.h>
#include <srrg_messages_ros/instances.h>
#include <srrg_pcl/instances.h>
#include <srrg_pcl/point_cloud.h>
#include <srrg_pcl/point_normal_curvature.h>
#include <srrg_pcl/point_projector.h>
#include <srrg_pcl/point_types.h>
#include <srrg_pcl/point_unprojector.h>
#include <srrg_solver/solver_core/solver.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>
#include <mad_ba/point_cloud_proc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>

using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_lidar3d_utils;
using namespace srrg2_core;
using namespace srrg2_solver;

ConfigurableManager manager;
std::shared_ptr<PipelineRunner> runner;

int main(int argc, char** argv) {
    // Initalize ROS
    srrgInit(argc, argv, "mad_ba");
    ros::init(argc, argv, "mad_ba");

    // Parse commandline arguments
    ParseCommandLine cmd(argv);
    ArgumentString config_file(&cmd, "c", "config", "config file to load", "");
    cmd.parse();

    // Find the dynamic libraries
    const std::string dl_path = ros::package::getPath("mad_ba") + "/dl.conf";
    ConfigurableManager::initFactory(dl_path);

    // Read the config file
    manager.read(config_file.value());
    // Retrieve a runner
    runner = manager.getByName<PipelineRunner>("runner");
    if (!runner) {
        std::cerr << std::string(environ[0]) + "|ERROR, cannot find runner, maybe wrong configuration path!" << std::endl;
    }
    // Retrieve a bag source
    auto source = dynamic_pointer_cast<MessageFileSourceBase>(runner->param_source.value());
    if (!source) {
        std::cerr << std::string(environ[0]) + "|ERROR, cannot find source, maybe wrong configuration path!" << std::endl;
    }
    // Retrieve a sink
    auto sink = manager.getByName<MessageSortedSink>("sink");
    if (!sink) {
        std::cerr << std::string(environ[0]) + "|ERROR, cannot find sink, maybe wrong configuration path!" << std::endl;
    }

    // source->open(source.value());
    runner->compute();

    manager.erase(runner);
    manager.erase(source);
    runner.reset();
    source.reset();
    return 0;
}