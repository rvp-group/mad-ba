#include <GL/glut.h>
#include <iostream>

#include <srrg_system_utils/system_utils.h>

#include "srrg_qgl_viewport_ros/viewer_core_ros_qgl.h"

using namespace srrg2_qgl_viewport_ros;

int main(int argc, char** argv) {
  if (argc < 3)
    throw std::runtime_error("usage: <exec> <node_name> <topic_name>");
  const std::string node_name(argv[1]);
  const std::string topic_name(argv[2]);

  std::cerr << "ros_client|client configuration" << std::endl;
  std::cerr << "node name      : " << FG_BCYAN(node_name) << std::endl;
  std::cerr << "topic name     : " << FG_BCYAN(topic_name) << std::endl;
  std::cerr << "\n\n";

  // ia allocate on the stack a QApplication to avoid strange memory effects
  QApplication qapp(argc, argv);

  // ia create a viewer core for this client app that receives shit over ros
  ViewerCoreRosQGL ros_viewer(
    argc, argv, &qapp, node_name, BUFFER_SIZE_2MEGABYTE, 5, 25, 500, true);
  ros_viewer.startViewerClient(topic_name);

  std::cerr << "ros_client|terminated" << std::endl;
  return 0;
}
