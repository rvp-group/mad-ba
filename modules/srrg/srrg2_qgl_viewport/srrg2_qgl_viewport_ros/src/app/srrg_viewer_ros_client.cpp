#include <GL/glut.h>
#include <iostream>

// ia system utilities
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>

#include "srrg_qgl_viewport_ros/viewer_core_ros_qgl.h"

using namespace srrg2_core;
using namespace srrg2_qgl_viewport_ros;

const char* banner[] = {
  "This program runs a ros viewer client.",
  "It subscribes to the buffer topic you specify and shows the content on a qgl viewport",
  "usage: [options] <unique_node_name>",
  0};

const std::string exe_name = "srrg_viewer_ros_client|";

int main(int argc, char** argv) {
  // ia parse command line
  ParseCommandLine cmd_line(argv, banner);
  ArgumentString canvas_name(
    &cmd_line,
    "c",
    "canvas",
    "canvas name = topic where buffer are published (do rostopic list :) )",
    "canvas");
  ArgumentInt buffer_size(&cmd_line,
                          "bs",
                          "buffer-size",
                          "size of the buffers (must match the one of the server)",
                          BUFFER_SIZE_5MEGABYTE);
  ArgumentInt buffer_num(&cmd_line, "bn", "buffer-num", "number of buffers ", 5);
  ArgumentInt ros_rate(&cmd_line, "rr", "ros-rate", "rate of the listner in ms", 1000);
  ArgumentInt gl_rate(&cmd_line, "fr", "frame-rate", "rate of viewport in ms", 25);
  cmd_line.parse();

  if (cmd_line.lastParsedArgs().empty()) {
    throw std::runtime_error(exe_name + "please specify a valid node name");
  }

  const std::string node_name = cmd_line.lastParsedArgs()[0];

  std::cerr << exe_name + "client configuration" << std::endl;
  std::cerr << "node name      : " << FG_BCYAN(node_name) << std::endl;
  ;
  std::cerr << "canvas/topic   : " << FG_BCYAN(canvas_name.value()) << std::endl;
  std::cerr << "buffer dimesion: " << FG_BCYAN(buffer_size.value()) << std::endl;
  std::cerr << "ros rate [ms]  : " << FG_BCYAN(ros_rate.value()) << std::endl;
  std::cerr << "gl rate  [ms]  : " << FG_BCYAN(gl_rate.value()) << std::endl;
  std::cerr << "\n\n";

  // ia allocate on the stack a QApplication to avoid strange memory effects
  QApplication qapp(argc, argv);

  ViewerCoreRosQGL ros_viewer(argc,
                              argv,
                              &qapp,
                              node_name,
                              buffer_size.value(),
                              buffer_num.value(),
                              gl_rate.value(),
                              ros_rate.value(),
                              true);
  ros_viewer.startViewerClient(canvas_name.value());

  std::cerr << exe_name + "quit" << std::endl;
  return 0;
}
