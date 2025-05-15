#include <GL/glut.h>
#include <iostream>
#include <thread>

#include <srrg_qgl_viewport/qgl_viewport.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_viewer_ros/viewer_manager_ros.h>

#include <signal.h>

#define CLIENT_SLEEP_MS 100
#define OPENGL_FRAME_MS 15
#define MAX_BUFFER_SIZE 1024 * 1024 * 2
#define MAX_BUFFER_NUMBER 3

#define CANVAS_NAME "test_viewer_manager_canvas"

std::atomic<bool> run;

void sigIntHandler(int signum_) {
  if (signum_ == SIGINT) {
    std::flush(std::cerr);
    std::cerr << FG_BRED("ros_client|shutdown request") << std::endl;
    std::flush(std::cerr);
    ros::shutdown();
    run = false;
  }
}

using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_qgl_viewport;

void receive() {
  while (ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(CLIENT_SLEEP_MS));
    ros::spinOnce();
  }
}

int main(int argc, char** argv) {
  if (argc < 2)
    throw std::runtime_error("usage: <exec> <node_name>");

  run = true;

  const std::string node_name(argv[1]);

  std::cerr << "ros_client|client configuration" << std::endl;
  std::cerr << "buffer size    : " << FG_BCYAN(MAX_BUFFER_SIZE) << "\n";
  std::cerr << "buffers number : " << FG_BCYAN(MAX_BUFFER_NUMBER) << "\n";
  std::cerr << "client rate    : " << FG_BCYAN(double(1000.0 / CLIENT_SLEEP_MS))
            << FG_BCYAN("Hz\n");
  std::cerr << "gl max fps     : " << FG_BCYAN(double(1000.0 / OPENGL_FRAME_MS))
            << FG_BCYAN("Hz\n");
  std::cerr << "topic name     : " << FG_BCYAN(CANVAS_NAME) << "\n";
  std::cerr << "\n\n";

  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;

  signal(SIGINT, sigIntHandler);

  glutInit(&argc, argv);
  QApplication* qapp = new QApplication(argc, argv);

  // ia create a viewport
  ViewerManagerRos viewer_lord(&nh);
  viewer_lord.param_buffer_size.setValue(MAX_BUFFER_SIZE);
  viewer_lord.param_max_num_buffers.setValue(MAX_BUFFER_NUMBER);
  viewer_lord.param_rosmaster_uri.setValue("http://srrg-07:11311/");

  QGLViewport* viewport = new QGLViewport();
  viewport->param_rendering_sleep_milliseconds.setValue(
    OPENGL_FRAME_MS); // ia cap opengl max fps to 33fps
  viewport->setQGLServer(qapp);

  // ia bind viewport to canvas
  viewer_lord.bindViewport(viewport, CANVAS_NAME);

  std::thread receiver_t(receive);

  while (viewport->isActive() && run) {
    viewport->update();
  }

  // ia spin and do stuff
  receiver_t.join();

  viewer_lord.unbindViewport(viewport, CANVAS_NAME);

  qapp->closeAllWindows();
  qapp->quit();
  delete viewport;
  delete qapp;

  return 0;
}
