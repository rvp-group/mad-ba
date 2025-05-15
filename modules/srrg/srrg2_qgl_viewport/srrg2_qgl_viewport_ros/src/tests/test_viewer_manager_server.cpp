#include <iostream>

#include <GL/glut.h>
#include <thread>

#include <srrg_qgl_viewport/qgl_viewport.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_viewer_ros/viewer_manager_ros.h>

#include <signal.h>

using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_qgl_viewport;
using namespace std;

#define SERVER_SLEEP_MS 500
#define MAX_BUFFER_SIZE 1024 * 1024 * 2
#define MAX_BUFFER_NUMBER 3

#define POINTS_BULK_SIZE 20000
#define WOLRD_SIZE 100

#define CANVAS_NAME "test_viewer_manager_canvas"

#define COLOR_RED Eigen::Vector3f(0.8, 0.2, 0.2)
#define COLOR_BLUE Eigen::Vector3f(0.2, 0.2, 0.8)
#define COLOR_GREEN Eigen::Vector3f(0.2, 0.8, 0.2)
#define COLOR_BLACK Eigen::Vector3f(0.15, 0.15, 0.15)

void sigIntHandler(int signum_) {
  if (signum_ == SIGINT) {
    std::flush(std::cerr);
    std::cerr << FG_BRED("ros_server|shutdown request") << std::endl;
    std::flush(std::cerr);
    ros::shutdown();
  }
}

void producer(ViewerCanvasPtr canvas_, const PointNormal3fVectorCloud& other_points_);

int main(int argc, char** argv) {
  std::cerr << "ros_server|server configuration" << std::endl;
  std::cerr << "points number  : " << FG_BCYAN(POINTS_BULK_SIZE) << "\n";
  std::cerr << "world size     : " << FG_BCYAN(WOLRD_SIZE) << "\n";
  std::cerr << "buffer size    : " << FG_BCYAN(MAX_BUFFER_SIZE) << "\n";
  std::cerr << "buffers number : " << FG_BCYAN(MAX_BUFFER_NUMBER) << "\n";
  std::cerr << "server rate    : " << FG_BCYAN(double(1000.0 / SERVER_SLEEP_MS))
            << FG_BCYAN("Hz\n");
  std::cerr << "topic name     : " << FG_BCYAN(CANVAS_NAME) << "\n";
  std::cerr << "\n\n";

  // ia generate points - remember to allocate things on the heap if you need a lot of space dumbass
  PointNormal3fVectorCloud pointcloud;
  pointcloud.resize(POINTS_BULK_SIZE);
  for (auto& p : pointcloud) {
    p.coordinates() = Vector3f::Random() * WOLRD_SIZE;
    p.normal()      = Vector3f::UnitX();
  }

  // ia appicc ros
  ros::init(argc, argv, "test_viewer_manager_server_node");
  ros::NodeHandle nh;
  ros::Rate ros_rate(1);
  signal(SIGINT, sigIntHandler);

  // ia setup the viewer manger
  ViewerManagerRos viewer_lord(&nh);
  viewer_lord.param_buffer_size.setValue(MAX_BUFFER_SIZE);
  viewer_lord.param_max_num_buffers.setValue(MAX_BUFFER_NUMBER);
  viewer_lord.param_rosmaster_uri.setValue("http://srrg-07:11311/");

  // ia get a canvas
  ViewerCanvasPtr canvas = viewer_lord.getCanvas(CANVAS_NAME);
  if (!canvas)
    throw std::runtime_error("ros_server|cannot get a canvas");

  // ia start producing data
  std::cerr << "ros_server|canvas created, starting producer" << std::endl;

  std::thread producer_t(producer, canvas, pointcloud);
  producer_t.join();

  return 0;
}

void producer(ViewerCanvasPtr canvas_, const PointNormal3fVectorCloud& other_points_) {
  // ia lettse draw (*mario style*)
  uint64_t cnt           = 0;
  uint64_t frame_counter = 0;
  double duration        = 0.0;
  while (ros::ok()) {
    std::cerr << "ros_server::producer|tick #" << cnt++ << "\r";
    std::flush(std::cerr);
    ++frame_counter;
    SystemUsageCounter::tic();
    // ia draw some red points
    canvas_->pushColor();
    canvas_->pushPointSize();
    canvas_->setPointSize(2.5);
    canvas_->setColor(COLOR_RED);
    canvas_->putPoints(other_points_);
    canvas_->popAttribute();
    canvas_->popAttribute();

    canvas_->pushColor();
    canvas_->setColor(COLOR_BLUE);
    canvas_->pushMatrix();
    Matrix4f transform = Matrix4f::Identity();
    Matrix3f rot;
    rot                         = Eigen::AngleAxisf(M_PI, Vector3f::UnitX());
    transform.block<3, 3>(0, 0) = rot;
    transform.block<3, 1>(0, 3) = Vector3f(2, 0, 0);
    canvas_->multMatrix(transform);
    canvas_->putPyramidWireframe(Vector2f(2.0f, 1.0f));
    canvas_->popMatrix();
    canvas_->popAttribute();

    canvas_->flush();
    duration += SystemUsageCounter::toc();

    if (duration >= 2.0) {
      std::cerr << "ros_server::producer|FPS = " << FG_GREEN((double) frame_counter / duration)
                << " Hz\r";
      std::flush(std::cerr);
      duration      = 0.0;
      frame_counter = 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(SERVER_SLEEP_MS));
  }

  std::cerr << "ros_server::producer|stop" << std::endl;
}
