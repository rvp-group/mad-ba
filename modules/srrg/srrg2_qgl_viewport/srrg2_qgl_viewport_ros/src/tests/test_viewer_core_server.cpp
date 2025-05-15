#include <iostream>
#include <srrg_system_utils/system_utils.h>

#include "../srrg_qgl_viewport_ros/viewer_core_ros_qgl.h"

using namespace srrg2_core;
using namespace srrg2_core_ros;
using namespace srrg2_qgl_viewport_ros;

#define SERVER_SLEEP_MS 1000

#define POINTS_BULK_SIZE 20000
#define WOLRD_SIZE 100

void producer(ViewerCanvasPtr canvas_);

int main(int argc, char** argv) {
  if (argc < 3)
    throw std::runtime_error("usage: <exec> <node_name> <canvas_name>");
  const std::string node_name(argv[1]);
  const std::string canvas_name(argv[2]);

  std::cerr << "ros_server|server configuration" << std::endl;
  std::cerr << "points number     : " << FG_BCYAN(POINTS_BULK_SIZE) << "\n";
  std::cerr << "world size        : " << FG_BCYAN(WOLRD_SIZE) << "\n";
  //  std::cerr << "server rate       : " << FG_BCYAN(float(1000.0f/SERVER_SLEEP_MS)) <<
  //  FG_BCYAN("Hz\n");
  std::cerr << "server rate       : " << FG_BCYAN("unlimited\n");
  std::cerr << "node name         : " << FG_BCYAN(node_name) << std::endl;
  std::cerr << "canvas/topic name : " << FG_BCYAN(canvas_name) << std::endl;
  std::cerr << "\n\n";

  // ia allocate on the stack a QApplication to avoid strange memory effects
  QApplication qapp(argc, argv);

  // ia create a viewer core for this server app that sends shit over ros - last argument tells the
  // system to install a sigint handler to properly shutdown everything
  ViewerCoreRosQGL ros_viewer(
    argc, argv, &qapp, node_name, BUFFER_SIZE_2MEGABYTE, 2, 25, 1000, true);
  const srrg2_core::ViewerCanvasPtr& canvas = ros_viewer.getCanvas(canvas_name);
  ros_viewer.startViewerServer();

  // ia start producing data
  std::cerr << "ros_server|starting producer" << std::endl;
  std::thread producer_t(producer, canvas);
  producer_t.join();

  return 0;
}

void producer(ViewerCanvasPtr canvas_) {
  // ia lettse draw (*mario style*)
  uint64_t cnt           = 0;
  uint64_t frame_counter = 0;
  double duration        = 0.0;
  // ia generate points
  PointNormal3fVectorCloud pointcloud;
  pointcloud.resize(POINTS_BULK_SIZE);

  while (ViewerCoreRosQGL::isRunning()) {
    std::cerr << "ros_server::producer|tic #" << cnt++ << "\r";
    std::flush(std::cerr);
    ++frame_counter;

    for (auto& p : pointcloud) {
      p.coordinates() = Vector3f::Random() * WOLRD_SIZE;
      p.normal()      = Vector3f::UnitX();
    }

    SystemUsageCounter::tic();
    // ia draw some red points
    canvas_->pushColor();
    canvas_->pushPointSize();
    canvas_->setPointSize(2.5);
    canvas_->setColor(ColorPalette::color3fRed());
    canvas_->putPoints(pointcloud);
    canvas_->popAttribute();
    canvas_->popAttribute();

    canvas_->pushColor();
    canvas_->setColor(ColorPalette::color3fGreen());
    canvas_->pushMatrix();
    for (size_t i = 0; i < cnt / 1000; ++i) {
      Isometry3f T    = Isometry3f::Identity();
      T.translation() = Vector3f(i, 0, 0);
      Matrix4f m      = T.matrix();
      canvas_->pushMatrix();
      canvas_->multMatrix(m);
      canvas_->putBox(Vector3f(1, 1, 1));
      canvas_->popMatrix();
    }
    canvas_->popAttribute();

    canvas_->pushColor();
    canvas_->setColor(ColorPalette::color3fBlue());
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

    canvas_->pushColor();
    canvas_->setColor(ColorPalette::color3fMint());
    canvas_->pushMatrix();
    transform                   = Matrix4f::Identity();
    transform.block<3, 1>(0, 3) = Vector3f(-6, 0, 0);
    canvas_->multMatrix(transform);
    canvas_->putText("server");
    canvas_->popMatrix();
    canvas_->popAttribute();

    canvas_->flush();

    //    std::this_thread::sleep_for(std::chrono::milliseconds(SERVER_SLEEP_MS));
    duration += SystemUsageCounter::toc();

    if (duration >= 2.0) {
      std::flush(std::cerr);
      std::cerr << "\nros_server::producer|FPS = " << FG_GREEN((double) frame_counter / duration)
                << " Hz\n";
      std::flush(std::cerr);
      duration      = 0.0;
      frame_counter = 0;
    }
  }

  std::cerr << "ros_server::producer|stop" << std::endl;
}
