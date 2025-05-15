#include <iostream>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>
#include <thread>

#include "srrg_qgl_viewport/viewer_core_shared_qgl.h"

const std::string exe_name("test_viewer_core_multicanvas|");
#define LOG std::cerr << exe_name

#define POINTS_BULK_ROWS 100
#define POINTS_BULK_COLS 100
#define POINTS_BULK_SIZE POINTS_BULK_ROWS* POINTS_BULK_COLS
#define WORLD_SIZE 10
#define CANVAS_0_NAME "canvas_0"
#define CANVAS_1_NAME "canvas_1"

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;

// ia producer thread, simulates a processing module with canvases
void producer(const ViewerCanvasPtr& canvas_0_, const ViewerCanvasPtr& canvas_1_);

int main(int argc, char** argv) {
  LOG << "testing multicanvas things" << std::endl;

  // ia create a STATIC QApplication in the main thread
  QApplication qapp(argc, argv);

  // ia create a guy - last argument specifies that we install also a sigint handler to properly
  // shutdown the app
  ViewerCoreSharedQGL viewer_core(argc, argv, &qapp, BUFFER_SIZE_1MEGABYTE, 5, 0, true);

  // ia get canvases
  const ViewerCanvasPtr& canvas_0 = viewer_core.getCanvas(CANVAS_0_NAME);
  const ViewerCanvasPtr& canvas_1 = viewer_core.getCanvasMultiview(CANVAS_1_NAME, 2);

  // ia start the processing module thread
  std::thread producer_t(producer, canvas_0, canvas_1);

  // ia start the viewer server
  viewer_core.startViewerServer();
  producer_t.join();

  return 0;
}

// ia producers
void producer(const ViewerCanvasPtr& canvas_0_, const ViewerCanvasPtr& canvas_1_) {
  PointNormal3fVectorCloud pointcloud_vector;
  pointcloud_vector.resize(POINTS_BULK_SIZE);

  PointNormalColor3fMatrixCloud pointcloud_matrix;
  pointcloud_matrix.resize(POINTS_BULK_ROWS, POINTS_BULK_ROWS);
  for (size_t r = 0; r < pointcloud_matrix.rows(); ++r) {
    for (size_t c = 0; c < pointcloud_matrix.rows(); ++c) {
      auto& p         = pointcloud_matrix.at(r, c);
      p.coordinates() = Vector3f::Random() * WORLD_SIZE;
      p.normal()      = Vector3f::UnitY();
      p.color()       = Vector3f::Random();
    }
  }

  // ia lettse draw (*mario style*)
  uint64_t frame_counter = 0;
  double duration        = 0.0;
  while (ViewerCoreSharedQGL::isRunning()) {
    ++frame_counter;
    for (auto& p : pointcloud_vector) {
      p.coordinates() = Vector3f::Random() * WORLD_SIZE;
      p.normal()      = Vector3f::UnitX();
    }

    SystemUsageCounter::tic();
    // ia draw points in the first canvas
    canvas_0_->pushColor();
    canvas_0_->pushPointSize();
    canvas_0_->setPointSize(2.5);
    canvas_0_->setColor(ColorPalette::color3fOrange());
    canvas_0_->putPoints(pointcloud_vector);
    canvas_0_->popAttribute();
    canvas_0_->popAttribute();
    canvas_0_->flush();

    // ia draw points in the second canvas
    canvas_1_->pushColor();
    canvas_1_->setColor(ColorPalette::color3fGreen());
    canvas_1_->pushMatrix();
    Matrix4f transform = Matrix4f::Identity();
    Matrix3f rot;
    rot                         = Eigen::AngleAxisf(M_PI, Vector3f::UnitX());
    transform.block<3, 3>(0, 0) = rot;
    transform.block<3, 1>(0, 3) = Vector3f(frame_counter / 1000, 0, 0);
    canvas_1_->multMatrix(transform);
    canvas_1_->putPyramidWireframe(Vector2f(2.0f, 1.0f));
    canvas_1_->popMatrix();
    canvas_1_->popAttribute();
    canvas_0_->pushPointSize();
    canvas_0_->setPointSize(1.5);
    canvas_1_->putPoints(pointcloud_matrix);
    canvas_0_->popAttribute();
    canvas_1_->flush();
    duration += SystemUsageCounter::toc();

    // ia producer fps
    if (duration >= 2.0) {
      std::cerr << "producer FPS = " << FG_GREEN((double) frame_counter / duration) << " Hz\r";
      std::flush(std::cerr);
      duration      = 0.0;
      frame_counter = 0;
    }
    //    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
