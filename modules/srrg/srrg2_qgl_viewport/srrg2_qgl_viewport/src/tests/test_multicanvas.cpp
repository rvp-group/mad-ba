#include <GL/glut.h>
#include <iostream>
#include <signal.h>
#include <thread>

#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_viewer/viewer_manager_shared.h>

#include "srrg_qgl_viewport/qgl_viewport.h"

#define POINTS_BULK_SIZE 1000
#define WORLD_SIZE 10
#define CANVAS_0_NAME "canvas_0"
#define CANVAS_1_NAME "canvas_1"

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace std;

std::atomic<bool> run;
static void sigIntHandler(int __attribute__((unused))) {
  cerr << "user interrupt, exiting" << endl;
  run = false;
}

void producer(const ViewerCanvasPtr& canvas_0_, const ViewerCanvasPtr& canvas_1_);

int main(int argc, char** argv) {
  run = true;
  signal(SIGINT, sigIntHandler);

  ViewerManagerShared viewer_lord;
  viewer_lord.param_buffer_size.setValue(BUFFER_SIZE_2MEGABYTE);
  viewer_lord.param_max_num_buffers.setValue(3);

  // ia request a new canvas
  ViewerCanvasPtr canvas_0 = viewer_lord.getCanvas(CANVAS_0_NAME);
  ViewerCanvasPtr canvas_1 = viewer_lord.getCanvas(CANVAS_1_NAME);
  if (!canvas_0 || !canvas_1)
    throw std::runtime_error("unexpected error");

  // ia start producing data
  std::thread producer_t(producer, canvas_0, canvas_1);

  // ia starting some threads
  glutInit(&argc, argv);
  QApplication* qapp = new QApplication(argc, argv);

  // ia create 3 viewports. 1 renders canvas0, the other 2 render canvas1
  QGLViewport* viewport_0  = new QGLViewport();
  QGLViewport* viewport_1A = new QGLViewport();
  QGLViewport* viewport_1B = new QGLViewport();
  viewport_0->param_rendering_sleep_milliseconds.setValue(10);
  viewport_1A->param_rendering_sleep_milliseconds.setValue(10);
  viewport_1B->param_rendering_sleep_milliseconds.setValue(10);
  viewport_0->setQGLServer(qapp);
  viewport_1A->setQGLServer(qapp);
  viewport_1B->setQGLServer(qapp);

  viewer_lord.bindViewport(viewport_0, CANVAS_0_NAME);
  viewer_lord.bindViewport(viewport_1A, CANVAS_1_NAME);
  viewer_lord.bindViewport(viewport_1B, CANVAS_1_NAME);

  // ia spin and do stuff
  while (run) {
    if (viewport_0->isActive())
      viewport_0->update();
    if (viewport_1A->isActive())
      viewport_1A->update();
    if (viewport_1B->isActive())
      viewport_1B->update();

    // ia if noone is active close all
    if (!viewport_0->isActive() && !viewport_1A->isActive() && !viewport_1B->isActive()) {
      run = false;
    }
  }

  // ia unbind the viewports
  viewer_lord.unbindViewport(viewport_0, CANVAS_0_NAME);
  viewer_lord.unbindViewport(viewport_1A, CANVAS_1_NAME);
  viewer_lord.unbindViewport(viewport_1B, CANVAS_1_NAME);

  // ia checkout things
  producer_t.join();
  qapp->closeAllWindows();
  qapp->quit();
  delete qapp;
  delete viewport_0;
  delete viewport_1A;
  delete viewport_1B;
  return 0;
}

void producer(const ViewerCanvasPtr& canvas_0_, const ViewerCanvasPtr& canvas_1_) {
  std::string text = "testing viewer manager";
  PointNormal3fVectorCloud pointcloud_0;
  pointcloud_0.resize(POINTS_BULK_SIZE);
  PointNormal3fVectorCloud pointcloud_1;
  pointcloud_1.resize(POINTS_BULK_SIZE);
  // ia lettse draw (*mario style*)
  uint64_t frame_counter = 0;
  double duration        = 0.0;
  while (run) {
    ++frame_counter;
    for (auto& p : pointcloud_0) {
      p.coordinates() = Vector3f::Random() * WORLD_SIZE;
      p.normal()      = Vector3f::UnitX();
    }

    for (size_t i = 0; i < pointcloud_0.size(); ++i) {
      pointcloud_1[i] = pointcloud_0[i];
    }

    SystemUsageCounter::tic();
    // ia draw points in the first canvas
    canvas_0_->pushColor();
    canvas_0_->pushPointSize();
    canvas_0_->setPointSize(2.5);
    canvas_0_->setColor(ColorPalette::color3fBlue());
    canvas_0_->putPoints(pointcloud_0);
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
