#include <GL/glut.h>
#include <iostream>
#include <thread>

#include <srrg_system_utils/system_utils.h>
#include <srrg_viewer/viewer_manager_shared.h>

#include "srrg_qgl_viewport/qgl_viewport.h"

#include <signal.h>

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;
using namespace std;

//#define SLEEP_MS_PRODUCER 25
#define SLEEP_MS_CONSUMER 30
#define MAX_BUFFER_SIZE 1024 * 1024 * 20
#define MAX_BUFFER_NUMBER 5

#define POINTS_BULK_SIZE 200000
#define WOLRD_SIZE 100

static const Vector3f COLOR_RED   = Vector3f(0.8, 0.2, 0.2);
static const Vector3f COLOR_BLUE  = Vector3f(0.2, 0.2, 0.8);
static const Vector3f COLOR_GREEN = Vector3f(0.2, 0.8, 0.2);
// static const Vector3f COLOR_BLACK = Vector3f(0.15,0.15,0.15);

static const std::string canvas_name("sto_canvas");

std::atomic<bool> run;
static void sigIntHandler(int __attribute__((unused))) {
  cerr << "user interrupt, exiting" << endl;
  run = false;
}

void producer(ViewerCanvasPtr canvas_, const PointNormalColor3fVectorCloud& other_points_);

int main(int argc, char** argv) {
  run = true;
  signal(SIGINT, sigIntHandler);

  std::cerr << "Test viewer manager configuration:" << std::endl;
  std::cerr << "POINTS_BULK_SIZE  : " << FG_BCYAN(POINTS_BULK_SIZE) << "\n";
  std::cerr << "MAX_BUFFER_SIZE   : " << FG_BCYAN(MAX_BUFFER_SIZE) << "\n";
  std::cerr << "MAX_BUFFER_NUMBER : " << FG_BCYAN(MAX_BUFFER_NUMBER) << "\n";
  std::cerr << "MAX_FPS_PRODUCER  : " << FG_BCYAN("unlimited Hz\n");
  std::cerr << "MAX_FPS_CONSUMER  : " << FG_BCYAN(double(1000.0 / SLEEP_MS_CONSUMER))
            << FG_BCYAN("Hz\n");
  std::cerr << "\n\n";

  // ia generate points - remember to allocate things on the heap if you need a
  // lot of space dumbass
  PointNormalColor3fVectorCloud pointcloud;
  pointcloud.resize(POINTS_BULK_SIZE);
  for (auto& p : pointcloud) {
    p.coordinates() = Vector3f::Random() * WOLRD_SIZE;
    p.normal()      = Vector3f::UnitX();
    p.color().setZero();
  }

  // ia setup the viewer manger
  ViewerManagerShared viewer_lord;
  viewer_lord.param_buffer_size.setValue(MAX_BUFFER_SIZE);
  viewer_lord.param_max_num_buffers.setValue(MAX_BUFFER_NUMBER);

  // ia request a new canvas
  ViewerCanvasPtr canvas = viewer_lord.getCanvas(canvas_name);
  if (!canvas)
    throw std::runtime_error("your life is meaningless");

  // ia start producing data
  std::thread producer_t(producer, canvas, pointcloud);

  // ia starting some threads
  glutInit(&argc, argv);
  QApplication* qapp = new QApplication(argc, argv);

  // ia create a viewport
  QGLViewport* viewport = new QGLViewport();
  viewport->param_rendering_sleep_milliseconds.setValue(
    SLEEP_MS_CONSUMER); // ia cap opengl max fps to 33fps
  viewport->setQGLServer(qapp);

  // ia bind viewport to canvas
  viewer_lord.bindViewport(viewport, canvas_name);

  // ia spin and do stuff
  while (viewport->isActive() && run) {
    viewport->update();
  }

  viewer_lord.unbindViewport(viewport, canvas_name);

  producer_t.join();

  qapp->closeAllWindows();
  qapp->quit();
  delete qapp;

  return 0;
}

void producer(ViewerCanvasPtr canvas_, const PointNormalColor3fVectorCloud& other_points_) {
  std::string text = "testing viewer manager";

  PointNormalColor3fVectorCloud polygon_points;
  PointNormalColor3f p_0;
  p_0.coordinates() = Vector3f(-5.0, -12.0, 0.0);
  p_0.color()       = COLOR_RED;
  polygon_points.push_back(p_0);
  PointNormalColor3f p_1;
  p_1.coordinates() = Vector3f(-7.0, 8.0, 0.0);
  p_1.color()       = COLOR_RED;
  polygon_points.push_back(p_1);
  PointNormalColor3f p_2;
  p_2.coordinates() = Vector3f(6.0, -5.5, 0.0);
  p_2.color()       = COLOR_RED;
  polygon_points.push_back(p_2);
  PointNormalColor3f p_3;
  p_3.coordinates() = Vector3f(11.0, 12.0, 0.0);
  p_3.color()       = COLOR_RED;
  polygon_points.push_back(p_3);

  // ia lettse draw (*mario style*)
  uint64_t frame_counter = 0;
  double duration        = 0.0;
  while (run) {
    ++frame_counter;
    SystemUsageCounter::tic();
    // ia draw a colored points polygon
    canvas_->pushPointSize();
    canvas_->setPointSize(1.5);
    canvas_->putPoints(other_points_);
    canvas_->popAttribute();

    // ia draw a cyan polygon
    canvas_->pushColor();
    canvas_->pushPointSize();
    canvas_->setPointSize(5.5);
    canvas_->setColor(ColorPalette::color4fCyan(0.6));
    canvas_->putPolygon(polygon_points);
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
    canvas_->putText("porca vacca");
    canvas_->putPyramidWireframe(Vector2f(2.0f, 1.0f));
    canvas_->popMatrix();
    canvas_->popAttribute();

    canvas_->flush();
    duration += SystemUsageCounter::toc();

    if (duration >= 2.0) {
      std::cerr << "producer FPS = " << FG_GREEN((double) frame_counter / duration) << " Hz\r";
      std::flush(std::cerr);
      duration      = 0.0;
      frame_counter = 0;
    }
    //    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS));
  }
}
