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

static const std::string canvas_name("sto_canvas");

std::atomic<bool> run;
static void sigIntHandler(int __attribute__((unused))) {
  cerr << "user interrupt, exiting" << endl;
  run = false;
}

void producer(ViewerCanvasPtr canvas_);

int main(int argc, char** argv) {
  run = true;
  signal(SIGINT, sigIntHandler);

  std::cerr << "Test viewer manager configuration:" << std::endl;
  std::cerr << "MAX_BUFFER_SIZE   : " << FG_BCYAN(MAX_BUFFER_SIZE) << "\n";
  std::cerr << "MAX_BUFFER_NUMBER : " << FG_BCYAN(MAX_BUFFER_NUMBER) << "\n";
  std::cerr << "MAX_FPS_PRODUCER  : " << FG_BCYAN("unlimited Hz\n");
  std::cerr << "MAX_FPS_CONSUMER  : " << FG_BCYAN(double(1000.0 / SLEEP_MS_CONSUMER))
            << FG_BCYAN("Hz\n");
  std::cerr << "\n\n";

  // ia setup the viewer manger
  ViewerManagerShared viewer_lord;
  viewer_lord.param_buffer_size.setValue(MAX_BUFFER_SIZE);
  viewer_lord.param_max_num_buffers.setValue(MAX_BUFFER_NUMBER);

  // ia request a new canvas
  ViewerCanvasPtr canvas = viewer_lord.getCanvas(canvas_name);
  if (!canvas)
    throw std::runtime_error("your life is meaningless");

  // ia start producing data
  std::thread producer_t(producer, canvas);

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
  cv::destroyAllWindows();

  viewer_lord.unbindViewport(viewport, canvas_name);

  producer_t.join();

  qapp->closeAllWindows();
  qapp->quit();
  delete qapp;

  return 0;
}

void producer(ViewerCanvasPtr canvas_) {
  // ia lettse draw (*mario style*)
  uint64_t frame_counter = 0;
  double duration        = 0.0;
  while (run) {
    ++frame_counter;
    SystemUsageCounter::tic();

    // ds put some controversial text
    canvas_->putText("nel mezzo del cammin di nostra vita .... matita");

    // ds put image into the opencv canvas
    cv::Mat image(500, 500, CV_8UC3);
    cv::randu(image, cv::Scalar(100, 100, 100), cv::Scalar(50, 50, 50));
    //    canvas_->putImage(image);

    // ds release to buffer
    canvas_->flush();
    duration += SystemUsageCounter::toc();
    if (duration >= 2.0) {
      std::cerr << "producer FPS = " << FG_GREEN((double) frame_counter / duration) << " Hz\r";
      std::flush(std::cerr);
      duration      = 0.0;
      frame_counter = 0;
    }
  }
}
