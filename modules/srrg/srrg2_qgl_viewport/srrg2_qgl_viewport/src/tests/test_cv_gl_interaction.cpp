#include <iostream>
#include <thread>

#include <GL/glut.h>
#include <opencv2/opencv.hpp>

#include <srrg_pcl/instances.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>

#include "srrg_qgl_viewport/viewer_core_shared_qgl.h"

const std::string exe_name("test_cv_gl_interaction");
#define LOG std::cerr << exe_name + "|"

using namespace srrg2_core;
using namespace srrg2_qgl_viewport;

size_t num_points               = 0;
std::string rgb_image_filename  = "";
const float cv_add_point_each_s = 0.1;

// ia slam thread
void processFn(const srrg2_core::ViewerCanvasPtr gl_canvas_);

int main(int argc, char** argv) {
  srrg2_core::ParseCommandLine cmd(argv);
  ArgumentString rgb_image_filepath(&cmd, "rgb", "image-rgb", "path to image", "");
  ArgumentInt number_of_points(&cmd, "np", "n-points", "number of points", 10000);
  cmd.parse();

  if (!rgb_image_filepath.isSet()) {
    throw std::runtime_error(exe_name + "|forgot rgb");
  }

  num_points         = number_of_points.value();
  rgb_image_filename = rgb_image_filepath.value();

  LOG << "start doing some shit\n";
  QApplication qapp(argc, argv);
  srrg2_qgl_viewport::ViewerCoreSharedQGL viewer_core(
    argc, argv, &qapp, BUFFER_SIZE_50MEGABYTE, 3, 25, true);
  ViewerCanvasPtr gl_canvas = viewer_core.getCanvas("gl canvas");

  std::thread process_t(processFn, gl_canvas);
  viewer_core.startViewerServer();

  process_t.join();

  return 0;
}

void processFn(const srrg2_core::ViewerCanvasPtr gl_canvas_) {
  PointNormalColor3fVectorCloud cloud;
  cloud.resize(num_points);

  cv::Mat cv_canvas(cv::imread(rgb_image_filename/* tg non compila con gcc 9.3, CV_LOAD_IMAGE_COLOR*/));

  bool time_expired = false;
  double t_delta    = 0;
  cv::Point cv_coords(4, 4);
  while (ViewerCoreSharedQGL::isRunning()) {
    SystemUsageCounter::tic();
    // ia construct a cloud
    for (auto& p : cloud) {
      p.coordinates() = Vector3f::Random() * 10.f;
      p.normal()      = Vector3f::UnitX();
      p.color()       = ColorPalette::color3fDarkCyan();
    }

    // ia draw a cloud
    gl_canvas_->putPoints(cloud);

    // ia add some red dot in the image each tot ms
    if (time_expired) {
      time_expired = false;
      cv_coords.x += 10;
      if (cv_coords.x > cv_canvas.cols) {
        cv_coords.x = 4;
        cv_coords.y += 10;

        if (cv_coords.y > cv_canvas.rows) {
          cv_coords.y = 4;
        }
      }
    }

    gl_canvas_->putImage(cv_canvas);

    gl_canvas_->flush();
    cv::circle(cv_canvas, cv_coords, 4, cv::Scalar(0, 0, 255));
    cv::imshow("external cv canvas", cv_canvas);

    t_delta += SystemUsageCounter::toc();

    //    std::cerr << "t_delta = " << t_delta << std::endl;
    //    std::cerr << "cv_point = " << cv_coords << std::endl;
    if (t_delta > cv_add_point_each_s) {
      t_delta      = 0;
      time_expired = true;
    }
  }
  return;
}
