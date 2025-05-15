#include <cstdio>

#include <GL/glew.h>
#include <GL/glut.h>
#include <QGLViewer/qglviewer.h>
#include <QtGui/QKeyEvent>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QWidget>
#include <QtWidgets/qapplication.h>

#include <opencv2/opencv.hpp>

#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_viewer/viewer_core/viewport.h>
#include <thread>

using namespace srrg2_core;
const std::string exe_name("test_image_rendering");
#define LOG std::cerr << exe_name + "|"
const float cv_add_point_each_s(0.1);
cv::Mat imayge;

// class ImageViewer : public QGLViewer {
// public:
//  ImageViewer(QWidget* parent = nullptr) {
//  }
//
//  ~ImageViewer() {
//  }
//
//  // Function turn a cv::Mat into a texture, and return the texture ID as a GLuint for use
//  static GLuint
//  matToTexture(const cv::Mat& mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter) {
//    std::cerr << "calling fn\n";
//    // Generate a number for our textureID's unique handle
//    GLuint textureID;
//    glGenTextures(1, &textureID);
//
//    // Bind to our texture handle
//    glBindTexture(GL_TEXTURE_2D, textureID);
//    std::cerr << "bind done\n";
//
//    // Catch silly-mistake texture interpolation method for magnification
//    if (magFilter == GL_LINEAR_MIPMAP_LINEAR || magFilter == GL_LINEAR_MIPMAP_NEAREST ||
//        magFilter == GL_NEAREST_MIPMAP_LINEAR || magFilter == GL_NEAREST_MIPMAP_NEAREST) {
//      std::cerr << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR"
//                << std::endl;
//      magFilter = GL_LINEAR;
//    }
//
//    // Set texture interpolation methods for minification and magnification
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);
//
//    // Set texture clamping method
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);
//
//    std::cerr << "texture params\n";
//
//    // Set incoming texture format to:
//    // GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
//    // GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
//    // Work out other mappings as required ( there's a list in comments in main() )
//    GLenum inputColourFormat = GL_BGR;
//    if (mat.channels() == 1) {
//      inputColourFormat = GL_LUMINANCE;
//    }
//
//    // Create the texture
//    std::cerr << "create texture\n";
//    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
//                 0,                 // Pyramid level (for mip-mapping) - 0 is the top level
//                 GL_RGB,            // Internal colour format to convert to
//                 mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
//                 mat.rows,          // Image height i.e. 480 for Kinect in standard mode
//                 0,                 // Border width in pixels (can either be 1 or 0)
//                 inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
//                 GL_UNSIGNED_BYTE,  // Image data type
//                 mat.ptr());        // The actual image data itself
//
//    std::cerr << "generating mipmap\n";
//    // If we're using mipmaps then generate them. Note: This requires OpenGL 3.0 or higher
//    if (minFilter == GL_LINEAR_MIPMAP_LINEAR || minFilter == GL_LINEAR_MIPMAP_NEAREST ||
//        minFilter == GL_NEAREST_MIPMAP_LINEAR || minFilter == GL_NEAREST_MIPMAP_NEAREST) {
//      glGenerateMipmap(GL_TEXTURE_2D);
//    }
//
//    std::cerr << "done\n";
//    return textureID;
//  }
//
//  void drawCvMat(const cv::Mat& frame) {
//    const int window_height = size().height();
//    const int window_width  = size().width();
//
//    // Clear color and depth buffers
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//    glMatrixMode(GL_MODELVIEW); // Operate on model-view matrix
//
//    glEnable(GL_TEXTURE_2D);
//    GLuint image_tex = matToTexture(frame, GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR, GL_CLAMP);
//
//    /* Draw a quad */
//    glBegin(GL_QUADS);
//    glTexCoord2i(0, 0);
//    glVertex2i(0, 0);
//    glTexCoord2i(0, 1);
//    glVertex2i(0, window_height);
//    glTexCoord2i(1, 1);
//    glVertex2i(window_width, window_height);
//    glTexCoord2i(1, 0);
//    glVertex2i(window_width, 0);
//    glEnd();
//
//    glDeleteTextures(1, &image_tex);
//    glDisable(GL_TEXTURE_2D);
//  }
//
//  void draw() override {
//    glClearColor(0.15, 0.15, 0.15, 1.00);
//    glPushAttrib(GL_COLOR | GL_POINT_SIZE | GL_LINE_WIDTH);
//    glPushMatrix();
//
//    //    glPushAttrib(GL_COLOR_BUFFER_BIT);
//    //    glColor3f(1.0f, 0.0f, 0.0f);
//    //    glBegin(GL_TRIANGLES);
//    //    glVertex3f(-1.0f, -1.0f, 0.0f);
//    //    glVertex3f(1.0f, -1.0f, 0.0f);
//    //    glVertex3f(0.0f, 1.0f, 0.0f);
//    //    glPopAttrib();
//    //    glEnd();
//
//    //    drawCvMat(imayge);
//  }
//
//  QImage image;
//};

// int main(int argc, char** argv) {
//  srrg2_core::ParseCommandLine cmd(argv);
//  ArgumentString rgb_image_filepath(&cmd, "rgb", "image-rgb", "path to image", "");
//  cmd.parse();
//
//  if (!rgb_image_filepath.isSet()) {
//    throw std::runtime_error(exe_name + "|forgot rgb");
//  }
//
//  const std::string rgb_image_filename(rgb_image_filepath.value());
//  LOG << "loading image [" << rgb_image_filename << "]\n";
//  imayge = cv::Mat(cv::imread(rgb_image_filename, CV_LOAD_IMAGE_COLOR));
//
//  // ia every tot millisecond draw something on the image
//  bool time_expired = false;
//  double t_delta    = 0;
//  cv::Point cv_coords(4, 4);
//
//  LOG << "starting qapp\n";
//  glutInit(&argc, argv);
//  glewInit();
//  QApplication qapp(argc, argv);
//  ImageViewer viewer;
//  viewer.show();
//
//  printf("OpenGL version supported by this platform (%s): \n", glGetString(GL_VERSION));
//
//  while (viewer.isVisible()) {
//    SystemUsageCounter::tic();
//
//    // ia count ms
//    if (time_expired) {
//      time_expired = false;
//      cv_coords.x += 10;
//      if (cv_coords.x > imayge.cols) {
//        cv_coords.x = 4;
//        cv_coords.y += 10;
//
//        if (cv_coords.y > imayge.rows) {
//          cv_coords.y = 4;
//        }
//      }
//    }
//
//    cv::circle(imayge, cv_coords, 4, cv::Scalar(0, 0, 255));
//    //    cv::imshow("external cv canvas", imayge);
//
//    if (t_delta > cv_add_point_each_s) {
//      t_delta      = 0;
//      time_expired = true;
//    }
//
//    viewer.updateGL();
//    qapp.processEvents();
//
//    const QSize q_size = viewer.size();
//    std::cerr << "\rq_size [height x width] = " << q_size.height() << " x " << q_size.width()
//              << " ";
//
//    t_delta += SystemUsageCounter::toc();
//  }
//  return 0;
//}
#include "qt_image_viewport.h"
int main(int argc, char** argv) {
  srrg2_core::ParseCommandLine cmd(argv);
  ArgumentString rgb_image_filepath_0(&cmd, "rgb0", "image-rgb-0", "path to image 1", "");
  ArgumentString rgb_image_filepath_1(&cmd, "rgb1", "image-rgb-1", "path to image 2", "");
  cmd.parse();

  if (!rgb_image_filepath_0.isSet() || !rgb_image_filepath_1.isSet()) {
    throw std::runtime_error(exe_name + "|forgot rgb");
  }

  std::cerr << rgb_image_filepath_0.value() << std::endl;
  std::cerr << rgb_image_filepath_1.value() << std::endl;

  std::vector<cv::Mat> imayges;
  imayges.push_back(
    cv::Mat(cv::imread(rgb_image_filepath_0.value(), CV_LOAD_IMAGE_COLOR))); // ia CV_8UC3 (uchar)
  imayges.push_back(
    cv::Mat(cv::imread(rgb_image_filepath_1.value(), CV_LOAD_IMAGE_COLOR))); // ia CV_8UC3 (uchar)

  QApplication qapp(argc, argv);
  QMainWindow window;
  srrg2_qgl_viewport::QtImageViewport qimage_viewer;
  window.setWindowTitle("Main Window");
  qimage_viewer.setWindowTitle("image viewer");
  window.setCentralWidget(&qimage_viewer);

  int i = 0;
  window.show();
  while (qimage_viewer.isVisible()) {
    //    std::cerr << i << std::endl;
    cv::imshow("cazzo", imayges[i++]);
    cv::waitKey(1000);
    //    qimage_viewer.showCvImage(imayges[i++]);
    //    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (i > 1) {
      i = 0;
    }
    qapp.processEvents();
  }
}
