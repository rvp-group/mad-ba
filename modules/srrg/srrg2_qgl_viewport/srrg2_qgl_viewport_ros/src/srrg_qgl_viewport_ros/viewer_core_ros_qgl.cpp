#include "viewer_core_ros_qgl.h"

#include <GL/glut.h>
#include <signal.h>

namespace srrg2_qgl_viewport_ros {

  ViewerCoreRosQGL::ViewerCoreRosQGL(int argc_,
                                     char** argv_,
                                     QApplication* qapp_,
                                     const std::string& node_name_,
                                     const size_t& buffer_size_,
                                     const size_t& buffers_num_,
                                     const size_t& opengl_frame_time_ms_,
                                     const size_t& ros_subscriber_rate_ms_,
                                     const bool install_sigint_handler_) :
    srrg2_qgl_viewport::ViewerCoreBaseQGL(argc_,
                                          argv_,
                                          qapp_,
                                          buffer_size_,
                                          buffers_num_,
                                          opengl_frame_time_ms_),
    _node_name(node_name_),
    _ros_subscriber_rate_ms(ros_subscriber_rate_ms_),
    _install_sighandler(install_sigint_handler_) {
    ros::init(argc_, argv_, _node_name);
    _node = new ros::NodeHandle();
    _rate = new ros::Rate((float) _ros_subscriber_rate_ms / 1000.0); // ia jic

    _viewer_manager = new srrg2_core_ros::ViewerManagerRos(_node);
    srrg2_core_ros::ViewerManagerRos* ros_viewer_manager =
      dynamic_cast<srrg2_core_ros::ViewerManagerRos*>(_viewer_manager);
    ros_viewer_manager->param_buffer_size.setValue(_buffer_size);
    ros_viewer_manager->param_max_num_buffers.setValue(_num_buffers);
  }

  ViewerCoreRosQGL::~ViewerCoreRosQGL() {
    if (_node) {
      delete _node;
      _node = 0;
    }
    if (_rate) {
      delete _rate;
      _rate = 0;
    }

    if (ros::ok()) {
      ros::shutdown();
    }
  }

  void ViewerCoreRosQGL::startViewerServer() {
    if (!_qapp) {
      throw std::runtime_error("ViewerCoreRosQGL::startViewerServer|invalid qapplication, exit");
    }

    if (_install_sighandler) {
      signal(SIGINT, ViewerCoreRosQGL::sigIntHandler);
    }
    _run = true;
  }

  void ViewerCoreRosQGL::startViewerClient(const std::string& canvas_name_) {
    if (!_qapp) {
      throw std::runtime_error("ViewerCoreRosQGL::startViewerClient|invalid qapplication, exit");
    }
    // ia register the canvas on the system (one view is enough. if you want more, just start more
    // clients)
    _canvas_viewport_map.insert(std::make_pair(canvas_name_, new CanvasViews(1)));

    // ia fire the engine
    signal(SIGINT, ViewerCoreRosQGL::sigIntHandler);

    // ia starting some threads
    glutInit(&_argc, _argv);

    // ia fire the engine
    _run = true;

    // ia create a viewport
    _bindAllViewports();

    // ia comm loop - 1Hz by default
    // ia (comm capacy required by default is 1Hz * 2MB = 4MB/s ~ 32 Mbit/s)
    // ia otherwise Mrs Tiziana Toni will complain
    std::thread receiver_t(ViewerCoreRosQGL::receive, _ros_subscriber_rate_ms);

    // ia spin and do stuff
    _updateActiveViewports();

    // ia checkout thigs, join threads, kiss a hen, do a twirl
    receiver_t.join();
    _unbindAllViewports();
  }

} // namespace srrg2_qgl_viewport_ros
