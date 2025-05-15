#include "viewer_core_shared_qgl.h"
#include <unistd.h>
#include <GL/glut.h>
#include <signal.h>

namespace srrg2_qgl_viewport {
  using namespace std;
  ViewerCoreSharedQGL::ViewerCoreSharedQGL(int argc_,
                                           char** argv_,
                                           QApplication* qapp_,
                                           const size_t& buffer_size_,
                                           const size_t& buffers_num_,
                                           const size_t& opengl_frame_time_ms_,
                                           const bool install_signal_handler_) :
    ViewerCoreBaseQGL(argc_, argv_, qapp_, buffer_size_, buffers_num_, opengl_frame_time_ms_),
    _install_sighandler(install_signal_handler_) {
    _viewer_manager = new srrg2_core::ViewerManagerShared();
    _viewer_manager->param_buffer_size.setValue(_buffer_size);
    _viewer_manager->param_max_num_buffers.setValue(_num_buffers);
  }

  void ViewerCoreSharedQGL::startViewerServer() {
    static bool is_init = false;
    // ia starting some threads
    if (!is_init) {
      glutInit(&_argc, _argv);
      is_init = true;
    }  


    // ds allocate a qt application if none is set - BEWARE OF THE PAIN
    if (!_qapp) {
      throw std::runtime_error("ViewerCoreSharedQGL::startViewerServer|invalid qapplication");
    }

    if (_install_sighandler) {
      std::cerr << "ViewerCoreSharedQGL::startViewerServer|installing sigint handler, press CTRL+C "
                   "to stop viewer"
                << std::endl;
      signal(SIGINT, ViewerCoreSharedQGL::sigIntHandler);
    }

    // ia create viewports bind viewport to canvas and start the engine
    _bindAllViewports();
    _run = true;

    // ia spin and do stuff
    _updateActiveViewports();

    // ia if we are here, there is no active viewport:
    // ia cleanup and checkout things
    _unbindAllViewports();
  }

  const srrg2_core::ViewerCanvasPtr&
  ViewerCoreSharedQGL::getCanvasMultiview(const std::string& canvas_name_,
                                          const size_t& num_views_) {
    if (!_viewer_manager) {
      throw std::runtime_error("ViewerCoreBase::getCanvas|invalid manager");
    }

    const srrg2_core::ViewerCanvasPtr& canvas = _viewer_manager->getCanvas(canvas_name_);
    if (!canvas) {
      throw std::runtime_error("ViewerCoreBase::getCanvas|unable to create valid canvas");
    }

    // ia never do shit with []operator. inserting a new canvas
    // ia this canvas will have multiple views. still, the views are not active
    // yet
    _canvas_viewport_map.insert(std::make_pair(canvas_name_, new CanvasViews(num_views_)));
    return canvas;
  }

} // namespace srrg2_qgl_viewport
