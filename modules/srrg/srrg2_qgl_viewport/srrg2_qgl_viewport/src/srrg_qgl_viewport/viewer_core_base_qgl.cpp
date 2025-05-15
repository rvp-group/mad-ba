#include "viewer_core_base_qgl.h"

namespace srrg2_qgl_viewport {

  ViewerCoreBaseQGL::ViewerCoreBaseQGL(int argc_,
                                       char** argv_,
                                       QApplication* qapp_,
                                       const size_t& buffer_size_,
                                       const size_t& buffers_num_,
                                       const size_t& opengl_frame_time_ms_) :
    srrg2_core::ViewerCoreBase(argc_, argv_, buffer_size_, buffers_num_),
    _opengl_ms(opengl_frame_time_ms_) {
    assert(qapp_ && "ViewerCoreBaseQGL::ViewerCoreBaseQGL|invalid qapplication man");
    _qapp = qapp_;
  }

  ViewerCoreBaseQGL::~ViewerCoreBaseQGL() {
    // ds viewer is not owning the app (allocated in main scope)
    if (_qapp) {
      std::cerr << "ViewerCoreBaseQGL::~ViewerCoreBaseQGL|shutting down qapplication\n";
      _qapp->closeAllWindows();
      _qapp->quit();
    }
  }


  void ViewerCoreBaseQGL::_bindAllViewports() {
    if (!_qapp) {
      throw std::runtime_error("ViewerCoreBaseQGL::_bindAllViewports|ERROR: QApplication not set");
    }
    // ia for all canvases
    StringCanvasViewsMap::iterator c_it = _canvas_viewport_map.begin();
    while (c_it != _canvas_viewport_map.end()) {
      CanvasViews* canvas_view = c_it->second;

      // ia for all the views attached to that canvas, create a QGLViewport
      for (size_t v = 0; v < canvas_view->num_views; ++v) {
        if (canvas_view->viewports[v])
          continue;
        QGLViewport* qgl_viewport = new QGLViewport();
        qgl_viewport->setWindowTitle(c_it->first.c_str());
        qgl_viewport->param_rendering_sleep_milliseconds.setValue(_opengl_ms);
        qgl_viewport->setQGLServer(_qapp);

        canvas_view->viewports[v] = qgl_viewport;
        _viewports.insert(qgl_viewport);

        // ia bind viewport to canvas
        _viewer_manager->bindViewport(qgl_viewport, c_it->first);
      }

      ++c_it;
    }
  }

} // namespace srrg2_qgl_viewport
