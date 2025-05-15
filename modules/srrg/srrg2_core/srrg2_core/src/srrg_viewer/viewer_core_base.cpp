#include "viewer_core_base.h"
#include <unistd.h>
namespace srrg2_core {
  using namespace std;
  
  std::atomic<bool> ViewerCoreBase::_run(true);

  ViewerCoreBase::ViewerCoreBase(const int& argc_,
                                 char** argv_,
                                 const size_t& buffer_size_,
                                 const size_t& buffer_num_) :
    _buffer_size(buffer_size_),
    _num_buffers(buffer_num_),
    _argc(argc_),
    _argv(argv_) {
    // ia do nothin
  }

  ViewerCoreBase::~ViewerCoreBase() {
    _run = false;
    for (ViewportBase* v : _viewports) {
      delete v;
      v = 0;
    }

    StringCanvasViewsMap::iterator c_it = _canvas_viewport_map.begin();
    while (c_it != _canvas_viewport_map.end()) {
      delete c_it->second;
      c_it->second = 0;
      ++c_it;
    }

    _viewports.clear();
    _canvas_viewport_map.clear();

    if (_viewer_manager) {
      delete _viewer_manager;
      _viewer_manager = 0;
    }
  }

  const srrg2_core::ViewerCanvasPtr& ViewerCoreBase::getCanvas(const std::string& canvas_name_) {
    if (!_viewer_manager) {
      throw std::runtime_error("ViewerCoreBase::getCanvas|invalid manager");
    }
    const ViewerCanvasPtr& canvas = _viewer_manager->getCanvas(canvas_name_);
    if (!canvas) {
      throw std::runtime_error("ViewerCoreBase::getCanvas|unable to create valid canvas");
    }

    // ia never do shit with []operator. inserting a new canvas, not yet binded to any viewport
    // ia single view, of the canvas. if you want to have multiple views of the same canvas
    // ia you can either use the specific function (in shared mode) or start multiple clients
    // ia (if you are in ros mode).
    _canvas_viewport_map.insert(std::make_pair(canvas_name_, new CanvasViews(1)));
    return canvas;
  }

  void ViewerCoreBase::_updateActiveViewports() {
    //    if (!_viewports.size()) {
    //      throw std::runtime_error("ViewerCoreBase::_updateActiveViewports|no viewports");
    //    }
    while (_run) {
      size_t num_active_viewports = 0;
      _bindAllViewports();

      StringCanvasViewsMap::iterator c_it = _canvas_viewport_map.begin();
      while (c_it != _canvas_viewport_map.end()) {
        CanvasViews* canvas_view = c_it->second;
        for (size_t v = 0; v < canvas_view->num_views; ++v) {
          srrg2_core::ViewportBase* viewport = canvas_view->viewports[v];
          if (!viewport) {
            std::cerr << "viewport not active" << std::endl;
            continue;
          }
          assert(viewport && "ViewerCoreBase::_updateLoop|invalid viewport");

          // ia if the viewport is active, do the rendering.
          // ia if not, unbind it from the context
          if (viewport->isActive()) {
            viewport->update();
            ++num_active_viewports;
          } else {
            // ia unbind the viewport
            _viewer_manager->unbindViewport(viewport, c_it->first);
            // ia remove it from the view
            canvas_view->viewports.erase(canvas_view->viewports.begin() + v);
            canvas_view->num_views--;
          }
        }
        ++c_it;
      }

      // ia if no viewport is active shutdown everything
      if (!num_active_viewports) {
        usleep(10000);
        ViewerCoreBase::stop();
      }
    }
  }

  bool ViewerCoreBase::detachCanvas(const std::string& canvas_name_) {
    auto it = _canvas_viewport_map.find(canvas_name_);
    if (it == _canvas_viewport_map.end()) {
      return false;
    }

    CanvasViews* canvas_view = it->second;
    for (size_t v = 0; v < canvas_view->num_views; ++v) {
      srrg2_core::ViewportBase* viewport = canvas_view->viewports[v];

      if (!viewport) {
        break;
      }

      if (viewport->isActive()) {
        _viewer_manager->unbindViewport(canvas_view->viewports[v], it->first);
      }
    }

    _canvas_viewport_map.erase(it);

    return true;
  }

  void ViewerCoreBase::_unbindAllViewports() {
    StringCanvasViewsMap::iterator c_it = _canvas_viewport_map.begin();
    while (c_it != _canvas_viewport_map.end()) {
      CanvasViews* canvas_view = c_it->second;
      for (size_t v = 0; v < canvas_view->num_views; ++v) {
        srrg2_core::ViewportBase* viewport = canvas_view->viewports[v];

        if (!viewport) {
          continue;
          //throw std::runtime_error("ViewerCoreBase::_unbindAllViewports|unexpected error");
        }

        if (viewport->isActive()) {
          _viewer_manager->unbindViewport(canvas_view->viewports[v], c_it->first);
        }
      }
      ++c_it;
    }
  }

} // namespace srrg2_core
