#pragma once
#include "viewer_manager_base.h"
#include <atomic>
namespace srrg2_core {

  //! @brief simple class that gives you access to all the viewer types
  //!        it will be specialized in multiple classes depending on
  //!        the viewport (QGL, Vulkan, RViz, ..) and the comm mean type (SharedMem, ROS, ..)
  class ViewerCoreBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! @brief useful typedefs
    struct CanvasViews {
      size_t num_views;
      ViewportBaseVector viewports;

      CanvasViews(const size_t& num_views_) {
        num_views = num_views_;
        viewports.resize(num_views_);
        for (size_t v = 0; v < num_views; ++v)
          viewports[v] = nullptr;
      }
    };
    using StringCanvasViewsMap = std::unordered_map<std::string, CanvasViews*>;

    ViewerCoreBase() = delete;
    //! @brief ctor. requires the shell arguments and some basic parameters
    ViewerCoreBase(const int& argc_,
                   char** argv_,
                   const size_t& buffer_size_ = BUFFER_SIZE_10MEGABYTE,
                   const size_t& buffer_num_  = 3);
    //! @brief checkout everything
    virtual ~ViewerCoreBase();

    //! @brief starts the viewer server.
    virtual void startViewerServer() = 0;

    //! @brief returns the inner canvas
    const srrg2_core::ViewerCanvasPtr& getCanvas(const std::string& canvas_name_);

    //! @brief detach a single canvas
    bool detachCanvas(const std::string& canvas_name_);

    //! @brief inline getter
    static const bool isRunning() {
      return _run;
    }

    //! @brief stops the engine
    static void stop() {
      _run = false;
    }

  protected:
    //! @brief viewer manager. gives the canvas and binds your viewport
    //!        this specifies the comm mean
    ViewerManagerBase* _viewer_manager = 0;

    StringCanvasViewsMap _canvas_viewport_map;

    //! @brief vieweport set - all viewports are here
    ViewportBaseSet _viewports;

    //! @brief memory setup
    const size_t _buffer_size; //! @brief buffer size
    const size_t _num_buffers; //! @brief buffer number

    //! @brief tells the viewer to stop. This can be triggered by a signal.
    static std::atomic<bool> _run;

    //! @brief required to start thing (like QApp and ROS)
    int _argc;
    char** _argv = nullptr;

  protected:
    //! @brief create vieweports for each specific canvas
    //!        and binds them to the relative canvas
    virtual void _bindAllViewports() = 0;

    //! @brief unbinds all the viewports
    void _unbindAllViewports();

    //! @brief updates all the active viewports - no matter their type
    void _updateActiveViewports();
  };

  using ViewerCoreBasePtr = std::shared_ptr<ViewerCoreBase>;

} // namespace srrg2_core
