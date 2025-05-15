#pragma once
#include "viewer_core_base_qgl.h"

namespace srrg2_qgl_viewport {

  //! @brief lazy aggragation of all the things needed for a
  //!        shared-qgl-based viewer.
  //!        Things You need to do:
  //!        1. contruct a stub.
  //!        2. request a canvas an give it to you Processing Module
  //!        3. start your processing thread (ALWAYS IN ANOTHER THREAD)
  //!        4. call the start method.
  //!        NOTE THAT: All the params of the viewer manager (like
  //!          - buffer size
  //!          - buffer number
  //!          - opengl max fps)
  //!        ARE ALL SET IN CONSTRUCTION and cannot be changed
  class ViewerCoreSharedQGL : public ViewerCoreBaseQGL {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! @brief disabilitate empty ctor. You must sprecify the arguments
    ViewerCoreSharedQGL() = delete;

    //! @brief creates a viewer and setups all basic parameters
    ViewerCoreSharedQGL(int argc_,
                        char** argv_,
                        QApplication* qapp_,
                        const size_t& buffer_size_          = BUFFER_SIZE_50MEGABYTE,
                        const size_t& buffers_num_          = 3,
                        const size_t& opengl_frame_time_ms_ = 25,
                        const bool install_signal_handler_  = false);

    //! @brief  destroyes the viewport and releases resources
    virtual ~ViewerCoreSharedQGL() {
    }

    //! @brief starts the viewer server.
    void startViewerServer() override;

    //! @brief modified version of getCanvas, on which you specify
    //!        the number of views of that canvas.
    const srrg2_core::ViewerCanvasPtr& getCanvasMultiview(const std::string& canvas_name_,
                                                          const size_t& num_views_);

    //! @brief base SIGINT handler
    static void sigIntHandler(int __attribute__((unused))) {
      std::flush(std::cerr);
      std::cerr << FG_BRED("ViewerCoreSharedQGL::sigIntHandler|terminating") << std::endl;
      std::flush(std::cerr);
      ViewerCoreBase::stop();
    }

  protected:
    const bool _install_sighandler;
  };

} // namespace srrg2_qgl_viewport
