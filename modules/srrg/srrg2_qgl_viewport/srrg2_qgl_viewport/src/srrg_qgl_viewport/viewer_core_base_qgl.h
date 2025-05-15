#pragma once
#include "qgl_viewport.h"
#include <srrg_viewer/viewer_core_base.h>
#include <srrg_viewer/viewer_manager_shared.h>

namespace srrg2_qgl_viewport {

  //! @brief intermediate class done by Mircolosi, to avoid overriding
  //!        the dtor
  class ViewerCoreBaseQGL : public srrg2_core::ViewerCoreBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! @brief disabilitate empty ctor. You must sprecify the arguments
    ViewerCoreBaseQGL() = delete;

    //! @brief creates a viewer and setups all basic parameters
    //! [IMPORTANT] qapplication must be created as STATIC in the main thread and passed as pointer
    //! here to avoid strange memory phoenomena.
    ViewerCoreBaseQGL(int argc_,
                      char** argv_,
                      QApplication* qapp_,
                      const size_t& buffer_size_          = BUFFER_SIZE_50MEGABYTE,
                      const size_t& buffers_num_          = 3,
                      const size_t& opengl_frame_time_ms_ = 25);

    //! @brief  destroyes the viewport and releases resources
    virtual ~ViewerCoreBaseQGL();

  protected:
    //! @brief Qt window host (malicious contraption, safest to allocate on stack)
    QApplication* _qapp = nullptr;

    //! @brief min frame duration for opengl (caps the FPS to avoid CPU melting)
    const size_t _opengl_ms;

  protected:
    //! @brief create viewports for the retrieved canvases
    virtual void _bindAllViewports() override;

  };

} // namespace srrg2_qgl_viewport
