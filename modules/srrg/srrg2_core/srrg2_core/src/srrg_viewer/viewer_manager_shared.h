#pragma once
#include "viewer_context_shared.h"
#include "viewer_core/buffer_sink_shared.h"
#include "viewer_core/buffer_source_shared.h"
#include "viewer_manager_base.h"

namespace srrg2_core {

  class ViewerManagerShared : public ViewerManagerBase {
  public:
    using ThisType = ViewerManagerShared;
    using BaseType = ViewerManagerBase;

    ViewerManagerShared();
    virtual ~ViewerManagerShared();

    //! @brief this creates a context with a specific name and returns a canvas.
    //!        the latter can be used by a processing module (e.g. ScanMatcher) to
    //!        draw something. The context associated with this canvas is unique.
    const ViewerCanvasPtr& getCanvas(const std::string& context_name_) override;

    //! @brief binds a viewport from a context_name
    void bindViewport(ViewportBase* viewport_, const std::string& context_name_) override;

    //! @breif unbinds a vieport from a context
    void unbindViewport(ViewportBase* viewport_, const std::string& context_name_) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using ViewerManagerSharedPtr = std::shared_ptr<ViewerManagerShared>;

} // namespace srrg2_core
