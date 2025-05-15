#pragma once
#include "viewer_context_base.h"

namespace srrg2_core {

  //! @brief base class that will be specialized in ViewerManagerShared and ViewerManagerNetwork
  //!        each of them will configurate the context in a different way and will do different
  //!        things when methods are called
  class ViewerManagerBase : public Configurable {
  public:
    using ViewerContextContainer = StringViewerContextBaseMap;

    PARAM(PropertyUnsignedInt, max_num_buffers, "maximum number of buffer in the loop", 5, 0);
    PARAM(PropertyUnsignedInt,
          buffer_size,
          "maximum size of each buffer",
          BUFFER_SIZE_1MEGABYTE,
          0);

    virtual ~ViewerManagerBase();

    //! @brief this creates a context with a specific name and returns a canvas.
    //!        the latter can be used by a processing module (e.g. ScanMatcher) to
    //!        draw something. The context associated with this canvas is unique.
    //!        if the context is alredy in use in the system, it returns that canvas
    virtual const ViewerCanvasPtr& getCanvas(const std::string& context_name_) = 0;

    //! @brief binds a viewport from a context_name
    virtual void bindViewport(ViewportBase* viewport_, const std::string& context_name_) = 0;

    //! @brief unbinds a viewport
    virtual void unbindViewport(ViewportBase* viewport_, const std::string& context_name_) = 0;

  protected:
    //! @brief pull of contexts - owned by this module
    ViewerContextContainer _contexts;
  };

  using ViewerManagerBasePtr = std::shared_ptr<ViewerManagerBase>;

} // namespace srrg2_core
