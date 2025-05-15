#pragma once
#include "drawable_base.h"

namespace srrg2_core {

  //! @brief base derivable class that represent an object that could be "drawn"
  //!        using OpenGL and OpenCV
  class ActiveDrawable: public DrawableBase {
  public:
    void addCanvas(ViewerCanvasPtr canvas);
    void removeCanvas(ViewerCanvasPtr canvas);
    void draw();
  protected:
    using ViewerCanvasWeakPtr = std::weak_ptr<ViewerCanvas>;
    std::list<ViewerCanvasWeakPtr> _canvases;
    std::list<ViewerCanvasWeakPtr>::iterator findCanvas(ViewerCanvasPtr canvas);
  };

  using ActiveDrawablePtr = std::shared_ptr<ActiveDrawable>;

} // namespace srrg2_core
