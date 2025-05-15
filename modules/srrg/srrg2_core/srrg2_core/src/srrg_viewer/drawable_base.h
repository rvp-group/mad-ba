#pragma once
#include "viewer_canvas.h"

namespace srrg2_core {

  //! @brief base derivable class that represent an object that could be "drawn"
  //!        using OpenGL and OpenCV
  class DrawableBase {
  public:
    inline void draw(ViewerCanvasPtr gl_canvas_) {
      if (!_need_redraw) {
        return;
      }

      _drawImpl(gl_canvas_);
      _need_redraw = false;
    }
    // ia now disabled, everything goes trough the same canvas - TODO rethink
    // virtual void draw(cv::Mat& cv_canvas_) const {
    //   // ds optional overriding
    // }
    virtual ~DrawableBase();
    inline bool needRedraw() const {
      return _need_redraw;
    }
    inline void setNeedRedraw(bool need_redraw) {
      _need_redraw = need_redraw;
    }

  protected:
    virtual void _drawImpl(ViewerCanvasPtr gl_canvas_) const {
      // ds optional overriding
    }
    bool _need_redraw = true;
  };

  using DrawableBasePtr = std::shared_ptr<DrawableBase>;

} // namespace srrg2_core
