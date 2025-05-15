#include "active_drawable.h"

namespace srrg2_core {

  void ActiveDrawable::draw() {
    if (! _need_redraw)
      return;
    for (auto& c: _canvases) {
      ViewerCanvasPtr sc=c.lock();
      if(sc)
        _drawImpl(sc);
    }
    this->_need_redraw=false;
  }

  std::list<ActiveDrawable::ViewerCanvasWeakPtr>::iterator
  ActiveDrawable::findCanvas(ViewerCanvasPtr canvas) {
    for (auto it=_canvases.begin(); it!=_canvases.end(); ++it) {
      ViewerCanvasPtr c = it->lock();
      if (c==canvas)
        return it;
    }
    return _canvases.end();
  }

  void ActiveDrawable::addCanvas(ViewerCanvasPtr canvas){
    auto it=findCanvas(canvas);
    if (it!=_canvases.end())
      return;
    _canvases.push_back(ViewerCanvasWeakPtr(canvas));
  }
  
  void ActiveDrawable::removeCanvas(ViewerCanvasPtr canvas){
    auto it=findCanvas(canvas);
    if (it==_canvases.end())
      return;
    _canvases.erase(it);
  }


} // namespace srrg2_core
