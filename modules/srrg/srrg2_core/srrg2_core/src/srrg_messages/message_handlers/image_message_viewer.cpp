#include "image_message_viewer.h"
#include "srrg_messages/messages/image_message.h"

namespace srrg2_core {
  using namespace std;

  bool ImageMessageViewer::putMessage(BaseSensorMessagePtr msg_) {
    _need_redraw=false;
    _images.resize(param_image_topics.size());
    for (size_t i=0; i<param_image_topics.size(); ++i) {
      if (msg_->topic.value()==param_image_topics.value(i)) {
        ImageMessagePtr img=std::dynamic_pointer_cast<ImageMessage>(msg_);
        if (! img) {
          continue;
        }
        if (img->image()) {
          img->image()->toCv(_images[i]);
          _need_redraw=true;
        }
      }
    }
    draw();
    return propagateMessage(msg_);
  }

  void ImageMessageViewer::_drawImpl(ViewerCanvasPtr canvas_) const {
    for (const auto& img: _images) {
      if (! img.empty())
        canvas_->putImage(img);
    }
    canvas_->flush();
  }


  void ImageMessageViewer::reset() {
    std::cerr << className() << "|reset" << std::endl;
    for (size_t i = 0; i < param_push_sinks.size(); ++i) {
      MessageSinkBasePtr sink = param_push_sinks.value(i);
      if (!sink) {
        continue;
      }
      sink->reset();
    }
  }


} // namespace srrg2_core
