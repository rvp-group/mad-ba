#include "image_message.h"

namespace srrg2_core {

  ImageMessage::ImageMessage(const std::string& topic_,
                             const std::string& frame_id_,
                             const int& seq_,
                             const double& timestamp_) :
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY_NV(image_data),
    SETUP_PROPERTY(image_rows, 0),
    SETUP_PROPERTY(image_cols, 0) {
    this->topic.bindChangedFlag(&_topic_name_changed);
  }

  const std::string& ImageMessage::_nameAttribute() const {
    std::string fn = topic.value();
    std::replace(fn.begin(), fn.end(), '/', '.');
    if (fn[0] == '.') {
      fn = fn.substr(1);
    }
    char buf[1024];
    sprintf(buf, "%s", fn.c_str());
    _name_attribute = buf;

    return _name_attribute;
  }

  void ImageMessage::serialize(ObjectData& odata, IdContext& context) {
    if (_topic_name_changed && image_data.value().get()) {
      image_data.value().setNameAttribute(_nameAttribute());
      _topic_name_changed = false;
    }
    BaseSensorMessage::serialize(odata, context);
  }

} // namespace srrg2_core
