#pragma once
#include "base_sensor_message.h"
#include "srrg_image/image_data.h"
#include "srrg_property/property_eigen.h"
#include "srrg_property/property_serializable.h"

namespace srrg2_core {

  class ImageMessage : public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImageMessage(const std::string& topic_    = "",
                 const std::string& frame_id_ = "",
                 const int& seq_              = -1,
                 const double& timestamp_     = -1.0);

    PropertySerializable_<ImageDataBLOBReference> image_data;

    inline BaseImage* image() {
      if (image_data.value().get()) {
        return image_data.value().get()->image();
      }
      return 0;
    }

    inline void setImage(BaseImage* img) {
      ImageData* idata = new ImageData;
      idata->setImage(img);
      image_data.value().set(idata);
    }

    virtual void serialize(ObjectData& odata, IdContext& context);

    // ds image dimension access without need to load image from disk
    // ds TODO spread use of this and update THIS constructor
    PropertyUnsignedInt image_rows;
    PropertyUnsignedInt image_cols;

  protected:
    bool _topic_name_changed = true;
    const std::string& _nameAttribute() const;
    mutable std::string _name_attribute;
  };

  using ImageMessagePtr = std::shared_ptr<ImageMessage>;

} // namespace srrg2_core
