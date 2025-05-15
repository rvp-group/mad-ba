#pragma once
#include "message_filter_base.h"
#include "srrg_data_structures/platform.h"
#include "srrg_messages/messages/base_sensor_message.h"

namespace srrg2_core {

  class MessageSourcePlatform : public MessageFilterBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PARAM_VECTOR(PropertyVector_<std::string>,
                 tf_topics,
                 "TF topics - each one is a separate transform tree",
                 0);

    MessageSourcePlatform();
    //! @brief ctor/dtor
    virtual ~MessageSourcePlatform() = default;

    //! @brief binds each TF topic to a transform tree,
    //!        allocates data structures and so on.
    void bindTfTopics();

    //! @brief gets a message and updates the platform.
    //!        returns the message read.
    BaseSensorMessagePtr getMessage() override;

    //! @brief get a platform from its TF topic name
    //!        returns 0 if not found.
    inline PlatformPtr platform(const std::string& tf_name_) const {
      auto it = _platforms_map.find(tf_name_);
      if (it != _platforms_map.end()) {
        return it->second;
      }
      return nullptr;
    }

  protected:
    StringPlatformPtrMap _platforms_map;
    bool _tf_topics_binded = false;
  };

  using MessageSourcePlatformPtr = std::shared_ptr<MessageSourcePlatform>;

} // namespace srrg2_core
