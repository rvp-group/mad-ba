#include "message_odom_subsampler_source.h"
#include "srrg_config/configurable.h"
#include "srrg_config/property_configurable.h"
#include "srrg_messages/message_handlers/message_source_base.h"
#include "srrg_messages/messages/odometry_message.h"

namespace srrg2_core {

  BaseSensorMessagePtr MessageOdomSubsamplerSource::getMessage() {
    MessageSourceBasePtr src = param_source.value();
    if (!src) {
      throw std::runtime_error("no source set");
    }

    // ds if both are zero this subsampler will drop all messages (unwanted I guess)
    assert(param_rotation_min.value() > 0 || param_translation_min.value() > 0);

    // ds check if we can subsample
    OdometryMessagePtr odom_msg = nullptr;
    while (_cum_rotation < param_rotation_min.value() &&
           _cum_translation < param_translation_min.value()) {
      BaseSensorMessagePtr msg = src->getMessage();
      if (!msg) {
        return nullptr;
      }
      odom_msg = std::dynamic_pointer_cast<OdometryMessage>(msg);
      if (!odom_msg) {
        return msg;
      }

      if (odom_msg->topic.value() != param_odom_topic.value()) {
        return odom_msg;
      }

      const Isometry3f delta = _prev_pose.inverse() * odom_msg->pose.value();
      _prev_pose             = odom_msg->pose.value();
      const Eigen::AngleAxisf delta_r(delta.linear());
      _cum_translation += delta.translation().norm();
      _cum_rotation += fabs(delta_r.angle());
    }

    _cum_translation = 0;
    _cum_rotation    = 0;
    return odom_msg;
  }

} // namespace srrg2_core
