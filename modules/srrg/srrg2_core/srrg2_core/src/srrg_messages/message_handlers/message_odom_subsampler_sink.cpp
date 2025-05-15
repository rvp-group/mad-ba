#include "message_odom_subsampler_sink.h"
#include "srrg_config/configurable.h"
#include "srrg_config/property_configurable.h"
#include "srrg_messages/message_handlers/message_sink_base.h"
#include "srrg_messages/messages/odometry_message.h"

namespace srrg2_core {

  bool MessageOdomSubsamplerSink::putMessage(BaseSensorMessagePtr msg_) {
    OdometryMessagePtr odom_msg = std::dynamic_pointer_cast<OdometryMessage>(msg_);
    if (!odom_msg || odom_msg->topic.value() != param_odom_topic.value()) {
      return propagateMessage(msg_);
    }

    const Isometry3f delta = _prev_pose.inverse() * odom_msg->pose.value();
    _prev_pose             = odom_msg->pose.value();
    const Eigen::AngleAxisf delta_r(delta.linear());
    _cum_translation += delta.translation().norm();
    _cum_rotation += fabs(delta_r.angle());

    if (_cum_rotation > param_rotation_min.value() ||
        _cum_translation > param_translation_min.value()) {
      _cum_translation = 0;
      _cum_rotation    = 0;
      return propagateMessage(odom_msg);
    }
    return false;
  }

  void MessageOdomSubsamplerSink::reset() {
    _cum_rotation    = 0;
    _cum_translation = 0;
    _prev_pose.setIdentity();
    MessageSinkBase::reset();
  }
} // namespace srrg2_core
