#include "instances.h"
#include "messages/camera_info_message.h"
#include "messages/cmd_vel_message.h"
#include "messages/grid_map_message.h"
#include "messages/image_message.h"
#include "messages/imu_message.h"
#include "messages/joints_message.h"
#include "messages/laser_message.h"
#include "messages/navsat_fix_message.h"
#include "messages/odometry_message.h"
#include "messages/path_message.h"
#include "messages/point_cloud2_message.h"
#include "messages/point_stamped_message.h"
#include "messages/pose_array_message.h"
#include "messages/pose_message.h"
#include "messages/pose_stamped_message.h"
#include "messages/pose_with_covariance_stamped_message.h"
#include "messages/range_message.h"
#include "messages/ticks_message.h"
#include "messages/transform_events_message.h"
#include "messages/twist_stamped_message.h"
#include "messages/planner_status_message.h"

#include "message_handlers/message_file_source.h"
#include "message_handlers/message_odom_subsampler_source.h"
#include "message_handlers/message_pack.h"
#include "message_handlers/message_sorted_source.h"
#include "message_handlers/message_source_platform.h"
#include "message_handlers/message_synchronized_source.h"

#include "message_handlers/message_file_sink.h"
#include "message_handlers/message_odom_subsampler_sink.h"
#include "message_handlers/message_platform_listener_sink.h"
#include "message_handlers/message_selector_sink.h"
#include "message_handlers/message_sorted_sink.h"
#include "message_handlers/message_synchronized_sink.h"
#include "message_handlers/image_message_viewer.h"


namespace srrg2_core {

  void messages_registerTypes() {
    BOSS_REGISTER_CLASS(PointCloud2DataBLOBReference); // bdc, no idea if it should be CLASS or BLOB

    BOSS_REGISTER_CLASS(CameraInfoMessage);
    BOSS_REGISTER_CLASS(ImageMessage);
    BOSS_REGISTER_CLASS(IMUMessage);
    BOSS_REGISTER_CLASS(LaserMessage);
    BOSS_REGISTER_CLASS(OdometryMessage);
    BOSS_REGISTER_CLASS(PointStampedMessage);
    BOSS_REGISTER_CLASS(RangeMessage);
    BOSS_REGISTER_CLASS(TicksMessage);
    BOSS_REGISTER_CLASS(TransformEventsMessage);
    BOSS_REGISTER_CLASS(TwistStampedMessage);
    BOSS_REGISTER_CLASS(PointCloud2Message);
    BOSS_REGISTER_CLASS(JointsMessage);
    BOSS_REGISTER_CLASS(NavsatFixMessage)
    BOSS_REGISTER_CLASS(PoseMessage);
    BOSS_REGISTER_CLASS(PoseArrayMessage);
    BOSS_REGISTER_CLASS(PoseStampedMessage);
    BOSS_REGISTER_CLASS(PoseWithCovarianceStampedMessage);
    BOSS_REGISTER_CLASS(PathMessage);
    BOSS_REGISTER_CLASS(CmdVelMessage);
    BOSS_REGISTER_CLASS(GridMapMessage);
    BOSS_REGISTER_CLASS(PlannerStatusMessage);

    BOSS_REGISTER_CLASS(MessageFileSource);
    //BOSS_REGISTER_CLASS(MessageSortedSource);
    //BOSS_REGISTER_CLASS(MessageSynchronizedSource);
    //BOSS_REGISTER_CLASS(MessageSourcePlatform);
    BOSS_REGISTER_CLASS(MessageOdomSubsamplerSource);

    BOSS_REGISTER_CLASS(MessagePack);
    BOSS_REGISTER_CLASS(MessageSelectorSink);
    BOSS_REGISTER_CLASS(MessageSinkBase);
    BOSS_REGISTER_CLASS(MessageFileSink);
    BOSS_REGISTER_CLASS(MessageSortedSink);
    BOSS_REGISTER_CLASS(MessageSynchronizedSink);
    BOSS_REGISTER_CLASS(MessageOdomSubsamplerSink);
    BOSS_REGISTER_CLASS(MessagePlatformListenerSink);

    BOSS_REGISTER_CLASS(ImageMessageViewer);
  }
} // namespace srrg2_core
