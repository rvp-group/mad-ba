#include "planner_status_message.h"

namespace srrg2_core {

  PlannerStatusMessage::PlannerStatusMessage(const std::string& topic_,
                                             const std::string& frame_id_,
                                             int seq_,
                                             const double& timestamp_):
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY(status, "idle"),
    SETUP_PROPERTY(a_star_status, 0),
    SETUP_PROPERTY(path_type, 0),
    SETUP_PROPERTY(cost_to_global_goal, 0.f),
    SETUP_PROPERTY(distance_to_global_goal, 0.f),
    SETUP_PROPERTY(distance_to_local_goal, 0.f)
  {
    
  }

}
