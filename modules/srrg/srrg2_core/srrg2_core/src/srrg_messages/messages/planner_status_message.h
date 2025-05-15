#pragma once
#include "base_sensor_message.h"

namespace srrg2_core {

  class PlannerStatusMessage: public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PlannerStatusMessage(const std::string& topic_ = "",
                         const std::string& frame_id_ = "",
                         int seq_ = -1,
                         const double& timestamp_ = -1);

    PropertyString status;
    PropertyInt    a_star_status;
    PropertyInt    path_type;
    PropertyFloat  cost_to_global_goal;
    PropertyFloat  distance_to_global_goal;
    PropertyFloat  distance_to_local_goal;
  };

  typedef std::shared_ptr<PlannerStatusMessage> PlannerStatusMessagePtr;

} /* namespace srrg2_core */
