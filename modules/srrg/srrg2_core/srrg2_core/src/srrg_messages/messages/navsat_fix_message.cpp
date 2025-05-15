#include "navsat_fix_message.h"

namespace srrg2_core {

  NavsatFixMessage::NavsatFixMessage(const std::string& topic_,
                                     const std::string& frame_id_,
                                     const int& seq_,
                                     const double& timestamp_) :
    BaseSensorMessage(topic_, frame_id_, seq_, timestamp_),
    SETUP_PROPERTY(status, FixStatus::NO_FIX),
    SETUP_PROPERTY(service, ServiceType::GPS),
    SETUP_PROPERTY(covariance_type, CovarianceType::UNKNOWN),
    SETUP_PROPERTY(has_altitude, false),
    SETUP_PROPERTY(latitude, 0.f),
    SETUP_PROPERTY(longitude, 0.f),
    SETUP_PROPERTY(altitude, std::numeric_limits<double>::max()),
    SETUP_PROPERTY(covariance, Matrix3f::Identity()) {
  }

  void NavsatFixMessage::getPosition(Vector3f& position_xyz_) {
    position_xyz_.setZero();

    // ia earth constants
    const double rad = 6378137.0f;         // ia earth radius
    const double f   = 1.0f / 298.257224f; // ia flattening parameter

    // ia computing stuff according to [ http://mathforum.org/library/drmath/view/51832.html ]
    const double cos_lat = std::cos(latitude.value() * M_PI / 180.0f);
    const double sin_lat = std::sin(latitude.value() * M_PI / 180.0f);
    const double cos_lon = std::cos(longitude.value() * M_PI / 180.0f);
    const double sin_lon = std::sin(longitude.value() * M_PI / 180.0f);
    const double C =
      1.0f / std::sqrt(cos_lat * cos_lat + (1.f - f) * (1.f - f) * sin_lat * sin_lat);
    const double S = (1.f - f) * (1.f - f) * C;

    // ia check if we have also altitude informations
    double h = 0.0;
    if (has_altitude.value()) {
      h = altitude.value();
    }

    position_xyz_.x() = (rad * C + h) * cos_lat * cos_lon;
    position_xyz_.y() = (rad * C + h) * cos_lat * sin_lon;
    position_xyz_.z() = (rad * S + h) * sin_lat;
  }

} // namespace srrg2_core
