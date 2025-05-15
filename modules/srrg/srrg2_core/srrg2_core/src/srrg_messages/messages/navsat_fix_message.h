#pragma once

#include "base_sensor_message.h"
#include <srrg_property/property_eigen.h>

namespace srrg2_core {
  class NavsatFixMessage : public BaseSensorMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! @brief confortable enums
    enum FixStatus : int8_t { NO_FIX = -1, FIX = 0, SBAS_FIX = 1, GBAS_FIX = 2 };
    enum ServiceType : uint16_t { GPS = 1, GLONASS = 2, COMPASS = 4, GALILEO = 8 };
    enum CovarianceType : uint8_t { UNKNOWN = 0, APPROXIMATED = 1, DIAGONAL_KNOWN = 2, KNOWN = 3 };

    //! @brief default ctor
    NavsatFixMessage(const std::string& topic_    = "",
                     const std::string& frame_id_ = "",
                     const int& seq_              = -1,
                     const double& timestamp_     = -1.f);

    //! @brief gets the 3d coordinates from the current reading using a non spherical model of the
    //! earth: link [ https://stackoverflow.com/a/8982005 ]
    void getPosition(Vector3f& position_xyz_);

    //! @brief status of the service
    //! {-1: no fix; 0 = unaugmented fix; 1: fix w/ satellite-based augmentation, 2: fix w/
    //! ground-based augmentation}
    PropertyInt status;

    //! @brief service type
    //! {1: gps; 2: glonass; 4: compass (also beidou); 8: galileo}
    PropertyUnsignedInt service;

    //! @brief covariance can be fully known, partially known (only diagonal),
    //! approximated or not known at all
    PropertyUInt8 covariance_type;

    //! @brief confortable bool that tells me if the altitude is set in this message
    PropertyBool has_altitude;

    //! @brief latitude[degrees].Positive is north of equator; negative is south.
    PropertyDouble latitude;

    //! @brief longitude[degrees].Positive is east of prime meridian; negative is west.
    PropertyDouble longitude;

    //! @brief altitude [m]. Positive is above the WGS 84 ellipsoid
    //! std::numeric_limits<double>::max means that there is no altitude information
    PropertyDouble altitude;

    //! @brief row major covariance type of the fix - row major reading from ros
    PropertyEigen_<Matrix3f> covariance;
  };

  using NavsatFixMessagePtr = std::shared_ptr<NavsatFixMessage>;
} // namespace srrg2_core
