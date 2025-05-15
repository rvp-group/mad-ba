#pragma once
#include <stdint.h>
// ia deg2rad and so on
#include "srrg_geometry/geometry_defs.h"

namespace srrg2_core {
  namespace srrg2_lidar3d_utils {
    enum LIDAR_TYPE {
      Invalid = 0x00,
      VLP_16  = 0x10,
      HDL_32  = 0x11,
      HDL_64E = 0x12,
      OS1_16  = 0x20,
      OS1_64  = 0x21
    };

    //! @brief this represents the intrinsic parameters of a specific 3D lidar.
    //!        those parameter depend on the manifacturer and so thei are fixed.
    //!        to know this parameters please check the sensor's datasheet
    template <uint8_t lidar_type_>
    struct Lidar3DSensor_ {
      //! @brief lidar model
      static constexpr uint8_t LidarType = lidar_type_;

      //! @brief ctor automatically sets those parameters depending on the sensor type
      Lidar3DSensor_() {
        switch (LidarType) {
          case LIDAR_TYPE::Invalid:
            throw std::runtime_error("Lidar3DSensor_::Lidar3DSensor_|ERROR, invalid sensor type");
          case LIDAR_TYPE::VLP_16:
            vertical_resolution      = 16;
            vertical_fov_lower_bound = srrg2_core::deg2Rad(-15.f);
            vertical_fov_upper_bound = srrg2_core::deg2Rad(15.f);
            break;
          case LIDAR_TYPE::HDL_32:
            vertical_resolution      = 32;
            vertical_fov_lower_bound = srrg2_core::deg2Rad(-30.67f);
            vertical_fov_upper_bound = srrg2_core::deg2Rad(10.67f);
            break;
          case LIDAR_TYPE::HDL_64E:
            vertical_resolution      = 64;
            vertical_fov_lower_bound = srrg2_core::deg2Rad(-24.9f);
            vertical_fov_upper_bound = srrg2_core::deg2Rad(2.f);
            break;
          case LIDAR_TYPE::OS1_16:
            vertical_resolution      = 16;
            vertical_fov_lower_bound = srrg2_core::deg2Rad(-16.6f);
            vertical_fov_upper_bound = srrg2_core::deg2Rad(16.6f);
            break;
          case LIDAR_TYPE::OS1_64:
            vertical_resolution      = 64;
            vertical_fov_lower_bound = srrg2_core::deg2Rad(-16.6f);
            vertical_fov_upper_bound = srrg2_core::deg2Rad(16.6f);
            break;
          default:
            throw std::runtime_error("Lidar3DSensor_::Lidar3DSensor_|ERROR, unknown sensor type");
            break;
        }
      }

      //! @brief constant accessor
      inline const uint16_t& verticalResolution() const {
        assert(vertical_resolution != 0 &&
               "Lidar3DSensor_::vericalResolution|ERROR, invalid value");
        return vertical_resolution;
      }

      //! @brief constant accessor
      inline const float& verticalFOVLower() const {
        assert(vertical_fov_lower_bound != 0.f &&
               "Lidar3DSensor_::verticalFOVLower|ERROR, invalid value");
        return vertical_fov_lower_bound;
      }

      //! @brief constant accessor
      inline const float& verticalFOVUpper() const {
        assert(vertical_fov_upper_bound != 0.f &&
               "Lidar3DSensor_::verticalFOVUpper|ERROR, invalid value");
        return vertical_fov_upper_bound;
      }

    protected:
      //! @brief number of beams
      uint16_t vertical_resolution = 0;
      //! @brief vertical FOV - lower bound
      float vertical_fov_lower_bound = 0.f;
      //! @brief vertical FOV - upper bound
      float vertical_fov_upper_bound = 0.f;
    };

    //! @brief velodyne 3d lidars
    using Lidar3DSensorVelodyneVLP16  = Lidar3DSensor_<LIDAR_TYPE::VLP_16>;
    using Lidar3DSensorVelodyneHDL32  = Lidar3DSensor_<LIDAR_TYPE::HDL_32>;
    using Lidar3DSensorVelodyneHDL64E = Lidar3DSensor_<LIDAR_TYPE::HDL_64E>;
    //! @brief ouster 3d lidars
    using Lidar3DSensorOusterOS1_16 = Lidar3DSensor_<LIDAR_TYPE::OS1_16>;
    using Lidar3DSensorOusterOS1_64 = Lidar3DSensor_<LIDAR_TYPE::OS1_64>;
  } // namespace srrg2_lidar3d_utils
} // namespace srrg2_core
