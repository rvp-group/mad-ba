#include "spacing.h"
#include <cfloat>
#include <cmath>
#include <iostream>
#include <limits>

namespace srrg2_core {
  namespace srrg2_lidar3d_utils {

    RegularSpacing::RegularSpacing(float start_angle_rad_,
                                   float stop_angle_rad_,
                                   const size_t& number_of_pixels_) {
      // normalize starting angle to [0,2pi)
      while (start_angle_rad_ >= 2 * M_PI) {
        start_angle_rad_ -= 2 * M_PI;
        stop_angle_rad_ -= 2 * M_PI;
      }
      while (start_angle_rad_ < 0.0) {
        start_angle_rad_ += 2 * M_PI;
        stop_angle_rad_ += 2 * M_PI;
      }
      _start_angle          = start_angle_rad_;
      const float range_rad = stop_angle_rad_ - _start_angle;
      _range_rad            = fabs(range_rad);
      _range_sign           = (range_rad < 0) ? -1.0 : 1.0;
      _pix_size_rad         = _range_rad / (float) number_of_pixels_;
    }

    bool RegularSpacing::getImageIndex(float angle_rad_, size_t& idx) {
      angle_rad_ -= _start_angle;
      angle_rad_ *= _range_sign;
      while (angle_rad_ < 0)
        angle_rad_ += 2 * M_PI;
      while (angle_rad_ >= 2 * M_PI)
        angle_rad_ -= 2 * M_PI;
      if (angle_rad_ > _range_rad)
        return false;
      idx = angle_rad_ / _pix_size_rad - .5f;
      return true;
    }

    bool RegularSpacing::getAngleRAD(const size_t& idx_, float& angle_rad_) {
      angle_rad_ =
        _start_angle + _range_sign * _pix_size_rad * ((float) idx_ + 0.5); // center of pixel
      return true;
    }

    bool IrregularSpacing::getImageIndex(float angle_rad_, size_t& idx_) {
      if (angle_rad_ * _pix_size_sign <
          (_v_angles_rad.front() - _avg_pix_size_rad) * _pix_size_sign)
        return false;
      if (angle_rad_ * _pix_size_sign > (_v_angles_rad.back() + _avg_pix_size_rad) * _pix_size_sign)
        return false;
      idx_         = 0;
      float oldDst = DBL_MAX;
      while ((int) idx_ < (int) _v_angles_rad.size()) {
        float cDst = fabs(angle_rad_ - _v_angles_rad[idx_]);
        if (cDst < oldDst) {
          oldDst = cDst;
          ++idx_;
        } else
          break;
      }
      idx_--;
      return true;
    }

    bool IrregularSpacing::getAngleRAD(const size_t& idx_, float& angle_rad_) {
      angle_rad_ = _v_angles_rad[idx_];
      return true;
    }

    // ia operator overload for easy printing
    std::ostream& operator<<(std::ostream& stream_,
                             const srrg2_core::srrg2_lidar3d_utils::RegularSpacing& spacing_) {
      stream_ << "RegularSpacing parameters:" << std::endl;
      stream_ << "\t start_angle   : " << spacing_._start_angle << std::endl;
      stream_ << "\t range_RAD     : " << spacing_._range_rad << std::endl;
      stream_ << "\t range_sign    : " << spacing_._range_sign << std::endl;
      stream_ << "\t pix_size_RAD_ : " << spacing_._pix_size_rad << std::endl;
      return stream_;
    }

  } // namespace srrg2_lidar3d_utils
} // namespace srrg2_core