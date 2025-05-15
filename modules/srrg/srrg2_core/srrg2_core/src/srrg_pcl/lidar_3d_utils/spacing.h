#pragma once
#include <iostream>
#include <memory>
#include <vector>

namespace srrg2_core {
  namespace srrg2_lidar3d_utils {

    // Spacing for velodyne beams. Can be regular/uniform or not
    class Spacing {
    public:
      virtual ~Spacing()                                              = default;
      virtual bool getImageIndex(float angle_rad_, size_t& idx_)      = 0;
      virtual bool getAngleRAD(const size_t& idx_, float& angle_rad_) = 0;
    };

    class RegularSpacing : public Spacing {
    public:
      // ia disable empty ctor
      RegularSpacing() = delete;
      RegularSpacing(float start_angle_rad_,
                     float stop_angle_rad_,
                     const size_t& number_of_pixels_);

      // ia default dtor
      ~RegularSpacing() = default;

      // ia overriding functions
      bool getImageIndex(float angle_rad_, size_t& idx_) final;
      bool getAngleRAD(const size_t& idx_, float& angle_rad_) final;

    protected:
      float _start_angle;  // is always in range [0,2pi)
      float _range_rad;    // positive value
      float _range_sign;   // +/-1, direction of rangeRAD
      float _pix_size_rad; // can be positive or negative (same sign as rangeSign)

    public:
      friend std::ostream& operator<<(std::ostream& stream_, const RegularSpacing& spacing_);
    };

    class IrregularSpacing : public Spacing {
    public:
      // ia disable empty ctor
      IrregularSpacing() = delete;

      // ia only ctor with parameter is allowed
      template <class InputIteratorT>
      IrregularSpacing(InputIteratorT angle_rad_begin_,
                       InputIteratorT angle_rad_end_,
                       const size_t& number_of_pixels_) {
        while (angle_rad_begin_ != angle_rad_end_) {
          _v_angles_rad.push_back(*angle_rad_begin_);
          ++angle_rad_begin_;
        }
        _avg_pix_size_rad =
          (_v_angles_rad.back() - _v_angles_rad.front()) / (float) number_of_pixels_;
        _pix_size_sign = (_avg_pix_size_rad < 0.0) ? -1.0 : 1.0;
      }

      // ia default dtor
      virtual ~IrregularSpacing() = default;

      // ia overriding functions
      bool getImageIndex(float angle_rad_, size_t& idx_) final;
      bool getAngleRAD(const size_t& idx_, float& angle_rad_) final;

    protected:
      std::vector<float> _v_angles_rad;
      float _avg_pix_size_rad; // can be positive or negative
      float _pix_size_sign;    // +/-1, sign of avgPixSizeRAD
    };

    using SpacingPtr      = std::shared_ptr<Spacing>;
    using SpacingConstPtr = std::shared_ptr<const Spacing>;

  } // namespace srrg2_lidar3d_utils
} // namespace srrg2_core
