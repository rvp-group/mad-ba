#pragma once
#include <srrg_geometry/geometry3d.h>

#include "lidar_3d_utils.h"
#include "spacing.h"

namespace srrg2_core {
  namespace srrg2_lidar3d_utils {

    //! @brief projection based on hash map. depending on the type of lidar and on the size of the
    //! image you want, you initialize a hashing table and then you do the computation. for each 3d
    //! point you will get the value uv in the "camera" space
    template <LIDAR_TYPE lidar3d_type_>
    class Lidar3DLookupProjector_ {
    public:
      using ThisType                        = Lidar3DLookupProjector_<lidar3d_type_>;
      using Lidar3DSensorType               = Lidar3DSensor_<lidar3d_type_>;
      static constexpr LIDAR_TYPE LidarType = lidar3d_type_;

      //! @brief object life
      Lidar3DLookupProjector_();
      virtual ~Lidar3DLookupProjector_();

    protected:
      //! @brief clean-up internally allocated things
      inline void _reset() {
        if (_hSpacing) {
          delete _hSpacing;
        }

        if (_vSpacing) {
          delete _vSpacing;
        }

        if (_xUnitSphere) {
          delete[] _xUnitSphere;
        }

        if (_yUnitSphere) {
          delete[] _yUnitSphere;
        }

        if (_zUnitSphere) {
          delete[] _zUnitSphere;
        }
      }

      //! @brief initialize all the thing to perform the hashing based projection (the unit spheres
      //! and the spacing)
      void _init(const float& horiz_start_angle_,
                 const float& horiz_stop_angle_,
                 const size_t& num_columns_);

      //! @brief given a point on the spherical image, get the u v coords
      inline bool _getImageIndexYP(const float& yaw,
                                   const float& pitch,
                                   size_t& hor_index,
                                   size_t& vert_index) const {
        assert(_hSpacing && _vSpacing &&
               "Lidar3DLookupProjector_::_getImageIndexYP|ERROR, invalid spacings");
        return (_hSpacing->getImageIndex(yaw, hor_index) &&
                _vSpacing->getImageIndex(pitch, vert_index));
      }

    protected:
      //! @brief specific lidar model
      Lidar3DSensor_<LidarType> _lidar;

      //! @brief horizontal and vertiacal fovs
      float _h_opening_angle;
      float _v_opening_angle;

      //! @brief projection size
      size_t _image_width;
      size_t _image_height;

      //! @brief spacing functions to to the hash-based projection
      Spacing* _hSpacing = nullptr;
      Spacing* _vSpacing = nullptr;

      //! @brief unit sphere vectors
      float* _xUnitSphere = nullptr;
      float* _yUnitSphere = nullptr;
      float* _zUnitSphere = nullptr;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  } // namespace srrg2_lidar3d_utils
} // namespace srrg2_core

#include "lidar_3d_lookup_projector.hpp"
