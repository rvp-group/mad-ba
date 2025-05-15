namespace srrg2_core {
  namespace srrg2_lidar3d_utils {

    template <LIDAR_TYPE lidar3d_type_>
    Lidar3DLookupProjector_<lidar3d_type_>::Lidar3DLookupProjector_() {
      _h_opening_angle = 0.f;
      _v_opening_angle = 0.f;

      _image_width  = 0;
      _image_height = 0;
    }

    template <LIDAR_TYPE lidar3d_type_>
    Lidar3DLookupProjector_<lidar3d_type_>::~Lidar3DLookupProjector_() {
#ifndef NDEBUG
      std::cerr << "Lidar3DLookupProjector_::~Lidar3DLookupProjector_|destroying" << std::endl;
#endif
      _reset();
    }

    template <LIDAR_TYPE lidar3d_type_>
    void Lidar3DLookupProjector_<lidar3d_type_>::_init(const float& horiz_start_angle_,
                                                       const float& horiz_stop_angle_,
                                                       const size_t& num_columns_) {
      // ia initialize the spacings based on the current configurations the spacings
      if (horiz_stop_angle_ >= horiz_start_angle_) {
        throw std::runtime_error("Lidar3DLookupProjector_::_init|invalid start/stop angles");
      }

      // ia check whether things are already there and delete everything
      _reset();

      _image_width  = num_columns_;
      _image_height = _lidar.verticalResolution();

      // ia Geiger sickness
      const float vert_start_angle = -_lidar.verticalFOVUpper();
      const float vert_stop_angle  = -_lidar.verticalFOVLower();
      _h_opening_angle             = horiz_start_angle_ - horiz_stop_angle_;
      _v_opening_angle             = vert_start_angle - vert_stop_angle;
      _hSpacing = new RegularSpacing(horiz_start_angle_, horiz_stop_angle_, _image_width);
      _vSpacing = new RegularSpacing(vert_start_angle, vert_stop_angle, _image_height);
      assert(_hSpacing && _vSpacing && "Lidar3DProjectorBase_::_init|ERROR, bad things happened");

      // ia talk only in debug
#ifndef NDEBUG
      std::cerr << "Lidar3DProjectorBase_::_init|spacings\n";
      std::cerr << *(dynamic_cast<RegularSpacing*>(_hSpacing)) << std::endl;
      std::cerr << *(dynamic_cast<RegularSpacing*>(_vSpacing)) << std::endl;
      std::cerr << "Lidar3DProjectorBase_::_init|image height = " << _image_height << std::endl;
      std::cerr << "Lidar3DProjectorBase_::_init|image width = " << _image_width << std::endl;
#endif

      // ia initialize the unit spheres
      _xUnitSphere = new float[_image_width * _image_height];
      _yUnitSphere = new float[_image_width * _image_height];
      _zUnitSphere = new float[_image_width * _image_height];

      srrg2_core::Vector3f unity(1.0, 0.0, 0.0);
      srrg2_core::Vector3f pt;
      srrg2_core::Matrix3f rot = srrg2_core::Matrix3f::Identity();
      float yaw, pitch;
      for (size_t row = 0; row < _image_height; ++row) {
        for (size_t col = 0; col < _image_width; ++col) {
          _hSpacing->getAngleRAD(col, yaw);
          _vSpacing->getAngleRAD(row, pitch);
          rot               = srrg2_core::geometry3d::yawPitch2Rot(yaw, pitch);
          pt                = rot * unity;
          const size_t idx  = row * _image_width + col;
          _xUnitSphere[idx] = pt[0];
          _yUnitSphere[idx] = pt[1];
          _zUnitSphere[idx] = pt[2];
        }
      }
    }
  } // namespace srrg2_lidar3d_utils
} // namespace srrg2_core
