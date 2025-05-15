namespace srrg2_core {

  template <srrg2_lidar3d_utils::LIDAR_TYPE lidar3d_type_, typename DestPointCloudType_>
  template <srrg2_core::PointUnprojectorMode payload_offset_idx,
            typename SrcChannelType,
            typename... SrcChannelsRestType>
  int PointUnprojectorLidar3D_<lidar3d_type_, DestPointCloudType_>::computeMatrix(
    DestMatrixType& dest,
    const SrcChannelType& src,
    const SrcChannelsRestType&... rest) {
    // ia if something has been changed then fuckin reinitialize everything
    if (_initialization_required) {
#ifndef NDEBUG
      std::cerr << "PointUnprojectorLidar3D_::compute|reinitializing lookup projection\n";
#endif
      this->_init(param_horizontal_start_angle.value(),
                  param_horizontal_end_angle.value(),
                  param_num_columns.value());
      _initialization_required = false;
    }

    dest.resize(src.rows(), src.cols());

    // ia caching things
    const size_t& lidar_projector_rows = this->_lidar.verticalResolution();
    const size_t& lidar_projector_cols = this->_image_width;

    if (lidar_projector_cols != src.cols() || lidar_projector_rows != src.rows()) {
      throw std::runtime_error("PointUnprojectorLidar3D_::computeMatrix|projection size mismatch");
    }

    int num_valid = 0;
    for (size_t r = 0; r < lidar_projector_rows; ++r) {
      for (size_t c = 0; c < lidar_projector_cols; ++c) {
        const auto& src_pt     = src.at(r, c);
        DestPointType& dest_pt = dest.at(r, c);

        if (src_pt < this->param_range_min.value()) {
          dest_pt.status = srrg2_core::Invalid;
          continue;
        }

        dest_pt.status = srrg2_core::Valid;
        // bdc actual unprojection w/ spherical lookup tables
        const size_t idx          = r * this->_image_width + c;
        dest_pt.coordinates().x() = src_pt * this->_xUnitSphere[idx];
        dest_pt.coordinates().y() = src_pt * this->_yUnitSphere[idx];
        dest_pt.coordinates().z() = src_pt * this->_zUnitSphere[idx];

        dest_pt.template transformInPlace<0, srrg2_core::TRANSFORM_CLASS::Isometry, IsometryType>(
          this->_camera_in_world);

        this->template _copyFields<payload_offset_idx, const SrcChannelsRestType&...>(
          dest_pt, r, c, rest...);
        ++num_valid;
      }
    }

    return num_valid;
  }

  template <srrg2_lidar3d_utils::LIDAR_TYPE lidar3d_type_, typename DestPointCloudType_>
  template <srrg2_core::PointUnprojectorMode payload_offset_idx,
            typename OutputIteratorType,
            typename SrcChannelType,
            typename... SrcChannelsRestType>
  int PointUnprojectorLidar3D_<lidar3d_type_, DestPointCloudType_>::compute(
    OutputIteratorType dest,
    const SrcChannelType& src,
    const SrcChannelsRestType&... rest) {
    // ia if something has been changed then fuckin reinitialize everything
    if (_initialization_required) {
#ifndef NDEBUG
      std::cerr << "PointUnprojectorLidar3D_::compute|reinitializing lookup projection\n";
#endif
      this->_init(param_horizontal_start_angle.value(),
                  param_horizontal_end_angle.value(),
                  param_num_columns.value());
      _initialization_required = false;
    }

    int num_valid = 0;
    for (size_t r = 0; r < src.rows(); ++r) {
      for (size_t c = 0; c < src.cols(); ++c) {
        const auto& src_pt = src.at(r, c);
        DestPointType dest_pt;
        if (src_pt < this->param_range_min.value()) {
          dest_pt.status = srrg2_core::Invalid;
          continue;
        }
        dest_pt.status            = srrg2_core::Valid;
        const size_t idx          = r * this->_image_width + c;
        dest_pt.coordinates().x() = src_pt * this->_xUnitSphere[idx];
        dest_pt.coordinates().y() = src_pt * this->_yUnitSphere[idx];
        dest_pt.coordinates().z() = src_pt * this->_zUnitSphere[idx];
        dest_pt.template transformInPlace<0, srrg2_core::TRANSFORM_CLASS::Isometry, IsometryType>(
          this->_camera_in_world);
        this->template _copyFields<payload_offset_idx, const SrcChannelsRestType&...>(
          dest_pt, r, c, rest...);
        *dest = dest_pt;
        ++dest;
        ++num_valid;
      }
    }
    return num_valid;
  }

  template <srrg2_lidar3d_utils::LIDAR_TYPE lidar3d_type_, typename DestPointCloudType_>
  template <srrg2_core::PointUnprojectorMode payload_offset_idx, typename PointCloudType_>
  size_t PointUnprojectorLidar3D_<lidar3d_type_, DestPointCloudType_>::compute(
    const PointCloudType_& points_image_coordinates_depth_,
    PointCloudType_& points_in_camera_frame_) {
    // ia if something has been changed then fuckin reinitialize everything
    if (_initialization_required) {
#ifndef NDEBUG
      std::cerr << "PointUnprojectorLidar3D_::compute|reinitializing lookup projection\n";
#endif
      this->_init(param_horizontal_start_angle.value(),
                  param_horizontal_end_angle.value(),
                  param_num_columns.value());
      _initialization_required = false;
    }

    throw std::runtime_error("PointUnprojectorLidar3D_<"
                             ">::compute | implement this particular "
                             "unprojection if you want and/or need");
    return 0;
  }

} // namespace srrg2_core
