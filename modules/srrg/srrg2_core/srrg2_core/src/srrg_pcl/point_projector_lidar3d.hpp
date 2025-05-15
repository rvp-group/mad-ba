namespace srrg2_core {

  template <srrg2_lidar3d_utils::LIDAR_TYPE lidar3d_type_, typename PointCloudType_>
  int PointProjectorLidar3D_<lidar3d_type_, PointCloudType_>::compute(
    typename BaseType::TargetMatrixType& target_,
    IteratorType begin_,
    IteratorType end_) {
    // ia some caching
    const size_t& matrix_rows = this->_lidar.verticalResolution();
    const size_t& matrix_cols = this->param_num_columns.value();

    // ia if something has been changed then fuckin reinitialize everything
    if (_initialization_required) {
#ifndef NDEBUG
      std::cerr << "PointProjectorLidar3D_::compute|reinitializing lookup projection\n";
#endif
      this->_init(param_horizontal_start_angle.value(),
                  param_horizontal_end_angle.value(),
                  param_num_columns.value());
      _initialization_required = false;
    }

#ifndef NDEBUG
    std::cerr << "PointProjectorLidar3D_::compute|resized target matrix to [ " << matrix_rows
              << " x " << matrix_cols << " ]\n";
#endif

    // ia allocate memory
    target_.resize(matrix_rows, matrix_cols);
    typename BaseType::TargetMatrixEntry entry;
    entry.source_it = end_;
    entry.projected.setZero();
    entry.projected.status = srrg2_core::Invalid;
    entry.transformed.setZero();
    entry.transformed.status = srrg2_core::Invalid;
    target_.fill(entry);

    int num_good = 0;
    int index    = 0;

    // bdc iterate over cloud
    for (auto it = begin_; it != end_; ++it, ++index) {
      // bdc get point reference
      const PointType& p = *it;
      if (p.status != srrg2_core::Valid) {
        continue;
      }

      // we transform the point
      const PointType tp =
        p.template transform<srrg2_core::TRANSFORM_CLASS::Isometry, IsometryType>(
          this->_world_in_camera);

      const float xrs   = p.coordinates().x() * p.coordinates().x();
      const float yrs   = p.coordinates().y() * p.coordinates().y();
      const float zrs   = p.coordinates().z() * p.coordinates().z();
      const float yaw   = atan2(p.coordinates().y(),
                              p.coordinates().x()); // atan2 returns in [-pi..+pi]
      const float pitch = atan2(-p.coordinates().z(),
                                sqrt(xrs + yrs)); // returns in [-pi..+pi]
      const float range = sqrt(xrs + yrs + zrs);
      if (range < this->param_range_min.value() || range > this->param_range_max.value()) {
        continue;
      }

      // bdc get row and col of projected point
      size_t r = 0, c = 0;
      if (!this->_getImageIndexYP(yaw, pitch, c, r)) {
        continue;
      }
      // depth_buffer
      typename BaseType::TargetMatrixEntry& target_cell = target_.at(r, c);
      if (range > target_cell.depth && target_cell.source_idx) {
        continue;
      }

      // point was not good and now it becomes good
      if (target_cell.source_idx < 0) {
        ++num_good;
      }

      target_cell.source_idx  = index;
      target_cell.source_it   = it;
      target_cell.transformed = tp;
      target_cell.depth       = range;
      // ia TODO, workaroud, we place here the (u,v,1) coordinates of the projected point
      target_cell.projected.template value<0>()[0] = c;
      target_cell.projected.template value<0>()[1] = r;
      target_cell.projected.template value<0>()[2] = 1.f;

    } // for points
    return num_good;
  }

} // namespace srrg2_core
