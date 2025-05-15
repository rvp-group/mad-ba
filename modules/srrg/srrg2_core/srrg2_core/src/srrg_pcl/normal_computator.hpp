
namespace srrg2_core {

  //! @brief specialization for spherical range image
  template <typename PointCloudType_, int idx_>
  void NormalComputatorSRI<PointCloudType_, idx_>::computeNormals(PointCloudType_& point_matrix_) {
    const size_t& rows = point_matrix_.rows();
    const size_t& cols = point_matrix_.cols();
    // from pix 0 we access right and down
    for (size_t r = 0; r < rows - 1; ++r) {
      for (size_t c = 0; c < cols - 1; ++c) {
        if (point_matrix_.at(r, c).status != POINT_STATUS::Valid)
          continue;
        // get coordinates
        const VectorType& p       = point_matrix_.at(r, c).template value<0>();
        const VectorType& p_right = point_matrix_.at(r, c + 1).template value<0>();
        const VectorType& p_down  = point_matrix_.at(r + 1, c).template value<0>();
        // get current spherical coordinates
        const auto theta       = atan2(p.y(), p.x());            // azimuth
        const auto phi         = atan2(p.z(), p.head(2).norm()); // elevation
        const auto rho         = p.norm();                       // range
        const auto theta_right = atan2(p_right.y(), p_right.x());
        const auto rho_right   = p_right.norm();
        const auto phi_down    = atan2(p_down.z(), p_down.head(2).norm());
        const auto rho_down    = p_down.norm();
        // get rotation matrix
        const MatrixType R = geometry3d::rotationZ(theta) * geometry3d::rotationY(phi);
        // computing img grad
        const auto ddth = (rho_right - rho) / (2. * (theta - theta_right));
        const auto ddph = (rho_down - rho) / (2. * (phi - phi_down));
        // sticking to vec
        VectorType del = VectorType(1., 1. / rho * cos(phi) * ddth, 1. / rho * ddph);
        // inserting normal
        VectorType& normal = point_matrix_.at(r, c).template value<idx_>();
        normal             = (-R * del).normalized();
        // std::cerr << " [ n:" << normal.transpose() << " r: " << r << " c: " << c << " th: " <<
        // theta
        //           << " phi: " << phi << " rho: " << rho << " theta r: " << theta_right
        //           << " rho r: " << rho_right << " phi d:" << phi_down << " rho d: " << rho_down
        //           << " ]\n";
      }
    }
  }

  //! @brief specialization for organized point clouds
  template <typename PointCloudType_, int idx_>
  void NormalComputator2DCrossProduct<PointCloudType_, idx_>::computeNormals(
    PointCloudType_& point_matrix_) {
    // std::cerr << "NormalComputator2DCrossProduct::computeNormals|normal idx
    // = " << idx_ << std::endl;

    const size_t& rows                 = point_matrix_.rows();
    const size_t& cols                 = point_matrix_.cols();
    const size_t& row_gap              = param_row_gap.value();
    const size_t& col_gap              = param_col_gap.value();
    const Scalar& squared_max_distance = param_squared_max_distance.value();

    for (size_t r = row_gap; r < (rows - row_gap); ++r) {
      const PointType* up_row   = point_matrix_.rowPtr(r - row_gap) + col_gap;
      const PointType* row      = point_matrix_.rowPtr(r) + col_gap;
      const PointType* down_row = point_matrix_.rowPtr(r + row_gap) + col_gap;
      PointType* normal_row     = point_matrix_.rowPtr(r) + col_gap;

      for (size_t c = col_gap; c < (cols - col_gap);
           ++c, ++up_row, ++row, ++down_row, ++normal_row) {
        const PointType&  point       = *row;
        if (point.status != POINT_STATUS::Valid)
          continue;

        const PointType& point_up    = *up_row;
        const PointType& point_down  = *down_row;
        const PointType& point_left  = *(row - col_gap);
        const PointType& point_right = *(row + col_gap);
        if (Dim > 2
            && (point_up.status       != POINT_STATUS::Valid
                || point_down.status  != POINT_STATUS::Valid
                || point_left.status  != POINT_STATUS::Valid
                || point_right.status != POINT_STATUS::Valid))
          continue;
        VectorType& normal      = normal_row->template value<idx_>();

        //auto dy = point_down - point_up;
        auto dy = point_down.template value<0>() - point_up.template value<0>();
        //auto dx = point_right - point_left;
        auto dx = point_right.template value<0>() - point_left.template value<0>();
        if (dx.squaredNorm() > squared_max_distance || dy.squaredNorm() > squared_max_distance)
          continue;
        normal = dy.cross(dx).normalized();
        if (normal.dot(point.template value<0>() ) > 0)
          normal *= -1.0;
      }
    }
  }

  // GIORGIO: unsafe, polish
  //! @brief specialization for organized point clouds
  template <typename PointCloudType_, int idx_>
  void NormalComputator2DSlidingWindow<PointCloudType_, idx_>::computeNormals(
    PointCloudType_& point_matrix_) {
    // std::cerr << "NormalComputator2DSlidingWindow::computeNormals|normal id
    // = " << idx_ << std::endl;
    const size_t& rows = point_matrix_.rows();
    const size_t& cols = point_matrix_.cols();

    Eigen::SelfAdjointEigenSolver<MatrixType> eigen_solver;

    int window_radius_rows = param_window_radius.value();
    int window_radius_cols = param_window_radius.value();

    if (Dim < 3)
      window_radius_rows = 0;

    for (size_t r = window_radius_rows; r < rows - window_radius_rows; ++r) {
      auto* dest_row = point_matrix_.rowPtr(r);
      for (size_t c = window_radius_cols; c < cols - window_radius_cols; ++c, ++dest_row) {
        auto& point = *dest_row;
        if (!(point.status == POINT_STATUS::Valid)) {
          continue;
        }

        const VectorType& coordinates = point.template value<0>();

        // ia shitty way of handling 2D and 3D points
        if (Dim > 2) {
          if (coordinates.z() == 0.f)
            continue;
        }

        point.template value<idx_>().setZero();

        VectorType mean     = VectorType::Zero();
        MatrixType variance = MatrixType::Zero();
        size_t num          = 0;

        const size_t r_up    = r - window_radius_rows;
        const size_t r_down  = r + window_radius_rows;
        const size_t c_left  = c - window_radius_cols;
        const size_t c_right = c + window_radius_cols;

        for (size_t rr = r_up; rr <= r_down; ++rr) {
          for (size_t cc = c_left; cc <= c_right; ++cc) {
            const auto& pp = point_matrix_.at(rr, cc);
            if (!(pp.status == POINT_STATUS::Valid)) {
              continue;
            }
            const VectorType& coordinates_pp = pp.template value<0>();

            mean.noalias() += coordinates_pp;
            variance.noalias() += coordinates_pp * coordinates_pp.transpose();
            ++num;
          }
        }

        if (!num)
          continue;

        const Scalar inv_num = 1.0 / (Scalar)(num);
        mean *= inv_num;

        variance *= inv_num;
        variance.noalias() -= mean * mean.transpose();
        eigen_solver.compute(variance);

        const auto& ev = eigen_solver.eigenvectors().col(0);
        if (coordinates.dot(ev) > 0)
          point.template value<idx_>() = -ev;
        else
          point.template value<idx_>() = ev;
      }
    }
  }

  template <typename PointCloudType_, int idx_>
  void NormalComputator1DSlidingWindow<PointCloudType_, idx_>::computeNormals(
    PointCloudType_& point_cloud_vector_) {
    // std::cerr << "NormalComputator1DSlidingWindow::computeNormals|normal id
    // = " << idx_ << std::endl; std::cerr <<
    // "NormalComputator1DSlidingWindow::computeNormals|normal pt dist = " <<
    // normal_point_distance << std::endl;

    Eigen::SelfAdjointEigenSolver<MatrixType> eigen_solver;
    const size_t& size                  = point_cloud_vector_.size();
    const Scalar& normal_point_distance = param_normal_point_distance.value();
    const int& normal_min_points        = param_normal_min_points.value();
    const Scalar& max_curvature         = param_max_curvature.value();

    //      std::cerr << "point cloud size = " << size << std::endl;

    // ia for all the points in the cloud (vector)
    for (size_t i = 0; i < size; ++i) {
      // std::cerr << "index = " << i << std::endl;

      PointType& point = point_cloud_vector_[i];
      if (point.status != Valid)
        continue;

      point.status                  = Invalid;
      const VectorType& coordinates = point.template value<0>();
      VectorType& normal            = point.template value<idx_>();
      normal.setZero();

      // ia create a sliding window based on the distance from the current
      // point
      size_t index_min = i, index_max = i;

      // ia compute the bottom of the window
      while (index_min > 0) {
        const auto& p1_coords = point_cloud_vector_[index_min].template value<0>();
        if ((p1_coords - coordinates).squaredNorm() >= normal_point_distance) {
          ++index_min;
          break;
        }
        --index_min;
      }

      // ia compute the top of the window
      while (index_max < size) {
        const auto& p1_coords = point_cloud_vector_[index_max].template value<0>();
        if ((p1_coords - coordinates).squaredNorm() >= normal_point_distance) {
          break;
        }
        ++index_max;
      }

      //        std::cerr << "index min = " << index_min << std::endl;
      //        std::cerr << "index max = " << index_max << std::endl;

      // ia if there aren't enough point -> normal is null
      int num_selected_points        = index_max - index_min;
      Scalar inv_num_selected_points = 1. / num_selected_points;
      if (num_selected_points < normal_min_points) {
        point.status = Invalid;
        continue;
      }
      // ia start accumulating points in the window
      VectorType mean     = VectorType::Zero();
      MatrixType variance = MatrixType::Zero();

      for (size_t ii = index_min; ii < index_max; ++ii) {
        const auto& pp_coords = point_cloud_vector_[ii].template value<0>();

        mean.noalias() += pp_coords;
        variance.noalias() += pp_coords * pp_coords.transpose();
      }

      // ia compute mean and variance
      mean *= inv_num_selected_points;
      variance *= inv_num_selected_points;
      variance.noalias() -= mean * mean.transpose();

      // ia compute the normal as the eigenvector relative to the lower
      // eigenvalue ia take the normal directed toward the camera
      eigen_solver.compute(variance);

      // check if the curvature is small enough
      Scalar curvature = eigen_solver.eigenvalues()(0) /
                         (eigen_solver.eigenvalues()(0) + eigen_solver.eigenvalues()(1));
      if (curvature > max_curvature) {
        continue;
      }

      const auto& ev = eigen_solver.eigenvectors().col(0);
      if (coordinates.dot(ev) > 0)
        normal = -ev;
      else
        normal = ev;
      point.status = Valid;
    }
  }

} // namespace srrg2_core
