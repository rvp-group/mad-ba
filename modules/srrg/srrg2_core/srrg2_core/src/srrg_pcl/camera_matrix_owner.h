#pragma once

#include "srrg_property/property.h"

#include "srrg_config/configurable.h"
#include "srrg_geometry/geometry_defs.h"

namespace srrg2_core {

  template <int GeometryDim_>
  class CameraMatrixOwner_ : public Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static constexpr int GeometryDim = GeometryDim_;
    using IsometryType               = Isometry_<float, GeometryDim>;
    virtual ~CameraMatrixOwner_() {
    }

    using CameraMatrixType = Eigen::Matrix<float, GeometryDim, GeometryDim>;

    inline void setCameraMatrix(const CameraMatrixType& camera_matrix_) {
      _camera_matrix = camera_matrix_;
    }

    inline const CameraMatrixType& cameraMatrix() const {
      return _camera_matrix;
    }

    inline void setCameraPose(const IsometryType& cam_in_world) {
      _camera_in_world = cam_in_world;
      _world_in_camera = cam_in_world.inverse();
    }

    inline const IsometryType& cameraPose() const {
      return _camera_in_world;
    }

    inline void setCanvasRows(const size_t& canvas_rows_) {
      _canvas_rows = canvas_rows_;
    }

    inline const size_t& canvasRows() const {
      return _canvas_rows;
    }

    inline void setCanvasCols(const size_t& canvas_cols_) {
      _canvas_cols = canvas_cols_;
    }

    inline const size_t& canvasCols() const {
      return _canvas_cols;
    }

  protected:
    CameraMatrixType _camera_matrix = CameraMatrixType::Identity();
    IsometryType _camera_in_world   = IsometryType::Identity();
    IsometryType _world_in_camera   = IsometryType::Identity();

    size_t _canvas_rows = 0;
    size_t _canvas_cols = 0;

    bool _config_changed = true;
  };
  using CameraMatrixOwner2D    = CameraMatrixOwner_<2>;
  using CameraMatrixOwner3D    = CameraMatrixOwner_<3>;
  using CameraMatrixOwner2DPtr = std::shared_ptr<CameraMatrixOwner2D>;
  using CameraMatrixOwner3DPtr = std::shared_ptr<CameraMatrixOwner3D>;

} // namespace srrg2_core
