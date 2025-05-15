#pragma once
#include <Eigen/Core>
#include <srrg_pcl/point_types.h>


namespace srrg2_solver {
  using namespace srrg2_core;

  class SE3EpipolarBAFactorBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using InformationMatrixType=Eigen::Matrix<float, 1, 1>;
    inline void setFixed(const Point3f& fixed_) {_fixed=fixed_;}
    inline void setMoving(const Point3f& moving_) {_moving=moving_;}
    inline const InformationMatrixType& informationMatrix() const {return _information_matrix;}
    inline void setInformationMatrix(const InformationMatrixType& info)  {_information_matrix = info;}

  protected:
    Point3f _fixed, _moving;
    InformationMatrixType _information_matrix=InformationMatrixType::Identity();
  };
}
