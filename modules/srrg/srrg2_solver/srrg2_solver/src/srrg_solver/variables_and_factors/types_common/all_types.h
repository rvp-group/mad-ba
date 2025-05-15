#pragma once
#include "variable_matrix.h"
#include "variable_matrix_ad.h"
#include "variable_vector.h"
#include "variable_vector_ad.h"

namespace srrg2_solver {
  class VariableVector2 : public VariableVector_<2> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void _drawImpl(ViewerCanvasPtr canvas_) const override;
  };
  using VariablePoint2 = VariableVector2;

  class VariableVector3 : public VariableVector_<3> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void _drawImpl(ViewerCanvasPtr canvas_) const override;
  };
  using VariablePoint3  = VariableVector3;
  using VariableVector4 = VariableVector_<4>;
  using VariableVector5 = VariableVector_<5>;
  using VariableVector6 = VariableVector_<6>;
  using VariableVector7 = VariableVector_<7>;
  using VariableVector8 = VariableVector_<8>;

  using VariableVector2AD = VariableVectorAD_<VariableVector2>;
  using VariablePoint2AD  = VariableVector2AD;

  using VariableVector3AD = VariableVectorAD_<VariableVector3>;
  using VariablePoint3AD  = VariableVector3AD;

  using VariableVector4AD = VariableVectorAD_<VariableVector4>;
  using VariableVector5AD = VariableVectorAD_<VariableVector5>;
  using VariableVector6AD = VariableVectorAD_<VariableVector6>;
  using VariableVector7AD = VariableVectorAD_<VariableVector7>;
  using VariableVector8AD = VariableVectorAD_<VariableVector8>;

  using VariableMatrix3_4   = VariableMatrix_<3, 4>;
  using VariableMatrix3_4AD = VariableMatrixAD_<VariableMatrix3_4>;

} // namespace srrg2_solver
