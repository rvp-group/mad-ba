#pragma once
#include "solver_action_base.h"
#include <srrg_viewer/active_drawable.h>

namespace srrg2_solver {

  //! @brief action that draws a factor graph on the canvas
  class SolverActionDraw : public srrg2_solver::SolverActionBase, public srrg2_core::ActiveDrawable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using BaseType = srrg2_solver::SolverActionBase;
    SolverActionDraw();
    virtual ~SolverActionDraw();

    void doAction();
  protected:
    void _drawImpl(srrg2_core::ViewerCanvasPtr gl_canvas_) const override;
  };

  using SolverActionDrawPtr = std::shared_ptr<SolverActionDraw>;
} // namespace srrg2_solver_gui
