#include "solver_action_draw.h"
#include <srrg_solver/solver_core/solver.h>
#include <thread>

namespace srrg2_solver {

  using namespace std; // evvaffanculo

  SolverActionDraw::SolverActionDraw() {
    param_event.setValue(Solver::SolverEvent::ComputeEnd);
  }

  SolverActionDraw::~SolverActionDraw() {
  }

  void SolverActionDraw::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!_solver_ptr) {
      throw std::runtime_error("SolverActionDraw::_drawImpl|invalid solver");
    }
    srrg2_solver::FactorGraphInterface& graph = _solver_ptr->graph();

    for (const auto& v_tuple : graph.variables()) {
      v_tuple.second->_drawImpl(canvas_);
    }

    for (const auto& f : graph.factors()) {
      f.second->_drawImpl(canvas_);
    }
    canvas_->flush();
  }

  void SolverActionDraw::doAction() {
    this->_need_redraw = true;
    std::cerr << "need redraw!!!" << this->needRedraw() << endl;
    ActiveDrawable::draw();
    Preemptible::preemptGlobal();
    SystemUsageCounter::tic();
    double toc = SystemUsageCounter::toc();
    while (toc < .5f) {
      toc = SystemUsageCounter::toc();
    }
  }
} // namespace srrg2_solver
