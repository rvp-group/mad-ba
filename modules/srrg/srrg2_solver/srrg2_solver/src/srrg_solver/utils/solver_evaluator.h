#pragma once
#include "srrg_solver/solver_core/factor_graph_view.h"

namespace srrg2_solver {

  struct SolverEvaluator {
    void setGroundTruth(FactorGraphInterface& gt_);
    void compute(FactorGraphInterface& view_);
    
    template <typename FactorType_>
    void align(FactorGraphInterface& view, bool adjust=false);

    void alignSE2(FactorGraphInterface& view, bool adjust=false);
    void alignSE3(FactorGraphInterface& view, bool adjust=false);
    void alignSim3(FactorGraphInterface& view, bool adjust=false);

    inline const float chiCutoff() const {return _chi_cutoff;}
    inline void setChiCutoff(const float& chi_cutoff_) {_chi_cutoff=chi_cutoff_;}
  protected:
    float _chi_cutoff=std::numeric_limits<float>::max();
    FactorGraphView _eval_view;
    FactorGraphInterface* _gt=nullptr;
  };
}
