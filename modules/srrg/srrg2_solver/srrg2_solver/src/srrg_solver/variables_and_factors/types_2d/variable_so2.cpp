#include "variable_so2.h"

//! include this: this contains all the implementations of the factors
//! that are hidden to the modules that do not need them to avoid excessive compilation times (EVIL)
#include "srrg_solver/solver_core/instance_macros.h"
#include "srrg_solver/solver_core/variable_impl.cpp"

namespace srrg2_solver {
  using namespace srrg2_core;

  INSTANTIATE(VariableSO2Right)
  INSTANTIATE(VariableSO2Left)

} // namespace srrg2_solver
