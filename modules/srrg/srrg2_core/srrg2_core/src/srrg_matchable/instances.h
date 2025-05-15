#pragma once
#include <iostream>

#include "matchable.h"
#include "srrg_property/vector_data.h"
#include "visual_matchable.h"

namespace srrg2_core {

  using MatchablefVectorData       = VectorData_<MatchablefVector>;
  using MatchabledVectorData       = VectorData_<MatchabledVector>;
  using VisualMatchablefVectorData = VectorData_<VisualMatchablefVector>;
  using VisualMatchabledVectorData = VectorData_<VisualMatchabledVector>;

  //! @brief serialization and instanciation of the types
  void matchable_registerTypes() __attribute__((constructor));

} // namespace srrg2_core
