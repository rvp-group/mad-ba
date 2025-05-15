#include "instances.h"

namespace srrg2_core {

  void matchable_registerTypes() {
    // ia TODO check this, should work
    BOSS_REGISTER_BLOB(MatchablefVectorData);
    BOSS_REGISTER_BLOB(MatchabledVectorData);
    BOSS_REGISTER_BLOB(VisualMatchablefVectorData);
    BOSS_REGISTER_BLOB(VisualMatchabledVectorData);
  }

} // namespace srrg2_core
