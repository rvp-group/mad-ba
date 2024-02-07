#include "surfel.h"

namespace structure_refinement {

Surfel::Surfel() {}
Surfel::~Surfel() {}
bool Surfel::putMessage(srrg2_core::BaseSensorMessagePtr msg_) {
    return true;
}

}  // namespace structure_refinement