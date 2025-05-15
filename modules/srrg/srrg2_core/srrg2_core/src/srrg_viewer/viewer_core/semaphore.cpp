#include "semaphore.h"

namespace srrg2_core {

  Semaphore::Semaphore(const uint64_t& value_) {
    _count = value_;
  }

  Semaphore::~Semaphore() {}

} //ia end namespace
