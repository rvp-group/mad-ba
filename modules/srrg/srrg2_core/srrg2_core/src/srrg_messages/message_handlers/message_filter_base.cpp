#include "message_filter_base.h"
namespace srrg2_core {

  void MessageFilterBase::reset() {
    if (param_source.value()) {
      return param_source->reset();
    }
  }

  MessageSourceBase* MessageFilterBase::getRootSource() {
    if (param_source.value()) {
      return param_source->getRootSource();
    }
    return nullptr;
  }
} // namespace srrg2_core
