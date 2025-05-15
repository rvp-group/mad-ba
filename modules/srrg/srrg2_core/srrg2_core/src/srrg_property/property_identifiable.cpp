#include "property_identifiable.h"

namespace srrg2_core {

  bool PropertyIdentifiablePtrVectorInterface::pushBack(Identifiable* id) {
    if (!canAssign(id))
      return false;
    resize(size() + 1);
    return assign(size() - 1, id);
  }

  bool PropertyIdentifiablePtrVectorInterface::pushBack(std::shared_ptr<Identifiable> id) {
    if (!canAssign(id)) {
      std::cerr << "PropertyIdentifiablePtrVectorInterface::pushBack|warning, cannot assign object "
                   "with ID ["
                << id << "]\n";
      return false;
    }
    resize(size() + 1);
    return assign(size() - 1, id);
  }

} // namespace srrg2_core
