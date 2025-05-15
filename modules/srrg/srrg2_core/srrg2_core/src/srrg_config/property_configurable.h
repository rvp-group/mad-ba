#pragma once
#include "configurable.h"
#include "srrg_property/property_identifiable.h"

namespace srrg2_core {

  template <typename ContainerType_>
  using PropertyConfigurable_ = PropertyIdentifiablePtr_<std::shared_ptr<ContainerType_>>;

  template <typename ContainerType_>
  using PropertyConfigurableNoOwnership_ = PropertyIdentifiablePtr_<std::weak_ptr<ContainerType_>>;
} // namespace srrg2_core
