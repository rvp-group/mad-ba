#include <cassert>
#include <iostream>

#include "property.h"
#include "property_container.h"

#include "srrg_boss/deserializer.h"
#include "srrg_boss/serializer.h"

#include "property_container_manager.h"
#include "property_identifiable.h"

namespace srrg2_core {

  using namespace std;

  void PropertyContainerBase::serialize(ObjectData& data, IdContext& context) {
    if (_name.length()) {
      data.setString("name", _name);
    }
    for (auto it = _properties.begin(); it != _properties.end(); ++it) {
      (it->second->serialize(data, context));
    }
  }

  void PropertyContainerBase::deserialize(ObjectData& data, IdContext& context) {
    if (data.getField("name")) {
      _name = data.getString("name");
    }
    for (auto it = _properties.begin(); it != _properties.end(); ++it) {
      ValueData* v = data.getField(it->first);
      if (v) {
        (it->second->deserialize(data, context));
      }
    }
  }

  void PropertyContainerBase::addProperty(PropertyBase* property) {
    // ds commented as per holy decree of the madonna c.c.c.n.i.c.
    //    assert(_properties.find(property->name()) == _properties.end() &&
    //           "a parameter with same name already exists");
    _properties.insert(std::make_pair(property->name(), property));
  }

  PropertyBase* PropertyContainerBase::property(const std::string& name) {
    auto it = _properties.find(name);
    if (it != _properties.end()) {
      return it->second;
    }
    return nullptr;
  }

  void PropertyContainerBase::getConnectedContainers(
    std::multimap<std::string, PropertyContainerIdentifiablePtr>& connected) {
    for (PropertyIdentifiablePtrInterfaceBase* it : _properties_identifiable) {
      PropertyIdentifiablePtrInterface* single =
        dynamic_cast<PropertyIdentifiablePtrInterface*>(it);
      if (single) {
        PropertyContainerIdentifiablePtr cont =
          dynamic_pointer_cast<PropertyContainerIdentifiable>(single->getSharedPtr());
        if (!cont)
          continue;
        connected.insert(std::make_pair(single->name(), cont));
        continue;
      }
      PropertyIdentifiablePtrVectorInterface* vec =
        dynamic_cast<PropertyIdentifiablePtrVectorInterface*>(it);
      if (vec) {
        for (size_t i = 0; i < vec->size(); ++i) {
          PropertyContainerIdentifiablePtr cont =
            dynamic_pointer_cast<PropertyContainerIdentifiable>(vec->getSharedPtr(i));
          if (!cont)
            continue;
          connected.insert(std::make_pair(vec->name(), cont));
        }
        continue;
      }
    }
  }

  void PropertyContainerBase::getReacheableContainers(
    std::set<PropertyContainerIdentifiablePtr>& reacheable) {
    for (PropertyIdentifiablePtrInterfaceBase* it : _properties_identifiable) {
      PropertyIdentifiablePtrInterface* single =
        dynamic_cast<PropertyIdentifiablePtrInterface*>(it);
      if (single) {
        PropertyContainerIdentifiablePtr cont =
          dynamic_pointer_cast<PropertyContainerIdentifiable>(single->getSharedPtr());
        if (!cont)
          continue;
        if (reacheable.count(cont))
          continue;
        reacheable.insert(cont);
        if (cont) {
          cont->getReacheableContainers(reacheable);
        }
        continue;
      }
      PropertyIdentifiablePtrVectorInterface* vec =
        dynamic_cast<PropertyIdentifiablePtrVectorInterface*>(it);
      if (vec) {
        for (size_t i = 0; i < vec->size(); ++i) {
          IdentifiablePtr item = vec->getSharedPtr(i);
          PropertyContainerIdentifiablePtr cont =
            std::dynamic_pointer_cast<PropertyContainerIdentifiable>(vec->getSharedPtr(i));
          if (!cont)
            continue;
          if (reacheable.count(cont))
            continue;
          reacheable.insert(cont);
          cont->getReacheableContainers(reacheable);
        }
      }
    }
  }

  PropertyContainerBase::~PropertyContainerBase() {
  }

} // namespace srrg2_core
