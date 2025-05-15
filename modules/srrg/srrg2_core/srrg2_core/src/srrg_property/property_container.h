#pragma once
#include "property.h"

namespace srrg2_core {

  // base configuration class
  // it stores a list of properties
  // and implements auto serialization
  using StringPropertyPtrMap       = std::map<std::string, PropertyBase*>;
  using StringPropertyUniquePtrMap = std::map<std::string, std::unique_ptr<PropertyBase>>;

  class PropertyIdentifiablePtrInterfaceBase;

  template <typename IdentBase_>
  class PropertyContainerDerived_;

  using PropertyContainerIdentifiable    = PropertyContainerDerived_<Identifiable>;
  using PropertyContainerIdentifiablePtr = std::shared_ptr<PropertyContainerIdentifiable>;

  class PropertyContainerBase {
    friend class PropertyBase;
    friend class PropertyContainerManager;

    friend class ConfigNode;
    friend class ConfigurableNodeManager;
    friend class edConfigurableNodeManager;

    friend struct PropertyIdentifiablePtrInterfaceBase;

  public:
    const std::string& name() const {
      return _name;
    }
    void setName(const std::string& name_) {
      _name = name_;
    }

    void serialize(ObjectData& data, IdContext& context);
    void deserialize(ObjectData& data, IdContext& context);
    virtual PropertyBase* property(const std::string& name);

    template <typename TargetType_>
    inline TargetType_* property(const std::string& name) {
      PropertyBase* p = property(name);
      if (!p) {
        return 0;
      }
      return dynamic_cast<TargetType_*>(p);
    }

    inline const StringPropertyPtrMap& properties() const {
      return _properties;
    }

    inline size_t size() const {
      return _properties.size();
    }

    virtual ~PropertyContainerBase();

  protected:
    void addProperty(PropertyBase* property);
    StringPropertyPtrMap _properties; //< all properties
    std::vector<PropertyIdentifiablePtrInterfaceBase*> _properties_identifiable;
    void
    getConnectedContainers(std::multimap<std::string, PropertyContainerIdentifiablePtr>& connected);
    void getReacheableContainers(std::set<PropertyContainerIdentifiablePtr>& reacheable);
    std::string _name = "";

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  template <typename IdentifiableType_>
  class PropertyContainerDerived_ : public IdentifiableType_, public PropertyContainerBase {
  public:
    using IdentifiableType = IdentifiableType_;

    void serialize(ObjectData& data, IdContext& context) override {
      IdentifiableType::serialize(data, context);
      PropertyContainerBase::serialize(data, context);
    }

    void deserialize(ObjectData& data, IdContext& context) override {
      IdentifiableType::deserialize(data, context);
      PropertyContainerBase::deserialize(data, context);
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  template <typename PropertyContainerBaseType_>
  class PropertyContainerDynamic_ : public PropertyContainerBaseType_ {
  public:
    using PropertyContainerBaseType = PropertyContainerBaseType_;

    virtual ~PropertyContainerDynamic_() {
      clear();
    }

    inline void clear() {
      while (!_dynamic_properties.empty()) {
        auto it_d = _dynamic_properties.begin();
        removeProperty(it_d->first);
      }
    }

    inline void addProperty(PropertyBase* p) {
      PropertyContainerBaseType_::addProperty(p);
      _dynamic_properties.insert(std::make_pair(p->name(), std::unique_ptr<PropertyBase>(p)));
    }

    inline void removeProperty(PropertyBase* p) {
      removeProperty(p->name());
    }

    inline void removeProperty(const std::string& p_name) {
      auto it_d = _dynamic_properties.find(p_name);
      if (it_d != _dynamic_properties.end()) {
        auto it_s = this->_properties.find(p_name);
        this->_properties.erase(it_s);
        _dynamic_properties.erase(it_d);
      }
    }

  protected:
    StringPropertyUniquePtrMap _dynamic_properties;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using PropertyContainerDynamic = PropertyContainerDynamic_<PropertyContainerBase>;
  using PropertyContainerBasePtr = std::shared_ptr<PropertyContainerBase>;

  using PropertyContainerIdentifiable = PropertyContainerDerived_<Identifiable>;
  using PropertyContainerDynamicIdentifiable =
    PropertyContainerDynamic_<PropertyContainerIdentifiable>;

  using PropertyContainerIdentifiablePtr       = std::shared_ptr<PropertyContainerIdentifiable>;
  using PropertyContainerIdentifiablePtrVector = std::vector<PropertyContainerIdentifiablePtr>;

  class PropertyContainerSerializable : public PropertyContainerBase, public Serializable {
  public:
    void serialize(ObjectData& data, IdContext& context) override {
      PropertyContainerBase::serialize(data, context);
    }
    void deserialize(ObjectData& data, IdContext& context) override {
      PropertyContainerBase::deserialize(data, context);
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using PropertyContainerDynamicSerializable =
    PropertyContainerDynamic_<PropertyContainerSerializable>;

  using PropertyContainerSerializablePtr       = std::shared_ptr<PropertyContainerSerializable>;
  using PropertyContainerSerializablePtrVector = std::vector<PropertyContainerSerializablePtr>;

} // namespace srrg2_core
