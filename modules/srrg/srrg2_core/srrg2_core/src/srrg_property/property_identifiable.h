#pragma once
#include "srrg_data_structures/pointer_traits.h"

#include "property.h"
#include "property_container.h"
#include "property_vector.h"
namespace srrg2_core {

  class PropertyIdentifiablePtrInterfaceBase {
  public:
    PropertyIdentifiablePtrInterfaceBase(PropertyContainerBase* container_) {
      if (container_) {
        container_->_properties_identifiable.push_back(this);
      }
    }

    virtual bool canAssign(Identifiable* id)                 = 0;
    virtual bool canAssign(std::shared_ptr<Identifiable> id) = 0;
    virtual ~PropertyIdentifiablePtrInterfaceBase() {
    }
    virtual const std::vector<std::string>& assignableTypes() const = 0;
    virtual std::vector<std::string>& assignableTypes()             = 0;
    virtual const std::string& name() const                         = 0;
  };

  class PropertyIdentifiablePtrInterface : public PropertyIdentifiablePtrInterfaceBase {
  public:
    PropertyIdentifiablePtrInterface(PropertyContainerBase* container_) :
      PropertyIdentifiablePtrInterfaceBase(container_) {
    }

    virtual bool assign(Identifiable* id)                 = 0;
    virtual bool assign(std::shared_ptr<Identifiable> id) = 0;
    virtual Identifiable* getRawPtr()                     = 0;
    virtual IdentifiablePtr getSharedPtr()                = 0;
    virtual ~PropertyIdentifiablePtrInterface() {
    }
  };

  class PropertyIdentifiablePtrVectorInterface : public PropertyIdentifiablePtrInterfaceBase {
  public:
    PropertyIdentifiablePtrVectorInterface(PropertyContainerBase* container_) :
      PropertyIdentifiablePtrInterfaceBase(container_) {
    }
    virtual bool assign(size_t idx, Identifiable* id)                 = 0;
    virtual bool assign(size_t idx, std::shared_ptr<Identifiable> id) = 0;

    template <typename T>
    bool assign(std::vector<T> idv) {
      for (T& val : idv) {
        if (!canAssign(val))
          return false;
      }
      resize(idv.size());
      for (size_t i = 0; i < idv.size(); ++i) {
        assign(i, idv[i]);
      }
      return true;
    }

    virtual Identifiable* getRawPtr(size_t idx)      = 0;
    virtual IdentifiablePtr getSharedPtr(size_t idx) = 0;
    virtual size_t size() const                      = 0;
    virtual bool empty() const                       = 0;
    virtual void resize(size_t new_size)             = 0;
    virtual void eraseAt(int idx)                    = 0;

    virtual bool pushBack(Identifiable* id);
    virtual bool pushBack(std::shared_ptr<Identifiable> id);
    virtual ~PropertyIdentifiablePtrVectorInterface() {
    }
  };

  // parameter for a configurable module
  template <typename IdentifiablePtrType_>
  class PropertyIdentifiablePtr_ : public PropertyIdentifiablePtrInterface,
                                   public Property_<IdentifiablePtrType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    using ValueType     = IdentifiablePtrType_;
    using PointerTraits = PointerTraits_<IdentifiablePtrType_>;

    PropertyIdentifiablePtr_(const std::string& name_,
                             const std::string& description_,
                             PropertyContainerBase* container_,
                             ValueType value_ = 0,
                             bool* flag       = 0) :
      PropertyIdentifiablePtrInterface(container_),
      Property_<ValueType>(name_, description_, container_, value_, flag) {
    }

    virtual void serialize(ObjectData& odata, IdContext& id_context) override {
      odata.setPointer(
        this->_name, PointerTraits::getRawPtr(this->_value), this->_description.c_str());
    }
    const std::string& name() const override {
      return Property_<IdentifiablePtrType_>::name();
    }

    bool canAssign(Identifiable* id) override {
      return PointerTraits::canAssign(id);
    }
    bool canAssign(std::shared_ptr<Identifiable> id) override {
      return PointerTraits::canAssign(id);
    }

    bool assign(Identifiable* id) override {
      return PointerTraits::assign(this->_value, id);
    }
    bool assign(std::shared_ptr<Identifiable> id) override {
      return PointerTraits::assign(this->_value, id);
    }

    virtual Identifiable* getRawPtr() override {
      return PointerTraits::getRawPtr(this->_value);
    }
    virtual IdentifiablePtr getSharedPtr() override {
      return PointerTraits::getSharedPtr(this->_value);
    }

    //! typed pointer wrappers (calling potential overrides internally) - break virtuality
    template <typename Type_>
    Type_* getRawPtr() {
      return dynamic_cast<Type_*>(getRawPtr());
    }
    template <typename Type_>
    std::shared_ptr<Type_> getSharedPtr() {
      return std::dynamic_pointer_cast<Type_>(getSharedPtr());
    }

    virtual void deserialize(ObjectData& odata, IdContext& id_context) override {
      odata.getReference(this->name()).bind(this->_value);
    }

    const std::vector<std::string>& assignableTypes() const override {
      return _assignable_types;
    }
    std::vector<std::string>& assignableTypes() override {
      return _assignable_types;
    }

  protected:
    static std::vector<std::string> _assignable_types;
  };

  template <typename IdentifiablePtrType_>
  std::vector<std::string> PropertyIdentifiablePtr_<IdentifiablePtrType_>::_assignable_types;

  template <typename IdentifiablePtrType_>
  class PropertyIdentifiablePtrVector_ : public PropertyIdentifiablePtrVectorInterface,
                                         public Property_<std::vector<IdentifiablePtrType_>> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
    using ValueType     = IdentifiablePtrType_;
    using VectorType    = std::vector<ValueType>;
    using PointerTraits = PointerTraits_<ValueType>;

    PropertyIdentifiablePtrVector_(const std::string& name_,
                                   const std::string& description_,
                                   PropertyContainerBase* container_,
                                   bool* flag = 0) :
      PropertyIdentifiablePtrVectorInterface(container_),
      Property_<VectorType>(name_, description_, container_, VectorType(), flag) {
    }

    // inline VectorType& value() {this->touch(); return this->_value;}
    // inline const VectorType& value() const {return this->_value;}
    // inline void setValue(const VectorType& value_) {this->_value=value_;
    // this->touch();}
    const std::string& name() const override {
      return Property_<std::vector<IdentifiablePtrType_>>::name();
    }

    virtual Identifiable* getRawPtr(size_t i) override {
      return PointerTraits::getRawPtr(this->_value[i]);
    }
    virtual IdentifiablePtr getSharedPtr(size_t i) override {
      return PointerTraits::getSharedPtr(this->_value[i]);
    }

    //! typed smart pointer wrapper (calling potential overrides internally)
    template <typename Type_>
    std::shared_ptr<Type_> getSharedPtr(const size_t& index_) {
      return std::dynamic_pointer_cast<Type_>(getSharedPtr(index_));
    }

    size_t size() const override {
      return this->_value.size();
    }
    bool empty() const override {
      return this->_value.empty();
    }
    void resize(size_t new_size) override {
      this->_value.resize(new_size);
    }
    void eraseAt(int idx) override {
      this->_value.erase(this->_value.begin() + idx);
    }
    inline const ValueType& value(int idx) const {
      return this->_value[idx];
    }

    inline void setValue(int idx, const ValueType& v) {
      this->_value[idx] = v;
      this->touch();
    }
    inline const ValueType& operator[](int idx) const {
      return this->_value[idx];
    }

    inline ValueType& operator[](int idx) {
      this->touch();
      return this->_value[idx];
    }

    bool canAssign(Identifiable* id) override {
      return PointerTraits::canAssign(id);
    }
    bool canAssign(std::shared_ptr<Identifiable> id) override {
      return PointerTraits::canAssign(id);
    }
    bool assign(size_t idx, Identifiable* id) override {
      return PointerTraits::assign(this->_value[idx], id);
    }
    bool assign(size_t idx, std::shared_ptr<Identifiable> id) override {
      return PointerTraits::assign(this->_value[idx], id);
    }

    void serialize(ObjectData& odata, IdContext& id_context) override {
      ArrayData* adata = new ArrayData;
      for (size_t i = 0; i < this->_value.size(); ++i) {
        adata->addPointer(PointerTraits::getRawPtr(this->_value[i]));
      }
      odata.setField(this->name(), adata);
    }

    void deserialize(ObjectData& odata, IdContext& id_context) override {
      ArrayData* adata = dynamic_cast<ArrayData*>(odata.getField(this->name()));
      this->_value.resize(adata->size());
      for (size_t i = 0; i < adata->size(); ++i) {
        (*adata)[i].getReference().bind(this->_value[i]);
      }
    }

    const std::vector<std::string>& assignableTypes() const override {
      return _assignable_types;
    }
    std::vector<std::string>& assignableTypes() override {
      return _assignable_types;
    }
    std::vector<IdentifiablePtrType_>& value() { return this->_value;}
  protected:
    static std::vector<std::string> _assignable_types;
  };

  template <typename IdentifiablePtrType_>
  std::vector<std::string> PropertyIdentifiablePtrVector_<IdentifiablePtrType_>::_assignable_types;

} // namespace srrg2_core
