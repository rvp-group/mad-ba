#pragma once
#include <cassert>
#include <memory>
#include <set>

#include <Eigen/Core>

#include "property_macros.h"
#include "srrg_boss/object_data.h"

namespace srrg2_core {

  class PropertyContainerBase;

  // abstract base param class
  // a parameter can be serialized
  // has a string name
  // had a striing description
  // has a changed flag that is set to true when calling the "touch" method
  // I added the explicit ctor parameter to force explicit registration of a
  // parameter in a config upon creation
  class PropertyBase {
  public:
    friend class PropertyContainerBase;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PropertyBase(const std::string& name_,
                 const std::string& description_,
                 PropertyContainerBase* config,
                 bool* flag = 0);
    virtual ~PropertyBase() {
    }

    inline void touch() {
      if (_changed_flag) {
        (*_changed_flag) = true;
      }
    }

    virtual void serialize(ObjectData& odata, IdContext& id_context);

    virtual void deserialize(ObjectData& odata, IdContext& id_context);

    inline const std::string& name() const {
      return _name;
    }

    inline const std::string& description() const {
      return _description;
    }

    virtual void copyTo(PropertyBase& other);

    virtual PropertyBase* clone();

    inline void bindChangedFlag(bool* changed_flag_) {
      _changed_flag = changed_flag_;
      touch();
    }

    inline void setName(const std::string& name_) {
      _name = name_;
    }

    virtual bool fromTokens(std::vector<std::string>& tokens);

  protected:
    std::string _name;
    std::string _description;
    bool* _changed_flag = 0; // set this on construction. If set this will
                             // toggle the flag when the variable is changed
  };

  // same as above but with a value
  template <typename ValueType_>
  class PropertyNonCopiable_ : public PropertyBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ValueType = ValueType_;
    using ThisType  = PropertyNonCopiable_<ValueType>;

    PropertyNonCopiable_(const std::string& name_,
                         const std::string& description_,
                         PropertyContainerBase* config_,
                         ValueType value_,
                         bool* flag = 0) :
      PropertyBase(name_, description_, config_, flag),
      _value(value_) {
    }

    PropertyNonCopiable_(const std::string& name_,
                         const std::string& description_,
                         PropertyContainerBase* config_,
                         bool* flag = 0) :
      PropertyBase(name_, description_, config_, flag) {
    }

    PROPERTY_ADD_SET_GET;

    PropertyBase* clone() override {
      ThisType* copy_of_self = new ThisType(name(), description(), 0);
      copyTo(*copy_of_self);
      return copy_of_self;
    }

  protected:
    ValueType_ _value;
  };

  // same as above but with a value
  template <typename ValueType_>
  class Property_ : public PropertyNonCopiable_<ValueType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using ValueType = ValueType_;
    using ThisType  = Property_<ValueType>;
    using BaseType  = PropertyNonCopiable_<ValueType_>;

    Property_(const std::string& name_,
              const std::string& description_,
              PropertyContainerBase* config_,
              ValueType value_,
              bool* flag = 0) :
      BaseType(name_, description_, config_, value_, flag) {
    }

    Property_(const std::string& name_,
              const std::string& description_,
              PropertyContainerBase* config_,
              bool* flag = 0) :
      BaseType(name_, description_, config_, flag) {
    }

    void copyTo(PropertyBase& other_) override {
      ThisType& other = dynamic_cast<ThisType&>(other_);
      assert(other.name() == this->name() && "name mismatch");
      other = *this;
    }
  };

  // partial specialization for classic types
  template <>
  class Property_<int> : public PropertyBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using ValueType = int;
    using ThisType  = Property_<int>;
    Property_(const std::string& name_,
              const std::string& description_,
              PropertyContainerBase* config_,
              int value_,
              bool* flag = 0);

    PROPERTY_ADD_SET_GET;

    PropertyBase* clone() override;
    void copyTo(PropertyBase& other) override;
    void serialize(ObjectData& odata, IdContext& id_context) override;
    void deserialize(ObjectData& odata, IdContext& id_context) override;
    bool fromTokens(std::vector<std::string>& tokens) override;

  protected:
    ValueType _value;
  };

  // partial specialization for classic types
  template <>
  class Property_<uint8_t> : public PropertyBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using ValueType = uint8_t;
    using ThisType  = Property_<uint8_t>;
    Property_(const std::string& name_,
              const std::string& description_,
              PropertyContainerBase* config_,
              uint8_t value_,
              bool* flag = 0);

    PROPERTY_ADD_SET_GET;

    PropertyBase* clone() override;
    void copyTo(PropertyBase& other) override;
    void serialize(ObjectData& odata, IdContext& id_context) override;
    void deserialize(ObjectData& odata, IdContext& id_context) override;
    bool fromTokens(std::vector<std::string>& tokens) override;

  protected:
    ValueType _value;
  };

  // partial specialization for classic types
  template <>
  class Property_<uint64_t> : public PropertyBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using ValueType = uint64_t;
    using ThisType  = Property_<uint64_t>;
    Property_(const std::string& name_,
              const std::string& description_,
              PropertyContainerBase* config_,
              uint64_t value_,
              bool* flag = 0);

    PROPERTY_ADD_SET_GET;

    PropertyBase* clone() override;
    void copyTo(PropertyBase& other) override;
    void serialize(ObjectData& odata, IdContext& id_context) override;
    void deserialize(ObjectData& odata, IdContext& id_context) override;
    bool fromTokens(std::vector<std::string>& tokens) override;

  protected:
    ValueType _value;
  };

  // partial specialization for classic types
  template <>
  class Property_<bool> : public PropertyBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using ValueType = bool;
    using ThisType  = Property_<bool>;
    Property_(const std::string& name_,
              const std::string& description_,
              PropertyContainerBase* config_,
              bool value_,
              bool* flag = 0);

    PROPERTY_ADD_SET_GET;

    PropertyBase* clone() override;
    void copyTo(PropertyBase& other) override;
    void serialize(ObjectData& odata, IdContext& id_context) override;
    void deserialize(ObjectData& odata, IdContext& id_context) override;
    bool fromTokens(std::vector<std::string>& tokens) override;

  protected:
    ValueType _value;
  };

  template <>
  class Property_<float> : public PropertyBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ValueType = float;
    using ThisType  = Property_<float>;
    Property_(const std::string& name_,
              const std::string& description_,
              PropertyContainerBase* config_,
              float value_,
              bool* flag = 0);

    PROPERTY_ADD_SET_GET;

    PropertyBase* clone() override;
    void copyTo(PropertyBase& other) override;
    void serialize(ObjectData& odata, IdContext& id_context) override;
    void deserialize(ObjectData& odata, IdContext& id_context) override;
    bool fromTokens(std::vector<std::string>& tokens) override;

  protected:
    ValueType _value;
  };

  template <>
  class Property_<double> : public PropertyBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ValueType = double;
    using ThisType  = Property_<double>;
    Property_(const std::string& name_,
              const std::string& description_,
              PropertyContainerBase* config_,
              double value_,
              bool* flag = 0);

    PROPERTY_ADD_SET_GET;

    PropertyBase* clone() override;
    void copyTo(PropertyBase& other) override;
    void serialize(ObjectData& odata, IdContext& id_context) override;
    void deserialize(ObjectData& odata, IdContext& id_context) override;
    bool fromTokens(std::vector<std::string>& tokens) override;

  protected:
    ValueType _value;
  };

  template <>
  class Property_<std::string> : public PropertyBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using ValueType = std::string;
    using ThisType  = Property_<std::string>;
    Property_(const std::string& name_,
              const std::string& description_,
              PropertyContainerBase* config_,
              const std::string& value_,
              bool* flag = 0);

    PROPERTY_ADD_SET_GET;

    PropertyBase* clone() override;
    void copyTo(PropertyBase& other) override;
    void serialize(ObjectData& odata, IdContext& id_context) override;
    void deserialize(ObjectData& odata, IdContext& id_context) override;
    bool fromTokens(std::vector<std::string>& tokens) override;

  protected:
    ValueType _value;
  };

  using PropertyInt         = Property_<int>;
  using PropertyUInt8       = Property_<uint8_t>;
  using PropertyUnsignedInt = Property_<uint64_t>;
  using PropertyBool        = Property_<bool>;
  using PropertyFloat       = Property_<float>;
  using PropertyDouble      = Property_<double>;
  using PropertyString      = Property_<std::string>;

} // namespace srrg2_core
