#include <cassert>
#include <iostream>

#include "srrg_boss/id_context.h"
#include "srrg_boss/object_data.h"

#include "property.h"
#include "property_container.h"

#define MAKE_CLONE(_type_)                                                    \
  PropertyBase* _type_::clone() {                                             \
    ThisType* copy_of_self = new ThisType(name(), description(), 0, value()); \
    copyTo(*copy_of_self);                                                    \
    return copy_of_self;                                                      \
  }

#define MAKE_COPY_TO(_type_)                                 \
  void _type_::copyTo(PropertyBase& other_) {                \
    ThisType& other = dynamic_cast<ThisType&>(other_);       \
    assert(other.name() == this->name() && "name mismatch"); \
    other = *this;                                           \
  }

#define MAKE_TRIVIAL_FROM_TOKENS(_type_)                      \
  bool _type_::fromTokens(std::vector<std::string>& tokens) { \
    for (std::string & s : tokens) {                          \
      istringstream is(s.c_str());                            \
      ValueType v;                                            \
      is >> v;                                                \
      setValue(v);                                            \
      cerr << "|fromString: new value: " << _value << endl;   \
    }                                                         \
    return true;                                              \
  }

namespace srrg2_core {

  using namespace std;

  PropertyBase::PropertyBase(const std::string& name_,
                             const std::string& description_,
                             PropertyContainerBase* config_,
                             bool* flag) :
    _name(name_),
    _description(description_) {
    if (config_) {
      config_->addProperty(this);
    }
    bindChangedFlag(flag);
  }

  void PropertyBase::serialize(ObjectData& odata, IdContext& id_context) {
    //    std::cerr << "PropertyBase::serialize|WARNING, property [ " << _name
    //              << " ] has no valid serialization method" << std::endl;
    throw std::runtime_error("PropertyBase::serialize|ERROR, not implemented for property [ " +
                             _name + " ]");
  }

  void PropertyBase::deserialize(ObjectData& odata, IdContext& id_context) {
    //    std::cerr << "PropertyBase::deserialize|WARNING, property [ " << _name
    //              << " ] has no valid deserialization method" << std::endl;
    throw std::runtime_error("PropertyBase::deserialize|ERROR, not implemented for property [ " +
                             _name + " ]");
  }

  bool PropertyBase::fromTokens(std::vector<std::string>& tokens) {
    std::cerr << "PropertyBase::fromTokens| fromString not implemented for "
                 "this type, parse error"
              << std::endl;
    return false;
  }

  PropertyBase* PropertyBase::clone() {
    throw std::runtime_error("PropertyBase::clone| not implemented");
  }

  void PropertyBase::copyTo(PropertyBase& other) {
    throw std::runtime_error("PropertyBase::copyTo| not implemented");
  }

  PropertyInt::Property_(const std::string& name_,
                         const std::string& description_,
                         PropertyContainerBase* config_,
                         int value_,
                         bool* flag) :
    PropertyBase(name_, description_, config_, flag),
    _value(value_) {
  }

  MAKE_CLONE(PropertyInt)

  MAKE_COPY_TO(PropertyInt)

  MAKE_TRIVIAL_FROM_TOKENS(PropertyInt)

  void PropertyInt::serialize(ObjectData& odata, IdContext& id_context) {
    odata.setInt(this->_name, this->_value, this->_description.c_str());
  }

  void PropertyInt::deserialize(ObjectData& odata, IdContext& id_context) {
    setValue(odata.getInt(this->_name));
  }

  // bdc uint8_t
  PropertyUInt8::Property_(const std::string& name_,
                           const std::string& description_,
                           PropertyContainerBase* config_,
                           uint8_t value_,
                           bool* flag) :
    PropertyBase(name_, description_, config_, flag),
    _value(value_) {
  }

  MAKE_CLONE(PropertyUInt8)

  MAKE_COPY_TO(PropertyUInt8)

  MAKE_TRIVIAL_FROM_TOKENS(PropertyUInt8)

  void PropertyUInt8::serialize(ObjectData& odata, IdContext& id_context) {
    odata.setUnsignedInt(this->_name, this->_value, this->_description.c_str());
  }

  void PropertyUInt8::deserialize(ObjectData& odata, IdContext& id_context) {
    setValue(odata.getUnsignedInt(this->_name));
  }

  // ia unsegned int
  PropertyUnsignedInt::Property_(const std::string& name_,
                                 const std::string& description_,
                                 PropertyContainerBase* config_,
                                 uint64_t value_,
                                 bool* flag) :
    PropertyBase(name_, description_, config_, flag),
    _value(value_) {
  }

  MAKE_CLONE(PropertyUnsignedInt)

  MAKE_COPY_TO(PropertyUnsignedInt)

  MAKE_TRIVIAL_FROM_TOKENS(PropertyUnsignedInt)

  void PropertyUnsignedInt::serialize(ObjectData& odata, IdContext& id_context) {
    odata.setUnsignedInt(this->_name, this->_value, this->_description.c_str());
  }

  void PropertyUnsignedInt::deserialize(ObjectData& odata, IdContext& id_context) {
    setValue(odata.getUnsignedInt(this->_name));
  }

  // ia property bool
  PropertyBool::Property_(const std::string& name_,
                          const std::string& description_,
                          PropertyContainerBase* config_,
                          bool value_,
                          bool* flag) :
    PropertyBase(name_, description_, config_, flag),
    _value(value_) {
  }

  MAKE_CLONE(PropertyBool)

  MAKE_COPY_TO(PropertyBool)

  MAKE_TRIVIAL_FROM_TOKENS(PropertyBool)

  void PropertyBool::serialize(ObjectData& odata, IdContext& id_context) {
    odata.setBool(this->_name, this->_value, this->_description.c_str());
  }

  void PropertyBool::deserialize(ObjectData& odata, IdContext& id_context) {
    setValue(odata.getBool(this->_name));
  }

  PropertyFloat::Property_(const std::string& name_,
                           const std::string& description_,
                           PropertyContainerBase* config_,
                           float value_,
                           bool* flag) :
    PropertyBase(name_, description_, config_, flag),
    _value(value_) {
  }

  MAKE_CLONE(PropertyFloat)

  MAKE_COPY_TO(PropertyFloat)

  MAKE_TRIVIAL_FROM_TOKENS(PropertyFloat)

  void PropertyFloat::serialize(ObjectData& odata, IdContext& id_context) {
    odata.setFloat(this->_name, this->_value, this->_description.c_str());
  }

  void PropertyFloat::deserialize(ObjectData& odata, IdContext& id_context) {
    setValue(odata.getFloat(this->_name));
  }

  PropertyDouble::Property_(const std::string& name_,
                            const std::string& description_,
                            PropertyContainerBase* config_,
                            double value_,
                            bool* flag) :
    PropertyBase(name_, description_, config_, flag),
    _value(value_) {
  }

  MAKE_CLONE(PropertyDouble)

  MAKE_COPY_TO(PropertyDouble)

  MAKE_TRIVIAL_FROM_TOKENS(PropertyDouble)

  void PropertyDouble::serialize(ObjectData& odata, IdContext& id_context) {
    odata.setDouble(this->_name, this->_value, this->_description.c_str());
  }

  void PropertyDouble::deserialize(ObjectData& odata, IdContext& id_context) {
    setValue(odata.getDouble(this->_name));
  }

  PropertyString::Property_(const std::string& name_,
                            const std::string& description_,
                            PropertyContainerBase* config_,
                            const std::string& value_,
                            bool* flag) :
    PropertyBase(name_, description_, config_, flag),
    _value(value_) {
  }

  MAKE_CLONE(PropertyString)

  MAKE_COPY_TO(PropertyString)

  MAKE_TRIVIAL_FROM_TOKENS(PropertyString)

  void PropertyString::serialize(ObjectData& odata, IdContext& id_context) {
    odata.setString(this->_name, this->_value, this->_description.c_str());
  }

  void PropertyString::deserialize(ObjectData& odata, IdContext& id_context) {
    setValue(odata.getString(this->_name));
  }

} // namespace srrg2_core
