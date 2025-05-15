#pragma once

// use this macro in to add default setters/getters
#define PROPERTY_ADD_SET_GET                               \
  inline ValueType& value() {                              \
    this->touch();                                         \
    return this->_value;                                   \
  }                                                        \
  template <typename Type_>                                \
  inline std::shared_ptr<Type_> value() {                  \
    this->touch();                                         \
    return std::dynamic_pointer_cast<Type_>(this->_value); \
  }                                                        \
  inline const ValueType& value() const {                  \
    return this->_value;                                   \
  }                                                        \
  inline const ValueType& constValue() const {             \
    return this->_value;                                   \
  }                                                        \
  inline void setValue(const ValueType& value_) {          \
    this->_value = value_;                                 \
    this->touch();                                         \
  }                                                        \
  inline const ValueType& operator->() const {             \
    return this->_value;                                   \
  }                                                        \
  inline ValueType& operator->() {                         \
    this->touch();                                         \
    return this->_value;                                   \
  }

// use this macro in the argument list of the configuration
#define SETUP_PROPERTY(param_name, value_) param_name(#param_name, "", this, value_)

// use this macro in the argument list of the configuration
#define SETUP_PROPERTY_NV(param_name) param_name(#param_name, "", this)
