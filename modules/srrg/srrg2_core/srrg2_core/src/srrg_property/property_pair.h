#pragma once
#include <iostream>

#include "property.h"

namespace srrg2_core {

  template <>
  class Property_<std::pair<size_t, size_t>> : public PropertyBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    using ValueType = std::pair<size_t, size_t>;
    using ThisType  = Property_<std::pair<size_t, size_t>>;

    Property_(const std::string& name_,
              const std::string& description_,
              PropertyContainerBase* config_,
              ValueType value_,
              bool* flag = 0) :
      PropertyBase(name_, description_, config_, flag) {
      _value = value_;
    }

    PROPERTY_ADD_SET_GET;

    void copyTo(PropertyBase& other_) override {
      ThisType& other = dynamic_cast<ThisType&>(other_);
      other           = *this;
    }

    PropertyBase* clone() override {
      ThisType* cloned = new ThisType(name(), description(), 0, value());
      copyTo(*cloned);
      return cloned;
    }

    void serialize(ObjectData& odata, IdContext& id_context) override {
      ArrayData* adata = new ArrayData;
      adata->add(_value.first);
      adata->add(_value.second);
      odata.setField(this->name(), adata);
    }

    void deserialize(ObjectData& odata, IdContext& id_context) override {
      ArrayData* adata = dynamic_cast<ArrayData*>(odata.getField(this->name()));
      ValueData& v1    = (*adata)[0];
      ValueData& v2    = (*adata)[1];
      this->_value.first  = size_t(v1);
      this->_value.second = size_t(v2);
    }

    bool fromTokens(std::vector<std::string>& tokens) override {
      for (std::string& s : tokens) {
        std::istringstream is(s.c_str());
        size_t v1;
        size_t v2;
        is >> v1 >> v2;
        setValue(std::make_pair(v1, v2));
        std::cerr << "|fromString: new value: " << _value.first << " "
                  << _value.second << std::endl;
      }
      return true;
    }

  protected:
    ValueType _value;
  };

  using PropertyPairUnsigned = Property_<std::pair<size_t, size_t>>;
} // namespace srrg2_core
