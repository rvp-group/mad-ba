#pragma once
#include <sstream>

#include "property.h"

namespace srrg2_core {

  // partial specialization for classic types
  template <typename T, typename AllocatorType_ = std::allocator<T>>
  class PropertyVector_ : public Property_<std::vector<T, AllocatorType_>> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
    using ValueType     = std::vector<T, AllocatorType_>;
    using AllocatorType = AllocatorType_;
    using ThisType      = PropertyVector_<T>;

    PropertyVector_(const std::string& name_,
                    const std::string& description_,
                    PropertyContainerBase* config_,
                    const ValueType& value_,
                    bool* flag = 0) :
      Property_<ValueType>(name_, description_, config_, value_, flag) {
    }

    PropertyVector_(const std::string& name_,
                    const std::string& description_,
                    PropertyContainerBase* config_,
                    bool* flag = 0) :
      Property_<ValueType>(name_, description_, config_, ValueType(), flag) {
    }

    inline size_t size() const {
      return _value.size();
    }

    inline void resize(size_t s) {
      this->_value.resize(s);
      this->touch();
    }

    inline void pushBack(const T& v) {
      this->_value.push_back(v);
    }
    PROPERTY_ADD_SET_GET;

    inline const typename ValueType::value_type& value(int idx) const {
      return this->_value[idx];
    }

    inline void setValue(int idx, const typename ValueType::value_type& v) {
      this->_value[idx] = v;
      this->touch();
    }

    void serialize(ObjectData& odata, IdContext& id_context) override {
      ArrayData* adata = new ArrayData;
      for (size_t i = 0; i < this->_value.size(); ++i) {
        adata->add(_value[i]);
      }
      odata.setField(this->name(), adata);
    }

    void deserialize(ObjectData& odata, IdContext& id_context) override {
      ArrayData* adata = dynamic_cast<ArrayData*>(odata.getField(this->name()));
      this->_value.resize(adata->size());
      for (size_t i = 0; i < adata->size(); ++i) {
        ValueData& v    = (*adata)[i];
        this->_value[i] = T(v);
      }
    }

    bool fromTokens(std::vector<std::string>& tokens) override {
      _value.resize(tokens.size());
      int idx = 0;
      for (std::string& s : tokens) {
        std::istringstream is(s.c_str());
        typename ValueType::value_type v;
        is >> v;
        setValue(idx, v);
        std::cerr << "|fromString: new value: " << v << std::endl;
        ++idx;
      }
      return true;
    }

  protected:
    ValueType _value;
  };

} // namespace srrg2_core
