#pragma once
#include "vector_interface.h"
#include <vector>

namespace srrg2_core {
  
  template <typename ValueType_>
  struct MemoryVectorInterface_: public VectorInterface_<ValueType_> {
    MemoryVectorInterface_() {
      this->_data=0;
      this->_size=0;
    }

    MemoryVectorInterface_(const VectorInterface_<ValueType_>& other) {
      copyFrom(other);
    }

    MemoryVectorInterface_& operator=(const VectorInterface_<ValueType_>& other) {
      copyFrom(other);
      return *this;
    }

    void clear() override {
      _values.clear();
      this->_data=0;
      this->_size=0;
    }
  
    void resize(size_t new_size) override {
      _values.resize(new_size);
      this->_size=_values.size();
      this->_data=_values.data();
    }
  
    virtual ~MemoryVectorInterface_() {}
  protected:
    std::vector<ValueType_> _values;
  };
}
