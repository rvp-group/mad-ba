#pragma once
#include <cstring>
#include <cassert>

namespace srrg2_core {
  
  template <typename ValueType_>
  struct VectorInterface_ {
    using ValueType=ValueType_;
  
    using value_type=ValueType_;

    inline const ValueType* data() const {return _data;}

    inline ValueType* data()  {return _data;}

    inline size_t size() const  {return _size;}

    inline ValueType* begin() {return _data;}

    inline ValueType* end() {return _data+_size;}

    inline const ValueType* begin() const {return _data;}

    inline const ValueType* end() const {return _data+_size;}

    inline ValueType& at(size_t pos) {assert(pos<_size && "pos error"); return _data[pos];}

    inline const ValueType& at(size_t pos) const {assert(pos<_size && "pos error"); return _data[pos];}

    inline ValueType& operator[](size_t pos) {return _data[pos];}

    inline const ValueType& operator[](size_t pos) const {return _data[pos];}

    virtual void clear() = 0;

    virtual void resize(size_t new_size) =0;

    virtual ~VectorInterface_(){};
  
  protected:
    void copyFrom(const VectorInterface_<ValueType>& other) {
      if (! other.size())
        return;
      resize(other.size());
      memcpy(_data, other._data, sizeof(ValueType_)*other.size());
    }
    ValueType* _data=0;
    size_t _size=0;
  };
}
