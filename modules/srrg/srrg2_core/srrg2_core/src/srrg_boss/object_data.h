#pragma once

#include <string>
#include <vector>
#include <map>
#include <limits>
#include <stdexcept>
#include "id_placeholder.h"
#include <Eigen/Core>

namespace srrg2_core {

enum ValueType {
  BOOL, NUMBER, STRING, ARRAY, OBJECT, POINTER, POINTER_REF
};

class ArrayData;
class ObjectData;
class PointerReference;
  class IDContext;
  
class ValueData {
  friend class ObjectData;
public:
  virtual int getInt();
  virtual uint64_t getUnsignedInt();
  virtual double getDouble();
  virtual float getFloat();
  virtual bool getBool();
  virtual const std::string& getString();
  virtual Identifiable* getPointer();
  virtual PointerReference& getReference();

  virtual ArrayData& getArray();
  virtual ObjectData& getObject();

  virtual ValueType type()=0;
  const std::string& typeName();
  operator bool() {
    return getBool();
  }
  operator int() {
    return getInt();
  }
  operator uint64_t() {
    return getUnsignedInt();
  }
  operator double() {
    return getDouble();
  }
  operator float() {
    return getFloat();
  }
  operator const std::string&() {
    return getString();
  }

  virtual ~ValueData();

  inline const char* description() const {return _description;}

  const char* _description=0;
};


class BoolData: public ValueData {
public:
  BoolData(bool value): _value(value) {};
  virtual bool getBool();
  virtual int getInt();
  virtual ValueType type();

protected:
  bool _value;
};

class NumberData: public ValueData {
public:
  NumberData(double value=0.0, int precision=std::numeric_limits<double>::max_digits10):
    _value(value),
    _precision(precision) {}
  NumberData(float value=0.0, int precision=std::numeric_limits<float>::max_digits10):
    _value(value),
    _precision(precision) {}
  NumberData(int value=0, int precision=std::numeric_limits<double>::digits10):
    _value(value),
    _precision(precision) {}
  NumberData(uint64_t value=0, int precision=std::numeric_limits<uint64_t>::digits10):
    _value(value),
    _precision(precision) {}
  virtual int getInt();
  virtual uint64_t getUnsignedInt();
  virtual double getDouble();
  virtual float getFloat();
  virtual bool getBool();
  virtual ValueType type();

  int precision() {
    return _precision;
  }
protected:
  double _value;
  int _precision;
};

class StringData: public ValueData {
public:
  StringData(const std::string& value): _value(value) {}
  virtual const std::string& getString();
  virtual ValueType type();

protected:
  std::string _value;
};

class ArrayData: public ValueData {
public:
  virtual ValueType type();
  virtual ArrayData& getArray();
  virtual ~ArrayData();
  void add(bool value);
  void add(int value);
  void add(uint64_t value);
  void add(double value);
  void add(float value);
  void add(const std::string& value);
  void add(const char* value);
  void add(ValueData* value);
  void addPointer(Identifiable* ptr);
  
  void set(size_t idx, bool value);
  void set(size_t idx, int value);
  void set(size_t idx, uint64_t value);
  void set(size_t idx, double value);
  void set(size_t idx, float value);
  void set(size_t idx, const std::string& value);
  void set(size_t idx, const char* value);
  void set(size_t idx, ValueData* value);
  void setPointer(size_t idx, Identifiable* ptr);

  //minimal vector interface (read only)
  std::vector<ValueData*>::const_iterator begin() {
    return _value.begin();
  }
  std::vector<ValueData*>::const_iterator end() {
    return _value.end();
  }
  size_t capacity() {
    return _value.capacity();
  }
  void reserve(size_t n) {
    _value.reserve(n);
  }
  ValueData& operator[](size_t n) {
    return *_value.at(n);
  }
  size_t size() {
    return _value.size();
  }
  void push_back(ValueData* value) {
    _value.push_back(value);
  }

protected:
  std::vector<ValueData*> _value;
};

class ObjectData: public ValueData {
public:
  virtual ValueType type();
  virtual ObjectData& getObject();

  void setField(const std::string& name, ValueData* value, const char* description_=0);
  void setInt(const std::string& name, int value, const char* description_=0);
  void setUnsignedInt(const std::string& name, uint64_t value, const char* description_=0);
  void setDouble(const std::string& name, double value, const char* description_=0);
  void setFloat(const std::string& name, float value, const char* description_=0);
  void setString(const std::string& name, const std::string& value, const char* description_=0);
  void setString(const std::string& name, const char* value, const char* description_=0);
  void setBool(const std::string& name, bool value, const char* description_=0);
  void setPointer(const std::string&name, Identifiable* ptr, const char* description_=0);
  void setPointer(const std::string&name, std::shared_ptr<Identifiable> ptr, const char* description_=0) { setPointer(name, ptr.get(), description_);}
  
  int getInt(const std::string& name) {
    return getField(name)->getInt();
  }
  
  uint64_t getUnsignedInt(const std::string& name) {
    return getField(name)->getUnsignedInt();
  }

  double getDouble(const std::string& name) {
    return getField(name)->getDouble();
  }
  
  float getFloat(const std::string& name) {
    return getField(name)->getFloat();
  }
  
  const std::string& getString(const std::string& name) {
    return getField(name)->getString();
  }
  
  bool getBool(const std::string& name) {
    return getField(name)->getBool();
  }
  
  const std::vector<std::string>& fields() {
    return _fields;
  }

  PointerReference& getReference(const std::string&name) {
    return getField(name)->getReference();
  }

  Identifiable* getPointer(const std::string&name) {
    return getField(name)->getPointer();
  }
  
  ValueData* getField(const std::string& name);

  template <typename EigenType_>
  EigenType_ getEigen(const std::string& name) {
    EigenType_ eigen_object;
    ValueData* v=getField(name);
    int rows=eigen_object.matrix().rows();
    int cols=eigen_object.matrix().cols();
    ArrayData* adata;
    if  (eigen_object.matrix().SizeAtCompileTime==Eigen::Dynamic){
        ObjectData* o=dynamic_cast<ObjectData*>(v);
        rows=o->getInt("rows");
        cols=o->getInt("cols");
        eigen_object.matrix().resize(rows,cols);
        v=o->getField("values");
        adata=dynamic_cast<ArrayData*>(v);
      } else {
      adata=dynamic_cast<ArrayData*>(v);
      int k=0;
      for (int r=0; r<rows; ++r)
        for (int c=0; c<cols; ++c, ++k) {
          eigen_object.matrix()(r,c) = (*adata)[k].getFloat();
        }
    }
    return eigen_object;
  }
  
  template <typename EigenType_>
  void setEigen(const std::string& name, const EigenType_& eigen_object) {
    ArrayData* adata=new ArrayData;
    int rows=eigen_object.matrix().rows();
    int cols=eigen_object.matrix().cols();
    int k=0;
    for (int r=0; r<rows; ++r)
      for (int c=0; c<cols; ++c, ++k) {
        adata->add(eigen_object.matrix()(r,c));
      }
    if (eigen_object.matrix().SizeAtCompileTime==Eigen::Dynamic) {
        ObjectData* o= new ObjectData;
        o->setInt("rows", rows);
        o->setInt("cols", cols);
        o->setField("values", adata);
        setField(name, o);
      } else {
        setField(name, adata);
    }
  }
  
  virtual ~ObjectData();

protected:
  std::map<std::string, ValueData*> _value;
  std::vector<std::string> _fields;
};

class PointerData: public ValueData {
public:
  PointerData(Identifiable* pointer): _pointer(pointer) {}
  virtual Identifiable* getPointer();
  virtual ValueType type();

protected:
  Identifiable* _pointer;
};

class PointerReference: public ValueData {
public:
  PointerReference(Identifiable* ref): _ref(ref) {}

  PointerReference& getReference() {
    return *this;
  }

  virtual Identifiable* getPointer();

  template<typename T>
  void bind(T*& pvar) {
    if (!_ref) {
      pvar=0;
      return;
    }
    IdPlaceholder* phRef=dynamic_cast<IdPlaceholder*>(_ref);
    if (phRef) {
      //std::cerr << "raw bind added variable to resolve ref" << std::endl;
      phRef->addVariable(pvar);
    } else {
      pvar=dynamic_cast<T*>(_ref);
      if (!pvar) {
        throw std::logic_error("bad cast: "+_ref->className());
      }
    }
  }

  template<typename T>
  void bind(std::shared_ptr<T>& pvar) {
    if (!_ref) {
      pvar=0;
      return;
    }
    IdPlaceholder* phRef=dynamic_cast<IdPlaceholder*>(_ref);
    if (phRef) {
      //std::cerr << "shared bind added variable to resolve ref" << std::endl;
      phRef->addVariable(pvar);
    } else {
      T* pvar_=dynamic_cast<T*>(_ref);
      if (!pvar_) {
        throw std::logic_error("bad cast: "+_ref->className());
      }
      pvar=std::dynamic_pointer_cast<T>(_ref->getSharedPtr());
    }
  }

  template<typename T>
  void bind(std::weak_ptr<T>& pvar) {
    if (!_ref) {
      pvar.reset();
      return;
    }
    IdPlaceholder* phRef=dynamic_cast<IdPlaceholder*>(_ref);
    if (phRef) {
      //std::cerr << "weak bind added variable to resolve ref" << std::endl;
      phRef->addVariable(pvar);
    } else {
      T* pvar_=dynamic_cast<T*>(_ref);
      if (!pvar_) {
        throw std::logic_error("bad cast: "+_ref->className());
      }
      pvar=std::dynamic_pointer_cast<T>(_ref->getSharedPtr());
    }
  }

  virtual ValueType type();
protected:
  Identifiable* _ref;

};


std::pair<const std::string&, const bool&> field(const std::string& nm, const bool& val);
std::pair<const std::string&, bool&> field(const std::string& nm, bool& val);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, const bool&> f);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, bool&> f);
ObjectData& operator >> (ObjectData& o, std::pair<const std::string&, bool&> f);

std::pair<const std::string&, const int&> field(const std::string& nm, const int& val);
std::pair<const std::string&, int&> field(const std::string& nm, int& val);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, const int&> f);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, int&> f);
ObjectData& operator >> (ObjectData& o, std::pair<const std::string&, int&> f);

std::pair<const std::string&, const uint64_t&> field(const std::string& nm, const uint64_t& val);
std::pair<const std::string&, uint64_t&> field(const std::string& nm, uint64_t& val);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, const uint64_t&> f);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, uint64_t&> f);
ObjectData& operator >> (ObjectData& o, std::pair<const std::string&, uint64_t&> f);


std::pair<const std::string&, const float&> field(const std::string& nm, const float& val);
std::pair<const std::string&, float&> field(const std::string& nm, float& val);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, const float&> f);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, float&> f);
ObjectData& operator >> (ObjectData& o, std::pair<const std::string&, float&> f);

std::pair<const std::string&, const double&> field(const std::string& nm, const double& val);
std::pair<const std::string&, double&> field(const std::string& nm, double& val);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, const double&> f);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, double&> f);
ObjectData& operator >> (ObjectData& o, std::pair<const std::string&, double&> f);

std::pair<const std::string&, const bool&> field(const std::string& nm, const bool& val);
std::pair<const std::string&, bool&> field(const std::string& nm, bool& val);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, const bool&> f);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, bool&> f);
ObjectData& operator >> (ObjectData& o, std::pair<const std::string&, bool&> f);

std::pair<const std::string&, const std::string&> field(const std::string& nm, const std::string& val);
std::pair<const std::string&, std::string&> field(const std::string& nm, std::string& val);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, const std::string&> f);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, std::string&> f);
ObjectData& operator >> (ObjectData& o, std::pair<const std::string&, std::string&> f);

std::pair<const std::string&, ValueData*&> field(const std::string& nm, ValueData*& val);
ObjectData& operator << (ObjectData& o, std::pair<const std::string&, ValueData*&> f);
ObjectData& operator >> (ObjectData& o, std::pair<const std::string&, ValueData*&> f);

}
