#include <stdexcept>
#include <sstream>

#include "object_data.h"
#include "id_placeholder.h"

using namespace std;
using namespace srrg2_core;

//ValueData
int ValueData::getInt() {
  throw logic_error("getInt not allowed for type "+typeName());
}

uint64_t ValueData::getUnsignedInt() {
  throw logic_error("getInt not allowed for type "+typeName());
}

double ValueData::getDouble() {
  throw logic_error("getDouble not allowed for type "+typeName());
}

float ValueData::getFloat() {
  throw logic_error("getFloat not allowed for type "+typeName());
}

bool ValueData::getBool() {
  throw logic_error("getBool not allowed for type "+typeName());
}

const string& ValueData::getString() {
  throw logic_error("getString not allowed for type "+typeName());
}

Identifiable* ValueData::getPointer() {
  throw logic_error("getPointer not allowed for type "+typeName());
}

PointerReference& ValueData::getReference() {
  throw logic_error("getReference not allowed for type "+typeName());
}

ArrayData& ValueData::getArray() {
  throw logic_error("getArray not allowed for type "+typeName());
}

ObjectData& ValueData::getObject() {
  throw logic_error("getObject not allowed for type "+typeName());
}

ValueData::~ValueData() {}

const string& ValueData::typeName() {
  static string boolTypeName="BoolData";
  static string numberTypeName="NumberData";
  static string stringTypeName="StringData";
  static string arrayTypeName="ArrayData";
  static string objectTypeName="ObjectData";
  static string pointerTypeName="PointerData";
  static string pointerRefTypeName="PointerReference";
  static string unknown="unknown";
  switch (type()) {
    case BOOL:
      return boolTypeName;
    case NUMBER:
      return numberTypeName;
    case STRING:
      return stringTypeName;
    case ARRAY:
      return arrayTypeName;
    case OBJECT:
      return objectTypeName;
    case POINTER:
      return pointerTypeName;
    case POINTER_REF:
      return pointerRefTypeName;
    default:
      return unknown;
  }
}

//BoolData
bool BoolData::getBool() {
  return _value;
}

int BoolData::getInt() {
  return (int) _value;
}

ValueType BoolData::type() {
  return BOOL;
}

//NumberData
double NumberData::getDouble() {
  return _value;
}

int NumberData::getInt() {
  return (int) _value;
}

uint64_t NumberData::getUnsignedInt() {
  return (uint64_t) _value;
}

bool NumberData::getBool() {
  return _value!=0.0;
}

float NumberData::getFloat() {
  return (float) _value;
}

ValueType NumberData::type() {
    return NUMBER;
}


//StringData
const string& StringData::getString() {
  return _value;
}

ValueType StringData::type() {
    return STRING;
}

//ArrayData
ValueType ArrayData::type() {
  return ARRAY;
}

ArrayData& ArrayData::getArray() {
  return *this;
}

ArrayData::~ArrayData() {
  for (vector<ValueData*>::iterator v_it=_value.begin();v_it!=_value.end();delete *(v_it++));
}

void ArrayData::add(ValueData* value) {
  _value.push_back(value);
}

void ArrayData::add(double value) {
  _value.push_back(new NumberData(value));
}

void ArrayData::add(float value) {
  _value.push_back(new NumberData(value));
}

void ArrayData::add(int value) {
  _value.push_back(new NumberData(value));
}

void ArrayData::add(uint64_t value) {
  _value.push_back(new NumberData(value));
}

void ArrayData::add(bool value) {
  _value.push_back(new BoolData(value));
}

void ArrayData::add(const string& value) {
  _value.push_back(new StringData(value));
}

void ArrayData::add(const char* value) {
  _value.push_back(new StringData(value));
}

void ArrayData::addPointer(Identifiable* ptr) {
  _value.push_back(new PointerData(ptr));
}

void ArrayData::set(size_t idx, ValueData* value) {
  if (_value.at(idx)) {
    delete _value.at(idx);
  }
  _value[idx]=value;
}

void ArrayData::set(size_t idx, double value) {
  set(idx, new NumberData(value));
}

void ArrayData::set(size_t idx, float value) {
  set(idx, new NumberData(value));
}

void ArrayData::set(size_t idx, int value) {
  set(idx, new NumberData(value));
}

void ArrayData::set(size_t idx, uint64_t value) {
  set(idx, new NumberData(value));
}

void ArrayData::set(size_t idx, bool value) {
  set(idx, new BoolData(value));
}

void ArrayData::set(size_t idx, const string& value) {
  set(idx, new StringData(value));
}

void ArrayData::set(size_t idx, const char* value) {
  set(idx, new StringData(value));
}


//ObjectData
ValueType ObjectData::type() {
  return OBJECT;
}

ObjectData& ObjectData::getObject() {
  return *this;
}

ObjectData::~ObjectData() {
  for (map<string, ValueData*>::iterator v_it=_value.begin();v_it!=_value.end();delete (*(v_it++)).second);
}

ValueData* ObjectData::getField(const string& name) {
  map<string,ValueData*>::iterator field=_value.find(name);
  if (field!=_value.end()) {
    return (*field).second;
  }
  return 0;
}

void ObjectData::setField(const string& name, ValueData* value, const char* description) {
  ValueData*& field=_value[name];
  if (field) {
    delete field;
  } else {
    _fields.push_back(name);
  }
  field=value;
  field->_description=description;
}

void ObjectData::setInt(const string& name, int value, const char* description) {
  setField(name, new NumberData(value), description);
}

void ObjectData::setUnsignedInt(const string& name, uint64_t value, const char* description) {
  setField(name, new NumberData(value), description);
}

void ObjectData::setDouble(const string& name, double value, const char* description) {
  setField(name, new NumberData(value), description);
}

void ObjectData::setFloat(const string& name, float value, const char* description) {
  setField(name, new NumberData(value), description);
}

void ObjectData::setBool(const string& name, bool value, const char* description) {
  setField(name, new BoolData(value), description);
}

void ObjectData::setString(const string& name, const string& value, const char* description) {
  setField(name, new StringData(value), description);
}

void ObjectData::setPointer(const string& name, Identifiable* ptr, const char* description) {
  setField(name, new PointerData(ptr), description);
}

//PointerData
ValueType PointerData::type() {
  return POINTER;
}

Identifiable* PointerData::getPointer() {
  return _pointer;
}

//PointerReference
ValueType PointerReference::type() {
  return POINTER_REF;
}

Identifiable* PointerReference::getPointer() {
  if (!_ref) {
    return 0;
  }
  IdPlaceholder* phRef=dynamic_cast<IdPlaceholder*>(_ref);
  if (phRef) {
    ostringstream msg;
    msg << "pointer not resolved yet: " << phRef->getId();
    throw std::logic_error(msg.str());
  }
  return _ref;
}

#define STREAM_OPS(field_type, setter, getter) \
  std::pair<const std::string&, field_type &> field(const std::string& nm, field_type & val) { \
    return std::pair<const std::string&, field_type &>(nm, val); \
  } \
  ObjectData& operator << (ObjectData& o, std::pair<const std::string&, field_type &> f) { \
    o.setter(f.first,f.second); \
    return o; \
  } \
  ObjectData& operator >> (ObjectData& o, std::pair<const std::string&, field_type &> f) { \
    ValueData* v=o.getField(f.first); \
    if (v) { \
      f.second=v->getter(); \
    } \
    return o; \
  } \
  std::pair<const std::string&, const field_type &> field(const std::string& nm, const field_type & val) { \
    return std::pair<const std::string&, const field_type &>(nm, val); \
  } \
  ObjectData& operator << (ObjectData& o, std::pair<const std::string&, const field_type &> f) { \
    o.setter(f.first,f.second); \
    return o; \
  }

namespace srrg2_core {
  
  STREAM_OPS(bool, setBool, getBool)
  STREAM_OPS(int, setInt, getInt)
  STREAM_OPS(uint64_t, setUnsignedInt, getUnsignedInt)
  STREAM_OPS(float, setFloat, getFloat)
  STREAM_OPS(double, setDouble, getDouble)
  STREAM_OPS(string, setString, getString)

  std::pair<const std::string&, ValueData*&> field(const std::string& nm, ValueData*& val) {
    return std::pair<const std::string&, ValueData*&>(nm, val);
  }
  ObjectData& operator << (ObjectData& o, std::pair<const std::string&, ValueData*&> f) {
    o.setField(f.first,f.second);
    return o;
  }
  ObjectData& operator >> (ObjectData& o, std::pair<const std::string&, ValueData*&> f) {
    f.second=o.getField(f.first);
    return o;
  }

}
