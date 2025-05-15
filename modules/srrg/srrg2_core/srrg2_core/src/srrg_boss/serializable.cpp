#include <stdexcept>
#include <sys/time.h>
#include <typeinfo>

#include "object_data.h"
#include "serializable.h"

namespace srrg2_core {
  using namespace std;

  static map<string, Serializable* (*) ()>& factoryMap() {
    static map<string, Serializable* (*) ()> FACTORY_MAP;
    return FACTORY_MAP;
  }

  static map<string, string>& typeIdMap() {
    static map<string, string> TYPEID_MAP;
    return TYPEID_MAP;
  }

  std::vector<std::string> getClassNames() {
    vector<string> r;
    map<string, Serializable* (*) ()>& fm = factoryMap();
    for (map<string, Serializable* (*) ()>::iterator it = fm.begin(); it != fm.end(); ++it) {
      r.push_back(it->first);
    }
    return r;
  }

  void Serializable::registerFactory(const string& className,
                                     const string& typeIdName,
                                     Serializable* (*func)()) {
    // cerr << "Registered class " << className << endl;
    factoryMap()[className] = func;
    typeIdMap()[typeIdName] = className;
  }

  Serializable* Serializable::createInstance(const string& className) {
    map<string, Serializable* (*) ()>::iterator function = factoryMap().find(className);
    if (function == factoryMap().end()) {
      throw logic_error("no factory function mapped for type " + className);
    }
    return (*function).second();
  }

  const string& Serializable::className() const {
    map<string, string>::const_iterator cname = typeIdMap().find(typeid(*this).name());
    if (cname == typeIdMap().end()) {
      throw logic_error("className() called on unregistered class " + string(typeid(*this).name()));
    }
    return (*cname).second;
  }

  ObjectData* Serializable::getSerializedData(IdContext& context) {
    ObjectData* o = new ObjectData();
    serialize(*o, context);
    return o;
  }

  void Serializable::deserializeComplete() {
  }

  double Serializable::getCurrentTime() {
    timeval time;
    gettimeofday(&time, 0);
    return time.tv_sec + (time.tv_usec / 1000000.0);
  }

} // namespace srrg2_core
