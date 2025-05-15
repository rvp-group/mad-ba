#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <typeinfo>
#include "trusted_loaders.h"

namespace srrg2_core {

  class ObjectData;
  class IdContext;

  std::vector<std::string> getClassNames();

  class Serializable {
  public:
    /*!
     * Save all your data here, don't forget to call superclass serialize method if not inheriting directly from
     * serializable
     */
    virtual void serialize(ObjectData& data, IdContext& context)=0;
    /*!
     * Restore object status here
     */
    virtual void deserialize(ObjectData& data, IdContext& context)=0;

    /*!
     * Deserializazion callback method, invoked by deserializer when all dangling pointers have been resolved
     */
    virtual void deserializeComplete();

    virtual ~Serializable() {}

    ObjectData* getSerializedData(IdContext& context);
    const std::string& className() const;
  
    static void registerFactory(const std::string& className, const std::string& typeIdName, Serializable* (*func)());
  
    /*!
     * throws std::logic_error if no factory method was registered for this class namespace
     */
    static Serializable* createInstance(const std::string& className);

    /*!
     * Utility function to get the current timestamp, for time-sensitive data
     */
    static double getCurrentTime();

  };

  template <class T> class AutoRegisterer {
  public:
    AutoRegisterer(const char* className, const char* typeIdName) {
      Serializable::registerFactory(className, typeIdName, &createInstance);
    }
  
    static Serializable* createInstance() {
      return new T();
    }
  };

  using SerializablePtr = std::shared_ptr<Serializable>;
  
#define BOSS_REGISTER_CLASS(class_name)                                 \
  static srrg2_core::AutoRegisterer<class_name > _reg_##class_name(#class_name,typeid(class_name).name());

#define BOSS_REGISTER_CLASS_WITH_NAME(class_name, type_name)            \
  static srrg2_core::AutoRegisterer<class_name > _reg_##class_name(#type_name,typeid(class_name).name()));

  
}
