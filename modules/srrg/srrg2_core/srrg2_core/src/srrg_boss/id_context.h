#pragma once

#include "serialization_context.h"
#include <map>
#include <mutex>
namespace srrg2_core {

  class Identifiable;
  class IdPlaceholder;

  class IdContext {
    friend class Identifiable;
    friend class IdPlaceholder;
  
  public:
    IdContext();
    Identifiable* getById(int id);
    bool add(Identifiable* obj);
    bool add(std::shared_ptr<Identifiable> obj);
    bool remove(Identifiable* obj);
    bool remove(std::shared_ptr<Identifiable> obj);
  
    bool update(Identifiable* obj, int oldId);
    IdPlaceholder* createPlaceHolder(int id);
    int generateId();
    void setStartingId(int id);
    virtual ~IdContext();
    inline SerializationContext* serializationContext() {return _serializationContext;}
    inline const SerializationContext* serializationContext() const {return _serializationContext;}
    IdentifiablePtr& getSharedPtr(Identifiable* obj);
  
  protected:
    virtual bool addImpl(Identifiable* obj);
  
    std::map<int, Identifiable*> _instances;
    std::map<Identifiable*, std::shared_ptr<Identifiable> > _instances_shared;

    std::mutex _instances_lock;
    int _lastGeneratedID;
    SerializationContext* _serializationContext;
  };


} // namespace srrg2_core
