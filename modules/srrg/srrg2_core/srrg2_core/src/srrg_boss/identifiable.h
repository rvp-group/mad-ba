#pragma once
#include <memory>
#include "serializable.h"

namespace srrg2_core {

  class IdContext;

  class Identifiable: public Serializable {
    friend class IdContext;
    friend class Deserializer;
  public:
    
    Identifiable(int id=-1, IdContext* context=0);
    virtual ~Identifiable();
    bool setId(int id, IdContext* context=0);
    bool setContext(IdContext* context);
    IdContext* getContext() {return _context;}
    const IdContext* getContext() const {return _context;}
    void ensureValidId(IdContext* context);
    int getId() const;
  
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
  
    friend class IdContext;
    std::shared_ptr<Identifiable>& getSharedPtr();
  protected:
    int _id;
  private:
    //An Identifiable object is unique by definition and cannot be copied
    Identifiable& operator=(const Identifiable& other);
    Identifiable(const Identifiable& other);
    IdContext* _context;
  };

  using IdentifiablePtr = std::shared_ptr<Identifiable>;
}
