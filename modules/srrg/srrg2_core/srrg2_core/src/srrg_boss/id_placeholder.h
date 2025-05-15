#pragma once

#include <vector>
#include <memory>

#include "identifiable.h"

namespace srrg2_core {

  class IdContext;

  class AbstractPlaceHolderAssigner {
  public:
    AbstractPlaceHolderAssigner(IdContext& context_):
      _context(&context_){}
    
    virtual void assign(Identifiable* instance)=0;
    IdentifiablePtr getSharedPtr(Identifiable* instance);
    virtual ~AbstractPlaceHolderAssigner() {}
  protected:
    IdContext* _context;
  };

  template<typename T> class PtrPlaceHolderAssigner: public AbstractPlaceHolderAssigner {
  public:
    PtrPlaceHolderAssigner(IdContext& context_, T*& var):
      AbstractPlaceHolderAssigner(context_),
      _var(&var) {}

    virtual void assign(Identifiable* instance) {
      *_var=dynamic_cast<T*>(instance);    
    }
    virtual ~PtrPlaceHolderAssigner() {}

    inline T** var() { return _var; }

  protected:
    T** _var;
  };

  template<typename T>
  class SharedPtrPlaceHolderAssigner: public AbstractPlaceHolderAssigner {
  public:
    SharedPtrPlaceHolderAssigner(IdContext& context_, std::shared_ptr<T>& var):
     AbstractPlaceHolderAssigner(context_),
     _var(&var) {}

    virtual void assign(Identifiable* instance) {
      //std::cerr << "shared assign " << instance << std::endl;
      if (instance) {
        *_var=std::dynamic_pointer_cast<T>(getSharedPtr(instance));
      } else {
        _var->reset();
      }
    }
    virtual ~SharedPtrPlaceHolderAssigner() {}

    inline std::shared_ptr<T>* var() { return _var; }

  protected:
    std::shared_ptr<T>* _var;
  };

  template<typename T>
  class WeakPtrPlaceHolderAssigner: public AbstractPlaceHolderAssigner {
  public:
    WeakPtrPlaceHolderAssigner(IdContext& context_, std::weak_ptr<T>& var):
      AbstractPlaceHolderAssigner(context_),
      _var(&var)
    {}

    virtual void assign(Identifiable* instance) {
      //std::cerr << "weak assign " << instance << " id: " << instance->getId() << " ctx: " << instance->getContext() << std::endl;
      if (instance) {
        //std::cerr << "sptr assign " << getSharedPtr(instance) << std::endl;
        *_var=std::dynamic_pointer_cast<T>(getSharedPtr(instance));
        //std::cerr << " OK" << _var->lock() << std::endl;
      } else {
        _var->reset();
      }
    }
    virtual ~WeakPtrPlaceHolderAssigner() {}

    inline std::weak_ptr<T>* var() { return _var; }

  protected:
    std::weak_ptr<T>* _var;
  };

  class IdPlaceholder: virtual public Identifiable {
  public:
    template<typename T> void addVariable(T*& var) {
      _assigners.push_back(new PtrPlaceHolderAssigner<T>(*getContext(),var));
    }

    template <typename T>
    void addVariable(std::shared_ptr<T>& var) {
      _assigners.push_back(new SharedPtrPlaceHolderAssigner<T>(*getContext(),var));
    }

    template <typename T>
    void addVariable(std::weak_ptr<T>& var) {
      _assigners.push_back(new WeakPtrPlaceHolderAssigner<T>(*getContext(),var));
    }

    void resolve(Identifiable* instance);
    virtual ~IdPlaceholder();

    friend class IdContext;

  protected:
    IdPlaceholder(int id, IdContext* context): Identifiable(id, context) {}
  
    std::vector<AbstractPlaceHolderAssigner*> _assigners;
  };

}
