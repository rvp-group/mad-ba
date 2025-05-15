#pragma once
#include <memory>

namespace srrg2_core {
  template <typename T>
  struct PointerTraits_{};

  template <typename T>
  struct PointerTraits_<T*> {
    using RawValueType   = T;
    using RawPointerType = T*;
    using ManagedPointerType = T*;
    static inline RawPointerType getRawPtr(ManagedPointerType t) {return t;}
    static inline std::shared_ptr<RawValueType> getSharedPtr(ManagedPointerType t) {
      throw std::runtime_error("cannot get a shared ptr from rawe ptr");
    }

    template <typename Other>
    static bool canAssign(Other* o) {
      if (!o)
        return true;
      return dynamic_cast<RawPointerType>(o);
    }

    template <typename Other>
    static bool canAssign(std::shared_ptr<Other> o) {
      return canAssign(o.get());
    }

    template <typename Other>
    static bool canAssign(std::weak_ptr<Other> o) {
      return canAssign(o.lock().get());
    }

    template <typename Other>
    static bool assign(ManagedPointerType& dest, Other* src) {
      if (!src) {
        dest=0;
        return true;
      }
      RawPointerType s=dynamic_cast<RawPointerType>(src);
      if (!s)
        return false;
      dest=s;
      return true;
    }

    template <typename Other>
    static bool assign(ManagedPointerType& dest, std::shared_ptr<Other> src) {
      return assign(dest, src.get());
    }

    template <typename Other>
    static bool assign(ManagedPointerType& dest, std::weak_ptr<Other> src) {
      return assign(dest, src.lock().get());
    }

  };

  template <typename T>
  struct PointerTraits_< std::shared_ptr<T> > {
    using RawValueType   = T;
    using RawPointerType = T*;
    using ManagedPointerType = std::shared_ptr<T>;
    static inline RawPointerType getRawPtr(ManagedPointerType t) {return t.get();}
    static inline std::shared_ptr<RawValueType> getSharedPtr(ManagedPointerType t) {
      return t;
    }
    static inline std::shared_ptr<RawValueType>& getSharedPtrRef(ManagedPointerType& t) {
      return t;
    }

    
    template <typename Other>
    static bool canAssign(Other* o) {
      throw std::logic_error("cannot assing a raw pointer to a managed pointer");
      return false;
    }

    template <typename Other>
    static bool canAssign(std::shared_ptr<Other> o) {
      if (!o)
        return true;
      return std::dynamic_pointer_cast<RawValueType>(o)!=0;
    }

    template <typename Other>
    static bool canAssign(std::weak_ptr<Other> o) {
      return canAssign(o.lock());
    }

    template <typename Other>
    static bool assign(ManagedPointerType& dest, Other* src) {
      throw std::logic_error("cannot assing a raw pointer to a managed pointer");
      return false;
    }

    template <typename Other>
    static bool assign(ManagedPointerType& dest, std::shared_ptr<Other> src) {
      if (!src) {
        dest.reset();
        return true;
      }
      ManagedPointerType s=std::dynamic_pointer_cast<RawValueType>(src);
      if (!s)
        return false;
      dest=s;
      return true;
    }

    template <typename Other>
    static bool assign(ManagedPointerType& dest, std::weak_ptr<Other> src) {
      return assign(dest, src.lock());
    }

  };

  template <typename T>
  struct PointerTraits_< std::weak_ptr<T> > {
    using RawValueType   = T;
    using RawPointerType = T*;
    using ManagedPointerType = std::weak_ptr<T>;
    static inline RawPointerType getRawPtr(ManagedPointerType t) {return t.lock().get();}
    static inline std::shared_ptr<RawValueType> getSharedPtr(ManagedPointerType t) {
      return t.lock();
    }
    static inline std::shared_ptr<RawValueType> getSharedPtrRef(ManagedPointerType t) {
      throw std::runtime_error("aaaargh");
      return t.lock();
    }


    template <typename Other>
    static bool canAssign(Other* o) {
      throw std::logic_error("cannot assing a raw pointer to a managed pointer");
      return false;
    }

    template <typename Other>
    static bool canAssign(std::shared_ptr<Other> src) {
      if (!src)
        return true;
      return std::dynamic_pointer_cast<RawValueType>(src)!=0;
    }

    template <typename Other>
    static bool canAssign(std::weak_ptr<Other> o) {
      return canAssign(o.lock());
    }


    template <typename Other>
    static bool assign(ManagedPointerType& dest, Other* src) {
      throw std::logic_error("cannot assing a raw pointer to a managed pointer");
      return false;
    }

    template <typename Other>
    static bool assign(ManagedPointerType& dest, std::shared_ptr<Other> src) {
      if (!src) {
        dest.reset();
        return true;
      }
      std::shared_ptr<RawValueType> s=std::dynamic_pointer_cast<RawValueType>(src);
      if (!s)
        return false;
      dest=s;
      return true;
    }

    template <typename Other>
    static bool assign(ManagedPointerType& dest, std::weak_ptr<Other> src) {
      return assign(dest, src.lock());
    }

  };

}
