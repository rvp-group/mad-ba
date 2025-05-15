#pragma once
#include <cassert>
#include <cstdint>
#include <memory>
#include <typeinfo>

// the motha of all item containers
namespace srrg2_core {

  // base class for an IMPLEMENTATION of an abstract iterator
  // the user does not see this
  template <typename KeyType_, typename ValueType_>
  struct AbstractMapIteratorImplBase_ {
    using KeyType   = KeyType_;
    using ValueType = ValueType_;
    using ThisType  = AbstractMapIteratorImplBase_<KeyType, ValueType>;
    // returns a copy of self
    virtual ThisType* clone() const = 0;

    // true if this is equals to other
    virtual bool equals(const ThisType& other) = 0;

    // increments
    virtual ThisType& operator++() = 0;

    // returns the ID
    virtual const KeyType& key() const = 0;

    // returns the reference to the value
    virtual ValueType& value() = 0;

    // returns the reference to the value
    virtual const ValueType& value() const = 0;
};

  template <typename KeyType_, typename ValueType_>
  struct AbstractMapView_;

  // concrete iterator implementation
  // this is visible in the container
  template <typename KeyType_, typename ValueType_>
  class AbstractIterator_ {
  public:
    using KeyType              = KeyType_;
    using ValueType            = ValueType_;
    using AbstractIteratorImpl = AbstractMapIteratorImplBase_<KeyType, ValueType>;
    using ThisType             = AbstractIterator_<KeyType, ValueType>;
    using ContainerType        = AbstractMapView_<KeyType, ValueType>;
    friend struct AbstractMapView_<KeyType, ValueType>;

    AbstractIterator_(ContainerType* container_ = 0, AbstractIteratorImpl* impl_ = 0) {
      _container = container_;
      _instance.reset(impl_);
    }

    AbstractIterator_(const ThisType& other) {
      _container = other._container;
      if (!other._instance.get()) {
        _instance.reset(0);
        return;
      }
      _instance.reset(other._instance->clone());
    }

    inline const KeyType& key() const {
      return _instance->key();
    }

    ValueType& value() {
      return _instance->value();
    }

    const ValueType& value() const {
      return _instance->value();
    }

    // wrapping in pair for stl compat
    std::pair<const KeyType&, ValueType&> operator*() {
      return std::pair<const KeyType&, ValueType&>(key(), value());
    }

    ThisType& operator=(const ThisType& other) {
      _container = other._container;
      if (!other._instance.get()) {
        _instance.reset(0);
      } else {
        _instance.reset(other._instance->clone());
      }
      return *this;
    }

    ThisType& operator++() {
      ++(*_instance);
      return *this;
    }

    bool operator==(const ThisType& other) const {
      if (other._container != _container) {
        return false;
      }
      // no type checking
      return _instance->equals(*other._instance);
    }

    bool operator!=(const ThisType& other) const {
      if (other._container != _container) {
        return true;
      }
      // no type checking
      return !_instance->equals(*other._instance);
    }

    const ContainerType* container() const {
      return _container;
    }

    const std::unique_ptr<AbstractIteratorImpl>& instance() const {
      return _instance;
    }

    std::unique_ptr<AbstractIteratorImpl>& instance() {
      return _instance;
    }
  protected:
    std::unique_ptr<AbstractIteratorImpl> _instance = nullptr;
    ContainerType* _container = nullptr;
  };

  template <typename KeyType_, typename ValueType_>
  class AbstractMapView_ {
  public:
    using KeyType   = KeyType_;
    using ValueType = ValueType_;
    using iterator  = AbstractIterator_<KeyType, ValueType_>;

    const iterator begin() const {
      return _begin;
    }

    const iterator end() const {
      return _end;
    }

    iterator begin() {
      return _begin;
    }

    iterator end() {
      return _end;
    }

    virtual iterator find(const KeyType&) = 0;
    virtual void erase(const iterator& it) = 0;
    virtual size_t size() const = 0;
    virtual void clear() = 0;
    virtual KeyType lastKey() const = 0;
    bool empty() const {
      return !size();
    }
    
  protected:
    virtual void updateBeginEnd() = 0;
    iterator _begin;
    iterator _end;
  };

} // namespace srrg2_core
