#pragma once
#include "abstract_map_view.h"
#include <map>

// the motha of all item containers
namespace srrg2_core {

  // getter class, retrieves a value from it->second
  template <typename ValueType_, typename ContainerType_>
  struct AbstractMapIteratorDefaultGetter_ {
    using ContainerIteratorType = typename ContainerType_::iterator;
    inline ValueType_& get(ContainerIteratorType& it) const {
      return it->second;
    }
    inline const ValueType_& get(const ContainerIteratorType& it) const {
      return it->second;
    }
  };

  template <typename KeyType_,
            typename ValueType_,
            typename ContainerType_,
            typename AbstractMapIteratorGetterType_ =
              AbstractMapIteratorDefaultGetter_<ValueType_, ContainerType_>>
  class AbstractMapIteratorImpl_ : public AbstractMapIteratorImplBase_<KeyType_, ValueType_> {
  public:
    using KeyType         = KeyType_;
    using ValueType       = ValueType_;
    using ContainerType   = ContainerType_;
    using GetterType      = AbstractMapIteratorGetterType_;
    using MapIteratorType = typename ContainerType_::iterator;
    using BaseType        = AbstractMapIteratorImplBase_<KeyType, ValueType>;
    using ThisType        = AbstractMapIteratorImpl_<KeyType, ValueType, ContainerType, GetterType>;

    AbstractMapIteratorImpl_(const MapIteratorType& it_) {
      _it = it_;
    }

    BaseType* clone() const override {
      return new ThisType(*this);
    }

    bool equals(const BaseType& other) override {
      const ThisType& other_ = reinterpret_cast<const ThisType&>(other);
      return other_._it == _it;
    }

    BaseType& operator++() override {
      ++_it;
      return *this;
    }

    const KeyType& key() const override {
      return _it->first;
    }

    ValueType& value() override {
      return getter.get(_it);
    }

    const ValueType& value() const override {
      return getter.get(_it);
    }

    const MapIteratorType& baseIterator() {
      return _it;
    }

  protected:
    MapIteratorType _it;
    GetterType getter;
  };

  template <typename KeyType_,
            typename ValueType_,
            typename BaseContainerType_ = std::map<KeyType_, ValueType_>,
            typename GetterType_ =
              AbstractMapIteratorDefaultGetter_<ValueType_, std::map<KeyType_, ValueType_>>>
  class AbstractMap_ : public AbstractMapView_<KeyType_, ValueType_> {
  public:
    using KeyType       = KeyType_;
    using ValueType     = ValueType_;
    using iterator      = AbstractIterator_<KeyType, ValueType_>;
    using BaseContainer = BaseContainerType_;
    using GetterType    = GetterType_;
    using IteratorImpl  = AbstractMapIteratorImpl_<KeyType, ValueType, BaseContainer, GetterType>;

    AbstractMap_() {
      updateBeginEnd();
    }

    iterator find(const KeyType& key) override {
      typename BaseContainer::iterator _it = _base_container.find(key);
      return iterator(this, new IteratorImpl(_it));
    }

    template <typename T>
    bool insert(T item) {
      typename BaseContainer::iterator _it = _base_container.find(item.first);
      if (_it != _base_container.end()) {
        return false;
      }
      _base_container.insert(item);
      //_base_container.insert(std::make_pair(item.first, item.second));
      updateBeginEnd();
      return true;
    }

    void erase(const iterator& it) override {
      if (it.container() != this) {
        return;
      }
      IteratorImpl* it_impl = dynamic_cast<IteratorImpl*>(it.instance().get());
      // std::pair <typename BaseContainer::iterator, bool> retval=
      _base_container.erase(it_impl->baseIterator());
      updateBeginEnd();
    }

    size_t size() const override {
      return _base_container.size();
    }

    void clear() override {
      _base_container.clear();
      updateBeginEnd();
    }

    BaseContainer& container() {return _base_container;}
    const BaseContainer& container() const {return _base_container;}

    KeyType lastKey() const override {
      return _base_container.rbegin()->first;
    }

  protected:
    void updateBeginEnd() override {
      this->_begin = iterator(this, new IteratorImpl(_base_container.begin()));
      this->_end   = iterator(this, new IteratorImpl(_base_container.end()));
    }

    BaseContainer _base_container;
  };

} // namespace srrg2_core
