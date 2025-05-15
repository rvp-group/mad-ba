#pragma once
#include "field_pack.h"
#include <memory>

namespace srrg2_core {

  template <typename BaseFieldType_>
  struct IteratorImplBase_ {
    using BaseFieldType=BaseFieldType_;
    using ThisType=IteratorImplBase_<BaseFieldType>;
    virtual ThisType* clone() const = 0;
    virtual void operator++() = 0;
    virtual void setBegin() = 0;
    virtual void setEnd() = 0;
    virtual bool isBegin() const =0;
    virtual bool isEnd() const =0;
    virtual bool equals(const ThisType& other) const = 0;
    virtual BaseFieldType& operator*() = 0;

    virtual ~IteratorImplBase_() {
    };
    void* _raw_container_ptr;
  };

  template <typename BaseFieldType_, typename BaseContainerType_>
  struct IteratorImpl_: public IteratorImplBase_<BaseFieldType_> {
    using BaseFieldType=BaseFieldType_;
    using BaseContainerType=BaseContainerType_;
    using BaseIteratorType=typename BaseContainerType::iterator;
    using ThisType=IteratorImpl_<BaseFieldType, BaseContainerType>;

    IteratorImpl_(BaseContainerType* base_container):
      _base_container(base_container) {
      this->_raw_container_ptr = _base_container;
    };

    void operator++() override {
      ++_base_iterator;
    }

    IteratorImplBase_<BaseFieldType>* clone() const override {
      return new ThisType(*this);
    };

    void setBegin() override {
      if (_base_container) {
        _base_iterator = _base_container->begin();
      }
    }

    void setEnd() override {
      if (_base_container) {
        _base_iterator = _base_container->end();
      }
    }

    bool isBegin() const override {
      if (!_base_container) {
        return true;
      }
      return _base_iterator == _base_container->begin();
    }

    bool isEnd() const override {
      if (!_base_container) {
        return true;
      }
      return _base_iterator == _base_container->end();
    }

    bool equals(const IteratorImplBase_<BaseFieldType>& other_) const override {
      if (this->_raw_container_ptr != other_._raw_container_ptr) {
        return false;
      }
      const ThisType& other = reinterpret_cast<const ThisType&>(other_);
      return _base_iterator == other._base_iterator;
    }

    BaseFieldType& operator*() override {
      return *_base_iterator;
    }

    BaseContainerType* _base_container = 0;
    BaseIteratorType _base_iterator;
  };
  
  template <int i, typename ContainerPackIterator_>
  struct InstantiateContainerPackIterator_ {
    using ContainerPackIterator=ContainerPackIterator_;
    using BaseFieldType= typename ContainerPackIterator::BaseFieldType;
    using ContainerPackType= typename ContainerPackIterator::ContainerPackType;

    static void instantiate(ContainerPackIterator& it) {
      using ThisBaseContainerType=typename ContainerPackType::template TypeAt<i>;
      it._iterator_impls[i].reset(new IteratorImpl_<BaseFieldType, ThisBaseContainerType>
                                  (&it._container_pack->template field<i>()));
      InstantiateContainerPackIterator_<i - 1, ContainerPackIterator_>::instantiate(it);
    }
  };

  template <typename ContainerPackIterator_>
  struct InstantiateContainerPackIterator_<0, ContainerPackIterator_> {
    using ContainerPackIterator=ContainerPackIterator_;
    using BaseFieldType= typename ContainerPackIterator::BaseFieldType;
    using ContainerPackType= typename ContainerPackIterator::ContainerPackType;

    static void instantiate(ContainerPackIterator& it) {
      using ThisBaseContainerType=typename ContainerPackType::template TypeAt<0>;
      it._iterator_impls[0].reset(new IteratorImpl_<BaseFieldType, ThisBaseContainerType>
                                  (&it._container_pack->template field<0>()));
    }
  };

  template <typename ContainerPackType_>
  struct ContainerPackIterator_ {
    using BaseFieldType   = typename ContainerPackType_::BaseFieldType;
    using IteratorImplBase  = IteratorImplBase_<BaseFieldType>;
    using ContainerPackType = ContainerPackType_;
    using ThisType=ContainerPackIterator_<ContainerPackType>;
    int _current_idx = 0;
    friend ContainerPackType_;
    static constexpr int NumFields = ContainerPackType::NumFields;

    ContainerPackIterator_() {
      _container_pack = 0;
    }

    ContainerPackIterator_(const ThisType& other) {
      for (int i = 0; i < NumFields; ++i) {
        if (other._iterator_impls[i]) {
          _iterator_impls[i].reset(other._iterator_impls[i]->clone());
        }
      }
      _container_pack = other._container_pack;
      _current_idx=other._current_idx;
    }

    ThisType& operator=(const ThisType& other) {
      for (int i = 0; i < NumFields; ++i) {
        if (other._iterator_impls[i]) {
          _iterator_impls[i].reset(other._iterator_impls[i]->clone());
        } else
          _iterator_impls[i].reset(0);
      }
      _container_pack = other._container_pack;
      _current_idx=other._current_idx;
      return *this;
    }

    ThisType& operator++() {
      if (_current_idx==NumFields){
        setEnd();
        return *this;
      }
      ++(*_iterator_impls[_current_idx]);
      while (_current_idx < NumFields - 1
             && _iterator_impls[_current_idx]->isEnd()) {
        ++_current_idx;
        _iterator_impls[_current_idx]->setBegin();
      }
      if (_current_idx == NumFields-1
          && _iterator_impls[_current_idx]->isEnd()) {
        setEnd();
        return *this;
      }
      return *this;
    }

    bool operator==(const ThisType& other) {
      if (!_container_pack) {
        return false;
      }
      if (_container_pack != other._container_pack) {
        return false;
      }
      if (_current_idx != other._current_idx) {
        return false;
      }
      return _iterator_impls[_current_idx]->equals(*other._iterator_impls[_current_idx]);
    }

    bool operator!=(const ThisType& other) {
      return !((*this) == other);
    }

    BaseFieldType& operator*() {
      return *(*_iterator_impls[_current_idx]);
    }

    ContainerPackIterator_(ContainerPackType* container_pack) {
      _container_pack = container_pack;
      InstantiateContainerPackIterator_<ContainerPackType_::NumFields - 1, ThisType>::instantiate(*this);
    }

    //protected:

    std::unique_ptr<IteratorImplBase> _iterator_impls[ContainerPackType_::NumFields];
    ContainerPackType* _container_pack = 0;

    void setBegin() {
      _current_idx = 0;
      _iterator_impls[_current_idx]->setBegin();
    }

    void setEnd() {
      _current_idx = NumFields - 1;
      _iterator_impls[_current_idx]->setEnd();
    }

  };

  template <typename ContainerPackType_, int i>
  struct ContainerPackSize_ {
    static int size(const ContainerPackType_& pack) {
      return pack.template field<i>().size() + ContainerPackSize_<const ContainerPackType_, i - 1>::size(pack);
    }
  };

  template <typename ContainerPackType_>
  struct ContainerPackSize_<ContainerPackType_, 0> {
    static int size(const ContainerPackType_& pack) {
      return pack.template field<0>().size();
    }
  };

  template <typename ContainerPackType_, int i>
  struct ContainerPackClear_ {
    static void clear(ContainerPackType_& pack) {
      pack.template field<i>().clear();
      ContainerPackClear_<ContainerPackType_, i - 1>::clear(pack);
    }
  };

  template <typename ContainerPackType_>
  struct ContainerPackClear_<ContainerPackType_, 0> {
    static void clear(ContainerPackType_& pack) {
      pack.template field<0>().clear();
    }
  };

  template <typename BaseFieldType_, typename ... ContainerTypes_>
  class ContainerPack_: public FieldPack_<ContainerTypes_...> {
  public:
    using FieldPackType=FieldPack_<ContainerTypes_...>;
    using BaseFieldType = BaseFieldType_;
    using ThisType=ContainerPack_<BaseFieldType, ContainerTypes_...>;
    using iterator=ContainerPackIterator_<ThisType>;
    using value_type=BaseFieldType;
    
    iterator& begin() {
      return _begin;
    }

    iterator& end() {
      return _end;
    }

    ContainerPack_():
      _begin(this),
      _end(this) {
      //updateBeginEnd();
    }

    void updateBeginEnd() {
      _begin = iterator(this);
      _end = iterator(this);
      _begin.setBegin();
      _end.setEnd();
    }

    size_t size() const {
      return ContainerPackSize_<ThisType, FieldPackType::NumFields - 1>::size(*this);
    }

    void clear() {
      ContainerPackClear_<ThisType, FieldPackType::NumFields - 1>::clear(*this);
    }

  protected:

    iterator _begin;
    iterator _end;

  };
}
