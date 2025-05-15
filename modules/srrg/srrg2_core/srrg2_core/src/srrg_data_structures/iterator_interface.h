#pragma once
namespace srrg2_core {

  // interface for a factor container to be used by the solver
  // it can map to a standard container, or it can do something funnier
  template <typename BaseType_>
  struct IteratorInterface_{
    using BaseType=BaseType_;

    // sets the iterator to the beginning
    virtual void setBegin() = 0;

    // true if end is reached
    virtual bool isEnd() = 0;
    // number of elements to iterate

    virtual size_t size() = 0;

    // gets the current element
    virtual BaseType& get() = 0;

    // increments the iterator
    virtual IteratorInterface_<BaseType>& next() = 0;
    
    // gets the next and increments the pointer
    // returns false if at the end of the container
    virtual bool getNext(BaseType& datum) = 0;
  };
  
  // adaptor for a simple container
  template <typename ContainerType_>
  struct ContainerIteratorInterface_:
    public IteratorInterface_<typename ContainerType_::value_type> {
    using ContainerType=ContainerType_;
    using BaseType=typename ContainerType_::value_type;
    using value_type = BaseType; //< stl compat
    
    ContainerIteratorInterface_(ContainerType& container_):
      _container(container_) 
    {}

    void setBegin() override {
      _iterator=_container.begin();
    }

    // true if end is reached
    bool isEnd() override {
      return _iterator==_container.end();
    }
    
    // number of elements to iterate
    virtual size_t size() override {
      return _container.size();
    }

    // gets the current element
    virtual value_type& get() override {
      return *_iterator;
    }

    // increments the iterator
    IteratorInterface_<BaseType>& next() override {
      ++_iterator;
      return *this;
    }

    
    // gets the current element, and increments the iterator
    virtual  bool getNext(BaseType& v) override {
      if (_iterator == _container.end())
        return false;
      v=*_iterator;
      ++_iterator;
      return true;
    }

  protected:
    typename ContainerType_::iterator _iterator;
    ContainerType& _container;
  };

  
}
