#pragma once
namespace srrg2_core {

  // wraps a generic vector, to allow polimorphic access
  template <typename BaseType_>
  struct IndexedContainerInterfaceBase_{
    using BaseType=BaseType_;

    virtual size_t size() = 0;

    // gets the current element
    virtual BaseType& at(size_t idx ) = 0;

    // gets the current element
    virtual const BaseType& const_at(size_t idx ) const = 0;
  };
  
  // adaptor for a simple container
  template <typename ContainerType_>
  struct IndexedContainerInterface_:
    public IndexedContainerInterfaceBase_<typename ContainerType_::value_type> {
    using ContainerType=ContainerType_;
    using BaseType=typename ContainerType_::value_type;
    using value_type = BaseType; //< stl compat
    
    IndexedContainerInterface_(const ContainerType& container_):
      _container(nullptr),
      _const_container(&container_)
    {}

    IndexedContainerInterface_(ContainerType& container_):
      _container(&container_),
      _const_container(&container_)
    {}

    
    // number of elements to iterate
    virtual size_t size() override {
      return _const_container->size();
    }

    // gets the current element
    virtual value_type& at(size_t idx) override {
      assert(_container && "non_const accessor not initialized");
      return _container->at(idx);
    }

    // gets the current element
    virtual const value_type& const_at(size_t idx) const override {
      return _const_container->at(idx);
    }

  protected:
    ContainerType* _container = nullptr;
    const ContainerType* _const_container = nullptr;
  };

  
}
