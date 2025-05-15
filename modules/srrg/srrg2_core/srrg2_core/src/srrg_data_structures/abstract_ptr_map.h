#pragma once
#include "abstract_map.h"

namespace srrg2_core {
  // getter class, retrieves a value from it->second, with managed pointers
  template <typename ValueType_, typename ContainerType_>
  struct AbstractMapIteratorPtrGetter_ {
    using ContainerIteratorType = typename ContainerType_::iterator;
    inline ValueType_& get(ContainerIteratorType& it) const {
      _returned_value = it->second.get();
      return _returned_value;
    }

    inline const ValueType_& get(const ContainerIteratorType& it) const {
      _returned_value = it->second.get();
      return _returned_value;
    }

  protected:
    mutable  ValueType_ _returned_value = 0;
  };

  // // inserter that creates a managed pointer from a pointer
  // template <typename ValueType_, typename BaseContainerType_>
  // struct AbstractMapPtrInserter_ {
  //   using BaseContainerType         = BaseContainerType_;
  //   using KeyType                   = typename BaseContainerType::key_type;
  //   using MapValueType              = typename BaseContainerType::mapped_type;
  //   using ValueType                 = ValueType_;
  //   using BaseContainerIteratorType = typename BaseContainerType_::iterator;
  //   static inline void insert(BaseContainerType_& dest,
  //                             const std::pair<KeyType, const ValueType*>& item) {
  //     ValueType* v = item.second;
  //     dest.insert(std::make_pair(item.first, MapValueType(v)));
  //   }
  // };

  
  template <typename KeyType_,
            typename ValueType_,
            typename PointerType_
            >
  using AbstractPtrMap_=AbstractMap_<KeyType_,
                                     ValueType_  *,
                                     std::map<KeyType_, PointerType_>,
                                     AbstractMapIteratorPtrGetter_<ValueType_   *,
                                                                   std::map<KeyType_, PointerType_>
                                                                   > >;
}
