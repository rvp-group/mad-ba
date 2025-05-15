#pragma once

namespace srrg2_solver {
  using namespace srrg2_core;

  // factor for a  variable problem  constructed out of error functions (ad and non ad)
  // usage:
  //   using MyConstraintFactor=ConstraintFactor_<MyErrorFunction>;

  // Ugly eh :))
  template <typename ThisType, int r, int c>
  struct ColUpdater {
    static inline void update(ThisType& factor) {
      factor.template _updateHBlock<r, c>();
      ColUpdater<ThisType, r, c - 1>::update(factor);
    }
  };

  template <typename ThisType, int r>
  struct ColUpdater<ThisType, r, r> {
    static inline void update(ThisType& factor) {
      factor.template _updateHBlock<r, r>();
    }
  };

  template <typename ThisType, int r, int c>
  struct RowUpdater {
    static inline void update(ThisType& factor) {
      ColUpdater<ThisType, r, c>::update(factor);
      RowUpdater<ThisType, r - 1, c>::update(factor);
      factor.template _updateBBlock<r>();
    }
  };

  template <typename ThisType, int c>
  struct RowUpdater<ThisType, 0, c> {
    static inline void update(ThisType& factor) {
      ColUpdater<ThisType, 0, c>::update(factor);
      factor.template _updateBBlock<0>();
    }
  };
}
