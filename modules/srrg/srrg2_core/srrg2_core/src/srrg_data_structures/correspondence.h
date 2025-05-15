#pragma once
#include <vector>

namespace srrg2_core {

  //! bookkeeping of the corespondence
  struct Correspondence {
    int fixed_idx  = -1;
    int moving_idx = -1;
    float response;

    Correspondence(int fixed_idx_ = -1, int moving_idx_ = -1, float response_ = 0.f) :
      fixed_idx(fixed_idx_),
      moving_idx(moving_idx_),
      response(response_) {
    }

    inline bool operator<(const Correspondence& other) const {
      return (fixed_idx < other.fixed_idx) ||
             (fixed_idx == other.fixed_idx && moving_idx < other.moving_idx);
    }
  };

  using CorrespondenceVector = std::vector<Correspondence>;
} // namespace srrg2_core
