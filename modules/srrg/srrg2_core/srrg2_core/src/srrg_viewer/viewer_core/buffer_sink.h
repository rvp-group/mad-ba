#pragma once
#include <atomic>
// ia properties stuff
#include "srrg_config/property_configurable.h"
#include "buffer_memory.h"

namespace srrg2_core {
  class BufferSinkBase : public Configurable {
  public:
    BufferSinkBase() {
      _is_active = false;
    }
    virtual ~BufferSinkBase() {
      _is_active = false;
    }
    //! @brief non blocking funcktion that will do something with
    //!        a full buffer - e.g. send it to a socket - and then
    //!        destroys it
    virtual void putBuffer(BufferMemory* buffer_) = 0;

    //! @brief this is a a pure virtual method that checks if connection
    //!        to the viewport is still up, modifying the variable _is_active if needed.
    //!        In this sense it will do this:
    //!        - SHARED-SINK: nothing since connection is always up
    //!        - ROS-SINK   : checks is ros is still ok and if there are
    //!                       subscribers.
    virtual void checkConnection() = 0;

    //! @brief tells if this sink is active. Depending on the
    //!        type of sink, we must check different things
    inline const bool isActive() const {
      return _is_active;
    }
    inline void setIsActive(const bool& flag_) {
      _is_active = flag_;
    }

  protected:
    std::atomic<bool> _is_active;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
  };

  using BufferSinkBasePtr = std::shared_ptr<BufferSinkBase>;
} // namespace srrg2_core
