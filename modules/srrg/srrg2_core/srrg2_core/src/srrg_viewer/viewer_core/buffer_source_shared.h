#pragma once
#include "buffer_source.h"
#include "buffer_manager.h"

namespace srrg2_core {

  //! @brief specialization that gets the buffer from another guy through tcp connection
  class BufferSourceShared : public BufferSourceBase {
  public:
    PARAM(PropertyConfigurable_<BufferManager>, manager_ptr, "buffer manager that gives us free buffers - ready to be written", 0, 0);

    virtual ~BufferSourceShared();

    //! @brief override of base method
    BufferMemory* getBuffer() override;

    //! @brief override of the base method
    void releaseBuffer(BufferMemory* buffer_) override;

    //! @brief waits for buffer until timeout happens, then returns 0
    BufferMemory* getBufferTimeout();

    //! @brief releases a buffer - after it has been written or read
    void pushBuffer(BufferMemory* buffer_);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using BufferSourceSharedPtr = std::shared_ptr<BufferSourceShared>;
  
} //ia end namespace
