#pragma once
// ia properties stuff
#include "srrg_config/property_configurable.h"

#include "buffer_deque.h"
#include "buffer_memory.h"

namespace srrg2_core {

  //! @brief This class will allocate buffer for you.
  //!        May be useful, may contain also the semaphore, who knows
  //!        allocates a tot of buffer in a ring buffer and whenever you say
  //!        "gimme a buffer" I will clean one buffer e te lo do
  class BufferManager : public Configurable {
  public:
    PARAM(PropertyUnsignedInt,
          max_buffer_size,
          "max dimension of the buffer used",
          BUFFER_SIZE_1MEGABYTE,
          0);
    PARAM(PropertyUnsignedInt, max_num_buffers, "max number of buffer available", 2, 0);

  public:
    virtual ~BufferManager();

    //! @brief initializes the ring buffer
    void init();

    //! @brief takes a buffer from the ring, cleans it, changes the buffer state and gives it to you
    //! @param read_flag: true for reading
    BufferMemory* getBuffer(const bool read_flag_ = false);

    BufferMemory* getBufferTimeout(const bool read_flag_ = false);

    //! @brief you release a buffer. depending on what is its state
    //!        it will be in another state.
    //!        read  -> free and clears the buffer
    //!        write -> ready
    //!        free  -> not possible
    //!        ready -> not possible
    void releaseBuffer(BufferMemory* buffer_);

    //! @brief releases and clears a buffer. no matter what is
    //!        its current state, its next one is Free.
    void freeBuffer(BufferMemory* buffer_);

  protected:
    BufferMemoryVector _buffers;
    BufferSynchedDeque* _free_buffers  = 0;
    BufferSynchedDeque* _ready_buffers = 0;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
  };

  using BufferManagerPtr = std::shared_ptr<BufferManager>;

} // namespace srrg2_core
