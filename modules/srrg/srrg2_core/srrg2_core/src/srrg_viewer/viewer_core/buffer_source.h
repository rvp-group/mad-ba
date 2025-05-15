#pragma once
//ia properties stuff
#include "srrg_property/property_container.h"
#include "srrg_config/configurable.h"
#include "srrg_config/property_configurable.h"

#include "packet_deserializer.h"
#include "packet_serializer.h"

namespace srrg2_core {
  class BufferSourceBase : public Configurable {
  public:
    BufferSourceBase() {}
    virtual ~BufferSourceBase() {}
    //! @brief this is blocking function that will be specialized
    //!        depending on where does the buffer come from (socket, shmem, things)
    virtual BufferMemory* getBuffer() = 0;

    //! @brief this will do something with a used buffer
    //!        depending on where does the buffer come from (socket, shmem, things)
    virtual void releaseBuffer(BufferMemory* buffer_) = 0;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using BufferSourceBasePtr=std::shared_ptr<BufferSourceBase>;
  
} //ia end namespace
