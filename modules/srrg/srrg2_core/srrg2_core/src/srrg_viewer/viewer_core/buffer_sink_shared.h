#pragma once
#include "buffer_sink.h"
#include "buffer_source_shared.h"

namespace srrg2_core {

  //! @brief specialization for tcp sink. It will send the buffer to another guy
  class BufferSinkShared : public BufferSinkBase {
  public:
    PARAM(PropertyConfigurable_<BufferManager>, manager_ptr, "buffer manager that gives us full buffers - ready to be rendered", 0, 0);
    PARAM(PropertyConfigurable_<BufferSourceShared>, source_ptr, "connected source that will receive the full buffer - shared memory", 0, 0);

    using ThisType = BufferSinkShared;
    using BaseType = BufferSinkBase;

    //! @brief ctor / dtor
    virtual ~BufferSinkShared();

    //! @brief check connections to the viewport. in this case
    //!        connection is always up so it will do nothin.
    void checkConnection() override {}

    //! @brief gives the buffer to the source
    void putBuffer(BufferMemory* buffer_) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using BufferSinkSharedPtr = std::shared_ptr<BufferSinkShared>;
  
} //ia end namespace 
