#pragma once
#include <srrg_viewer/viewer_core/buffer_source.h>
#include <srrg_viewer/viewer_core/buffer_manager.h>

#include "buffer_subscriber.h"

namespace srrg2_core_ros {

  class BufferSourceRos : public srrg2_core::BufferSourceBase {
  public:
    PARAM(srrg2_core::PropertyConfigurable_<srrg2_core::BufferManager>,
          manager_ptr,
          "buffer manager that gives us full buffers - ready to be rendered", 0,0);
    PARAM(srrg2_core::PropertyConfigurable_<BufferSubscriber>,
          subscriber_ptr, "subscriber that receives ros buffer messages", 0,0);

  public:
    virtual ~BufferSourceRos();

    //! @brief this is blocking function that will be specialized
    //!        depending on where does the buffer come from (socket, shmem, things)
    srrg2_core::BufferMemory* getBuffer() override;

    //! @brief this will do something with a used buffer
    //!        depending on where does the buffer come from (socket, shmem, things)
    void releaseBuffer(srrg2_core::BufferMemory* buffer_) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  };

  using BufferSourceRosPtr = std::shared_ptr<BufferSourceRos>;

} //ia end namespace srrg2_core_ros

