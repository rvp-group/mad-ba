#pragma once
#include <srrg_viewer/viewer_core/buffer_sink.h>
#include <srrg_viewer/viewer_core/buffer_manager.h>

#include "buffer_publisher.h"

namespace srrg2_core_ros {

  class BufferSinkRos : public srrg2_core::BufferSinkBase {
  public:

    PARAM(srrg2_core::PropertyConfigurable_<srrg2_core::BufferManager>,
          manager_ptr,
          "buffer manager that gives us full buffers - ready to be rendered", 0,0);
    PARAM(srrg2_core::PropertyConfigurable_<BufferPublisher>,
          publisher_ptr,
          "publisher that will publish the buffer through ros", 0,0);

    virtual ~BufferSinkRos();

    void checkConnection() override {
      if (!param_publisher_ptr.value() || !ros::ok())
        _is_active = false;

      if (param_publisher_ptr->numSubscribers() > 0)
        _is_active = true;
      else
        _is_active = false;
    }

    void putBuffer(srrg2_core::BufferMemory* buffer_) override;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  };

using BufferSinkRosPtr = std::shared_ptr<BufferSinkRos>;

} //ia end namespace srrg2_core_ros

