#pragma once
#include <ros/master.h>
#include <srrg_viewer/viewer_manager_base.h>
#include "viewer_context_ros.h"
#include "viewer_core_ros/buffer_sink_ros.h"
#include "viewer_core_ros/buffer_source_ros.h"

namespace srrg2_core_ros {

  class ViewerManagerRos : public srrg2_core::ViewerManagerBase {
  public:
    PARAM(srrg2_core::PropertyString, rosmaster_uri, "URI of the rosmaster", "http://192.168.0.1:11311/", 0);

    ViewerManagerRos(ros::NodeHandle* node_ = nullptr);
    virtual ~ViewerManagerRos();

    //! @brief this creates a context with a specific name and returns a canvas.
    //!        this is called by the server (ONLY BY THE SERVER).
    const srrg2_core::ViewerCanvasPtr& getCanvas(const std::string& context_name_) override;

    //! @brief binds a viewport from a context_name
    //!        this is called by the client (ONLY BY THE CLIENT).
    //!        since this will be called from another process
    //!        if the context doesn't exist it will create one.
    void bindViewport(srrg2_core::ViewportBase* viewport_,
                      const std::string& context_name_) override;

    //! @brief unbinds a viewport
    //!        this is called by the client (ONLY BY THE CLIENT).
    void unbindViewport(srrg2_core::ViewportBase* viewport_,
                        const std::string& context_name_) override;

  protected:
    ros::NodeHandle* _ros_node = 0;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  };

} //ia end namespace srrg2_core_ros

