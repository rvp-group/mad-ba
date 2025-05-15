#pragma once
#include <srrg_qgl_viewport/qgl_viewport.h>
#include <srrg_qgl_viewport/viewer_core_base_qgl.h>
#include <srrg_viewer_ros/viewer_manager_ros.h>
#include <thread>

namespace srrg2_qgl_viewport_ros {

  //! @brief lazy aggragation of all the things needed for a
  //!        shared-qgl-based viewer.
  //!        Things You need to do:
  //!        1. contruct a stub.
  //!        2. request a canvas an give it to you Processing Module
  //!        3. start your processing thread (ALWAYS IN ANOTHER THREAD)
  //!        4. call the start method.
  //!        NB: the canvas name corresponds to the topic of that canvas
  class ViewerCoreRosQGL : public srrg2_qgl_viewport::ViewerCoreBaseQGL {
  public:
    //! @brief disabilitate empty ctor. You must sprecify the arguments
    ViewerCoreRosQGL() = delete;

    //! @brief creates a viewer manager and setups all the parameters
    ViewerCoreRosQGL(int argc_,
                     char** argv_,
                     QApplication* qapp_,
                     const std::string& node_name_,
                     const size_t& buffer_size_            = BUFFER_SIZE_2MEGABYTE,
                     const size_t& buffers_num_            = 2,
                     const size_t& opengl_frame_time_ms_   = 25,
                     const size_t& ros_subscriber_rate_ms_ = 1000,
                     const bool install_sigint_handler_    = false);

    //! @brief destroyes the viewport and releases resources
    virtual ~ViewerCoreRosQGL();

    //! @brief starts comm for the rosserver until stop is requested - automatically done
    //!        just for symmetry :)
    void startViewerServer() override;

    //! @brief starts comm for the rosclient and the rendering thread until stop is requested
    //! @param[in] canvas_name_, name of the canvas (and so also the topic) that we want to render
    void startViewerClient(const std::string& canvas_name_);

  protected:
    //! @brief node name - unique
    const std::string _node_name;

    //! @brief ros node and rate
    ros::NodeHandle* _node = 0;
    ros::Rate* _rate       = 0;

    //! @brief rate of the listner thread
    size_t _ros_subscriber_rate_ms;

    //! @brief if true intercepts sigint to properly shutdown everything
    const bool _install_sighandler;

  protected:
    //! @brief client comm function - 1Hz should be enough to avoid comm shutdown
    static void receive(const size_t& rate_) {
      while (ros::ok() && _run) {
        std::this_thread::sleep_for(std::chrono::milliseconds(rate_));
        ros::spinOnce();
      }
    }

    //! @brief sigint handler, shuts down also ros
    static void sigIntHandler(int __attribute__((unused))) {
      std::flush(std::cerr);
      std::cerr << FG_BRED("ViewerCoreRosQGL::sigIntHandler|terminating") << std::endl;
      std::flush(std::cerr);
      ViewerCoreBase::stop();
      if (ros::ok())
        ros::shutdown();
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

} // namespace srrg2_qgl_viewport_ros
