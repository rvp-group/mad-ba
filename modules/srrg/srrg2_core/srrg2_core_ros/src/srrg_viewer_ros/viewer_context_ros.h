#pragma once
#include <srrg_viewer/viewer_context_base.h>
#include "viewer_core_ros/buffer_sink_ros.h"
#include "viewer_core_ros/buffer_source_ros.h"

namespace srrg2_core_ros {
  class ViewerContextRos : public srrg2_core::ViewerContextBase {
  public:

    using IPAddress = std::string;
    using IPAddressVector = std::vector<IPAddress>;

    //! @brief creates connection between buffer manager and the canvas only
    void setup() override;

    //! @brief inline getters
    inline const IPAddressVector& subscriberIPAddresses() const {return _subscribers_ip_addresses;}
    inline const size_t& numSubscribers() const {return _num_subscribers;}
    inline const IPAddress& rosmasterIPAddress() const {return _rosmaster_ip_address;}
    inline const std::string& rosmasterURI() const {return _rosmaster_uri;}

  protected:
    //! @brief protected ctor/dtor - only ViewerManagers can create this thing
    //!        it allocates all the structure needed and configures the memory stuff
    //! @param[in] context_name - required
    //! @param[in] manager_ - pointer to the creator - required
    //! @param[in, optional] num_buffers_ - number of buffers for this context, default 5
    //! @param[in, optional] buffer_size_ - buffer size for this context, default 1MB
    ViewerContextRos() = delete;
    ViewerContextRos(const std::string& context_name_,
                     srrg2_core::ViewerManagerBase* manager_,
                     const size_t& num_buffers_ = 5,
                     const size_t& buffer_size_ = BUFFER_SIZE_1MEGABYTE,
                     ros::NodeHandle* node_ = 0);
    virtual ~ViewerContextRos();

    void _setupPublisher(const std::string& topic_name_ = "/buffer_topic",
                         const std::string& frame_id_name_ = "/map");
    void _setupSubscriber(const std::string& topic_name_ = "/buffer_topic");

    //! @brief IP addresses of the subscribers - maybe will be removed
    IPAddressVector _subscribers_ip_addresses;

    //! @brief number of connected subscribers
    size_t _num_subscribers;

    //! @brief IP address of the master
    IPAddress _rosmaster_ip_address;

    //! @brief URI of the ROS master in this setup
    std::string _rosmaster_uri;

    //! @brief pointer to the node
    ros::NodeHandle* _node = 0;

    //! @brief publisher and subscriber
    BufferPublisherPtr  _publisher;
    BufferSubscriberPtr _subscriber;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    //ia teletubbies friendship
    friend class ViewerManagerRos;
  };

} //ia end namespace srrg2_core_ros

