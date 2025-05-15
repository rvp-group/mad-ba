#include "viewer_manager_ros.h"

namespace srrg2_core_ros {

  using namespace srrg2_core;

  ViewerManagerRos::ViewerManagerRos(ros::NodeHandle* node_)
  {
    if (!node_)
      throw std::runtime_error("ViewerManagerRos::ViewerManagerRos|invalid node, exit,");
    _ros_node = node_;
  }


  ViewerManagerRos::~ViewerManagerRos() {}


  const ViewerCanvasPtr& ViewerManagerRos::getCanvas(const std::string& context_name_) {
    //ia SERVER SIDE
    ViewerContextRos* context = 0;
    ViewerContextContainer::iterator it=_contexts.find(context_name_);
    if (it != _contexts.end()) {
      std::cerr << "ViewerManagerRos::getCanvas|found a context with name: " << FG_CYAN(context_name_) << std::endl;

      //ia check if this context fullfills our requirements
      if (it->second->status() != ViewerContextBase::ViewerContextStatus::Active)
        throw std::runtime_error("ViewerManagerRos::getCanvas|unexpected context status, exit");
      if (it->second->type() != ViewerContextBase::ViewerContextType::ROS)
        throw std::runtime_error("ViewerManagerRos::getCanvas|unexpected context type, exit");

      context = dynamic_cast<ViewerContextRos*>(it->second);
      return context->canvas();
    }

    //ia if we do not have this shitty context, we create one
    context = new ViewerContextRos(context_name_, this, param_max_num_buffers.value(), param_buffer_size.value(), _ros_node);

    _contexts.insert(std::make_pair(context_name_, context));

    //ia setup of all shits
    context->setup();
    context->_setupPublisher(context_name_);

    std::cerr << "ViewerManagerRos::getCanvas|created a context with name "
              << FG_CYAN(context_name_)
              << " published on topic " << FG_CYAN(context_name_) << std::endl;
    return context->canvas();
  }


  void ViewerManagerRos::bindViewport(srrg2_core::ViewportBase* viewport_,
                                      const std::string& context_name_) {
    //ia CLIENT SIDE
    //ia TODO check that everithing is fine before
    //ia ask to ros all the topics and check whether our topic exists
    //ia if the context doesnt exist create one.
    ViewerContextRos* context = 0;
    ViewerContextContainer::iterator it=_contexts.find(context_name_);
    if(it!=_contexts.end()){
      std::cerr << "ViewerManagerRos::bindViewport|found a context with name: " << FG_CYAN(context_name_) << std::endl;

      //ia check if this context fullfills our requirements
      if (it->second->status() != ViewerContextBase::ViewerContextStatus::Active)
        throw std::runtime_error("ViewerManagerRos::bindViewport|unexpected context status, exit");
      if (it->second->type() != ViewerContextBase::ViewerContextType::ROS)
        throw std::runtime_error("ViewerManagerRos::bindViewport|unexpected context type, exit");

      context = dynamic_cast<ViewerContextRos*>(it->second);
    } else {
      context = new ViewerContextRos(context_name_, this, param_max_num_buffers.value(), param_buffer_size.value(), _ros_node);
    }


    if (!context)
      throw std::runtime_error("ViewerManagerRos::bindViewport|unable to get a context, exit");
    context->_setupSubscriber(context_name_);

    viewport_->param_source.setValue(context->source());

    //ia adjust the status
    std::cerr << "ViewerManagerShared::bindViewport|context "
              << FG_CYAN(context_name_) << " has been binded to viewport ["
              << FG_CYAN((size_t)viewport_) << "]" << std::endl;

    context->_status = ViewerContextBase::ViewerContextStatus::Active;
  }



  void ViewerManagerRos::unbindViewport(srrg2_core::ViewportBase* viewport_,
                                        const std::string& context_name_) {

  }

} //ia end namespace srrg2_core_ros
