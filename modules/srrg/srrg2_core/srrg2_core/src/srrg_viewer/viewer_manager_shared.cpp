#include "viewer_manager_shared.h"
#include <iostream>

namespace srrg2_core {

  ViewerManagerShared::ViewerManagerShared() {
  }

  ViewerManagerShared::~ViewerManagerShared() {
  }

  const ViewerCanvasPtr& ViewerManagerShared::getCanvas(const std::string& context_name_) {
    // ia see if we already have this shitty context
    ViewerContextShared* context        = 0;
    ViewerContextContainer::iterator it = _contexts.find(context_name_);
    if (it != _contexts.end()) {
      std::cerr << "ViewerManagerShared::getCanvas|found a context with name [ "
                << FG_CYAN(context_name_) << " ]\n";

      // ia check if this context fullfills our requirements
      if (it->second->status() != ViewerContextBase::ViewerContextStatus::Active)
        throw std::runtime_error("ViewerManagerShared::getCanvas|unexpected context status, exit");
      if (it->second->type() != ViewerContextBase::ViewerContextType::Shared)
        throw std::runtime_error("ViewerManagerShared::getCanvas|unexpected context type, exit");

      context = dynamic_cast<ViewerContextShared*>(it->second);
      return context->canvas();
    }

    // ia if we do not have this shitty context, we create one
    context = new ViewerContextShared(
      context_name_, this, param_max_num_buffers.value(), param_buffer_size.value());

    _contexts.insert(std::make_pair(context_name_, context));

    // ia setup connections required for shared viewer
    context->setup();

    std::cerr << "ViewerManagerShared::getCanvas|created a context with name [ "
              << FG_CYAN(context_name_) << " ]\n";
    return context->canvas();
  }

  void ViewerManagerShared::bindViewport(ViewportBase* viewport_,
                                         const std::string& context_name_) {
    // ia see if we have this shitty context
    ViewerContextShared* context          = 0;
    ViewerContextContainer::iterator c_it = _contexts.find(context_name_);
    if (c_it == _contexts.end())
      throw std::runtime_error(
        "ViewerManagerShared::bindViewport|cannot find a context with this name, exit");
    if (c_it->second->type() != ViewerContextShared::ViewerContextType::Shared)
      throw std::runtime_error("ViewerContext::bindViewport|unexpected context type, exit");

    context = dynamic_cast<ViewerContextShared*>(c_it->second);

    // ia setup the viewport in shared mode
    viewport_->param_source.setValue(context->source());

    // ia a bit of bookkeeping
    context->_viewports.insert(viewport_);

    // ia adjust the status
    std::cerr << "ViewerManagerShared::bindViewport|context [ " << FG_CYAN(context_name_)
              << " ] has been binded to viewport [ " << FG_CYAN(viewport_) << " ]" << std::endl;
    context->_status = ViewerContextShared::ViewerContextStatus::Active;
    context->_sink->setIsActive(true);
  }

  void ViewerManagerShared::unbindViewport(ViewportBase* viewport_,
                                           const std::string& context_name_) {
    assert(viewport_ && "ViewerManagerShared::unbindViewport|invalid viewport, exit");

    ViewerContextShared* context          = 0;
    ViewerContextContainer::iterator c_it = _contexts.find(context_name_);

    if (c_it == _contexts.end())
      throw std::runtime_error(
        "ViewerManagerShared::unbindViewport|cannot find a context with this name, exit");
    if (c_it->second->type() != ViewerContextShared::ViewerContextType::Shared)
      throw std::runtime_error("ViewerContext::bindViewport|unexpected context type, exit");

    context = dynamic_cast<ViewerContextShared*>(c_it->second);

    // remove the viewport from the context
    ViewportBaseSet::iterator v_it = context->_viewports.find(viewport_);
    if (v_it == context->_viewports.end())
      throw std::runtime_error("ViewerManagerShared::unbindViewport|cannot find a viewport "
                               "attached to context [ " +
                               context_name_ + " ]");

    context->_viewports.erase(v_it);

    std::cerr << "\nViewerManagerShared::unbindViewport|viewport unbinded" << std::endl;
    if (!context->_viewports.size()) {
      // ia release the buffer source
      BufferSinkSharedPtr sink_ = std::dynamic_pointer_cast<BufferSinkShared>(context->_sink);
      sink_->param_source_ptr.setValue(BufferSourceSharedPtr(0));
      context->_sink->setIsActive(false);
      std::cerr << FG_YELLOW(
                     "ViewerManagerShared::unbindViewport|no viewport attached to context [ ")
                << context_name_ << FG_YELLOW(" ] -- buffers will be released from now on\n")
                << std::endl;
    }
  }

} // namespace srrg2_core
