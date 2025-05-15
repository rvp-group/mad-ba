#include "viewer_context_shared.h"

#include "viewer_core/buffer_sink_shared.h"
#include "viewer_core/buffer_source_shared.h"

namespace srrg2_core {
  ViewerContextShared::ViewerContextShared(const std::string& context_name_,
                                           ViewerManagerBase* manager_,
                                           const size_t& num_buffers_,
                                           const size_t& buffer_size_) :
                                               ViewerContextBase(context_name_,
                                                                 manager_,
                                                                 num_buffers_,
                                                                 buffer_size_)
  {

    //ia create shared memory substratum
    _type = ViewerContextType::Shared;
    _source.reset(new BufferSourceShared());
    _sink.reset(new BufferSinkShared());

    //ia jic
    _viewports.clear();

    //ia context is created but not binded
    _status = ViewerContextStatus::Inactive;
  }


  ViewerContextShared::~ViewerContextShared() {
    _viewports.clear();
    _status = ViewerContextStatus::Invalid;
  }


  void ViewerContextShared::setup() {
    if (_status != ViewerContextStatus::Inactive || _viewports.size())
      throw std::runtime_error("ViewerContextShared::setup|current context is already in use or not well formed, exit");

    if (_type != ViewerContextType::Shared)
      throw std::runtime_error("ViewerContextShared::setup|current context is not shared, exit");

    BufferSinkSharedPtr shared_sink = std::dynamic_pointer_cast<BufferSinkShared>(_sink);
    BufferSourceSharedPtr shared_source = std::dynamic_pointer_cast<BufferSourceShared>(_source);
    assert(shared_sink && shared_source && "ViewerContextShared::setup|unexpected error, exit");

    shared_sink->param_source_ptr.setValue(shared_source);
    shared_sink->param_manager_ptr.setValue(_buffer_manager);
    shared_source->param_manager_ptr.setValue(_buffer_manager);

    _canvas->param_buffer_sink_ptr.setValue(_sink);
    _canvas->param_buffer_manager_ptr.setValue(_buffer_manager);

    _status = ViewerContextStatus::Ready;
  }


} //ia end namespace srrg2_core
