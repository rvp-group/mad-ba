#include "viewer_context_base.h"

namespace srrg2_core {

  ViewerContextBase::ViewerContextBase(const std::string& context_name_,
                                       ViewerManagerBase* viewer_manager_,
                                       const size_t& num_buffers_,
                                       const size_t& buffer_size_) :
                                         _context_name(context_name_) {
    if (!viewer_manager_)
      throw std::runtime_error("ViewerContextBase::ViewerContextBase|invalid viewer manager, quit");

    //ia basic stuff
    _viewer_manger = viewer_manager_;

    //ia setup memory
    _buffer_manager.reset(new BufferManager);
    _buffer_manager->param_max_buffer_size.setValue(buffer_size_);
    _buffer_manager->param_max_num_buffers.setValue(num_buffers_);
    _buffer_manager->init();

    //ia allocate a goddamn canvas
    _canvas.reset(new ViewerCanvas());

    //ia done here
  }
  ViewerContextBase::~ViewerContextBase() {
    //ia evertying is a sharedptr so we do not have to do anything
    _status = ViewerContextStatus::Invalid;
  }

} //ia end namespace srrg2_core
