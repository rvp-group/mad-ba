#include "viewer_manager_base.h"

namespace srrg2_core {

  ViewerManagerBase::~ViewerManagerBase() {
    //ia delete all the contexts
    ViewerContextContainer::iterator c_it = _contexts.begin();
    ViewerContextContainer::iterator c_end = _contexts.end();

    while(c_it != c_end) {
      delete c_it->second;
      ++c_it;
    }

    _contexts.clear();
  }

} //ia end namespace srrg2_core
