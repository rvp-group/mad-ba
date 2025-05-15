#include "pipeline_runner.h"

namespace srrg2_core {

  void srrg2_core_registerRunner() {
    BOSS_REGISTER_CLASS(PipelineRunner);
  }

  void PipelineRunner::compute() {
    BaseSensorMessagePtr msg;
    while ((msg = param_source->getMessage()) || (!isFlushed())) {
      bool do_preempt = false;
      if (msg) {
        _compute_mutex.lock();
        do_preempt = putMessage(msg);
        _compute_mutex.unlock();
      } else {
        do_preempt = flush();
      }
      if (do_preempt)
        preemptLocal();
    }
  }

  void PipelineRunner::reset() {
    MessageSinkBase::reset();
    if (param_source.value())
      param_source->reset();
  }

} // namespace srrg2_core
