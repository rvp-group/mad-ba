#pragma once
#include "configurable.h"
#include "preemptible.h"
#include "srrg_messages/message_handlers/message_source_base.h"
#include "srrg_messages/message_handlers/message_sink_base.h"
#include <mutex>
namespace srrg2_core {
  struct PipelineRunner: public MessageSinkBase, public Preemptible {
    PARAM(PropertyConfigurable_<MessageSourceBase>, source, "the source of the pipeline", nullptr, nullptr);
    void compute() override ;
    void reset() override;
    std::mutex& computeMutex() {return _compute_mutex;}
  protected:
    std::mutex _compute_mutex;
  };

  void srrg2_core_registerRunner() __attribute__((constructor));
}
