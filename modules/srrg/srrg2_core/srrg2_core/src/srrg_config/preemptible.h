#pragma once
#include <ucontext.h>
#include <memory>
#include <signal.h>
#include <vector>

namespace srrg2_core {
  using namespace std;
  class Preemptible;
  class PreemptibleController {
  public:
    friend class Preemptible;
    static void init(PreemptibleController*,
                     bool handle_sigint);
    static void fini();
    
    bool run(std::shared_ptr<Preemptible> new_runner);
    void stop();    
    void resume();    
    void terminate();
    inline bool stepMode() {return _step_mode;}
    inline void setStepMode(bool step_mode_) {_step_mode=step_mode_;}
    static inline std::unique_ptr<PreemptibleController>& instance() {return _instance;}
    inline std::shared_ptr<Preemptible> getRunner() {return _runner.lock();}
    virtual ~PreemptibleController() = default;
  protected:
    std::weak_ptr<Preemptible> _runner;
    ucontext_t _context;
    volatile bool _step_mode=false;
    static void handleSigInt(int v);
    static std::unique_ptr<PreemptibleController> _instance;
    static struct sigaction _new_stop_action, _old_stop_action;
  };
    
  class Preemptible {
  public:
    enum Status {Running=0x1, Stopped=0x2, Terminated=0x3};
    virtual void compute() = 0;
    virtual ~Preemptible();
    static void preemptGlobal(const char* message=0); // call this to add a global preemption point anywhere
    void preemptLocal(const char* message=0);
    inline Status status() const { return _status;}
    virtual void reset();
  protected:
    friend class PreemptibleController;
    // call this to add a preemption point to the current class
    
    void setController(PreemptibleController& controller_);
    virtual void terminateCallback();
    virtual void stopCallback();
    virtual void resumeCallback();
    virtual void endCallback();
    static void startCompute(Preemptible* p);
    size_t _stack_size=8<<20;
    ucontext_t _context;
    std::vector<unsigned char> _stack;
    Status _status = Terminated;
    PreemptibleController* _controller=nullptr;
    
  };

  using PreemptiblePtr = std::shared_ptr<Preemptible>;
}
