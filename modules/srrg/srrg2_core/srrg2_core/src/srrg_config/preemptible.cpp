#include "preemptible.h"
#include <iostream>

namespace srrg2_core {
  using namespace std;

  std::unique_ptr<PreemptibleController> PreemptibleController::_instance = nullptr;

  struct sigaction PreemptibleController::_new_stop_action, PreemptibleController::_old_stop_action;

  void PreemptibleController::handleSigInt(int v __attribute__((unused))) {
    if (! instance())
      return;
    auto& controller = PreemptibleController::instance();
    if (!controller) {
      return;
    }
    auto runner = controller->_runner.lock();
    if (!runner) {
      return;
    }
    if (runner->_status==Preemptible::Running)
      runner->_status=Preemptible::Stopped;
  }

  void PreemptibleController::fini() {
    if (!_instance)
      return;
    int sigstop_result = sigaction(SIGINT, &_old_stop_action, nullptr);
    cerr << "PreemptibleController|fini, sa [" << sigstop_result << "]" << endl;
    _instance.reset(0);
  }

  void PreemptibleController::init(PreemptibleController* controller, bool handle_sigint) {
    if (_instance) {
      fini();
    }

    _instance.reset(controller);
    sigemptyset(&_new_stop_action.sa_mask);
    _new_stop_action.sa_flags   = SA_RESTART;
    _new_stop_action.sa_handler = PreemptibleController::handleSigInt;
    if (handle_sigint) {
      int sigstop_result = sigaction(SIGINT, &_new_stop_action, &_old_stop_action);
      cerr << "PreemptibleController |init, sigadded [" << sigstop_result << "]" << endl;
    } else {
      int sigstop_result = sigaction(SIGINT, nullptr, &_old_stop_action);
      cerr << "PreemptibleController|init, sig saved [" << sigstop_result << "]" << endl;
    }
  }

  bool PreemptibleController::run(shared_ptr<Preemptible> new_runner) {
    auto p = _runner.lock();
    if (p) {
      return false;
    }
    _runner=new_runner;
    auto runner=_runner.lock();
    runner->setController(*this);
    swapcontext(&_context, &runner->_context);
    return true;
  }

  void PreemptibleController::stop() {
    auto runner = _runner.lock();
    if (!runner) {
      return;
    }
    if (runner->status() == Preemptible::Stopped) {
      runner->stopCallback();
      swapcontext(&runner->_context, &_context);
    }
  }

  void PreemptibleController::resume() {
    auto runner=_runner.lock();
    if (! runner)
      return;
    if (runner->status() == Preemptible::Stopped) {
      runner->_status = Preemptible::Running;
      runner->resumeCallback();
      swapcontext(&_context, &runner->_context);
    }
  }

  void PreemptibleController::terminate() {
    auto runner = _runner.lock();
    if (!runner)
      return;
    if (runner->status() == Preemptible::Terminated)
      return;
    runner->_status = Preemptible::Terminated;
    runner->terminateCallback();
    _runner.reset();
  }

  void Preemptible::terminateCallback() {
  }
  void Preemptible::stopCallback() {
  }
  void Preemptible::resumeCallback() {
  }
  void Preemptible::endCallback() {
  }
  Preemptible::~Preemptible() {
    _controller = nullptr;
    _status     = Terminated;
    _stack.clear();
  }

  void Preemptible::preemptLocal(const char* message) {
    if (!_controller) 
      return;
    
    if (_status==Running && _controller->_step_mode)
      _status=Stopped;
    
    if (_status==Stopped) {
      if (message) {
        cerr << "(stop) [" << message << "]" << endl;
      }
      _controller->stop();
    }
  }

  void Preemptible::preemptGlobal(const char* message) {
    auto& controller = PreemptibleController::instance();
    if (!controller) 
      return;
    
    auto runner = controller->_runner.lock();
    if (!runner) 
      return;
    
    runner->preemptLocal(message);
  }
  
  void Preemptible::setController(PreemptibleController& controller_) {
    _controller = &controller_;
    getcontext(&_context);
    _stack.resize(_stack_size);
    _context.uc_stack.ss_sp   = &(_stack[0]);
    _context.uc_stack.ss_size = _stack_size;
    _context.uc_link          = &(_controller->_context);
    makecontext(&_context, (void (*)()) Preemptible::startCompute, 1, this);
  }

  void Preemptible::startCompute(Preemptible* p) {
    cerr << "startCompute" << endl;
    p->_status = Preemptible::Running;
    p->compute();
    p->endCallback();
    p->_status = Preemptible::Terminated;
    p->_controller->_runner.reset();
  }

  void Preemptible::reset() {
  }
} // namespace srrg2_core
