#include "preemptible.h"
#include <iostream>
#include <unistd.h>

using namespace srrg2_core;
using namespace std;

// to run something in the controller, override the
// compute method and add preempt() in the preemption point
// optionally you might want to override some callback
// that is triggered by the events of the context

void foo() {
  cerr << "in foo" << endl;
  Preemptible::preemptGlobal("in global_preempt");
}

class ExampleContext : public Preemptible {
public:
  ExampleContext(const std::string& name_) : name(name_) {
  }

  std::string name;
  void compute() override {
    for (int i = 0; i < 100; ++i) {
      cerr << name << " " << i << endl;
      preemptLocal("in local preempt");
      sleep(1);
      foo();
      sleep(1);
    }
  }
};

const char* banner[] = {"h: help",
                        "r: runs a context",
                        "c: continues a stopped context",
                        "k: kills a running context",
                        "ctrl-c: stops a running context",
                        0};

int main(int argc, char** argv) {
  // starts a controller with signal handlers
  PreemptibleController::init(new PreemptibleController, true);
  auto& controller = PreemptibleController::instance();

  auto p   = std::shared_ptr<Preemptible>(new ExampleContext("pippo"));
  bool run = true;
  std::string token;
  cerr << "type h for help" << endl;
  while (cin && run) {
    cerr << "controller> ";
    cin >> token;
    if (token == "r") {
      cerr << "run" << endl;
      controller->run(p);
    }
    if (token == "h") {
      const char** c = banner;
      while (*c) {
        cerr << *c << endl;
        ++c;
      }
    }
    if (token == "c") {
      cerr << "resume" << endl;
      controller->resume();
    }
    if (token == "k") {
      cerr << "kill" << endl;
      controller->terminate();
    }
    if (token == "s") {
      controller->setStepMode(!controller->stepMode());
      cerr << "step mode: " << controller->stepMode() << endl;
    }
    if (token == "q") {
      run = false;
      cerr << "quitting" << endl;
    }
  }
}
