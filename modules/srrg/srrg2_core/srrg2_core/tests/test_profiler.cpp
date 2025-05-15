#include "srrg_system_utils/profiler.h"
#include "srrg_test/test_helper.hpp"

using namespace srrg2_core;

// ds the object we want to profile
class Foo : public Profiler {
public:
  Foo() {
    PROFILE_TIME("Foo::Foo");
    usleep(10000);
  }
  ~Foo() {
    PROFILE_TIME("Foo::~Foo");
    usleep(20000);
  }
  void bar() {
    PROFILE_TIME("Foo::bar");
    usleep(30000);
  }
};

int main(int argc_, char** argv_) {
  Profiler::enable_logging = true;
  return srrg2_test::runTests(argc_, argv_);
}

// ds TODO enable actual unittests
TEST(Profiler, ObjectLife) {
  Foo foo;
}

TEST(Profiler, ObjectFunctionSingle) {
  Foo foo;
  foo.bar();
}

TEST(Profiler, ObjectFunctionMultiple) {
  Foo foo;
  for (size_t i = 0; i < 10; ++i) {
    foo.bar();
  }
}
