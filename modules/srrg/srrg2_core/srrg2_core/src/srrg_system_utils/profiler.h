#pragma once
#include <assert.h>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include <vector>

#include "srrg_geometry/geometry_defs.h"
#include "system_utils.h"

// ds functionality can be disabled at compile-time - default is active
#ifdef SRRG_DISABLE_PROFILING
#define PROFILE_TIME(NAME)
#else
#define PROFILE_TIME(NAME) Profiler::Timer timer(this, NAME)
#endif

namespace srrg2_core {

  //! RAII principle based profiling header-only structure for scopes
  //! timers measure their lifetime in a particular scope and store it in the profiler
  //! on destruction of the profiler (usually pipeline teardown) it logs all the timings
  //!
  //! HOWTO use this class
  //! 1. add the Profiler to the inherited class list of the class you want to profile
  //! 2. add the macro PROFILE_TIME("unique_and_useful_description"); to the scopes you want to time
  //! 3. enjoy automatic timing statistics upon destruction of the class
  //!
  //! EXAMPLE: test_profiler.cpp
  class Profiler {
  public:
    // ds print functionality can be toggled with this static global bool (ahem)
    static bool enable_logging;
    Profiler() {
    }
    ~Profiler();

    // ds RAII time measuring function, starts at construction, stops at destruction (scope)
    struct Timer {
      Timer() = delete;
      Timer(Profiler* handle_, const std::string& name_);
      ~Timer();
      Profiler* handle;
      std::string name;
      double start_time_seconds;
    };

  protected:
    std::unordered_map<std::string, std::vector<double>> _runtimes_per_module;
  };

} // namespace srrg2_core
