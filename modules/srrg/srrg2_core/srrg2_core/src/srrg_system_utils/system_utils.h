#pragma once

#include <string>
#include <sys/resource.h>
#include <sys/time.h>

//! @Author Giorgio Grisetti
//! @date   December 2014
//! @brief  This file contains some common routines and classes to get the system time, to print a
//! banner on a command line and to monitor the process resource usage
namespace srrg2_core {

  extern int srrg_argc;
  extern char** srrg_argv;
  extern char** srrg_argv_with_pid;
  extern std::string srrg_opts;
  void srrgInit(int argc, char** argv, const char* opts = 0);

  //crawls for a file until the root of the filesystem returns the
  // path to the file if found
  std::string crawlForFile(const std::string plain_filename, const std::string& current_dir=".");
  
  //! @brief printing utility
  //! prints a null terminated array of char*.
  //! each item of the array prints on a separate line
  //! the last component of the array should be 0
  //! @param[in] banner the input char**
  void printBanner(const char** banner_);

  //! @brief returns the system time in seconds
  //! @return the system time in seconds
  double getTime();

  //! @brief returns the system time in seconds as formatted string HH:MM:SS.mmm
  //! @return the system time in seconds as formatted string HH:MM:SS.mmm
  std::string getTimestamp();

  //! @brief converts a system timeval to seconds
  //! @return converts a system timeval in seconds
  inline double tv2sec(const struct timeval& tv_) {
    return (static_cast<double>(tv_.tv_sec) + 1e-6 * static_cast<double>(tv_.tv_usec));
  }

  //! @brief removes '/' from beginning of input_string_
  //! @return converted string
  std::string removeStringTokens(const std::string& input_string_, bool at_begining_only_ = true);

  //! @class SystemUsageCounter
  //! @brief system usage analysis
  //! monitors the resources of a process between two updates
  //! declare one instance of this object in the process you want to monitor
  //! and call once in a while the update() function
  class SystemUsageCounter {
  public:
    //! @brief default constructor
    SystemUsageCounter();

    //! @brief system CPU usage
    //! @return system CPU usage in seconds
    inline double systemCPUUsage() const {
      return _system_cpu;
    }

    //! @brief user CPU usage
    //! @return user CPU usage in seconds
    inline double userCPUUsage() const {
      return _user_cpu;
    }

    //! @brief total CPU usage
    //! @return combined system and user CPU usage in seconds
    inline double totalCPUUsage() const {
      return _user_cpu + _system_cpu;
    }

    //! @brief total memory usage
    //! @return total memory used by the project
    inline int totalMemory() const {
      return _total_memory;
    }

    //! @brief start measuring time when called (NOT THREAD SAFE)
    static void tic() {
      _time_seconds_last_tic = getTime();
    };

    //! @brief compute time passed since the last tic() call in seconds (NOT THREAD SAFE)
    static double toc() {
      return (getTime() - _time_seconds_last_tic);
    };

    //! @brief call this once in a while to update the internal counters
    void update();

  protected:
    //! @brief timeval of previous update
    struct timeval _last_update_time;

    //! @brief CPU usage of previous update
    struct rusage _last_usage;

    //! @brief current system CPU usage
    double _system_cpu;

    //! @brief current user CPU usage
    double _user_cpu;

    //! @brief current memory usage
    int _total_memory;

    //! @brief time of last tic call (timing) (NOT THREAD SAFE)
    static double _time_seconds_last_tic;
  };

  //! @brief a utility function evaluating if an element (file or directory) on disk is accessible
  //! or not
  //! @param[in] file_or_directory_ a string containing the relative or absolute file path to the
  //! target element
  //! @return a boolean, telling whether the parameter is accessible by the program or not
  //! @warning unchecked behavior on multiple platforms
  const bool isAccessible(const std::string& file_or_directory_);
} // namespace srrg2_core
