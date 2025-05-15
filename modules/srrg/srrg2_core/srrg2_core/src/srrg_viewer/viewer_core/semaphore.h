#pragma once
#include <atomic>
#include <deque>
#include <mutex>
#include <chrono>
#include <condition_variable>

namespace srrg2_core {
  class Semaphore {
  public:
    //! @brief default ctor-dtor
    Semaphore(const uint64_t& value_ = 0);
    ~Semaphore();

    //! @brief return semaphore value;
    inline const uint64_t value() {
      _mutex.lock();
      uint64_t c = _count;
      _mutex.unlock();
      return c;
    }

    //! @brief increments the value by one
    inline void post() {
      std::unique_lock<decltype(_mutex)> lock(_mutex);
      ++_count;
      _condition.notify_all();
    }

    //! @brief wait until the counter is up again
    inline void wait() {
      std::unique_lock<decltype(_mutex)> lock(_mutex);
      while(!_count) // Handle spurious wake-ups.
        _condition.wait(lock);
      --_count;
    }

    inline bool waitSeconds(const size_t& seconds_) {
      std::unique_lock<decltype(_mutex)> lock(_mutex);

      while (!_count) {
        auto status = _condition.wait_for(lock, std::chrono::milliseconds(1000*seconds_));
        if (status == std::cv_status::timeout) {
          return false;
        }
      }
      --_count;
      return true;
    }

  protected:
    uint64_t _count = 0;
    std::mutex _mutex;
    std::condition_variable _condition;
  };

} //ia end namespace
