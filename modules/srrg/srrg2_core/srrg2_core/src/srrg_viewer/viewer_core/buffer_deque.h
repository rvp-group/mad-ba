#pragma once
#include "semaphore.h"
#include "buffer_memory.h"

namespace srrg2_core {
  //! brief guarded queue
   struct BufferSynchedDeque {
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

     std::mutex mutex;
     Semaphore* semaphore_empty = 0;
     Semaphore* semaphore_full  = 0;
     BufferMemoryDeque buffers_ptr;

     BufferSynchedDeque(const size_t& max_deque_size_) {
       semaphore_empty = new Semaphore(max_deque_size_);
       semaphore_full = new Semaphore();
     }

     ~BufferSynchedDeque() {
       delete semaphore_empty;
       delete semaphore_full;
       buffers_ptr.clear();
     }

     BufferMemory* popFront() {
       semaphore_full->wait();
       mutex.lock();
       BufferMemory* buffer_ptr = buffers_ptr.front();
       buffers_ptr.pop_front();
       mutex.unlock();
       semaphore_empty->post();

       return buffer_ptr;
     }

     BufferMemory* popFrontTimeout(const size_t timeout_seconds_) {
       if(!semaphore_full->waitSeconds(timeout_seconds_))
         return 0;
       mutex.lock();
       BufferMemory* buffer_ptr = buffers_ptr.front();
       buffers_ptr.pop_front();
       mutex.unlock();
       semaphore_empty->post();

       return buffer_ptr;
     }

     void pushBack(BufferMemory* buffer_) {
       semaphore_empty->wait();
       mutex.lock();
       buffers_ptr.push_back(buffer_);
       mutex.unlock();
       semaphore_full->post();
     }
   };

} //ia end namespace
