#include "buffer_manager.h"
//#include <iostream>

namespace srrg2_core {

  BufferManager::~BufferManager() {
    for (BufferMemory* buf : _buffers) {
      assert(buf && "[BufferMemory::~BufferManager]| wtf");
      delete buf;
    }

    if (_free_buffers)
      delete _free_buffers;
    if (_ready_buffers)
      delete _ready_buffers;
  }

  void BufferManager::init() {
    _buffers.resize(param_max_num_buffers.value());
    assert(_buffers.size() == param_max_num_buffers.value() && "[BufferManager::init]| error in resizing the buffer vector, exit");

    //ia bookeeping
    _free_buffers = new BufferSynchedDeque(param_max_num_buffers.value());
    _ready_buffers = new BufferSynchedDeque(param_max_num_buffers.value());

    for (size_t i = 0; i < param_max_num_buffers.value(); ++i) {
      _buffers[i] = new BufferMemory();
      _buffers[i]->allocate(param_max_buffer_size.value());
      _free_buffers->pushBack(_buffers[i]);
    }

  }

  BufferMemory* BufferManager::getBuffer(const bool read_flag_) {
    BufferMemory* buffer_ptr = 0;

    //ia I want a buffer for reading,
    //ia thus I must get the most recent buffer in READY status
    if (read_flag_) {
      buffer_ptr = _ready_buffers->popFront();

      assert(buffer_ptr && "[BufferManager::getBuffer]| element not initialized, exit");
      if (buffer_ptr->status != BufferMemory::BufferStatus::Ready)
        throw std::runtime_error("[BufferManager::getBuffer]| element has wrong status [FLAG=TRUE]");

      buffer_ptr->status = BufferMemory::BufferStatus::Read;
    } else {
      buffer_ptr = _free_buffers->popFront();

      assert(buffer_ptr && "[BufferManager::getBuffer]| element not initialized, exit");
//      std::cerr << "buffer_ptr->status=" << buffer_ptr->status << std::endl;
      if (buffer_ptr->status != BufferMemory::BufferStatus::Free)
        throw std::runtime_error("[BufferManager::getBuffer]| element has wrong status [FLAG=FALSE]");

      buffer_ptr->clear();
      buffer_ptr->status = BufferMemory::BufferStatus::Write;
    }


    return buffer_ptr;
  }


  BufferMemory* BufferManager::getBufferTimeout(const bool read_flag_) {
    BufferMemory* buffer_ptr = 0;
    const size_t timeout_seconds = 10;

    //ia I want a buffer for reading,
    //ia thus I must get the most recent buffer in READY status
    if (read_flag_) {
      buffer_ptr = _ready_buffers->popFrontTimeout(timeout_seconds);
      if (!buffer_ptr)
        return 0;

      if (buffer_ptr->status != BufferMemory::BufferStatus::Ready)
        throw std::runtime_error("[BufferManager::getBufferTimeout]| element has wrong status");

      buffer_ptr->status = BufferMemory::BufferStatus::Read;
    } else {
      buffer_ptr = _free_buffers->popFrontTimeout(timeout_seconds);
      if (!buffer_ptr)
        return 0;

      if (buffer_ptr->status != BufferMemory::BufferStatus::Free)
        throw std::runtime_error("[BufferManager::getBufferTimeout]| element has wrong status");

      buffer_ptr->clear();
      buffer_ptr->status = BufferMemory::BufferStatus::Write;
    }


    return buffer_ptr;
  }


  void BufferManager::releaseBuffer(BufferMemory* buffer_) {
    assert(buffer_);

//    std::cerr << "BufferManager::releaseBuffer| ****** BYTES WRITTEN IN THIS BUFFER: " << buffer_->data - buffer_->buffer_start << std::endl;
    ////1 get indecs
    //int idx=buffer-_buffers[0];

    //ia when releasing a buffer we want to change its status and
    //ia eventually clear the buffer (or hard delete the buffer)
    switch (buffer_->status){
    case BufferMemory::BufferStatus::Read:
      //ia if there are no other ready buffers, put again this in the list
      if (!_ready_buffers->buffers_ptr.size()) {
        buffer_->status = BufferMemory::BufferStatus::Ready;
        buffer_->data = buffer_->buffer_start;
        _ready_buffers->pushBack(buffer_);
        break;
      }

      //ia otherwise, clear the buffer
      buffer_->status = BufferMemory::BufferStatus::Free;
      buffer_->clear();
      _free_buffers->pushBack(buffer_);
      break;
    case BufferMemory::BufferStatus::Write:
      //ia check if there are no other free buffer, discard this buffer and put it in the list of free buffers
      if (!_free_buffers->buffers_ptr.size()) {
        buffer_->status = BufferMemory::BufferStatus::Free;
        buffer_->clear();
        _free_buffers->pushBack(buffer_);
        break;
      }

      buffer_->status = BufferMemory::BufferStatus::Ready;
      //ia reset the goddamn pointer to the begin of the buffer
      buffer_->data = buffer_->buffer_start;
      _ready_buffers->pushBack(buffer_);
      break;
    default:
      throw std::runtime_error("[BufferManager::releaseBuffer]| error impossible to release this buffer, invalid state");
    }

  }

  void BufferManager::freeBuffer(BufferMemory* buffer_) {
    assert(buffer_);
    buffer_->status = BufferMemory::BufferStatus::Free;
    buffer_->clear();
    _free_buffers->pushBack(buffer_);
  }
} /* namespace srrg2_buffered_viewer */
