#pragma once
#include "vector_interface.h"
#include "file_handle.h"
#include <cassert>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string>
#include <iostream>
#include <memory>
#include <cstring>
#include <iostream>
#include <errno.h>
#include <cassert>

namespace srrg2_core {
  using namespace std;
  
  template <typename ValueType_>
  struct FileVectorInterface_: public VectorInterface_<ValueType_> {

    struct Header {
      size_t item_size=sizeof(ValueType_);
      size_t num_items=0;
      size_t size_in_bytes=sizeof(Header);
      size_t aligned_chunk_length=getpagesize();
    };

    static inline size_t roundToPageSize(size_t address) {
      if (!address%getpagesize())
        return address;
      return (address/getpagesize()+1)*getpagesize();
    }
    
    FileVectorInterface_(FileHandlePtr& file_handle, size_t offset=0):
      _file_handle(file_handle) {

      // offset should be good
      if (offset%getpagesize()){
        cerr << "error in offset boundary, must be a multiple of pagesize" << endl;
        exit(0);
      }
      _offset=offset;
      
      size_t file_size=_file_handle->size();
      Header temp_header;
      
      // if empty file we write the header
      if (file_size<_offset+getpagesize()) {
        //cerr << "writing header" << endl;
        //cerr << "offset" << _offset << endl;
        ftruncate(file_handle->_fd, offset+getpagesize());
        _file_handle->lseek(_offset);
        _file_handle->write(&temp_header, sizeof(Header));
        //cerr << "file up, size:" << _file_handle->size() << endl;
      } else {
        // otherwise we read the header
        _file_handle->lseek(_offset);
        _file_handle->read(&temp_header, sizeof(Header));
        assert(temp_header.item_size==sizeof(ValueType_)
               && "invalid item size");
        assert(temp_header.size_in_bytes==sizeof(ValueType_)*temp_header.num_items+sizeof(ValueType_)
               && "inconsistent header");
        assert(file_size>(_offset +temp_header.aligned_chunk_length)
               && "file too small");
        this->_size=temp_header.num_items;
      }

      // set up mmap based on the file permissions
      int mmap_prot=PROT_READ;
      int mmap_flags=MAP_SHARED;
      //cerr << __PRETTY_FUNCTION__ << endl;
      if (! file_handle->readOnly()) {
        mmap_prot|=PROT_WRITE;
        mmap_flags=MAP_SHARED;
        //cerr << "mmap rw" << endl;
      } else {
        //cerr << "mmap rd only" << endl;
      }

      // cerr << "offset: " << _offset
      //      << " size_in_bytes: " << temp_header.size_in_bytes
      //      << " mmapped_chunk: "  << temp_header.aligned_chunk_length
      //      << " file_size: " << file_handle->size() << endl;

      void* v=(char*)mmap(nullptr, temp_header.aligned_chunk_length, mmap_prot, mmap_flags, file_handle->_fd, _offset);
      if (v==(void*)0xffffffffffffffff) {
        cerr << "error in mmap" << strerror(errno) << endl;
        exit(0);
      }
      char* _data_start=(char*)v;
      //cerr << "ptr: " << v << endl;
      if (! _data_start) {
        cerr << "mmap error" << endl;
        exit(0);
      }
      _header=(Header*)_data_start;
      assert(_header && "mmap_error");
      this->_data=0;
      this->_size=_header->num_items;
      //cerr << "header num items: " << _header->num_items << endl;
      if (this->_size) {
        this->_data=(ValueType_*)(_data_start+sizeof(Header));
      }
      //cerr << "ctor ok" << endl;
    }

    FileVectorInterface_<ValueType_>& operator=(const VectorInterface_<ValueType_>& other) {
      this->copyFrom(other);
      return *this;
    }

    void clear() override {
      resize(0);
    }

    void resize(size_t new_size) override {
      if (this->_size==new_size)
        return;
      //cerr << "in resize" << endl;
      if (_file_handle->readOnly()) {
        assert(0 && "cannot resize  afile in read only");
      }
      size_t new_size_in_bytes=new_size*sizeof(ValueType_)+sizeof(Header);

      size_t old_chunk_size=_header->aligned_chunk_length;
      size_t new_chunk_size=roundToPageSize(new_size_in_bytes);
      if (old_chunk_size!=new_chunk_size) {
        munmap(this->_header, _header->aligned_chunk_length);
        //cerr << "changing file size" << endl;
        int fd=_file_handle->_fd;
        if (fd>0) {
          int retval=ftruncate(fd, new_chunk_size+_offset);
          assert(!retval && "error on truncate");
        }
      
        int mmap_prot=PROT_READ;
        int mmap_flags=MAP_SHARED;
        if (!_file_handle->readOnly()) {
          mmap_prot|=PROT_WRITE;
          mmap_flags=MAP_SHARED;
        }

        char* _data_start=(char*)mmap(nullptr, new_chunk_size, mmap_prot, mmap_flags, fd, _offset);
        assert(_data_start && "mmap error");
        this->_data=(ValueType_*)(_data_start+sizeof(Header));
        _header=(Header*) (_data_start);
        _header->aligned_chunk_length=new_chunk_size;
      }
      this->_size=new_size;
      _header->num_items=this->_size;
      _header->size_in_bytes=new_size_in_bytes;
    }
    virtual ~FileVectorInterface_() {
      munmap(this->_header, _header->aligned_chunk_length);
    }
    size_t mmappedSize() const {return _header->size_in_bytes;}
    size_t mmappedChunkSize() const {return _header->aligned_chunk_length;}
  protected:

    Header* _header=0;
    size_t _offset=0;
    FileHandlePtr _file_handle;
  };
}
