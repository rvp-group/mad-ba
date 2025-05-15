#include "file_handle.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#undef NDEBUG
#include <cassert>

namespace srrg2_core {
  
  void FileHandle::open(std::string filename,
                        bool read_only,
                        bool create) {
    close();
    _read_only=read_only;
    int flags=O_RDWR;
    if (_read_only)
      flags=O_RDONLY;
    if(create)
      flags|=O_CREAT;
    _fd=::open(filename.c_str(), flags, 0660);
    assert(_fd>0 && "error in opening file");
  }
  
  void FileHandle::close() {
    if (_fd>0) {
      int retval = ::close(_fd);
      assert(!retval || "close error");
      _fd=0;
    }
    _read_only=false;
  }

  size_t FileHandle::size() const {
    struct stat statbuf;
    if (fstat(_fd, &statbuf)<0) {
      assert(0 && "file size fail");
    }
    return statbuf.st_size;
  }

  void FileHandle::lseek(size_t pos) {
    assert(_fd>0 && "invalid fd");
    int retval = ::lseek(_fd, pos, SEEK_SET);
    assert(retval>=0 && "seek error");
  }

  void FileHandle::write(const void* data, size_t size) {
    assert(_fd>0 && "invalid fd");
    int retval  = ::write(_fd, data, size);
    assert(retval>=0 && "write error");
  }

  void FileHandle::read(void* data, int size){
    int retval  = ::read(_fd, data, size);
    assert(retval>=0 && "read error");
  }    

  FileHandle::~FileHandle() {
    close();
  }
}
