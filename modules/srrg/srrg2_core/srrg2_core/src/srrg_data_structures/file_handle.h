#pragma once
#include <string>
#include <memory>

namespace srrg2_core {
  
  struct FileHandle {
    FileHandle(){}
    FileHandle(std::string filename,
               bool read_only=false,
               bool create=false) {
      open(filename, read_only, create);
    }

    FileHandle(const FileHandle& other) = delete;
    FileHandle& operator=(const FileHandle&) =delete;
  
    void open(std::string filename,
              bool read_only=false,
              bool create=false);
  
    void close();
  
    inline bool readOnly() const {return _read_only;}
    size_t size() const;
    void lseek(size_t pos);
    void write(const void* data, size_t size);
    void read(void* data, int size);
    ~FileHandle();
  
    int _fd=-1;
    bool _read_only=false;
  };

  using FileHandlePtr = std::shared_ptr<FileHandle>;
}
