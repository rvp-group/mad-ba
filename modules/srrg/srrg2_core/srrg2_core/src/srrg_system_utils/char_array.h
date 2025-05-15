#include <cstring>
#include <stdexcept>

namespace srrg2_core {
  template <typename T>
  void writeToCharArray(char*& dest, size_t& size, const T& v) {
    if (size<sizeof(T)) {
      throw std::runtime_error("buffer roverrun");
    }
    const char* c=(const char*) (&v);
    memcpy(dest, c, sizeof(T));
    dest += sizeof(T);
    size -= sizeof(T);
  }

  template <typename T>
  void readFromCharArray(T& v, const char*& src, size_t& size) {
    if (size<sizeof(T)) {
      throw std::runtime_error("buffer roverrun");
    }

    char* c=( char*) (&v);
    memcpy(c, src, sizeof(T));
    src += sizeof(T);
    size -= sizeof(T);
  }
}
