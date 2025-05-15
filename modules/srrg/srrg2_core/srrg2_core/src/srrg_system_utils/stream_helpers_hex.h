#include <iostream>

namespace srrg2_core {
  /*!
    Class that contains static methods for binary serialization of
    POD objects
   */
  class StreamHelpersHex {
  public:
    
    inline static char _h2c(const char c) {
      if (c < 10)
        return '0' + c;
      return 'A' + (c - 10);
    }
    static inline char _c2h(const char c) {
      if (c < 'A')
        return c - '0';
      return c - 'A' + 10;
    }

    template <typename T>
    inline static void toHex(char* dest, const T& src) {
      const char* c  = (const char*) &src;
      const size_t s = sizeof(T);
      for (size_t i = 0; i < s; ++i, ++c) {
        *dest = _h2c((*c) & 0x0F);
        ++dest;
        *dest = _h2c((*c >> 4) & 0x0F);
        ++dest;
      }
    }

    template <typename T>
    inline static void fromHex(T& dest, const char* src) {
      char* c = (char*)&dest;
      const size_t s=sizeof(T);
      for (size_t i=0; i<s; ++i, ++c, src+=2) {
        c=_c2h(*src)|(_c2h(*(src+1))<<4);
      }
    }

    //! writes on a binary stream the POD v
    template <typename T>
    inline static void writeHex(std::ostream& os, const T& v) {
      char hbuf[2 * sizeof(T)];
      toHex(hbuf, v);
      os.write(hbuf, 2 * sizeof(T));
    }

    //! reads from a binary stream the POD v
    template <typename T>
    inline static void readHex(std::istream& is, T& v) {
      char hbuf[2 * sizeof(T)];
      is.read(hbuf, 2 * sizeof(T));
      fromHex(v, hbuf);
    }
  };
} // namespace srrg2_core
