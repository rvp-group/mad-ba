#pragma once
#include <set>
#include <deque>
#include <vector>
#include "srrg_geometry/geometry_defs.h"

#define BUFFER_SIZE_1MEGABYTE 1024*1024
#define BUFFER_SIZE_2MEGABYTE 1024*1024*2
#define BUFFER_SIZE_5MEGABYTE 1024*1024*5
#define BUFFER_SIZE_10MEGABYTE 1024*1024*10
#define BUFFER_SIZE_20MEGABYTE 1024*1024*20
#define BUFFER_SIZE_50MEGABYTE 1024*1024*50
#define BUFFER_SIZE_100MEGABYTE 1024*1024*100
#define BUFFER_SIZE_200MEGABYTE 1024*1024*200
#define BUFFER_SIZE_500MEGABYTE 1024*1024*500
#define BUFFER_SIZE_1GIGABYTE 1024*1024*1024

namespace srrg2_core {


  //! @brief data structure that actually has a buffer behind
  //!        it owns the memory and can contain everything you want
  struct BufferMemory {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum BufferStatus {Free=0, Read=1, Write=2, Ready=3};

    size_t size         = 0;    //! @brief size of the raw buffer
    size_t num_packets  = 0;    //! @brief number of packets inside
    BufferStatus status = Free; //! @brief status of this buffer

    char* buffer_start    = 0;  //! @brief pointer to the start of the buffer
    char* data            = 0;  //! @brief raw buffer

    //! @brief ctor - does nothing
    BufferMemory();

    //! @brief dtor - frees the memory
    virtual ~BufferMemory();

    //! @breif deep copy of the object
    BufferMemory* clone() const;

    //! @brief allocate a char* buffer of size=size_
    void allocate(const size_t& size_);

    //! @brief frees the memory
    void erase();

    //! @brief memsets the buffer to 0
    void clear();

  };

  //! @brief useful typedefs
  using BufferMemoryVector  = std::vector<BufferMemory*, Eigen::aligned_allocator<BufferMemory*> >;
  using BufferMemorySet     = std::set<BufferMemory*, std::less<BufferMemory*>, Eigen::aligned_allocator<BufferMemory*> >;
  using BufferMemoryDeque   = std::deque<BufferMemory*, Eigen::aligned_allocator<BufferMemory*> >;

} //ia end namespace srrg2_buffered_viewer
