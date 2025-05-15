#pragma once
//ia viewer stuff
#include "buffer_source.h"
#include "packet_processor.h"

//! @brief this class is in charge of doing the rendering of the
//!        things. It has a buffer source and a deserializer to
//!        receive a buffer and decode the packets. Once you have
//!        the packets, all you have to do is simply to draw the
//!        packets somehow.
namespace srrg2_core {

  class ViewportBase : Configurable {
  public:
    
    PARAM(PropertyUnsignedInt, rendering_sleep_milliseconds, "sleep time in the rendering loop, to cap opengl fps", 25, 0);
    PARAM(PropertyConfigurable_<BufferSourceBase>, source, "source of buffer to be rendered", 0, 0);

    virtual ~ViewportBase();

    //! @brief returns true if viewport is visible, false otherwise
    virtual bool isActive() const = 0;

    //! @brief updates the rendering
    virtual void update() = 0;

  protected:
    PacketDeserializer   _deserializer;          //! @brief packet deserializer - this is not a pointer, since it is the same for everyone
    PacketProcessorBase* _packet_processor = 0;  //! @brief specialized on the specific rendering framework that we use (openGL, vulkan, dx12, ...)
    BufferMemory*        _buffer = 0;            //! @brief buffer that will be received. You are in charge of this buffer.

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using ViewportBaseVector = std::vector<ViewportBase*, Eigen::aligned_allocator<ViewportBase*> >;
  using ViewportBaseSet = std::set<ViewportBase*, std::less<ViewportBase*>, Eigen::aligned_allocator<ViewportBase*> >;
} //ia end namespace
