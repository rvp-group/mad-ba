#pragma once
#include <unordered_map>
#include "packets/packet_base.h"
namespace srrg2_core {

  //! @brief base class that processes packets in the viewport. Each packet will correspond to a
  //!        specific function. The process packet will choose the right function to call depending on packet type
  //!        This class must be derived for each type of viewport youwant to implement (QGL-Directx12-Vulkan based)
  class PacketProcessorBase {
  public:
    using PacketMaskMap = std::unordered_map<uint8_t, bool>;
    enum NormalProcessingType {ShaderOnly=0, Shown=1};

    //! @brief default ctor/dtor
    PacketProcessorBase() {
      _show_normal = NormalProcessingType::ShaderOnly;
      _process_payloads = true;
      _process_objects = true;
      _process_point_vectors = true;
      _normal_scale = 0.1f;
    };
    virtual ~PacketProcessorBase() {};

    //! @brief function that does the job, doing different things for different packet types
    virtual void processPacket(PacketBase* packet_) = 0;

    //! @brief inline get methods - to add entries
    inline const NormalProcessingType& showNormal() const {return _show_normal;}
    inline void setShowNormal(const NormalProcessingType& show_normal_) {_show_normal = show_normal_;}

    inline const bool& processPayloads() const {return _process_payloads;}
    inline const bool& processObjects() const {return _process_objects;}
    inline const bool& processPointVectors() const {return _process_point_vectors;}
    inline const float& normalScale() const {return _normal_scale;}

    inline void setProcessPayloads(const bool flag_) {_process_payloads = flag_;}
    inline void setProcessObjects(const bool flag_) {_process_objects = flag_;}
    inline void setProcessPointVectors(const bool flag_) {_process_point_vectors = flag_;}
    inline void setNormalScale(const float& scale_) {_normal_scale = scale_;}

  protected:
    //! @brief we want to be flaxible in the rendering. Here we say if we want to
    //! see the normals as small lines [NormalProcessingType::Shown],
    //! or just to put a normal in openGL for the shaders [NormalProcessingType::ShaderOnly]
    NormalProcessingType _show_normal;
    //! @brief if you render the normal as lines, this is the scale of the line
    float _normal_scale;

    //! @brief enable or disable macro-area of packets
    bool _process_payloads;
    bool _process_objects;
    bool _process_point_vectors;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} //ia end namespace

