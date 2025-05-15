#pragma once
#include <map>
#include <unordered_map>

#include "packets.h"
#include "srrg_system_utils/shell_colors.h"

namespace srrg2_core {

  struct PacketCreatorBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PacketCreatorBase() {
    }
    virtual ~PacketCreatorBase() {
    }
    virtual PacketBase* create() = 0;
  };

  template <typename PacketType_>
  struct PacketCreator_ : public PacketCreatorBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PacketCreator_() {
    }
    virtual ~PacketCreator_() {
    }

    PacketBase* create() override {
      return new PacketType_;
    }
  };

  using PacketFactoryHashMap =
    std::unordered_map<uint8_t,
                       PacketCreatorBase*,
                       std::hash<uint8_t>,
                       std::equal_to<uint8_t>,
                       Eigen::aligned_allocator<std::pair<uint8_t, PacketCreatorBase*>>>;
  using PacketFactoryMap =
    std::map<uint8_t,
             PacketCreatorBase*,
             std::less<uint8_t>,
             Eigen::aligned_allocator<std::pair<uint8_t, PacketCreatorBase*>>>;

  class PacketFactory {
  public:
//! @brief macro that adds a packet in the factory
//!        you have to cast the goddamn guy to uint8_t otherwise it does'n work
#define add_packet_type(PACKET_TYPE)                                                   \
  if (_factory_map.count((uint8_t) PACKET_TYPE::PacketType)) {                         \
    std::printf("PacketFactory::add_packet_type|packet [%02X] already defined \n",     \
                (uint8_t) PACKET_TYPE::PacketType);                                    \
    throw std::runtime_error("PacketFactoryadd_packet_type|packet double definition"); \
  }                                                                                    \
  _factory_map[(uint8_t) PACKET_TYPE::PacketType] = new PacketCreator_<PACKET_TYPE>;

    PacketFactory() {
      add_packet_type(PacketPayloadPoints);
      add_packet_type(PacketPayloadLines);
      add_packet_type(PacketPayloadSegments);

      add_packet_type(PacketAttributeColorRGBA);
      add_packet_type(PacketAttributeColorRGB);
      add_packet_type(PacketAttributePointSize);
      add_packet_type(PacketAttributeLineWidth);

      add_packet_type(PacketCommandPushMatrix);
      add_packet_type(PacketCommandPopMatrix);
      add_packet_type(PacketCommandPushColor);
      add_packet_type(PacketCommandPushPointSize);
      add_packet_type(PacketCommandPushLineWidth);
      add_packet_type(PacketCommandPopAttribute);

      add_packet_type(PacketCreateList);
      add_packet_type(PacketBeginList);
      add_packet_type(PacketEndList);
      add_packet_type(PacketCallList);
      add_packet_type(PacketDestroyList);
      
      add_packet_type(PacketObjectPlane);
      add_packet_type(PacketObjectBox);
      add_packet_type(PacketObjectBoxWireframe);
      add_packet_type(PacketObjectSphere);
      add_packet_type(PacketObjectPyramid);
      add_packet_type(PacketObjectPyramidWireframe);
      add_packet_type(PacketObjectEllipsoid);
      add_packet_type(PacketObjectCone);
      add_packet_type(PacketObjectDisk);
      add_packet_type(PacketObjectCylinder);
      add_packet_type(PacketObjectArrow2D);
      add_packet_type(PacketObjectReferenceFrame);
      add_packet_type(PacketObjectText);

      add_packet_type(PacketTransformMultTraslation);
      add_packet_type(PacketTransformMultScale);
      add_packet_type(PacketTransformMultRotation);
      add_packet_type(PacketTransformMultMatrix);

      add_packet_type(PacketPoint2fVectorCloud);
      add_packet_type(PacketPointNormal2fVectorCloud);
      add_packet_type(PacketPointNormalColor2fVectorCloud);
      add_packet_type(PacketPoint3fVectorCloud);
      add_packet_type(PacketPointNormal3fVectorCloud);
      add_packet_type(PacketPointNormalColor3fVectorCloud);
      add_packet_type(PacketPoint4fVectorCloud);
      add_packet_type(PacketPointNormal4fVectorCloud);
      add_packet_type(PacketPointNormalColor4fVectorCloud);

      add_packet_type(PacketPoint2dVectorCloud);
      add_packet_type(PacketPointNormal2dVectorCloud);
      add_packet_type(PacketPointNormalColor2dVectorCloud);
      add_packet_type(PacketPoint3dVectorCloud);
      add_packet_type(PacketPointNormal3dVectorCloud);
      add_packet_type(PacketPointNormalColor3dVectorCloud);
      add_packet_type(PacketPoint4dVectorCloud);
      add_packet_type(PacketPointNormal4dVectorCloud);
      add_packet_type(PacketPointNormalColor4dVectorCloud);

      add_packet_type(PacketPoint2fMatrixCloud);
      add_packet_type(PacketPointNormal2fMatrixCloud);
      add_packet_type(PacketPointNormalColor2fMatrixCloud);
      add_packet_type(PacketPoint3fMatrixCloud);
      add_packet_type(PacketPointNormal3fMatrixCloud);
      add_packet_type(PacketPointNormalColor3fMatrixCloud);
      add_packet_type(PacketPoint4fMatrixCloud);
      add_packet_type(PacketPointNormal4fMatrixCloud);
      add_packet_type(PacketPointNormalColor4fMatrixCloud);

      add_packet_type(PacketPoint2dMatrixCloud);
      add_packet_type(PacketPointNormal2dMatrixCloud);
      add_packet_type(PacketPointNormalColor2dMatrixCloud);
      add_packet_type(PacketPoint3dMatrixCloud);
      add_packet_type(PacketPointNormal3dMatrixCloud);
      add_packet_type(PacketPointNormalColor3dMatrixCloud);
      add_packet_type(PacketPoint4dMatrixCloud);
      add_packet_type(PacketPointNormal4dMatrixCloud);
      add_packet_type(PacketPointNormalColor4dMatrixCloud);

      add_packet_type(PacketPolygonPointNormalColor3fVectorCloud);
      add_packet_type(PacketPolygonWireframePointNormalColor3fVectorCloud);

      add_packet_type(PacketPointIntensityDescriptor2fVectorCloud);
      add_packet_type(PacketPointIntensityDescriptor3fVectorCloud);

      add_packet_type(PacketPointNormalCurvature3fVectorCloud);
      add_packet_type(PacketPointNormalCurvature3dVectorCloud);

      add_packet_type(PacketMatchablefVector);
      add_packet_type(PacketMatchabledVector);
      add_packet_type(PacketVisualMatchablefVector);
      add_packet_type(PacketVisualMatchabledVector);

      // ia cv image packet
      add_packet_type(PacketCvMat);

      add_packet_type(PacketInfoEndEpoch);
      //      std::cerr << "PacketFactory::PacketFactory|registered " <<
      //      FG_BWHITE(_factory_map.size()) << " packet types\n";
    }

    virtual ~PacketFactory() {
      PacketFactoryHashMap::iterator it     = _factory_map.begin();
      PacketFactoryHashMap::iterator it_end = _factory_map.end();
      while (it != it_end) {
        delete it->second;
        ++it;
      }
    };

    inline PacketBase* createPacket(const uint8_t packet_type_) const {
      PacketFactoryHashMap::const_iterator it = _factory_map.find(packet_type_);
      if (it == _factory_map.end()) {
        std::cerr << FG_YELLOW("PacketFactory::createPacket|ERROR: unregistered packet type")
                  << std::endl;
        return nullptr;
      }

      return it->second->create();
    }

  protected:
    PacketFactoryHashMap _factory_map;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace srrg2_core
